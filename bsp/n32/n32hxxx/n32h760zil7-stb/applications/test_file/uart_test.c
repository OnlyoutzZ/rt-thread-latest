/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2026-09-03     ox-horse        uart DMA bulk one-way 32KB alternating pair test (N32H760 uart2 <-> uart3)
 *
 * Usage:
 *   uart_pair uart2|uart3|uart4|uart5|... [chunk_kb] [frame_len]
 *     - Ports are paired by adjacent index: (uart2,uart3), (uart4,uart5), (uart6,uart7)...
 *       The first argument may name either port of a pair; the partner is
 *       resolved automatically and the named port is the one that sends first
 *     - Defaults: chunk=32KB, frame=128B. The sender streams 32KB while the
 *       peer receives and verifies it frame by frame; the direction then
 *       alternates and the peer sends 32KB back for the original sender to
 *       receive and verify; this repeats cycle after cycle forever (soak)
 *   uart_pair_stop
 *     - Stop the test and print the accumulated counters
 *
 *   - The two ports of a pair must be cross-wired (hardware loopback), e.g.
 *       uart2/uart3: uart2.TX(PA2)->uart3.RX(PB11), uart3.TX(PB10)->uart2.RX(PA3)
 *       The pins and GPIO alternate functions of the other pairs (uart4/uart5
 *       etc.) must be set up per board (see board/Cube_Config and DEBUG_NOTE:
 *       only uart1/2/3 GPIO are configured so far)
 *   - The first argument selects the port that sends first; the direction then
 *     alternates every 32KB
 *   - Frame format: magic(4)+len LE(2)+seq LE(2)+payload(len-8); the payload is
 *     derived from seq on the fly, and the receiver rebuilds and compares it
 *     byte by byte from the embedded seq, so lost / shifted / duplicated /
 *     corrupted frames are all detectable
 *   - After open, CTRL1/CTRL3 are read back and printed; CTRL3 bit5 (0x20) =
 *     DMARXEN being set means RX-DMA is ready
 *
 * Implementation notes (tied to the driver semantics):
 *   1. The driver reports RX DMA only once per USART IDLE (end of frame), and
 *      the FIFO is only 16KB < the 32KB chunk -- if the sender streams without
 *      gaps the receiver's ring buffer is bound to be overrun and lose data.
 *      The TX side therefore waits for each DMA-TX completion and then delays
 *      2ms, guaranteeing >1 byte time of line idle between frames so that RX
 *      raises an independent IDLE report per frame.
 *   2. The receiver verifies seq=0..N-1 strictly in order; any anomaly
 *      (timeout / length / corruption / out-of-order) fails the block and
 *      stops the test automatically (a bulk one-way model cannot
 *      resynchronise the way a lock-step one can, so stopping beats running
 *      on with wrong data).
 *   3. The devices are kept open and never closed (the N32 driver frees its LLI
 *      list on close and does not reallocate it on reopen, which would crash on
 *      a NULL pointer), so they stay open after the test stops.
 */

#include <rtthread.h>
#include <board.h>

#if defined(RT_SERIAL_USING_DMA)

#define UART_PAIR_DEFAULT_CHUNK_KB 32      /* data per direction per round (KB) */
#define UART_PAIR_DEFAULT_FRAME    128
#define UART_PAIR_FRAME_MIN        16
#define UART_PAIR_FRAME_MAX        512
#define UART_PAIR_HDR_SIZE         8

/* TX pacing: after DMA-TX completes, delay TX_GAP_TICKS so the line goes idle
 * between frames and the receiver gets one IDLE per frame.
 * 2ms @ 115200 = ~23 byte times, ample margin */
#define UART_PAIR_TX_GAP_TICKS (RT_TICK_PER_SECOND / 500 < 1 ? 1 : RT_TICK_PER_SECOND / 500)
#define UART_PAIR_WAIT_SEC     3       /* per-frame tx/rx and semaphore wait timeout (s) */

/* Frame format: magic(4) + len LE(2) + seq LE(2) + payload(len-8)
 * payload[i] = (seq*157 + i*107 + 31) & 0xff; the receiver rebuilds it from
 * seq and compares */
#define FRAME_MAGIC0 0x55
#define FRAME_MAGIC1 0xAA
#define FRAME_MAGIC2 0x4E    /* 'N' */
#define FRAME_MAGIC3 0x33    /* '3' */

/* CTRL1 @ +0, CTRL3 @ +8 (register read-back diagnostics) */
#define CTRL3_DMARXEN 0x20

/* Print mutex: two port threads calling rt_kprintf at once interleave at
 * character level, so lock to make each line atomic */
static struct rt_mutex uart_pair_print_mtx;
#define PAIR_LOG(...)                                            \
    do                                                           \
    {                                                            \
        rt_mutex_take(&uart_pair_print_mtx, RT_WAITING_FOREVER); \
        rt_kprintf(__VA_ARGS__);                                 \
        rt_mutex_release(&uart_pair_print_mtx);                  \
    } while (0)

/* Serial receive message (ISR -> message queue -> thread) */
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

struct uart_pair_ctx;

struct uart_pair_port
{
    char name[RT_NAME_MAX + 1];  /* own copy, does not point into the command argv */
    rt_device_t dev;
    struct rt_messagequeue mq;
    rt_uint8_t mq_pool[512];
    rt_uint8_t txbuf[UART_PAIR_FRAME_MAX]; /* DMA_TX source buffer (reused only after completion) */
    rt_uint8_t rxbuf[UART_PAIR_FRAME_MAX]; /* receive frame buffer */
    rt_sem_t txc_sem;    /* DMA-TX completion signal (per-frame pacing) */
    rt_uint32_t tx_ok;      /* total frames sent successfully */
    rt_uint32_t tx_fail;
    rt_uint32_t rx_ok;      /* total frames received and verified */
    rt_uint32_t rx_fail;
    rt_uint32_t mq_drop;    /* notifications dropped because the mq was full in rx_indicate */
    struct uart_pair_ctx *ctx;
};

struct uart_pair_ctx
{
    struct uart_pair_port a;          /* port that sends first */
    struct uart_pair_port b;          /* port that receives first */
    rt_sem_t slave_ready;    /* b is ready (open + flush done) */
    rt_sem_t done_a;
    rt_sem_t done_b;
    rt_sem_t stopped;        /* cleanup finished (released by the reaper) */
    rt_uint32_t chunk_bytes;    /* data per direction per round */
    rt_uint32_t frame_len;
    rt_uint32_t frames;         /* frames per chunk = chunk_bytes / frame_len */
    rt_uint32_t cycles;         /* completed cycles (one round in each direction) */
    rt_bool_t stop;
    rt_bool_t failed;         /* an anomaly stopped the test automatically */
};

/* ------------------------- frame build / verify ------------------------- */

static void uart_pair_build_frame(rt_uint8_t *buf, rt_uint32_t len, rt_uint32_t seq)
{
    rt_uint32_t i;

    buf[0] = FRAME_MAGIC0;
    buf[1] = FRAME_MAGIC1;
    buf[2] = FRAME_MAGIC2;
    buf[3] = FRAME_MAGIC3;
    buf[4] = (rt_uint8_t)(len & 0xff);
    buf[5] = (rt_uint8_t)((len >> 8) & 0xff);
    buf[6] = (rt_uint8_t)(seq & 0xff);
    buf[7] = (rt_uint8_t)((seq >> 8) & 0xff);

    for (i = UART_PAIR_HDR_SIZE; i < len; i++)
    {
        buf[i] = (rt_uint8_t)(((seq & 0xffff) * 157u + i * 107u + 31u) & 0xff);
    }
}

/* Compare against the expected frame byte by byte (rebuilt from len/seq);
 * returns 0 if all match, otherwise the offset of the first difference */
static rt_uint32_t uart_pair_cmp_frame(const rt_uint8_t *rx, rt_uint32_t rx_len,
                                       rt_uint32_t expect_len, rt_uint32_t seq)
{
    rt_uint32_t i;

    if (rx_len != expect_len)
    {
        return 0xFFFFFFFF;
    }

    for (i = 0; i < expect_len; i++)
    {
        rt_uint8_t exp;

        if (i < UART_PAIR_HDR_SIZE)
        {
            switch (i)
            {
            case 0:
                exp = FRAME_MAGIC0;
                break;
            case 1:
                exp = FRAME_MAGIC1;
                break;
            case 2:
                exp = FRAME_MAGIC2;
                break;
            case 3:
                exp = FRAME_MAGIC3;
                break;
            case 4:
                exp = (rt_uint8_t)(expect_len & 0xff);
                break;
            case 5:
                exp = (rt_uint8_t)((expect_len >> 8) & 0xff);
                break;
            case 6:
                exp = (rt_uint8_t)(seq & 0xff);
                break;
            default:
                exp = (rt_uint8_t)((seq >> 8) & 0xff);
                break;
            }
        }
        else
        {
            exp = (rt_uint8_t)(((seq & 0xffff) * 157u + i * 107u + 31u) & 0xff);
        }

        if (rx[i] != exp)
        {
            return i;
        }
    }
    return 0;
}

/* Minimal decimal parser, avoids depending on libc */
static rt_uint32_t uart_pair_atoi(const char *s)
{
    rt_uint32_t v = 0;

    while (*s >= '0' && *s <= '9')
    {
        v = v * 10 + (rt_uint32_t)(*s - '0');
        s++;
    }
    return v;
}

/* ------------------------- drain FIFO (clear startup residue) ------------------------- */

struct uart_pair_drain_info
{
    rt_uint32_t total;
};

static void uart_pair_rx_drain(struct uart_pair_port *port, struct uart_pair_drain_info *info)
{
    struct rx_msg msg;
    rt_uint8_t tmp[UART_PAIR_FRAME_MAX];
    rt_ssize_t n;

    info->total = 0;

    for (;;)
    {
        while ((n = rt_device_read(port->dev, 0, tmp, sizeof(tmp))) > 0)
        {
            info->total += (rt_uint32_t)n;
        }

        /* wait 200ms of silence to confirm no late frames */
        rt_memset(&msg, 0, sizeof(msg));
        if (rt_mq_recv(&port->mq, &msg, sizeof(msg), RT_TICK_PER_SECOND / 5) < 0)
        {
            break;
        }
    }
}

/* ------------------------- device name parse / pairing / register base ------------------------- */

/* Parse the index of "uartN" (N=1..99) or "lpuartN" (N=1/2); returns -1 on failure */
static int uart_pair_name_index(const char *name)
{
    const char *p;

    if (rt_strncmp(name, "uart", 4) == 0)
    {
        p = name + 4;
    }
    else if (rt_strncmp(name, "lpuart", 6) == 0)
    {
        p = name + 6;
    }
    else
    {
        return -1;
    }
    if (*p < '1' || *p > '9')
    {
        return -1;
    }
    return uart_pair_atoi(p);
}

/* Pairing: (uart2,uart3)(uart4,uart5)... even index +1, odd index -1; uart1 has no partner */
static int uart_pair_partner_index(int idx)
{
    if (idx <= 1)
    {
        return -1;
    }
    return (idx % 2 == 0) ? (idx + 1) : (idx - 1);
}

/* Build the partner device name "uart%d"; lpuart input is not paired for now */
static void uart_pair_build_name(char *buf, rt_size_t sz, int idx)
{
    rt_snprintf(buf, sz, "uart%d", idx);
}

static rt_bool_t uart_pair_name_prefix_is_lpuart(const char *name)
{
    return (rt_strncmp(name, "lpuart", 6) == 0) ? RT_TRUE : RT_FALSE;
}

/* USART/UART register base (for diagnostics read-back only); values taken from
 * packages/n32h7xx_cmsis_driver-latest/device/n32h7xx.h
 *   APB1 = 0x40000000, APB2 = APB1+0xD0000, APB5 = APB1+0x18000000
 *   uart1..4: USART1..4_BASE; uart9..12: UART9..12_BASE (APB1)
 *   uart5..8: USART5..8_BASE; uart13..15: UART13..15_BASE (APB2)
 *   lpuart1/2: LPUART1/2_BASE (APB5)
 * (the device header macros are not referenced directly: they may not be
 * visible in this translation unit, and the mapping would then read as 0) */
static rt_uint32_t uart_pair_reg_base(const char *name)
{
    int idx = uart_pair_name_index(name);

    if (uart_pair_name_prefix_is_lpuart(name))
    {
        if (idx == 1)
        {
            return 0x58000800UL;
        }
        if (idx == 2)
        {
            return 0x58000C00UL;
        }
        return 0;
    }
    switch (idx)
    {
    case 1:
        return 0x4000C400UL;
    case 2:
        return 0x4000C800UL;
    case 3:
        return 0x4000CC00UL;
    case 4:
        return 0x4000D000UL;
    case 9:
        return 0x4000D400UL;
    case 10:
        return 0x4000D800UL;
    case 11:
        return 0x4000DC00UL;
    case 12:
        return 0x4000E000UL;
    case 5:
        return 0x400DE000UL;
    case 6:
        return 0x400DE400UL;
    case 7:
        return 0x400DE800UL;
    case 8:
        return 0x400DEC00UL;
    case 13:
        return 0x400DF000UL;
    case 14:
        return 0x400DF400UL;
    case 15:
        return 0x400DF800UL;
    default:
        return 0;
    }
}

static rt_uint32_t uart_pair_reg_read(const char *name, rt_uint32_t offset)
{
    rt_uint32_t base = uart_pair_reg_base(name);

    return base ? *(volatile rt_uint32_t *)(base + offset) : 0;
}

/* ------------------------- callbacks (ISR context) ------------------------- */

static rt_err_t uart_pair_rx_indicate(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    struct uart_pair_ctx *ctx = (struct uart_pair_ctx *)dev->user_data;
    struct uart_pair_port *port;

    if (ctx == RT_NULL)
    {
        return RT_ERROR;
    }
    port = (dev == ctx->a.dev) ? &ctx->a : ((dev == ctx->b.dev) ? &ctx->b : RT_NULL);
    if (port == RT_NULL)
    {
        return RT_ERROR;
    }

    msg.dev = dev;
    msg.size = size;
    if (rt_mq_send(&port->mq, &msg, sizeof(msg)) != RT_EOK)
    {
        port->mq_drop++;
    }
    return RT_EOK;
}

static rt_err_t uart_pair_tx_done(rt_device_t dev, void *buffer)
{
    struct uart_pair_ctx *ctx = (struct uart_pair_ctx *)dev->user_data;
    struct uart_pair_port *port;

    if (ctx == RT_NULL)
    {
        return RT_ERROR;
    }
    port = (dev == ctx->a.dev) ? &ctx->a : ((dev == ctx->b.dev) ? &ctx->b : RT_NULL);
    if (port == RT_NULL || port->txc_sem == RT_NULL)
    {
        return RT_ERROR;
    }
    rt_sem_release(port->txc_sem);      /* 1:1 counting, only one frame in flight at a time */
    return RT_EOK;
}

/* ------------------------- port open ------------------------- */

static rt_err_t uart_pair_port_open(struct uart_pair_port *port)
{
    rt_uint32_t ctrl3;

    rt_mq_init(&port->mq, (port->ctx->a.dev == port->dev) ? "a_mq" : "b_mq",
               port->mq_pool, sizeof(struct rx_msg), sizeof(port->mq_pool), RT_IPC_FLAG_FIFO);

    port->txc_sem = rt_sem_create((port->ctx->a.dev == port->dev) ? "a_txc" : "b_txc",
                                  0, RT_IPC_FLAG_FIFO);
    if (port->txc_sem == RT_NULL)
    {
        PAIR_LOG("[thr] %s create txc sem failed!\n", port->name);
        return -RT_ERROR;
    }

    /* Attach the callbacks before open: open enables the DMA/IDLE interrupts,
     * and this way the first frame's notification is not lost */
    rt_device_set_rx_indicate(port->dev, uart_pair_rx_indicate);
    rt_device_set_tx_complete(port->dev, uart_pair_tx_done);

    if (rt_device_open(port->dev, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX) != RT_EOK)
    {
        PAIR_LOG("[thr] %s open failed!\n", port->name);
        return -RT_ERROR;
    }

    ctrl3 = uart_pair_reg_read(port->name, 8);
    PAIR_LOG("[thr] %s open: CTRL1=0x%08x CTRL3=0x%08x\n",
             port->name, uart_pair_reg_read(port->name, 0), ctrl3);
    if (uart_pair_reg_base(port->name) == 0)
    {
        /* This port is not in the device header (lpuart, or no peripheral
         * mapping yet), so register read-back is unavailable */
        PAIR_LOG("[thr] %s info: register base not mapped, skip DMA check\n", port->name);
    }
    else if ((ctrl3 & CTRL3_DMARXEN) == 0)
    {
        PAIR_LOG("[thr] %s warn: RX-DMA(CTRL3 bit5) not enabled!\n", port->name);
    }
    return RT_EOK;
}

/* ------------------------- receive one frame (streaming reassembly) -------------------------
 * Under normal pacing one rx_indicate notification equals one frame; returns
 * once frame_len bytes have arrived.
 * Empty notifications (a late or leftover previous frame) are skipped; a
 * timeout or any anomaly returns non-EOK. */
static rt_err_t uart_pair_recv_frame(struct uart_pair_port *port, struct uart_pair_ctx *ctx)
{
    struct rx_msg msg;
    rt_uint32_t off = 0;
    rt_uint32_t stale = 0;
    rt_ssize_t res;

    for (;;)
    {
        rt_memset(&msg, 0, sizeof(msg));
        res = rt_mq_recv(&port->mq, &msg, sizeof(msg), UART_PAIR_WAIT_SEC * RT_TICK_PER_SECOND);
        if (res < 0)
        {
            return -RT_ERROR;
        }

        if (off < ctx->frame_len)
        {
            rt_ssize_t n = rt_device_read(msg.dev, 0, port->rxbuf + off, ctx->frame_len - off);

            if (n <= 0)
            {
                if (++stale > 16)
                {
                    return -RT_ERROR;
                }
                continue;
            }
            off += (rt_uint32_t)n;
        }
        if (off >= ctx->frame_len)
        {
            return RT_EOK;
        }
    }
}

/* ------------------------- send one whole chunk ------------------------- */

static rt_err_t uart_pair_tx_chunk(struct uart_pair_port *port, struct uart_pair_ctx *ctx)
{
    rt_uint32_t i;
    rt_uint32_t len = ctx->frame_len;

    for (i = 0; i < ctx->frames; i++)
    {
        if (ctx->stop)
        {
            return -RT_ERROR;
        }

        uart_pair_build_frame(port->txbuf, len, i);

        if (rt_device_write(port->dev, 0, port->txbuf, len) != (rt_ssize_t)len)
        {
            PAIR_LOG("[thr] %s TX write fail @frame %u!\n", port->name, i);
            return -RT_ERROR;
        }

        /* Wait for this frame's DMA-TX to finish, then delay to create the
         * inter-frame gap (so the receiver gets one IDLE per frame) */
        if (rt_sem_take(port->txc_sem, UART_PAIR_WAIT_SEC * RT_TICK_PER_SECOND) != RT_EOK)
        {
            PAIR_LOG("[thr] %s TX complete timeout @frame %u!\n", port->name, i);
            return -RT_ERROR;
        }
        rt_thread_delay(UART_PAIR_TX_GAP_TICKS);
    }
    return RT_EOK;
}

/* ------------------------- receive one whole chunk and verify strictly ------------------------- */

static rt_err_t uart_pair_rx_chunk(struct uart_pair_port *port, struct uart_pair_ctx *ctx)
{
    rt_uint32_t i;

    for (i = 0; i < ctx->frames; i++)
    {
        rt_uint32_t diff;

        if (ctx->stop)
        {
            return -RT_ERROR;
        }

        if (uart_pair_recv_frame(port, ctx) != RT_EOK)
        {
            PAIR_LOG("[thr] %s RX frame %u/%u recv timeout/fail\n",
                     port->name, i, ctx->frames);
            return -RT_ERROR;
        }

        /* Strict in-order check: seq must equal the index within this chunk */
        if ((rt_uint32_t)(port->rxbuf[6] | (port->rxbuf[7] << 8)) != i)
        {
            PAIR_LOG("[thr] %s RX frame %u FAIL: seq=%u (order error)\n",
                     port->name, i, (rt_uint32_t)(port->rxbuf[6] | (port->rxbuf[7] << 8)));
            return -RT_ERROR;
        }
        diff = uart_pair_cmp_frame(port->rxbuf, ctx->frame_len, ctx->frame_len, i);
        if (diff != 0)
        {
            PAIR_LOG("[thr] %s RX frame %u FAIL: content diff@%u rx[0]=0x%02x\n",
                     port->name, i, diff, port->rxbuf[0]);
            return -RT_ERROR;
        }
    }
    return RT_EOK;
}

/* ------------------------- run threads -------------------------
 * port a: send then receive; port b: receive then send; each completed
 * "send + receive" is one round (the direction alternates once) */

static void uart_pair_dir_entry(void *param)
{
    struct uart_pair_port *port = (struct uart_pair_port *)param;
    struct uart_pair_ctx *ctx = port->ctx;
    rt_bool_t is_a = (port == &ctx->a);
    struct uart_pair_drain_info dinfo;
    rt_err_t res;
    rt_uint32_t chunk = ctx->chunk_bytes;

    if (uart_pair_port_open(port) != RT_EOK)
    {
        ctx->failed = RT_TRUE;
        ctx->stop = RT_TRUE;
        if (is_a)
        {
            rt_sem_release(ctx->done_a);
        }
        else
        {
            rt_sem_release(ctx->slave_ready);   /* so the peer does not wait forever */
            rt_sem_release(ctx->done_b);
        }
        return;
    }

    /* Clear residue from a previous run (the devices stay open, so data can
     * be left over) */
    uart_pair_rx_drain(port, &dinfo);
    if (dinfo.total > 0)
    {
        PAIR_LOG("[thr] %s start flush: drained %uB\n", port->name, dinfo.total);
    }

    if (is_a)
    {
        /* Wait for b to be ready, then start the first round of sending */
        while (rt_sem_take(ctx->slave_ready, RT_TICK_PER_SECOND / 5) != RT_EOK)
        {
            if (ctx->stop || ctx->failed)
            {
                rt_sem_release(ctx->done_a);
                return;
            }
        }
    }
    else
    {
        rt_sem_release(ctx->slave_ready);   /* b is ready, a may start sending */
    }

    while (!ctx->stop)
    {
        if (is_a)
        {
            res = uart_pair_tx_chunk(port, ctx);
            if (res == RT_EOK)
            {
                port->tx_ok += ctx->frames;
                PAIR_LOG("[thr] %s TX chunk ok (%uB, %u frames, seq 0-%u)\n",
                         port->name, chunk, ctx->frames, ctx->frames - 1);
            }
            else
            {
                if (!ctx->stop)
                {
                    port->tx_fail++;
                    ctx->failed = RT_TRUE;
                    ctx->stop = RT_TRUE;
                }
                break;
            }

            res = uart_pair_rx_chunk(port, ctx);
            if (res == RT_EOK)
            {
                port->rx_ok += ctx->frames;
                PAIR_LOG("[thr] %s RX chunk ok (%uB, %u frames, seq 0-%u)\n",
                         port->name, chunk, ctx->frames, ctx->frames - 1);
            }
            else
            {
                if (!ctx->stop)
                {
                    port->rx_fail++;
                    ctx->failed = RT_TRUE;
                    ctx->stop = RT_TRUE;
                }
                break;
            }

            ctx->cycles++;
        }
        else
        {
            res = uart_pair_rx_chunk(port, ctx);
            if (res == RT_EOK)
            {
                port->rx_ok += ctx->frames;
                PAIR_LOG("[thr] %s RX chunk ok (%uB, %u frames, seq 0-%u)\n",
                         port->name, chunk, ctx->frames, ctx->frames - 1);
            }
            else
            {
                if (!ctx->stop)
                {
                    port->rx_fail++;
                    ctx->failed = RT_TRUE;
                    ctx->stop = RT_TRUE;
                }
                break;
            }

            res = uart_pair_tx_chunk(port, ctx);
            if (res == RT_EOK)
            {
                port->tx_ok += ctx->frames;
                PAIR_LOG("[thr] %s TX chunk ok (%uB, %u frames, seq 0-%u)\n",
                         port->name, chunk, ctx->frames, ctx->frames - 1);
            }
            else
            {
                if (!ctx->stop)
                {
                    port->tx_fail++;
                    ctx->failed = RT_TRUE;
                    ctx->stop = RT_TRUE;
                }
                break;
            }
        }
    }

    PAIR_LOG("[thr] %s done (tx_ok=%u tx_fail=%u rx_ok=%u rx_fail=%u)%s\n",
             port->name, port->tx_ok, port->tx_fail, port->rx_ok, port->rx_fail,
             ctx->failed ? " [FAIL auto-stop]" : "");
    if (is_a)
    {
        rt_sem_release(ctx->done_a);
    }
    else
    {
        rt_sem_release(ctx->done_b);
    }
}

/* ------------------------- globals: command state (threads/callbacks outlive the command) ------------------------- */

static struct uart_pair_ctx uart_pair_ctx;
static rt_bool_t uart_pair_running = RT_FALSE;
static struct rt_semaphore uart_pair_stopped_obj;   /* static instance of the stopped semaphore, reused across runs */
static char uart_pair_partner_name[RT_NAME_MAX + 1]; /* partner device name built at run time */

/* ------------------------- reaper thread (cleans up after both port threads exit) ------------------------- */

static void uart_pair_reaper_entry(void *param)
{
    struct uart_pair_ctx *ctx = &uart_pair_ctx;

    rt_sem_take(ctx->done_a, RT_WAITING_FOREVER);
    rt_sem_take(ctx->done_b, RT_WAITING_FOREVER);

    if (ctx->a.dev)
    {
        rt_device_set_rx_indicate(ctx->a.dev, RT_NULL);
        rt_device_set_tx_complete(ctx->a.dev, RT_NULL);
        ctx->a.dev->user_data = RT_NULL;
    }
    if (ctx->b.dev)
    {
        rt_device_set_rx_indicate(ctx->b.dev, RT_NULL);
        rt_device_set_tx_complete(ctx->b.dev, RT_NULL);
        ctx->b.dev->user_data = RT_NULL;
    }
    rt_sem_delete(ctx->slave_ready);
    rt_sem_delete(ctx->done_a);
    rt_sem_delete(ctx->done_b);
    if (ctx->a.txc_sem)
    {
        rt_sem_delete(ctx->a.txc_sem);
    }
    if (ctx->b.txc_sem)
    {
        rt_sem_delete(ctx->b.txc_sem);
    }

    uart_pair_running = RT_FALSE;
    rt_sem_release(ctx->stopped);
}

/* ------------------------- command entry ------------------------- */

static int uart_pair(int argc, char *argv[])
{
    struct uart_pair_ctx *ctx = &uart_pair_ctx;
    const char *tx_first_name = "uart2";
    const char *rx_first_name;
    rt_thread_t tid;
    rt_uint32_t chunk_kb = UART_PAIR_DEFAULT_CHUNK_KB;

    if (uart_pair_running)
    {
        PAIR_LOG("uart_pair already running, use uart_pair_stop first!\n");
        return -RT_ERROR;
    }

    rt_memset(ctx, 0, sizeof(*ctx));

    if (argc >= 2)
    {
        int idx = uart_pair_name_index(argv[1]);
        int pid;

        tx_first_name = argv[1];
        if (idx < 2 || uart_pair_name_prefix_is_lpuart(argv[1]))
        {
            PAIR_LOG("unsupported port '%s': use uart2|uart3|uart4|uart5|...\n", argv[1]);
            return -RT_ERROR;
        }
        pid = uart_pair_partner_index(idx);
        if (pid < 0)
        {
            PAIR_LOG("port '%s' has no pair partner (uart1 is console)\n", argv[1]);
            return -RT_ERROR;
        }
        uart_pair_build_name(uart_pair_partner_name, sizeof(uart_pair_partner_name), pid);
        rx_first_name = uart_pair_partner_name;
    }
    else
    {
        uart_pair_build_name(uart_pair_partner_name, sizeof(uart_pair_partner_name), 3);
        rx_first_name = uart_pair_partner_name;
        tx_first_name = "uart2";
    }

    if (argc >= 3)
    {
        chunk_kb = uart_pair_atoi(argv[2]);
        if (chunk_kb < 1 || chunk_kb > 1024)
        {
            PAIR_LOG("chunk_kb must be 1~1024\n");
            return -RT_ERROR;
        }
    }
    ctx->frame_len = UART_PAIR_DEFAULT_FRAME;
    if (argc >= 4)
    {
        ctx->frame_len = uart_pair_atoi(argv[3]);
        if (ctx->frame_len < UART_PAIR_FRAME_MIN || ctx->frame_len > UART_PAIR_FRAME_MAX)
        {
            PAIR_LOG("frame_len must be %d~%d\n", UART_PAIR_FRAME_MIN, UART_PAIR_FRAME_MAX);
            return -RT_ERROR;
        }
    }

    ctx->chunk_bytes = chunk_kb * 1024;
    ctx->frames = ctx->chunk_bytes / ctx->frame_len;
    if (ctx->frames == 0 || (ctx->chunk_bytes % ctx->frame_len) != 0)
    {
        PAIR_LOG("chunk %uB not divisible by frame %uB\n", ctx->chunk_bytes, ctx->frame_len);
        return -RT_ERROR;
    }

    /* Keep our own copy of the name (argv is invalid once the command returns) */
    rt_memset(ctx->a.name, 0, sizeof(ctx->a.name));
    rt_strncpy(ctx->a.name, tx_first_name, sizeof(ctx->a.name) - 1);
    rt_memset(ctx->b.name, 0, sizeof(ctx->b.name));
    rt_strncpy(ctx->b.name, rx_first_name, sizeof(ctx->b.name) - 1);
    ctx->a.ctx = ctx;
    ctx->b.ctx = ctx;

    ctx->a.dev = rt_device_find(tx_first_name);
    ctx->b.dev = rt_device_find(rx_first_name);
    if (ctx->a.dev == RT_NULL || ctx->b.dev == RT_NULL)
    {
        PAIR_LOG("find %s/%s failed!\n", tx_first_name, rx_first_name);
        return -RT_ERROR;
    }
    ctx->a.dev->user_data = ctx;
    ctx->b.dev->user_data = ctx;

    ctx->slave_ready = rt_sem_create("rdy", 0, RT_IPC_FLAG_FIFO);
    ctx->done_a = rt_sem_create("dn_a", 0, RT_IPC_FLAG_FIFO);
    ctx->done_b = rt_sem_create("dn_b", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&uart_pair_stopped_obj, "stpd", 0, RT_IPC_FLAG_FIFO);
    ctx->stopped = &uart_pair_stopped_obj;  /* static object, not destroyed with the run */
    if (ctx->slave_ready == RT_NULL || ctx->done_a == RT_NULL || ctx->done_b == RT_NULL)
    {
        PAIR_LOG("create semaphore failed!\n");
        ctx->a.dev->user_data = RT_NULL;
        ctx->b.dev->user_data = RT_NULL;
        return -RT_ERROR;
    }

    rt_mutex_init(&uart_pair_print_mtx, "plog", RT_IPC_FLAG_FIFO);

    rt_kprintf("[pair] bulk test %s(TX first) <-> %s (chunk=%uKB=%uB, frame=%uB, %u frames/chunk)\n",
               tx_first_name, rx_first_name, chunk_kb, ctx->chunk_bytes,
               ctx->frame_len, ctx->frames);
    rt_kprintf("[pair] pattern: %s TX %uKB -> %s RX/verify, then %s TX %uKB -> %s RX/verify, loop\n",
               tx_first_name, chunk_kb, rx_first_name, rx_first_name, chunk_kb, tx_first_name);
    rt_kprintf("[pair] press 'uart_pair_stop' to stop\n");

    tid = rt_thread_create("pair_b", uart_pair_dir_entry, &ctx->b, 2048, 22, 10);
    if (tid == RT_NULL)
    {
        PAIR_LOG("create b thread failed!\n");
        goto __err;
    }
    rt_thread_startup(tid);

    tid = rt_thread_create("pair_a", uart_pair_dir_entry, &ctx->a, 2048, 23, 10);
    if (tid == RT_NULL)
    {
        PAIR_LOG("create a thread failed!\n");
        ctx->stop = RT_TRUE;
        rt_sem_take(ctx->done_b, (UART_PAIR_WAIT_SEC + 2) * RT_TICK_PER_SECOND);
        goto __err;
    }
    rt_thread_startup(tid);

    /* reaper: frees resources after both port threads exit */
    tid = rt_thread_create("pair_rep", uart_pair_reaper_entry, RT_NULL, 1024, 24, 10);
    if (tid == RT_NULL)
    {
        PAIR_LOG("create reaper thread failed!\n");
        ctx->stop = RT_TRUE;
        rt_sem_take(ctx->done_a, (UART_PAIR_WAIT_SEC + 2) * RT_TICK_PER_SECOND);
        rt_sem_take(ctx->done_b, (UART_PAIR_WAIT_SEC + 2) * RT_TICK_PER_SECOND);
        goto __err;
    }
    rt_thread_startup(tid);

    uart_pair_running = RT_TRUE;
    return RT_EOK;

__err:
    rt_sem_delete(ctx->slave_ready);
    rt_sem_delete(ctx->done_a);
    rt_sem_delete(ctx->done_b);
    if (ctx->a.txc_sem)
    {
        rt_sem_delete(ctx->a.txc_sem);
    }
    if (ctx->b.txc_sem)
    {
        rt_sem_delete(ctx->b.txc_sem);
    }
    ctx->a.dev->user_data = RT_NULL;
    ctx->b.dev->user_data = RT_NULL;
    return -RT_ERROR;
}
MSH_CMD_EXPORT(uart_pair, uart dma bulk alt test(pair 2 / 3, 4 / 5...) : uart_pair uart4[chunk_kb][frame_len]);

/* ------------------------- stop command ------------------------- */

static int uart_pair_stop(void)
{
    struct uart_pair_ctx *ctx = &uart_pair_ctx;

    if (!uart_pair_running)
    {
        PAIR_LOG("uart_pair not running\n");
        return -RT_ERROR;
    }

    ctx->stop = RT_TRUE;

    /* Wait for the reaper to finish cleanup (a thread exits within one frame
     * timeout, 3s at worst) */
    if (rt_sem_take(ctx->stopped, (UART_PAIR_WAIT_SEC + 2) * RT_TICK_PER_SECOND) != RT_EOK)
    {
        PAIR_LOG("[pair] stop timeout (threads may still run)!\n");
        return -RT_ERROR;
    }

    rt_kprintf("[pair] stopped after %u cycles\n", ctx->cycles);
    rt_kprintf("[pair] a(%s): tx_ok=%u tx_fail=%u rx_ok=%u rx_fail=%u (mq_drop=%u)\n",
               ctx->a.name, ctx->a.tx_ok, ctx->a.tx_fail, ctx->a.rx_ok, ctx->a.rx_fail,
               ctx->a.mq_drop);
    rt_kprintf("[pair] b(%s): tx_ok=%u tx_fail=%u rx_ok=%u rx_fail=%u (mq_drop=%u)\n",
               ctx->b.name, ctx->b.tx_ok, ctx->b.tx_fail, ctx->b.rx_ok, ctx->b.rx_fail,
               ctx->b.mq_drop);
    if (ctx->failed)
    {
        PAIR_LOG("[pair] FAIL detected, test auto-stopped\n");
    }
    else
    {
        PAIR_LOG("[pair] all clean\n");
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(uart_pair_stop, stop running uart_pair bulk test);

#endif /* RT_SERIAL_USING_DMA */
