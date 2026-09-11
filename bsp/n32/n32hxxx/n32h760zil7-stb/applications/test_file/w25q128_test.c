/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-09-11     ox-horse         first version
 */

/* W25Q128 (16MB SPI NOR) console test.
 *
 * One chip, one wiring at a time: the flash is driven by whichever SPI bus the
 * caller names, so the same test runs on spi1..spi7 without a rebuild. Every
 * case is issued as its own msh command and prints exactly one verdict line so
 * the output can be diffed across buses, rates and DMA settings.
 *
 *   w25q list
 *   w25q id    <spi>        [cs=PFx] [hz=K] [dma=0|1] [mode=N] [eng=raw|sfud] [pins=def|alt]
 *   w25q rw    <spi> <len>  [cs=PFx] [hz=K] [dma=0|1] [mode=N] [chunk=B] [off=B] [eng=] [noerase] [pins=]
 *   w25q sweep <spi>        [cs=PFx] [hz=K] [dma=0|1] [mode=N] [chunk=B] [off=B] [eng=] [pins=]
 *   w25q rate  <spi> <len>  [cs=PFx] [dma=0|1] [mode=N] [chunk=B] [off=B] [eng=] [pins=]
 *
 * pins= only means anything on spi7, the one bus the AFIO table maps twice
 * (see g_sig7_alt); it is ignored everywhere else.
 *
 * Two engines, same bus, same parameters -- pick with eng=:
 *
 *   eng=raw   the command set below issued directly over rt_spi_send_then_recv
 *   eng=sfud  the same operations through RT-Thread's SFUD stack
 *             (components/drivers/spi/sfud + dev_spi_flash_sfud.c), which adds
 *             JEDEC/SFDP probing, its own page and erase-granularity handling
 *             and its own WIP polling. Needs RT_USING_SFUD.
 *
 * Running the same case under both is the point: they share nothing above the
 * rt_spi_* layer, so a case that passes on one engine and fails on the other
 * localises the fault to that layer rather than to the wire. eng= also lands in
 * every verdict line, so sweeps from the two engines diff directly.
 *
 * DMA is switched at runtime by writing the driver's per-bus spi_dma_flag, so
 * one firmware covers both "DMA on" and "DMA off" -- no rtconfig rebuild and no
 * reflash between the two halves of a matrix. The flag may only be raised for a
 * bus whose DMA was compiled in (config->dma_rx/dma_tx are NULL otherwise and
 * spixfer would dereference them), so w25q_dma_compiled() gates it per bus.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "drv_spi.h"

#if defined(RT_USING_SFUD)
#include "dev_spi_flash_sfud.h"
#endif

#if defined(BSP_USING_SPI) && defined(RT_USING_SPI)

/* ------------------------------------------------------------------ *
 *  W25Q128 command set (only what the test needs)
 * ------------------------------------------------------------------ */

#define W25Q_CMD_WREN     0x06u   /* write enable                        */
#define W25Q_CMD_RDSR1    0x05u   /* read status register 1              */
#define W25Q_CMD_READ     0x03u   /* read data                           */
#define W25Q_CMD_PAGEPROG 0x02u   /* page program (<=256B, one page)     */
#define W25Q_CMD_SECERASE 0x20u   /* 4KB sector erase                    */
#define W25Q_CMD_BLKERASE 0xD8u   /* 64KB block erase                    */
#define W25Q_CMD_JEDECID  0x9Fu   /* JEDEC ID                            */

#define W25Q_SR1_WIP 0x01u   /* write in progress                   */

#define W25Q_PAGE_SIZE   256u
#define W25Q_SECTOR_SIZE 4096u
#define W25Q_BLOCK_SIZE  65536u
#define W25Q_CAPACITY    (16u * 1024u * 1024u)

/* Erase/program completion budgets. A 64KB block erase is 2s worst case and a
 * page program 3ms; the margins below are for a slow part, not a stuck one. */
#define W25Q_TIMEOUT_ERASE_MS 10000u
#define W25Q_TIMEOUT_PROG_MS  500u

/* Per-arm ceiling of the driver: spixfer chunks every message at 4096 items,
 * and a 4096-item arm is the largest that is verified intact on this silicon.
 * The default chunk buffer matches it, so one chunk == one DMA arm. */
#define W25Q_DEFAULT_CHUNK 4096u
#define W25Q_MIN_CHUNK     64u
#define W25Q_MAX_CHUNK     (64u * 1024u)

/* ------------------------------------------------------------------ *
 *  Bus registry: one attached device per SPI number, re-attached when
 *  the caller changes the CS pin.
 * ------------------------------------------------------------------ */

/* Which stack issues the flash commands for a case. */
enum w25q_eng
{
    W25Q_ENG_RAW = 0,   /* rt_spi_send_then_recv + the command set below */
    W25Q_ENG_SFUD       /* sfud_read/sfud_write/sfud_erase               */
};

struct w25q_bus
{
    int spi_n;      /* 1..7, 0 = slot free               */
    rt_base_t cs_pin;
    rt_bool_t alt;        /* g_sig_alt this slot was muxed with */
    struct rt_spi_device *dev;       /* attached flash device             */
    void *sf_dev;    /* SFUD's block device over dev      */
};

#define W25Q_MAX_BUSES 7
static struct w25q_bus g_bus[W25Q_MAX_BUSES];

/* Recommended CS pin per SPI: an NSS alternate of that peripheral which is
 * free on this board's 4-wire loopback wiring. Only the default -- any free
 * GPIO works, so the caller can always override with cs= */
static const char * const g_rec_cs[8] = {
    RT_NULL, "PA4", "PA11", "PA15", "PA0", "PF6", "PE4", "PH13"
};

static const char *g_rec_pins[8] = {
    RT_NULL,
    "SCK=PA5 MISO=PA6 MOSI=PA7",
    "SCK=PD3 MISO=PC2 MOSI=PC3",
    "SCK=PB3 MISO=PB4 MOSI=PB2",
    "SCK=PG13 MISO=PG12 MOSI=PG14",
    "SCK=PF7 MISO=PF8 MOSI=PF9",
    "SCK=PE2 MISO=PE5 MOSI=PE6",
    "SCK=PI8 MISO=PI13 MOSI=PI14 (or PJ1/PJ3/PJ4, pins=alt)",
};

/* ------------------------------------------------------------------ *
 *  Pin muxing
 *
 *  The board's Cube_Config wires the six loopback SPIs by role: SPI1/3/5 come
 *  up as masters (SCK+MOSI AF_PP) and SPI2/4/6 as slaves (SCK+MOSI left as
 *  inputs, MISO AF_PP). A flash needs a MASTER, so attaching it to an even bus
 *  would leave SCK and MOSI undriven and the part would never be clocked.
 *  SPI7 has no Cube_Config entry at all. So each bus is re-muxed to master
 *  role here, for the bus about to be driven and no other.
 *
 *  AF numbers and pins are the chip's own table (UM N32H78X AFIO):
 *    SPI1 SCK PA5/4   MISO PA6/4   MOSI PA7/5
 *    SPI2 SCK PD3/4   MISO PC2/6   MOSI PC3/6
 *    SPI3 SCK PB3/2   MISO PB4/4   MOSI PB2/6
 *    SPI4 SCK PG13/5  MISO PG12/7  MOSI PG14/6
 *    SPI5 SCK PF7/4   MISO PF8/5   MOSI PF9/5
 *    SPI6 SCK PE2/4   MISO PE5/4   MOSI PE6/3
 *    SPI7 SCK PI8/3   MISO PI13/4  MOSI PI14/4
 * ------------------------------------------------------------------ */

struct w25q_sig
{
    char port;   /* 'A'..'K' */
    rt_uint8_t pin;    /* 0..15    */
    rt_uint8_t af;     /* alternate function number */
};

/* [spi_n][0]=SCK, [1]=MISO, [2]=MOSI. Index 0 is unused. */
static const struct w25q_sig g_sig[8][3] = {
    { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },   /* -    */
    { { 'A', 5, 4 }, { 'A', 6, 4 }, { 'A', 7, 5 } },
    { { 'D', 3, 4 }, { 'C', 2, 6 }, { 'C', 3, 6 } },
    { { 'B', 3, 2 }, { 'B', 4, 4 }, { 'B', 2, 6 } },
    { { 'G', 13, 5 }, { 'G', 12, 7 }, { 'G', 14, 6 } },
    { { 'F', 7, 4 }, { 'F', 8, 5 }, { 'F', 9, 5 } },
    { { 'E', 2, 4 }, { 'E', 5, 4 }, { 'E', 6, 3 } },
    { { 'I', 8, 3 }, { 'I', 13, 4 }, { 'I', 14, 4 } },
};

/* SPI7 is the one bus the AFIO table maps twice, and neither mapping needs a
 * remap bit (only SPI4 has a counterpart, AFIO_RMP_CFG.SPI4SEL). The table
 * above drives PI8/PI13/PI14; Cube_Config programs this PJ set instead. A
 * flash wired to whichever one the schematic calls "SPI7" can only be told
 * apart by the caller -- hence pins=alt. Guessing wrong is not subtle: the
 * pads the flash is not on stay undriven and JEDEC reads FF FF FF. */
static const struct w25q_sig g_sig7_alt[3] = {
    { 'J', 1, 4 },   /* SPI7_SCK  */
    { 'J', 3, 3 },   /* SPI7_MISO */
    { 'J', 4, 3 },   /* SPI7_MOSI */
};

/* Set by pins=alt in w25q_parse_opts(): a property of the wiring, not of one
 * case, so it lives outside struct w25q_opt. */
static rt_bool_t g_sig_alt = RT_FALSE;

/* The pins bus spi_n is driven on: [0]=SCK, [1]=MISO, [2]=MOSI. */
static const struct w25q_sig *w25q_sig_of(int spi_n, int idx)
{
    if (spi_n == 7 && g_sig_alt)
    {
        return &g_sig7_alt[idx];
    }
    return &g_sig[spi_n][idx];
}

/* "P<port><n>" for log lines. */
static const char *w25q_pin_str(rt_base_t pin, char *buf, rt_size_t len)
{
    rt_snprintf(buf, len, "P%c%d", (char)('A' + (pin / 16)), (int)(pin % 16));
    return buf;
}

/* Ports A..H hang off AHB5EN1, I..K off AHB5EN2; the second bank is not
 * enabled by Cube_Config, so SPI7's GPIOCLK has to be turned on here. */
static void w25q_port_clk_enable(char port)
{
    static const rt_uint32_t en1[] = {
        RCC_AHB5_PERIPHEN_M7_GPIOA,
        RCC_AHB5_PERIPHEN_M7_GPIOB,
        RCC_AHB5_PERIPHEN_M7_GPIOC,
        RCC_AHB5_PERIPHEN_M7_GPIOD,
        RCC_AHB5_PERIPHEN_M7_GPIOE,
        RCC_AHB5_PERIPHEN_M7_GPIOF,
        RCC_AHB5_PERIPHEN_M7_GPIOG,
        RCC_AHB5_PERIPHEN_M7_GPIOH,
    };
    static const rt_uint32_t en2[] = {
        RCC_AHB5_PERIPHEN_M7_GPIOI,
        RCC_AHB5_PERIPHEN_M7_GPIOJ,
        RCC_AHB5_PERIPHEN_M7_GPIOK,
    };
    int i = port - 'A';

    if (i >= 0 && i < (int)(sizeof(en1) / sizeof(en1[0])))
    {
        RCC_EnableAHB5PeriphClk1(en1[i], ENABLE);
    }
    else if (i >= 8 && i < 8 + (int)(sizeof(en2) / sizeof(en2[0])))
    {
        RCC_EnableAHB5PeriphClk2(en2[i - 8], ENABLE);
    }
}

static void w25q_mux_master(int spi_n)
{
    const struct w25q_sig *sck, *miso, *mosi;
    GPIO_InitType g;

    if (spi_n < 1 || spi_n > 7)
    {
        return;
    }

    sck = w25q_sig_of(spi_n, 0);
    miso = w25q_sig_of(spi_n, 1);
    mosi = w25q_sig_of(spi_n, 2);

    w25q_port_clk_enable(sck->port);
    w25q_port_clk_enable(miso->port);
    w25q_port_clk_enable(mosi->port);

    /* SCK + MOSI are the master's outputs: push-pull on their alternate. */
    GPIO_InitStruct(&g);
    g.GPIO_Mode = GPIO_MODE_AF_PP;
    g.GPIO_Pull = GPIO_NO_PULL;
    g.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    g.GPIO_Current = GPIO_DC_2mA;

    g.GPIO_Alternate = sck->af;
    g.Pin = 1u << sck->pin;
    GPIO_InitPeripheral(GPIO_GET_PERIPH(sck->port - 'A'), &g);

    g.GPIO_Alternate = mosi->af;
    g.Pin = 1u << mosi->pin;
    GPIO_InitPeripheral(GPIO_GET_PERIPH(mosi->port - 'A'), &g);

    /* MISO is an input; a pull-up keeps it defined while CS is high. */
    g.GPIO_Mode = GPIO_MODE_INPUT;
    g.GPIO_Pull = GPIO_PULL_UP;
    g.GPIO_Alternate = miso->af;
    g.Pin = 1u << miso->pin;
    GPIO_InitPeripheral(GPIO_GET_PERIPH(miso->port - 'A'), &g);
}

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

/* "PF6" -> RT-Thread pin number. Ports A..K are contiguous at a 0x400 stride
 * (n32h7xx.h), which is exactly what GET_PIN()'s arithmetic assumes. */
static rt_bool_t w25q_parse_pin(const char *s, rt_base_t *out)
{
    int port, num;

    if (s == RT_NULL || rt_strlen(s) < 2)
    {
        return RT_FALSE;
    }
    if (s[0] != 'P' && s[0] != 'p')
    {
        return RT_FALSE;
    }

    port = s[1];
    if (port >= 'a' && port <= 'z')
    {
        port -= 32;
    }
    if (port < 'A' || port > 'K')
    {
        return RT_FALSE;
    }
    if (s[2] == '\0')
    {
        return RT_FALSE;
    }

    num = 0;
    if (s[3] == '\0')
    {
        if (s[2] < '0' || s[2] > '9')
        {
            return RT_FALSE;
        }
        num = s[2] - '0';
    }
    else if (s[4] == '\0')
    {
        if (s[2] < '0' || s[2] > '9' || s[3] < '0' || s[3] > '9')
        {
            return RT_FALSE;
        }
        num = (s[2] - '0') * 10 + (s[3] - '0');
    }
    else
    {
        return RT_FALSE;
    }

    if (num > 15)
    {
        return RT_FALSE;
    }

    *out = (rt_base_t)(16 * (port - 'A') + num);
    return RT_TRUE;
}

/* Was TX+RX DMA compiled in for this bus? Raising spi_dma_flag on a bus whose
 * dma_rx/dma_tx are NULL would be dereferenced inside spixfer. */
static rt_bool_t w25q_dma_compiled(int spi_n)
{
    switch (spi_n)
    {
#if defined(BSP_SPI1_TX_USING_DMA) && defined(BSP_SPI1_RX_USING_DMA)
    case 1:
        return RT_TRUE;
#endif
#if defined(BSP_SPI2_TX_USING_DMA) && defined(BSP_SPI2_RX_USING_DMA)
    case 2:
        return RT_TRUE;
#endif
#if defined(BSP_SPI3_TX_USING_DMA) && defined(BSP_SPI3_RX_USING_DMA)
    case 3:
        return RT_TRUE;
#endif
#if defined(BSP_SPI4_TX_USING_DMA) && defined(BSP_SPI4_RX_USING_DMA)
    case 4:
        return RT_TRUE;
#endif
#if defined(BSP_SPI5_TX_USING_DMA) && defined(BSP_SPI5_RX_USING_DMA)
    case 5:
        return RT_TRUE;
#endif
#if defined(BSP_SPI6_TX_USING_DMA) && defined(BSP_SPI6_RX_USING_DMA)
    case 6:
        return RT_TRUE;
#endif
#if defined(BSP_SPI7_TX_USING_DMA) && defined(BSP_SPI7_RX_USING_DMA)
    case 7:
        return RT_TRUE;
#endif
    default:
        return RT_FALSE;
    }
}

/* The driver keeps its programmed prescaler in private state; reading it back
 * is the only way to report the rate the hardware is really running at, as
 * opposed to the one that was asked for. 150MHz / 2^n on this board, so
 * e.g. a 30MHz request lands on 18.75MHz and a 500kHz request on 585.9kHz. */
static rt_uint32_t w25q_actual_hz(struct n32_spi *drv)
{
    RCC_ClocksTypeDef clks = { 0 };
    rt_uint32_t clk;
    rt_uint32_t div;

    RCC_GetClocksFreqValue(&clks);

    if (drv->config->SPIx == SPI1 || drv->config->SPIx == SPI2)
    {
        clk = clks.APB2ClkFreq;
    }
    else if (drv->config->SPIx == SPI3)
    {
        clk = clks.APB1ClkFreq;
    }
    else
    {
        clk = clks.APB5ClkFreq;   /* SPI4..SPI7 */
    }

    div = 2u << (drv->SPI_InitStructure.BaudRatePres & 0x7u);
    return clk / div;
}

/* ------------------------------------------------------------------ *
 *  Attach / configure
 * ------------------------------------------------------------------ */

static struct n32_spi *w25q_drv(struct rt_spi_device *dev)
{
    return rt_container_of(dev->bus, struct n32_spi, spi_bus);
}

static rt_err_t w25q_dma_set(struct rt_spi_device *dev, rt_bool_t on)
{
    struct n32_spi *drv = w25q_drv(dev);

    drv->spi_dma_flag = on ? (rt_uint8_t)(SPI_USING_RX_DMA_FLAG | SPI_USING_TX_DMA_FLAG)
                           : (rt_uint8_t)0;
    return RT_EOK;
}

/* ------------------------------------------------------------------ *
 *  SFUD engine
 *
 *  rt_sfud_flash_probe() needs an already-attached rt_spi_device, so the attach
 *  above still runs first. SFUD then puts its own configuration on the bus for
 *  the probe (RT_SFUD_DEFAULT_SPI_CFG: mode 0, 8 bit, RT_SFUD_SPI_MAX_HZ) and
 *  leaves it there -- so w25q_run_case() re-applies the case's rate and mode
 *  after the probe, exactly as it does after any other reconfiguration.
 *
 *  Erase and page/wrap handling are deliberately not shared with the raw path:
 *  sfud_erase() rounds out to whole erase units the same way w25q_erase() does,
 *  and sfud_write() splits on 256B pages the same way w25q_write() does, so for
 *  one set of parameters the two engines put the same traffic on the wire. That
 *  is what makes a raw-vs-sfud diff of the same case meaningful.
 * ------------------------------------------------------------------ */

#if defined(RT_USING_SFUD)

static void w25q_sfud_release(struct w25q_bus *b)
{
    if (b->sf_dev != RT_NULL)
    {
        rt_sfud_flash_delete((rt_spi_flash_device_t)b->sf_dev);
        b->sf_dev = RT_NULL;
    }
    if (b->dev != RT_NULL)
    {
        /* rt_sfud_flash_probe() parks its block device here; it now points at
         * freed memory, and w25q_attach() is about to free dev as well. */
        b->dev->user_data = RT_NULL;
    }
}

/* rt_sfud_flash_probe() hands back its rt_spi_flash_device_t, and the
 * sfud_flash the read/write calls want hangs off that device's user_data --
 * the two are not the same pointer. Every caller gets the sfud_flash. */
static sfud_flash *w25q_sfud_of(rt_spi_flash_device_t dev)
{
    return (dev == RT_NULL) ? RT_NULL : (sfud_flash *)dev->user_data;
}

/* Probe on first use, then hand back the same handle for the life of this
 * attachment. NULL means the probe failed (no part, bad wiring, SFDP and ID
 * table both missed) -- the caller prints that, it is a result not a crash. */
static void *w25q_sfud_ensure(struct w25q_bus *b)
{
    char flash_name[12];
    char spi_name[12];

    if (b->sf_dev != RT_NULL)
    {
        return w25q_sfud_of((rt_spi_flash_device_t)b->sf_dev);
    }

    rt_snprintf(spi_name, sizeof(spi_name), "w25q%d", b->spi_n);
    rt_snprintf(flash_name, sizeof(flash_name), "sfud%d", b->spi_n);

    b->sf_dev = rt_sfud_flash_probe(flash_name, spi_name);

    if (b->sf_dev == RT_NULL)
    {
        /* rt_sfud_flash_probe() sets dev->user_data before it initialises the
         * chip, and its failure path frees that block without clearing the
         * back-pointer. Our device outlives the failed probe, so clear it here
         * rather than leave it aimed at freed memory. */
        b->dev->user_data = RT_NULL;
    }

    return w25q_sfud_of((rt_spi_flash_device_t)b->sf_dev);
}

static rt_err_t w25q_sfud_erase(void *sf, rt_uint32_t off, rt_uint32_t len)
{
    return (sfud_erase((const sfud_flash *)sf, off, len) == SFUD_SUCCESS) ? RT_EOK : -RT_EIO;
}

static rt_err_t w25q_sfud_write(void *sf, rt_uint32_t off,
                                const rt_uint8_t *buf, rt_uint32_t len)
{
    return (sfud_write((const sfud_flash *)sf, off, len, buf) == SFUD_SUCCESS) ? RT_EOK : -RT_EIO;
}

static rt_err_t w25q_sfud_read(void *sf, rt_uint32_t off,
                               rt_uint8_t *buf, rt_uint32_t len)
{
    return (sfud_read((const sfud_flash *)sf, off, len, buf) == SFUD_SUCCESS) ? RT_EOK : -RT_EIO;
}

#define W25Q_HAVE_SFUD 1

#else /* !RT_USING_SFUD */

/* Same signatures, so w25q_run_case() needs no #if of its own. eng=sfud is
 * refused at parse time in this build, so none of these are ever reached. */
static void w25q_sfud_release(struct w25q_bus *b)
{
    b->sf_dev = RT_NULL;
}

static void *w25q_sfud_ensure(struct w25q_bus *b)
{
    (void)b;
    return RT_NULL;
}

static rt_err_t w25q_sfud_erase(void *sf, rt_uint32_t off, rt_uint32_t len)
{
    (void)sf;
    (void)off;
    (void)len;
    return -RT_EIO;
}

static rt_err_t w25q_sfud_write(void *sf, rt_uint32_t off,
                                const rt_uint8_t *buf, rt_uint32_t len)
{
    (void)sf;
    (void)off;
    (void)buf;
    (void)len;
    return -RT_EIO;
}

static rt_err_t w25q_sfud_read(void *sf, rt_uint32_t off,
                               rt_uint8_t *buf, rt_uint32_t len)
{
    (void)sf;
    (void)off;
    (void)buf;
    (void)len;
    return -RT_EIO;
}

#define W25Q_HAVE_SFUD 0

#endif /* RT_USING_SFUD */

static rt_err_t w25q_attach(struct w25q_bus **out, int spi_n, rt_base_t cs_pin)
{
    struct w25q_bus *slot = RT_NULL;
    char bus_name[8];
    char dev_name[12];
    int i;

    for (i = 0; i < W25Q_MAX_BUSES; i++)
    {
        if (g_bus[i].spi_n == spi_n)
        {
            slot = &g_bus[i];
            break;
        }
    }
    if (slot == RT_NULL)
    {
        for (i = 0; i < W25Q_MAX_BUSES; i++)
        {
            if (g_bus[i].spi_n == 0)
            {
                slot = &g_bus[i];
                break;
            }
        }
    }
    if (slot == RT_NULL)
    {
        rt_kprintf("[w25q] no free bus slot\n");
        return -RT_ENOMEM;
    }

    rt_snprintf(bus_name, sizeof(bus_name), "spi%d", spi_n);
    rt_snprintf(dev_name, sizeof(dev_name), "w25q%d", spi_n);

    /* Same bus, same CS and same pin set: keep the attachment and its live
     * config. A pins= change has to fall through and re-mux the pads. */
    if (slot->spi_n == spi_n && slot->dev != RT_NULL && slot->cs_pin == cs_pin && slot->alt == g_sig_alt)
    {
        *out = slot;
        return RT_EOK;
    }

    /* CS changed under us: drop the old device before re-attaching, otherwise
     * the bus keeps an owner whose cs_pin is stale. SFUD's handle holds a
     * pointer to that device, so it goes first. */
    w25q_sfud_release(slot);
    if (slot->dev != RT_NULL)
    {
        rt_spi_bus_detach_device(slot->dev);
        rt_free(slot->dev);
        slot->dev = RT_NULL;
    }
    slot->spi_n = 0;

    if (rt_device_find(bus_name) == RT_NULL)
    {
        rt_kprintf("[w25q] bus %s not registered (BSP_USING_SPI%d off?)\n",
                   bus_name, spi_n);
        return -RT_EIO;
    }

    struct rt_spi_device *dev = (struct rt_spi_device *)rt_malloc(sizeof(*dev));
    if (dev == RT_NULL)
    {
        rt_kprintf("[w25q] out of heap for %s\n", dev_name);
        return -RT_ENOMEM;
    }

    rt_err_t res = rt_spi_bus_attach_device_cspin(dev, dev_name, bus_name, cs_pin, RT_NULL);
    if (res != RT_EOK)
    {
        rt_kprintf("[w25q] attach %s to %s failed, %d\n", dev_name, bus_name, res);
        rt_free(dev);
        return res;
    }

    /* Drive the chosen bus, not the role Cube_Config gave it. */
    w25q_mux_master(spi_n);

    slot->spi_n = spi_n;
    slot->cs_pin = cs_pin;
    slot->alt = g_sig_alt;
    slot->dev = dev;
    *out = slot;
    return RT_EOK;
}

/* ------------------------------------------------------------------ *
 *  Raw flash transactions
 * ------------------------------------------------------------------ */

static rt_err_t w25q_wait_ready(struct rt_spi_device *dev, rt_uint32_t timeout_ms)
{
    rt_uint8_t cmd = W25Q_CMD_RDSR1;
    rt_uint8_t sr;
    rt_uint32_t elapsed;

    for (elapsed = 0; elapsed < timeout_ms; elapsed++)
    {
        if (rt_spi_send_then_recv(dev, &cmd, 1, &sr, 1) != RT_EOK)
        {
            return -RT_EIO;
        }
        if ((sr & W25Q_SR1_WIP) == 0)
        {
            return RT_EOK;
        }
        rt_thread_mdelay(1);
    }
    return -RT_ETIMEOUT;
}

static rt_err_t w25q_op(struct rt_spi_device *dev, rt_uint8_t cmd,
                        rt_uint32_t addr, const rt_uint8_t *wbuf,
                        rt_uint32_t wlen, rt_uint32_t timeout_ms)
{
    rt_uint8_t hdr[4];

    if (rt_spi_send(dev, &(rt_uint8_t){ W25Q_CMD_WREN }, 1) != 1)
    {
        return -RT_EIO;
    }

    hdr[0] = cmd;
    hdr[1] = (rt_uint8_t)(addr >> 16);
    hdr[2] = (rt_uint8_t)(addr >> 8);
    hdr[3] = (rt_uint8_t)(addr);

    if (wbuf != RT_NULL && wlen != 0u)
    {
        if (rt_spi_send_then_send(dev, hdr, 4, wbuf, wlen) != RT_EOK)
        {
            return -RT_EIO;
        }
    }
    else if (rt_spi_send(dev, hdr, 4) != 4)
    {
        return -RT_EIO;
    }

    return w25q_wait_ready(dev, timeout_ms);
}

static rt_err_t w25q_read(struct rt_spi_device *dev, rt_uint32_t addr,
                          rt_uint8_t *buf, rt_uint32_t len)
{
    rt_uint8_t hdr[4];

    if (len == 0u)
    {
        return RT_EOK;
    }

    hdr[0] = W25Q_CMD_READ;
    hdr[1] = (rt_uint8_t)(addr >> 16);
    hdr[2] = (rt_uint8_t)(addr >> 8);
    hdr[3] = (rt_uint8_t)(addr);

    return rt_spi_send_then_recv(dev, hdr, 4, buf, len);
}

/* Erase every sector the range [off, off+len) touches. 64KB blocks are used
 * where the range is aligned, which is what keeps a 1MB case to ~2.4s instead
 * of the ~11.5s the same range costs in 4KB sectors. */
static rt_err_t w25q_erase(struct rt_spi_device *dev, rt_uint32_t off, rt_uint32_t len)
{
    while (len != 0u)
    {
        rt_uint32_t end = off + len;
        rt_uint32_t step;

        if ((off % W25Q_BLOCK_SIZE) == 0u && len >= W25Q_BLOCK_SIZE)
        {
            if (w25q_op(dev, W25Q_CMD_BLKERASE, off, RT_NULL, 0, W25Q_TIMEOUT_ERASE_MS) != RT_EOK)
            {
                return -RT_EIO;
            }
            step = W25Q_BLOCK_SIZE;
        }
        else
        {
            rt_uint32_t base = off & ~(W25Q_SECTOR_SIZE - 1u);

            if (w25q_op(dev, W25Q_CMD_SECERASE, base, RT_NULL, 0, W25Q_TIMEOUT_ERASE_MS) != RT_EOK)
            {
                return -RT_EIO;
            }
            /* First iteration may sit mid-sector: only advance to the sector
             * end, not a whole sector, so the range does not shrink too fast. */
            step = (base + W25Q_SECTOR_SIZE) - off;
        }

        off = (step >= len) ? end : (off + step);
        len = (step >= len) ? 0u : (len - step);
    }
    return RT_EOK;
}

/* Page program never crosses a 256B page boundary: the part wraps within the
 * page and would otherwise silently corrupt the start of the next one. */
static rt_err_t w25q_write(struct rt_spi_device *dev, rt_uint32_t off,
                           const rt_uint8_t *buf, rt_uint32_t len)
{
    while (len != 0u)
    {
        rt_uint32_t room = W25Q_PAGE_SIZE - (off % W25Q_PAGE_SIZE);
        rt_uint32_t n = (len < room) ? len : room;

        if (w25q_op(dev, W25Q_CMD_PAGEPROG, off, buf, n, W25Q_TIMEOUT_PROG_MS) != RT_EOK)
        {
            return -RT_EIO;
        }
        off += n;
        buf += n;
        len -= n;
    }
    return RT_EOK;
}

/* ------------------------------------------------------------------ *
 *  Test pattern
 * ------------------------------------------------------------------ */

/* Address-dependent so a page-wrap or address-aliasing fault changes the byte
 * at that offset; a plain (off & 0xff) pattern repeats every 256B and hides it. */
static rt_uint8_t w25q_pat(rt_uint32_t off)
{
    rt_uint32_t x = off * 2654435761u;

    x ^= x >> 15;
    return (rt_uint8_t)(x ^ (x >> 8));
}

/* ------------------------------------------------------------------ *
 *  Option parsing
 * ------------------------------------------------------------------ */

struct w25q_opt
{
    rt_uint32_t len;              /* payload bytes of this case */
    rt_base_t cs;
    rt_uint32_t hz_khz;
    rt_bool_t dma;
    rt_uint8_t mode;
    rt_uint32_t chunk;
    rt_uint32_t off;
    rt_bool_t noerase;
    enum w25q_eng eng;
};

static rt_bool_t w25q_arg_u32(const char *s, rt_uint32_t *out)
{
    rt_uint32_t v = 0;

    if (s == RT_NULL || *s == '\0')
    {
        return RT_FALSE;
    }
    while (*s != '\0')
    {
        if (*s < '0' || *s > '9')
        {
            return RT_FALSE;
        }
        v = v * 10u + (rt_uint32_t)(*s - '0');
        s++;
    }
    *out = v;
    return RT_TRUE;
}

/* key=value options shared by every subcommand. Returns the number of argv
 * entries consumed, or -1 on a malformed option. */
static int w25q_parse_opts(int argc, char **argv, int first, struct w25q_opt *o)
{
    int i;

    o->len = 0u;                /* callers that have a length set it after */
    o->cs = PIN_NONE;
    o->hz_khz = 18750u;            /* 150MHz/8, the top rate with margin  */
    o->dma = RT_TRUE;
    o->mode = 0u;
    o->chunk = W25Q_DEFAULT_CHUNK;
    o->off = 0u;
    o->noerase = RT_FALSE;
    o->eng = W25Q_ENG_RAW;
    g_sig_alt = RT_FALSE;          /* pins= is per command, like cs=      */

    for (i = first; i < argc; i++)
    {
        char *a = argv[i];
        char *eq = strchr(a, '=');
        rt_uint32_t v;

        if (eq == RT_NULL)
        {
            if (rt_strcmp(a, "noerase") == 0)
            {
                o->noerase = RT_TRUE;
                continue;
            }
            rt_kprintf("[w25q] unknown option '%s'\n", a);
            return -1;
        }

        *eq = '\0';

        if (rt_strcmp(a, "cs") == 0)
        {
            if (!w25q_parse_pin(eq + 1, &o->cs))
            {
                rt_kprintf("[w25q] bad cs pin '%s' (want e.g. PF6)\n", eq + 1);
                return -1;
            }
        }
        else if (rt_strcmp(a, "hz") == 0)
        {
            if (!w25q_arg_u32(eq + 1, &v) || v == 0u || v > 75000u)
            {
                rt_kprintf("[w25q] bad hz '%s' (kHz, 1..75000)\n", eq + 1);
                return -1;
            }
            o->hz_khz = v;
        }
        else if (rt_strcmp(a, "dma") == 0)
        {
            if (!w25q_arg_u32(eq + 1, &v) || v > 1u)
            {
                rt_kprintf("[w25q] bad dma '%s' (0|1)\n", eq + 1);
                return -1;
            }
            o->dma = (v != 0u) ? RT_TRUE : RT_FALSE;
        }
        else if (rt_strcmp(a, "mode") == 0)
        {
            if (!w25q_arg_u32(eq + 1, &v) || v > 3u)
            {
                rt_kprintf("[w25q] bad mode '%s' (0..3)\n", eq + 1);
                return -1;
            }
            o->mode = (rt_uint8_t)v;
        }
        else if (rt_strcmp(a, "chunk") == 0)
        {
            if (!w25q_arg_u32(eq + 1, &v) || v < W25Q_MIN_CHUNK || v > W25Q_MAX_CHUNK)
            {
                rt_kprintf("[w25q] bad chunk '%s' (%u..%u)\n", eq + 1, W25Q_MIN_CHUNK, W25Q_MAX_CHUNK);
                return -1;
            }
            o->chunk = v;
        }
        else if (rt_strcmp(a, "off") == 0)
        {
            if (!w25q_arg_u32(eq + 1, &v) || v >= W25Q_CAPACITY)
            {
                rt_kprintf("[w25q] bad off '%s' (<%u)\n", eq + 1, W25Q_CAPACITY);
                return -1;
            }
            o->off = v;
        }
        else if (rt_strcmp(a, "eng") == 0)
        {
            if (rt_strcmp(eq + 1, "raw") == 0)
            {
                o->eng = W25Q_ENG_RAW;
            }
            else if (rt_strcmp(eq + 1, "sfud") == 0)
            {
#if W25Q_HAVE_SFUD
                o->eng = W25Q_ENG_SFUD;
#else
                rt_kprintf("[w25q] eng=sfud needs RT_USING_SFUD in rtconfig.h "
                           "(and the sfud sources in the Keil project)\n");
                return -1;
#endif
            }
            else
            {
                rt_kprintf("[w25q] bad eng '%s' (raw|sfud)\n", eq + 1);
                return -1;
            }
        }
        else if (rt_strcmp(a, "pins") == 0)
        {
            /* Which SPI7 mapping the flash is physically wired to. No effect
             * on any other bus; they all have exactly one pin set. */
            if (rt_strcmp(eq + 1, "alt") == 0 || rt_strcmp(eq + 1, "pj") == 0)
            {
                g_sig_alt = RT_TRUE;
            }
            else if (rt_strcmp(eq + 1, "def") == 0 || rt_strcmp(eq + 1, "pi") == 0)
            {
                g_sig_alt = RT_FALSE;
            }
            else
            {
                rt_kprintf("[w25q] bad pins '%s' (def|alt, spi7 only)\n", eq + 1);
                return -1;
            }
        }
        else
        {
            rt_kprintf("[w25q] unknown option '%s'\n", a);
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ *
 *  One read-write-verify case
 * ------------------------------------------------------------------ */

struct w25q_case
{
    rt_uint32_t len;
    rt_uint32_t hz_req_khz;
    rt_uint32_t hz_act_hz;
    rt_bool_t dma;
    rt_uint8_t mode;
    rt_bool_t mode_rise;     /* this case followed a 0 -> 3 switch */
    rt_uint32_t chunk;
    rt_uint32_t off;
    enum w25q_eng eng;
    rt_uint8_t erase_state;   /* 0 = failed, 1 = erased, 2 = skipped */
    rt_uint8_t verify_ran;    /* 0 = aborted before comparing bytes  */
    rt_uint32_t verr_off;      /* first mismatch, 0xffffffff = none */
    rt_uint8_t verr_exp;
    rt_uint8_t verr_act;
    rt_uint32_t erase_ms;
    rt_uint32_t write_ms;
    rt_uint32_t read_ms;
};

static const char *w25q_case_line(const struct w25q_case *c, int spi_n,
                                  const char *cs_str, const char *verdict,
                                  rt_uint32_t hz_req_khz_used)
{
    static char line[320];

    rt_snprintf(line, sizeof(line),
                "spi=%d cs=%s dma=%d eng=%s hz_req=%u hz_act=%u mode=%u len=%u "
                "chunk=%u off=%u erase=%s write=%u ms read=%u ms verify=%s => %s",
                spi_n, cs_str, c->dma ? 1 : 0,
                (c->eng == W25Q_ENG_SFUD) ? "sfud" : "raw", hz_req_khz_used,
                (unsigned)(c->hz_act_hz / 1000u), (unsigned)c->mode,
                (unsigned)c->len, (unsigned)c->chunk, (unsigned)c->off,
                (c->erase_state == 1u) ? "ok" : ((c->erase_state == 2u) ? "skip" : "ERR"),
                (unsigned)c->write_ms, (unsigned)c->read_ms,
                /* verr_off is 0xffffffff both when every byte matched and when
                 * the case aborted before it compared anything; verify_ran
                 * separates the two. */
                (!c->verify_ran) ? "-"
                                 : ((c->verr_off == 0xffffffffu) ? "ok" : "ERR"),
                verdict);

    if (c->verify_ran && c->verr_off != 0xffffffffu)
    {
        rt_size_t used = rt_strlen(line);
        rt_snprintf(line + used, sizeof(line) - used,
                    " [first_bad@%u exp=%02X act=%02X]",
                    (unsigned)c->verr_off, c->verr_exp, c->verr_act);
    }
    return line;
}

static rt_err_t w25q_run_case(struct w25q_bus *b, const struct w25q_opt *o,
                              struct w25q_case *c)
{
    struct rt_spi_device *dev = b->dev;
    struct rt_spi_configuration cfg = { 0 };
    void *sf = RT_NULL;
    rt_uint8_t *buf = RT_NULL;
    rt_uint32_t done;
    rt_uint32_t t0;
    rt_err_t res;

    c->erase_state = 0u;
    c->verify_ran = 0u;
    c->mode_rise = RT_FALSE;
    c->write_ms = 0u;
    c->read_ms = 0u;
    c->verr_off = 0xffffffffu;
    c->eng = o->eng;

    /* DMA goes on first: the SFUD probe issues real transfers, and they should
     * run under the same engine as the case that asked for them. */
    res = w25q_dma_set(dev, o->dma);
    if (res != RT_EOK)
    {
        return res;
    }

    if (o->eng == W25Q_ENG_SFUD)
    {
        sf = w25q_sfud_ensure(b);
        if (sf == RT_NULL)
        {
            rt_kprintf("[w25q] sfud probe failed on spi%d -- the part answered "
                       "neither SFDP nor the JEDEC ID table, check wiring\n",
                       b->spi_n);
            return -RT_EIO;
        }
        /* The probe left its own configuration on the bus. The case's goes back
         * on below -- the rt_spi_device is untouched, so sf stays valid. */
    }

    cfg.data_width = 8u;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MSB | (rt_uint8_t)(o->mode & 0x3u);
    cfg.max_hz = o->hz_khz * 1000u;

    {
        /* Switching CPOL/CPHA is the one configuration change that has been seen
         * to lose the first transfer after it, so remember whether this case is
         * the far side of such a switch -- the JEDEC gate below says so if it
         * trips.
         *
         * Ask the device rather than a variable of our own: rt_spi_configure()
         * records the mode here (dev_spi_core.c:287) on every path that touches
         * this bus, including the ones this file does not drive -- the SFUD
         * probe above, and the `w25q id` command's own configure. A hand-kept
         * copy misses exactly those, and then stays silent about a real 0 -> 3
         * switch when it should not be.
         *
         * Mask off CPOL/CPHA rather than comparing to RT_SPI_MODE_0: what gets
         * stored is cfg->mode & RT_SPI_MODE_MASK, and that mask also carries
         * RT_SPI_MSB -- which this file sets -- so a stored 0 means mode 0 and
         * RT_SPI_MODE_0 is only ever equal to it by accident. */
        rt_bool_t mode_rise =
            ((dev->config.mode & (RT_SPI_CPOL | RT_SPI_CPHA)) == 0u) &&
            ((o->mode & 0x3u) == 3u);

        res = rt_spi_configure(dev, &cfg);
        if (res != RT_EOK)
        {
            return res;
        }
        c->mode_rise = mode_rise;
    }
    c->hz_act_hz = w25q_actual_hz(w25q_drv(dev));

    /* Verify the part answers before touching it. */
    {
        rt_uint8_t id_cmd = W25Q_CMD_JEDECID;
        rt_uint8_t id[3] = { 0, 0, 0 };

        if (rt_spi_send_then_recv(dev, &id_cmd, 1, id, 3) != RT_EOK)
        {
            return -RT_EIO;
        }
        if (id[0] != 0xEFu)
        {
            rt_kprintf("[w25q] JEDEC ID %02X %02X %02X is not a W25Q "
                       "(want EF .. ..)\n",
                       id[0], id[1], id[2]);
            if (c->mode_rise)
            {
                /* Measured on this board: the first transfer after 0 -> 3 reads
                 * all-ones or all-zeroes at every rate, with and without DMA,
                 * while the reverse switch and a second transfer at mode 3 are
                 * both clean. Retry at this mode before suspecting the wiring. */
                rt_kprintf("[w25q] note: this bus just went mode 0 -> 3, and the "
                           "first transfer after that switch is a known failure "
                           "here -- re-run this case to tell that apart from a "
                           "wiring fault\n");
            }
            else
            {
                rt_kprintf("[w25q] check MISO/MOSI/CS wiring\n");
            }
            return -RT_EIO;
        }
    }

    if (!o->noerase)
    {
        t0 = rt_tick_get();
        res = (o->eng == W25Q_ENG_SFUD) ? w25q_sfud_erase(sf, o->off, o->len)
                                        : w25q_erase(dev, o->off, o->len);
        c->erase_ms = (rt_tick_get() - t0) * 1000u / RT_TICK_PER_SECOND;
        if (res != RT_EOK)
        {
            return res;
        }
        c->erase_state = 1u;
    }
    else
    {
        /* Caller claims the range is already erased. Report it as skipped, not
         * as ok: nothing was done, and an unwritten flash will fail the verify
         * with 0xFF, which should not be read as a driver fault. */
        c->erase_state = 2u;
    }

    buf = (rt_uint8_t *)rt_malloc(o->chunk);
    if (buf == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    t0 = rt_tick_get();
    for (done = 0u; done < o->len;)
    {
        rt_uint32_t n = o->len - done;
        rt_uint32_t i;

        if (n > o->chunk)
        {
            n = o->chunk;
        }
        for (i = 0u; i < n; i++)
        {
            buf[i] = w25q_pat(o->off + done + i);
        }
        res = (o->eng == W25Q_ENG_SFUD) ? w25q_sfud_write(sf, o->off + done, buf, n)
                                        : w25q_write(dev, o->off + done, buf, n);
        if (res != RT_EOK)
        {
            rt_free(buf);
            return res;
        }
        done += n;
    }
    c->write_ms = (rt_tick_get() - t0) * 1000u / RT_TICK_PER_SECOND;

    t0 = rt_tick_get();
    for (done = 0u; done < o->len;)
    {
        rt_uint32_t n = o->len - done;
        rt_uint32_t i;

        if (n > o->chunk)
        {
            n = o->chunk;
        }
        res = (o->eng == W25Q_ENG_SFUD) ? w25q_sfud_read(sf, o->off + done, buf, n)
                                        : w25q_read(dev, o->off + done, buf, n);
        if (res != RT_EOK)
        {
            rt_free(buf);
            return res;
        }
        c->verify_ran = 1u;   /* from here on verr_off means something */
        for (i = 0u; i < n; i++)
        {
            rt_uint8_t exp = w25q_pat(o->off + done + i);

            if (buf[i] != exp)
            {
                c->verr_off = o->off + done + i;
                c->verr_exp = exp;
                c->verr_act = buf[i];
                c->read_ms = (rt_tick_get() - t0) * 1000u / RT_TICK_PER_SECOND;
                rt_free(buf);
                /* The mismatch itself is the finding, and the caller prints it
                 * from c -- but this still has to come back as an error: the
                 * callers take RT_EOK to mean PASS, so returning it here would
                 * count a corrupted transfer as a pass. */
                return -RT_EIO;
            }
        }
        done += n;
    }
    c->read_ms = (rt_tick_get() - t0) * 1000u / RT_TICK_PER_SECOND;

    rt_free(buf);
    return RT_EOK;
}

/* Shared prologue: parse bus/CS, optionally re-attach, print the header. */
static rt_bool_t w25q_check_dma(int spi_n, rt_bool_t want)
{
    if (!want)
    {
        return RT_TRUE;
    }
    if (!w25q_dma_compiled(spi_n))
    {
        rt_kprintf("[w25q] spi%d was built without DMA -- use dma=0, or add "
                   "BSP_SPI%d_TX/RX_USING_DMA to rtconfig.h and reflash\n",
                   spi_n, spi_n);
        return RT_FALSE;
    }
    return RT_TRUE;
}

/* ------------------------------------------------------------------ *
 *  msh: w25q list
 * ------------------------------------------------------------------ */

static void w25q_list(void)
{
    int i;
    int found = 0;

    rt_kprintf("[w25q] SPI buses registered in this build:\n");
    for (i = 1; i <= 7; i++)
    {
        char bus_name[8];
        char dev_name[12];

        rt_snprintf(bus_name, sizeof(bus_name), "spi%d", i);
        rt_snprintf(dev_name, sizeof(dev_name), "w25q%d", i);

        if (rt_device_find(bus_name) == RT_NULL)
        {
            continue;
        }
        found++;

        rt_kprintf("  spi%d  dma_build=%d  default_cs=%-5s  %s  (%s)\n",
                   i, w25q_dma_compiled(i) ? 1 : 0, g_rec_cs[i],
                   g_rec_pins[i],
                   (rt_device_find(dev_name) != RT_NULL) ? "flash attached" : "no flash yet");
    }
    if (found == 0)
    {
        rt_kprintf("  none -- BSP_USING_SPI* is off\n");
    }

#if W25Q_HAVE_SFUD
    rt_kprintf("\n[w25q] engines: raw, sfud (RT_USING_SFUD is on, so the 'sf' "
               "command is available too)\n");
#else
    rt_kprintf("\n[w25q] engines: raw only (RT_USING_SFUD is off, so eng=sfud "
               "is refused)\n");
#endif

    rt_kprintf("[w25q] wiring for a case: flash VCC/GND, then\n"
               "       CLK -> that bus's SCK, DI(IO0) -> MOSI, DO(IO1) -> MISO,\n"
               "       CS  -> the default_cs pin above (or any free GPIO via cs=)\n"
               "       Only one bus can drive the flash at a time.\n");
}

/* ------------------------------------------------------------------ *
 *  msh: w25q id <spi>
 * ------------------------------------------------------------------ */

static int w25q_cmd_id(int argc, char **argv)
{
    struct w25q_opt o;
    struct w25q_bus *b;
    rt_uint8_t id_cmd = W25Q_CMD_JEDECID;
    rt_uint8_t id[3] = { 0, 0, 0 };
    rt_uint8_t sr = 0;
    rt_uint8_t sr_cmd = W25Q_CMD_RDSR1;
    rt_uint32_t spi_n;
    rt_base_t cs;
    rt_err_t res;
    char cs_str[8];

    if (argc < 3)
    {
        rt_kprintf("usage: w25q id <spi 1..7> [cs=PFx] [hz=K] [dma=0|1] [mode=N]\n");
        return -1;
    }
    if (!w25q_arg_u32(argv[2], &spi_n) || spi_n < 1u || spi_n > 7u)
    {
        rt_kprintf("[w25q] bad spi '%s' (1..7)\n", argv[2]);
        return -1;
    }
    if (w25q_parse_opts(argc, argv, 3, &o) != 0)
    {
        return -1;
    }
    if (o.cs == PIN_NONE && !w25q_parse_pin(g_rec_cs[spi_n], &o.cs))
    {
        rt_kprintf("[w25q] no default CS for spi%u, pass cs=PFx\n", (unsigned)spi_n);
        return -1;
    }
    if (!w25q_check_dma((int)spi_n, o.dma))
    {
        return -1;
    }

    res = w25q_attach(&b, (int)spi_n, o.cs);
    if (res != RT_EOK)
    {
        return -1;
    }

    struct rt_spi_configuration cfg = { 0 };
    cfg.data_width = 8u;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MSB | (rt_uint8_t)(o.mode & 0x3u);
    cfg.max_hz = o.hz_khz * 1000u;
    if (rt_spi_configure(b->dev, &cfg) != RT_EOK)
    {
        rt_kprintf("[w25q] configure failed\n");
        return -1;
    }
    w25q_dma_set(b->dev, o.dma);

    cs = o.cs;
    w25q_pin_str(cs, cs_str, sizeof(cs_str));

    if (rt_spi_send_then_recv(b->dev, &id_cmd, 1, id, 3) != RT_EOK)
    {
        rt_kprintf("[w25q] id spi=%u cs=%s dma=%d hz_req=%u hz_act=%u => FAIL(transfer)\n",
                   (unsigned)spi_n, cs_str, o.dma ? 1 : 0, (unsigned)o.hz_khz,
                   (unsigned)(w25q_actual_hz(w25q_drv(b->dev)) / 1000u));
        return -1;
    }
    rt_spi_send_then_recv(b->dev, &sr_cmd, 1, &sr, 1);

    rt_kprintf("[w25q] id spi=%u cs=%s dma=%d hz_req=%u hz_act=%u mode=%u "
               "jedec=%02X %02X %02X sr1=%02X => %s\n",
               (unsigned)spi_n, cs_str, o.dma ? 1 : 0, (unsigned)o.hz_khz,
               (unsigned)(w25q_actual_hz(w25q_drv(b->dev)) / 1000u), (unsigned)o.mode,
               id[0], id[1], id[2], sr,
               (id[0] == 0xEFu) ? "PASS" : "FAIL(id)");

#if W25Q_HAVE_SFUD
    /* The same part identified by the framework rather than by the command set:
     * SFUD probes from scratch (JEDEC ID, then SFDP or its own chip table) and
     * reports the geometry every later sfud_read/sfud_write will use. A JEDEC
     * match above with a probe failure here points at SFDP, not at the wire. */
    if (o.eng == W25Q_ENG_SFUD)
    {
        void *sf = w25q_sfud_ensure(b);

        if (sf == RT_NULL)
        {
            rt_kprintf("[w25q] sfud spi=%u => FAIL(probe)\n", (unsigned)spi_n);
            return -1;
        }
        {
            const sfud_flash *f = (const sfud_flash *)sf;

#ifdef SFUD_USING_SFDP
            int sfdp_ok = f->sfdp.available ? 1 : 0;
#else
            int sfdp_ok = 0;
#endif
            /* sfud.c:284 clears chip.name when the geometry came from SFDP
             * instead of the static chip table -- the table is the only place a
             * part name is ever recorded. Print "-" there rather than hand NULL
             * to %s, which is undefined behaviour even though armcc renders it. */
            const char *nm = (f->chip.name != RT_NULL) ? f->chip.name : "-";

            rt_kprintf("[w25q] sfud spi=%u name=%s jedec=%02X %02X %02X cap=%lu "
                       "write_mode=%04X erase_gran=%lu erase_cmd=%02X sfdp=%d => PASS(probe)\n",
                       (unsigned)spi_n, nm,
                       f->chip.mf_id, f->chip.type_id, f->chip.capacity_id,
                       (unsigned long)f->chip.capacity, (unsigned)f->chip.write_mode,
                       (unsigned long)f->chip.erase_gran, f->chip.erase_gran_cmd,
                       sfdp_ok);
        }
    }
#endif /* W25Q_HAVE_SFUD */
    return 0;
}

/* ------------------------------------------------------------------ *
 *  msh: w25q rw <spi> <len>
 * ------------------------------------------------------------------ */

static int w25q_cmd_rw(int argc, char **argv)
{
    struct w25q_opt o;
    struct w25q_bus *b;
    struct w25q_case c = { 0 };
    rt_uint32_t spi_n, len;
    rt_base_t cs;
    rt_err_t res;
    char cs_str[8];

    if (argc < 4)
    {
        rt_kprintf("usage: w25q rw <spi 1..7> <len bytes> [cs=] [hz=] [dma=] [mode=] "
                   "[chunk=] [off=] [noerase]\n");
        return -1;
    }
    if (!w25q_arg_u32(argv[2], &spi_n) || spi_n < 1u || spi_n > 7u)
    {
        rt_kprintf("[w25q] bad spi '%s' (1..7)\n", argv[2]);
        return -1;
    }
    if (!w25q_arg_u32(argv[3], &len) || len == 0u)
    {
        rt_kprintf("[w25q] bad len '%s'\n", argv[3]);
        return -1;
    }
    if (w25q_parse_opts(argc, argv, 4, &o) != 0)
    {
        return -1;
    }
    if (len > W25Q_CAPACITY || o.off + len > W25Q_CAPACITY)
    {
        rt_kprintf("[w25q] len=%u off=%u exceeds the 16MB part\n",
                   (unsigned)len, (unsigned)o.off);
        return -1;
    }
    if (o.cs == PIN_NONE && !w25q_parse_pin(g_rec_cs[spi_n], &o.cs))
    {
        rt_kprintf("[w25q] no default CS for spi%u, pass cs=PFx\n", (unsigned)spi_n);
        return -1;
    }
    if (!w25q_check_dma((int)spi_n, o.dma))
    {
        return -1;
    }

    res = w25q_attach(&b, (int)spi_n, o.cs);
    if (res != RT_EOK)
    {
        return -1;
    }

    cs = o.cs;
    w25q_pin_str(cs, cs_str, sizeof(cs_str));

    o.len = len;
    c.len = len;
    c.dma = o.dma;
    c.mode = o.mode;
    c.chunk = o.chunk;
    c.off = o.off;
    c.eng = o.eng;

    res = w25q_run_case(b, &o, &c);

    rt_kprintf("[w25q] case %s\n",
               w25q_case_line(&c, (int)spi_n, cs_str,
                              (res == RT_EOK) ? "PASS" : "FAIL", o.hz_khz));
    if (res == -RT_ENOMEM)
    {
        rt_kprintf("[w25q] (harness out of heap for chunk=%u)\n", (unsigned)o.chunk);
    }
    return (res == RT_EOK) ? 0 : -1;
}

/* ------------------------------------------------------------------ *
 *  msh: w25q sweep <spi>   -- the size ladder
 * ------------------------------------------------------------------ */

static const rt_uint32_t g_sizes[] = {
    32u, 64u, 128u, 256u, 512u, 1024u, 4096u, 16384u, 65536u, 262144u, 1048576u
};

static int w25q_cmd_sweep(int argc, char **argv)
{
    struct w25q_opt o;
    struct w25q_bus *b;
    rt_uint32_t spi_n, i;
    rt_base_t cs;
    rt_err_t res;
    int pass = 0, fail = 0;
    char cs_str[8];

    if (argc < 3)
    {
        rt_kprintf("usage: w25q sweep <spi 1..7> [cs=] [hz=] [dma=] [mode=] [chunk=] [off=]\n");
        return -1;
    }
    if (!w25q_arg_u32(argv[2], &spi_n) || spi_n < 1u || spi_n > 7u)
    {
        rt_kprintf("[w25q] bad spi '%s' (1..7)\n", argv[2]);
        return -1;
    }
    if (w25q_parse_opts(argc, argv, 3, &o) != 0)
    {
        return -1;
    }
    if (o.cs == PIN_NONE && !w25q_parse_pin(g_rec_cs[spi_n], &o.cs))
    {
        rt_kprintf("[w25q] no default CS for spi%u, pass cs=PFx\n", (unsigned)spi_n);
        return -1;
    }
    if (!w25q_check_dma((int)spi_n, o.dma))
    {
        return -1;
    }

    res = w25q_attach(&b, (int)spi_n, o.cs);
    if (res != RT_EOK)
    {
        return -1;
    }

    cs = o.cs;
    w25q_pin_str(cs, cs_str, sizeof(cs_str));

    rt_kprintf("[w25q] sweep start spi=%u cs=%s dma=%d eng=%s hz_req=%u mode=%u sizes=%u\n",
               (unsigned)spi_n, cs_str, o.dma ? 1 : 0,
               (o.eng == W25Q_ENG_SFUD) ? "sfud" : "raw", (unsigned)o.hz_khz,
               (unsigned)o.mode, (unsigned)(sizeof(g_sizes) / sizeof(g_sizes[0])));

    for (i = 0u; i < sizeof(g_sizes) / sizeof(g_sizes[0]); i++)
    {
        struct w25q_opt one = o;
        struct w25q_case c = { 0 };

        one.len = g_sizes[i];
        if (one.off + one.len > W25Q_CAPACITY)
        {
            continue;
        }

        c.len = one.len;
        c.dma = one.dma;
        c.mode = one.mode;
        c.chunk = one.chunk;
        c.off = one.off;
        c.eng = one.eng;

        res = w25q_run_case(b, &one, &c);

        rt_kprintf("[w25q] case %s\n",
                   w25q_case_line(&c, (int)spi_n, cs_str,
                                  (res == RT_EOK) ? "PASS" : "FAIL", one.hz_khz));
        if (res == RT_EOK)
        {
            pass++;
        }
        else
        {
            fail++;
            if (res == -RT_ENOMEM)
            {
                rt_kprintf("[w25q] (harness out of heap for chunk=%u)\n", (unsigned)one.chunk);
            }
        }
    }

    rt_kprintf("[w25q] sweep done spi=%u dma=%d eng=%s hz_req=%u pass=%d fail=%d\n",
               (unsigned)spi_n, o.dma ? 1 : 0,
               (o.eng == W25Q_ENG_SFUD) ? "sfud" : "raw", (unsigned)o.hz_khz, pass, fail);
    return (fail == 0) ? 0 : -1;
}

/* ------------------------------------------------------------------ *
 *  msh: w25q rate <spi> <len>   -- the rate ladder
 * ------------------------------------------------------------------ */

/* 500k..30M as asked for. The part is clocked from a 150MHz APB through a
 * power-of-two prescaler, so only 150MHz/2^n is reachable: 30M lands on
 * 18.75M and 500k on 585.9k. hz_act in each verdict line is what was actually
 * programmed -- compare against hz_req, do not assume they match. */
static const rt_uint32_t g_rates_khz[] = {
    500u, 750u, 1000u, 2000u, 4000u, 8000u, 15000u, 30000u
};

static int w25q_cmd_rate(int argc, char **argv)
{
    struct w25q_opt o;
    struct w25q_bus *b;
    rt_uint32_t spi_n, len, i;
    rt_base_t cs;
    rt_err_t res;
    int pass = 0, fail = 0;
    char cs_str[8];

    if (argc < 4)
    {
        rt_kprintf("usage: w25q rate <spi 1..7> <len bytes> [cs=] [dma=] [mode=] [chunk=] [off=]\n");
        return -1;
    }
    if (!w25q_arg_u32(argv[2], &spi_n) || spi_n < 1u || spi_n > 7u)
    {
        rt_kprintf("[w25q] bad spi '%s' (1..7)\n", argv[2]);
        return -1;
    }
    if (!w25q_arg_u32(argv[3], &len) || len == 0u)
    {
        rt_kprintf("[w25q] bad len '%s'\n", argv[3]);
        return -1;
    }
    if (w25q_parse_opts(argc, argv, 4, &o) != 0)
    {
        return -1;
    }
    if (len > W25Q_CAPACITY || o.off + len > W25Q_CAPACITY)
    {
        rt_kprintf("[w25q] len=%u off=%u exceeds the 16MB part\n",
                   (unsigned)len, (unsigned)o.off);
        return -1;
    }
    if (o.cs == PIN_NONE && !w25q_parse_pin(g_rec_cs[spi_n], &o.cs))
    {
        rt_kprintf("[w25q] no default CS for spi%u, pass cs=PFx\n", (unsigned)spi_n);
        return -1;
    }
    if (!w25q_check_dma((int)spi_n, o.dma))
    {
        return -1;
    }

    res = w25q_attach(&b, (int)spi_n, o.cs);
    if (res != RT_EOK)
    {
        return -1;
    }

    cs = o.cs;
    w25q_pin_str(cs, cs_str, sizeof(cs_str));

    rt_kprintf("[w25q] rate start spi=%u cs=%s dma=%d eng=%s mode=%u len=%u\n",
               (unsigned)spi_n, cs_str, o.dma ? 1 : 0,
               (o.eng == W25Q_ENG_SFUD) ? "sfud" : "raw", (unsigned)o.mode, (unsigned)len);

    for (i = 0u; i < sizeof(g_rates_khz) / sizeof(g_rates_khz[0]); i++)
    {
        struct w25q_opt one = o;
        struct w25q_case c = { 0 };

        one.len = len;
        one.hz_khz = g_rates_khz[i];

        c.len = one.len;
        c.dma = one.dma;
        c.mode = one.mode;
        c.chunk = one.chunk;
        c.off = one.off;
        c.eng = one.eng;

        res = w25q_run_case(b, &one, &c);

        rt_kprintf("[w25q] case %s\n",
                   w25q_case_line(&c, (int)spi_n, cs_str,
                                  (res == RT_EOK) ? "PASS" : "FAIL", one.hz_khz));
        if (res == RT_EOK)
        {
            pass++;
        }
        else
        {
            fail++;
            if (res == -RT_ENOMEM)
            {
                rt_kprintf("[w25q] (harness out of heap for chunk=%u)\n", (unsigned)one.chunk);
            }
        }
    }

    rt_kprintf("[w25q] rate done spi=%u dma=%d eng=%s len=%u pass=%d fail=%d\n",
               (unsigned)spi_n, o.dma ? 1 : 0,
               (o.eng == W25Q_ENG_SFUD) ? "sfud" : "raw", (unsigned)len, pass, fail);
    return (fail == 0) ? 0 : -1;
}

/* ------------------------------------------------------------------ *
 *  Register probe
 *
 *  A bus that moves no data has two very different causes and the console
 *  output cannot tell them apart: the SPI engine itself is not running
 *  (bus clock gate off, peripheral held in reset, base address wrong), or
 *  the engine runs but the pads it drives are not the ones the flash is
 *  wired to. `w25q probe` splits the two.
 *
 *  A peripheral whose bus clock is gated answers 0 to every read and drops
 *  every write, so writing a scratch register inside the SPI block and
 *  reading it back proves whether the engine is alive at all. The pin mux
 *  is read back the same way: a pad that still reads IN after
 *  w25q_mux_master() ran means the port clock or the AF number is wrong,
 *  and no amount of re-driving the bus will help.
 *
 *  SPI7 additionally dumps the Cube_Config pin set (PJ1/PJ3/PJ4) next to
 *  the one this file drives (PI8/PI13/PI14). UM N32H78X AFIO lists both as
 *  valid SPI7 mappings -- neither is a typo -- so the readback is what says
 *  which pads the flash is actually soldered to.
 * ------------------------------------------------------------------ */

static const char *w25q_pmode_str(rt_uint32_t m)
{
    switch (m & 0x3u)
    {
    case 0u:
        return "IN";
    case 1u:
        return "OUT";
    case 2u:
        return "AF";
    default:
        return "ANALOG";
    }
}

static const char *w25q_pupd_str(rt_uint32_t p)
{
    switch (p & 0x3u)
    {
    case 0u:
        return "none";
    case 1u:
        return "up";
    case 2u:
        return "down";
    default:
        return "-";
    }
}

static void w25q_probe_pin(const char *tag, char port, rt_uint8_t pin)
{
    GPIO_Module *p = GPIO_GET_PERIPH(port - 'A');
    rt_uint32_t mode = (p->PMODE >> (2u * pin)) & 0x3u;
    rt_uint32_t pupd = (p->PUPD >> (2u * pin)) & 0x3u;
    rt_uint32_t af = (pin < 8u) ? ((p->AFL >> (4u * pin)) & 0xFu)
                                : ((p->AFH >> (4u * (pin - 8u))) & 0xFu);

    rt_kprintf("[probe]   %-4s P%c%-2u mode=%-6s pull=%-4s af=%-2u in=%u\n",
               tag, port, (unsigned)pin, w25q_pmode_str(mode),
               w25q_pupd_str(pupd), (unsigned)af,
               (unsigned)((p->PID >> pin) & 1u));
}

static void w25q_probe_dump_spi(const char *tag, SPI_Module *spi)
{
    /* SPE is CTRL2 bit 0; the std driver keeps that bit's name private to
     * its own .c file, so ask the register directly. */
    rt_kprintf("[probe] %s CTRL1=%04X CTRL2=%04X(spe=%u) STS=%04X "
               "CFGR=%04X FIFONUM=%04X TRANSNUM=%04X CR3=%04X\n",
               tag, (unsigned)spi->CTRL1, (unsigned)spi->CTRL2,
               (unsigned)((spi->CTRL2 & 0x0001u) ? 1u : 0u),
               (unsigned)spi->STS, (unsigned)spi->SPI_I2S_CFGR,
               (unsigned)spi->FIFONUM, (unsigned)spi->TRANSNUM,
               (unsigned)spi->CR3);
}

static int w25q_cmd_probe(int argc, char **argv)
{
    struct w25q_opt o;
    struct w25q_bus *b;
    SPI_Module *spi;
    rt_uint32_t spi_n;
    rt_uint8_t id_cmd = W25Q_CMD_JEDECID;
    rt_uint8_t id[3] = { 0, 0, 0 };
    rt_uint16_t saved, rb;
    rt_err_t res;
    char cs_str[8];

    if (argc < 3)
    {
        rt_kprintf("usage: w25q probe <spi 1..7> [cs=PFx] [hz=K] [dma=0|1] [mode=N]\n");
        return -1;
    }
    if (!w25q_arg_u32(argv[2], &spi_n) || spi_n < 1u || spi_n > 7u)
    {
        rt_kprintf("[w25q] bad spi '%s' (1..7)\n", argv[2]);
        return -1;
    }
    if (w25q_parse_opts(argc, argv, 3, &o) != 0)
    {
        return -1;
    }
    if (o.cs == PIN_NONE && !w25q_parse_pin(g_rec_cs[spi_n], &o.cs))
    {
        rt_kprintf("[w25q] no default CS for spi%u, pass cs=PFx\n", (unsigned)spi_n);
        return -1;
    }
    if (!w25q_check_dma((int)spi_n, o.dma))
    {
        return -1;
    }

    res = w25q_attach(&b, (int)spi_n, o.cs);
    if (res != RT_EOK)
    {
        return -1;
    }

    struct rt_spi_configuration cfg = { 0 };
    cfg.data_width = 8u;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MSB | (rt_uint8_t)(o.mode & 0x3u);
    cfg.max_hz = o.hz_khz * 1000u;
    if (rt_spi_configure(b->dev, &cfg) != RT_EOK)
    {
        rt_kprintf("[probe] configure failed\n");
        return -1;
    }
    w25q_dma_set(b->dev, o.dma);

    spi = w25q_drv(b->dev)->config->SPIx;
    w25q_pin_str(o.cs, cs_str, sizeof(cs_str));

    rt_kprintf("[probe] spi%u cs=%s dma=%d mode=%u hz_act=%u base=%p\n",
               (unsigned)spi_n, cs_str, o.dma ? 1 : 0, (unsigned)(o.mode & 0x3u),
               (unsigned)(w25q_actual_hz(w25q_drv(b->dev)) / 1000u), (void *)spi);

    /* What turns the engine on: the core-assignment/clock gates and the
     * reset line, straight from RCC. */
    rt_kprintf("[probe] RCC APB1EN2=%08X APB2EN2=%08X APB5EN1=%08X "
               "APB5EN2=%08X APB5RST1=%08X\n",
               (unsigned)RCC->APB1EN2, (unsigned)RCC->APB2EN2,
               (unsigned)RCC->APB5EN1, (unsigned)RCC->APB5EN2,
               (unsigned)RCC->APB5RST1);

    /* What w25q_mux_master() actually landed on the pads. */
    w25q_probe_pin("SCK", w25q_sig_of((int)spi_n, 0)->port, w25q_sig_of((int)spi_n, 0)->pin);
    w25q_probe_pin("MISO", w25q_sig_of((int)spi_n, 1)->port, w25q_sig_of((int)spi_n, 1)->pin);
    w25q_probe_pin("MOSI", w25q_sig_of((int)spi_n, 2)->port, w25q_sig_of((int)spi_n, 2)->pin);
    w25q_probe_pin("CS", (char)('A' + (int)((o.cs >> 4) & 0xFu)),
                   (rt_uint8_t)(o.cs & 0xFu));

    if (spi_n == 7u)
    {
        /* Also show the mapping that was NOT muxed. Its pads still read as
         * Cube_Config left them, so a flash wired over there is obvious. */
        const struct w25q_sig *oth = g_sig_alt ? g_sig[7] : g_sig7_alt;

        rt_kprintf("[probe] pinset=%s\n", g_sig_alt ? "alt (PJ1/PJ3/PJ4)"
                                                    : "def (PI8/PI13/PI14)");
        w25q_probe_pin("oth", oth[0].port, oth[0].pin);
        w25q_probe_pin("oth", oth[1].port, oth[1].pin);
        w25q_probe_pin("oth", oth[2].port, oth[2].pin);
    }

    w25q_probe_dump_spi("pre ", spi);

    /* Alive test: CRCPOLY is a plain RW register nothing else touches here.
     * A gated peripheral reads back 0 no matter what was written. */
    saved = spi->CRCPOLY;
    spi->CRCPOLY = 0x00A5u;
    rb = spi->CRCPOLY;
    spi->CRCPOLY = saved;
    rt_kprintf("[probe] scratch write 0x00A5 -> read 0x%04X : engine %s "
               "(CRCPOLY was 0x%04X)\n",
               (unsigned)rb,
               (rb == 0x00A5u) ? "RESPONDS" : "DEAD", (unsigned)saved);

    res = rt_spi_send_then_recv(b->dev, &id_cmd, 1, id, 3);
    rt_kprintf("[probe] jedec xfer res=%d id=%02X %02X %02X\n",
               (int)res, id[0], id[1], id[2]);

    w25q_probe_dump_spi("post", spi);
    return 0;
}

/* ------------------------------------------------------------------ *
 *  msh entry
 * ------------------------------------------------------------------ */

static int w25q(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("w25q -- W25Q128 console test, one SPI bus at a time\n"
                   "  w25q list\n"
                   "  w25q id    <spi 1..7>        [cs=PFx] [hz=K] [dma=0|1] [mode=0..3] [eng=]\n"
                   "  w25q rw    <spi 1..7> <len>  [cs=] [hz=] [dma=] [mode=] [chunk=] [off=] [eng=] [noerase]\n"
                   "  w25q sweep <spi 1..7>        [cs=] [hz=] [dma=] [mode=] [chunk=] [off=] [eng=]\n"
                   "  w25q rate  <spi 1..7> <len>  [cs=] [dma=] [mode=] [chunk=] [off=] [eng=]\n"
                   "  w25q probe <spi 1..7>        [cs=] [hz=] [dma=] [mode=]   -- clock gate,\n"
                   "                               pin mux and SPI registers, no flash access\n"
                   "  All of them take [pins=def|alt], which picks which of SPI7's two AFIO\n"
                   "  mappings the flash is wired to; it does nothing on the other buses.\n"
                   "  hz is kHz, default 18750 (the top rate with margin); dma defaults to 1,\n"
                   "  chunk defaults to 4096. eng is raw (default) or sfud; run the same case\n"
                   "  under both to tell a driver fault from a flash-stack fault.\n"
                   "  Data from off=0 is destroyed -- it is a test part.\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "list") == 0)
    {
        w25q_list();
        return 0;
    }
    if (rt_strcmp(argv[1], "id") == 0)
    {
        return w25q_cmd_id(argc, argv);
    }
    if (rt_strcmp(argv[1], "rw") == 0)
    {
        return w25q_cmd_rw(argc, argv);
    }
    if (rt_strcmp(argv[1], "sweep") == 0)
    {
        return w25q_cmd_sweep(argc, argv);
    }
    if (rt_strcmp(argv[1], "rate") == 0)
    {
        return w25q_cmd_rate(argc, argv);
    }
    if (rt_strcmp(argv[1], "probe") == 0)
    {
        return w25q_cmd_probe(argc, argv);
    }

    rt_kprintf("[w25q] unknown subcommand '%s', try 'w25q'\n", argv[1]);
    return -1;
}
MSH_CMD_EXPORT(w25q, W25Q128 SPI flash test on any of spi1..spi7);

#endif /* BSP_USING_SPI && RT_USING_SPI */
