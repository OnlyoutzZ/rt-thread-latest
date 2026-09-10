/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2026-09-03     ox-horse        SPI1(master) <-> SPI2(slave) DMA pair test (N32H760)
 *
 * Modes covered (drv_spi.c data-direction handling):
 *   1. FULL DUPLEX (4-wire, spi_pair / spi_bat):
 *      both ends issue a simultaneous same-length full-duplex rt_spi_transfer
 *      (>=10B DMA path, <10B PIO path), byte-by-byte compare.
 *   2. HALF DUPLEX / 3-wire (spi_hdu): RT_SPI_3WIRE, per-message direction
 *      alternation -> stresses the driver's SINGLELINE_TX/RX re-init and the
 *      disable-SPI-on-message-release logic. Each round alternates
 *      A(tx)->B(rx) then B(tx)->A(rx).
 *   3. Clock mode 0..3 selectable (last parameter) on both tests.
 *
 * Usage:
 *   spi_pair <len> [aligned] [rounds] [speed_khz] [mode]
 *     - full duplex 4-wire pair. len 4B..32768B (default 8192: crosses the
 *       4095B DMA single-block limit -> chunking covered)
 *     - aligned: 0 = rt_malloc send buffer (8B aligned -> driver copies to a
 *       32B-aligned buffer, normal path)
 *                1 = __align(32) static array (driver DMA direct-use path;
 *       TX/RX buffers stay separate - both alignments are expected PASS)
 *     - rounds: 0 = infinite soak (default); failure auto-stops
 *     - mode: 0..3 = CPOL/CPHA clock mode (default 0)
 *   spi_bat [rounds_each] [speed_khz]
 *     - battery: len={9,4095,4096,8192} x aligned={0,1}, 8 cases; all
 *       expected PASS in DMA-on builds (len=9: master PIO + slave DMA)
 *   spi_hdu <len> [rounds] [speed_khz] [mode]
 *     - half-duplex 3-wire alternation test. The master send leg (AB) polls
 *       with PIO below 10 elements; the master recv leg (BA) goes through
 *       DMA at every length (driver rule since 2026-09-10 - the PIO poll
 *       raced the free-running RX clock). Both legs expected PASS.
 *   spi_rxonly <len> [rounds] [speed_khz]
 *     - master recv-only vs slave full-duplex. The driver clocks a
 *       deterministic 0xFF fill on MOSI (DMA leg: static fill buffer as TX
 *       source; PIO leg: constant writes), so len>=10 (DMA) and len<10 (PIO)
 *       are expected PASS in DMA-on builds. Only in a no-DMA build does the
 *       slave's own <10B full-duplex message hit the driver's master-only
 *       PIO gate (-EIO, slave PIO is unsupported) - expected FAIL there.
 *   spi_all [rounds_each] [speed_khz]
 *     - current-group sequential combination matrix: FD(4-wire) align 0/1 +
 *       clock mode 0..3 sweep (expected PASS), then HD(3-wire) PIO/DMA rows
 *       (observe only). The HD rows need the 3-wire single-data-line hookup
 *       (master MOSI <-> slave MISO); on a 4-wire hookup that net does not
 *       exist, so HD rows FAIL there and that is a wiring artifact, not a
 *       driver verdict - the 3-wire path is judged by spi_3wall on the
 *       dedicated single-line hookup. Note two former "known defect" classes
 *       (aligned=1 FD DMA copy-back, len<10 slave -EIO) are fixed in the
 *       current driver, so they are plain PASS rows now.
 *   spi_pair_stop
 *     - stop whichever test is running (current round finishes; worst-case
 *       driver internal timeout is 1000 ticks per 4095B chunk)
 *
 * Wiring, group g0 = SPI1 master <-> SPI2 slave (SPI1 pads PA5/PA7/PA6):
 *   FULL DUPLEX (4-wire; slave soft-NSS always selected, no CS/NSS line):
 *     SPI1.SCK  --> SPI2.SCK
 *     SPI1.MOSI --> SPI2.MOSI   (master data out -> slave data in)
 *     SPI1.MISO <-- SPI2.MISO   (slave data out -> master data in)
 *     GND common
 *   HALF DUPLEX (3-wire; N32H7 BIDIR data pin: master=MOSI, slave=MISO):
 *     SPI1.SCK  --> SPI2.SCK
 *     SPI1.MOSI --> SPI2.MISO   (the single shared data line)
 *     SPI1.MISO / SPI2.MOSI unused/floating
 *     GND common
 * Wiring, group g1 = SPI3 master <-> SPI4 slave; same-name topology on
 * SPI3 pads PB2(MOSI)/PB3(SCK)/PB4(MISO), SPI4 pads PG14(MOSI)/PG13(SCK)/
 * PG12(MISO):
 *   FULL DUPLEX: PB3<->PG13(SCK)  PB2<->PG14(MOSI)  PB4<->PG12(MISO)  GND
 *   HALF DUPLEX: PB3<->PG13(SCK)  PB2<->PG12 (master MOSI <-> slave MISO
 *                single data line), PB4/PG14 floating
 *   Master/slave is harness configuration only: same-name links work with
 *   either module driving SCK, rows always declare side A master, B slave.
 *   Rows compile in only for pairs whose buses are enabled (SPI1&&SPI2 or
 *   SPI3&&SPI4); the first present row becomes the default group, so a
 *   single-pair firmware needs no group selector on its commands.
 *   Pin mux is configured externally (Cube_Config), not by this file.
 *
 * Design notes:
 *   1. Slave thread priority (23) > master (22): after the slave releases
 *      the ready semaphore it must finish its DMA arming before the master
 *      starts clocking.
 *   2. Both ends rebuild peer data deterministically from (round, salt);
 *      no dependency on previously received content.
 *   3. RX buffers pre-filled with 0xA5; failure while still all 0xA5 prints
 *      "rx buffer untouched!" (signature of the aligned direct-path bug).
 *   4. The device objects survive across runs (attached once); only run
 *      parameters are reset between invocations.
 */

#include <rtthread.h>
#include <board.h>

#if defined(RT_USING_SPI) && \
    ((defined(BSP_USING_SPI1) && defined(BSP_USING_SPI2)) || \
     (defined(BSP_USING_SPI3) && defined(BSP_USING_SPI4)))

#include <drivers/dev_spi.h>

#define SPI_PAIR_MAX_LEN        32768   /* aligned static buffer limit (AHB SRAM) */
#define SPI_PAIR_LEN_MAX        16384   /* max length (4x malloc: heap bound) */
#define SPI_PAIR_RX_FILL        0xA5    /* rx buffer pre-fill byte */
#define SPI_PAIR_ROUND_WAIT_MS  6000    /* per-leg handshake timeout, 4-wire (ms) */
#define SPI_PAIR_ROUND_WAIT_3W_MS 10000 /* per-leg handshake timeout, 3-wire (ms):
                                           every message cold-restarts the engine
                                           (DeInit+Init direction flip + tail
                                           disable), so a leg may legitimately
                                           outrun the 4-wire budget */
#define SPI_PAIR_ARM_MARGIN_MS  8       /* margin after slave arm (ms): 从机需在 master
                                           起时钟前完成 DMA arm + SPI_Enable; CPHA=0(mode0/2)
                                           第一边沿采样更早, 2ms 边缘, 调大验证 */
#define SPI_PAIR_DEFAULT_KHZ    1000
#define SPI_PAIR_LEN_MIN        1
#define SPI_PAIR_SPEED_MIN_KHZ  100
#define SPI_PAIR_SPEED_MAX_KHZ  20000

/* device names are group-row data now (see spi_pair_groups below):
 * group 0 uses master "spi10" on "spi1" and slave "spi20" on "spi2";
 * group 1 uses master "spi30" on "spi3" and slave "spi40" on "spi4". */

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6010050)
#define SPI_PAIR_ALIGN32        __align(32)
#else
#define SPI_PAIR_ALIGN32        __attribute__((aligned(32)))
#endif

/* SPI register readback for diagnostics (bases from n32h7xx device header:
 * PERIPH_BASE=0x40000000, APB2=+0xD0000, SPI1=APB2+0xC000, SPI2=APB2+0xC400;
 * hardcoded like uart_test because device headers may not be visible here)
 * CTRL1@+0x00: MSEL=0x40 SSEL=0x800 SSMEN=0x1000 BIDIRMODE/BIDIROEN/RONLY...
 * CTRL2@+0x04: SPIEN=0x1 RDMAEN=0x2 TDMAEN=0x4 ERRINTEN=0x40
 * STS  @+0x08: TE=0x1 RNE=0x2 BUSY=0x4 OVER=0x20
 */
#define SPI_PAIR_SPI1_BASE      0x400DC000UL
#define SPI_PAIR_SPI2_BASE      0x400DC400UL
#define SPI_PAIR_SPI3_BASE      0x4000E400UL   /* SPI3: APB1 base +0xE400 */
#define SPI_PAIR_SPI4_BASE      0x58002000UL   /* SPI4: APB5 (0x58000000) +0x2000 */

/* DMA controller/channel register diagnostic (device header:
 * AHB1PERIPH_BASE=0x40040000, DMA1=+0x6800, DMA2=+0x6C00, DMA3=+0x7000;
 * channel block stride 0x58: SA@0x00 DA@0x08 CTRL@0x18(64b) CFG@0x40(64b);
 * controller: RAWTCINTSTS@0x2C0 TCINTMSK@0x310 RAWERRINTSTS@0x2E0 CHEN@0x3A0)
 * Involved channels of this test: spi1 TX=DMA1ch6 RX=DMA2ch5,
 *                                   spi2 TX=DMA1ch7 RX=DMA2ch6 */
#define SPI_PAIR_DMA1_BASE      0x40046800UL
#define SPI_PAIR_DMA2_BASE      0x40046C00UL
#define SPI_PAIR_DMA3_BASE      0x40047000UL   /* DMA3: AHB1PERIPH +0x7000 */
#define SPI_PAIR_DMA_CH_STEP    0x58UL

/* log mutex: two threads interleave rt_kprintf at char level otherwise.
 * Lazy init: every command path (incl. early error returns before the run
 * setup) may log, and rt_mutex_take on a zeroed mutex would corrupt lists. */
static struct rt_mutex spi_pair_print_mtx;
static rt_bool_t spi_pair_log_ready = RT_FALSE;
#define PAIR_LOG(...) \
    do { if (!spi_pair_log_ready) \
        { rt_mutex_init(&spi_pair_print_mtx, "splog", RT_IPC_FLAG_FIFO); \
          spi_pair_log_ready = RT_TRUE; } \
         rt_mutex_take(&spi_pair_print_mtx, RT_WAITING_FOREVER); \
         rt_kprintf(__VA_ARGS__); \
         rt_mutex_release(&spi_pair_print_mtx); } while (0)

#define SPI_PAIR_SALT_A         0xA1u    /* salt of A(master) TX data */
#define SPI_PAIR_SALT_B         0xB2u    /* salt of B(slave)  TX data */

/* ---------------- group table: which SPI master/slave pair is tested ---- */
/* One test body serves every "group" = one wired SPI pair on the bench.
 * A row carries everything that differs between pairs (attach names, reg
 * peek bases, DMA identities, device objects). Commands accept an optional
 * trailing group selector (bare "index" or "--g <name>"); without one the
 * default is the first row whose pair is enabled in this build (rows are
 * #if'd on the bus macros), so a single-pair firmware needs no selector
 * and a multi-pair firmware keeps the original spi1/2 pair first, so all
 * pre-existing invocations parse identically. Adding another wired pair =
 * appending one row; device names must stay unique system-wide. */
#define SPI_PAIR_DMA_SLOTS      4   /* [0]=A-tx [1]=A-rx [2]=B-tx [3]=B-rx */
#define SPI_PAIR_GRP_CAP_DM2    0x01u /* row may run spi_dm2 (pins+module bound) */

struct spi_pair_dma_slot
{
    rt_uint32_t ctrl_base;          /* DMA1/DMA2 controller base */
    rt_uint8_t  ch;                 /* channel 0..7 */
    const char *label;              /* print identity "spi1-TX DMA1ch6" */
};

struct spi_pair_group
{
    const char *name;               /* selector token, e.g. "g0" */
    const char *a_bus;              /* side A bus (master) */
    const char *a_dev;              /* side A device - unique per group */
    const char *b_bus;              /* side B bus (slave) */
    const char *b_dev;              /* side B device - unique per group */
    const char *a_tag;              /* "spi1(M)" print identity */
    const char *b_tag;              /* "spi2(S)" print identity */
    rt_uint32_t a_base;             /* SPI CTRL1 base, master side */
    rt_uint32_t b_base;             /* SPI CTRL1 base, slave side */
    struct spi_pair_dma_slot dma[SPI_PAIR_DMA_SLOTS];
    rt_uint32_t caps;               /* 0 = matrix commands only */
    struct rt_spi_device dev_a;     /* master device: attached once, persistent */
    struct rt_spi_device dev_b;     /* slave device: attached once, persistent */
    rt_bool_t attached;
};

static struct spi_pair_group spi_pair_groups[] =
{
#if defined(BSP_USING_SPI1) && defined(BSP_USING_SPI2)
    {
        .name = "g0",
        .a_bus = "spi1", .a_dev = "spi10",
        .b_bus = "spi2", .b_dev = "spi20",
        .a_tag = "spi1(M)", .b_tag = "spi2(S)",
        .a_base = SPI_PAIR_SPI1_BASE, .b_base = SPI_PAIR_SPI2_BASE,
        .dma = {
            { SPI_PAIR_DMA1_BASE, 6, "spi1-TX DMA1ch6" },
            { SPI_PAIR_DMA2_BASE, 5, "spi1-RX DMA2ch5" },
            { SPI_PAIR_DMA1_BASE, 7, "spi2-TX DMA1ch7" },
            { SPI_PAIR_DMA2_BASE, 6, "spi2-RX DMA2ch6" },
        },
        .caps = SPI_PAIR_GRP_CAP_DM2,
    },
#endif
#if defined(BSP_USING_SPI3) && defined(BSP_USING_SPI4)
    {
        .name = "g1",
        .a_bus = "spi3", .a_dev = "spi30",
        .b_bus = "spi4", .b_dev = "spi40",
        .a_tag = "spi3(M)", .b_tag = "spi4(S)",
        .a_base = SPI_PAIR_SPI3_BASE, .b_base = SPI_PAIR_SPI4_BASE,
        .dma = {
            { SPI_PAIR_DMA2_BASE, 0, "spi3-TX DMA2ch0" },
            { SPI_PAIR_DMA2_BASE, 7, "spi3-RX DMA2ch7" },
            { SPI_PAIR_DMA2_BASE, 1, "spi4-TX DMA2ch1" },
            { SPI_PAIR_DMA3_BASE, 0, "spi4-RX DMA3ch0" },
        },
        .caps = 0,
    },
#endif
};
#define SPI_PAIR_GROUP_NUM \
    (sizeof(spi_pair_groups) / sizeof(spi_pair_groups[0]))

/* Group under test. Static-initialized to g0 so any code path that runs
 * before a command-layer selector keeps today's exact behavior. */
static struct spi_pair_group *spi_pair_cur_g = &spi_pair_groups[0];

/* ------------------------- shared helpers ------------------------- */

static void spi_pair_gen(rt_uint8_t *buf, rt_uint32_t len,
                         rt_uint32_t round, rt_uint32_t salt, rt_uint32_t width)
{
    rt_uint32_t i;

    if (width == 16u)
    {
        for (i = 0; i < len; i++)
        {
            ((rt_uint16_t *)buf)[i] = (rt_uint16_t)((round * 157u + salt * 37u + i * 107u + 31u) & 0xffffu);
        }
        return;
    }
    for (i = 0; i < len; i++)
    {
        buf[i] = (rt_uint8_t)(((round & 0xffff) * 157u + salt * 37u + i * 107u + 31u) & 0xff);
    }
}

static rt_uint8_t spi_pair_gen_byte(rt_uint32_t i, rt_uint32_t round, rt_uint32_t salt)
{
    return (rt_uint8_t)(((round & 0xffff) * 157u + salt * 37u + i * 107u + 31u) & 0xff);
}

static rt_uint32_t spi_pair_atoi(const char *s)
{
    rt_uint32_t v = 0;

    while (*s >= '0' && *s <= '9')
    {
        v = v * 10 + (rt_uint32_t)(*s - '0');
        s++;
    }
    return v;
}

static rt_bool_t spi_pair_all_digits(const char *s)
{
    if (*s == '\0')
    {
        return RT_FALSE;
    }
    while (*s)
    {
        if (*s < '0' || *s > '9')
        {
            return RT_FALSE;
        }
        s++;
    }
    return RT_TRUE;
}

/* resolve a group selector token: all-digits -> array index, otherwise
 * row name; on failure print the available groups and return RT_NULL */
static struct spi_pair_group *spi_pair_group_find(const char *sel)
{
    rt_uint32_t i;

    if (spi_pair_all_digits(sel))
    {
        i = spi_pair_atoi(sel);
        if (i < SPI_PAIR_GROUP_NUM)
        {
            return &spi_pair_groups[i];
        }
    }
    else
    {
        for (i = 0; i < SPI_PAIR_GROUP_NUM; i++)
        {
            if (rt_strcmp(spi_pair_groups[i].name, sel) == 0)
            {
                return &spi_pair_groups[i];
            }
        }
    }
    PAIR_LOG("[spi] unknown group '%s', available:", sel);
    for (i = 0; i < SPI_PAIR_GROUP_NUM; i++)
    {
        PAIR_LOG(" %s", spi_pair_groups[i].name);
    }
    PAIR_LOG("\n");
    return RT_NULL;
}

/* parse the optional group token that follows a command's positional
 * arguments: nothing -> keep current group; "<token>" or "--g <token>".
 * Returns RT_NULL (after an error line) when the selector is malformed. */
static struct spi_pair_group *spi_pair_group_sel(int argc, char *argv[], int npos)
{
    int extra = argc - 1 - npos;
    const char *sel;

    if (extra <= 0)
    {
        return spi_pair_cur_g;
    }
    if (extra == 1)
    {
        sel = argv[npos + 1];
    }
    else if (extra == 2 && rt_strcmp(argv[npos + 1], "--g") == 0)
    {
        sel = argv[npos + 2];
    }
    else
    {
        PAIR_LOG("[spi] bad group selector: expected <gid> or --g <name>\n");
        return RT_NULL;
    }
    return spi_pair_group_find(sel);
}

/* resolve optional trailing group token ([<gid>|<name>] or --g <gid|name>)
 * after npos positional args; on success commit it as the current group */
static int spi_pair_group_apply(int argc, char *argv[], int npos)
{
    struct spi_pair_group *g = spi_pair_group_sel(argc, argv, npos);

    if (g == RT_NULL)
    {
        return -RT_ERROR;
    }
    spi_pair_cur_g = g;
    return RT_EOK;
}

static rt_uint32_t spi_pair_tick2ms(rt_uint32_t ticks)
{
    return (ticks * 1000u) / RT_TICK_PER_SECOND;
}

static rt_uint32_t spi_pair_reg(rt_bool_t is_master, rt_uint32_t offset)
{
    rt_uint32_t base = is_master ? spi_pair_cur_g->a_base
                                 : spi_pair_cur_g->b_base;

    return (rt_uint32_t)(*(volatile rt_uint16_t *)(base + offset));
}

static void spi_pair_regw(rt_bool_t is_master, rt_uint32_t offset, rt_uint16_t val)
{
    rt_uint32_t base = is_master ? spi_pair_cur_g->a_base
                                 : spi_pair_cur_g->b_base;

    *(volatile rt_uint16_t *)(base + offset) = val;
}

static void spi_pair_dma_ch_dump(rt_uint32_t ctrl_base, rt_uint32_t ch,
                                 const char *name)
{
    rt_uint32_t cb = ctrl_base + ch * SPI_PAIR_DMA_CH_STEP;
    rt_uint32_t sa = *(volatile rt_uint32_t *)(cb + 0x00);
    rt_uint32_t da = *(volatile rt_uint32_t *)(cb + 0x08);
    rt_uint32_t ctl = *(volatile rt_uint32_t *)(cb + 0x18);
    rt_uint32_t cfgl = *(volatile rt_uint32_t *)(cb + 0x40);
    rt_uint32_t cfgh = *(volatile rt_uint32_t *)(cb + 0x44);
    rt_uint32_t chen = *(volatile rt_uint32_t *)(ctrl_base + 0x3A0);
    rt_uint32_t rawtc = *(volatile rt_uint32_t *)(ctrl_base + 0x2C0);
    rt_uint32_t tmask = *(volatile rt_uint32_t *)(ctrl_base + 0x310);
    rt_uint32_t rawerr = *(volatile rt_uint32_t *)(ctrl_base + 0x2E0);

    PAIR_LOG("[%s] DMA %s: SA=0x%08x DA=0x%08x CTRL=0x%08x CFGl=0x%08x CFGh=0x%08x"
             " CHEN=%u RAWTC=%u TCMSK=%u RAWERR=%u\n",
             name,
             (unsigned)sa, (unsigned)da,
             (unsigned)ctl, (unsigned)cfgl, (unsigned)cfgh,
             (unsigned)((chen >> ch) & 1u), (unsigned)((rawtc >> ch) & 1u),
             (unsigned)((tmask >> ch) & 1u), (unsigned)((rawerr >> ch) & 1u));
}

/* dump the four DMA channels of the group under test ([0]A-tx [1]A-rx
 * [2]B-tx [3]B-rx; group 0 prints the historical spi1/spi2 order) */
static void spi_pair_dma_dump_all(const char *tag)
{
    struct spi_pair_group *g = spi_pair_cur_g;
    rt_uint32_t i;

    PAIR_LOG("[%s] DMA channel states:\n", tag);
    for (i = 0; i < SPI_PAIR_DMA_SLOTS; i++)
    {
        spi_pair_dma_ch_dump(g->dma[i].ctrl_base, g->dma[i].ch,
                             g->dma[i].label);
    }
}

static void spi_pair_reg_dump(const char *tag, rt_bool_t is_master)
{
    rt_uint32_t c1 = spi_pair_reg(is_master, 0x00);
    rt_uint32_t c2 = spi_pair_reg(is_master, 0x04);
    rt_uint32_t st = spi_pair_reg(is_master, 0x08);

    PAIR_LOG("[%s] %s regs: CTRL1=0x%04x CTRL2=0x%04x STS=0x%04x"
             " (SPIEN=%u TDMAEN=%u RDMAEN=%u ERRINTEN=%u BUSY=%u OVER=%u RNE=%u)\n",
             tag, is_master ? spi_pair_cur_g->a_tag : spi_pair_cur_g->b_tag,
             c1, c2, st,
             (unsigned)((c2 >> 0) & 1u), (unsigned)((c2 >> 2) & 1u), (unsigned)((c2 >> 1) & 1u),
             (unsigned)((c2 >> 6) & 1u), (unsigned)((st >> 2) & 1u), (unsigned)((st >> 5) & 1u),
             (unsigned)((st >> 1) & 1u));

    if (rt_strcmp(tag, "open") != 0)
    {
        spi_pair_dma_dump_all(tag);
    }
}

static rt_bool_t spi_pair_is_all(const rt_uint8_t *buf, rt_uint32_t len, rt_uint8_t v)
{
    rt_uint32_t i;

    for (i = 0; i < len; i++)
    {
        if (buf[i] != v)
        {
            return RT_FALSE;
        }
    }
    return RT_TRUE;
}

static rt_uint32_t spi_pair_cmp_gen(const rt_uint8_t *buf, rt_uint32_t len,
                                    rt_uint32_t round, rt_uint32_t salt,
                                    rt_uint32_t width)
{
    rt_uint32_t i;

    if (width == 16u)
    {
        for (i = 0; i < len; i++)
        {
            if (((const rt_uint16_t *)buf)[i] !=
                (rt_uint16_t)((round * 157u + salt * 37u + i * 107u + 31u) & 0xffffu))
            {
                return i;
            }
        }
        return len;
    }
    for (i = 0; i < len; i++)
    {
        if (buf[i] != spi_pair_gen_byte(i, round, salt))
        {
            return i;
        }
    }
    return len;
}

/* ------------------------- context ------------------------- */

struct spi_pair_ctx
{
    const char *tag;            /* "PAIR" / "HDU" for logs */
    rt_uint32_t len;
    rt_bool_t   aligned;
    rt_bool_t   three_wire;     /* half-duplex 3-wire test */
    rt_uint32_t round_wait_ms;  /* per-leg handshake timeout (wire-type driven) */
    rt_uint32_t rounds;         /* 0 = infinite soak */
    rt_uint32_t speed_hz;
    rt_uint32_t spi_mode;       /* 0..3 = CPOL/CPHA */
    rt_int32_t  spi_mode_b;     /* slave mode override, -1 = follow spi_mode */
    rt_bool_t   rxonly;         /* master recv-only special mode */
    rt_uint32_t xdir;           /* 1=SO master tx-only/slave rx-only,
                                   2=RO master rx-only/slave tx-only (0=bidir) */
    rt_uint32_t legs;           /* HDU: 1=AB only, 2=BA only, 3=both */
    rt_uint32_t salt_a;         /* master TX salt (default SPI_PAIR_SALT_A) */
    rt_uint32_t salt_b;         /* slave  TX salt (default SPI_PAIR_SALT_B) */
    rt_uint32_t data_width;     /* 8 or 16 (default 8) */
    rt_bool_t   lsb;            /* LSB-first (default MSB) */

    struct spi_pair_group *g;   /* group under test; synced from spi_pair_cur_g
                                   by setup() every run (reset does not clear) */

    rt_uint8_t *tx_a;
    rt_uint8_t *rx_a;
    rt_uint8_t *tx_b;
    rt_uint8_t *rx_b;
    rt_bool_t   bufs_heap;

    rt_sem_t ready_b;           /* slave armed signal (per leg) */
    rt_sem_t done_a;
    rt_sem_t done_b;
    rt_sem_t stopped;

    rt_uint32_t ok_rounds;
    rt_uint32_t min_ms;
    rt_uint32_t max_ms;
    rt_bool_t   stop;
    rt_bool_t   failed;
};

static struct spi_pair_ctx spi_pair_ctx;
static rt_bool_t spi_pair_running = RT_FALSE;
static struct rt_semaphore spi_pair_stopped_obj;
static rt_bool_t spi_pair_stopped_init = RT_FALSE;

/* 32B-aligned static buffers (aligned=1 test path; AC5 __align cannot be
 * applied to struct members, hence file scope) */
SPI_PAIR_ALIGN32 static rt_uint8_t spi_al_a_tx[SPI_PAIR_MAX_LEN];
SPI_PAIR_ALIGN32 static rt_uint8_t spi_al_a_rx[SPI_PAIR_MAX_LEN];
SPI_PAIR_ALIGN32 static rt_uint8_t spi_al_b_tx[SPI_PAIR_MAX_LEN];
SPI_PAIR_ALIGN32 static rt_uint8_t spi_al_b_rx[SPI_PAIR_MAX_LEN];

/* ------------------------- attach / configure ------------------------- */

static rt_err_t spi_pair_attach(struct spi_pair_ctx *ctx)
{
    struct spi_pair_group *g = ctx->g;

    if (g->attached)
    {
        return RT_EOK;
    }
    /* cs_pin = RT_NULL: driver spixfer does not touch CS. Slave soft-NSS
     * (SSMEN=1,SSEL=0) always selected, master soft-NSS no MODF: no CS/NSS
     * wire needed. Attach happens once per group row; device objects live
     * in the row and survive every run and every group switch. */
    if (rt_spi_bus_attach_device(&g->dev_a, g->a_dev, g->a_bus, RT_NULL) != RT_EOK)
    {
        PAIR_LOG("[spi] attach %s to %s failed!\n", g->a_dev, g->a_bus);
        return -RT_ERROR;
    }
    if (rt_spi_bus_attach_device(&g->dev_b, g->b_dev, g->b_bus, RT_NULL) != RT_EOK)
    {
        PAIR_LOG("[spi] attach %s to %s failed!\n", g->b_dev, g->b_bus);
        return -RT_ERROR;
    }
    g->attached = RT_TRUE;
    return RT_EOK;
}

static rt_err_t spi_pair_config_dev(struct rt_spi_device *dev, rt_bool_t slave,
                                    struct spi_pair_ctx *ctx)
{
    struct rt_spi_configuration cfg;

    rt_uint32_t dev_mode = (slave && ctx->spi_mode_b >= 0) ?
                           (rt_uint32_t)ctx->spi_mode_b : ctx->spi_mode;
    cfg.data_width = (rt_uint8_t)ctx->data_width;
    cfg.mode = (slave ? RT_SPI_SLAVE : RT_SPI_MASTER) | RT_SPI_NO_CS;
    cfg.mode |= (ctx->lsb) ? RT_SPI_LSB : RT_SPI_MSB;
    if (dev_mode & 0x1)
    {
        cfg.mode |= RT_SPI_CPHA;
    }
    if (dev_mode & 0x2)
    {
        cfg.mode |= RT_SPI_CPOL;
    }
    if (ctx->three_wire)
    {
        cfg.mode |= RT_SPI_3WIRE;   /* half duplex: SI/SO shared */
    }
    cfg.max_hz = ctx->speed_hz;
    return rt_spi_configure(dev, &cfg);
}

static rt_err_t spi_pair_config(struct spi_pair_ctx *ctx)
{
    if (spi_pair_config_dev(&ctx->g->dev_a, RT_FALSE, ctx) != RT_EOK)
    {
        PAIR_LOG("[spi] %s configure failed!\n", ctx->g->a_bus);
        return -RT_ERROR;
    }
    if (spi_pair_config_dev(&ctx->g->dev_b, RT_TRUE, ctx) != RT_EOK)
    {
        PAIR_LOG("[spi] %s configure failed!\n", ctx->g->b_bus);
        return -RT_ERROR;
    }
    return RT_EOK;
}

/* reset run parameters only; ctx->g (group) and row device objects survive */
static void spi_pair_ctx_reset(struct spi_pair_ctx *ctx)
{
    ctx->len = 0;
    ctx->aligned = RT_FALSE;
    ctx->three_wire = RT_FALSE;
    ctx->rounds = 0;
    ctx->speed_hz = 0;
    ctx->spi_mode = 0;
    ctx->spi_mode_b = -1;
    ctx->rxonly = RT_FALSE;
    ctx->xdir = 0;
    ctx->legs = 3;
    ctx->salt_a = SPI_PAIR_SALT_A;
    ctx->salt_b = SPI_PAIR_SALT_B;
    ctx->data_width = 8;
    ctx->lsb = RT_FALSE;
    ctx->ok_rounds = 0;
    ctx->min_ms = 0;
    ctx->max_ms = 0;
    ctx->stop = RT_FALSE;
    ctx->failed = RT_FALSE;
}

static rt_err_t spi_pair_buf_prepare(struct spi_pair_ctx *ctx)
{
    rt_uint32_t len = ctx->len;
    rt_uint32_t i;

    /* free heap buffers of the previous run only (static arrays are owned) */
    if (ctx->bufs_heap)
    {
        if (ctx->tx_a) { rt_free(ctx->tx_a); ctx->tx_a = RT_NULL; }
        if (ctx->rx_a) { rt_free(ctx->rx_a); ctx->rx_a = RT_NULL; }
        if (ctx->tx_b) { rt_free(ctx->tx_b); ctx->tx_b = RT_NULL; }
        if (ctx->rx_b) { rt_free(ctx->rx_b); ctx->rx_b = RT_NULL; }
    }

    if (ctx->aligned)
    {
        ctx->bufs_heap = RT_FALSE;
        ctx->tx_a = spi_al_a_tx;
        ctx->rx_a = spi_al_a_rx;
        ctx->tx_b = spi_al_b_tx;
        ctx->rx_b = spi_al_b_rx;
    }
    else
    {
        ctx->bufs_heap = RT_TRUE;
        ctx->tx_a = (rt_uint8_t *)rt_malloc(len * (ctx->data_width / 8u));
        ctx->rx_a = (rt_uint8_t *)rt_malloc(len * (ctx->data_width / 8u));
        ctx->tx_b = (rt_uint8_t *)rt_malloc(len * (ctx->data_width / 8u));
        ctx->rx_b = (rt_uint8_t *)rt_malloc(len * (ctx->data_width / 8u));
        if (ctx->tx_a == RT_NULL || ctx->rx_a == RT_NULL ||
            ctx->tx_b == RT_NULL || ctx->rx_b == RT_NULL)
        {
            PAIR_LOG("[spi] malloc %uB buffers failed!\n", len);
            return -RT_ERROR;
        }
    }

    for (i = 0; i < len * (ctx->data_width / 8u); i++)
    {
        ctx->rx_a[i] = SPI_PAIR_RX_FILL;
        ctx->rx_b[i] = SPI_PAIR_RX_FILL;
    }
    return RT_EOK;
}

/* ------------------------- teardown / controller ------------------------- */

static rt_uint32_t spi_pair_chunk_worst_s(struct spi_pair_ctx *ctx)
{
    return ((ctx->len / 4095u) + 1u) * 4u + 5u;
}

static rt_bool_t spi_pair_wait_done_b(struct spi_pair_ctx *ctx)
{
    rt_uint32_t worst_s = spi_pair_chunk_worst_s(ctx);

    return (rt_sem_take(ctx->done_b, worst_s * RT_TICK_PER_SECOND) == RT_EOK);
}

static void spi_pair_finish_case(struct spi_pair_ctx *ctx, rt_bool_t b_done)
{
    rt_err_t res = ctx->failed ? -RT_ERROR : RT_EOK;

    PAIR_LOG("[spi] case done: %s len=%u mode=%u align=%s ok=%u min=%ums max=%ums => %s\n",
             ctx->tag, ctx->len, ctx->spi_mode, ctx->aligned ? "1" : "0",
             ctx->ok_rounds, ctx->min_ms, ctx->max_ms, res == RT_EOK ? "PASS" : "FAIL");
    spi_pair_reg_dump("done", RT_TRUE);
    spi_pair_reg_dump("done", RT_FALSE);

    rt_sem_release(ctx->stopped);   /* wake spi_pair_stop if waiting */

    if (!b_done)
    {
        /* slave still alive: must not delete sems or threads (use-after-free) */
        PAIR_LOG("[spi] slave thread did not exit within %us (stuck in driver?),"
                 " skip cleanup\n", spi_pair_chunk_worst_s(ctx));
        spi_pair_running = RT_FALSE;
        return;
    }

    /* NOTE: dynamic threads are auto-reaped by the idle thread
     * (rt_defunct_execute) once they exit - never rt_thread_delete() them */
    rt_sem_delete(ctx->ready_b);
    rt_sem_delete(ctx->done_a);
    rt_sem_delete(ctx->done_b);
    ctx->ready_b = RT_NULL;
    ctx->done_a = RT_NULL;
    ctx->done_b = RT_NULL;
    spi_pair_running = RT_FALSE;
}

/* soak (rounds=0) controller: cmd thread stays free, waits and tears down */
static void spi_pair_controller_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_bool_t b_done;

    rt_sem_take(ctx->done_a, RT_WAITING_FOREVER);
    b_done = spi_pair_wait_done_b(ctx);
    spi_pair_finish_case(ctx, b_done);
}

/* spawn a soak-mode controller or block synchronously for finite rounds;
 * returns after threads have started, leaving teardown to the caller path */
static rt_err_t spi_pair_wait_finish(struct spi_pair_ctx *ctx, rt_bool_t soak,
                                     rt_uint32_t rounds)
{
    rt_uint32_t wait_s;

    if (soak)
    {
        rt_thread_t tid_c;

        tid_c = rt_thread_create("spi_ctrl", spi_pair_controller_entry, ctx, 2048, 21, 10);
        if (tid_c == RT_NULL)
        {
            PAIR_LOG("[spi] create controller thread failed!\n");
            ctx->stop = RT_TRUE;
            if (ctx->ready_b)
            {
                rt_sem_release(ctx->ready_b);
            }
            return -RT_ERROR;
        }
        rt_thread_startup(tid_c);
        return RT_EOK;  /* result printed asynchronously by controller */
    }

    wait_s = (rounds + 2) * (ctx->round_wait_ms / 1000u) + 2 + spi_pair_chunk_worst_s(ctx);
    rt_sem_take(ctx->done_a, wait_s * RT_TICK_PER_SECOND);
    spi_pair_finish_case(ctx, spi_pair_wait_done_b(ctx));
    return ctx->failed ? -RT_ERROR : RT_EOK;
}

/* validate common parameters and reset context; returns RT_EOK when ready */
static rt_err_t spi_pair_setup(struct spi_pair_ctx *ctx, const char *tag,
                               rt_uint32_t len, rt_bool_t aligned,
                               rt_uint32_t rounds, rt_uint32_t speed_khz,
                               rt_uint32_t mode, rt_bool_t three_wire,
                               rt_bool_t rxonly, rt_uint32_t data_width,
                               rt_bool_t lsb)
{
    if (spi_pair_running)
    {
        PAIR_LOG("spi test already running, use spi_pair_stop first!\n");
        return -RT_ERROR;
    }
    if (len < SPI_PAIR_LEN_MIN || (data_width != 8u && data_width != 16u))
    {
        PAIR_LOG("len must be >=%u, data_width 8/16\n", SPI_PAIR_LEN_MIN);
        return -RT_ERROR;
    }
    if (len * (data_width / 8u) > (aligned ? SPI_PAIR_MAX_LEN : SPI_PAIR_LEN_MAX))
    {
        PAIR_LOG("len %u x %u-bit exceeds %u-byte budget\n",
                 len, data_width, aligned ? SPI_PAIR_MAX_LEN : SPI_PAIR_LEN_MAX);
        return -RT_ERROR;
    }    if (speed_khz < SPI_PAIR_SPEED_MIN_KHZ || speed_khz > SPI_PAIR_SPEED_MAX_KHZ)
    {
        PAIR_LOG("speed_khz must be %u..%u\n", SPI_PAIR_SPEED_MIN_KHZ, SPI_PAIR_SPEED_MAX_KHZ);
        return -RT_ERROR;
    }
    if (mode > 3)
    {
        PAIR_LOG("mode must be 0..3 (CPOL/CPHA)\n");
        return -RT_ERROR;
    }

    spi_pair_ctx_reset(ctx);
    ctx->g = spi_pair_cur_g;    /* group selection is command-layer state */
    ctx->tag = tag;
    ctx->len = len;
    ctx->data_width = data_width;
    ctx->lsb = lsb;
    ctx->aligned = aligned;
    ctx->rounds = rounds;
    ctx->speed_hz = speed_khz * 1000u;
    ctx->spi_mode = mode;
    ctx->three_wire = three_wire;
    ctx->round_wait_ms = three_wire ? SPI_PAIR_ROUND_WAIT_3W_MS
                                    : SPI_PAIR_ROUND_WAIT_MS;
    ctx->rxonly = rxonly;

    if (spi_pair_attach(ctx) != RT_EOK ||
        spi_pair_config(ctx) != RT_EOK ||
        spi_pair_buf_prepare(ctx) != RT_EOK)
    {
        return -RT_ERROR;
    }

    ctx->ready_b = rt_sem_create("rdy", 0, RT_IPC_FLAG_FIFO);
    ctx->done_a = rt_sem_create("dn_a", 0, RT_IPC_FLAG_FIFO);
    ctx->done_b = rt_sem_create("dn_b", 0, RT_IPC_FLAG_FIFO);
    /* stopped sem is a static object: init exactly ONCE. Re-initting in a
     * later case would find itself on the object list -> rt_object_init
     * asserts (obj != object). Leftover counts from finished cases are
     * drained below so a later spi_pair_stop cannot consume stale credits. */
    if (!spi_pair_stopped_init)
    {
        rt_sem_init(&spi_pair_stopped_obj, "stpd", 0, RT_IPC_FLAG_FIFO);
        spi_pair_stopped_init = RT_TRUE;
    }
    while (rt_sem_take(&spi_pair_stopped_obj, 0) == RT_EOK)
    {
        /* drain prior finish_case releases */
    }
    ctx->stopped = &spi_pair_stopped_obj;
    if (ctx->ready_b == RT_NULL || ctx->done_a == RT_NULL || ctx->done_b == RT_NULL)
    {
        PAIR_LOG("[spi] create semaphore failed!\n");
        return -RT_ERROR;
    }
    return RT_EOK;
}

/* slave (prio 23) outranks master (prio 22): slave arms its DMA first */
static rt_err_t spi_pair_spawn(struct spi_pair_ctx *ctx,
                               void (*slave_entry)(void *),
                               void (*master_entry)(void *))
{
    rt_thread_t tid_b, tid_a;

    tid_b = rt_thread_create("spi_slv", slave_entry, ctx, 2048, 23, 10);
    if (tid_b == RT_NULL)
    {
        PAIR_LOG("[spi] create slave thread failed!\n");
        return -RT_ERROR;
    }
    rt_thread_startup(tid_b);

    tid_a = rt_thread_create("spi_mst", master_entry, ctx, 2048, 22, 10);
    if (tid_a == RT_NULL)
    {
        PAIR_LOG("[spi] create master thread failed!\n");
        ctx->stop = RT_TRUE;
        rt_sem_take(ctx->done_b, RT_WAITING_FOREVER);
        return -RT_ERROR;
    }
    spi_pair_running = RT_TRUE;     /* stop command valid from here on */
    rt_thread_startup(tid_a);
    return RT_EOK;
}

/* ------------------------- FULL DUPLEX: master thread ------------------------- */

static void spi_pair_master_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_TRUE);

    while (!ctx->stop)
    {
        rt_uint32_t t0, ms;
        rt_ssize_t n;
        rt_uint32_t diff;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }

        if (rt_sem_take(ctx->ready_b, RT_TICK_PER_SECOND * (ctx->round_wait_ms / 1000u)) != RT_EOK)
        {
            if (ctx->stop)
            {
                break;  /* user stop: clean exit */
            }
            PAIR_LOG("[spi] master: slave-ready timeout!\n");
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
        if (ctx->stop)
        {
            break;
        }
        if (SPI_PAIR_ARM_MARGIN_MS > 0)
        {
            rt_thread_mdelay(SPI_PAIR_ARM_MARGIN_MS);
        }

        if (!ctx->rxonly)
        {
            spi_pair_gen(ctx->tx_a, ctx->len, r, ctx->salt_a, ctx->data_width);
        }

        t0 = rt_tick_get();
        if (ctx->rxonly)
        {
            n = rt_spi_transfer(&ctx->g->dev_a, RT_NULL, ctx->rx_a, ctx->len);
        }
        else
        {
            n = rt_spi_transfer(&ctx->g->dev_a, ctx->tx_a, ctx->rx_a, ctx->len);
        }
        ms = spi_pair_tick2ms(rt_tick_get() - t0);

        if (n != (rt_ssize_t)ctx->len)
        {
            PAIR_LOG("[spi] master transfer FAIL: ret=%d len=%u (%ums)\n",
                     (int)n, ctx->len, ms);
            spi_pair_reg_dump("fail", RT_TRUE);
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
        if (ctx->ok_rounds == 0)
        {
            ctx->min_ms = ms;
            ctx->max_ms = ms;
        }
        else
        {
            if (ms < ctx->min_ms) ctx->min_ms = ms;
            if (ms > ctx->max_ms) ctx->max_ms = ms;
        }

        /* wait until slave finished this round (its rx fully in RAM) */
        if (rt_sem_take(ctx->ready_b, RT_TICK_PER_SECOND * (ctx->round_wait_ms / 1000u)) != RT_EOK)
        {
            if (ctx->stop)
            {
                break;
            }
            PAIR_LOG("[spi] master: slave round-done timeout!\n");
            spi_pair_reg_dump("fail", RT_FALSE);
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
        if (ctx->stop)
        {
            break;
        }

        /* master RX side: expect slave content (saltB) */
        diff = spi_pair_cmp_gen(ctx->rx_a, ctx->len, r, ctx->salt_b, ctx->data_width);
        if (diff != ctx->len)
        {
            PAIR_LOG("[spi] MASTER rx FAIL r%u: diff@%u rx[0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                     r, diff, ctx->rx_a[0], ctx->rx_a[1], ctx->rx_a[2], ctx->rx_a[3],
                     ctx->rx_a[4], ctx->rx_a[5], ctx->rx_a[6], ctx->rx_a[7],
                     ctx->rx_a[8], ctx->rx_a[9], ctx->rx_a[10], ctx->rx_a[11],
                     ctx->rx_a[12], ctx->rx_a[13], ctx->rx_a[14], ctx->rx_a[15],
                     spi_pair_is_all(ctx->rx_a, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                     " [rx buffer untouched!]" : "");
            PAIR_LOG("[spi] MASTER rx[16..47]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                     ctx->rx_a[16], ctx->rx_a[17], ctx->rx_a[18], ctx->rx_a[19],
                     ctx->rx_a[20], ctx->rx_a[21], ctx->rx_a[22], ctx->rx_a[23],
                     ctx->rx_a[24], ctx->rx_a[25], ctx->rx_a[26], ctx->rx_a[27],
                     ctx->rx_a[28], ctx->rx_a[29], ctx->rx_a[30], ctx->rx_a[31],
                     ctx->rx_a[32], ctx->rx_a[33], ctx->rx_a[34], ctx->rx_a[35],
                     ctx->rx_a[36], ctx->rx_a[37], ctx->rx_a[38], ctx->rx_a[39],
                     ctx->rx_a[40], ctx->rx_a[41], ctx->rx_a[42], ctx->rx_a[43],
                     ctx->rx_a[44], ctx->rx_a[45], ctx->rx_a[46], ctx->rx_a[47]);
            {
                rt_uint8_t exp[16], sbuf[16];
                rt_uint32_t k;
                for (k = 0; k < 16; k++)
                {
                    exp[k] = spi_pair_gen_byte(k, r, ctx->salt_b);
                    sbuf[k] = ctx->tx_b[k];
                }
                PAIR_LOG("[spi] expect(saltB r%u)[0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                         r, exp[0], exp[1], exp[2], exp[3], exp[4], exp[5], exp[6], exp[7],
                         exp[8], exp[9], exp[10], exp[11], exp[12], exp[13], exp[14], exp[15]);
                PAIR_LOG("[spi] slave tx_b [0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                         sbuf[0], sbuf[1], sbuf[2], sbuf[3], sbuf[4], sbuf[5], sbuf[6], sbuf[7],
                         sbuf[8], sbuf[9], sbuf[10], sbuf[11], sbuf[12], sbuf[13], sbuf[14], sbuf[15]);
            }
            spi_pair_reg_dump("fail", RT_TRUE);
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }

        /* slave RX side: expect master content (saltA). rxonly mode master
         * does not send saltA (0xFF expected), slave thread checks its own */
        if (!ctx->rxonly)
        {
            diff = spi_pair_cmp_gen(ctx->rx_b, ctx->len, r, ctx->salt_a, ctx->data_width);
            if (diff != ctx->len)
            {
                PAIR_LOG("[spi] SLAVE  rx FAIL r%u: diff@%u rx[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                         r, diff, ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                         ctx->rx_b[4], ctx->rx_b[5], ctx->rx_b[6], ctx->rx_b[7],
                         spi_pair_is_all(ctx->rx_b, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                         " [rx buffer untouched!]" : "");
                spi_pair_reg_dump("fail", RT_FALSE);
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
        }

        /* aligned direct path: check TX buffer got RX-echo-mutated */
        if (ctx->aligned &&
            (spi_pair_cmp_gen(ctx->tx_a, ctx->len, r, ctx->salt_a, ctx->data_width) != ctx->len ||
             spi_pair_cmp_gen(ctx->tx_b, ctx->len, r, ctx->salt_b, ctx->data_width) != ctx->len))
        {
            PAIR_LOG("[spi] tx buf mutated by RX echo (driver shares TX/RX DMA buffer)\n");
        }

        ctx->ok_rounds++;
        if (ctx->rounds == 0 && ctx->ok_rounds % 100 == 0)
        {
            PAIR_LOG("[spi] r%u ok (%ums)\n", r, ms);
        }
        r++;
    }

    rt_sem_release(ctx->done_a);
}

/* ------------------------- FULL DUPLEX: slave thread ------------------------- */

static void spi_pair_slave_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_FALSE);

    while (!ctx->stop)
    {
        rt_uint32_t t0;
        rt_ssize_t n;
        rt_bool_t round_ok = RT_TRUE;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }

        spi_pair_gen(ctx->tx_b, ctx->len, r, ctx->salt_b, ctx->data_width);

        /* release ready then enter transfer: this thread (prio 23) outranks
         * the master (22), so it finishes DMA arming and blocks on completion
         * before the master is scheduled to start clocking */
        rt_sem_release(ctx->ready_b);
        t0 = rt_tick_get();
        n = rt_spi_transfer(&ctx->g->dev_b, ctx->tx_b, ctx->rx_b, ctx->len);

        if (n != (rt_ssize_t)ctx->len)
        {
            PAIR_LOG("[spi] slave transfer FAIL: ret=%d len=%u (%ums) rx_b[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                     (int)n, ctx->len, spi_pair_tick2ms(rt_tick_get() - t0),
                     ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                     ctx->rx_b[4], ctx->rx_b[5], ctx->rx_b[6], ctx->rx_b[7],
                     spi_pair_is_all(ctx->rx_b, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                     " [rx buffer untouched!]" : "");
            spi_pair_reg_dump("fail", RT_FALSE);
            round_ok = RT_FALSE;
        }
        else if (!ctx->rxonly)
        {
            /* slave-side self-check (runs even when the master bails on its own
             * compare): what the wire delivered to the slave vs master saltA.
             * Diagnostic for round-N frame-grid skew (CPOL=1 phantom edges). */
            rt_uint32_t diff = spi_pair_cmp_gen(ctx->rx_b, ctx->len, r, ctx->salt_a, ctx->data_width);
            if (diff != ctx->len)
            {
                PAIR_LOG("[spi] SLAVE self rx FAIL r%u: diff@%u rx[0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                         r, diff, ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                         ctx->rx_b[4], ctx->rx_b[5], ctx->rx_b[6], ctx->rx_b[7],
                         ctx->rx_b[8], ctx->rx_b[9], ctx->rx_b[10], ctx->rx_b[11],
                         ctx->rx_b[12], ctx->rx_b[13], ctx->rx_b[14], ctx->rx_b[15]);
                spi_pair_reg_dump("slv", RT_FALSE);
                round_ok = RT_FALSE;
            }
            else if (ctx->rounds > 1)
            {
                PAIR_LOG("[spi] SLAVE self rx OK r%u\n", r);
            }
        }
        else if (ctx->rxonly)
        {
            /* expect the deterministic 0xFF fill a recv-only master must
             * send (DMA leg sources the static 0xFF fill buffer, PIO leg
             * writes a constant 0xFF); any other byte is a driver defect */
            rt_uint32_t i;

            for (i = 0; i < ctx->len; i++)
            {
                if (ctx->rx_b[i] != 0xFF)
                {
                    break;
                }
            }
            if (i == ctx->len)
            {
                PAIR_LOG("[spi] slave r%u ok: master dummy all 0xFF\n", r);
            }
            else
            {
                PAIR_LOG("[spi] slave r%u FAIL: expect 0xFF dummy, got non-0xFF@%u"
                         " rx[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x"
                         " (master recv-only TX was not 0xFF dummy, driver defect)\n",
                         r, i, ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                         ctx->rx_b[4], ctx->rx_b[5], ctx->rx_b[6], ctx->rx_b[7]);
                spi_pair_reg_dump("rxonly", RT_FALSE);
                round_ok = RT_FALSE;
            }
        }

        rt_sem_release(ctx->ready_b);   /* round done: master may verify rx_b */
        r++;
        if (!round_ok)
        {
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
    }

    rt_sem_release(ctx->done_b);
}

/* -------------------- FD one-way message (SO/RO) threads --------------------
 * xdir=1 (SO): master send-only  <-> slave recv-only
 * xdir=2 (RO): master recv-only  <-> slave send-only
 * Exercises spixfer send-only/recv-only dispatch in FULL-DUPLEX config:
 * DMA  >=10B -> SPI_DMA_Transmit / SPI_DMA_Receive (+ ISR Direct==SPI_Tx/Rx)
 * PIO  <10B  -> SPI_Transmit    / SP_Receive
 * Slave PIO path allowed for one-way (only bidirectional PIO is master-only).
 */
static void spi_dir_slave_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_FALSE);

    while (!ctx->stop)
    {
        rt_ssize_t n;
        rt_uint32_t diff;
        rt_bool_t round_ok = RT_TRUE;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }

        if (ctx->xdir == 1u)
        {
            /* slave recv-only: expect master saltA stream */
            rt_sem_release(ctx->ready_b);   /* armed: master may clock */
            n = rt_spi_transfer(&ctx->g->dev_b, RT_NULL, ctx->rx_b, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[dir] slave recv FAIL: ret=%d len=%u\n", (int)n, ctx->len);
                spi_pair_reg_dump("fail", RT_FALSE);
                round_ok = RT_FALSE;
            }
            else
            {
                diff = spi_pair_cmp_gen(ctx->rx_b, ctx->len, r, ctx->salt_a, ctx->data_width);
                if (diff != ctx->len)
                {
                    PAIR_LOG("[dir] SLAVE(rx-only) rx FAIL r%u: diff@%u rx[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                             r, diff, ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                             ctx->rx_b[4], ctx->rx_b[5], ctx->rx_b[6], ctx->rx_b[7],
                             spi_pair_is_all(ctx->rx_b, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                             " [rx buffer untouched!]" : "");
                    spi_pair_reg_dump("fail", RT_FALSE);
                    round_ok = RT_FALSE;
                }
            }
        }
        else
        {
            /* slave send-only: put saltB on MISO for master clocking */
            spi_pair_gen(ctx->tx_b, ctx->len, r, ctx->salt_b, ctx->data_width);
            rt_sem_release(ctx->ready_b);
            n = rt_spi_transfer(&ctx->g->dev_b, ctx->tx_b, RT_NULL, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[dir] slave send FAIL: ret=%d len=%u\n", (int)n, ctx->len);
                spi_pair_reg_dump("fail", RT_FALSE);
                round_ok = RT_FALSE;
            }
        }

        r++;
        if (!round_ok)
        {
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
    }

    rt_sem_release(ctx->done_b);
}

static void spi_dir_master_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_TRUE);

    while (!ctx->stop)
    {
        rt_ssize_t n;
        rt_uint32_t diff;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }

        if (rt_sem_take(ctx->ready_b, RT_TICK_PER_SECOND * (ctx->round_wait_ms / 1000u)) != RT_EOK)
        {
            if (ctx->stop)
            {
                break;
            }
            PAIR_LOG("[dir] master: slave-ready timeout!\n");
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
        if (ctx->stop)
        {
            break;
        }
        if (SPI_PAIR_ARM_MARGIN_MS > 0)
        {
            rt_thread_mdelay(SPI_PAIR_ARM_MARGIN_MS);
        }

        if (ctx->xdir == 1u)
        {
            /* master send-only: clock out saltA */
            spi_pair_gen(ctx->tx_a, ctx->len, r, ctx->salt_a, ctx->data_width);
            n = rt_spi_transfer(&ctx->g->dev_a, ctx->tx_a, RT_NULL, ctx->len);
        }
        else
        {
            /* master recv-only: clock in saltB (driver emits fill dummy) */
            n = rt_spi_transfer(&ctx->g->dev_a, RT_NULL, ctx->rx_a, ctx->len);
            if (n == (rt_ssize_t)ctx->len)
            {
                diff = spi_pair_cmp_gen(ctx->rx_a, ctx->len, r, ctx->salt_b, ctx->data_width);
                if (diff != ctx->len)
                {
                    PAIR_LOG("[dir] MASTER(rx-only) rx FAIL r%u: diff@%u rx[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                             r, diff, ctx->rx_a[0], ctx->rx_a[1], ctx->rx_a[2], ctx->rx_a[3],
                             ctx->rx_a[4], ctx->rx_a[5], ctx->rx_a[6], ctx->rx_a[7],
                             spi_pair_is_all(ctx->rx_a, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                             " [rx buffer untouched!]" : "");
                    spi_pair_reg_dump("fail", RT_TRUE);
                    ctx->failed = RT_TRUE;
                    ctx->stop = RT_TRUE;
                    break;
                }
            }
        }

        if (n != (rt_ssize_t)ctx->len)
        {
            PAIR_LOG("[dir] master %s FAIL: ret=%d len=%u\n",
                     ctx->xdir == 1u ? "send" : "recv", (int)n, ctx->len);
            spi_pair_reg_dump("fail", RT_TRUE);
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }

        r++;
    }

    rt_sem_release(ctx->done_a);
}

static int spi_dir_run_case(rt_uint32_t len, rt_uint32_t rounds,
                            rt_uint32_t speed_khz, rt_uint32_t mode,
                            rt_uint32_t xdir, rt_uint32_t data_width, rt_bool_t lsb)
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;
    rt_err_t rc;

    rc = spi_pair_setup(ctx, xdir == 1u ? "SO" : "RO", len, RT_FALSE, rounds,
                        speed_khz, mode, RT_FALSE, RT_FALSE, data_width, lsb);
    if (rc != RT_EOK)
    {
        return -RT_ERROR;
    }
    ctx->xdir = xdir;
    PAIR_LOG("[spi] %s len=%u mode=%u khz=%u rounds=%s 8bit %s\n",
             xdir == 1u ? "SO  master tx-only<->slave rx-only"
                        : "RO  master rx-only<->slave tx-only",
             len, mode, speed_khz, rounds == 0 ? "INF" : "finite",
             (len < 10) ? "(PIO)" : "(DMA)");
    if (spi_pair_spawn(ctx, spi_dir_slave_entry, spi_dir_master_entry) != RT_EOK)
    {
        return -RT_ERROR;
    }
    return spi_pair_wait_finish(ctx, rounds == 0, rounds);
}

/* ------------------------- FULL DUPLEX: runner ------------------------- */

static int spi_pair_run_case(rt_uint32_t len, rt_bool_t aligned, rt_uint32_t rounds,
                             rt_uint32_t speed_khz, rt_uint32_t mode,
                             rt_bool_t rxonly, rt_uint32_t data_width,
                             rt_bool_t lsb)
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;

    if (rxonly && data_width != 8u)
    {
        PAIR_LOG("rxonly is 8-bit only\n");
        return -RT_ERROR;
    }
    if (spi_pair_setup(ctx, rxonly ? "RXONLY" : "PAIR", len, aligned, rounds,
                       speed_khz, mode, RT_FALSE, rxonly, data_width, lsb) != RT_EOK)
    {
        return -RT_ERROR;
    }

    {
        /* case banner: RXONLY probe or PAIR with the group's print tags */
        char desc[64];

        if (rxonly)
        {
            rt_snprintf(desc, sizeof(desc),
                        "RXONLY(master recv-only)");
        }
        else
        {
            rt_snprintf(desc, sizeof(desc), "PAIR  %s<->%s full-duplex",
                        ctx->g->a_tag, ctx->g->b_tag);
        }
        PAIR_LOG("[spi] %s len=%u mode=%u align=%s khz=%u rounds=%s 8bit %s\n",
                 desc, len, mode, aligned ? "1" : "0", speed_khz,
                 rounds == 0 ? "INF" : "finite",
                 (len < 10) ? "(PIO leg: <10B is CPU-polled)"
                            : "");
    }
    if (rxonly && len >= 10)
    {
        PAIR_LOG("[spi] rxonly DMA leg clocks the 0xFF fill (fixed); a FAIL"
                 " below is a real driver defect\n");
    }

    if (spi_pair_spawn(ctx, spi_pair_slave_entry, spi_pair_master_entry) != RT_EOK)
    {
        return -RT_ERROR;
    }
    return spi_pair_wait_finish(ctx, rounds == 0, rounds);
}

/* ------------------------- HALF DUPLEX (3-wire): threads ------------------------- */

/* Half-duplex alternation, one leg per direction per round:
 *   leg AB: master send-only (tx_a)  <-> slave recv-only (rx_b)
 *   leg BA: master recv-only (rx_a)  <-> slave send-only (tx_b)
 * Both ends self-verify their own rx against the deterministic peer pattern.
 * The driver toggles SINGLELINE_TX/SINGLELINE_RX (with DeInit/Init) on every
 * message and disables SPI at message end (3-wire) -> this is the mode-switch
 * stress test. */
static void spi_hdu_slave_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_FALSE);

    while (!ctx->stop)
    {
        rt_ssize_t n;
        rt_uint32_t diff;
        rt_bool_t round_ok = RT_TRUE;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }

        /* leg AB: slave receives master data */
        if ((ctx->legs & 1u) != 0u)
        {
            rt_sem_release(ctx->ready_b);
            n = rt_spi_transfer(&ctx->g->dev_b, RT_NULL, ctx->rx_b, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[hdu] slave(AB recv) FAIL: ret=%d len=%u\n", (int)n, ctx->len);
                spi_pair_reg_dump("fail", RT_FALSE);
                round_ok = RT_FALSE;
                goto __leg_done;
            }
            diff = spi_pair_cmp_gen(ctx->rx_b, ctx->len, r, ctx->salt_a, ctx->data_width);
            if (diff != ctx->len)
            {
                PAIR_LOG("[hdu] slave(AB recv) FAIL r%u: diff@%u rx[0..3]=%02x %02x %02x %02x%s\n",
                         r, diff, ctx->rx_b[0], ctx->rx_b[1], ctx->rx_b[2], ctx->rx_b[3],
                         spi_pair_is_all(ctx->rx_b, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                         " [rx buffer untouched!]" : "");
                spi_pair_reg_dump("fail", RT_FALSE);
                round_ok = RT_FALSE;
                goto __leg_done;
            }
        }

        /* leg BA: slave sends its pattern */
        if ((ctx->legs & 2u) != 0u)
        {
            spi_pair_gen(ctx->tx_b, ctx->len, r, ctx->salt_b, ctx->data_width);
            rt_sem_release(ctx->ready_b);
            n = rt_spi_transfer(&ctx->g->dev_b, ctx->tx_b, RT_NULL, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[hdu] slave(BA send) FAIL: ret=%d len=%u\n", (int)n, ctx->len);
                spi_pair_reg_dump("fail", RT_FALSE);
                round_ok = RT_FALSE;
            }
        }

__leg_done:
        r++;
        if (!round_ok)
        {
            ctx->failed = RT_TRUE;
            ctx->stop = RT_TRUE;
            break;
        }
        /* pacing heartbeat not needed: master drives rounds */
    }

    rt_sem_release(ctx->done_b);
}

static void spi_hdu_master_entry(void *param)
{
    struct spi_pair_ctx *ctx = (struct spi_pair_ctx *)param;
    rt_uint32_t r = 0;

    spi_pair_reg_dump("open", RT_TRUE);

    while (!ctx->stop)
    {
        rt_uint32_t t0;
        rt_ssize_t n;
        rt_uint32_t diff;

        if (ctx->rounds != 0 && r >= ctx->rounds)
        {
            break;
        }
        t0 = rt_tick_get();

        /* leg AB: master sends */
        if ((ctx->legs & 1u) != 0u)
        {
            if (rt_sem_take(ctx->ready_b, RT_TICK_PER_SECOND * (ctx->round_wait_ms / 1000u)) != RT_EOK)
            {
                if (ctx->stop)
                {
                    break;
                }
                PAIR_LOG("[hdu] master: slave-ready timeout (leg AB)!\n");
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
            if (ctx->stop)
            {
                break;
            }
            if (SPI_PAIR_ARM_MARGIN_MS > 0)
            {
                rt_thread_mdelay(SPI_PAIR_ARM_MARGIN_MS);
            }
            spi_pair_gen(ctx->tx_a, ctx->len, r, ctx->salt_a, ctx->data_width);
            n = rt_spi_transfer(&ctx->g->dev_a, ctx->tx_a, RT_NULL, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[hdu] master(AB send) FAIL: ret=%d len=%u\n", (int)n, ctx->len);
                spi_pair_reg_dump("fail", RT_TRUE);
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
        }

        /* leg BA: master receives slave data */
        if ((ctx->legs & 2u) != 0u)
        {
            if (rt_sem_take(ctx->ready_b, RT_TICK_PER_SECOND * (ctx->round_wait_ms / 1000u)) != RT_EOK)
            {
                if (ctx->stop)
                {
                    break;
                }
                PAIR_LOG("[hdu] master: slave-ready timeout (leg BA)!\n");
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
            if (ctx->stop)
            {
                break;
            }
            if (SPI_PAIR_ARM_MARGIN_MS > 0)
            {
                rt_thread_mdelay(SPI_PAIR_ARM_MARGIN_MS);
            }
            n = rt_spi_transfer(&ctx->g->dev_a, RT_NULL, ctx->rx_a, ctx->len);
            if (n != (rt_ssize_t)ctx->len)
            {
                PAIR_LOG("[hdu] master(BA recv) FAIL: ret=%d len=%u (%ums)\n",
                         (int)n, ctx->len, spi_pair_tick2ms(rt_tick_get() - t0));
                spi_pair_reg_dump("fail", RT_TRUE);
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
            diff = spi_pair_cmp_gen(ctx->rx_a, ctx->len, r, ctx->salt_b, ctx->data_width);
            if (diff != ctx->len)
            {
                PAIR_LOG("[hdu] master(BA recv) FAIL r%u: diff@%u rx[0..3]=%02x %02x %02x %02x rx[%u..%u]=%02x %02x %02x %02x%s\n",
                         r, diff, ctx->rx_a[0], ctx->rx_a[1], ctx->rx_a[2], ctx->rx_a[3],
                         ctx->len > 4 ? ctx->len - 4 : 0, ctx->len - 1,
                         ctx->rx_a[ctx->len > 4 ? ctx->len - 4 : 0],
                         ctx->rx_a[ctx->len > 3 ? ctx->len - 3 : 0],
                         ctx->rx_a[ctx->len > 2 ? ctx->len - 2 : 0],
                         ctx->rx_a[ctx->len - 1],
                         spi_pair_is_all(ctx->rx_a, ctx->len * (ctx->data_width / 8u), SPI_PAIR_RX_FILL) ?
                         " [rx buffer untouched!]" : "");
                spi_pair_reg_dump("fail", RT_TRUE);
                ctx->failed = RT_TRUE;
                ctx->stop = RT_TRUE;
                break;
            }
        }

        {
            rt_uint32_t ms = spi_pair_tick2ms(rt_tick_get() - t0);
            if (ctx->ok_rounds == 0)
            {
                ctx->min_ms = ms;
                ctx->max_ms = ms;
            }
            else
            {
                if (ms < ctx->min_ms) ctx->min_ms = ms;
                if (ms > ctx->max_ms) ctx->max_ms = ms;
            }
        }
        ctx->ok_rounds++;
        if (ctx->rounds == 0 && ctx->ok_rounds % 100 == 0)
        {
            PAIR_LOG("[hdu] r%u ok\n", r);
        }
        r++;
    }

    rt_sem_release(ctx->done_a);
}

/* ------------------------- HALF DUPLEX: runner ------------------------- */

/* Engine annotation for the 3-wire rows. The driver polls with PIO only for
 * the master SEND leg (AB) below 10 elements; the master RECV leg (BA) goes
 * through DMA at every length (driver rule since 2026-09-10: the PIO poll
 * raced the free-running RX clock and stored partially shifted bytes, see
 * the master recv-only dispatch note in drv_spi.c spixfer). A leg set that
 * contains BA is therefore DMA for the BA part. */
static const char *spi_3w_xfer_kind(rt_uint32_t len, rt_uint32_t legs)
{
    if (len >= 10u)
    {
        return "(DMA)";
    }
    if (legs == 1u)
    {
        return "(PIO)";
    }
    if (legs == 2u)
    {
        return "(DMA)";
    }
    return "(AB:PIO,BA:DMA)";
}

static int spi_hdu_run_case(rt_uint32_t len, rt_uint32_t rounds,
                            rt_uint32_t speed_khz, rt_uint32_t mode,
                            rt_uint32_t legs, rt_int32_t mode_b)
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;

    if (spi_pair_setup(ctx, "HDU", len, RT_FALSE, rounds, speed_khz, mode,
                       RT_TRUE, RT_FALSE, 8u, RT_FALSE) != RT_EOK)
    {
        return -RT_ERROR;
    }
    ctx->legs = legs;
    ctx->spi_mode_b = mode_b;
    /* setup() configured both devices while spi_mode_b was still reset -1;
     * re-apply config so the slave-mode override reaches the hardware. */
    if (spi_pair_config(ctx) != RT_EOK)
    {
        return -RT_ERROR;
    }

    PAIR_LOG("[spi] HDU(3-wire half-duplex) len=%u mode=%u%s khz=%u rounds=%s "
             "legs=%u %s\n",
             len, mode, (mode_b >= 0) ? "+b" : "", speed_khz,
             rounds == 0 ? "INF" : "finite",
             legs, spi_3w_xfer_kind(len, legs));
    if (mode_b >= 0)
    {
        PAIR_LOG("[spi]   slave mode override: %d (diag)\n", mode_b);
    }
    if (spi_pair_spawn(ctx, spi_hdu_slave_entry, spi_hdu_master_entry) != RT_EOK)
    {
        return -RT_ERROR;
    }
    return spi_pair_wait_finish(ctx, rounds == 0, rounds);
}

/* ------------------------- MSH commands ------------------------- */

static int spi_pair(int argc, char *argv[])
{
    rt_uint32_t len = 8192;
    rt_bool_t aligned = RT_FALSE;
    rt_uint32_t rounds = 0;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode = 0;
    rt_uint32_t width = 8;
    rt_uint32_t lsb = 0;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) aligned = (spi_pair_atoi(argv[2]) != 0);
    if (argc >= 4) rounds = spi_pair_atoi(argv[3]);
    if (argc >= 5) khz = spi_pair_atoi(argv[4]);
    if (argc >= 6) mode = spi_pair_atoi(argv[5]);
    if (argc >= 7) width = spi_pair_atoi(argv[6]);
    if (argc >= 8) lsb = spi_pair_atoi(argv[7]);

    if (spi_pair_group_apply(argc, argv, 7) != RT_EOK) return -RT_ERROR;
    return spi_pair_run_case(len, aligned, rounds, khz, mode, RT_FALSE, width,
                             lsb != 0);
}
MSH_CMD_EXPORT(spi_pair, FD 4-wire full-duplex pair: spi_pair [len] [aligned] [rounds] [khz] [mode0-3]);

static int spi_hdu(int argc, char *argv[])
{
    rt_uint32_t len = 8;
    rt_uint32_t rounds = 1;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode = 0;
    rt_uint32_t legs = 3;
    rt_int32_t mode_b = -1;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) rounds = spi_pair_atoi(argv[2]);
    if (argc >= 4) khz = spi_pair_atoi(argv[3]);
    if (argc >= 5) mode = spi_pair_atoi(argv[4]);
    if (argc >= 6) legs = spi_pair_atoi(argv[5]);
    if (argc >= 7) mode_b = (rt_int32_t)spi_pair_atoi(argv[6]);
    if (rounds == 0)
    {
        PAIR_LOG("hdu rounds must be >=1\n");
        return -RT_ERROR;
    }
    if (legs < 1 || legs > 3)
    {
        PAIR_LOG("hdu legs must be 1(AB) 2(BA) or 3(both)\n");
        return -RT_ERROR;
    }
    if (mode_b > 3)
    {
        PAIR_LOG("hdu slave mode 0..3\n");
        return -RT_ERROR;
    }
    if (spi_pair_group_apply(argc, argv, 6) != RT_EOK) return -RT_ERROR;
    return spi_hdu_run_case(len, rounds, khz, mode, legs, mode_b);
}
MSH_CMD_EXPORT(spi_hdu, HD 3-wire HD alt: spi_hdu [len] [rounds] [khz] [mode0-3] [legs1-3] [slv_mode0-3]);

static int spi_bat(int argc, char *argv[])
{
    rt_uint32_t rounds = 2;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    static const rt_uint32_t lens[] = { 9, 4095, 4096, 8192 };
    rt_uint32_t ai, li;
    int failed = 0;

    if (argc >= 2) rounds = spi_pair_atoi(argv[1]);
    if (argc >= 3) khz = spi_pair_atoi(argv[2]);
    if (rounds == 0 || rounds > 100)
    {
        PAIR_LOG("rounds_each must be 1..100\n");
        return -RT_ERROR;
    }

    if (spi_pair_group_apply(argc, argv, 2) != RT_EOK) return -RT_ERROR;
    for (ai = 0; ai < 2; ai++)
    {
        for (li = 0; li < sizeof(lens) / sizeof(lens[0]); li++)
        {
            PAIR_LOG("===== battery: len=%u aligned=%u rounds=%u =====\n",
                     lens[li], ai, rounds);
            if (spi_pair_run_case(lens[li], (rt_bool_t)ai, rounds, khz, 0, RT_FALSE,
                                 8u, RT_FALSE) != RT_EOK)
            {
                failed++;
            }
        }
    }

    PAIR_LOG("[spi] battery done: %u/8 cases failed\n", failed);
    return failed ? -RT_ERROR : RT_EOK;
}
MSH_CMD_EXPORT(spi_bat, spi battery FD (len 9/4095/4096/8192 x aligned 0/1): spi_bat [rounds_each] [khz]);

/* ------------------------- combination matrix (spi_all) ------------------------- */

/* 归类:
 *   EXPECT_PASS : 4 线全双工行 -> 预期 PASS
 *   OBSERVE     : 半双工(3-wire)行 -> 需 3W 单线接法(主 MOSI <-> 从 MISO);
 *                 4 线接法下该网不存在, FAIL 属接法产物 -> 结果如实记录,
 *                 PASS/FAIL 均不算"意外"
 * 历史(勿再当缺陷): 早期把 "aligned=1 FD DMA 直发(共用缓冲, recv_buf 不拷回)"
 * 与 "len<10 全双工 slave PIO -EIO" 记为 known 缺陷(预期 FAIL);两者均已修复
 * (前者早于 6ae7b60e3d, 后者随 slave 腿恒走 DMA)——2026-09-10 实测该类 6 行
 * 全部 PASS, 故不再单列 known/false_pass 计数, 也不再有"预期 FAIL"的用例。
 */
enum spi_all_kind
{
    SPI_ALL_EXPECT_PASS = 0,
    SPI_ALL_OBSERVE,
};

struct spi_all_result
{
    rt_uint32_t pass;
    rt_uint32_t unexpected_fail;
    rt_uint32_t observe_pass;
    rt_uint32_t observe_fail;
};

static void spi_all_report_one(const char *phase, const char *desc,
                               enum spi_all_kind kind, int res,
                               struct spi_all_result *agg)
{
    const char *verdict;

    if (res == RT_EOK)
    {
        if (kind == SPI_ALL_OBSERVE)
        {
            agg->observe_pass++;
            verdict = "PASS(observe)";
        }
        else
        {
            agg->pass++;
            verdict = "PASS";
        }
    }
    else
    {
        switch (kind)
        {
        case SPI_ALL_OBSERVE:
            agg->observe_fail++;
            verdict = "FAIL(observe: needs the 3-wire single-line hookup)";
            break;
        default:
            agg->unexpected_fail++;
            verdict = "FAIL(unexpected!)";
            break;
        }
    }
    PAIR_LOG("[spi_all] [%s] %s => %s\n", phase, desc, verdict);
}

static int spi_all(int argc, char *argv[])
{
    rt_uint32_t rounds = 1;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    struct spi_all_result agg;
    rt_uint32_t mode;

    if (argc >= 2) rounds = spi_pair_atoi(argv[1]);
    if (argc >= 3) khz = spi_pair_atoi(argv[2]);
    if (rounds == 0 || rounds > 20)
    {
        PAIR_LOG("rounds_each must be 1..20\n");
        return -RT_ERROR;
    }

    if (spi_pair_group_apply(argc, argv, 2) != RT_EOK) return -RT_ERROR;
    rt_memset(&agg, 0, sizeof(agg));
    PAIR_LOG("===== spi_all: %s<->%s combination matrix (rounds=%u khz=%u)"
             " 4-wire hookup; HD rows need the 3-wire single-line hookup =====\n",
             spi_pair_cur_g->a_tag, spi_pair_cur_g->b_tag, rounds, khz);

    /* Phase A: FD(4-wire) length/aligned matrix */
    {
        static const rt_uint32_t lens[] = { 9, 64, 4095, 4096, 8192 };
        rt_uint32_t ai, li;
        char desc[48];
        int res;

        for (ai = 0; ai < 2; ai++)
        {
            for (li = 0; li < sizeof(lens) / sizeof(lens[0]); li++)
            {
                rt_snprintf(desc, sizeof(desc), "FD len=%u align=%s mode0",
                            lens[li], ai ? "1" : "0");
                res = spi_pair_run_case(lens[li], (rt_bool_t)ai, rounds, khz, 0, RT_FALSE,
                                   8u, RT_FALSE);
                spi_all_report_one("A-FD", desc, SPI_ALL_EXPECT_PASS, res, &agg);
            }
        }
    }

    /* Phase B: FD clock-mode sweep 0..3 */
    for (mode = 0; mode <= 3; mode++)
    {
        char desc[48];
        int res;

        rt_snprintf(desc, sizeof(desc), "FD mode%u len=128 align=0", mode);
        res = spi_pair_run_case(128, RT_FALSE, rounds, khz, mode, RT_FALSE,
                             8u, RT_FALSE);
        spi_all_report_one("B-MODE", desc, SPI_ALL_EXPECT_PASS, res, &agg);
    }

    /* Phase C: HD(3-wire) PIO (<10B), mode 0 and 3 */
    {
        static const rt_uint32_t hdu_lens[] = { 8, 9 };
        rt_uint32_t li;

        for (li = 0; li < sizeof(hdu_lens) / sizeof(hdu_lens[0]); li++)
        {
            char desc[48];
            int res;

            rt_snprintf(desc, sizeof(desc), "HD len=%u mode0(PIO)", hdu_lens[li]);
            res = spi_hdu_run_case(hdu_lens[li], rounds, khz, 0, 3u, -1);
            spi_all_report_one("C-HD-PIO", desc, SPI_ALL_OBSERVE, res, &agg);
        }
    }

    /* Phase D: HD(3-wire) DMA >=10B (recv-only legs probe driver bug) */
    {
        static const rt_uint32_t hdu_lens[] = { 16, 256, 4095 };
        rt_uint32_t li;

        for (li = 0; li < sizeof(hdu_lens) / sizeof(hdu_lens[0]); li++)
        {
            char desc[48];
            int res;

            rt_snprintf(desc, sizeof(desc), "HD len=%u mode0(DMA)", hdu_lens[li]);
            res = spi_hdu_run_case(hdu_lens[li], rounds, khz, 0, 3u, -1);
            spi_all_report_one("D-HD-DMA", desc, SPI_ALL_OBSERVE, res, &agg);
        }
    }

    PAIR_LOG("===== spi_all summary: pass=%u unexpected_fail=%u"
             " observe(PASS/FAIL)=%u/%u =====\n",
             agg.pass, agg.unexpected_fail,
             agg.observe_pass, agg.observe_fail);

    if (agg.unexpected_fail != 0)
    {
        PAIR_LOG("[spi_all] NEEDS ATTENTION: unexpected fail above!\n");
        return -RT_ERROR;
    }
    PAIR_LOG("[spi_all] done (no unexpected failures; HD rows are wiring-bound)\n");
    return RT_EOK;
}
MSH_CMD_EXPORT(spi_all, current-group sequential combo matrix FD+HD+modes: spi_all [rounds_each] [khz]);

/* ---------------------- FD 4-wire full matrix (spi_fdall) ----------------------
 * Full-coverage runner for the 4-wire full-duplex DMA path only: every
 * combination of mode 0..3 x len {9..8192} x aligned {0,1}. All rows are
 * expected PASS in the current driver (len<10 now rides the slave's always-DMA
 * leg, so the old slave-PIO -EIO limitation no longer applies; aligned=1 FD was
 * fixed in the DMA regression). A len<10 failure would still be reported as
 * observe rather than "unexpected". */

static int spi_fdall(int argc, char *argv[])
{
    static const rt_uint32_t lens[] = { 9, 10, 64, 128, 4095, 4096, 8192 };
    rt_uint32_t rounds = 3;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode, li, ai;
    rt_uint32_t total = 0, n_pass = 0, n_obs = 0, n_unexp = 0;

    if (argc >= 2) rounds = spi_pair_atoi(argv[1]);
    if (argc >= 3) khz = spi_pair_atoi(argv[2]);
    if (rounds == 0 || rounds > 50)
    {
        PAIR_LOG("rounds_each must be 1..50\n");
        return -RT_ERROR;
    }
    if (khz < SPI_PAIR_SPEED_MIN_KHZ || khz > SPI_PAIR_SPEED_MAX_KHZ)
    {
        PAIR_LOG("khz must be %u..%u\n", SPI_PAIR_SPEED_MIN_KHZ,
                 SPI_PAIR_SPEED_MAX_KHZ);
        return -RT_ERROR;
    }

    if (spi_pair_group_apply(argc, argv, 2) != RT_EOK) return -RT_ERROR;
    PAIR_LOG("===== spi_fdall: 4-wire FD matrix mode0-3 x len x align"
             " (rounds=%u khz=%u) =====\n", rounds, khz);
    for (mode = 0; mode <= 3; mode++)
    {
        for (li = 0; li < sizeof(lens) / sizeof(lens[0]); li++)
        {
            for (ai = 0; ai < 2; ai++)
            {
                const char *verdict;
                int res;

                total++;
                res = spi_pair_run_case(lens[li], (rt_bool_t)ai, rounds, khz,
                                        mode, RT_FALSE, 8u, RT_FALSE);
                if (res == RT_EOK)
                {
                    n_pass++;
                    verdict = "PASS";
                }
                else if (lens[li] < 10u)
                {
                    n_obs++;
                    verdict = "FAIL(observe: slave PIO <10B limitation)";
                }
                else
                {
                    n_unexp++;
                    verdict = "FAIL(unexpected!)";
                }
                PAIR_LOG("[fdall] mode%u len=%-4u align=%u => %s\n",
                         mode, lens[li], ai, verdict);
            }
        }
    }
    PAIR_LOG("===== spi_fdall summary: total=%u pass=%u observe_fail=%u"
             " unexpected_fail=%u =====\n", total, n_pass, n_obs, n_unexp);
    return n_unexp ? -RT_ERROR : RT_EOK;
}
MSH_CMD_EXPORT(spi_fdall, FD 4-wire full matrix mode0-3 x len x align: spi_fdall [rounds_each] [khz]);

/* ---------------------- 3-wire full matrix (spi_3wall) ----------------------
 * Full-coverage runner for the 3-wire single-data-line path (master MOSI <->
 * slave MISO): every combination of mode 0..3 x len {9..8192} x
 * align {0,1} x leg {AB,BA} = 112 rows. Each row is an independent
 * hdu-style case (fresh setup/arm/spawn/wait_finish) so failures stay
 * isolated and attributable to one leg; the len<10 rows exercise the master
 * PIO branch (driver DMA threshold is 10). mode2 rows that fail with the
 * documented cold-start first-bit race (drv_spi.c NOTE) are counted in a
 * known bucket; any other failure is unexpected. */

static int spi_3wall_run_row(rt_uint32_t len, rt_bool_t aligned,
                             rt_uint32_t rounds, rt_uint32_t khz,
                             rt_uint32_t mode, rt_uint32_t legs)
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;

    if (spi_pair_setup(ctx, "3WALL", len, aligned, rounds, khz, mode,
                       RT_TRUE, RT_FALSE, 8u, RT_FALSE) != RT_EOK)
    {
        return -RT_ERROR;
    }
    ctx->legs = legs;           /* 1 = AB master send, 2 = BA master recv */
    PAIR_LOG("[spi] 3WALL len=%u mode=%u align=%s leg=%s khz=%u rounds=%u %s\n",
             len, mode, aligned ? "1" : "0", legs == 1u ? "AB" : "BA",
             khz, rounds, spi_3w_xfer_kind(len, legs));
    if (spi_pair_spawn(ctx, spi_hdu_slave_entry, spi_hdu_master_entry) != RT_EOK)
    {
        return -RT_ERROR;
    }
    return spi_pair_wait_finish(ctx, RT_FALSE, rounds);
}

static int spi_3wall(int argc, char *argv[])
{
    static const rt_uint32_t lens[] = { 9, 10, 64, 128, 4095, 4096, 8192 };
    rt_uint32_t rounds = 3;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode, li, ai, leg;
    rt_uint32_t total = 0, n_pass = 0, n_known = 0, n_obs = 0, n_unexp = 0;

    if (argc >= 2) rounds = spi_pair_atoi(argv[1]);
    if (argc >= 3) khz = spi_pair_atoi(argv[2]);
    if (rounds == 0 || rounds > 50)
    {
        PAIR_LOG("rounds_each must be 1..50\n");
        return -RT_ERROR;
    }
    if (khz < SPI_PAIR_SPEED_MIN_KHZ || khz > SPI_PAIR_SPEED_MAX_KHZ)
    {
        PAIR_LOG("khz must be %u..%u\n", SPI_PAIR_SPEED_MIN_KHZ,
                 SPI_PAIR_SPEED_MAX_KHZ);
        return -RT_ERROR;
    }
    if (spi_pair_group_apply(argc, argv, 2) != RT_EOK) return -RT_ERROR;

    PAIR_LOG("===== spi_3wall: 3-wire single-line matrix mode0-3 x len x align"
             " x leg (rounds=%u khz=%u) =====\n", rounds, khz);
    for (mode = 0; mode <= 3; mode++)
    {
        for (li = 0; li < sizeof(lens) / sizeof(lens[0]); li++)
        {
            for (ai = 0; ai < 2; ai++)
            {
                for (leg = 1; leg <= 2; leg++)
                {
                    const char *verdict;
                    const char *leg_name;
                    int res;

                    leg_name = (leg == 1u) ? "AB" : "BA";
                    total++;
                    res = spi_3wall_run_row(lens[li], (rt_bool_t)ai, rounds,
                                            khz, mode, leg);
                    if (res == RT_EOK)
                    {
                        n_pass++;
                        verdict = "PASS";
                    }
                    else if (mode == 2u)
                    {
                        /* documented cold-start first-bit race (drv_spi NOTE);
                         * driver-fix candidate, see plan phase C */
                        n_known++;
                        verdict = "FAIL(known: mode2 3-wire cold-start race)";
                    }
                    else
                    {
                        n_unexp++;
                        verdict = "FAIL(unexpected!)";
                    }
                    PAIR_LOG("[3wall] mode%u len=%-4u align=%u leg=%s => %s\n",
                             mode, lens[li], ai, leg_name, verdict);
                }
            }
        }
    }
    PAIR_LOG("===== spi_3wall summary: total=%u pass=%u known_mode2_fail=%u"
             " observe_fail=%u unexpected_fail=%u =====\n",
             total, n_pass, n_known, n_obs, n_unexp);
    return n_unexp ? -RT_ERROR : RT_EOK;
}
MSH_CMD_EXPORT(spi_3wall, 3-wire full matrix mode0-3 x len x align x leg: spi_3wall [rounds_each] [khz]);

static int spi_rxonly(int argc, char *argv[])
{
    rt_uint32_t len = 64;
    rt_uint32_t rounds = 1;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) rounds = spi_pair_atoi(argv[2]);
    if (argc >= 4) khz = spi_pair_atoi(argv[3]);
    if (rounds == 0)
    {
        PAIR_LOG("rxonly rounds must be >=1\n");
        return -RT_ERROR;
    }
    if (spi_pair_group_apply(argc, argv, 3) != RT_EOK) return -RT_ERROR;
    return spi_pair_run_case(len, RT_FALSE, rounds, khz, 0, RT_TRUE, 8u, RT_FALSE);
}

/* one-way message matrix: SO = master send-only vs slave recv-only;
 * RO = master recv-only vs slave send-only. mode 0..3 selectable. */
static int spi_so(int argc, char *argv[])
{
    rt_uint32_t len = 128;
    rt_uint32_t rounds = 1;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode = 0;
    rt_uint32_t width = 8;
    rt_uint32_t lsb = 0;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) rounds = spi_pair_atoi(argv[2]);
    if (argc >= 4) khz = spi_pair_atoi(argv[3]);
    if (argc >= 5) mode = spi_pair_atoi(argv[4]);
    if (argc >= 6) width = spi_pair_atoi(argv[5]);
    if (argc >= 7) lsb = spi_pair_atoi(argv[6]);
    if (spi_pair_group_apply(argc, argv, 6) != RT_EOK) return -RT_ERROR;
    return spi_dir_run_case(len, rounds, khz, mode, 1u, width, lsb != 0);
}
MSH_CMD_EXPORT(spi_so, master send-only vs slave recv-only: spi_so [len] [rounds] [khz] [mode0-3]);

static int spi_ro(int argc, char *argv[])
{
    rt_uint32_t len = 128;
    rt_uint32_t rounds = 1;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t mode = 0;
    rt_uint32_t width = 8;
    rt_uint32_t lsb = 0;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) rounds = spi_pair_atoi(argv[2]);
    if (argc >= 4) khz = spi_pair_atoi(argv[3]);
    if (argc >= 5) mode = spi_pair_atoi(argv[4]);
    if (argc >= 6) width = spi_pair_atoi(argv[5]);
    if (argc >= 7) lsb = spi_pair_atoi(argv[6]);
    if (spi_pair_group_apply(argc, argv, 6) != RT_EOK) return -RT_ERROR;
    return spi_dir_run_case(len, rounds, khz, mode, 2u, width, lsb != 0);
}
MSH_CMD_EXPORT(spi_ro, master recv-only vs slave send-only: spi_ro [len] [rounds] [khz] [mode0-3]);
MSH_CMD_EXPORT(spi_rxonly, master recv-only vs slave fdx (drv bug probe): spi_rxonly [len] [rounds] [khz]);

static int spi_pair_stop(void)
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;

    if (!spi_pair_running)
    {
        PAIR_LOG("spi test not running\n");
        return -RT_ERROR;
    }

    ctx->stop = RT_TRUE;
    if (ctx->ready_b)
    {
        rt_sem_release(ctx->ready_b);   /* wake master blocked on ready */
    }

    /* wait for teardown (threads stuck in driver waits: up to 4s per chunk) */
    if (rt_sem_take(ctx->stopped, (spi_pair_chunk_worst_s(ctx) + 3) * RT_TICK_PER_SECOND) != RT_EOK)
    {
        PAIR_LOG("[spi] stop timeout (threads may still be stuck in driver)!\n");
        return -RT_ERROR;
    }

    PAIR_LOG("[spi] stopped: ok=%u min=%ums max=%ums\n",
             ctx->ok_rounds, ctx->min_ms, ctx->max_ms);
    return RT_EOK;
}
MSH_CMD_EXPORT(spi_pair_stop, stop running spi pair test);

/* ----------------- demo-replica CPU per-byte mode2 control (spi_dm2) -----------------
 * Register-level replica of the vendor FullDuplex_SoftNSS demo loop, to
 * discriminate whether the DMA driver's mode2 (CPOL=1/CPHA=0) one-bit-late
 * stream is caused by the per-message engine cold start or by something in
 * the DMA flow. Single msh thread, no DMA, no semaphores; mode fixed 2,
 * MSB 8-bit (changeable below via salt/mode only by editing).
 *   policy 0 = demo replica: engines enabled once, never cycled (warm)
 *   policy 1 = driver replica: SPIEN cold 0->1 before every round
 * Registers (see header comment L108): CTRL2@+0x04 SPIEN=bit0;
 * STS@+0x08 TE=bit0 RNE=bit1; DAT@+0x0C. */
#define SPI_DM2_POLL_MAX        500000u

static rt_err_t spi_dm2_run_bytes(struct spi_pair_ctx *ctx)
{
    rt_uint32_t i, t;

    for (i = 0; i < ctx->len; i++)
    {
        t = 0;
        while ((spi_pair_reg(RT_TRUE, 0x08) & 0x1u) == 0u)  /* master TE */
        {
            if (++t > SPI_DM2_POLL_MAX)
            {
                return -1;      /* master TX never drained */
            }
        }
        /* demo choreography: preload slave TX while clock idle, then the
         * master write starts the clock; read both sides after RNE */
        spi_pair_regw(RT_FALSE, 0x0C, ctx->tx_b[i]);
        spi_pair_regw(RT_TRUE, 0x0C, ctx->tx_a[i]);
        t = 0;
        while ((spi_pair_reg(RT_FALSE, 0x08) & 0x2u) == 0u) /* slave RNE */
        {
            if (++t > SPI_DM2_POLL_MAX)
            {
                return -2;      /* slave never received */
            }
        }
        ctx->rx_b[i] = (rt_uint8_t)spi_pair_reg(RT_FALSE, 0x0C);
        t = 0;
        while ((spi_pair_reg(RT_TRUE, 0x08) & 0x2u) == 0u)  /* master RNE */
        {
            if (++t > SPI_DM2_POLL_MAX)
            {
                return -3;      /* master never received */
            }
        }
        ctx->rx_a[i] = (rt_uint8_t)spi_pair_reg(RT_TRUE, 0x0C);
    }
    return RT_EOK;
}

/* master and/or slave SPIEN on/off (RMW: only CTRL2 bit0 touched) */
static void spi_dm2_engine(rt_bool_t m_on, rt_bool_t s_on)
{
    rt_uint16_t c2 = (rt_uint16_t)spi_pair_reg(RT_TRUE, 0x04);

    c2 = (rt_uint16_t)(m_on ? (c2 | 0x1u) : (c2 & (rt_uint16_t)~0x1u));
    spi_pair_regw(RT_TRUE, 0x04, c2);
    c2 = (rt_uint16_t)spi_pair_reg(RT_FALSE, 0x04);
    c2 = (rt_uint16_t)(s_on ? (c2 | 0x1u) : (c2 & (rt_uint16_t)~0x1u));
    spi_pair_regw(RT_FALSE, 0x04, c2);
}

/* ---- wire-level probes: PID (pad input data) of the SPI net pins ----
 * GPIO PID@+0x10. Bases (N32H76x AHB5): GPIOA 0x58032800, GPIOC 0x58033000,
 * GPIOD 0x58033400. Pins: PA5=SPI1.SCK(master out, bit5), PA7=MOSI(bit7),
 * PA6=MISO(bit6), PD3=SPI2.SCK(slave in, bit3), PC3=SPI2.MOSI(bit3),
 * PC2=SPI2.MISO(bit2). */
#define SPI_DM2_PID_A  0x58032810UL
#define SPI_DM2_PID_C  0x58033010UL
#define SPI_DM2_PID_D  0x58033410UL

static void spi_dm2_wire_probe(const char *tag)
{
    rt_uint32_t a = *(volatile rt_uint32_t *)SPI_DM2_PID_A;
    rt_uint32_t c = *(volatile rt_uint32_t *)SPI_DM2_PID_C;
    rt_uint32_t d = *(volatile rt_uint32_t *)SPI_DM2_PID_D;

    PAIR_LOG("[dm2] wire %-4s SCK PA5=%u PD3=%u | MOSI PA7=%u PC3=%u | MISO PA6=%u PC2=%u\n",
             tag, (a >> 5) & 1u, (d >> 3) & 1u, (a >> 7) & 1u, (c >> 3) & 1u,
             (a >> 6) & 1u, (c >> 2) & 1u);
}

/* print first failing line (rx vs expect) of one direction, 8 bytes each */
static void spi_dm2_fail_dump(const char *who, rt_uint32_t r, rt_uint32_t diff,
                              const rt_uint8_t *rx, const rt_uint8_t *exp,
                              rt_uint32_t n)
{
    rt_uint32_t i;

    if (n > 8)
    {
        n = 8;
    }
    PAIR_LOG("[dm2] %s rx FAIL r%u: diff@%u\n", who, r, diff);
    rt_kprintf("[dm2]   rx =");
    for (i = 0; i < n; i++)
    {
        rt_kprintf(" %02x", rx[i]);
    }
    rt_kprintf("\n[dm2]   exp=");
    for (i = 0; i < n; i++)
    {
        rt_kprintf(" %02x", exp[i]);
    }
    rt_kprintf("\n");
}

/* vendor-demo per-case setup replica: RCC-reset both SPIs, vendor SPI_Init
 * (mode2, soft NSS, /256, 8-bit FD), then slave->master enable = cold order */
static void spi_dm2_demo_setup(rt_bool_t lsb)
{
    SPI_InitType st;

    SPI_I2S_DeInit(SPI1);
    SPI_I2S_DeInit(SPI2);
    SPI_InitStruct(&st);
    st.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    st.SpiMode       = SPI_MODE_MASTER;
    st.DataLen       = SPI_DATA_SIZE_8BITS;
    st.CLKPOL        = SPI_CLKPOL_HIGH;
    st.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
    st.NSS           = SPI_NSS_SOFT;
    st.BaudRatePres  = SPI_BR_PRESCALER_256;
    st.FirstBit      = lsb ? SPI_FB_LSB : SPI_FB_MSB;
    st.CRCPoly       = 7;
    SPI_Init(SPI1, &st);
    st.SpiMode = SPI_MODE_SLAVE;
    SPI_Init(SPI2, &st);
    spi_dm2_wire_probe("ini2");     /* engines off right after vendor init */
    spi_pair_reg_dump("p2ini", RT_TRUE);
    spi_pair_reg_dump("p2ini", RT_FALSE);
    SPI_Enable(SPI2, ENABLE);
    spi_dm2_wire_probe("slvon");
    SPI_Enable(SPI1, ENABLE);
    spi_dm2_wire_probe("bothon");
}

static int spi_dm2(int argc, char *argv[])
{
    struct spi_pair_ctx *ctx = &spi_pair_ctx;
    rt_uint32_t len = 512;
    rt_uint32_t rounds = 3;
    rt_uint32_t policy = 0;
    rt_uint32_t khz = SPI_PAIR_DEFAULT_KHZ;
    rt_uint32_t lsb = 0;
    rt_uint32_t r, i, ok = 0, t0, ms = 0;
    rt_bool_t fail = RT_FALSE;
    rt_err_t rc;

    if (argc >= 2) len = spi_pair_atoi(argv[1]);
    if (argc >= 3) rounds = spi_pair_atoi(argv[2]);
    if (argc >= 4) policy = spi_pair_atoi(argv[3]);
    if (argc >= 5) khz = spi_pair_atoi(argv[4]);
    if (argc >= 6) lsb = spi_pair_atoi(argv[5]);

    if (spi_pair_group_apply(argc, argv, 5) != RT_EOK) return -RT_ERROR;
    if ((spi_pair_cur_g->caps & SPI_PAIR_GRP_CAP_DM2) == 0u)
    {
        PAIR_LOG("[dm2] register-level probe only supported on %s\n",
                 spi_pair_groups[0].name);
        return -RT_ERROR;
    }
    if (spi_pair_running)
    {
        PAIR_LOG("[dm2] pair run active: stop it first\n");
        return -RT_ERROR;
    }
    if (policy > 2)
    {
        PAIR_LOG("[dm2] policy 0=warm 1=cold/round 2=demo-replica cold\n");
        return -RT_ERROR;
    }
    if (len < 1 || len > SPI_PAIR_LEN_MAX)
    {
        PAIR_LOG("[dm2] len 1..%u\n", SPI_PAIR_LEN_MAX);
        return -RT_ERROR;
    }
    if (khz < SPI_PAIR_SPEED_MIN_KHZ || khz > SPI_PAIR_SPEED_MAX_KHZ)
    {
        PAIR_LOG("[dm2] khz %u..%u\n", SPI_PAIR_SPEED_MIN_KHZ, SPI_PAIR_SPEED_MAX_KHZ);
        return -RT_ERROR;
    }

    spi_pair_ctx_reset(ctx);
    ctx->tag = "DM2";
    ctx->len = len;
    ctx->rounds = rounds;
    ctx->speed_hz = khz * 1000u;
    ctx->spi_mode = 2;
    ctx->lsb = (lsb != 0);
    if (spi_pair_attach(ctx) != RT_EOK ||
        spi_pair_config(ctx) != RT_EOK ||
        spi_pair_buf_prepare(ctx) != RT_EOK)
    {
        return -RT_ERROR;
    }

    PAIR_LOG("[dm2] demo-replica CPU per-byte mode2 len=%u rounds=%u"
             " policy=%u khz=%u 8bit %s\n", len, rounds, policy, khz,
             ctx->lsb ? "LSB" : "MSB");

    for (r = 0; r < rounds; r++)
    {
        if (policy == 2)
        {
            /* vendor-demo cold replica: per round RCC-reset + SPI_Init + slave
             * first enable; if this passes, the difference that matters is the
             * missing per-message reset/re-init, not the engine order */
            if (r == 0)
            {
                spi_dm2_wire_probe("cfg");      /* driver-config state */
            }
            GPIO_Configuration();               /* demo matrix preamble replica */
            spi_dm2_wire_probe("gpiore");
            spi_dm2_demo_setup(ctx->lsb);
        }
        else if (policy == 1)
        {
            /* driver per-message cold cycle: slave armed first, then master
             * enable; CPOL=1 pad rise lands on already-enabled slave */
            if (r == 0)
            {
                spi_dm2_wire_probe("cfg");      /* both engines off post-config */
            }
            spi_dm2_engine(RT_FALSE, RT_FALSE);
            spi_dm2_wire_probe("off");
            spi_dm2_engine(RT_FALSE, RT_TRUE);
            spi_dm2_wire_probe("slvon");        /* slave on, master off */
            spi_dm2_engine(RT_TRUE, RT_TRUE);
            spi_dm2_wire_probe("bothon");       /* master just enabled */
        }
        else if (r == 0)
        {
            /* demo: enable once at start, keep engines running all rounds */
            if (r == 0)
            {
                spi_dm2_wire_probe("cfg");      /* both engines off post-config */
            }
            spi_dm2_engine(RT_TRUE, RT_TRUE);
            spi_dm2_wire_probe("bothon");
        }
        if (((spi_pair_reg(RT_TRUE, 0x04) | spi_pair_reg(RT_FALSE, 0x04)) & 0x1u) == 0u)
        {
            PAIR_LOG("[dm2] r%u engines failed to enable!\n", r);
            spi_pair_reg_dump("dm2", RT_TRUE);
            spi_pair_reg_dump("dm2", RT_FALSE);
            fail = RT_TRUE;
            break;
        }

        spi_pair_gen(ctx->tx_a, len, r, ctx->salt_a, 8);
        spi_pair_gen(ctx->tx_b, len, r, ctx->salt_b, 8);
        rt_memset(ctx->rx_a, SPI_PAIR_RX_FILL, len);
        rt_memset(ctx->rx_b, SPI_PAIR_RX_FILL, len);

        t0 = rt_tick_get();
        rc = spi_dm2_run_bytes(ctx);
        ms = spi_pair_tick2ms(rt_tick_get() - t0);
        if (rc != RT_EOK)
        {
            PAIR_LOG("[dm2] r%u byte-loop abort rc=%d at ~%ums (engine stuck)\n",
                     r, rc, ms);
            spi_pair_reg_dump("dm2", RT_TRUE);
            spi_pair_reg_dump("dm2", RT_FALSE);
            fail = RT_TRUE;
            break;
        }
        for (i = 0; i < len; i++)  /* slave should have received master TX */
        {
            if (ctx->rx_b[i] != ctx->tx_a[i])
            {
                break;
            }
        }
        if (i < len)
        {
            spi_dm2_fail_dump("SLAVE", r, i, ctx->rx_b, ctx->tx_a, len);
            fail = RT_TRUE;
        }
        for (i = 0; i < len; i++)  /* master should have received slave TX */
        {
            if (ctx->rx_a[i] != ctx->tx_b[i])
            {
                break;
            }
        }
        if (i < len)
        {
            spi_dm2_fail_dump("MASTER", r, i, ctx->rx_a, ctx->tx_b, len);
            fail = RT_TRUE;
        }
        if (fail)
        {
            break;
        }
        ok++;
        PAIR_LOG("[dm2] r%u PASS (%ums)\n", r, ms);
        if (policy == 1)
        {
            /* driver message tail: slave engine off before master, so the
             * CPOL=1 pad drop of the master disable never lands on an
             * enabled slave (would tick its bit counter) */
            spi_dm2_engine(RT_TRUE, RT_FALSE);
            spi_dm2_engine(RT_FALSE, RT_FALSE);
            spi_dm2_wire_probe("tailoff");      /* master just disabled */
        }
    }

    spi_dm2_engine(RT_FALSE, RT_FALSE);
    spi_dm2_wire_probe("end");
    PAIR_LOG("[dm2] case done: len=%u rounds=%u policy=%u khz=%u ok=%u => %s\n",
             len, rounds, policy, khz, ok,
             (!fail && ok == rounds) ? "PASS" : "FAIL");
    if (fail)
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(spi_dm2, mode2 demo-replica control: spi_dm2 [len] [rounds] [policy0=warm/1=cold] [khz] [lsb0/1]);

#endif /* RT_USING_SPI && (SPI1+SPI2 or SPI3+SPI4 pair enabled) */
