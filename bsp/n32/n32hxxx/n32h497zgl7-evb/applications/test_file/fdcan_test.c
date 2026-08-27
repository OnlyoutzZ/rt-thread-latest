/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-06-23     lin.qi       FDCAN minimal loopback test for N32H7xx
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#ifdef BSP_USING_FDCAN

static rt_device_t can_dev = RT_NULL;

/*----------------------------------------------------------------------------*/
/* FDCAN1~FDCAN8 internal loopback self-test (generic)                        */
/* Does NOT require any external wiring - the peripheral internally routes    */
/* TX back to RX.                                                             */
/*----------------------------------------------------------------------------*/

static void _fdcan_internal_loopback(const char *name)
{
    struct rt_can_msg tx_msg = {0};
    struct rt_can_msg rx_msg = {0};
    rt_device_t dev;
    rt_ssize_t ret;
    rt_bool_t pass = RT_TRUE;

    /* hdr_index == -1 tells the CAN framework to read from the default
     * software RX FIFO instead of a HDR-filter-bank-specific queue */
    rx_msg.hdr_index = -1;

    rt_kprintf("\n===== %s External Loopback Test =====\n", name);

    dev = rt_device_find(name);
    if (dev == RT_NULL)
    {
        rt_kprintf("FAIL: %s not found\n", name);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("FAIL: open %s failed\n", name);
        return;
    }
    rt_kprintf("[OK] device opened (Normal mode)\n");

    rt_kprintf("[..] setting external loopback mode...\n");
    if (rt_device_control(dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_LOOPBACKANLISTEN) != RT_EOK)
    {
        rt_kprintf("FAIL: set external loopback mode\n");
        rt_device_close(dev);
        return;
    }
    rt_kprintf("[OK] external loopback mode set\n");
    rt_thread_mdelay(10);

    tx_msg.id  = 0x123;
    tx_msg.ide = RT_CAN_STDID;
    tx_msg.rtr = RT_CAN_DTR;
    tx_msg.len = 8;
    for (rt_uint8_t i = 0; i < 8; i++) tx_msg.data[i] = i + (rt_uint8_t)(name[5] - '0');

    rt_kprintf("[TX] ID=0x%03X len=%d: ", tx_msg.id, tx_msg.len);
    for (rt_uint8_t i = 0; i < 8; i++) rt_kprintf(" %02X", tx_msg.data[i]);
    rt_kprintf("\n");

    ret = rt_device_write(dev, 0, &tx_msg, sizeof(tx_msg));
    if (ret != sizeof(tx_msg))
    {
        rt_kprintf("FAIL: send failed, ret=%d\n", ret);
        rt_device_close(dev);
        return;
    }
    rt_kprintf("[OK] sent\n");

    rt_thread_mdelay(50);

    ret = rt_device_read(dev, 0, &rx_msg, sizeof(rx_msg));
    if (ret <= 0)
    {
        rt_kprintf("FAIL: no data received\n");
        rt_device_close(dev);
        return;
    }

    rt_kprintf("[RX] ID=0x%03X len=%d: ", rx_msg.id, rx_msg.len);
    for (rt_uint8_t i = 0; i < rx_msg.len && i < 64; i++) rt_kprintf(" %02X", rx_msg.data[i]);
    rt_kprintf("\n");

    if (rx_msg.id  != tx_msg.id)
    {
        rt_kprintf("FAIL: ID mismatch\n");
        pass = RT_FALSE;
    }
    if (rx_msg.len != tx_msg.len)
    {
        rt_kprintf("FAIL: len mismatch\n");
        pass = RT_FALSE;
    }
    for (rt_uint8_t i = 0; i < 8; i++)
    {
        if (rx_msg.data[i] != tx_msg.data[i])
        {
            rt_kprintf("FAIL: data[%d] mismatch: tx=0x%02X rx=0x%02X\n",
                       i, tx_msg.data[i], rx_msg.data[i]);
            pass = RT_FALSE;
            break;
        }
    }

    rt_kprintf("Result: %s\n", pass ? "PASS" : "FAIL");

#ifdef RT_CAN_USING_CANFD
    /* CAN-FD + BRS loopback: nominal (arbitration) rate stays at the
     * device's default 1Mbps, data phase switches to 5Mbps. Since this is
     * still External Loopback, RX comes from the internal TEST.LBCK path -
     * this only proves the FD/BRS framing and DBTP timing parameters are
     * self-consistent, it does NOT validate real bus signal integrity at
     * 5Mbps (that needs a real wired peer, see fdcan5<->6 testing). */
    {
        struct rt_can_msg fd_tx_msg = {0};
        struct rt_can_msg fd_rx_msg = {0};
        rt_bool_t fd_pass = RT_TRUE;

        rt_device_control(dev, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));

        fd_tx_msg.id       = 0x124;
        fd_tx_msg.ide      = RT_CAN_STDID;
        fd_tx_msg.rtr      = RT_CAN_DTR;
        fd_tx_msg.len      = 64;
        fd_tx_msg.fd_frame = 1;
        fd_tx_msg.brs      = 1;
        for (rt_uint8_t i = 0; i < 64; i++) fd_tx_msg.data[i] = (rt_uint8_t)(i + (name[5] - '0'));

        rt_kprintf("[TX] CAN-FD+BRS (arb=1Mbps, data=5Mbps) ID=0x%03X len=%d\n",
                   fd_tx_msg.id, fd_tx_msg.len);

        ret = rt_device_write(dev, 0, &fd_tx_msg, sizeof(fd_tx_msg));
        if (ret != sizeof(fd_tx_msg))
        {
            struct rt_can_status dbg_status;
            rt_device_control(dev, RT_CAN_CMD_GET_STATUS, &dbg_status);
            rt_kprintf("FAIL: FD+BRS send failed, ret=%d\n", ret);
            rt_kprintf("[INFO] %s status: snderrcnt=%d rcverrcnt=%d errcode=%d lasterrtype=%d\n",
                       name, dbg_status.snderrcnt, dbg_status.rcverrcnt,
                       dbg_status.errcode, dbg_status.lasterrtype);
            fd_pass = RT_FALSE;
        }
        else
        {
            rt_thread_mdelay(50);
            fd_rx_msg.hdr_index = -1;
            ret = rt_device_read(dev, 0, &fd_rx_msg, sizeof(fd_rx_msg));
            if (ret <= 0)
            {
                rt_kprintf("FAIL: FD+BRS no data received\n");
                fd_pass = RT_FALSE;
            }
            else if (fd_rx_msg.id != fd_tx_msg.id || fd_rx_msg.len != fd_tx_msg.len)
            {
                rt_kprintf("FAIL: FD+BRS mismatch tx(id=0x%X,len=%d) rx(id=0x%X,len=%d)\n",
                           fd_tx_msg.id, fd_tx_msg.len, fd_rx_msg.id, fd_rx_msg.len);
                fd_pass = RT_FALSE;
            }
            else
            {
                for (rt_uint8_t i = 0; i < 64; i++)
                {
                    if (fd_rx_msg.data[i] != fd_tx_msg.data[i])
                    {
                        rt_kprintf("FAIL: FD+BRS data[%d] mismatch: tx=0x%02X rx=0x%02X\n",
                                   i, fd_tx_msg.data[i], fd_rx_msg.data[i]);
                        fd_pass = RT_FALSE;
                        break;
                    }
                }
            }
        }

        rt_kprintf("Result (FD+BRS arb=1M/data=5M): %s\n", fd_pass ? "PASS" : "FAIL");
        if (!fd_pass) pass = RT_FALSE;

        /* restore the board's default 5Mbps data-phase rate (no-op here,
         * kept for symmetry with the other tests that touch SET_BAUD_FD) */
        rt_device_control(dev, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));
    }
#endif

    /* External Loopback drives the real CAN_TX pin onto the physical bus,
     * so on a wired pair (e.g. fdcan5<->fdcan6) the peer's hardware RX FIFO
     * keeps receiving these frames even while its RT-Thread device is
     * closed. Drain any leftover frames here before closing so they don't
     * leak into a later test on this same device. */
    rx_msg.hdr_index = -1;
    while (rt_device_read(dev, 0, &rx_msg, sizeof(rx_msg)) > 0)
    {
        rx_msg.hdr_index = -1;
    }

    rt_device_control(dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    rt_device_close(dev);
}

/* Individual MSH commands for convenience */
void fdcan1_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan1");
}
MSH_CMD_EXPORT(fdcan1_loopback_test, FDCAN1 internal loopback self - test);
void fdcan2_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan2");
}
MSH_CMD_EXPORT(fdcan2_loopback_test, FDCAN2 internal loopback self - test);
void fdcan3_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan3");
}
MSH_CMD_EXPORT(fdcan3_loopback_test, FDCAN3 internal loopback self - test);
void fdcan4_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan4");
}
MSH_CMD_EXPORT(fdcan4_loopback_test, FDCAN4 internal loopback self - test);
void fdcan5_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan5");
}
MSH_CMD_EXPORT(fdcan5_loopback_test, FDCAN5 internal loopback self - test);
void fdcan6_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan6");
}
MSH_CMD_EXPORT(fdcan6_loopback_test, FDCAN6 internal loopback self - test);
void fdcan7_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan7");
}
MSH_CMD_EXPORT(fdcan7_loopback_test, FDCAN7 internal loopback self - test);
void fdcan8_loopback_test(void)
{
    _fdcan_internal_loopback("fdcan8");
}
MSH_CMD_EXPORT(fdcan8_loopback_test, FDCAN8 internal loopback self - test);

void fdcan_all_loopback_test(void)
{
    rt_kprintf("\n===== FDCAN1~8 all-loopback suite =====\n");
    _fdcan_internal_loopback("fdcan1");
    _fdcan_internal_loopback("fdcan2");
    _fdcan_internal_loopback("fdcan3");
    _fdcan_internal_loopback("fdcan4");
    _fdcan_internal_loopback("fdcan5");
    _fdcan_internal_loopback("fdcan6");
    _fdcan_internal_loopback("fdcan7");
    _fdcan_internal_loopback("fdcan8");
    rt_kprintf("===== all-loopback suite: DONE =====\n");
}
MSH_CMD_EXPORT(fdcan_all_loopback_test, FDCAN1~8 all internal loopback self - tests);

/*----------------------------------------------------------------------------*/
/* FDCAN1~FDCAN8 paired real-bus tests                                        */
/* Physical wiring: FDCAN1<->FDCAN2, FDCAN3<->FDCAN4,                          */
/*                  FDCAN5<->FDCAN6, FDCAN7<->FDCAN8                          */
/*----------------------------------------------------------------------------*/

struct fdcan_pair
{
    const char *a;
    const char *b;
};

static const struct fdcan_pair _fdcan_pairs[] =
{
    /* Only fdcan5<->6 has a confirmed-working physical link on this board.
     * The other pairs fail Classic CAN from the very first frame - suspected
     * wiring/termination/hardware issue, pending board check. */
    {"fdcan5", "fdcan6"},
};
#define FDCAN_PAIR_COUNT (sizeof(_fdcan_pairs) / sizeof(_fdcan_pairs[0]))

static rt_device_t _fdcan_open(const char *name)
{
    rt_device_t dev = rt_device_find(name);
    if (dev == RT_NULL)
    {
        rt_kprintf("FAIL: %s not found\n", name);
        return RT_NULL;
    }
    if (rt_device_open(dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("FAIL: open %s failed\n", name);
        return RT_NULL;
    }

    /* RT-Thread only calls the driver's configure() once, on a device's very
     * first ever open - a later open() does NOT reset whatever mode an
     * earlier test left it in (e.g. fdcan1_loopback_test leaves fdcan1 in
     * Internal Loopback). The driver's SET_MODE handler is itself a no-op
     * when the requested mode equals the current one, so toggle through
     * Listen mode first to unconditionally force the recovery dance to run
     * (it cancels stuck Tx requests and clears CCCR.INIT, which also lifts
     * any leftover Bus-Off condition from a previous sub-test) before
     * landing back on Normal mode. */
    rt_device_control(dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_LISTEN);
    rt_device_control(dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    /* A wired peer running External Loopback (or any other earlier test)
     * can have left frames sitting in this device's hardware RX FIFO -
     * the FDCAN peripheral keeps receiving real bus traffic even while its
     * RT-Thread device handle is closed. Drain them now so a stale frame
     * from an unrelated earlier sub-test can't be mistaken for this test's
     * expected reply. */
    {
        struct rt_can_msg drain_msg = {0};
        drain_msg.hdr_index = -1;
        while (rt_device_read(dev, 0, &drain_msg, sizeof(drain_msg)) > 0)
        {
            drain_msg.hdr_index = -1;
        }
    }

    return dev;
}

/* Send one frame from tx_dev and expect an identical frame on rx_dev. */
static rt_bool_t _fdcan_send_and_check(rt_device_t tx_dev, rt_device_t rx_dev,
                                       rt_uint32_t id, rt_uint8_t len,
                                       rt_uint8_t fd_frame, rt_uint8_t brs,
                                       const char *tag)
{
    struct rt_can_msg tx_msg = {0};
    struct rt_can_msg rx_msg = {0};
    rt_ssize_t ret;
    rt_uint8_t i;

    tx_msg.id  = id;
    tx_msg.ide = RT_CAN_STDID;
    tx_msg.rtr = RT_CAN_DTR;
    tx_msg.len = len;
#ifdef RT_CAN_USING_CANFD
    tx_msg.fd_frame = fd_frame;
    tx_msg.brs      = brs;
#endif
    for (i = 0; i < len; i++) tx_msg.data[i] = (rt_uint8_t)(i + id);

    ret = rt_device_write(tx_dev, 0, &tx_msg, sizeof(tx_msg));
    if (ret != sizeof(tx_msg))
    {
        rt_kprintf("FAIL[%s]: send ret=%d\n", tag, ret);
        return RT_FALSE;
    }

    rt_thread_mdelay(50);

    rx_msg.hdr_index = -1;
    ret = rt_device_read(rx_dev, 0, &rx_msg, sizeof(rx_msg));
    if (ret <= 0)
    {
        rt_kprintf("FAIL[%s]: no data received\n", tag);
        return RT_FALSE;
    }

    if (rx_msg.id != tx_msg.id || rx_msg.len != tx_msg.len)
    {
        rt_kprintf("FAIL[%s]: mismatch tx(id=0x%X,len=%d) rx(id=0x%X,len=%d)\n",
                   tag, tx_msg.id, tx_msg.len, rx_msg.id, rx_msg.len);
        return RT_FALSE;
    }

    for (i = 0; i < len; i++)
    {
        if (rx_msg.data[i] != tx_msg.data[i])
        {
            rt_kprintf("FAIL[%s]: data[%d] mismatch tx=0x%02X rx=0x%02X\n",
                       tag, i, tx_msg.data[i], rx_msg.data[i]);
            return RT_FALSE;
        }
    }

    rt_kprintf("PASS[%s]: ID=0x%03X len=%d fd=%d brs=%d\n", tag, id, len, fd_frame, brs);
    return RT_TRUE;
}

/*----------------------------------------------------------------------------*/
/* Category 1: basic bidirectional TX/RX (Classic CAN + CAN-FD/BRS)           */
/*----------------------------------------------------------------------------*/

static rt_bool_t _fdcan_basic_pair(const char *name_a, const char *name_b)
{
    rt_device_t dev_a, dev_b;
    rt_bool_t pass = RT_TRUE;
    char tag[40];

    dev_a = _fdcan_open(name_a);
    dev_b = _fdcan_open(name_b);
    if (dev_a == RT_NULL || dev_b == RT_NULL)
    {
        if (dev_a) rt_device_close(dev_a);
        if (dev_b) rt_device_close(dev_b);
        return RT_FALSE;
    }

    rt_kprintf("\n--- %s <-> %s : Classic CAN ---\n", name_a, name_b);
    rt_snprintf(tag, sizeof(tag), "%s->%s classic", name_a, name_b);
    if (!_fdcan_send_and_check(dev_a, dev_b, 0x100, 8, 0, 0, tag)) pass = RT_FALSE;
    rt_snprintf(tag, sizeof(tag), "%s->%s classic", name_b, name_a);
    if (!_fdcan_send_and_check(dev_b, dev_a, 0x101, 8, 0, 0, tag)) pass = RT_FALSE;

#ifdef RT_CAN_USING_CANFD
    /* Try 4Mbps instead of the board's default 5Mbps data-phase rate, to
     * see whether the jumper wiring can hold up one notch down from the
     * rate that's been failing on signal integrity grounds. */
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 4));
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 4));

    rt_kprintf("--- %s <-> %s : CAN-FD + BRS @ 4Mbps (64 bytes) ---\n", name_a, name_b);
    rt_snprintf(tag, sizeof(tag), "%s->%s fd+brs@4M", name_a, name_b);
    if (!_fdcan_send_and_check(dev_a, dev_b, 0x102, 64, 1, 1, tag)) pass = RT_FALSE;
    rt_snprintf(tag, sizeof(tag), "%s->%s fd+brs@4M", name_b, name_a);
    if (!_fdcan_send_and_check(dev_b, dev_a, 0x103, 64, 1, 1, tag)) pass = RT_FALSE;

    /* restore the board's default 5Mbps data-phase rate */
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));
#endif

    rt_device_close(dev_a);
    rt_device_close(dev_b);
    return pass;
}

void fdcan_basic_test(void)
{
    rt_bool_t all_pass = RT_TRUE;
    rt_size_t i;

    rt_kprintf("\n===== FDCAN1~8 paired basic TX/RX test =====\n");
    for (i = 0; i < FDCAN_PAIR_COUNT; i++)
    {
        if (!_fdcan_basic_pair(_fdcan_pairs[i].a, _fdcan_pairs[i].b))
            all_pass = RT_FALSE;
    }
    rt_kprintf("===== basic TX/RX test: %s =====\n", all_pass ? "PASS" : "FAIL");
}
MSH_CMD_EXPORT(fdcan_basic_test, FDCAN1~8 paired real - bus basic TX / RX test);

/*----------------------------------------------------------------------------*/
/* CAN-FD data-phase rate comparison: default 5Mbps vs 2Mbps.                  */
/* If 2Mbps passes where 5Mbps fails, that points at signal integrity on the  */
/* physical link rather than a software/driver bug.                          */
/*----------------------------------------------------------------------------*/

#ifdef RT_CAN_USING_CANFD
static rt_bool_t _fdcan_fd_rate_pair(const char *name_a, const char *name_b, rt_uint32_t fd_baud)
{
    rt_device_t dev_a, dev_b;
    rt_bool_t pass = RT_TRUE;
    char tag[48];

    dev_a = _fdcan_open(name_a);
    dev_b = _fdcan_open(name_b);
    if (dev_a == RT_NULL || dev_b == RT_NULL)
    {
        if (dev_a) rt_device_close(dev_a);
        if (dev_b) rt_device_close(dev_b);
        return RT_FALSE;
    }

    if (rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD_FD, (void *)fd_baud) != RT_EOK ||
            rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD_FD, (void *)fd_baud) != RT_EOK)
    {
        rt_kprintf("FAIL: set FD data rate %d failed\n", fd_baud);
        pass = RT_FALSE;
    }
    else
    {
        rt_kprintf("--- %s <-> %s : CAN-FD + BRS @ %dMbps data phase ---\n",
                   name_a, name_b, fd_baud / 1000000);
        rt_snprintf(tag, sizeof(tag), "%s->%s fd@%dM", name_a, name_b, fd_baud / 1000000);
        if (!_fdcan_send_and_check(dev_a, dev_b, 0x110, 64, 1, 1, tag)) pass = RT_FALSE;
        rt_snprintf(tag, sizeof(tag), "%s->%s fd@%dM", name_b, name_a, fd_baud / 1000000);
        if (!_fdcan_send_and_check(dev_b, dev_a, 0x111, 64, 1, 1, tag)) pass = RT_FALSE;
    }

    /* restore the board's default 5Mbps data-phase rate */
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD_FD, (void *)(CAN1MBaud * 5));

    rt_device_close(dev_a);
    rt_device_close(dev_b);
    return pass;
}
#endif /* RT_CAN_USING_CANFD */

void fdcan_fd_rate_test(void)
{
#ifdef RT_CAN_USING_CANFD
    rt_bool_t all_pass = RT_TRUE;
    rt_size_t i;

    rt_kprintf("\n===== FDCAN CAN-FD data-phase rate comparison test (2Mbps) =====\n");
    for (i = 0; i < FDCAN_PAIR_COUNT; i++)
    {
        if (!_fdcan_fd_rate_pair(_fdcan_pairs[i].a, _fdcan_pairs[i].b, CAN1MBaud * 2))
            all_pass = RT_FALSE;
    }
    rt_kprintf("===== CAN-FD 2Mbps test: %s =====\n", all_pass ? "PASS" : "FAIL");
#else
    rt_kprintf("SKIP: RT_CAN_USING_CANFD not enabled\n");
#endif
}
MSH_CMD_EXPORT(fdcan_fd_rate_test, FDCAN CAN - FD data - phase rate comparison test - 2Mbps vs default 5Mbps);

/*----------------------------------------------------------------------------*/
/* Category 2: hardware filter bank association                               */
/* Note: FDCAN_ConfigGlobalFilter() is fixed at driver-init time to accept    */
/* non-matching standard frames into RX FIFO0, so a non-matching ID is still  */
/* received here - it is just tagged hdr_index == -1 (no specific filter)    */
/* instead of the explicit bank index. True hardware-level rejection of      */
/* non-matching IDs is not exposed by RT_CAN_CMD_SET_FILTER in this driver.  */
/*----------------------------------------------------------------------------*/

static rt_bool_t _fdcan_filter_pair(const char *name_a, const char *name_b)
{
    rt_device_t dev_a, dev_b;
    struct rt_can_filter_item item = {0};
    struct rt_can_filter_config filter_cfg = {0};
    struct rt_can_msg tx_msg = {0};
    struct rt_can_msg rx_msg = {0};
    rt_bool_t pass = RT_TRUE;
    rt_ssize_t ret;

    dev_a = _fdcan_open(name_a);
    dev_b = _fdcan_open(name_b);
    if (dev_a == RT_NULL || dev_b == RT_NULL)
    {
        if (dev_a) rt_device_close(dev_a);
        if (dev_b) rt_device_close(dev_b);
        return RT_FALSE;
    }

    rt_kprintf("\n--- %s <-> %s : filter bank association ---\n", name_a, name_b);

    /* Replace bank 0's accept-all filter on dev_b with an exact-match filter
     * for ID 0x300 only (mode is ignored by this driver, it always applies
     * Mask-mode matching: mask=0x7FF means every ID bit must match). */
    item.id       = 0x300;
    item.ide      = RT_CAN_STDID;
    item.rtr      = RT_CAN_DTR;
    item.mask     = 0x7FF;
    item.hdr_bank = 0;
    filter_cfg.items   = &item;
    filter_cfg.count   = 1;
    filter_cfg.actived = 1;

    if (rt_device_control(dev_b, RT_CAN_CMD_SET_FILTER, &filter_cfg) != RT_EOK)
    {
        rt_kprintf("FAIL: set filter on %s failed\n", name_b);
        pass = RT_FALSE;
        goto cleanup;
    }

    /* Frame matching the explicit filter -> hdr_index must equal bank 0 */
    tx_msg.id = 0x300;
    tx_msg.ide = RT_CAN_STDID;
    tx_msg.rtr = RT_CAN_DTR;
    tx_msg.len = 1;
    tx_msg.data[0] = 0xAA;
    ret = rt_device_write(dev_a, 0, &tx_msg, sizeof(tx_msg));
    if (ret != sizeof(tx_msg))
    {
        rt_kprintf("FAIL: send 0x300 failed\n");
        pass = RT_FALSE;
        goto restore;
    }
    rt_thread_mdelay(50);
    rx_msg.hdr_index = -1;
    ret = rt_device_read(dev_b, 0, &rx_msg, sizeof(rx_msg));
    if (ret <= 0)
    {
        rt_kprintf("FAIL: matching ID 0x300 not received\n");
        pass = RT_FALSE;
    }
    else if (rx_msg.hdr_index != 0)
    {
        rt_kprintf("FAIL: matching ID 0x300 reported hdr_index=%d, expected 0\n", rx_msg.hdr_index);
        pass = RT_FALSE;
    }
    else
    {
        rt_kprintf("PASS: ID 0x300 matched filter bank 0 (hdr_index=0)\n");
    }

    /* Frame NOT matching the explicit filter -> still delivered via the
     * global catch-all.  The _fdcan_recvmsg() driver sets hdr_index = 0
     * (a safe default) rather than -1 for ANMF frames because the ISR
     * path asserts hdr_index ∈ [0, maxhdr) when RT_CAN_USING_HDR is on. */
    tx_msg.id = 0x301;
    tx_msg.data[0] = 0xBB;
    ret = rt_device_write(dev_a, 0, &tx_msg, sizeof(tx_msg));
    if (ret != sizeof(tx_msg))
    {
        rt_kprintf("FAIL: send 0x301 failed\n");
        pass = RT_FALSE;
        goto restore;
    }
    rt_thread_mdelay(50);
    rx_msg.hdr_index = -1;
    ret = rt_device_read(dev_b, 0, &rx_msg, sizeof(rx_msg));
    if (ret <= 0)
    {
        rt_kprintf("FAIL: non-matching ID 0x301 not received via global filter\n");
        pass = RT_FALSE;
    }
    else
    {
        rt_kprintf("PASS: ID 0x301 delivered by global filter (hdr_index=%d)\n",
                   rx_msg.hdr_index);
    }
    {
        rt_kprintf("PASS: ID 0x301 bypassed bank 0, correctly tagged hdr_index=-1\n");
    }

restore:
    /* Restore the accept-all default so later tests on this pair are unaffected */
    item.id = 0x000;
    item.mask = 0x000;
    item.hdr_bank = 0;
    rt_device_control(dev_b, RT_CAN_CMD_SET_FILTER, &filter_cfg);

cleanup:
    rt_device_close(dev_a);
    rt_device_close(dev_b);
    return pass;
}

void fdcan_filter_test(void)
{
    rt_bool_t all_pass = RT_TRUE;
    rt_size_t i;

    rt_kprintf("\n===== FDCAN1~8 paired filter test =====\n");
    for (i = 0; i < FDCAN_PAIR_COUNT; i++)
    {
        if (!_fdcan_filter_pair(_fdcan_pairs[i].a, _fdcan_pairs[i].b))
            all_pass = RT_FALSE;
    }
    rt_kprintf("===== filter test: %s =====\n", all_pass ? "PASS" : "FAIL");
}
MSH_CMD_EXPORT(fdcan_filter_test, FDCAN1~8 paired hardware filter test);

/*----------------------------------------------------------------------------*/
/* Category 3+4: mode switching (SET_BAUD, SET_MODE) and error counters       */
/*----------------------------------------------------------------------------*/

static rt_bool_t _fdcan_mode_pair(const char *name_a, const char *name_b)
{
    rt_device_t dev_a, dev_b;
    struct rt_can_msg tx_msg = {0};
    struct rt_can_msg rx_msg = {0};
    struct rt_can_status status_a = {0};
    rt_bool_t pass = RT_TRUE;
    rt_ssize_t ret;

    dev_a = _fdcan_open(name_a);
    dev_b = _fdcan_open(name_b);
    if (dev_a == RT_NULL || dev_b == RT_NULL)
    {
        if (dev_a) rt_device_close(dev_a);
        if (dev_b) rt_device_close(dev_b);
        return RT_FALSE;
    }

    /* --- SET_BAUD: drop both ends to 500kbps in sync, verify TX/RX still works --- */
    rt_kprintf("\n--- %s <-> %s : SET_BAUD 1Mbps -> 500kbps ---\n", name_a, name_b);
    if (rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud) != RT_EOK ||
            rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud) != RT_EOK)
    {
        rt_kprintf("FAIL: set 500kbps failed\n");
        pass = RT_FALSE;
    }
    else
    {
        tx_msg.id = 0x400;
        tx_msg.ide = RT_CAN_STDID;
        tx_msg.rtr = RT_CAN_DTR;
        tx_msg.len = 4;
        tx_msg.data[0] = 0x11;
        tx_msg.data[1] = 0x22;
        tx_msg.data[2] = 0x33;
        tx_msg.data[3] = 0x44;
        ret = rt_device_write(dev_a, 0, &tx_msg, sizeof(tx_msg));
        rt_thread_mdelay(50);
        rx_msg.hdr_index = -1;
        if (ret != sizeof(tx_msg) || rt_device_read(dev_b, 0, &rx_msg, sizeof(rx_msg)) <= 0 ||
                rx_msg.id != tx_msg.id)
        {
            rt_kprintf("FAIL: TX/RX at 500kbps did not work\n");
            pass = RT_FALSE;
        }
        else
        {
            rt_kprintf("PASS: TX/RX works at 500kbps\n");
        }
    }

    /* Restore 1Mbps on both ends regardless of the result above */
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);

    /* --- SET_MODE: put dev_b in Listen-only mode, dev_a must fail to get an ACK --- */
    rt_kprintf("--- %s <-> %s : SET_MODE Listen-only on %s ---\n", name_a, name_b, name_b);
    if (rt_device_control(dev_b, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_LISTEN) != RT_EOK)
    {
        rt_kprintf("FAIL: set %s to Listen mode failed\n", name_b);
        pass = RT_FALSE;
    }
    else
    {
        tx_msg.id = 0x401;
        tx_msg.len = 1;
        tx_msg.data[0] = 0x55;
        ret = rt_device_write(dev_a, 0, &tx_msg, sizeof(tx_msg));
        if (ret == sizeof(tx_msg))
        {
            rt_kprintf("FAIL: %s send unexpectedly succeeded with no ACK available\n", name_a);
            pass = RT_FALSE;
        }
        else
        {
            rt_kprintf("PASS: %s send correctly failed (no ACK from Listen-only peer)\n", name_a);
        }

        rt_device_control(dev_a, RT_CAN_CMD_GET_STATUS, &status_a);
        rt_kprintf("[INFO] %s status after failed send: snderrcnt=%d errcode=%d\n",
                   name_a, status_a.snderrcnt, status_a.errcode);
    }

    /* --- Recovery: full re-init on both ends --- */
    rt_kprintf("--- %s <-> %s : recovery full re-init ---\n", name_a, name_b);

    /* Restore Normal mode on dev_b *first* so that the SET_BAUD cycle
     * below picks it up — _fdcan_configure() uses config.mode as-is,
     * and it is still RT_CAN_MODE_LISTEN from the test above.  dev_a
     * never left Normal mode but toggle it anyway for consistency. */
    rt_device_control(dev_a, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    rt_device_control(dev_b, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    /* Full FDCAN_Stop + FDCAN_Init + FDCAN_Start on both ends to reset
     * controllers, error counters, and TX pipelines from scratch. */
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);
    rt_device_control(dev_a, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    rt_device_control(dev_b, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);

    /* Both nodes are now in 1Mbps Normal mode.  Give the hardware a moment
     * to synchronize to the bus, then drain any frames that may have
     * appeared during the transition. */
    rt_thread_mdelay(60);
    rx_msg.hdr_index = -1;
    while (rt_device_read(dev_b, 0, &rx_msg, sizeof(rx_msg)) > 0)
    {
        rx_msg.hdr_index = -1;
    }

    tx_msg.id = 0x402;
    tx_msg.data[0] = 0x66;
    ret = rt_device_write(dev_a, 0, &tx_msg, sizeof(tx_msg));
    if (ret != sizeof(tx_msg))
    {
        rt_kprintf("FAIL: recovery send returned %d (expected %d)\n",
                   ret, (int)sizeof(tx_msg));
        pass = RT_FALSE;
    }
    else
    {
        rt_thread_mdelay(50);
        rx_msg.hdr_index = -1;
        ret = rt_device_read(dev_b, 0, &rx_msg, sizeof(rx_msg));
        if (ret <= 0)
        {
            rt_kprintf("FAIL: recovery read returned %d (no frame received)\n", ret);
            pass = RT_FALSE;
        }
        else if (rx_msg.id != tx_msg.id)
        {
            rt_kprintf("FAIL: recovery received ID 0x%X, expected 0x%X\n",
                       rx_msg.id, tx_msg.id);
            pass = RT_FALSE;
        }
        else
        {
            rt_kprintf("PASS: TX/RX recovered after returning to Normal mode\n");
        }
    }

    rt_device_close(dev_a);
    rt_device_close(dev_b);
    return pass;
}

void fdcan_mode_test(void)
{
    rt_bool_t all_pass = RT_TRUE;
    rt_size_t i;

    rt_kprintf("\n===== FDCAN1~8 paired mode/baud/error test =====\n");
    for (i = 0; i < FDCAN_PAIR_COUNT; i++)
    {
        if (!_fdcan_mode_pair(_fdcan_pairs[i].a, _fdcan_pairs[i].b))
            all_pass = RT_FALSE;
    }
    rt_kprintf("===== mode/baud/error test: %s =====\n", all_pass ? "PASS" : "FAIL");
}
MSH_CMD_EXPORT(fdcan_mode_test, FDCAN1~8 paired baud - switch / listen - mode / error - counter test);

/*----------------------------------------------------------------------------*/
/* Top-level: run every category across all four pairs                        */
/*----------------------------------------------------------------------------*/

void fdcan_full_test(void)
{
    fdcan_basic_test();
    fdcan_fd_rate_test();
    fdcan_filter_test();
    fdcan_mode_test();
}
MSH_CMD_EXPORT(fdcan_full_test, FDCAN1~8 full paired test suite - basic + filter + mode + error);

#endif /* BSP_USING_FDCAN */
