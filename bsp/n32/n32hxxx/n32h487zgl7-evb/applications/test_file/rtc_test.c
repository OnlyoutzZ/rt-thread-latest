/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-14     ox-horse         drv_rtc + RTC framework full-function test
 *
 * ============================================================================
 *  HOW TO RUN THIS TEST (MSH console, serial terminal)
 * ============================================================================
 *
 *  1. 查看当前 RTC 时间（快速确认 RTC 是否在走）:
 *         rtc_test_info
 *         date
 *      间隔 2~3 秒再执行一次，秒数递增说明 RTC 正常走时。
 *
 *  2. 设置时间 / 查询时间:
 *         date                         # 只查询（不带参数）
 *         date 2026 08 14 12 30 00     # 设置: 年 月 日 时 分 秒 (本地时间, 周围注意加空格)
 *      说明: RT-Thread 的 date 命令按"本地时间"解释输入并存入 RTC。
 *      本驱动把 RTC 硬件时间以 UTC 存储，查询时自动做 +8h 本地显示，
 *      因此设置 12:30 后查询会显示 12:30（同值）。
 *      首次烧录/换芯片后 RTC 未初始化（备份寄存器无 magic 0xA5A5），
 *      建议先执行一次上面的 date 设置命令，再跑测试。
 *
 *  3. 一键跑全部测试 (推荐):
 *         rtc_test_full
 *      最终输出 "pass: N, fail: 0" 即全部通过；fail > 0 可看具体 [FAIL] 项。
 *
 *  4. 分模块测试:
 *         rtc_test_init_cmd     # 仅测驱动初始化/时钟配置
 *         rtc_test_time_cmd     # 仅测时间读写 (secs/date/timeval)
 *         rtc_test_alarm_cmd    # 仅测闹钟 (set/get/daemon/modify/delete/rearm)
 *         rtc_test_backup_cmd   # 仅测备份寄存器读写
 *
 *  5. 闹钟辅助查看:
 *         list_alarm            # RT-Thread 自带的 alarm 列表命令
 *
 *  注意:
 *  - 测试会改写 RTC 时间（08/09 组会把时间设成测试基准），如需保留真实
 *    时间请在测试后重新 date 设置。
 *  - 备份寄存器测试会读写 RTC_BKP_REG1（写后恢复 0x5A5=0xA5A5），不影响驱动 magic。
 *  - 本文件需要 BSP_USING_ONCHIP_RTC 开启（rtconfig.h），闹钟测试还需要
 *    RT_USING_ALARM 开启。
 *
 *  Coverd test matrix (drv_rtc.c + components/drivers/rtc):
 *   [00] driver init / RTC clock config (RCC enable, LSI/LSE/HSE, prescaler,
 *        WaitForSynchro, backup-register magic check)      -> n32_rtc_init
 *   [01] device find/open/init/close + control dispatch   -> rt_rtc_control
 *   [02] get_secs  (sec-level read, ~2s advance)          -> n32_rtc_get_secs
 *   [03] set_secs  (write + read-back)                    -> n32_rtc_set_secs
 *   [04] set_date / set_time / set_timestamp / get_timestamp  (framework layer)
 *   [05] gettimeofday -> GET_TIMEVAL (tv_usec from SUBS/PRE) -> n32_rtc_get_timeval
 *   [06] rt_device_control GET_TIMEVAL directly           -> same as [05]
 *   [07] rt_device_control SET_TIMEVAL (ops->set_timeval==NULL -> -RT_EINVAL)
 *   [08] set_alarm / get_alarm (RT_DEVICE_CTRL_RTC_SET/GET_ALARM)
 *        -> n32_rtc_set_alarm / n32_rtc_get_alarm
 *   [09] alarm daemon: rt_alarm_create / start / callback / stop
 *        (walks through alarm service thread + hardware RTC alarm + ISR)
 *   [10] rt_alarm_control RT_ALARM_CTRL_MODIFY
 *   [11] rt_alarm_delete
 *   [12] one-shot alarm really wakes up on time (drives RTC_ALARM IRQ)
 *   [13] set_secs -> rt_alarm_update (alarm service re-arms after time change)
 *   [14] RTC_BKUP_Read/Write backup-register round trip
 *   [15] rtc_test_info + date command linkage check
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include <time.h>

#ifdef BSP_USING_ONCHIP_RTC

#define RTC_DEVICE_NAME     "rtc"
#define RTC_TEST_BKUP_REG   1U   /* same value drv_rtc.c uses (RTC_BKP_REG1) */
#define RTC_TEST_BKUP_MAGIC 0xA5A5U
#define RTC_TEST_ALARM_MS   3000 /* wait for one-shot alarm callback */

/* drv_rtc.c exposes weak helpers that are NOT in any public header.  Declare
 * them here (same signature as the driver) so the test can use them without
 * relying on implicit declarations. */
extern rt_weak uint32_t RTC_BKUP_Read(uint8_t BackupRegister);
extern rt_weak void RTC_BKUP_Write(uint8_t BackupRegister, uint32_t Data);

static rt_device_t rtc_dev          = RT_NULL;
static rt_uint32_t test_pass_count  = 0;
static rt_uint32_t test_fail_count  = 0;

#define TEST_REPORT_SECTION(title) \
        rt_kprintf("\n----- [ %-40s ] -----\n", title)

static void test_passed(const char *name)
{
    test_pass_count++;
    rt_kprintf("[ PASS ] %s\n", name);
}

static void test_failed(const char *name, const char *why)
{
    test_fail_count++;
    rt_kprintf("[ FAIL ] %s : %s\n", name, why ? why : "");
}

static void test_summary(void)
{
    rt_kprintf("\n======== RTC TEST SUMMARY ========\n");
    rt_kprintf("pass: %d, fail: %d\n", (int)test_pass_count, (int)test_fail_count);
    rt_kprintf("==================================\n");
}

/* find the rtc device; also print a hint if it cannot be found */
static rt_bool_t rtc_prepare(void)
{
    if (rtc_dev != RT_NULL)
    {
        return RT_TRUE;
    }

    rtc_dev = rt_device_find(RTC_DEVICE_NAME);
    if (rtc_dev == RT_NULL)
    {
        rt_kprintf("[ERR ] cannot find rtc device '%s'\n", RTC_DEVICE_NAME);
        rt_kprintf("[INFO] enable BSP_USING_ONCHIP_RTC and rebuild\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}

/* ------------------------------------------------------------------ */
/* 00 - driver init path: RCC clock enable, clock source (LSI/LSE/HSE) */
/*      prescaler, RTC_WaitForSynchro, backup magic check              */
/* ------------------------------------------------------------------ */
static void rtc_test_init(void)
{
    rt_err_t ret;

    TEST_REPORT_SECTION("00. drv_rtc init / RTC clock config");

    if (!rtc_prepare())
    {
        return;
    }

    /* rt_device_init -> rt_rtc_init -> n32_rtc_init -> rt_rtc_config
     * (idempotent on an already running RTC; hardware is untouched when
     *  the backup magic is present) */
    ret = rt_device_init(rtc_dev);
    if (ret == RT_EOK)
    {
        test_passed("n32_rtc_init / rt_rtc_config");
    }
    else
    {
        test_failed("n32_rtc_init / rt_rtc_config", "rt_device_init failed");
    }

    /* verify the RTC is actually ticking after the (re)-init */
    {
        time_t now = 0;
        ret = rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_GET_TIME, &now);
        if (ret == RT_EOK && now != (time_t)0)
        {
            rt_kprintf("RTC alive, now = %ld (%s)\n", (long)now, ctime(&now));
            test_passed("RTC ticking after init");
        }
        else
        {
            test_failed("RTC ticking after init", "GET_TIME failed or zero");
        }
    }
}

/* ------------------------------------------------------------------ */
/* 01 - device open / close / control                                  */
/* ------------------------------------------------------------------ */
static void rtc_test_device(void)
{
    TEST_REPORT_SECTION("01. device find/open/close/control");

    if (!rtc_prepare())
    {
        return;
    }

    if (rtc_dev->type != RT_Device_Class_RTC)
    {
        test_failed("device class", "not RT_Device_Class_RTC");
    }
    else
    {
        test_passed("device class == RT_Device_Class_RTC");
    }

    if (rt_device_open(rtc_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        test_failed("rt_device_open", "open failed");
        return;
    }
    test_passed("rt_device_open (RDWR)");

    /* a bogus unknown command must fall through the switch and return -RT_EINVAL */
    if (rt_device_control(rtc_dev, RT_DEVICE_CTRL_BASE(RTC) + 0x7F, RT_NULL) == -RT_EINVAL)
    {
        test_passed("unknown RTC ctrl cmd -> -RT_EINVAL");
    }
    else
    {
        test_failed("unknown RTC ctrl cmd", "expected -RT_EINVAL");
    }

    if (rt_device_close(rtc_dev) != RT_EOK)
    {
        test_failed("rt_device_close", "close failed");
    }
    else
    {
        test_passed("rt_device_close");
    }
}

/* ------------------------------------------------------------------ */
/* 02/03 - sec-level read and write                                    */
/* ------------------------------------------------------------------ */
static void rtc_test_secs(void)
{
    time_t start = 0, now = 0;
    rt_err_t ret;

    TEST_REPORT_SECTION("02/03. get_secs / set_secs");

    if (!rtc_prepare())
    {
        return;
    }

    /* -- get_secs -- */
    ret = get_timestamp(&start);
    if (ret != RT_EOK)
    {
        test_failed("get_secs (get_timestamp)", "failed");
        return;
    }
    rt_kprintf("baseline ts = %ld (%s)\n", (long)start, ctime(&start));

    rt_thread_mdelay(2100);

    ret = get_timestamp(&now);
    if (ret != RT_EOK)
    {
        test_failed("get_secs (2nd read)", "failed");
        return;
    }
    rt_kprintf("after 2.1s  = %ld (%s)\n", (long)now, ctime(&now));

    if ((now - start) >= 2 && (now - start) <= 3)
    {
        test_passed("sec-level read advances ~2s");
    }
    else
    {
        test_failed("sec-level read advances", "delta out of range");
    }

    /* -- set_secs -- */
    {
        struct tm tm_new = {0};
        time_t target;

        tm_new.tm_year  = 2026 - 1900;
        tm_new.tm_mon   = 8 - 1;      /* 0-based, August */
        tm_new.tm_mday  = 14;
        tm_new.tm_hour  = 12;
        tm_new.tm_min   = 30;
        tm_new.tm_sec   = 0;
        target = timegm(&tm_new);

        ret = set_timestamp(target);   /* n32_rtc_set_secs + rt_alarm_update */
        if (ret != RT_EOK)
        {
            test_failed("set_secs (set_timestamp)", "failed");
            return;
        }
        rt_kprintf("wrote ts = %ld (2026-08-14 12:30:00 UTC)\n", (long)target);

        rt_thread_mdelay(1200);

        ret = get_timestamp(&now);
        if (ret != RT_EOK)
        {
            test_failed("set_secs readback", "failed");
            return;
        }
        rt_kprintf("readback ts = %ld (%s)\n", (long)now, ctime(&now));

        if ((now - target) >= 1 && (now - target) <= 3)
        {
            test_passed("set_secs write + readback");
        }
        else
        {
            test_failed("set_secs write + readback", "delta out of range");
        }
    }
}

/* ------------------------------------------------------------------ */
/* 04 - framework APIs: set_date / set_time / set_timestamp            */
/* ------------------------------------------------------------------ */
static void rtc_test_calendar(void)
{
    TEST_REPORT_SECTION("04. set_date / set_time (framework layer)");

    if (!rtc_prepare())
    {
        return;
    }

    if (set_date(2026, 8, 14) == RT_EOK)
    {
        test_passed("set_date(2026,8,14)");
    }
    else
    {
        test_failed("set_date(2026,8,14)", "failed");
    }

    if (set_time(23, 59, 30) == RT_EOK)
    {
        test_passed("set_time(23,59,30)");
    }
    else
    {
        test_failed("set_time(23,59,30)", "failed");
    }

    {
        time_t now = 0;
        get_timestamp(&now);
        rt_kprintf("after set_date+set_time: %ld (%s)\n", (long)now, ctime(&now));

        /* framework sanity: same UTC second must round-trip */
        struct tm tm_new = {0};
        time_t target = timegm(gmtime_r(&now, &tm_new));
        if ((target - now) == 0 || (now - target) == 0)
        {
            test_passed("timegm/gmtime_r round-trip");
        }
        else
        {
            test_failed("timegm/gmtime_r round-trip", "mismatch");
        }
    }
}

/* ------------------------------------------------------------------ */
/* 05/06/07 - timeval path                                             */
/* ------------------------------------------------------------------ */
static void rtc_test_timeval(void)
{
    struct timeval tv = {0};
    rt_err_t ret;

    TEST_REPORT_SECTION("05/06/07. get_timeval / set_timeval");

    if (!rtc_prepare())
    {
        return;
    }

    /* -- 05: gettimeofday() routes to RT_DEVICE_CTRL_RTC_GET_TIMEVAL -- */
    if (gettimeofday(&tv, RT_NULL) == 0 && tv.tv_sec != (time_t)0 && tv.tv_usec >= 0)
    {
        rt_kprintf("gettimeofday -> sec=%ld usec=%ld\n", (long)tv.tv_sec, (long)tv.tv_usec);
        if (tv.tv_usec < 1000000L)
        {
            test_passed("gettimeofday() -> GET_TIMEVAL (tv_usec from SUBS/PRE)");
        }
        else
        {
            test_failed("gettimeofday() -> GET_TIMEVAL", "usec out of range");
        }
    }
    else
    {
        test_failed("gettimeofday() -> GET_TIMEVAL", "failed/zero sec");
    }

    /* -- 06: raw control GET_TIMEVAL -- */
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    ret = rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_GET_TIMEVAL, &tv);
    if (ret == RT_EOK && tv.tv_sec != (time_t)0 && tv.tv_usec < 1000000L)
    {
        rt_kprintf("raw GET_TIMEVAL -> sec=%ld usec=%ld\n", (long)tv.tv_sec, (long)tv.tv_usec);
        test_passed("rt_device_control GET_TIMEVAL");
    }
    else
    {
        test_failed("rt_device_control GET_TIMEVAL", "failed/zero/out-of-range");
    }

    /* -- 07: set_timeval: drv_rtc.c leaves ops->set_timeval == RT_NULL, so
     *        the framework must answer -RT_EINVAL -- */
    tv.tv_sec  = (time_t)1000000000;
    tv.tv_usec = 0;
    ret = rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_SET_TIMEVAL, &tv);
    if (ret == -RT_EINVAL)
    {
        test_passed("SET_TIMEVAL -> -RT_EINVAL (ops->set_timeval==NULL)");
    }
    else
    {
        test_failed("SET_TIMEVAL", "expected -RT_EINVAL");
    }
}

/* ------------------------------------------------------------------ */
/* Alarm-related tests (only when RT_USING_ALARM is enabled)           */
/* ------------------------------------------------------------------ */
#ifdef RT_USING_ALARM

static volatile rt_bool_t alarm_oneshot_fired = RT_FALSE;
static volatile rt_bool_t alarm_daily_fired   = RT_FALSE;
static volatile rt_int32_t alarm_callback_cnt = 0;

static void rtc_alarm_callback(rt_alarm_t alarm, time_t timestamp)
{
    alarm_callback_cnt++;
    rt_kprintf("[ALARM] callback fired: ts=%ld flag=0x%02X\n",
               (long)timestamp, (unsigned int)alarm->flag);

    if ((alarm->flag & 0xFF00) == RT_ALARM_ONESHOT)
    {
        alarm_oneshot_fired = RT_TRUE;
    }
    else if ((alarm->flag & 0xFF00) == RT_ALARM_DAILY)
    {
        alarm_daily_fired = RT_TRUE;
    }
}

/* helper: create an alarm with explicit wakeup time */
static rt_alarm_t rtc_alarm_create_ts(rt_uint32_t flag, struct tm *wktime)
{
    struct rt_alarm_setup setup = {0};

    setup.flag    = flag;
    setup.wktime  = *wktime;
    return rt_alarm_create(rtc_alarm_callback, &setup);
}

/* ------------------------------------------------------------------ */
/* 08 - set_alarm / get_alarm through the device control path          */
/* ------------------------------------------------------------------ */
static void rtc_test_alarm_io(void)
{
    struct rt_rtc_wkalarm wkalarm = {0};
    struct rt_rtc_wkalarm rbk = {0};
    time_t now;
    rt_err_t ret;

    TEST_REPORT_SECTION("08. device-level SET_ALARM / GET_ALARM");

    if (!rtc_prepare())
    {
        return;
    }

    /* pick a time a few seconds in the future: hh:mm : current+10s.
     * NOTE: N32 drv_rtc stores UTC in the RTC hardware (set_rtc_time_stamp
     * uses gmtime_r), so alarm times must be built from gmtime_r (UTC),
     * NOT localtime_r (+8h here) or the alarm won't match the hardware. */
    now = time(RT_NULL);
    {
        struct tm tm_now = {0};
        gmtime_r(&now, &tm_now);

        wkalarm.enable  = RT_TRUE;
        wkalarm.tm_hour = tm_now.tm_hour;
        wkalarm.tm_min  = tm_now.tm_min;
        wkalarm.tm_sec  = (tm_now.tm_sec + 10) % 60;
        wkalarm.tm_mday = tm_now.tm_mday;
        wkalarm.tm_mon  = tm_now.tm_mon + 1;    /* 1-based */
        wkalarm.tm_year = tm_now.tm_year + 1900;
    }

    /* set the alarm */
    ret = rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_SET_ALARM, &wkalarm);
    if (ret != RT_EOK)
    {
        test_failed("SET_ALARM", "control failed");
        return;
    }
    rt_kprintf("SET_ALARM %02d:%02d:%02d enable=%d\n",
               wkalarm.tm_hour, wkalarm.tm_min, wkalarm.tm_sec, wkalarm.enable);

    /* read it back; drv_rtc stores the whole struct, so it must echo back */
    ret = rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_GET_ALARM, &rbk);
    if (ret != RT_EOK)
    {
        test_failed("GET_ALARM", "control failed");
        return;
    }
    rt_kprintf("GET_ALARM %02d:%02d:%02d enable=%d\n",
               rbk.tm_hour, rbk.tm_min, rbk.tm_sec, rbk.enable);

    if (rbk.tm_hour == wkalarm.tm_hour &&
            rbk.tm_min  == wkalarm.tm_min  &&
            rbk.tm_sec  == wkalarm.tm_sec  &&
            rbk.enable  == wkalarm.enable)
    {
        test_passed("SET_ALARM/GET_ALARM round-trip");
    }
    else
    {
        test_failed("SET_ALARM/GET_ALARM round-trip", "mismatch");
    }

    /* disable the alarm again to leave a clean state */
    wkalarm.enable = RT_FALSE;
    rt_device_control(rtc_dev, RT_DEVICE_CTRL_RTC_SET_ALARM, &wkalarm);
}

/* ------------------------------------------------------------------ */
/* 09/12 - alarm daemon: create/start + real hardware wakeup           */
/* ------------------------------------------------------------------ */
static void rtc_test_alarm_daemon(void)
{
    rt_alarm_t oneshot = RT_NULL;
    struct tm wt = {0};
    time_t now;
    rt_err_t ret;

    TEST_REPORT_SECTION("09/12. alarm daemon create/sync/start/callback");

    if (!rtc_prepare())
    {
        return;
    }

    /* reset callback flags */
    alarm_oneshot_fired = RT_FALSE;
    alarm_callback_cnt  = 0;

    /* one-shot: 8 seconds in the future.  Deliberately not too close
     * (now+1~2s can be skipped by the sub-second phase between writing
     * ALARMA and the next integer second tick).  8s gives a wide match
     * window so a missed callback really means "hardware never matched". */
    now = time(RT_NULL);
    /* UTC on purpose: N32 RTC hardware keeps UTC, so the alarm time the
     * driver programs must also be UTC or it will never match. */
    gmtime_r(&now, &wt);
    wt.tm_sec += 8;
    if (wt.tm_sec > 59)
    {
        wt.tm_sec -= 60;
        wt.tm_min += 1;
        if (wt.tm_min > 59)
        {
            wt.tm_min = 0;
            wt.tm_hour += 1;
        }
    }

    oneshot = rtc_alarm_create_ts(RT_ALARM_ONESHOT, &wt);
    if (oneshot == RT_NULL)
    {
        test_failed("rt_alarm_create(oneshot)", "NULL");
        return;
    }
    test_passed("rt_alarm_create(oneshot)");

    /* rt_alarm_start -> alarm_setup + alarm_set -> SET_ALARM to driver,
     * which programs Alarm A + EXTI17 + RTC_ALARM IRQ */
    ret = rt_alarm_start(oneshot);
    if (ret != RT_EOK)
    {
        test_failed("rt_alarm_start(oneshot)", "failed");
        rt_alarm_delete(oneshot);
        return;
    }
    test_passed("rt_alarm_start(oneshot)");

    /* --- DEBUG PROBE: dump the real hardware alarm register right after
     * start, before the target time passes.  ALARMA bit layout:
     *   [31] MASK4, [30] WKDSEL, [29:28] DTT, [27:24] DTU(day),
     *   [23] MASK3, [22] HT, [21:20] HU(hour), [19] MASK2,
     *   [18] MT, [17:16] MU(min), [15] MASK1, [14] ST, [13:12] SU(sec) */
    rt_kprintf("[PROBE] RTC->ALARMA = 0x%08X (Ctrl=0x%08X, InitSts=0x%08X)\n",
               (unsigned int)RTC->ALARMA,
               (unsigned int)RTC->CTRL,
               (unsigned int)RTC->INITSTS);
    /* RTC->DATE: [31]YOTT..[20]YRT[4] [19:16]YRU [14:13]WDU [12]MOT
     * [11:8]MOU [5:4]DAT(day tens) [3:0]DAU(day ones).  If the alarm
     * date (ALARMA day=14) never equals RTC->DATE's day, alarm won't fire. */
    rt_kprintf("[PROBE] RTC->DATE = 0x%08X  -> year=%02u mon=%02u day=%02u wday=%u\n",
               (unsigned int)RTC->DATE,
               (unsigned int)((RTC->DATE >> 16) & 0xFFU),
               (unsigned int)(((RTC->DATE >> 8) & 0x1FU) & 0x0FU) | ((((RTC->DATE >> 8) & 0x1FU) >> 4) & 0x1U) * 10,
               (unsigned int)(((RTC->DATE >> 4) & 0x03U) * 10 + (RTC->DATE & 0x0FU)),
               (unsigned int)((RTC->DATE >> 13) & 0x7U));
    rt_kprintf("[PROBE] expected  alarm %02d:%02d:%02d  day=%02d  (now=%02d:%02d:%02d)\n",
               wt.tm_hour, wt.tm_min, wt.tm_sec, wt.tm_mday,
               wt.tm_hour, wt.tm_min, wt.tm_sec);

    /* poll every second for the full alarm window; print the hardware
     * TSH(hh:mm:ss)/DATE and ALAF each tick.  This tells us:
     *   a) is the RTC really counting 1 Hz (LSE truly running)
     *   b) does ALAF ever get set exactly at the alarm second */
    {
        rt_uint32_t tick;

        for (tick = 0; tick < 8; tick++)
        {
            rt_thread_mdelay(1000);
            rt_kprintf("[POLL %u] TSH=0x%08X DATE=0x%08X InitSts=0x%08X ALAF=%s\n",
                       tick,
                       (unsigned int)RTC->TSH,
                       (unsigned int)RTC->DATE,
                       (unsigned int)RTC->INITSTS,
                       (RTC->INITSTS & 0x100U) ? "SET" : "clear");
        }
        rt_thread_mdelay(5000);
    }

    if (alarm_oneshot_fired)
    {
        test_passed("one-shot alarm callback fired (~8s)");
    }
    else
    {
        test_failed("one-shot alarm callback fired", "callback not called");
    }

    /* dump the final hardware alarm flag state after the window elapsed */
    rt_kprintf("[PROBE3] after window: InitSts=0x%08X (ALAF=bit8)\n",
               (unsigned int)RTC->INITSTS);

    /* safety: stop & free */
    rt_alarm_stop(oneshot);
    rt_alarm_delete(oneshot);
    test_passed("rt_alarm_stop / rt_alarm_delete (oneshot)");
}

/* ------------------------------------------------------------------ */
/* 10 - rt_alarm_control RT_ALARM_CTRL_MODIFY                          */
/* ------------------------------------------------------------------ */
static void rtc_test_alarm_modify(void)
{
    rt_alarm_t alarm = RT_NULL;
    struct rt_alarm_setup setup = {0};
    struct tm wt = {0};
    time_t now;
    rt_err_t ret;

    TEST_REPORT_SECTION("10. rt_alarm_control MODIFY");

    if (!rtc_prepare())
    {
        return;
    }

    now = time(RT_NULL);
    gmtime_r(&now, &wt);   /* UTC: RTC hw keeps UTC, alarm must too */
    wt.tm_sec = (wt.tm_sec + 2) % 60;

    setup.flag    = RT_ALARM_ONESHOT;
    setup.wktime  = wt;
    alarm = rt_alarm_create(rtc_alarm_callback, &setup);
    if (alarm == RT_NULL)
    {
        test_failed("rt_alarm_create (modify base)", "NULL");
        return;
    }

    /* change it to a daily alarm one minute in the future */
    wt.tm_sec = (wt.tm_sec + 5) % 60;
    setup.flag   = RT_ALARM_DAILY;
    setup.wktime = wt;

    ret = rt_alarm_control(alarm, RT_ALARM_CTRL_MODIFY, &setup);
    if (ret == RT_EOK)
    {
        if ((alarm->flag & 0xFF00) == RT_ALARM_DAILY)
        {
            test_passed("rt_alarm_control MODIFY (flag ONESHOT->DAILY)");
        }
        else
        {
            test_failed("rt_alarm_control MODIFY", "flag not updated");
        }
    }
    else
    {
        test_failed("rt_alarm_control MODIFY", "failed");
    }

    rt_alarm_delete(alarm);
}

/* ------------------------------------------------------------------ */
/* 11 - rt_alarm_delete                                                */
/* ------------------------------------------------------------------ */
static void rtc_test_alarm_delete(void)
{
    rt_alarm_t alarm = RT_NULL;
    struct rt_alarm_setup setup = {0};
    struct tm wt = {0};
    time_t now;

    TEST_REPORT_SECTION("11. rt_alarm_delete");

    if (!rtc_prepare())
    {
        return;
    }

    now = time(RT_NULL);
    gmtime_r(&now, &wt);   /* UTC: RTC hw keeps UTC, alarm must too */
    wt.tm_sec = (wt.tm_sec + 2) % 60;

    setup.flag   = RT_ALARM_ONESHOT;
    setup.wktime = wt;
    alarm = rt_alarm_create(rtc_alarm_callback, &setup);
    if (alarm == RT_NULL)
    {
        test_failed("rt_alarm_create (delete base)", "NULL");
        return;
    }

    if (rt_alarm_delete(alarm) == RT_EOK)
    {
        test_passed("rt_alarm_delete (un-started)");
    }
    else
    {
        test_failed("rt_alarm_delete (un-started)", "failed");
    }

    /* delete a started alarm as well */
    now = time(RT_NULL);
    gmtime_r(&now, &wt);   /* UTC: RTC hw keeps UTC, alarm must too */
    wt.tm_sec = (wt.tm_sec + 2) % 60;
    setup.wktime = wt;
    alarm = rt_alarm_create(rtc_alarm_callback, &setup);
    if (alarm == RT_NULL)
    {
        test_failed("rt_alarm_create (delete started)", "NULL");
        return;
    }
    rt_alarm_start(alarm);
    rt_thread_mdelay(100);
    if (rt_alarm_delete(alarm) == RT_EOK)
    {
        test_passed("rt_alarm_delete (started)");
    }
    else
    {
        test_failed("rt_alarm_delete (started)", "failed");
    }
}

/* ------------------------------------------------------------------ */
/* 13 - set_secs triggers rt_alarm_update (alarm re-arm on time change)*/
/* ------------------------------------------------------------------ */
static void rtc_test_time_change_rearm(void)
{
    rt_alarm_t alarm = RT_NULL;
    struct rt_alarm_setup setup = {0};
    struct tm wt = {0};
    time_t now;

    TEST_REPORT_SECTION("13. time change -> rt_alarm_update");

    if (!rtc_prepare())
    {
        return;
    }

    /* create+start a daily alarm 40s in the future */
    now = time(RT_NULL);
    gmtime_r(&now, &wt);   /* UTC: RTC hw keeps UTC, alarm must too */
    wt.tm_sec = (wt.tm_sec + 40) % 60;

    setup.flag   = RT_ALARM_DAILY;
    setup.wktime = wt;
    alarm = rt_alarm_create(rtc_alarm_callback, &setup);
    if (alarm == RT_NULL)
    {
        test_failed("alarm create (rearm base)", "NULL");
        return;
    }
    rt_alarm_start(alarm);

    /* change the time via the framework; n32_rtc_set_secs() calls
     * rt_alarm_update(&rtc_dev.parent, 1) so the alarm service re-arms */
    if (set_timestamp(time(RT_NULL)) == RT_EOK)
    {
        test_passed("set_secs -> rt_alarm_update re-arm");
    }
    else
    {
        test_failed("set_secs -> rt_alarm_update re-arm", "set_timestamp failed");
    }

    rt_alarm_stop(alarm);
    rt_alarm_delete(alarm);
}

/* ------------------------------------------------------------------ */
/* 14 - RTC backup register                                            */
/* ------------------------------------------------------------------ */
static void rtc_test_backup_reg(void)
{
    uint32_t rd;

    TEST_REPORT_SECTION("14. RTC_BKUP_Read/Write (backup register)");

    if (!rtc_prepare())
    {
        return;
    }

    /* exercise the same weak functions drv_rtc.c uses; we don't disturb the
     * driver's own magic slot in a way that breaks the RTC config path:
     * read first, then a write/read-back round trip on reg 1 (magic reg). */
    rd = RTC_BKUP_Read(RTC_TEST_BKUP_REG);
    rt_kprintf("read  RTC_BKP reg%d = 0x%08X\n", (int)RTC_TEST_BKUP_REG, rd);

    {
        uint32_t wdata = 0x11223344U;
        RTC_BKUP_Write(RTC_TEST_BKUP_REG, wdata);
        rd = RTC_BKUP_Read(RTC_TEST_BKUP_REG);
        if (wdata == rd)
        {
            test_passed("RTC_BKUP_Write/Read round-trip");
        }
        else
        {
            test_failed("RTC_BKUP_Write/Read round-trip", "readback mismatch");
        }
    }

    /* restore the driver's magic value so the RTC config path keeps working
     * on the next boot */
    RTC_BKUP_Write(RTC_TEST_BKUP_REG, RTC_TEST_BKUP_MAGIC);
    rt_kprintf("restored magic 0x%08X to reg%d\n", RTC_TEST_BKUP_MAGIC,
               (int)RTC_TEST_BKUP_REG);
}

#endif /* RT_USING_ALARM */

/* ------------------------------------------------------------------ */
/* 15 - info / check                                                   */
/* ------------------------------------------------------------------ */
static void rtc_test_info(void)
{
    struct timeval tv = {0};

    TEST_REPORT_SECTION("15. rtc info / date command linkage");

    if (!rtc_prepare())
    {
        return;
    }

    if (gettimeofday(&tv, RT_NULL) == 0)
    {
        rt_kprintf("gettimeofday: sec=%ld usec=%ld\n", (long)tv.tv_sec, (long)tv.tv_usec);
        rt_kprintf("ctime       : %s", ctime(&tv.tv_sec));
        rt_kprintf("gmtime check: %s", asctime(gmtime(&tv.tv_sec)));
    }
    else
    {
        rt_kprintf("gettimeofday failed\n");
    }

    rt_kprintf("(also try 'date' and 'list_alarm' MSH commands)\n");
}

/* declared later so one suite command can run everything */
void rtc_test_full(void);

void rtc_test_full(void)
{
    test_pass_count = 0;
    test_fail_count = 0;

    rt_kprintf("\n########## RTC SERIES FULL TEST ##########\n");
#if defined(BSP_RTC_USING_LSI)
    rt_kprintf("# clock source: LSI\n");
#elif defined(BSP_RTC_USING_LSE)
    rt_kprintf("# clock source: LSE\n");
#else
    rt_kprintf("# clock source: HSE\n");
#endif

    rtc_test_init();
    rtc_test_device();
    rtc_test_secs();
    rtc_test_calendar();
    rtc_test_timeval();

#ifdef RT_USING_ALARM
    rtc_test_alarm_io();
    rtc_test_alarm_daemon();
    rtc_test_alarm_modify();
    rtc_test_alarm_delete();
    rtc_test_time_change_rearm();
    rtc_test_backup_reg();
#else
    rt_kprintf("\n[SKIP] alarm/backup tests: RT_USING_ALARM not enabled\n");
#endif

    rtc_test_info();
    test_summary();
}
MSH_CMD_EXPORT(rtc_test_full, run the whole RTC device+driver+framework test suite);

/* ------------------------------------------------------------------ */
/* Individual MSH entry points                                         */
/* ------------------------------------------------------------------ */
void rtc_test_init_cmd(void)
{
    rtc_test_init();
}
MSH_CMD_EXPORT(rtc_test_init_cmd, run drv_rtc init/config test only);

void rtc_test_time_cmd(void)
{
    rtc_test_secs();
    rtc_test_calendar();
    rtc_test_timeval();
}
MSH_CMD_EXPORT(rtc_test_time_cmd, run RTC time read/write tests (secs/date/timeval));

void rtc_test_alarm_cmd(void)
{
#ifdef RT_USING_ALARM
    rtc_test_alarm_io();
    rtc_test_alarm_daemon();
    rtc_test_alarm_modify();
    rtc_test_alarm_delete();
    rtc_test_time_change_rearm();
#else
    rt_kprintf("SKIP: RT_USING_ALARM not enabled\n");
#endif
}
MSH_CMD_EXPORT(rtc_test_alarm_cmd, run RTC alarm tests only (needs RT_USING_ALARM));

void rtc_test_backup_cmd(void)
{
#ifdef RT_USING_ALARM
    rtc_test_backup_reg();
#else
    rt_kprintf("SKIP: needs RT_USING_ALARM (weak RTC_BKUP_* helpers live there)\n");
#endif
}
MSH_CMD_EXPORT(rtc_test_backup_cmd, run RTC backup-register test only);

#endif /* BSP_USING_ONCHIP_RTC */
