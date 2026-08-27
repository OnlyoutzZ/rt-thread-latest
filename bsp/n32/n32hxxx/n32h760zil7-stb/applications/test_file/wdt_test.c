/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-06-16     li.mengmeng      wdt test for N32H7xx
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_WDT

#define WDT_DEVICE_NAME     "wdt"
#define WDT_TEST_TIMEOUT_S  5       /* 5 seconds timeout for test */
#define WDT_FEED_INTERVAL_S 2       /* feed every 2 seconds */

static rt_device_t  wdt_dev = RT_NULL;
static rt_bool_t    test_running = RT_FALSE;

static void wdt_feed_thread_entry(void *parameter)
{
    rt_tick_t timeout = (rt_tick_t)WDT_FEED_INTERVAL_S * RT_TICK_PER_SECOND;

    rt_kprintf("wdt feed thread started, feeding every %d seconds\n", WDT_FEED_INTERVAL_S);

    while (test_running)
    {
        rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
        rt_kprintf("wdt fed at tick %d\n", (int)rt_tick_get());
        rt_thread_mdelay((rt_int32_t)timeout);
    }

    rt_kprintf("wdt feed thread stopped\n");
}

static int wdt_test_info(int argc, char **argv)
{
    rt_uint32_t timeout_s = 0;

    if (!wdt_dev)
    {
        wdt_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdt_dev)
        {
            rt_kprintf("cannot find wdt device '%s'\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
    }

    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_GET_TIMEOUT, &timeout_s);
    rt_kprintf("=== WDT Device Info ===\n");
    rt_kprintf("device name : %s\n", WDT_DEVICE_NAME);
    rt_kprintf("timeout     : %d s\n", (int)timeout_s);
    rt_kprintf("test running: %s\n", test_running ? "yes" : "no");

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_info, show wdt device info);

static int wdt_test_set_timeout(int argc, char **argv)
{
    rt_uint32_t timeout_s;
    rt_err_t result;

    if (!wdt_dev)
    {
        wdt_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdt_dev)
        {
            rt_kprintf("cannot find wdt device '%s'\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
    }

    if (argc < 2)
    {
        rt_kprintf("usage: wdt_test_set_timeout <seconds>\n");
        rt_kprintf("       current timeout: query with wdt_test_info\n");
        return -RT_ERROR;
    }

    timeout_s = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    if (timeout_s == 0)
    {
        rt_kprintf("timeout must be > 0\n");
        return -RT_EINVAL;
    }

    result = rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout_s);
    if (result != RT_EOK)
    {
        rt_kprintf("set timeout failed: %d\n", result);
        return result;
    }

    rt_kprintf("set timeout to %d seconds OK\n", (int)timeout_s);

    /* verify */
    rt_uint32_t verify_s = 0;
    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_GET_TIMEOUT, &verify_s);
    rt_kprintf("verify timeout: %d seconds\n", (int)verify_s);

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_set_timeout, set wdt timeout: wdt_test_set_timeout <seconds>);

static int wdt_test_start(int argc, char **argv)
{
    rt_err_t result;
    rt_thread_t feed_thread;

    if (!wdt_dev)
    {
        wdt_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdt_dev)
        {
            rt_kprintf("cannot find wdt device '%s'\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
    }

    if (test_running)
    {
        rt_kprintf("WDT test is already running\n");
        return -RT_EBUSY;
    }

    /* init the device */
    result = rt_device_init(wdt_dev);
    if (result != RT_EOK)
    {
        rt_kprintf("wdt init failed: %d\n", result);
        return result;
    }

    rt_uint32_t timeout_s = WDT_TEST_TIMEOUT_S;
    if (argc >= 2)
    {
        timeout_s = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }

    /* set a safe timeout first */
    result = rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout_s);
    if (result != RT_EOK)
    {
        rt_kprintf("set timeout failed: %d\n", result);
        return result;
    }
    rt_kprintf("set timeout to %d seconds\n", (int)timeout_s);

    /* start the watchdog */
    result = rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (result != RT_EOK)
    {
        rt_kprintf("wdt start failed: %d\n", result);
        return result;
    }
    rt_kprintf("wdt started, will reset if not fed within %d seconds\n", (int)timeout_s);

    /* start feed thread */
    test_running = RT_TRUE;
    feed_thread = rt_thread_create("wdt_feed",
                                   wdt_feed_thread_entry,
                                   RT_NULL,
                                   1024,
                                   RT_THREAD_PRIORITY_MAX - 2,
                                   10);
    if (feed_thread)
    {
        rt_thread_startup(feed_thread);
        rt_kprintf("=== WDT test running ===\n");
        rt_kprintf("Feed thread created. To stop: wdt_test_stop\n");
        rt_kprintf("To test reset: kill feed thread and wait %d seconds\n", (int)timeout_s);
    }
    else
    {
        test_running = RT_FALSE;
        rt_kprintf("failed to create feed thread\n");
        return -RT_ENOMEM;
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_start, start wdt test: wdt_test_start [timeout_seconds]);

static int wdt_test_stop(int argc, char **argv)
{
    if (!test_running)
    {
        rt_kprintf("WDT test is not running\n");
        return RT_EOK;
    }

    test_running = RT_FALSE;

    /* wait for feed thread to exit */
    rt_thread_mdelay(100);

    /* keep feeding to prevent reset until we return */
    if (wdt_dev)
    {
        rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
    }

    rt_kprintf("WDT test stopped.\n");
    rt_kprintf("NOTE: IWDG is still running and CANNOT be disabled!\n");
    rt_kprintf("      System will reset if not fed before timeout.\n");
    rt_kprintf("      Please reset the MCU to disable IWDG.\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_stop, stop wdt feed thread(IWDG still runs!));

int wdt_test_simple(void)
{
    rt_err_t result;
    rt_uint32_t timeout_s = 8;

    if (!wdt_dev)
    {
        wdt_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdt_dev)
        {
            rt_kprintf("cannot find wdt device '%s'\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
    }


    rt_kprintf("=== WDT Simple Test (no auto-feed) ===\n");
    rt_kprintf("timeout set to %d seconds\n", (int)timeout_s);
    rt_kprintf("You must manually feed with 'wdt_test_feed' before timeout!\n\n");

    result = rt_device_init(wdt_dev);
    if (result != RT_EOK)
    {
        rt_kprintf("wdt init failed: %d\n", result);
        return result;
    }

    result = rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout_s);
    if (result != RT_EOK)
    {
        rt_kprintf("set timeout failed: %d\n", result);
        return result;
    }

    result = rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (result != RT_EOK)
    {
        rt_kprintf("wdt start failed: %d\n", result);
        return result;
    }

    rt_kprintf("IWDG started! Feed with 'wdt_test_feed' within %d seconds or system resets.\n", (int)timeout_s);

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_simple, simple wdt test(manual feed): wdt_test_simple [timeout_seconds]);

int wdt_test_feed(void)
{
    if (!wdt_dev)
    {
        wdt_dev = rt_device_find(WDT_DEVICE_NAME);
        if (!wdt_dev)
        {
            rt_kprintf("cannot find wdt device '%s'\n", WDT_DEVICE_NAME);
            return -RT_ERROR;
        }
    }

    rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
    rt_kprintf("wdt fed at tick %d\n", (int)rt_tick_get());

    return RT_EOK;
}
MSH_CMD_EXPORT(wdt_test_feed, feed the watchdog manually);

#endif /* RT_USING_WDT */
