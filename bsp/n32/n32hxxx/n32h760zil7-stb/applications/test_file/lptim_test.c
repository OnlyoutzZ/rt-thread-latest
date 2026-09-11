/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-07-07     li.mengmeng      lptim test for N32H7xx
 */

#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <drv_lptim.h>

#ifdef BSP_USING_LPTIM

#define LPTIM_DEV_NAME "lptim5"

/* LPTIM hardware frequency: LSI (32768 Hz) / DIV32 = 1024 Hz, 1 second = 1024 ticks */
#define LPTIM_TICK_PER_SEC 1024

/* ========================= ONESHOT timeout test ========================= */
static rt_err_t lptim_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("[LPTIM] oneshot timeout callback triggered! tick: %d\n", rt_tick_get());
    return 0;
}

int lptim_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t hw_dev = RT_NULL;
    rt_uint32_t timeout_ticks = 3 * LPTIM_TICK_PER_SEC;  /* 3 seconds = 3072 ticks */

    /* Find the LPTIM device */
    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("lptim sample run failed! can't find %s device!\n", LPTIM_DEV_NAME);
        return RT_ERROR;
    }

    /* Open the device for read/write */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", LPTIM_DEV_NAME);
        return ret;
    }

    /* Set the timeout callback */
    rt_device_set_rx_indicate(hw_dev, lptim_timeout_cb);

    /* Start ONESHOT mode directly, passing the raw tick count
     * (3 seconds x 1024 Hz = 3072 ticks) */
    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    if (ret != RT_EOK)
    {
        rt_kprintf("lptim start failed! ret: %d\n", ret);
        return ret;
    }

    rt_kprintf("[LPTIM] oneshot mode started, waiting 3 seconds (%d ticks)...\n", timeout_ticks);

    /* Wait for the timeout to fire */
    rt_thread_mdelay(4000);

    return ret;
}
MSH_CMD_EXPORT(lptim_sample, lptim oneshot timeout test);


/* ========================= software period mode test ========================= */
static rt_device_t g_lptim_period_dev = RT_NULL;

static rt_err_t lptim_period_cb(rt_device_t dev, rt_size_t size)
{
    static rt_uint32_t count = 0;
    rt_uint32_t timeout_ticks = 2 * LPTIM_TICK_PER_SEC;  /* 2 seconds = 2048 ticks */

    count++;
    rt_kprintf("[LPTIM] period callback #%d @ tick: %d\n", count, rt_tick_get());

    /* Restart ONESHOT to emulate a software period */
    if (g_lptim_period_dev != RT_NULL)
    {
        rt_device_control(g_lptim_period_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    }

    return 0;
}

int lptim_period_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t hw_dev = RT_NULL;
    rt_uint32_t timeout_ticks = 2 * LPTIM_TICK_PER_SEC;  /* 2 seconds = 2048 ticks */

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("lptim period test failed! can't find %s device!\n", LPTIM_DEV_NAME);
        return RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", LPTIM_DEV_NAME);
        return ret;
    }

    g_lptim_period_dev = hw_dev;

    /* Set the period callback (it restarts the timer itself) */
    rt_device_set_rx_indicate(hw_dev, lptim_period_cb);

    /* Start the first ONESHOT timeout */
    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    if (ret != RT_EOK)
    {
        rt_kprintf("lptim period start failed! ret: %d\n", ret);
        return ret;
    }

    rt_kprintf("[LPTIM] software period mode started, interval = 2 seconds...\n");

    return ret;
}
MSH_CMD_EXPORT(lptim_period_sample, lptim period mode test);


/* ========================= LPTIM device info read test ========================= */
int lptim_info_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t hw_dev = RT_NULL;
    rt_uint32_t tick_max = 0;
    rt_uint32_t freq = 0;
    rt_uint32_t count = 0;

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("lptim info test failed! can't find %s device!\n", LPTIM_DEV_NAME);
        return RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", LPTIM_DEV_NAME);
        return ret;
    }

    /* Get the LPTIM maximum counter value (0xFFFF) */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_TICK_MAX, &tick_max);
    rt_kprintf("[LPTIM] max tick value: %d (0x%04X)\n", tick_max, tick_max);

    /* Get the LPTIM frequency */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_FREQ, &freq);
    rt_kprintf("[LPTIM] timer frequency: %d Hz\n", freq);

    /* Read the current counter value */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_COUNT, &count);
    rt_kprintf("[LPTIM] current counter value: %d\n", count);

    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_info_sample, lptim get info test);


/* ========================= LPTIM STOP test ========================= */
int lptim_stop_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t hw_dev = RT_NULL;

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("lptim stop test failed! can't find %s device!\n", LPTIM_DEV_NAME);
        return RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", LPTIM_DEV_NAME);
        return ret;
    }

    /* Clear the callback so the period-mode callback cannot interfere */
    rt_device_set_rx_indicate(hw_dev, RT_NULL);
    g_lptim_period_dev = RT_NULL;

    /* Stop the timer */
    rt_device_control(hw_dev, CLOCK_TIMER_CTRL_STOP, RT_NULL);
    rt_kprintf("[LPTIM] timer stopped.\n");

    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_stop_sample, lptim stop timer test);

#endif /* BSP_USING_LPTIM */
