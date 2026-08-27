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

#define LPTIM_DEV_NAME   "lptim5"

/* LPTIM 硬件频率: LSI(32768Hz) / DIV32 = 1024Hz, 1秒 = 1024 tick */
#define LPTIM_TICK_PER_SEC  1024

/* ========================= ONESHOT 超时测试 ========================= */
static rt_err_t lptim_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("[LPTIM] oneshot timeout callback triggered! tick: %d\n", rt_tick_get());
    return 0;
}

int lptim_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t hw_dev = RT_NULL;
    rt_uint32_t timeout_ticks = 3 * LPTIM_TICK_PER_SEC;  /* 3 秒 = 3072 ticks */

    /* 查找 LPTIM 设备 */
    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("lptim sample run failed! can't find %s device!\n", LPTIM_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", LPTIM_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, lptim_timeout_cb);

    /* 直接启动 ONESHOT 模式，传入原始 tick 值（3秒 × 1024Hz = 3072 ticks） */
    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    if (ret != RT_EOK)
    {
        rt_kprintf("lptim start failed! ret: %d\n", ret);
        return ret;
    }

    rt_kprintf("[LPTIM] oneshot mode started, waiting 3 seconds (%d ticks)...\n", timeout_ticks);

    /* 等待超时触发 */
    rt_thread_mdelay(4000);

    return ret;
}
MSH_CMD_EXPORT(lptim_sample, lptim oneshot timeout test);


/* ========================= 软件周期模式测试 ========================= */
static rt_device_t g_lptim_period_dev = RT_NULL;

static rt_err_t lptim_period_cb(rt_device_t dev, rt_size_t size)
{
    static rt_uint32_t count = 0;
    rt_uint32_t timeout_ticks = 2 * LPTIM_TICK_PER_SEC;  /* 2 秒 = 2048 ticks */

    count++;
    rt_kprintf("[LPTIM] period callback #%d @ tick: %d\n", count, rt_tick_get());

    /* 重新启动 ONESHOT，实现软件周期效果 */
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
    rt_uint32_t timeout_ticks = 2 * LPTIM_TICK_PER_SEC;  /* 2 秒 = 2048 ticks */

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

    /* 设置周期回调（回调中会自动重新启动） */
    rt_device_set_rx_indicate(hw_dev, lptim_period_cb);

    /* 启动第一次 ONESHOT 超时 */
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


/* ========================= LPTIM 设备信息读取测试 ========================= */
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

    /* 获取 LPTIM 最大计数值 (0xFFFF) */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_TICK_MAX, &tick_max);
    rt_kprintf("[LPTIM] max tick value: %d (0x%04X)\n", tick_max, tick_max);

    /* 获取 LPTIM 频率 */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_FREQ, &freq);
    rt_kprintf("[LPTIM] timer frequency: %d Hz\n", freq);

    /* 读取当前计数值 */
    rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_COUNT, &count);
    rt_kprintf("[LPTIM] current counter value: %d\n", count);

    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_info_sample, lptim get info test);


/* ========================= LPTIM STOP 测试 ========================= */
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

    /* 清除回调，避免周期模式回调干扰 */
    rt_device_set_rx_indicate(hw_dev, RT_NULL);
    g_lptim_period_dev = RT_NULL;

    /* 停止定时器 */
    rt_device_control(hw_dev, CLOCK_TIMER_CTRL_STOP, RT_NULL);
    rt_kprintf("[LPTIM] timer stopped.\n");

    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_stop_sample, lptim stop timer test);

#endif /* BSP_USING_LPTIM */
