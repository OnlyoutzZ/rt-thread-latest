/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-19     ox-horse         LPTIM test for N32H47x_48x
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drv_lptim.h>

#if defined(BSP_USING_LPTIM) && defined(BSP_USING_LPTIM2)

#define LPTIM_DEV_NAME     "lptim2"
#define LPTIM_TICK_PER_SEC 1024U

static rt_device_t lptim_period_dev = RT_NULL;

static rt_err_t lptim_timeout_cb(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    (void)size;

    rt_kprintf("[LPTIM] oneshot timeout, tick=%u\n", rt_tick_get());
    return RT_EOK;
}

static rt_err_t lptim_period_cb(rt_device_t dev, rt_size_t size)
{
    rt_uint32_t timeout_ticks = 2U * LPTIM_TICK_PER_SEC;
    static rt_uint32_t count;

    (void)dev;
    (void)size;

    count++;
    rt_kprintf("[LPTIM] period callback #%u, tick=%u\n", count, rt_tick_get());
    if (lptim_period_dev != RT_NULL)
    {
        rt_device_control(lptim_period_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    }

    return RT_EOK;
}

static int lptim_sample(void)
{
    rt_device_t hw_dev;
    rt_uint32_t timeout_ticks = 3U * LPTIM_TICK_PER_SEC;
    rt_err_t ret;

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", LPTIM_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        return ret;
    }

    rt_device_set_rx_indicate(hw_dev, lptim_timeout_cb);
    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    if (ret != RT_EOK)
    {
        rt_device_close(hw_dev);
        return ret;
    }

    rt_kprintf("[LPTIM] %s oneshot started, ticks=%u\n", LPTIM_DEV_NAME, timeout_ticks);
    rt_thread_mdelay(4000);
    rt_device_close(hw_dev);
    return RT_EOK;
}
MSH_CMD_EXPORT(lptim_sample, test N32H47x_48x LPTIM2 oneshot);

static int lptim_period_sample(void)
{
    rt_device_t hw_dev;
    rt_uint32_t timeout_ticks = 2U * LPTIM_TICK_PER_SEC;
    rt_err_t ret;

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", LPTIM_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        return ret;
    }

    lptim_period_dev = hw_dev;
    rt_device_set_rx_indicate(hw_dev, lptim_period_cb);
    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_START, &timeout_ticks);
    if (ret != RT_EOK)
    {
        lptim_period_dev = RT_NULL;
        rt_device_close(hw_dev);
    }
    return ret;
}
MSH_CMD_EXPORT(lptim_period_sample, test N32H47x_48x LPTIM2 software period);

static int lptim_info_sample(void)
{
    rt_device_t hw_dev;
    rt_uint32_t tick_max = 0;
    rt_uint32_t freq = 0;
    rt_uint32_t count = 0;
    rt_err_t ret;

    hw_dev = rt_device_find(LPTIM_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", LPTIM_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_TICK_MAX, &tick_max);
    if (ret == RT_EOK)
    {
        ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_FREQ, &freq);
    }
    if (ret == RT_EOK)
    {
        ret = rt_device_control(hw_dev, DRV_HW_LPTIMER_CTRL_GET_COUNT, &count);
    }

    rt_kprintf("[LPTIM] max=%u, freq=%u Hz, count=%u, ret=%d\n",
               tick_max, freq, count, ret);
    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_info_sample, test N32H47x_48x LPTIM2 information);

static int lptim_stop_sample(void)
{
    rt_device_t hw_dev;
    rt_err_t ret;

    hw_dev = lptim_period_dev;
    if (hw_dev == RT_NULL)
    {
        hw_dev = rt_device_find(LPTIM_DEV_NAME);
        if (hw_dev == RT_NULL)
        {
            rt_kprintf("can't find %s\n", LPTIM_DEV_NAME);
            return -RT_ERROR;
        }

        ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    rt_device_set_rx_indicate(hw_dev, RT_NULL);
    lptim_period_dev = RT_NULL;
    ret = rt_device_control(hw_dev, CLOCK_TIMER_CTRL_STOP, RT_NULL);
    rt_device_close(hw_dev);
    return ret;
}
MSH_CMD_EXPORT(lptim_stop_sample, stop N32H47x_48x LPTIM2 test);

#endif
