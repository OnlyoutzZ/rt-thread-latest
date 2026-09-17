/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-19     ox-horse         hwtimer test for N32H49x
 */

#include <rtthread.h>
#include <rtdevice.h>

#if defined(BSP_USING_CLOCK_TIMER) && defined(BSP_USING_GTIM1)

#define HWTIMER_DEV_NAME "timer5" /* 32-bit GTIM1 */

static rt_err_t hwtimer_timeout_cb(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    (void)size;

    rt_kprintf("[HWTIMER] timeout, tick=%u\n", rt_tick_get());
    return RT_EOK;
}

static int hwtimer_sample(void)
{
    rt_device_t hw_dev;
    rt_clock_timerval_t timeout;
    rt_clock_timer_mode_t mode = CLOCK_TIMER_MODE_PERIOD;
    rt_uint32_t freq = 10000;
    rt_err_t ret;

    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", HWTIMER_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s failed: %d\n", HWTIMER_DEV_NAME, ret);
        return ret;
    }

    rt_device_set_rx_indicate(hw_dev, hwtimer_timeout_cb);

    ret = rt_device_control(hw_dev, CLOCK_TIMER_CTRL_FREQ_SET, &freq);
    if (ret == RT_EOK)
    {
        ret = rt_device_control(hw_dev, CLOCK_TIMER_CTRL_MODE_SET, &mode);
    }
    if (ret != RT_EOK)
    {
        rt_kprintf("configure %s failed: %d\n", HWTIMER_DEV_NAME, ret);
        rt_device_close(hw_dev);
        return ret;
    }

    timeout.sec = 5;
    timeout.usec = 0;
    if (rt_device_write(hw_dev, 0, &timeout, sizeof(timeout)) != sizeof(timeout))
    {
        rt_kprintf("start %s failed\n", HWTIMER_DEV_NAME);
        rt_device_close(hw_dev);
        return -RT_ERROR;
    }

    rt_thread_mdelay(3500);
    rt_device_read(hw_dev, 0, &timeout, sizeof(timeout));
    rt_kprintf("[HWTIMER] elapsed: %u s, %u us\n", timeout.sec, timeout.usec);

    rt_thread_mdelay(2000);
    rt_device_control(hw_dev, CLOCK_TIMER_CTRL_STOP, RT_NULL);
    rt_device_close(hw_dev);

    return RT_EOK;
}
MSH_CMD_EXPORT(hwtimer_sample, test N32H49x 32-bit GTIM1 hwtimer);

#endif
