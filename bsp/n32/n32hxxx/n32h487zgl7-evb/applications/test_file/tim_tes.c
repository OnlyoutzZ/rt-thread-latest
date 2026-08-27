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
#include <drv_gpio.h>


#ifdef BSP_USING_CLOCK_TIMER


#define HWTIMER_DEV_NAME   "timer12"

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("this is hwtimer timeout callback fucntion!\n");
    rt_kprintf("tick is :%d !\n", rt_tick_get());

    return 0;
}

int hwtimer_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_clock_timerval_t timeout_s;      /* ?????? */
    rt_device_t hw_dev = RT_NULL;   /* ??????? */
    rt_clock_timer_mode_t mode;         /* ????? */
    rt_uint32_t freq = 10000;               /* ???? */

    /* ??????? */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* ????????? */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* ???????? */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    /* ??????(??????,???1Mhz ? ?????????) */
    rt_device_control(hw_dev, CLOCK_TIMER_CTRL_FREQ_SET, &freq);
    /* ???????????(????,???HWTIMER_MODE_ONESHOT)*/
    mode = CLOCK_TIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, CLOCK_TIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    /* ?????????5s?????? */
    timeout_s.sec = 5;      /* ? */
    timeout_s.usec = 0;     /* ?? */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    /* ??3500ms */
    rt_thread_mdelay(3500);

    /* ???????? */
    rt_device_read(hw_dev, 0, &timeout_s, sizeof(timeout_s));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", timeout_s.sec, timeout_s.usec);

    return ret;
}



#endif /* RT_USING_TIMER */
