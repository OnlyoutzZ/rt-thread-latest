/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-07-28     ox-horse         lcd test
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

static void lcd_thread(void *arg)
{
    rt_device_t lcd_dev = (rt_device_t)arg;
    struct rt_device_graphic_info info;

    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &info);

    rt_uint32_t buf_size = info.width * info.height * info.bits_per_pixel / 8;

    while (1)
    {
        if (info.pixel_format == RTGRAPHIC_PIXEL_FORMAT_RGB565)
        {
            uint32_t *buf32 = (uint32_t *)info.framebuffer;
            /* red */
            for (int i = 0; i < buf_size / 4; i++)
            {
                buf32[i] = 0xF800F800;
            }
            rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
            rt_thread_mdelay(1000);
            /* green */
            for (int i = 0; i < buf_size / 4; i++)
            {
                buf32[i] = 0x07E007E0;
            }
            rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
            rt_thread_mdelay(1000);
            /* blue */
            for (int i = 0; i < buf_size / 4; i++)
            {
                buf32[i] = 0x001F001F;
            }
        }
        else if (info.pixel_format == RTGRAPHIC_PIXEL_FORMAT_RGB888)
        {
            uint8_t *buf = (uint8_t *)info.framebuffer;
            /* red */
            for (int i = 0; i < buf_size / 3; i++)
            {
                buf[3 * i] = 0x00;
                buf[3 * i + 1] = 0x00;
                buf[3 * i + 2] = 0xff;
            }
            rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
            rt_thread_mdelay(1000);
            /* green */
            for (int i = 0; i < buf_size / 3; i++)
            {
                buf[3 * i] = 0x00;
                buf[3 * i + 1] = 0xff;
                buf[3 * i + 2] = 0x00;
            }
            rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
            rt_thread_mdelay(1000);
            /* blue */
            for (int i = 0; i < buf_size / 3; i++)
            {
                buf[3 * i] = 0xff;
                buf[3 * i + 1] = 0x00;
                buf[3 * i + 2] = 0x00;
            }
        }

        rt_device_control(lcd_dev, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
        rt_thread_mdelay(1000);
    }
}

int lcd_test(void)
{
    rt_device_t lcd_dev = rt_device_find("lcd");
    if (lcd_dev == RT_NULL)
    {
        rt_kprintf("Failed to find LCD device!\n");
        return -RT_ERROR;
    }

    const char *thread_name = "lcd_test";
    rt_thread_t thread = rt_thread_create(thread_name, lcd_thread, lcd_dev, 4096,
                                          26, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("%s created failed.\n", thread_name);
        return -RT_ERROR;
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(lcd_test, Create thread test lcd);
