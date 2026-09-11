/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-08-28     ox-horse     first version
 */

#include <rtthread.h>
#include <stdint.h>

#ifdef RT_CHERRYUSB_DEVICE_TEMPLATE_HID_CUSTOM

#include "usbd_core.h"

/* N32H4x USB Full-Speed Device register base (USB_BASE, see usb_glue_nation.c) */
#define USBFS_REG_BASE 0x40004800UL

/* Keep in sync with hid_custom_inout_template.c (FS: 63-byte report + id) */
#define HIDRAW_IN_EP           0x81
#define HID_CUSTOM_REPORT_SIZE 63

extern void hid_custom_init(uint8_t busid, uintptr_t reg_base);

/* IN report buffer: [report id 0x02][63-byte payload] */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usb_hid_send_buffer[HID_CUSTOM_REPORT_SIZE + 1];

static void hid_custom_thread_entry(void *param)
{
    uint8_t counter = 0;

    /* Device -> host direction: push one 63-byte report every 2 s once the
     * host has enumerated the device (payload carries a running counter so
     * changes are visible on the host side).
     * Host -> device direction is exercised by the loop-back inside
     * hid_custom_inout_template.c: OUT ep 0x02 data is echoed back on
     * IN ep 0x81. */
    while (1)
    {
        if (usb_device_is_configured(0))
        {
            memset(usb_hid_send_buffer, 0, sizeof(usb_hid_send_buffer));
            usb_hid_send_buffer[0] = 0x02; /* IN report id */
            for (int i = 1; i < sizeof(usb_hid_send_buffer); i++)
            {
                usb_hid_send_buffer[i] = (uint8_t)(counter + i);
            }
            usbd_ep_start_write(0, HIDRAW_IN_EP, usb_hid_send_buffer, sizeof(usb_hid_send_buffer));
        }
        counter++;
        rt_thread_mdelay(2000);
    }
}

static int cherryusb_hid_custom_init(void)
{
    rt_thread_t tid;

    hid_custom_init(0, USBFS_REG_BASE);
    rt_kprintf("cherryusb fsdev hid custom inout example started.\r\n");

    tid = rt_thread_create("hid_io",
                           hid_custom_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("failed to create hid custom test thread.\r\n");
    }

    return 0;
}
INIT_APP_EXPORT(cherryusb_hid_custom_init);

#endif /* RT_CHERRYUSB_DEVICE_TEMPLATE_HID_CUSTOM */
