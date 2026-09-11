/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-08-28     ox-horse     first version
 *
 * N32H497ZGL7-EVB USBFSDEV HID keyboard test:
 *  - WKUP(PA0)/KEY1(PC13)/KEY2(PA15)/KEY3(PC8) -> type A/B/C/D to the host
 *  - host CapsLock -> LED2(PB3), host NumLock -> LED3(PA8)
 */

#include <rtthread.h>
#include <stdint.h>
#include <board.h>
#include <drv_gpio.h>
#include "usbd_core.h"
#include "usbd_hid.h"

#ifdef RT_CHERRYUSB_DEVICE_TEMPLATE_HID_KEYBOARD

/* N32H4x USB Full-Speed Device register base (USB_BASE, see usb_glue_nation.c) */
#define USBFS_REG_BASE 0x40004800UL

#define USBD_VID           0x19F5
#define USBD_PID           0x5560
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define HID_INT_EP      0x81
#define HID_OUT_EP      0x02
#define HID_EP_SIZE     8
#define HID_EP_INTERVAL 10

#define USB_CONFIG_SIZE               41
#define HID_KEYBOARD_REPORT_DESC_SIZE 63

/* Key map: WKUP -> 'A', KEY1 -> 'B', KEY2 -> 'C', KEY3 -> 'D' */
#define KEY_WKUP_PIN  GET_PIN(A, 0)
#define KEY_1_PIN     GET_PIN(C, 13)
#define KEY_2_PIN     GET_PIN(A, 15)
#define KEY_3_PIN     GET_PIN(C, 8)
#define KEY_WKUP_CODE (HID_KBD_USAGE_A + 0) /* 'A' = 0x04, 'B'-'Z' follow */
#define KEY_1_CODE    (HID_KBD_USAGE_A + 1) /* 'B' */
#define KEY_2_CODE    (HID_KBD_USAGE_A + 2) /* 'C' */
#define KEY_3_CODE    (HID_KBD_USAGE_A + 3) /* 'D' */

/* Host CapsLock -> LED2(PB3), host NumLock -> LED3(PA8) */
#define LED2_CAPS_PIN GET_PIN(B, 3)
#define LED3_NUM_PIN  GET_PIN(A, 8)

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0002, 0x01)
};

/* Standard boot-keyboard HID device with an extra OUT endpoint so the host
 * can send the LED output report (CapsLock/NumLock...). */
static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* interface: bNumEndpoints=2, HID, protocol=1 (keyboard) */
    0x09,
    USB_DESCRIPTOR_TYPE_INTERFACE,
    0x00,
    0x00,
    0x02,
    0x03,
    0x01,
    0x01,
    0x00,
    /* HID descriptor */
    0x09,
    HID_DESCRIPTOR_TYPE_HID,
    0x11,
    0x01,
    0x00,
    0x01,
    0x22,
    WBVAL(HID_KEYBOARD_REPORT_DESC_SIZE),
    /* IN endpoint (key report) */
    0x07,
    USB_DESCRIPTOR_TYPE_ENDPOINT,
    HID_INT_EP,
    0x03,
    WBVAL(HID_EP_SIZE),
    HID_EP_INTERVAL,
    /* OUT endpoint (LED report) */
    0x07,
    USB_DESCRIPTOR_TYPE_ENDPOINT,
    HID_OUT_EP,
    0x03,
    WBVAL(HID_EP_SIZE),
    HID_EP_INTERVAL,
};

static const uint8_t device_quality_descriptor[] = {
    /* device qualifier descriptor */
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid */
    "N32",                        /* Manufacturer */
    "N32H497 EVB HID Keyboard",   /* Product */
    "20260828",                   /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor usb_hid_kbd_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};

static const uint8_t hid_keyboard_report_desc[HID_KEYBOARD_REPORT_DESC_SIZE] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x06, // USAGE (Keyboard)
    0xa1, 0x01, // COLLECTION (Application)
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x75, 0x01, // REPORT_SIZE (1)
    0x95, 0x08, // REPORT_COUNT (8)
    0x81, 0x02, // INPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x08, // REPORT_SIZE (8)
    0x81, 0x03, // INPUT (Cnst,Var,Abs)
    0x95, 0x05, // REPORT_COUNT (5)
    0x75, 0x01, // REPORT_SIZE (1)
    0x05, 0x08, // USAGE_PAGE (LEDs)
    0x19, 0x01, // USAGE_MINIMUM (Num Lock)
    0x29, 0x05, // USAGE_MAXIMUM (Kana)
    0x91, 0x02, // OUTPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x03, // REPORT_SIZE (3)
    0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
    0x95, 0x06, // REPORT_COUNT (6)
    0x75, 0x08, // REPORT_SIZE (8)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0xFF, // LOGICAL_MAXIMUM (255)
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00, // INPUT (Data,Ary,Abs)
    0xc0        // END_COLLECTION
};

/* Keyboard input report: [modifier, reserved, key0..key5] */
static uint8_t key_report[8] = { 0 };
static volatile uint8_t send_busy;

/* OUT report buffer: bit0=NumLock, bit1=CapsLock, bit2=ScrollLock... */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t led_report[HID_EP_SIZE];

static void usbd_hid_int_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    send_busy = 0;
}

static void usbd_hid_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    if (nbytes >= 1)
    {
        rt_pin_write(LED2_CAPS_PIN, (led_report[0] & 0x02) ? PIN_HIGH : PIN_LOW);
        rt_pin_write(LED3_NUM_PIN, (led_report[0] & 0x01) ? PIN_HIGH : PIN_LOW);
    }
    usbd_ep_start_read(busid, ep, led_report, HID_EP_SIZE);
}

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event)
    {
    case USBD_EVENT_CONFIGURED:
        send_busy = 0;
            /* arm the first OUT transfer to receive the LED report */
        usbd_ep_start_read(busid, HID_OUT_EP, led_report, HID_EP_SIZE);
        break;
    default:
        break;
    }
}

static struct usbd_endpoint hid_in_ep = {
    .ep_cb = usbd_hid_int_callback,
    .ep_addr = HID_INT_EP
};

static struct usbd_endpoint hid_out_ep = {
    .ep_cb = usbd_hid_out_callback,
    .ep_addr = HID_OUT_EP
};

static struct usbd_interface intf0;

static void usb_hid_keyboard_init(uint8_t busid, uintptr_t reg_base)
{
    usbd_desc_register(busid, &usb_hid_kbd_descriptor);

    usbd_add_interface(busid, usbd_hid_init_intf(busid, &intf0, hid_keyboard_report_desc, HID_KEYBOARD_REPORT_DESC_SIZE));
    usbd_add_endpoint(busid, &hid_in_ep);
    usbd_add_endpoint(busid, &hid_out_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

/* Compose the 8-byte key report from the pressed-key bit mask and send it.
 * A press with no key (mask == 0) means "release everything". */
static void send_key_report(uint8_t keys)
{
    uint8_t idx = 2;

    if (send_busy || !usb_device_is_configured(0))
    {
        return;
    }

    memset(&key_report[2], 0, 6);
    if (keys & 0x01)
    {
        key_report[idx++] = KEY_WKUP_CODE;
    }
    if (keys & 0x02)
    {
        key_report[idx++] = KEY_1_CODE;
    }
    if (keys & 0x04)
    {
        key_report[idx++] = KEY_2_CODE;
    }
    if (keys & 0x08)
    {
        key_report[idx++] = KEY_3_CODE;
    }

    send_busy = 1;
    usbd_ep_start_write(0, HID_INT_EP, key_report, 8);
}

static void key_scan_thread_entry(void *param)
{
    uint8_t last = 0;
    uint8_t cur;

    while (1)
    {
        rt_thread_mdelay(10);

        cur = 0;
        if (rt_pin_read(KEY_WKUP_PIN) == PIN_LOW)
        {
            cur |= 0x01;
        }
        if (rt_pin_read(KEY_1_PIN) == PIN_LOW)
        {
            cur |= 0x02;
        }
        if (rt_pin_read(KEY_2_PIN) == PIN_LOW)
        {
            cur |= 0x04;
        }
        if (rt_pin_read(KEY_3_PIN) == PIN_LOW)
        {
            cur |= 0x08;
        }

        if (cur != last)
        {
            rt_thread_mdelay(10); /* debounce */
            last = cur;
            send_key_report(cur);
        }
    }
}

static int cherryusb_hid_keyboard_init(void)
{
    rt_thread_t tid;

    /* board keys: input with pull-up (pressed = low) */
    rt_pin_mode(KEY_WKUP_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY_1_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY_2_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY_3_PIN, PIN_MODE_INPUT_PULLUP);

    /* LED2/LED3: output, off initially */
    rt_pin_mode(LED2_CAPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED3_NUM_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED2_CAPS_PIN, PIN_LOW);
    rt_pin_write(LED3_NUM_PIN, PIN_LOW);

    usb_hid_keyboard_init(0, USBFS_REG_BASE);
    rt_kprintf("cherryusb fsdev hid keyboard test started.\r\n");

    tid = rt_thread_create("key_scan",
                           key_scan_thread_entry,
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
        rt_kprintf("failed to create key scan thread.\r\n");
    }

    return 0;
}
INIT_APP_EXPORT(cherryusb_hid_keyboard_init);

#endif /* RT_CHERRYUSB_DEVICE_TEMPLATE_HID_KEYBOARD */
