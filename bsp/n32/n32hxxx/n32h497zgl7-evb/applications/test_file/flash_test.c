/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-06-16     li.mengmeng      flash test for N32H7xx
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#ifdef BSP_USING_ON_CHIP_FLASH
#include "drv_flash.h"

#define FLASH_TEST_ADDR     (N32_FLASH_END_ADDRESS - (128 * 1024+256))  /* last 128KB for test */
#define FLASH_TEST_SIZE     (4096+256+102)                                   /* 4KB test area */

static rt_uint8_t  test_write_buf[FLASH_TEST_SIZE];
static rt_uint8_t  test_read_buf[FLASH_TEST_SIZE];

static void flash_test_fill_pattern(rt_uint8_t *buf, rt_uint32_t size, rt_uint32_t seed)
{
    for (rt_uint32_t i = 0; i < size; i++)
    {
        buf[i] = (rt_uint8_t)((i + seed) & 0xFF);
    }
}

static rt_bool_t flash_test_verify(rt_uint8_t *buf1, rt_uint8_t *buf2, rt_uint32_t size)
{
    for (rt_uint32_t i = 0; i < size; i++)
    {
        if (buf1[i] != buf2[i])
        {
            rt_kprintf("verify failed at offset %d: write=0x%02X read=0x%02X\n",
                       i, buf1[i], buf2[i]);
            return RT_FALSE;
        }
    }
    return RT_TRUE;
}

static int flash_test_read(int argc, char **argv)
{
    rt_uint32_t addr = FLASH_TEST_ADDR;
    rt_uint32_t size = FLASH_TEST_SIZE;
    int result;

    if (argc >= 2)
    {
        addr = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }
    if (argc >= 3)
    {
        size = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
    }

    rt_memset(test_read_buf, 0, sizeof(test_read_buf));

    result = n32_flash_read(addr, test_read_buf, size);
    if (result < 0)
    {
        rt_kprintf("flash read failed at 0x%08X, size=%d, err=%d\n", addr, size, result);
        return -RT_ERROR;
    }

    rt_kprintf("flash read success: addr=0x%08X, size=%d\n", addr, size);
    rt_kprintf("first 32 bytes: ");
    for (int i = 0; i < 32 && i < (int)size; i++)
    {
        rt_kprintf("%02X ", test_read_buf[i]);
    }
    rt_kprintf("\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(flash_test_read, flash read test: flash_test_read [addr] [size]);

static int flash_test_erase(int argc, char **argv)
{
    rt_uint32_t addr = FLASH_TEST_ADDR;
    rt_uint32_t size = FLASH_TEST_SIZE;
    int result;

    if (argc >= 2)
    {
        addr = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }
    if (argc >= 3)
    {
        size = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
    }

    result = n32_flash_erase(addr, size);
    if (result < 0)
    {
        rt_kprintf("flash erase failed at 0x%08X, size=%d, err=%d\n", addr, size, result);
        return -RT_ERROR;
    }

    rt_kprintf("flash erase success: addr=0x%08X, size=%d\n", addr, size);

    /* verify erased state (all 0xFF) */
    rt_memset(test_read_buf, 0, sizeof(test_read_buf));
    result = n32_flash_read(addr, test_read_buf, size);
    if (result > 0)
    {
        rt_bool_t erased = RT_TRUE;
        for (int i = 0; i < (int)size; i++)
        {
            if (test_read_buf[i] != 0xFF)
            {
                rt_kprintf("not fully erased at offset %d: 0x%02X\n", i, test_read_buf[i]);
                erased = RT_FALSE;
                break;
            }
        }
        if (erased)
        {
            rt_kprintf("verify erased OK: all 0xFF\n");
        }
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(flash_test_erase, flash erase test: flash_test_erase [addr] [size]);

static int flash_test_write(int argc, char **argv)
{
    rt_uint32_t addr = FLASH_TEST_ADDR;
    rt_uint32_t size = FLASH_TEST_SIZE;
    int result;

    if (argc >= 2)
    {
        addr = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }
    if (argc >= 3)
    {
        size = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
    }

    flash_test_fill_pattern(test_write_buf, size, 0xAA);

    result = n32_flash_write(addr, test_write_buf, size);
    if (result < 0)
    {
        rt_kprintf("flash write failed at 0x%08X, size=%d, err=%d\n", addr, size, result);
        return -RT_ERROR;
    }

    rt_kprintf("flash write success: addr=0x%08X, size=%d\n", addr, size);

    /* read back and verify */
    rt_memset(test_read_buf, 0, sizeof(test_read_buf));
    result = n32_flash_read(addr, test_read_buf, size);
    if (result > 0)
    {
        if (flash_test_verify(test_write_buf, test_read_buf, size))
        {
            rt_kprintf("verify OK: written data matches\n");
        }
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(flash_test_write, flash write test: flash_test_write [addr] [size]);

int flash_test_full(void)
{
    rt_uint32_t addr = FLASH_TEST_ADDR;
    rt_uint32_t size = FLASH_TEST_SIZE;
    int result;


    rt_kprintf("=== Flash Full Test ===\n");
    rt_kprintf("addr=0x%08X, size=%d\n", addr, size);

    /* step 1: erase */
    rt_kprintf("\n[1/3] Erasing...\n");
    result = n32_flash_erase(addr, size);
    if (result < 0)
    {
        rt_kprintf("FAIL: erase error %d\n", result);
        return -RT_ERROR;
    }
    rt_kprintf("PASS: erase OK\n");

    /* step 2: write pattern */
    rt_kprintf("\n[2/3] Writing...\n");
    flash_test_fill_pattern(test_write_buf, size, 0x5A);

    result = n32_flash_write(addr, test_write_buf, size);
    if (result < 0)
    {
        rt_kprintf("FAIL: write error %d\n", result);
        return -RT_ERROR;
    }
    rt_kprintf("PASS: write OK\n");

    /* step 3: verify */
    rt_kprintf("\n[3/3] Verifying...\n");
    rt_memset(test_read_buf, 0, sizeof(test_read_buf));
    result = n32_flash_read(addr, test_read_buf, size);
    if (result < 0)
    {
        rt_kprintf("FAIL: read error %d\n", result);
        return -RT_ERROR;
    }

    if (flash_test_verify(test_write_buf, test_read_buf, size))
    {
        rt_kprintf("PASS: verify OK\n");
    }
    else
    {
        rt_kprintf("FAIL: verify mismatch\n");
        return -RT_ERROR;
    }

    rt_kprintf("\n=== Flash Test All Passed ===\n");
    return RT_EOK;
}
MSH_CMD_EXPORT(flash_test_full, flash full test: erase / write / verify);

#endif /* BSP_USING_ON_CHIP_FLASH */
