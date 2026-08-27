/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-07-10     ox-horse         NAND test for N32H7xx
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/mtd_nand.h>

#ifdef BSP_USING_NAND

#define NAND_DEVICE_NAME    "nand0"
#define NAND_TEST_ZONE      0
#define NAND_TEST_BLOCK     0
#define NAND_TEST_PAGE      0

#define NAND_BUFFER_SIZE    2048

static rt_uint8_t tx_buffer[NAND_BUFFER_SIZE];
static rt_uint8_t rx_buffer[NAND_BUFFER_SIZE];

static void fill_buffer(rt_uint8_t *buf, rt_uint32_t size, rt_uint32_t offset)
{
    for (rt_uint32_t i = 0; i < size; i++)
    {
        buf[i] = (rt_uint8_t)(i + offset);
    }
}

static int buffer_compare(const rt_uint8_t *buf1, const rt_uint8_t *buf2, rt_uint32_t size)
{
    for (rt_uint32_t i = 0; i < size; i++)
    {
        if (buf1[i] != buf2[i])
        {
            return i;  /* return first mismatch position */
        }
    }
    return -1;  /* all match */
}

/*
 * NAND test (SDK demo style):
 *   1. Erase block
 *   2. Read erased page, verify all 0xFF
 *   3. Write page with test pattern
 *   4. Read back and compare
 */
void nand_test(void)
{
    struct rt_mtd_nand_device *nand;
    rt_uint32_t id;
    rt_err_t ret;
    rt_uint32_t i, status_pass, status_fail;
    rt_uint32_t block = NAND_TEST_BLOCK;
    rt_uint32_t page  = NAND_TEST_PAGE;
    rt_uint32_t page_addr;
    rt_uint32_t page_size;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(NAND_DEVICE_NAME));
    if (!nand)
    {
        rt_kprintf("nand_test: device '%s' not found\n", NAND_DEVICE_NAME);
        return;
    }

    page_size = nand->page_size;
    page_addr = block * nand->pages_per_block + page;

    /* 1. Read NAND ID (SDK: FEMC_Nand_ReadID) */
    id = rt_mtd_nand_read_id(nand);
    rt_kprintf("NAND ID: 0x%08X\n", id);
    rt_kprintf("Device: page_size=%d, oob_size=%d, pages_per_block=%d, block_total=%d\n\n",
               nand->page_size, nand->oob_size, nand->pages_per_block, nand->block_total);

    /* 2. Erase block (SDK: FEMC_Nand_Erase_Block) */
    rt_kprintf("Erase block %d ... ", block);
    ret = rt_mtd_nand_erase_block(nand, block);
    if (ret != RT_EOK)
    {
        rt_kprintf("FAIL (err=%d)\n", ret);
        return;
    }
    rt_kprintf("OK\n");

    /* 3. Read erased page, verify all 0xFF (SDK: check after erase) */
    rt_kprintf("Read erased page %d ... ", page);
    ret = rt_mtd_nand_read(nand, (rt_off_t)page_addr, rx_buffer, page_size, RT_NULL, 0);
    if (ret != RT_EOK)
    {
        rt_kprintf("FAIL (err=%d)\n", ret);
        return;
    }

    status_fail = 0;
    for (i = 0; i < page_size; i++)
    {
        if (rx_buffer[i] != 0xFF)
        {
            status_fail = 1;
            rt_kprintf("  byte %d: expected 0xFF, got 0x%02X\n", i, rx_buffer[i]);
            break;
        }
    }
    if (status_fail)
    {
        rt_kprintf("FAIL (not all 0xFF after erase)\n");
        return;
    }
    rt_kprintf("OK (all 0xFF)\n");

    /* 4. Write test pattern (SDK: FEMC_Nand_WritePage) */
    rt_kprintf("Write page %d ... ", page);
    fill_buffer(tx_buffer, page_size, 0x0001);

    ret = rt_mtd_nand_write(nand, (rt_off_t)page_addr, tx_buffer, page_size, RT_NULL, 0);
    if (ret != RT_EOK)
    {
        rt_kprintf("FAIL (err=%d)\n", ret);
        return;
    }
    rt_kprintf("OK\n");

    /* 5. Read back and compare (SDK: FEMC_Nand_ReadPage + Buffer8cmp) */
    rt_kprintf("Read page %d ... ", page);
    rt_memset(rx_buffer, 0, page_size);

    ret = rt_mtd_nand_read(nand, (rt_off_t)page_addr, rx_buffer, page_size, RT_NULL, 0);
    if (ret != RT_EOK)
    {
        rt_kprintf("FAIL (err=%d)\n", ret);
        return;
    }

    i = buffer_compare(tx_buffer, rx_buffer, page_size);
    if (i != (rt_uint32_t) -1)
    {
        rt_kprintf("FAIL (mismatch at byte %d: wrote 0x%02X, read 0x%02X)\n",
                   i, tx_buffer[i], rx_buffer[i]);
        return;
    }
    rt_kprintf("OK\n");

    rt_kprintf("\nnand_test: PASSED\n");
}

/*
 * Standalone erase test: erase a block, then verify all pages are 0xFF.
 */
void nand_erase_test(void)
{
    struct rt_mtd_nand_device *nand;
    rt_uint32_t block = 0;
    rt_uint32_t p, i;
    rt_err_t ret;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(NAND_DEVICE_NAME));
    if (!nand)
    {
        rt_kprintf("nand_erase_test: device '%s' not found\n", NAND_DEVICE_NAME);
        return;
    }

    /* 1. Erase */
    rt_kprintf("Erase block %d ... ", block);
    ret = rt_mtd_nand_erase_block(nand, block);
    if (ret != RT_EOK)
    {
        rt_kprintf("FAIL (err=%d)\n", ret);
        return;
    }
    rt_kprintf("OK\n");

    /* 2. Verify all 0xFF */
    rt_kprintf("Verify block %d (all 0xFF) ...\n", block);
    for (p = 0; p < nand->pages_per_block; p++)
    {
        ret = rt_mtd_nand_read(nand, (rt_off_t)(block * nand->pages_per_block + p),
                               rx_buffer, nand->page_size, RT_NULL, 0);
        if (ret != RT_EOK)
        {
            rt_kprintf("  page %d: read FAIL (err=%d)\n", p, ret);
            return;
        }
        for (i = 0; i < nand->page_size; i++)
        {
            if (rx_buffer[i] != 0xFF)
            {
                rt_kprintf("  page %d byte %d: expected 0xFF, got 0x%02X\n", p, i, rx_buffer[i]);
                return;
            }
        }
    }
    rt_kprintf("PASSED (all 0xFF)\n");
}

#endif /* BSP_USING_NAND */
