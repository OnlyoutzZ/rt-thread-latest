/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-07-10     ox-horse         NAND test for N32H7xx
 */

/*
 * NAND test commands (run in msh):
 *
 *   nand_test
 *       Erase block 0, verify all pages are 0xFF, write every page with a
 *       page-specific pattern, read back and compare.
 *
 *   nand_multi_test [block] [count]
 *       Multi-block test: for each block, erase -> write all pages ->
 *       read back and compare. Covers several blocks (higher row addresses).
 *         block: start block number, default 0
 *         count: number of blocks to test, default 2
 *       e.g.  nand_multi_test 100 5   -> test blocks 100..104
 *
 *   nand_erase_test
 *       Erase block 0 and verify all its pages are 0xFF.
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <drivers/mtd_nand.h>

#ifdef BSP_USING_NAND

#define NAND_DEVICE_NAME    "nand0"
#define NAND_TEST_ZONE      0
#define NAND_TEST_BLOCK     0
#define NAND_TEST_PAGE      0
#define NAND_TEST_BLOCK_NUM 2    /* number of blocks tested by nand_multi_test */

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
 * NAND test (SDK demo style), covering every page of the block:
 *   1. Erase block
 *   2. Read erased pages, verify all 0xFF
 *   3. Write each page with a page-specific test pattern
 *   4. Read back and compare
 */
void nand_test(void)
{
    struct rt_mtd_nand_device *nand;
    rt_uint32_t id;
    rt_err_t ret;
    rt_uint32_t i, p, status_pass;
    rt_uint32_t block = NAND_TEST_BLOCK;
    rt_uint32_t page_num;
    rt_uint32_t page_addr;
    rt_uint32_t page_size;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(NAND_DEVICE_NAME));
    if (!nand)
    {
        rt_kprintf("nand_test: device '%s' not found\n", NAND_DEVICE_NAME);
        return;
    }

    page_size = nand->page_size;
    page_num  = nand->pages_per_block;

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

    /* 3. Read every erased page, verify all 0xFF */
    rt_kprintf("Verify block %d erased pages (all 0xFF) ...\n", block);
    for (p = 0; p < page_num; p++)
    {
        page_addr = block * page_num + p;

        ret = rt_mtd_nand_read(nand, (rt_off_t)page_addr, rx_buffer, page_size, RT_NULL, 0);
        if (ret != RT_EOK)
        {
            rt_kprintf("  page %d: read FAIL (err=%d)\n", p, ret);
            return;
        }

        status_pass = 0;
        for (i = 0; i < page_size; i++)
        {
            if (rx_buffer[i] != 0xFF)
            {
                status_pass = 1;
                rt_kprintf("  page %d byte %d: expected 0xFF, got 0x%02X\n", p, i, rx_buffer[i]);
                break;
            }
        }
        if (status_pass)
        {
            rt_kprintf("FAIL (not all 0xFF after erase)\n");
            return;
        }
    }
    rt_kprintf("OK (all pages 0xFF)\n");

    /* 4. Write test pattern to every page (page-specific offset) */
    rt_kprintf("Write %d pages ... ", page_num);
    for (p = 0; p < page_num; p++)
    {
        page_addr = block * page_num + p;

        fill_buffer(tx_buffer, page_size, 0x0001 + p);

        ret = rt_mtd_nand_write(nand, (rt_off_t)page_addr, tx_buffer, page_size, RT_NULL, 0);
        if (ret != RT_EOK)
        {
            rt_kprintf("\n  page %d: write FAIL (err=%d)\n", p, ret);
            return;
        }
    }
    rt_kprintf("OK\n");

    /* 5. Read back and compare every page */
    rt_kprintf("Read back and compare %d pages ...\n", page_num);
    for (p = 0; p < page_num; p++)
    {
        page_addr = block * page_num + p;

        rt_memset(rx_buffer, 0, page_size);

        ret = rt_mtd_nand_read(nand, (rt_off_t)page_addr, rx_buffer, page_size, RT_NULL, 0);
        if (ret != RT_EOK)
        {
            rt_kprintf("  page %d: read FAIL (err=%d)\n", p, ret);
            return;
        }

        fill_buffer(tx_buffer, page_size, 0x0001 + p);
        i = buffer_compare(tx_buffer, rx_buffer, page_size);
        if (i != (rt_uint32_t) -1)
        {
            rt_kprintf("  page %d: mismatch at byte %d (wrote 0x%02X, read 0x%02X)\n",
                       p, i, tx_buffer[i], rx_buffer[i]);
            return;
        }
    }
    rt_kprintf("OK\n");

    rt_kprintf("\nnand_test: PASSED (block %d, %d pages)\n", block, page_num);
}
MSH_CMD_EXPORT(nand_test, nand flash test: erase+write+readback all pages in block 0);

/*
 * Multi-block NAND test: for each block, erase -> write all pages with
 * page-specific pattern -> read back and compare. Covers several blocks,
 * exercising higher row addresses across the device.
 *
 * msh: nand_multi_test [block] [count]
 *      block: start block number, defaults to NAND_TEST_BLOCK
 *      count: number of blocks to test, defaults to NAND_TEST_BLOCK_NUM
 */
static void nand_multi_test_run(rt_uint32_t block, rt_uint32_t count)
{
    struct rt_mtd_nand_device *nand;
    rt_uint32_t i, p, b;
    rt_uint32_t page_num, page_size;
    rt_uint32_t page_addr;
    rt_err_t ret;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(NAND_DEVICE_NAME));
    if (!nand)
    {
        rt_kprintf("nand_multi_test: device '%s' not found\n", NAND_DEVICE_NAME);
        return;
    }

    page_size = nand->page_size;
    page_num  = nand->pages_per_block;

    if (block + count > nand->block_total)
    {
        rt_kprintf("nand_multi_test: range [%d, %d) exceeds block_total %d\n",
                   block, block + count, nand->block_total);
        return;
    }

    rt_kprintf("nand_multi_test: %d block(s) from block %d\n", count, block);

    for (b = 0; b < count; b++)
    {
        rt_kprintf("\n[block %d] erase ... ", block + b);
        ret = rt_mtd_nand_erase_block(nand, block + b);
        if (ret != RT_EOK)
        {
            rt_kprintf("FAIL (err=%d)\n", ret);
            return;
        }
        rt_kprintf("OK\n");

        /* write every page with page-specific pattern */
        for (p = 0; p < page_num; p++)
        {
            page_addr = (block + b) * page_num + p;
            fill_buffer(tx_buffer, page_size, 0x0100 * (b + 1) + p);

            ret = rt_mtd_nand_write(nand, (rt_off_t)page_addr, tx_buffer, page_size, RT_NULL, 0);
            if (ret != RT_EOK)
            {
                rt_kprintf("  block %d page %d: write FAIL (err=%d)\n", block + b, p, ret);
                return;
            }
        }

        /* read back and compare every page */
        for (p = 0; p < page_num; p++)
        {
            page_addr = (block + b) * page_num + p;

            rt_memset(rx_buffer, 0, page_size);
            ret = rt_mtd_nand_read(nand, (rt_off_t)page_addr, rx_buffer, page_size, RT_NULL, 0);
            if (ret != RT_EOK)
            {
                rt_kprintf("  block %d page %d: read FAIL (err=%d)\n", block + b, p, ret);
                return;
            }

            fill_buffer(tx_buffer, page_size, 0x0100 * (b + 1) + p);
            i = buffer_compare(tx_buffer, rx_buffer, page_size);
            if (i != (rt_uint32_t) -1)
            {
                rt_kprintf("  block %d page %d: mismatch at byte %d (wrote 0x%02X, read 0x%02X)\n",
                           block + b, p, i, tx_buffer[i], rx_buffer[i]);
                return;
            }
        }
        rt_kprintf("[block %d] write+readback PASSED (%d pages)\n", block + b, page_num);
    }

    rt_kprintf("\nnand_multi_test: PASSED (%d blocks)\n", count);
}

static int cmd_nand_multi_test(int argc, char **argv)
{
    rt_uint32_t block = NAND_TEST_BLOCK;
    rt_uint32_t count = NAND_TEST_BLOCK_NUM;

    if (argc > 1)
    {
        block = (rt_uint32_t)strtoul(argv[1], RT_NULL, 0);
    }
    if (argc > 2)
    {
        count = (rt_uint32_t)strtoul(argv[2], RT_NULL, 0);
    }

    nand_multi_test_run(block, count);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_nand_multi_test, nand_multi_test, nand multi-block test from a start block);

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
MSH_CMD_EXPORT(nand_erase_test, erase a nand block and verify 0xFF);

#endif /* BSP_USING_NAND */
