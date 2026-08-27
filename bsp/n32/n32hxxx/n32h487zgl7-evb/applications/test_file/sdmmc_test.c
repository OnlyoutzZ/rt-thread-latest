/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/classes/block.h>

#ifdef BSP_USING_SDIO

#define SD_TEST_DEVICE_NAME    "sd0"
#define SD_TEST_RETRY_MS       200
#define SD_TEST_RETRY_COUNT    50
#define SD_TEST_RW_BLOCKS      8U

static rt_uint32_t sd_test_get_le32(const rt_uint8_t *data)
{
    return (rt_uint32_t)data[0]
         | ((rt_uint32_t)data[1] << 8)
         | ((rt_uint32_t)data[2] << 16)
         | ((rt_uint32_t)data[3] << 24);
}

static void sd_test_dump_hex16(const char *label, const rt_uint8_t *data)
{
    rt_kprintf("  [DATA] %s: "
               "%02X %02X %02X %02X %02X %02X %02X %02X "
               "%02X %02X %02X %02X %02X %02X %02X %02X\n",
               label,
               data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7],
               data[8], data[9], data[10], data[11],
               data[12], data[13], data[14], data[15]);
}

static rt_bool_t sd_test_read_sector(rt_device_t device,
                                     rt_uint32_t sector_no,
                                     rt_uint8_t *buffer,
                                     rt_size_t sector_size)
{
    rt_size_t unchanged = 0;
    rt_size_t index;
    rt_ssize_t result;

    rt_memset(buffer, 0xa5, sector_size);
    result = rt_device_read(device, sector_no, buffer, 1);
    if (result != 1)
    {
        rt_kprintf("  [FAIL] read sector %lu: ret=%d\n",
                   sector_no, (int)result);
        return RT_FALSE;
    }

    for (index = 0; index < sector_size; index++)
    {
        if (buffer[index] == 0xa5)
            unchanged++;
    }

    rt_kprintf("  [PASS] sector %lu: sentinel=%lu/%lu, "
               "head=%08X %08X %08X %08X, tail=%02X%02X\n",
               sector_no, (rt_uint32_t)unchanged,
               (rt_uint32_t)sector_size,
               sd_test_get_le32(&buffer[0]),
               sd_test_get_le32(&buffer[4]),
               sd_test_get_le32(&buffer[8]),
               sd_test_get_le32(&buffer[12]),
               buffer[sector_size - 1], buffer[sector_size - 2]);
    return RT_TRUE;
}

static void sd_test_fill_pattern(rt_uint8_t *buffer, rt_size_t size)
{
    rt_size_t index;

    for (index = 0; index < size; index++)
    {
        buffer[index] = (rt_uint8_t)((index * 37U + 0x5aU) & 0xffU);
    }
}

static rt_bool_t sd_test_check_pattern(const rt_uint8_t *buffer,
                                       rt_size_t size)
{
    rt_size_t index;

    for (index = 0; index < size; index++)
    {
        if (buffer[index] !=
            (rt_uint8_t)((index * 37U + 0x5aU) & 0xffU))
        {
            rt_kprintf("  [FAIL] pattern mismatch at byte %lu: "
                       "read=%02X expected=%02X\n",
                       (rt_uint32_t)index, buffer[index],
                       (rt_uint8_t)((index * 37U + 0x5aU) & 0xffU));
            return RT_FALSE;
        }
    }

    return RT_TRUE;
}

void sdmmc_test_run(void)
{
    struct rt_device_blk_geometry geometry;
    rt_device_t device = RT_NULL;
    rt_uint8_t *backup = RT_NULL;
    rt_uint8_t *work = RT_NULL;
    rt_uint32_t sectors[4];
    rt_uint32_t test_sector;
    rt_uint32_t pass = 0;
    rt_uint32_t fail = 0;
    rt_uint32_t retry;
    rt_uint32_t index;
    rt_size_t test_size;
    rt_ssize_t result;
    rt_bool_t original_saved = RT_FALSE;

    for (retry = 0; retry < SD_TEST_RETRY_COUNT; retry++)
    {
        device = rt_device_find(SD_TEST_DEVICE_NAME);
        if (device != RT_NULL)
            break;
        rt_thread_mdelay(SD_TEST_RETRY_MS);
    }

    rt_kprintf("\n========================================\n");
    rt_kprintf("  SD Card Internal Read/Write Test\n");
    rt_kprintf("========================================\n");

    if (device == RT_NULL)
    {
        rt_kprintf("  [FAIL] %s not found after %d ms\n",
                   SD_TEST_DEVICE_NAME,
                   SD_TEST_RETRY_MS * SD_TEST_RETRY_COUNT);
        return;
    }

    if (rt_device_open(device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("  [FAIL] open %s failed\n", SD_TEST_DEVICE_NAME);
        return;
    }

    rt_memset(&geometry, 0, sizeof(geometry));
    if (rt_device_control(device, RT_DEVICE_CTRL_BLK_GETGEOME,
                          &geometry) != RT_EOK)
    {
        rt_kprintf("  [FAIL] get geometry failed\n");
        goto _close;
    }

    rt_kprintf("  [INFO] sectors=%lu, sector_size=%lu, block_size=%lu\n",
               (rt_uint32_t)geometry.sector_count,
               (rt_uint32_t)geometry.bytes_per_sector,
               (rt_uint32_t)geometry.block_size);

    if (geometry.sector_count < SD_TEST_RW_BLOCKS ||
        geometry.bytes_per_sector != 512U)
    {
        rt_kprintf("  [FAIL] unsupported geometry for this test\n");
        goto _close;
    }

    test_size = geometry.bytes_per_sector * SD_TEST_RW_BLOCKS;
    backup = rt_malloc(test_size);
    work = rt_malloc(test_size);
    if (backup == RT_NULL || work == RT_NULL)
    {
        rt_kprintf("  [FAIL] allocate %lu-byte test buffers failed\n",
                   (rt_uint32_t)test_size);
        goto _free;
    }

    if (sd_test_read_sector(device, 0, work,
                            geometry.bytes_per_sector))
    {
        pass++;
        sd_test_dump_hex16("000", &work[0]);
        sd_test_dump_hex16("016", &work[16]);
        rt_kprintf("  [INFO] sig=%02X%02X, oem='%.8s', "
                   "fat16='%.8s', fat32='%.8s'\n",
                   work[511], work[510], &work[3],
                   &work[54], &work[82]);
    }
    else
    {
        fail++;
    }

    sectors[0] = 1;
    sectors[1] = 2048;
    sectors[2] = (rt_uint32_t)geometry.sector_count / 2;
    sectors[3] = (rt_uint32_t)geometry.sector_count - 1;

    for (index = 0; index < sizeof(sectors) / sizeof(sectors[0]); index++)
    {
        if (sectors[index] >= geometry.sector_count)
            continue;

        if (sd_test_read_sector(device, sectors[index], work,
                                geometry.bytes_per_sector))
            pass++;
        else
            fail++;
    }

    test_sector = (rt_uint32_t)geometry.sector_count - SD_TEST_RW_BLOCKS;
    rt_kprintf("  [INFO] SDMA multi-block test: sector %lu..%lu (%u blocks)\n",
               test_sector, test_sector + SD_TEST_RW_BLOCKS - 1,
               SD_TEST_RW_BLOCKS);

    result = rt_device_read(device, test_sector, backup, SD_TEST_RW_BLOCKS);
    if (result != SD_TEST_RW_BLOCKS)
    {
        rt_kprintf("  [FAIL] backup read: ret=%d\n", (int)result);
        fail++;
        goto _summary;
    }
    original_saved = RT_TRUE;

    sd_test_fill_pattern(work, test_size);
    result = rt_device_write(device, test_sector, work, SD_TEST_RW_BLOCKS);
    if (result != SD_TEST_RW_BLOCKS)
    {
        rt_kprintf("  [FAIL] pattern write: ret=%d\n", (int)result);
        fail++;
        goto _restore;
    }

    rt_memset(work, 0, test_size);
    result = rt_device_read(device, test_sector, work, SD_TEST_RW_BLOCKS);
    if (result == SD_TEST_RW_BLOCKS &&
        sd_test_check_pattern(work, test_size))
    {
        rt_kprintf("  [PASS] multi-block write/read pattern verify\n");
        pass++;
    }
    else
    {
        if (result != SD_TEST_RW_BLOCKS)
            rt_kprintf("  [FAIL] pattern read: ret=%d\n", (int)result);
        fail++;
    }

_restore:
    result = rt_device_write(device, test_sector, backup, SD_TEST_RW_BLOCKS);
    if (result != SD_TEST_RW_BLOCKS)
    {
        rt_kprintf("  [FAIL] restore write: ret=%d; card tail data changed\n",
                   (int)result);
        fail++;
        goto _summary;
    }

    rt_memset(work, 0, test_size);
    result = rt_device_read(device, test_sector, work, SD_TEST_RW_BLOCKS);
    if (result == SD_TEST_RW_BLOCKS &&
        rt_memcmp(work, backup, test_size) == 0)
    {
        rt_kprintf("  [PASS] original data restored and verified\n");
        pass++;
    }
    else
    {
        rt_kprintf("  [FAIL] restored data verify failed: ret=%d\n",
                   (int)result);
        fail++;
    }

_summary:
    if (!original_saved)
        rt_kprintf("  [WARN] write test skipped because backup failed\n");
    rt_kprintf("  [SUMMARY] %lu PASS, %lu FAIL\n\n", pass, fail);

_free:
    if (work != RT_NULL)
        rt_free(work);
    if (backup != RT_NULL)
        rt_free(backup);
_close:
    rt_device_close(device);
}

#endif /* BSP_USING_SDIO */
