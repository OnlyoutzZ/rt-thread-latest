/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2023-12-03     Meco Man     support nano version
 * 2026-09-03     ox-horse     N32H47x_48x XSPI/QSPI test demo
 */

#include <board.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <string.h>
#include "drv_qspi.h"

#ifndef RT_USING_NANO
#include <rtdevice.h>
#endif /* RT_USING_NANO */

/*
 * N32H47x_48x XSPI pinout used by this demo (adjust to your board):
 *
 *   signal      pin     AF
 *   --------------------------
 *   XSPI_NSS1   PF0     AF9   (chip select, slave 1)
 *   XSPI_CLK    PF10    AF9
 *   XSPI_IO0    PF8     AF9
 *   XSPI_IO1    PF9     AF9
 *   XSPI_IO2    PF7     AF9
 *   XSPI_IO3    PF6     AF9
 */
#define XSPI_FLASH_NSS_PIN    (GPIO_PIN_0)
#define XSPI_FLASH_NSS_PORT   (GPIOF)
#define XSPI_FLASH_NSS_AF     (GPIO_AF9)

#define XSPI_FLASH_SCK_PIN    (GPIO_PIN_10)
#define XSPI_FLASH_SCK_PORT   (GPIOF)
#define XSPI_FLASH_SCK_AF     (GPIO_AF9)

#define XSPI_FLASH_D0_PIN     (GPIO_PIN_8)
#define XSPI_FLASH_D0_PORT    (GPIOF)
#define XSPI_FLASH_D0_AF      (GPIO_AF9)

#define XSPI_FLASH_D1_PIN     (GPIO_PIN_9)
#define XSPI_FLASH_D1_PORT    (GPIOF)
#define XSPI_FLASH_D1_AF      (GPIO_AF9)

#define XSPI_FLASH_D2_PIN     (GPIO_PIN_7)
#define XSPI_FLASH_D2_PORT    (GPIOF)
#define XSPI_FLASH_D2_AF      (GPIO_AF9)

#define XSPI_FLASH_D3_PIN     (GPIO_PIN_6)
#define XSPI_FLASH_D3_PORT    (GPIOF)
#define XSPI_FLASH_D3_AF      (GPIO_AF9)

/* SCK = Fssi_clk / (QSPI_BAUDR * 2), adjust to your system clock (driver applies BAUD = baudr<<1) */
#define QSPI_BAUDR            80

/* P25Q40H JEDEC ID (85 60 13) */
#define W25Q_CMD_READ_JEDEC_ID    0x9F
#define P25Q40_JEDEC_ID           0x00856013

#define QSPI_W25Q_DEVICE_NAME     "qspi10"

struct rt_qspi_message msg;
struct rt_qspi_device *qspi_dev;
struct rt_qspi_configuration qspi_cfg;

static void QSPI_GPIO_Configuration(void)
{
    
    /* GPIOA, GPIOC, GPIOD clock enable */
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA|RCC_AHB_PERIPHEN_GPIOC|RCC_AHB_PERIPHEN_GPIOD ,ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);
    
    /* XSPI clock enable */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_XSPI, ENABLE);
    
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);
    /* Confugure NSS0 pins */   
    GPIO_InitStructure.Pin       = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SR_SLOW_SLEW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
			
    /* Confugure SCK pin  */
    GPIO_InitStructure.Pin       = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    
    /* Confugure IO0\IO1 pin  */
    GPIO_InitStructure.Pin       = GPIO_PIN_6| GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF15;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    
    /* Confugure IO2\IO3 pin  */
    GPIO_InitStructure.Pin       = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
}

static struct n32_xspi_config _w25q_xspi_cfg =
{
    .scph              = XSPI_CTRL0_SCPH_FIRST_EDGE,        /* SPI mode 0 */
    .scpol             = XSPI_CTRL0_SCPOL_LOW,
    .role              = XSPI_Mode_Master,
    .frame_format      = XSPI_CTRL0_SPIFRF_STANDARD_FORMAT, /* standard SPI (single line) */
    .data_frame_size   = XSPI_CTRL0_DFS_8_BIT,
    .transfer_mode     = XSPI_CTRL0_TMOD_TX_AND_RX,
    .baudr             = QSPI_BAUDR,
    .rxd_sampling_edge = 0,
    .rxd_sample_delay  = 0,
    .nss_toggle        = 0,                                 /* SSTE disabled */
    .slave_sel         = XSPI_SLAVE_EN_SEN_0,               /* NSS0 -> PD3 */
};

static int qspi_w25q_attach(void)
{
    /* board-level gpio init */
    QSPI_GPIO_Configuration();

    /* attach P25Q40H to xspi bus in standard SPI mode (1 data line) */
    if (rt_hw_xspi_device_attach("qspi", QSPI_W25Q_DEVICE_NAME, 1,
                                 RT_NULL, RT_NULL, &_w25q_xspi_cfg) != RT_EOK)
    {
        rt_kprintf("attach %s failed!\n", QSPI_W25Q_DEVICE_NAME);
        return -RT_ERROR;
    }

    rt_kprintf("attach %s on xspi success\n", QSPI_W25Q_DEVICE_NAME);
    return RT_EOK;
}

void GPIO_HD_WP_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Confugure IO2 WP pin  */
    GPIO_InitStructure.Pin       = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SR_SLOW_SLEW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    
    /* Confugure IO3 HD pin  */
    GPIO_InitStructure.Pin       = GPIO_PIN_5;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    
    GPIO_WriteBits(GPIOC, GPIO_PIN_4, Bit_SET);
    GPIO_WriteBits(GPIOC, GPIO_PIN_5, Bit_SET);
}

/* Reconnect IO2 (WP) / IO3 (HOLD) to the XSPI peripheral for quad mode.
 * Call after Flash_Quad_Mode_Enable(): once QE is set these pins become
 * IO2/IO3 data lines and must no longer be held as GPIO output. */
static void GPIO_HD_WP_AF_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    GPIO_InitStructure.Pin            = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SR_SLOW_SLEW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_12mA;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
}

static void Flash_ENABLE_test(void)
{
    rt_uint8_t fwe = 0x06;
    rt_uint8_t send_reg1[2] = {0x05, 0xFF};
    rt_uint8_t read_reg1[2] = {0x00, 0x00};

    rt_memset(&msg, 0, sizeof(msg));

    msg.instruction.qspi_lines = 1;
    msg.qspi_data_lines        = 1;
    msg.parent.send_buf        = &fwe;
    msg.parent.recv_buf        = RT_NULL;
    msg.parent.length          = 1;
    msg.parent.cs_take         = 1;
    msg.parent.cs_release      = 1;
    msg.parent.next            = RT_NULL;

    if (rt_qspi_transfer_message(qspi_dev, &msg) != 1)
    {
        rt_kprintf("write enable failed!\n");
    }

    do
    {
        rt_memset(&msg, 0, sizeof(msg));

        msg.instruction.qspi_lines = 1;
        msg.qspi_data_lines        = 1;
        msg.parent.send_buf        = send_reg1;
        msg.parent.recv_buf        = read_reg1;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("read status failed!\n");
        }
    } while ((read_reg1[1] & 0x03) != 0x02);
}

static void Flash_Check_Busy_TEST(void)
{
    rt_uint8_t send_reg1[2] = {0x05, 0xFF};
    rt_uint8_t read_reg1[2] = {0x00, 0x00};

    do
    {
        rt_memset(&msg, 0, sizeof(msg));

        msg.instruction.qspi_lines = 1;
        msg.qspi_data_lines        = 1;
        msg.parent.send_buf        = send_reg1;
        msg.parent.recv_buf        = read_reg1;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("read status failed!\n");
        }
    } while (read_reg1[1] & 0x01);  /* wait while WIP = 1 */
}

static void Flash_Sector_Erase_TEST(uint32_t SectorAddr)
{
    rt_uint8_t bufw[4] = {0};

    Flash_ENABLE_test();

    rt_memset(&msg, 0, sizeof(msg));

    bufw[0] = 0x20;
    bufw[1] = (SectorAddr & 0xff0000) >> 16;
    bufw[2] = (SectorAddr & 0xff00) >> 8;
    bufw[3] = SectorAddr & 0xff;

    msg.instruction.qspi_lines = 1;
    msg.qspi_data_lines        = 1;
    msg.parent.send_buf        = bufw;
    msg.parent.recv_buf        = RT_NULL;
    msg.parent.length          = 4;
    msg.parent.cs_take         = 1;
    msg.parent.cs_release      = 1;
    msg.parent.next            = RT_NULL;

    if (rt_qspi_transfer_message(qspi_dev, &msg) != 4)
    {
        rt_kprintf("sector erase failed!\n");
    }

    Flash_Check_Busy_TEST();
}

static void Flash_Quad_Mode_Enable(void)
{
    uint8_t bufw[4] = {0};
    uint8_t bufr[4] = {0};

    Flash_ENABLE_test();

    do
    {
        rt_memset(&msg, 0, sizeof(msg));

        bufw[0] = 0x31;   /* Write Status Register-1 */
        bufw[1] = 0x02;   /* QE = S9 = bit1 */
        msg.instruction.qspi_lines = 1;
        msg.qspi_data_lines        = 1;
        msg.parent.send_buf        = bufw;
        msg.parent.recv_buf        = RT_NULL;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("quad mode write failed!\n");
        }

        rt_memset(&msg, 0, sizeof(msg));

        bufw[0] = 0x35;   /* Read Status Register-1 */
        bufw[1] = 0x00;
        msg.instruction.qspi_lines = 1;
        msg.qspi_data_lines        = 1;
        msg.parent.send_buf        = bufw;
        msg.parent.recv_buf        = bufr;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("quad mode read failed!\n");
        }
    } while ((bufr[1] & 0x02) != 0x02);
}

static int qspi_read_id(int argc, char **argv)
{
    rt_uint8_t id[4] = {0, 0, 0, 0};
    rt_uint8_t sendbuf[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    rt_uint8_t send_data[260] = {0};
    rt_uint8_t read_data[256] = {0};
    rt_uint8_t quad_wbuf[256] = {0};
    rt_uint8_t quad_rbuf[256] = {0};
    rt_uint8_t read_addr[4] = {0x03, 0x00, 0x00, 0x00};
    rt_uint32_t jedec_id;

    qspi_w25q_attach();

    qspi_dev = (struct rt_qspi_device *)rt_device_find(QSPI_W25Q_DEVICE_NAME);
    if (qspi_dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", QSPI_W25Q_DEVICE_NAME);
        return -RT_ERROR;
    }

    /* configure QSPI: mode 0, 8-bit data width, 1 data line */
    qspi_cfg.parent.mode       = RT_SPI_MODE_0;
    qspi_cfg.parent.data_width = 8;
    qspi_cfg.parent.reserved   = 0;
    qspi_cfg.medium_size       = 512 * 1024;   /* 512KB for P25Q40H (4Mbit) */
    qspi_cfg.ddr_mode          = 0;
    qspi_cfg.qspi_dl_width     = 1;

    if (rt_qspi_configure(qspi_dev, &qspi_cfg) != RT_EOK)
    {
        rt_kprintf("qspi configure failed!\n");
        return -RT_ERROR;
    }

    /* read JEDEC ID: send 0x9F command, read 3 bytes ID */
    rt_memset(&msg, 0, sizeof(msg));
    
    GPIO_HD_WP_Configuration();
    msg.instruction.qspi_lines = 1;
    msg.qspi_data_lines        = 1;
    msg.parent.send_buf        = sendbuf;
    msg.parent.recv_buf        = id;
    msg.parent.length          = 4;
    msg.parent.cs_take         = 1;
    msg.parent.cs_release      = 1;
    msg.parent.next            = RT_NULL;

    if (rt_qspi_transfer_message(qspi_dev, &msg) != sizeof(id))
    {
        rt_kprintf("read JEDEC ID failed!\n");
        return -RT_ERROR;
    }
     GPIO_Configuration();
    
    jedec_id = ((rt_uint32_t)id[1] << 16) | ((rt_uint32_t)id[2] << 8) | id[3];

    rt_kprintf("P25Q40H JEDEC ID: %02X %02X %02X\n", id[1], id[2], id[3]);

    if (jedec_id == P25Q40_JEDEC_ID)
    {
        rt_kprintf("ID check OK (0x%06X)\n", jedec_id);
    }
    else
    {
        rt_kprintf("ID check failed, expect 0x%06X, got 0x%06X\n",
                   P25Q40_JEDEC_ID, jedec_id);
        return -RT_ERROR;
    }
    GPIO_HD_WP_Configuration();
    Flash_Sector_Erase_TEST(0x00000000);

    rt_kprintf("\n XSPI Single Write and Read page Start \r\n");

    Flash_ENABLE_test();
    
//  GPIO_Configuration();
    for (uint16_t i = 0; i < 256; i++) send_data[i + 4] = i;
    send_data[0] = 0x02;   /* Page Program */
    send_data[1] = 0x00;
    send_data[2] = 0x00;
    send_data[3] = 0x00;
    rt_qspi_send(qspi_dev, send_data, 260);

    Flash_Check_Busy_TEST();

    rt_qspi_send_then_recv(qspi_dev, read_addr, 4, read_data, 256);

    rt_uint32_t single_err_cnt = 0;
    for (uint16_t i = 0; i < 256; i++)
    {
        if (read_data[i] != i)
        {
            if (single_err_cnt < 8)
            {
                rt_kprintf("single rd err: idx=%d, exp=0x%02X, rd=0x%02X\r\n",
                           i, i, read_data[i]);
            }
            single_err_cnt++;
        }
    }
    rt_kprintf("single read: %lu errors\r\n", (unsigned long)single_err_cnt);
    GPIO_Configuration();
    Flash_Quad_Mode_Enable();
    GPIO_HD_WP_AF_Configuration();

    /* ====== Quad (4-line) Write and Read Test ====== */
    rt_kprintf("\n XSPI Quad Write and Read page Start \r\n");

    {
        rt_uint32_t quad_addr = 0x00000200;

        qspi_cfg.parent.data_width = 8;
        if (rt_qspi_configure(qspi_dev, &qspi_cfg) != RT_EOK)
        {
            rt_kprintf("qspi configure failed!\n");
            return -RT_ERROR;
        }

        /* Quad Page Program (0x32): cmd=1-line, addr=1-line, data=4-line */
        for (uint16_t i = 0; i < 256; i++)
        {
            quad_wbuf[i] = 0xA0 + i;
        }

        Flash_ENABLE_test();

        rt_memset(&msg, 0, sizeof(msg));
        msg.instruction.content    = 0x32;     /* Quad Input Page Program */
        msg.instruction.qspi_lines = 1;
        msg.address.content        = quad_addr;
        msg.address.size           = 24;
        msg.address.qspi_lines     = 1;
        msg.qspi_data_lines        = 4;
        msg.parent.send_buf        = quad_wbuf;
        msg.parent.recv_buf        = RT_NULL;
        msg.parent.length          = 256;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 256)
        {
            rt_kprintf("Quad Page Program failed!\n");
            return -RT_ERROR;
        }

        rt_kprintf("Quad Page Program done, 256 bytes @ 0x%06X\n", (unsigned int)quad_addr);

        Flash_Check_Busy_TEST();

        /* Quad Read (0x6B): cmd=1, addr=1, dummy=8, data=4 */
        rt_memset(&msg, 0, sizeof(msg));
        msg.instruction.content    = 0x6B;
        msg.instruction.qspi_lines = 1;
        msg.address.content        = quad_addr;
        msg.address.size           = 24;
        msg.address.qspi_lines     = 1;
        msg.dummy_cycles           = 8;
        msg.qspi_data_lines        = 4;
        msg.parent.send_buf        = RT_NULL;
        msg.parent.recv_buf        = quad_rbuf;
        msg.parent.length          = 256;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 256)
        {
            rt_kprintf("Quad Read failed!\n");
            return -RT_ERROR;
        }
        rt_kprintf("Quad Read 256 bytes @ 0x%06X done\n", (unsigned int)quad_addr);

        /* verify data */
        rt_uint32_t quad_err_cnt = 0;

        for (uint16_t i = 0; i < 256; i++)
        {
            if (quad_rbuf[i] != quad_wbuf[i])
            {
                if (quad_err_cnt < 5)
                {
                    rt_kprintf("Quad verify err: idx=%d, wr=0x%02X, rd=0x%02X\n",
                               i, quad_wbuf[i], quad_rbuf[i]);
                }
                quad_err_cnt++;
            }
        }

        if (quad_err_cnt == 0)
        {
            rt_kprintf("Quad write/read verify OK! (256 bytes all match)\n");
        }
        else
        {
            rt_kprintf("Quad verify FAILED: %ld errors\n", (unsigned long)quad_err_cnt);
        }
    }

    return RT_EOK;
}

MSH_CMD_EXPORT(qspi_read_id, read P25Q40H JEDEC ID via QSPI);

#define LED0_PIN    GET_PIN(B, 1)

int main(void)
{
    GPIO_Configuration();
    USART_Configuration();
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
