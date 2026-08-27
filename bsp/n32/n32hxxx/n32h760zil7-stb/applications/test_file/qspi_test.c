#include "main.h"
#include "rtconfig.h"


#if defined(BSP_USING_QSPI2)

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_qspi.h"




uint8_t Single_Rbuffer[256]={0};
uint8_t Single_Wbuffer[260]={0};


/**
*\*\name    RCC_Configuration.
*\*\fun     Configures the different system clocks.
*\*\param   none
*\*\return  none
**/

void QSPI2_Configuration(void)
{
    /* Enable peripheral clocks --------------------------------------------------*/
    /* GPIOB, GPIOC, GPIOD, GPIOF clock enable */
    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB | RCC_AHB5_PERIPHEN_M7_GPIOF | RCC_AHB5_PERIPHEN_M7_GPIOA , ENABLE);
    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOG | RCC_AHB5_PERIPHEN_M7_GPIOH , ENABLE);
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI | RCC_AHB5_PERIPHEN_M7_GPIOK, ENABLE);
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO, ENABLE);

    /* XSPI clock enable */  
    RCC_ConfigPll2(RCC_PLL_SRC_HSI,64000000,416000000,ENABLE); /*XSPI == 416000000 */
    
    RCC_ConfigPLL2ADivider (RCC_PLLA_DIV1);
	RCC_ConfigXSPI2SSIClk(RCC_XSPISSICLK_SRC_PLL2A);
    
    RCC_EnableAXIPeriphClk4(RCC_AXI_PERIPHEN_M7_XSPI2   | RCC_AXI_PERIPHEN_M4_XSPI2, ENABLE);
    RCC_EnableAXIPeriphClk4(RCC_AXI_PERIPHEN_M7_XSPI2LP | RCC_AXI_PERIPHEN_M4_XSPI2LP, ENABLE);
}

void FLASH_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
     
    /* XSPI_NCS4 PI6    XSPI_IO0  PF8*/
    /* XSPI_IO1  PF9    XSPI_IO2  PI11*/ 
    /* XSPI_IO3  PI12   XSPI_CLK  PF10*/ 

     /* XSPI2 for EVB board  NCS4 */
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SR_FAST_SLEW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
    GPIO_InitPeripheral(GPIOI, &GPIO_InitStructure);

    /* Confugure SCK pin: PF10	AF2 */
    GPIO_InitStructure.Pin       = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1;
    GPIO_InitPeripheral(GPIOF, &GPIO_InitStructure);

    /* Confugure IO0\IO1  PF8\PF9   */
    GPIO_InitStructure.Pin       = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1;
    GPIO_InitPeripheral(GPIOF, &GPIO_InitStructure);
		

    /* Confugure IO2\IO3 pin: PI11\PI12   */
    GPIO_InitStructure.Pin       = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0;
    GPIO_InitPeripheral(GPIOI, &GPIO_InitStructure);
}



#define Baudr 80
/* W25Q128 JEDEC ID comman d and expected ID (EF 40 18) */
#define W25Q_CMD_READ_JEDEC_ID    0x9F
#define W25Q128_JEDEC_ID          0x00C46016

#define QSPI_W25Q_DEVICE_NAME     "qspi20"

struct rt_qspi_message msg;
struct rt_qspi_device *qspi_dev;
struct rt_qspi_configuration qspi_cfg;

static struct n32_xspi_config _w25q_xspi_cfg =
{
    .scph               = XSPI_SCPH_FIRST_EDGE,        /* SPI mode 0 */
    .scpol              = XSPI_SCPOL_LOW_LEVEL,
    .role               = XSPI_MASTER_ROLE,
    .frame_format       = XSPI_STANDARD_MODE,          /* standard SPI (single line) */
    .data_frame_size    = XSPI_FRAME_SIZE_8_BIT,
    .transfer_mode      = XSPI_TX_AND_RX_MODE,
    .baudr              = Baudr,                         /* 416MHz / 416 = 1MHz SCK */
    .rxd_sampling_edge  = XSPI_RXD_HCLK_RISING_SAMPLING,
    .rxd_sample_delay   = Baudr / 2,
    .nss_toggle         = XSPI_NSS_TOGGLE_DISABLE,
    .slave_sel          = XSPI_SELECT_SLAVE_4,
    .Enhance_clock_strech = XSPI_CLOCK_STRETCH_DISABLE,
};

static struct n32_xspi_config _w25q_xip_cfg =
{
    .scph               = XSPI_SCPH_FIRST_EDGE,        /* SPI mode 0 */
    .scpol              = XSPI_SCPOL_LOW_LEVEL,
    .role               = XSPI_MASTER_ROLE,
    .frame_format       = XSPI_QUAD_LINE_MODE,          /* standard SPI (single line) */
    .data_frame_size    = XSPI_FRAME_SIZE_32_BIT,
    .transfer_mode      = XSPI_RX_ONLY_MODE,
    .baudr              = Baudr,                         /* 416MHz / 416 = 1MHz SCK */
    .rxd_sampling_edge  =    XSPI_RXD_HCLK_RISING_SAMPLING,
    .rxd_sample_delay   =    3U,
    .nss_toggle         =   XSPI_NSS_TOGGLE_DISABLE,
    .slave_sel          =   XSPI_SELECT_SLAVE_4,
    .Enhance_clock_strech = XSPI_CLOCK_STRETCH_DISABLE,
    .Enhance_TransferType = XSPI_INST_SINGLE_LINE_ADDR_MULTI_LINE,
    .Enhance_AddrLen =      XSPI_ADDR_LEN_24BIT,
    .Enhance_InstructLen =  XSPI_INST_LEN_8BIT,
    .Enhance_WaitCycles =   XSPI_WAIT_4_CYCLES,
    .Enhance_DDR =          XSPI_DDR_DISABLE ,
};

static int qspi_w25q_attach(void)
{
    /* board-level clock and gpio init */
    QSPI2_Configuration();
    
    FLASH_GPIO_Configuration();

    /* attach W25Q128 to xspi2 bus in standard SPI mode (1 data line) */
    if (rt_hw_xspi_device_attach("qspi2", QSPI_W25Q_DEVICE_NAME, 1,
                                 RT_NULL, RT_NULL, &_w25q_xspi_cfg) != RT_EOK)
    {
        rt_kprintf("attach %s failed!\n", QSPI_W25Q_DEVICE_NAME);
        return -RT_ERROR;
    }

    rt_kprintf("attach %s on xspi2 success\n", QSPI_W25Q_DEVICE_NAME);
    return RT_EOK;
}


static int qspi_xip_attach(void)
{
    /* attach W25Q128 to xspi2 bus in quad mode (4 data lines, XIP ready) */
    if (rt_hw_xspi_device_attach("qspi2", "qspi21", 4,
                                 RT_NULL, RT_NULL, &_w25q_xip_cfg) != RT_EOK)
    {
        rt_kprintf("XIP attach failed!\n");
        return -RT_ERROR;
    }

    rt_kprintf("XIP attach %s on xspi2 success\n", "qspi20_xip");
    return RT_EOK;
}


void Flash_ENABLE_test()
{
    rt_uint8_t fwe = 0;
    rt_memset(&msg, 0, sizeof(msg));
    
    rt_uint8_t send_reg1[2] = {0x05,0xFF};
    rt_uint8_t read_reg1[2] = {0x00,0x00};
    
    fwe = 0x06;
    
    msg.instruction.qspi_lines = 1;      /* command on 1 line */
    msg.qspi_data_lines        = 1;      /* data on 1 line */
    msg.parent.send_buf        = &fwe;
    msg.parent.recv_buf        = RT_NULL;
    msg.parent.length          = 1;
    msg.parent.cs_take         = 1;
    msg.parent.cs_release      = 1;
    msg.parent.next            = RT_NULL;
    
    if (rt_qspi_transfer_message(qspi_dev, &msg) != 1)
    {
        rt_kprintf("write data failed!\n");
    } 
    
    do
    {
        msg.instruction.qspi_lines = 1;      /* command on 1 line */
        msg.qspi_data_lines        = 1;      /* data on 1 line */
        msg.parent.send_buf        = send_reg1;
        msg.parent.recv_buf        = read_reg1;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;
        
        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("write data failed!\n");
        }        
    
        
    }while(((read_reg1[1] & 0x03) != 0x02)); 
    
}

void Flash_Check_Busy_TEST()
{
    rt_memset(&msg, 0, sizeof(msg));
    
    rt_uint8_t send_reg1[2] = {0x05,0xFF};
    rt_uint8_t read_reg1[2] = {0x00,0x00}; 
    

    while (XSPI_GetFlagStatus(xSPI2, XSPI_TXFE_FLAG) != SET);


    while (XSPI_GetFlagStatus(xSPI2, XSPI_BUSY_FLAG) != RESET);

    
    do
    {
        rt_memset(&msg, 0, sizeof(msg));
        
        msg.instruction.qspi_lines = 1;      /* command on 1 line */
        msg.qspi_data_lines        = 1;      /* data on 1 line */
        msg.parent.send_buf        = send_reg1;
        msg.parent.recv_buf        = read_reg1;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;
        
        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("write data failed!\n");
        }        
       
    }while(((read_reg1[1] & 0x03) == 0x03));
}

void Flash_Sector_Erase_TEST(uint32_t SectorAddr)
{
    Flash_ENABLE_test();
    rt_memset(&msg, 0, sizeof(msg));
    rt_uint8_t bufw[4] = {0xff,0xff};
    /*Erase flash*/
    bufw[0] = 0x20;
    bufw[1] = (SectorAddr & 0xff0000) >> 16;
    bufw[2] = (SectorAddr & 0xff00) >> 8;
    bufw[3] = SectorAddr & 0xff;
    
    msg.instruction.qspi_lines = 1;      /* command on 1 line */
    msg.qspi_data_lines        = 1;      /* data on 1 line */
    msg.parent.send_buf        = bufw;
    msg.parent.recv_buf        = RT_NULL;
    msg.parent.length          = 4;
    msg.parent.cs_take         = 1;
    msg.parent.cs_release      = 1;
    msg.parent.next            = RT_NULL;
    
    if (rt_qspi_transfer_message(qspi_dev, &msg) != 4)
    {
        rt_kprintf("write data failed!\n");
    }  
    
    Flash_Check_Busy_TEST();
}

void GT25Q32_Quad_Mode_Enable(void)
{
    uint8_t bufw[4] = {0};
    uint8_t bufr[4] = {0};
    
    Flash_ENABLE_test();

    do
    {
        rt_memset(&msg, 0, sizeof(msg));
        
        bufw[0] = 0x31;
        bufw[1] = 0x02;
        msg.instruction.qspi_lines = 1;      /* command on 1 line */
        msg.qspi_data_lines        = 1;      /* data on 1 line */
        msg.parent.send_buf        = bufw;
        msg.parent.recv_buf        = RT_NULL;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;
        
        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("write data failed!\n");
        }      

        rt_memset(&msg, 0, sizeof(msg));
        
        bufw[0] = 0x35;
        bufw[1] = 0x00;
        msg.instruction.qspi_lines = 1;      /* command on 1 line */
        msg.qspi_data_lines        = 1;      /* data on 1 line */
        msg.parent.send_buf        = bufw;
        msg.parent.recv_buf        = bufr;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        msg.parent.next            = RT_NULL;
        
        if (rt_qspi_transfer_message(qspi_dev, &msg) != 2)
        {
            rt_kprintf("write data failed!\n");
        }         
    }while(((bufr[1] & 0x02) != 0x02));
    
}


        
static int qspi_read_id(int argc, char **argv)
{

    qspi_w25q_attach();
  
    rt_uint8_t id[4] = {0,0,0,0} ;
    
    rt_uint8_t sendbuf[4] = {0x9F,0xA5,0,0};
    
    rt_uint8_t send_data[260] ={0};
    rt_uint8_t read_data[256] ={0};
    rt_uint8_t quad_wbuf[256] = {0};
    rt_uint8_t quad_rbuf[256] = {0};
    rt_uint8_t read_addr[4] = {0x03,0x10,0x00,0x00};
    rt_uint32_t jedec_id;

    qspi_dev = (struct rt_qspi_device *)rt_device_find(QSPI_W25Q_DEVICE_NAME);
    if (qspi_dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", QSPI_W25Q_DEVICE_NAME);
        return -RT_ERROR;
    }

    /* configure QSPI: mode 0, 8-bit data width, 1 data line */
    qspi_cfg.parent.mode       = RT_SPI_MODE_0 ;
    qspi_cfg.parent.data_width = 8;
    qspi_cfg.parent.reserved   = 0;
    qspi_cfg.medium_size       = 16 * 1024 * 1024;  /* 16MB for W25Q128 */
    qspi_cfg.ddr_mode          = 0;
    qspi_cfg.qspi_dl_width     = 1;  /* single line */

    if (rt_qspi_configure(qspi_dev, &qspi_cfg) != RT_EOK)
    {
        rt_kprintf("qspi configure failed!\n");
        return -RT_ERROR;
    }

    /* build QSPI message: send 0x9F command, read 3 bytes ID */
    rt_memset(&msg, 0, sizeof(msg));

    msg.instruction.qspi_lines = 1;      /* command on 1 line */
    msg.qspi_data_lines        = 1;      /* data on 1 line */
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

    jedec_id = ((rt_uint32_t)id[1] << 16) | ((rt_uint32_t)id[2] << 8) | id[3];

    rt_kprintf("W25Q128 JEDEC ID: %02X %02X %02X\n", id[1], id[2], id[3]);

    if (jedec_id == W25Q128_JEDEC_ID)
    {
        rt_kprintf("ID check OK (0x%06X)\n", jedec_id);
    }
    else
    {
        rt_kprintf("ID check failed, expect 0x%06X, got 0x%06X\n",
                   W25Q128_JEDEC_ID, jedec_id);

        return -RT_ERROR;
    }

    Flash_Sector_Erase_TEST(0x00100000);
    
    rt_kprintf("\n XSPI Single Write and Read page Start \r\n");
    
    Flash_ENABLE_test();
    
    for(uint16_t i=0;i<256;i++) send_data[i+4] = i;
    send_data[0] = 0x02;
    send_data[1] = 0x10;
    rt_qspi_send(qspi_dev,send_data,260);

    Flash_Check_Busy_TEST();
    
    rt_qspi_send_then_recv(qspi_dev,read_addr,4,read_data,256);
    
    for(uint16_t i=0;i<256;i++)
    {
        if(read_data[i] != i)
        {
            rt_kprintf("data read error \r\n");
        }
    }

    GT25Q32_Quad_Mode_Enable();

    /* verify QE bit is actually set */
    {
        uint8_t sr2_tx[2] = {0x35, 0xFF};
        uint8_t sr2_rx[2] = {0, 0};
        rt_memset(&msg, 0, sizeof(msg));
        msg.instruction.qspi_lines = 1;
        msg.qspi_data_lines        = 1;
        msg.parent.send_buf        = sr2_tx;
        msg.parent.recv_buf        = sr2_rx;
        msg.parent.length          = 2;
        msg.parent.cs_take         = 1;
        msg.parent.cs_release      = 1;
        rt_qspi_transfer_message(qspi_dev, &msg);
        rt_kprintf("[QE check] Status Reg-2 = 0x%02X (QE=%s)\n",
                   sr2_rx[1], (sr2_rx[1] & 0x02) ? "ON" : "OFF");
    }

    /* ====== Quad (4-line) Write and Read Test ====== */
    rt_kprintf("\n XSPI Quad Write and Read page Start \r\n");

    {

        rt_uint32_t quad_addr = 0x00000200;

        /* 2. Write Enable for Quad Page Program */
       
        
        qspi_cfg.parent.data_width = 8;
        if (rt_qspi_configure(qspi_dev, &qspi_cfg) != RT_EOK)
        {
            rt_kprintf("qspi configure failed!\n");
            return -RT_ERROR;
        }
        /* 3. Quad Page Program (0x32): cmd=1-line, addr=1-line, data=4-line */
        for (uint16_t i = 0; i < 256; i++)
        {
            quad_wbuf[i] = 0xA0 + i;   /* fill with different pattern than single-line test */
        }

        Flash_ENABLE_test();
         
        rt_memset(&msg, 0, sizeof(msg));
        msg.instruction.content   = 0x32;     /* Quad Input Page Program */
        msg.instruction.qspi_lines = 1;        /* command on 1 line */
        msg.address.content       = quad_addr;
        msg.address.size          = 24;        /* 3-byte address */
        msg.address.qspi_lines    = 1;         /* address on 1 line */
        msg.qspi_data_lines       = 4;         /* data on 4 lines */
        msg.parent.send_buf       = quad_wbuf;
        msg.parent.recv_buf       = RT_NULL;
        msg.parent.length         = 256;
        msg.parent.cs_take        = 1;
        msg.parent.cs_release     = 1;
        msg.parent.next           = RT_NULL;

        if (rt_qspi_transfer_message(qspi_dev, &msg) != 256)
        {
            rt_kprintf("Quad Page Program failed!\n");
            return -RT_ERROR;
        }
        
        rt_kprintf("Quad Page Program done, 256 bytes @ 0x%06X\n", (unsigned int)quad_addr);

        Flash_Check_Busy_TEST();
     
        
        /* 5. Quad Read (0x6B): cmd=1, addr=1, dummy=8, data=4 */
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

        /* print read data (first 32 bytes) */
        rt_kprintf("[QUAD DEBUG] read  buf: ");
        for (int i = 0; i < 32; i++) rt_kprintf("%02X ", quad_rbuf[i]);
        rt_kprintf("\n");

        /* 6. Verify data (compare all 256 bytes) */
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




MSH_CMD_EXPORT(qspi_read_id, read W25Q128 JEDEC ID via QSPI);


/*
 * XIP 初始化 + 读 64 字节 + 退出
 *
 */
static int xip_demo(int argc, char **argv)
{
    qspi_xip_attach();
 
    qspi_dev = (struct rt_qspi_device *)rt_device_find("qspi21");
    if (qspi_dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", "qspi21");
        return -RT_ERROR;
    }
    
    uint8_t *xbase;
    struct xspi_xip_config cfg = {0};
    
    cfg.XipIncrOpcode         = (uint16_t)(0xEB & 0xFFFF);
    cfg.XipWrapOpcode         = (uint16_t)(0xEB & 0xFFFF);
    
    cfg.XipInstructLen        = XSPI_INST_LEN_8BIT;
    cfg.XipInstrctEnable      = XSPI_XIP_INSTRUCT_ENABLE;
    cfg.XipDFSHC              = XSPI_XIP_DFSHC_ENABLE;
    cfg.XipPrefetch           = XSPI_XIP_PREFETCH_ENABLE;
    cfg.XipModeBit            = XSPI_XIP_MODE_BIT_ENABLE;
    cfg.XipModeBitLen         = XSPI_XIP_MODE_BIT_LEN_8_BIT;
    cfg.XipModeBit_Data       = 0xF0;
    cfg.XipContinousTransfer  = XSPI_XIP_CONTINUE_TRANSFER_ENABLE;
    cfg.XipWatchDogTimeout    = 100U;

    if (rt_device_control((rt_device_t)qspi_dev, XSPI_CTRL_ENTER_XIP, &cfg) != RT_EOK)
    { 
        rt_kprintf("XIP enter fail\n"); 
        return -1; 
    }
    
    rt_device_control((rt_device_t)qspi_dev, XSPI_CTRL_GET_XIP_ADDR, &xbase);

    rt_kprintf("XIP: base=%p\n", xbase);
    rt_kprintf("Mapped[%p]: ", xbase);
    for (int i = 0; i < 64; i++) 
    {
        rt_kprintf("%X ", (xbase+0x00000200)[i+4]);
        if (i % 32 == 31) rt_kprintf("\n                           ");
    }

    rt_device_control((rt_device_t)qspi_dev, XSPI_CTRL_EXIT_XIP, RT_NULL);
    rt_kprintf("\nXIP exit\n");
    return 0;
}


MSH_CMD_EXPORT(xip_demo, XIP memory-map test);

#endif
