#include <rtthread.h>
#include <rtdevice.h>
#include "n32h7xx_gpio.h"
#define PULSE_ENCODER_DEV_NAME    "pulse13"    /* ??????? */

int pulse_encoder_sample(void)
{
    rt_err_t ret = RT_EOK;
    rt_device_t pulse_encoder_dev = RT_NULL;   /* ????????? */
    rt_uint32_t index, index1;
    rt_int32_t count;

    GPIO_SetBits(GPIOF, GPIO_PIN_8);
    GPIO_SetBits(GPIOF, GPIO_PIN_9);
    /* ????????? */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }

    /* ????????? */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }


    for (index = 0; index <= 10; index ++)
    {
        for (index1 = 0; index1 <= 2; index1 ++)
        {
            GPIO_ResetBits(GPIOF, GPIO_PIN_8);
            rt_thread_mdelay(10);
            GPIO_ResetBits(GPIOF, GPIO_PIN_9);
            rt_thread_mdelay(50);
            GPIO_SetBits(GPIOF, GPIO_PIN_8);
            rt_thread_mdelay(10);
            GPIO_SetBits(GPIOF, GPIO_PIN_9);
            rt_thread_mdelay(50);
        }
//        rt_thread_mdelay(500);
        /* ?????????? */
        rt_device_read(pulse_encoder_dev, 0, &count, 1);
        /* ?????????? */
        rt_device_control(pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        rt_kprintf("get count %d\n", count);
    }

    rt_device_close(pulse_encoder_dev);
    return ret;
}
/* ??? msh ????? */
MSH_CMD_EXPORT(pulse_encoder_sample, pulse encoder sample);
