/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-19     ox-horse         pulse encoder test for N32H47x_48x
 */

#include <board.h>
#include <rtdevice.h>

#if defined(RT_USING_PULSE_ENCODER) && defined(BSP_USING_PULSE_ENCODER13)

#define PULSE_ENCODER_DEV_NAME "pulse13" /* GTIM9 */

/* Connect PB8 to PA6 (GTIM9_CH1), and PB9 to PA7 (GTIM9_CH2). */
static void pulse_encoder_gpio_init(void)
{
    GPIO_InitType gpio;

    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOB, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.Pin = GPIO_PIN_6;
    gpio.GPIO_Mode = GPIO_MODE_INPUT;
    gpio.GPIO_Pull = GPIO_PULL_UP;
    gpio.GPIO_Alternate = GPIO_AF_GTIM9_CH1_PA6;
    GPIO_InitPeripheral(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.GPIO_Alternate = GPIO_AF_GTIM9_CH2_PA7;
    GPIO_InitPeripheral(GPIOA, &gpio);

    GPIO_InitStruct(&gpio);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    gpio.GPIO_Pull = GPIO_NO_PULL;
    gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    gpio.GPIO_Current = GPIO_DS_4mA;
    GPIO_InitPeripheral(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
}

static void pulse_encoder_generate_cycle(rt_bool_t reverse)
{
    if (reverse)
    {
        GPIO_ResetBits(GPIOB, GPIO_PIN_9);
        rt_thread_mdelay(10);
        GPIO_ResetBits(GPIOB, GPIO_PIN_8);
        rt_thread_mdelay(10);
        GPIO_SetBits(GPIOB, GPIO_PIN_9);
        rt_thread_mdelay(10);
        GPIO_SetBits(GPIOB, GPIO_PIN_8);
        rt_thread_mdelay(10);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_PIN_8);
        rt_thread_mdelay(10);
        GPIO_ResetBits(GPIOB, GPIO_PIN_9);
        rt_thread_mdelay(10);
        GPIO_SetBits(GPIOB, GPIO_PIN_8);
        rt_thread_mdelay(10);
        GPIO_SetBits(GPIOB, GPIO_PIN_9);
        rt_thread_mdelay(10);
    }
}

static int pulse_encoder_sample(void)
{
    rt_device_t encoder_dev;
    rt_int32_t forward_count = 0;
    rt_int32_t reverse_count = 0;
    rt_uint32_t index;
    rt_err_t ret;

    pulse_encoder_gpio_init();

    encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (encoder_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", PULSE_ENCODER_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_device_open(encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s failed: %d\n", PULSE_ENCODER_DEV_NAME, ret);
        return ret;
    }

    /* Start from a known state. The counter is not reset when the encoder is
     * opened, so a timer left free-running by someone else (e.g. the GTIM9 PWM
     * test, which shares this timer) would otherwise leak into the reading. */
    rt_device_control(encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);

    for (index = 0; index < 10; index++)
    {
        pulse_encoder_generate_cycle(RT_FALSE);
    }

    if (rt_device_read(encoder_dev, 0, &forward_count, 1) != 1)
    {
        ret = -RT_ERROR;
    }
    else
    {
        ret = rt_device_control(encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        if (ret == RT_EOK)
        {
            for (index = 0; index < 10; index++)
            {
                pulse_encoder_generate_cycle(RT_TRUE);
            }

            rt_kprintf("[ENCODER_REG] CTRL1=%08x SMCTRL=%08x STS=%08x DINTEN=%08x AR=%u CNT=%u IRQ_EN=%u IRQ_PEND=%u\n",
                       GTIM9->CTRL1,
                       GTIM9->SMCTRL,
                       GTIM9->STS,
                       GTIM9->DINTEN,
                       GTIM9->AR,
                       GTIM9->CNT,
                       NVIC_GetEnableIRQ(GTIM9_IRQn),
                       NVIC_GetPendingIRQ(GTIM9_IRQn));

            if (rt_device_read(encoder_dev, 0, &reverse_count, 1) != 1)
            {
                ret = -RT_ERROR;
            }
        }

        rt_kprintf("[ENCODER] %s forward=%d reverse=%d %s\n",
                   PULSE_ENCODER_DEV_NAME,
                   forward_count,
                   reverse_count,
                   (forward_count == 40 && reverse_count == -40) ? "PASS" : "FAIL");

        if (ret == RT_EOK && (forward_count != 40 || reverse_count != -40))
        {
            ret = -RT_ERROR;
        }
        rt_device_control(encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    }

    rt_device_close(encoder_dev);
    return ret;
}
MSH_CMD_EXPORT(pulse_encoder_sample, test N32H47x_48x GTIM9 pulse encoder);

#endif
