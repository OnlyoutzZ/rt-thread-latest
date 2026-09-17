/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-19     ox-horse         PWM test for N32H49x
 */

#include <board.h>
#include <rtdevice.h>

#if defined(BSP_USING_PWM1) && defined(BSP_USING_PWM1_CH1)

#define PWM_DEV_NAME    "pwm1" /* ATIM1 */
#define PWM_DEV_CHANNEL 1

static void pwm_output_gpio_init(void)
{
    GPIO_InitType gpio;

    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPHEN_AFIO, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.Pin = GPIO_PIN_8;
    gpio.GPIO_Mode = GPIO_MODE_AF_PP;
    gpio.GPIO_Pull = GPIO_PULL_DOWN;
    gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    gpio.GPIO_Current = GPIO_DS_4mA;
    gpio.GPIO_Alternate = GPIO_AF_ATIM1_CH1_PA8;
    GPIO_InitPeripheral(GPIOA, &gpio);
}

static int pwm_sample(void)
{
    struct rt_device_pwm *pwm_dev;
    rt_uint32_t period = 500000;
    rt_uint32_t pulse = 250000;
    rt_err_t ret;

    pwm_output_gpio_init();

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    if (ret == RT_EOK)
    {
        ret = rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
    }

    rt_kprintf("[PWM] %s channel %u, period=%u ns, pulse=%u ns, ret=%d\n",
               PWM_DEV_NAME, PWM_DEV_CHANNEL, period, pulse, ret);
    return ret;
}
MSH_CMD_EXPORT(pwm_sample, test N32H49x ATIM1 PWM on PA8);

static void pwm_reg_dump(const char *tag)
{
    rt_kprintf("[PWM] %s ATIM1: base=0x%08X AHBPCLKEN=0x%08X AHB1PCLKEN=0x%08X CTRL1=0x%08X CCEN=0x%08X PSC=%u AR=%u CNT=%u CCDAT1=%u\n",
               tag,
               (unsigned int)ATIM1,
               (unsigned int)RCC->AHBPCLKEN,
               (unsigned int)RCC->AHB1PCLKEN,
               (unsigned int)ATIM1->CTRL1,
               (unsigned int)ATIM1->CCEN,
               (unsigned int)ATIM1->PSC,
               (unsigned int)ATIM1->AR,
               (unsigned int)ATIM1->CNT,
               (unsigned int)ATIM1->CCDAT1);
}

static int pwm_auto_start(void)
{
    struct rt_device_pwm *pwm_dev;
    rt_err_t ret;

    pwm_output_gpio_init();

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("[PWM] can't find %s\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    ret = rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, 500000, 250000);
    if (ret == RT_EOK)
    {
        ret = rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
    }
    rt_kprintf("[PWM] auto-start ret=%d\n", ret);

    pwm_reg_dump("after-enable");
    rt_thread_mdelay(100);
    pwm_reg_dump("after-100ms");

    return ret;
}
INIT_APP_EXPORT(pwm_auto_start);

#endif
