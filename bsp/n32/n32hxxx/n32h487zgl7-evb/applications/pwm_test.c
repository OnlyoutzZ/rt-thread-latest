/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2026-08-19     ox-horse         PWM test for N32H47x_48x
 */

#include <board.h>
#include <rtdevice.h>
#include <drv_tim.h>

#if defined(BSP_USING_PWM13) && defined(BSP_USING_PWM13_CH1) && \
    defined(BSP_USING_PWM6) && defined(BSP_USING_PWM6_CH2)

#define PWM13_DEV_NAME     "pwm13" /* GTIM9 */
#define PWM13_DEV_CHANNEL  1
#define PWM13_PERIOD_NS    100000U
#define PWM13_PULSE_NS     30000U

#define PWM6_DEV_NAME      "pwm6" /* GTIM2 */
#define PWM6_DEV_CHANNEL   2
#define PWM6_PERIOD_NS     31250U
#define PWM6_PULSE_NS      12500U

static void pwm_output_gpio_init(void)
{
    GPIO_InitType gpio;

    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOB, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    GPIO_InitStruct(&gpio);
    gpio.GPIO_Mode = GPIO_MODE_AF_PP;
    gpio.GPIO_Pull = GPIO_PULL_DOWN;
    gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    gpio.GPIO_Current = GPIO_DS_4mA;

    gpio.Pin = GPIO_PIN_6;
    gpio.GPIO_Alternate = GPIO_AF_GTIM9_CH1_PA6;
    GPIO_InitPeripheral(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_5;
    gpio.GPIO_Alternate = GPIO_AF_GTIM2_CH2_PB5;
    GPIO_InitPeripheral(GPIOB, &gpio);
}

static rt_uint32_t pwm_compare_get(TIM_Module *timer, rt_uint32_t channel)
{
    if (channel == 1U)
    {
        return timer->CCDAT1;
    }

    return timer->CCDAT2;
}

static void pwm_register_dump(const char *name, TIM_Module *timer, rt_uint32_t channel)
{
    rt_uint32_t timer_clock = n32_tim_clock_freq_get(timer);
    rt_uint32_t prescaler = timer->PSC + 1U;
    rt_uint32_t period_count = timer->AR + 1U;
    rt_uint32_t compare = pwm_compare_get(timer, channel);
    rt_uint32_t frequency = timer_clock / prescaler / period_count;
    rt_uint32_t duty_x100 = (rt_uint32_t)(((rt_uint64_t)compare * 10000U) / period_count);

    rt_kprintf("[%s REG] CLK=%u CTRL1=0x%08x PSC=%u ARR=%u CCR%u=%u\n",
               name, timer_clock, timer->CTRL1, timer->PSC, timer->AR, channel, compare);
    rt_kprintf("[%s REG] CCMOD1=0x%08x CCEN=0x%08x BKDT=0x%08x CNT=%u\n",
               name, timer->CCMOD1, timer->CCEN, timer->BKDT, timer->CNT);
    rt_kprintf("[%s CALC] frequency=%u Hz, duty=%u.%02u%%\n",
               name, frequency, duty_x100 / 100U, duty_x100 % 100U);
}

static rt_err_t pwm_start(const char *name, rt_uint32_t channel,
                          rt_uint32_t period, rt_uint32_t pulse)
{
    struct rt_device_pwm *pwm_dev;
    rt_err_t ret;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(name);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("can't find %s\n", name);
        return -RT_ERROR;
    }

    ret = rt_pwm_set(pwm_dev, channel, period, pulse);
    if (ret == RT_EOK)
    {
        ret = rt_pwm_enable(pwm_dev, channel);
    }

    rt_kprintf("[PWM] %s channel %u, period=%u ns, pulse=%u ns, ret=%d\n",
               name, channel, period, pulse, ret);
    return ret;
}

static int pwm_sample(void)
{
    rt_err_t pwm13_ret;
    rt_err_t pwm6_ret;

    pwm_output_gpio_init();

    pwm13_ret = pwm_start(PWM13_DEV_NAME, PWM13_DEV_CHANNEL,
                          PWM13_PERIOD_NS, PWM13_PULSE_NS);
    pwm6_ret = pwm_start(PWM6_DEV_NAME, PWM6_DEV_CHANNEL,
                         PWM6_PERIOD_NS, PWM6_PULSE_NS);

    if ((pwm13_ret == RT_EOK) && (pwm6_ret == RT_EOK))
    {
        pwm_register_dump("PWM13", GTIM9, PWM13_DEV_CHANNEL);
        pwm_register_dump("PWM6", GTIM2, PWM6_DEV_CHANNEL);
        rt_kprintf("[GPIO REG] PA6 mode=%u af=%u, PB5 mode=%u af=%u\n",
                   (GPIOA->PMODE >> 12U) & 0x3U,
                   (GPIOA->AFSEL[1] >> 16U) & 0xFFU,
                   (GPIOB->PMODE >> 10U) & 0x3U,
                   (GPIOB->AFSEL[1] >> 8U) & 0xFFU);
        return RT_EOK;
    }

    return (pwm13_ret != RT_EOK) ? pwm13_ret : pwm6_ret;
}
MSH_CMD_EXPORT(pwm_sample, output PWM13 10kHz 30pct on PA6 and PWM6 32kHz 40pct on PB5);

#endif
