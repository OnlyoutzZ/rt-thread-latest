#include <rtthread.h>
#include <rtdevice.h>

#define PWM_DEV_NAME        "pwm14"  /* PWM???? */
#define PWM_DEV_CHANNEL     1       /* PWM?? */

struct rt_device_pwm *pwm_dev;      /* PWM???? */

int pwm_led_sample(void)
{
    rt_uint32_t period, pulse, dir;

    period = 500000;    /* ???0.5ms,?????ns */
    dir = 1;            /* PWM?????????? */
    pulse = 5000;          /* PWM?????,?????ns */

    /* ???? */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }

    /* ??PWM?????????? */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    /* ???? */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);


    return RT_EOK;
//    while (1)
//    {
//        rt_thread_mdelay(50);
//        if (dir)
//        {
//            pulse += 5000;      /* ?0???????5000ns */
//        }
//        else
//        {
//            pulse -= 5000;      /* ??????????5000ns */
//        }
//        if (pulse >= period)
//        {
//            dir = 0;
//        }
//        if (0 == pulse)
//        {
//            dir = 1;
//        }

//        /* ??PWM??????? */
//        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
//    }
}
/* ??? msh ????? */
MSH_CMD_EXPORT(pwm_led_sample, pwm sample);

