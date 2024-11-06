/**
 * @file ci130x_pwm.c
 * @brief  PWM驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include "ci130x_pwm.h"
#include "ci_assert.h"
#include "ci_log.h"
#include "platform_config.h"

#define PWM_EXIT_CLK  (get_osc_clk()/8)

typedef struct
{
	/*配置寄存器*/
	volatile unsigned int pwm_cfg;		/*offect:0x00;功能：分频系数、计数模式、时钟源、中断极性、中断信号宽度*/
	/*配置寄存器1*/
	volatile unsigned int pwm_cfg1;		/*offect:0x04;功能：重新计数、暂停计数、清除中断、当前计数值位置*/
	/*事件寄存器*/
	volatile unsigned int pwm_ew;		/*offect:0x08;功能：事件计数*/
	/*周期寄存器*/
	volatile unsigned int pwm_sc;		/*offect:0x0c;功能：计数周期值*/
	/*计数值寄存器*/
	volatile unsigned int pwm_cc;		/*offect:0x10;功能：当前计数值*/
	/*pwm周期寄存器*/
	volatile unsigned int pwm_spwmc;	/*offect:0x14;功能：pwm周期值*/
	/*配置寄存器0*/
	volatile unsigned int pwm_cfg0;		/*offect:0x18;功能：计数器时钟、timer_reset_in信号使能、timer_ru信号使能*/
    /*毛刺处理寄存器*/
    volatile unsigned int pwm_restart_md; /*offect:0x1c;功能：配置RES后是立即生效还是等待正在输出的PWM波完成以后才生效*/
}pwm_register_t;

/**
 * @brief pwm初始化
 *
 * @param base 外设基地址
 * @param init 初始化结构体
 */
void pwm_init(pwm_base_t base,pwm_init_t init)
{
    /* 功能实现 */
    pwm_register_t* pwm_p = (pwm_register_t*)base;
    /*配置pwm*/
	unsigned int temp = 0;
    temp &= ~(0x1 << 0);/*分频系数 不分频*/
    temp |= (0x1 << 2);/*计数模式 auto*/
    if(init.clk_sel == 0)
    {
    	temp &= ~(0x1 << 5);/*时钟源（0 pclk，1 exit）*/
    }
    else
    {
    	temp |= (0x1 << 5);/*时钟源（0 pclk，1 exit）*/
    }
	pwm_p->pwm_cfg = temp;
    pwm_p->pwm_cfg1 |= (0x1 << 3);/*当前计数值（1当前位置计数值，0当前位置前的计数值）*/
    if(init.clk_sel == 0)
    {
    	pwm_p->pwm_sc = get_apb_clk() / (init.freq);/*计数周期*/
    }
    else
    {
    	pwm_p->pwm_sc = get_src_clk() / 16 / (init.freq);/*计数周期，选src时钟时，默认16分频*/
    }
	pwm_p->pwm_spwmc = pwm_p->pwm_sc * (init.duty) / (init.duty_max);/*pwm周期*/
}

/**
 * @brief 启动pwm
 *
 * @param base 外设基地址
 */
void pwm_start(pwm_base_t base)
{
    /* 功能实现 */
    pwm_register_t* pwm_p = (pwm_register_t*)base;
    /*计数值复位*/
	pwm_p->pwm_cfg1 |= (0x1 << 0);/*重新计数（1重新计数，0无影响）*/
    /*开始正常计数*/
    pwm_p->pwm_cfg1 &= ~(0x1 << 1);/*暂停计数（1暂停计数，0正常计数）*/
}

/**
 * @brief 暂停pwm
 *
 * @param base 外设基地址
 */
void pwm_stop(pwm_base_t base)
{
    /* 功能实现 */
    pwm_register_t* pwm_p = (pwm_register_t*)base;
    /*暂停计数*/
	pwm_p->pwm_cfg1 |= (0x1 << 1);/*暂停计数（1暂停计数，0正常计数）*/
}

/**
 * @brief 设置pwm占空比
 *
 * @param base 外设基地址
 * @param duty 占空比
 * @param duty_max 最大占空比
 */
void pwm_set_duty(pwm_base_t base,unsigned int duty,unsigned int duty_max)
{
    /* 功能实现 */
    pwm_register_t* pwm_p = (pwm_register_t*)base;
    pwm_p->pwm_spwmc = pwm_p->pwm_sc * duty / duty_max;

    pwm_p->pwm_cfg1 |= (0x1 << 0);/*重新计数（1重新计数，0无影响）*/
}

/**
 * @brief 设置pwm restart_md 重新计数生效模式
 *
 * @param base 外设基地址
 * @param cmd 0，重新计数立即生效；1，等待正在进行的PWM波完整输出后重新计数才生效
 */
void pwm_set_restart_md(pwm_base_t base, uint8_t cmd)
{
    /* 功能实现 */
    pwm_register_t* pwm_p = (pwm_register_t*)base;

    if(cmd)
    {
        pwm_p->pwm_restart_md |= (0x1 << 0); /*重新计数（1：等待正在输出的PWM波完成以后才生效）*/
    }
    else
    {
        pwm_p->pwm_restart_md &= ~(0x1 << 0); /*重新计数（0：立即生效）*/
    }
}
