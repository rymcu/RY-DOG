/**
 * @file ci130x_timer.c
 * @brief  Timer驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include "ci130x_core_eclic.h"
#include "ci130x_timer.h"
#include "ci_assert.h"
#include "ci_log.h"

typedef struct
{
	/*配置寄存器*/
	volatile unsigned int timer_cfg;	/*offect:0x00;功能：分频系数、计数模式、时钟源、中断极性、中断信号宽度*/
	/*配置寄存器1*/
	volatile unsigned int timer_cfg1;	/*offect:0x04;功能：重新计数、暂停计数、清除中断、当前计数值位置*/
	/*事件寄存器*/
	volatile unsigned int timer_ew;		/*offect:0x08;功能：事件计数*/
	/*周期寄存器*/
	volatile unsigned int timer_sc;		/*offect:0x0c;功能：计数周期值*/
	/*计数值寄存器*/
	volatile unsigned int timer_cc;		/*offect:0x10;功能：当前计数值*/
	/*pwm周期寄存器*/
	volatile unsigned int timer_spwmc;	/*offect:0x14;功能：pwm周期值*/
	/*配置寄存器0*/
	volatile unsigned int timer_cfg0;	/*offect:0x18;功能：计数器时钟、timer_reset_in信号使能、timer_ru信号使能*/
}timer_register_t;

/**
 * @brief 定时器初始化
 *
 * @param base 外设基地址
 * @param init 初始化结构体
 */
void timer_init(timer_base_t base,timer_init_t init)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
    if(init.width != timer_iqr_width_f)
    {
        switch(base)
        {
            case TIMER0:
            {
                eclic_set_posedge_trig(TIMER0_IRQn);
                break;
            }
            case TIMER1:
            {
                eclic_set_posedge_trig(TIMER1_IRQn);
                break;
            }
            case TIMER2:
            {
                eclic_set_posedge_trig(TIMER2_IRQn);
                break;
            }
            case TIMER3:
            {
                eclic_set_posedge_trig(TIMER3_IRQn);
                break;
            }
            case AON_TIMER0:
            {
                eclic_set_posedge_trig(AON_TIM_INT0_IRQn);
                break;
            }
            case AON_TIMER1:
            {
                eclic_set_posedge_trig(AON_TIM_INT1_IRQn);
                break;
            }
        }
    }
    else
    {
        switch(base)
        {
            case TIMER0:
            {
            	eclic_set_level_trig(TIMER0_IRQn);
                break;
            }
            case TIMER1:
            {
            	eclic_set_level_trig(TIMER1_IRQn);
                break;
            }
            case TIMER2:
            {
            	eclic_set_level_trig(TIMER2_IRQn);
                break;
            }
            case TIMER3:
            {
            	eclic_set_level_trig(TIMER3_IRQn);
                break;
            }
            case AON_TIMER0:
            {
            	eclic_set_level_trig(AON_TIM_INT0_IRQn);
                break;
            }
            case AON_TIMER1:
            {
            	eclic_set_level_trig(AON_TIM_INT1_IRQn);
                break;
            }
        }
    }
    /*配置定时器*/
	unsigned int temp = 0;
    temp |= init.div;
    temp |= (init.mode << 2);
    temp &= ~(0x1 << 5);/*时钟源（0 pclk，1 src时钟）*/
    temp &= ~(0x1 << 6);/*中断极性(0高有效，1低有效)*/
    temp &= ~(0x3 << 7);
    temp |= (init.width << 7);/*中断信号宽度*/
	timer_p->timer_cfg = temp;
	temp = 0;
    temp |= (0x1 << 2);/*清除中断（1清除中断，0无影响）*/
    temp |= (0x1 << 3);/*当前计数值（1当前位置计数值，0当前位置前的计数值）*/
	timer_p->timer_cfg1 = temp;
    timer_p->timer_ew = 0x1;/*事件计数(0 无影响，1 计数器减一)*/
    timer_p->timer_sc = init.count;/*计数周期值*/
}

/**
 * @brief 设置计数模式
 *
 * @param base TIMER0/TIMER1/TIMER2
 * @param mode timer_mode_single/timer_mode_auto/timer_mode_free/timer_mode_event
 */
void timer_set_mode(timer_base_t base,timer_count_mode_t mode)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
    /*选择定时器模式*/
    timer_p->timer_cfg &= ~(7 << 2);
    timer_p->timer_cfg |= (mode << 2);
}

/**
 * @brief 启动定时器
 *
 * @param base TIMER0/TIMER1/TIMER2
 */
void timer_start(timer_base_t base)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
    /*计数值复位*/
	timer_p->timer_cfg1 |= (0x1 << 0);/*重新计数控制（1重新计数，0无影响）*/
    /*开始正常计数*/
    timer_p->timer_cfg1 &= ~(0x1 << 1);/*暂停计数控制（1暂停计数，0正常计数）*/
    /*清除中断*/
    timer_p->timer_cfg1 |= 1 << 2;
}

/**
 * @brief 暂停定时器
 *
 * @param base TIMER0/TIMER1/TIMER2
 */
void timer_stop(timer_base_t base)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
    timer_p->timer_cfg1 |= (0x1 << 1);/*暂停计数（1暂停计数，0正常计数）*/
    /*清除中断*/
    timer_p->timer_cfg1 |= 1 << 2;
}

/**
 * @brief 事件计数(必须处于事件计数模式下)
 *
 * @param base TIMER0/TIMER1/TIMER2
 */
void timer_event_start(timer_base_t base)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
    timer_p->timer_ew = 0x1;/*事件计数(0 无影响，1 计数器减一),设置事件标志。*/
}

/**
 * @brief 设置计时周期
 *
 * @param base TIMER0/TIMER1/TIMER2
 * @param count 计数周期
 */
void timer_set_count(timer_base_t base,unsigned int count)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
	timer_p->timer_sc = count;/*设置计数周期*/
}

/**
 * @brief 获取当前计数值
 *
 * @param base TIMER0/TIMER1/TIMER2
 * @param count 当前计数值
 */
void timer_get_count(timer_base_t base,unsigned int* count)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
	*count = timer_p->timer_cc;/*获取当前计数值*/
}

/**
 * @brief 设置级联时钟计数模式
 *
 * @param base TIMER0/TIMER1/TIMER2
 * @param count 计数值
 */
void timer_cascade_set(timer_base_t base,unsigned int count)
{
    /* 功能实现 */
    timer_register_t* timer_p = (timer_register_t*)base;
	timer_p->timer_cfg0 |= (0x1 << 0);/*获取当前计数值*/
    timer_p->timer_sc = count;/*设置计数周期*/
}


/**
 * @brief 清除中断
 *
 * @param base TIMER0/TIMER1/TIMER2
 */
void timer_clear_irq(timer_base_t base)
{
    timer_register_t* timer_p = (timer_register_t*)base;
	/*清除中断*/
    timer_p->timer_cfg1 |= 1 << 2;
}
