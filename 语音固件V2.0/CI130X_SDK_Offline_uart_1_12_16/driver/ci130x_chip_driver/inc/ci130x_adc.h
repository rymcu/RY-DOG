/**
 * @file ci130x_adc.h
 * @brief CI130X芯片ADC模块的驱动程序头文件
 * @version 0.1
 * @date 2019-05-17
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef __CI130X_ADC_H
#define __CI130X_ADC_H


//是否使用操作系统相关的队列，如果不适用则使用回调函数模式
#define ADC_DRIVER_USE_RTOS 0


#include "ci130x_system.h"

#ifdef __cplusplus
extern "C"{
#endif

//采样保持时间选择
typedef enum
{
    //采样保持1个cycle
    ADC_CLKCYCLE_1 = 0,
    //采样保持2个cycle
	ADC_CLKCYCLE_2	=1,
    //采样保持3个cycle
	ADC_CLKCYCLE_3	=2,
    //采样保持4个cycle
	ADC_CLKCYCLE_4	=3,
    //采样保持5个cycle
	ADC_CLKCYCLE_5	=4,
}adc_clkcyclex_t;

/**
 * @brief ADC通道编号
 *
 */
typedef enum
{
    ADC_CHANNEL_0	=0,
    ADC_CHANNEL_1	=1,
    ADC_CHANNEL_2	=2,
    ADC_CHANNEL_3	=3,
    ADC_CHANNEL_4	=4,
    ADC_CHANNEL_5	=5,
    ADC_CHANNEL_MAX =6,
}adc_channelx_t;

/**
 * @brief ADC中断条件选择
 *
 */
typedef enum
{
    //!每次转换完成触发中断
    ADC_INT_MODE_TRANS_END = 0,
    //!采样值出现异常的时候触发中断
    ADC_INT_MODE_VALUE_NOT_MEET = 1,
}adc_int_mode_t;

void adc_int_clear(adc_channelx_t channel);
void adc_soc_soft_ctrl(FunctionalState cmd);
int8_t adc_get_vol_value(adc_channelx_t cha,float* vol_val);
void adc_poweron(void);
void adc_reset(void);
void adc_signal_mode(adc_channelx_t cha);
void adc_series_mode(adc_channelx_t cha);
void adc_cycle_mode(adc_channelx_t cha,uint16_t cycle);
void adc_caculate_mode(void);
uint32_t adc_get_result(adc_channelx_t channel);
void adc_calibrate(FunctionalState cmd);


//TEST  ADC
#define ADC_INIT_ADDR           0x1ff53000
#define AIN0     *(volatile unsigned int *)(ADC_INIT_ADDR + 0*4)
#define AIN1     *(volatile unsigned int *)(ADC_INIT_ADDR + 1*4)
#define AIN2	 *(volatile unsigned int *)(ADC_INIT_ADDR + 2*4)
#define AIN3	 *(volatile unsigned int *)(ADC_INIT_ADDR + 3*4)

#define ADC_INIT_RSEILT_ADDR    0x1ff54000
#define RESULT_AIN0     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +0*4)//1
#define RESULT_AIN1     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +1*4)//1
#define RESULT_AIN2     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +2*4)//1
#define RESULT_AIN3     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +3*4)//1

#define RESULT_JY_AIN0     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +4*4)//1
#define RESULT_JY_AIN1     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +5*4)//1
#define RESULT_JY_AIN2     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +6*4)//1
#define RESULT_JY_AIN3     *(volatile unsigned int *)(ADC_INIT_RSEILT_ADDR +7*4)//1

#define a3_result_in       (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR  +8*4)//5

#define an0_result_other   (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+  13*4)//24
#define an1_result_other   (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24)*4)//12
#define an2_result_other   (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12)*4)//8
#define an3_result_other   (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12+8)*4)//6

#define an0_result_last    (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12+8+6)*4)//3
#define an1_result_last    (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12+8+6+3)*4)//3
#define an2_result_last    (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12+8+6+3+3)*4)//3
#define an3_result_last    (volatile unsigned int *)(ADC_INIT_RSEILT_ADDR+ (13+24+12+8+6+3+3+3)*4)//3

extern void adc_wait_int(adc_channelx_t cha);
void ADC_irqhandle(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /*__CI100X_ADC_H*/
