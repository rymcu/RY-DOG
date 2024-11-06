/**
 * @file ci130x_alc.h
 * @brief CI130X的ALC模块驱动的头文件
 * @version 0.1
 * @date 2019-05-10
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef __CI130X_ALC_H
#define __CI130X_ALC_H

#include "ci130x_system.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


//ALC调节的粒度，每一档调节增益调整的大小（dB为单位）
typedef enum
{
    //ALC每次增减0.5dB
    ALC_STEP_GRD_0_5dB = 0,
    //ALC每次增减1dB
    ALC_STEP_GRD_1dB   = 1,
    //ALC每次增减1.5dB
    ALC_STEP_GRD_1_5dB = 2,
    //ALC每次增减2dB
    ALC_STEP_GRD_2dB =   3,
    //ALC每次增减2.5dB
    ALC_STEP_GRD_2_5dB = 4,
    //ALC每次增减3dB
    ALC_STEP_GRD_3dB =   5,
    //ALC每次增减3.5dB
    ALC_STEP_GRD_3_5dB = 6,
    //ALC每次增减4dB
    ALC_STEP_GRD_4dB =   7,
}alc_aux_step_grd_t;


//ALC每次调节的次数
typedef enum
{
    //ALC每次调节增益调节1次
    ALC_STEP_CNT_1 = 0,
    //ALC每次调节增益调节2次
    ALC_STEP_CNT_2 = 1,
    //ALC每次调节增益调节3次
    ALC_STEP_CNT_3 = 2,
    //ALC每次调节增益调节4次
    ALC_STEP_CNT_4 = 3,
}alc_aux_step_cnt_t;


//ALC通道选择
typedef enum
{
    //ALC右通道
    ALC_Right_Cha = 0,
    //ALC左通道
    ALC_Left_Cha  = 1,
}alc_aux_cha_t;



//ALC中断配置结构体类型
typedef struct
{
    //ALC输出的增益比设置的最大增益高
    uint32_t crs_up_max_db_int_en;
    //ALC输出的增益比设置的最小增益低
    uint32_t crs_down_min_db_int_en;
    
    //ALC输出的增益比设置的最大增益低
    uint32_t crs_down_max_db_int_en;
    //ALC输出的增益比设置的最小增益高
    uint32_t crs_up_min_db_int_en;
    
    //处于连续ATTACK过程超过设定的次数，说明输入信号幅度较大
    uint32_t atk_limit_int_en;
    //处于连续DECAY过程超过设定的次数，说明输入信号幅度较小
    uint32_t dcy_limit_int_en;
    
    //检测到gain_ch翻转（inner codec给出的信号，每次inner codec ALC输出的增益变化，都会使这个信号翻转）
    uint32_t gain_change_int_en;
    //检测到gain_in变化（检测inner codec ALC输出的增益改变）
    uint32_t gain_in_change_int_en;
    //GAIN出现非连续变化（异常情况）
    uint32_t gain_discr_int_en;
    
    //配置PDM寄存器后读回数据对比出错
    uint32_t read_err_int_en;
}alc_aux_int_t;


//ALC配置结构体类型
typedef struct 
{
    //ALC调节粒度设置
    alc_aux_step_grd_t  step_grd;
    //ALC每次调节次数设置
    alc_aux_step_cnt_t  step_cnt;
    //当GAIN为0时，对应的8bit数字增益起点
    uint32_t      db_offset;
    //增益减小的时候，每次GAIN变化调节子级需要的时间间隔，需根据AHP时钟设置
    uint32_t      step_tmr_atk;
    //增益增加的时候，每次GAIN变化调节子级需要的时间间隔，需根据AHP时钟设置
    uint32_t      step_tmr_dcy;
    //ALC输出的增益配置，需要送往PDM的数字增益配置寄存器地址
    uint32_t      cfg_addr;
    //实际运行DB下限配置，超出范围，将进行中断报警
    uint32_t      min_db_alert;
    //实际运行DB上限配置，超出范围，将进行中断报警
    uint32_t      max_db_alert;

    //连续处于ATK次数的上限，超过后进行中断报警
    uint32_t      atk_limit_num;
    //连续处于DCY次数的上限，超过后进行中断报警
    uint32_t      dcy_limit_num;
}alc_aux_config_t;


//ALC左通道的输入增益来自inner codec的那个通道
typedef enum
{
    //ALC左通道的输入为inner PDM左通道输出
    ALC_L_GAIN_FROM_PDM_L = 0,
    //ALC左通道的输入为inner PDM右通道输出
    ALC_L_GAIN_FROM_PDM_R = 1,
}alc_aux_l_gain_sel_t;


//ALC右通道的输入增益来自inner codec的那个通道
typedef enum
{
    //ALC右通道的输入为inner PDM左通道输出
    ALC_R_GAIN_FROM_PDM_R = 0,
    //ALC右通道的输入为inner PDM右通道输出
    ALC_R_GAIN_FROM_PDM_L = 1,
}alc_aux_r_gain_sel_t;


//ALC检测模式设置
typedef enum
{
    //配合DETECT_NUM检测
    ALC_DETECT_MD_NUM = 0,
    //检测电平翻转
    ALC_DETECT_MD_GAIN_CHANGE = 1,
}alc_aux_detect_md_sel_t;


//ALC全局配置结构体类型
typedef struct 
{
    //ALC左通道检测的增益来自inner codec左通道或者右通道
    alc_aux_l_gain_sel_t left_gain_sel;
    //ALC右通道检测的增益来自inner codec左通道或者右通道
    alc_aux_r_gain_sel_t right_gain_sel;
    //连续相同的周期数达到该值表示数据稳定
    uint32_t detect_num;
    //ALC检测模式设置
    alc_aux_detect_md_sel_t detect_mode;
}alc_aux_globle_config_t;



//当前ALC调整增益的状态
typedef struct 
{
    //由codec输入的db值
    uint32_t gain_db_from_codec; 
    //实时配置给codec数字增益模块的db值
    uint32_t gain_db_to_codec;  
    //目前进展到的实际增益调整级数 
    uint32_t step_cnt;           
}alc_aux_db_status_t;


//当前ALC处于的阶段
typedef struct 
{
    //是否处于atk阶段
    uint32_t is_atk;    
    //是否处于dcy阶段 
    uint32_t is_dcy;     
    //是否处于hold阶段
    uint32_t is_hold;   
    //连续ATK的计数值 
    uint32_t atk_cnt;    
    //连续dcy的计数值
    uint32_t dcy_cnt;    
}alc_aux_atk_dcy_status_t;


//当前ALC其他的一些状态
typedef struct 
{
    //AHB PAUSE状态
    uint32_t is_ahp_pause_en;    
    //L通道是否处于PAUSE状态
    uint32_t is_l_pause_en;      
    //R通道是否处于PAUSE状态
    uint32_t is_r_pause_en;     
    //L通道是否处于IDLE状态 
    uint32_t is_l_idle;         
    //R通道是否处于IDLE状态 
    uint32_t is_r_idle;          
    //l通道gain翻转信号
    uint32_t is_gain_ch_l;       
    //r通道gain翻转信号
    uint32_t is_gain_ch_r;       
    //L通道gain输入
    uint32_t gain_in_l;          
    //R通道gain输入
    uint32_t gain_in_r;          
}alc_aux_other_status_t;


void alc_interrupt_handler(ALC_TypeDef* alc);
void alc_aux_left_config(ALC_TypeDef* alc,alc_aux_config_t* ALC_Type);
void alc_aux_right_config(ALC_TypeDef* alc,alc_aux_config_t* ALC_Type);
void alc_aux_intterupt_config(ALC_TypeDef* alc,alc_aux_int_t* ALC_INT_Type,alc_aux_cha_t cha);
void alc_aux_globle_enable(ALC_TypeDef* alc,FunctionalState cmd);
void alc_aux_globle_config(ALC_TypeDef* alc,alc_aux_globle_config_t* ALC_Glb_Type);
void alc_aux_left_cha_en(ALC_TypeDef* alc,FunctionalState cmd);
void alc_aux_right_cha_en(ALC_TypeDef* alc,FunctionalState cmd);

#ifdef __cplusplus
}
#endif

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

