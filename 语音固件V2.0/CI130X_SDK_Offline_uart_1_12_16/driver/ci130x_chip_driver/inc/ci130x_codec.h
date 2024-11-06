/**
 * @file ci130x_codec.h
 * @brief CI130X inner codec驱动的头文件
 * @version 0.1
 * @date 2019-05-10
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CODEC_H
#define __CODEC_H

#include "ci130x_system.h"
#include <stdbool.h>
#include <math.h>


#define USE_CI130X_ALC 0

#ifdef __cplusplus
 extern "C" {
#endif


#if 1
/**
 * @brief inner CODEC预充电电流
 * 
 */
typedef enum
{
    INNER_CODEC_CURRENT_I   = 0x80,
    INNER_CODEC_CURRENT_2I  = 0x40,
    INNER_CODEC_CURRENT_4I  = 0x20,
    INNER_CODEC_CURRENT_8I  = 0x10,
    INNER_CODEC_CURRENT_16I = 0x08,
    INNER_CODEC_CURRENT_32I = 0x04,
    INNER_CODEC_CURRENT_64I = 0x02,
    INNER_CODEC_CURRENT_128I= 0x01,
        
}inner_codec_current_t;


/**
 * @brief inner CODEC高通滤波器截止频率设置
 * 
 */
typedef enum
{
    //!CODEC高通滤波器截止频率设置为20Hz
    INNER_CODEC_HIGHPASS_CUT_OFF_20HZ = 0,
    //!CODEC高通滤波器截止频率设置为245Hz
    INNER_CODEC_HIGHPASS_CUT_OFF_245HZ = 1,
    //!CODEC高通滤波器截止频率设置为612Hz
    INNER_CODEC_HIGHPASS_CUT_OFF_612HZ  = 2, 
    //!CODEC高通滤波器截止频率设置为1Hz
    INNER_CODEC_HIGHPASS_CUT_OFF_1HZ  = 3, 
}inner_codec_highpass_cut_off_t;


/**
 * @brief inner CODEC ALC判断信号选择
 * 
 */
typedef enum
{
    //!ALC的判断信号来自高通滤波器之前（同时也是数字增益之前）
    INNER_CODEC_ALC_JUDGE_BEFOR_HIGHPASS = 0,
    //!ALC的判断信号来自高通滤波器之后（同时也是数字增益之后）
    INNER_CODEC_ALC_JUDGE_AFTER_HIGHPASS = 1,
}inner_codec_alc_judge_t;


/**
 * @brief inner codec低通滤波器类型选择（之前固定6.67K低通）
 * 
 */
typedef enum
{
    INNER_CODEC_LP_FILTER_IIR = 0,
    INNER_CODEC_LP_FILTER_FIR = 1,
}inner_codec_lp_filter_mode_t;


/**
 * @brief inner CODEC ALC模式设置
 * 
 */
typedef enum
{
    //!ALC normal模式
    INNER_CODEC_ALC_MODE_NORMAL = 0,
    //!ALC 模式为jackway1
    INNER_CODEC_ALC_MODE_JACKWAY1 = 1,
    //!ALC 模式为jackway2
    INNER_CODEC_ALC_MODE_JACKWAY2 = 2,
    //!ALC 模式为jackway3
    INNER_CODEC_ALC_MODE_JACKWAY3 = 3,
}inner_codec_alc_mode_t;


/**
 * @brief inner CODEC使用或者不适用inner CODEC本身的ALC控制其模拟PGA的增益
 * 
 */
typedef enum
{
    //!不使用ALC控制PGA增益
    INNER_CODEC_WONT_USE_ALC_CONTROL_PGA_GAIN = 0,
    //!使用ALC控制PGA增益
    INNER_CODEC_USE_ALC_CONTROL_PGA_GAIN,
}inner_codec_use_alc_control_pgagain_t;



/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_codec ci130x_codec
 * @brief CI130X芯片inner CODEC驱动
 * @{
 */


/**
 * @brief inner CODEC 主从模式设置
 * 
 */
typedef enum
{
    //!CODEC主模式
    INNER_CODEC_MODE_MASTER = 3,
    //!CODEC从模式
    INNER_CODEC_MODE_SLAVE  = 0,      
}inner_codec_mode_t;


/**
 * @brief inner CODEC数据窗口的宽度
 * 
 */
typedef enum
{
    //!数据窗口宽度为32bit
    INNER_CODEC_FRAME_LEN_32BIT = 3,
    //!数据窗口宽度为24bit
    INNER_CODEC_FRAME_LEN_24BIT = 2,
}inner_codec_frame_1_2len_t;


/**
 * @brief inner CODEC有效数据宽度
 * 
 */
typedef enum
{
    //!有效数据宽度为32bit
    INNER_CODEC_VALID_LEN_32BIT = 3,
    //!有效数据宽度为24bit
    INNER_CODEC_VALID_LEN_24BIT = 2,
    //!有效数据宽度为20bit
    INNER_CODEC_VALID_LEN_20BIT = 1,
    //!有效数据宽度为16bit
    INNER_CODEC_VALID_LEN_16BIT = 0,
}inner_codec_valid_word_len_t;


/**
 * @brief inner CODEC数据格式
 * 
 */
typedef enum
{
    //!inner codec的数据格式为PCM格式
    INNER_CODEC_I2S_DATA_FORMAT_PCM_MODE = 3,
    //!inner codec的数据格式为标准I2S格式
    INNER_CODEC_I2S_DATA_FORMAT_I2S_MODE = 2,
    //!inner codec的数据格式为左对齐格式，slave模式下不支持
    INNER_CODEC_I2S_DATA_FORMAT_LJ_MODE  = 1,
    //!inner codec的数据格式为右对齐格式
    INNER_CODEC_I2S_DATA_FORMAT_RJ_MODE  = 0,
}inner_codec_i2s_data_famat_t;


/**
 * @brief inner CODEC MIC输入端查分或者单端模式设置
 * 
 */
typedef enum
{
    //!输入为差分输入模式
    INNER_CODEC_INPUT_MODE_DIFF = 1,
    //!输入为单端输入模式
    INNER_CODEC_INPUT_MODE_SINGGLE_ENDED = 2,
}inner_codec_input_mode_t;


/**
 * @brief inner CODEC MIC增益配置
 * 
 */
typedef enum
{
    INNER_CODEC_MIC_AMP_0dB = 0,
    INNER_CODEC_MIC_AMP_6dB,
    INNER_CODEC_MIC_AMP_9dB,
    INNER_CODEC_MIC_AMP_12dB,
    INNER_CODEC_MIC_AMP_16dB,
    INNER_CODEC_MIC_AMP_20dB,
}inner_codec_mic_amplify_t;


/**
 * @brief inner CODEC采样率设置
 * 
 */
typedef enum
{
    //!采样率为96K
    INNER_CODEC_SAMPLERATE_96K = 0,
    //!采样率为48K
    INNER_CODEC_SAMPLERATE_48K = 1,
    //!采样率为44.1K
    INNER_CODEC_SAMPLERATE_44_1K = 2,
    //!采样率为32K
    INNER_CODEC_SAMPLERATE_32K = 3,
    //!采样率为24K
    INNER_CODEC_SAMPLERATE_24K = 4,
    //!采样率为16K
    INNER_CODEC_SAMPLERATE_16K = 5,
    //!采样率为12K
    INNER_CODEC_SAMPLERATE_12K = 6,
    //!采样率为8K
    INNER_CODEC_SAMPLERATE_8K = 7,
}inner_codec_samplerate_t;


/**
 * @brief inner CODEC ALC hold time设置
 * 
 */
typedef enum
{
    //!ALC hold time 0
    INNER_CODEC_ALC_HOLD_TIME_0MS = 0,
    //!ALC hold time 2ms
    INNER_CODEC_ALC_HOLD_TIME_2MS = 1,
    //!ALC hold time 4ms
    INNER_CODEC_ALC_HOLD_TIME_4MS = 2,
    //!ALC hold time 8ms
    INNER_CODEC_ALC_HOLD_TIME_8MS = 3,
    //!ALC hold time 16ms
    INNER_CODEC_ALC_HOLD_TIME_16MS = 4,
    //!ALC hold time 32ms
    INNER_CODEC_ALC_HOLD_TIME_32MS = 5,
    //!ALC hold time 64ms
    INNER_CODEC_ALC_HOLD_TIME_64MS = 6,
    //!ALC hold time 128ms
    INNER_CODEC_ALC_HOLD_TIME_128MS = 7,
    //!ALC hold time 256ms
    INNER_CODEC_ALC_HOLD_TIME_256MS = 8,
    //!ALC hold time 512ms
    INNER_CODEC_ALC_HOLD_TIME_512MS = 9,
    //!ALC hold time 1024ms
    INNER_CODEC_ALC_HOLD_TIME_1024MS = 10,
}inner_codec_alc_hold_time_t;


/**
 * @brief inner CODEC ALC decay time配置
 * 
 */
typedef enum
{
    //!ALC decay time 500us
    INNER_CODEC_ALC_DECAY_TIME_500US = 0,
    //!ALC decay time 1ms
    INNER_CODEC_ALC_DECAY_TIME_1MS = 1,
    //!ALC decay time 2ms
    INNER_CODEC_ALC_DECAY_TIME_2MS = 2,
    //!ALC decay time 4ms
    INNER_CODEC_ALC_DECAY_TIME_4MS = 3,
    //!ALC decay time 8ms
    INNER_CODEC_ALC_DECAY_TIME_8MS = 4,
    //!ALC decay time 16ms
    INNER_CODEC_ALC_DECAY_TIME_16MS = 5,
    //!ALC decay time 32ms
    INNER_CODEC_ALC_DECAY_TIME_32MS = 6,
    //!ALC decay time 64ms
    INNER_CODEC_ALC_DECAY_TIME_64MS = 7,
    //!ALC decay time 128ms
    INNER_CODEC_ALC_DECAY_TIME_128MS = 8,
    //!ALC decay time 256ms
    INNER_CODEC_ALC_DECAY_TIME_256MS = 9,
    //!ALC decay time 512ms
    INNER_CODEC_ALC_DECAY_TIME_512MS = 10,
}inner_codec_alc_decay_time_t;


/**
 * @brief inner CODEC ALC attack time设置
 * 
 */
typedef enum
{
    //!ALC attack time 125us
    INNER_CODEC_ALC_ATTACK_TIME_125US = 0,
    //!ALC attack time 250us
    INNER_CODEC_ALC_ATTACK_TIME_250US = 1,
    //!ALC attack time 500us
    INNER_CODEC_ALC_ATTACK_TIME_500US = 2,
    //!ALC attack time 1ms
    INNER_CODEC_ALC_ATTACK_TIME_1MS = 3,
    //!ALC attack time 2ms
    INNER_CODEC_ALC_ATTACK_TIME_2MS = 4,
    //!ALC attack time 4ms
    INNER_CODEC_ALC_ATTACK_TIME_4MS = 5,
    //!ALC attack time 8ms
    INNER_CODEC_ALC_ATTACK_TIME_8MS = 6,
    //!ALC attack time 16ms
    INNER_CODEC_ALC_ATTACK_TIME_16MS = 7,
    //!ALC attack time 32ms
    INNER_CODEC_ALC_ATTACK_TIME_32MS = 8,
    //!ALC attack time 64ms
    INNER_CODEC_ALC_ATTACK_TIME_64MS = 9,
    //!ALC attack time 128ms
    INNER_CODEC_ALC_ATTACK_TIME_128MS = 10,
}inner_codec_alc_attack_time_t;


/**
 * @brief inner CODEC ALC noise gate门限
 * 
 */
typedef enum
{
    //!ALC噪声门限39dB
    INNER_CODEC_NOISE_GATE_THRE_39dB = 0,
    //!ALC噪声门限45dB
    INNER_CODEC_NOISE_GATE_THRE_45dB = 1,
    //!ALC噪声门限51dB
    INNER_CODEC_NOISE_GATE_THRE_51dB = 2,
    //!ALC噪声门限57dB
    INNER_CODEC_NOISE_GATE_THRE_57dB = 3,   
}inner_codec_noise_gate_threshold_t;


/**
 * @brief inner CODEC全局开关
 * 
 */
typedef enum
{
    //!打开CODEC
    INNER_CODEC_GATE_ENABLE  =   1,
    //!关闭CODEC
    INNER_CODEC_GATE_DISABLE =   0,
}inner_cedoc_gate_t;


/**
 * @brief inner CODEC通道设置
 * 
 */
typedef enum
{
    //!CODEC ALC左通道
    INNER_CODEC_LEFT_CHA = 0,
    //!CODEC ALC右通道
    INNER_CODEC_RIGHT_CHA = 1,
}inner_codec_cha_sel_t;


/**
 * @brief inner CODEC ALC 最大增益设置
 * 
 */
typedef enum
{
    //!ALC设置PGA的增益上限为-13.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN__13_5dB = 0,
    //!ALC设置PGA的增益上限为-7.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN__7_5dB = 1,
    //!ALC设置PGA的增益上限为-1.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN__1_5dB = 2,
    //!ALC设置PGA的增益上限为4.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN_4_5dB  = 3,
    //!ALC设置PGA的增益上限为10.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN_10_5dB = 4,
    //!ALC设置PGA的增益上限为16.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN_16_5dB = 5,
    //!ALC设置PGA的增益上限为22.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN_22_5dB = 6,
    //!ALC设置PGA的增益上限为28.5dB
    INNER_CODEC_ALC_PGA_MAX_GAIN_28_5dB = 7,
}inner_codec_alc_pga_max_gain_t;



/**
 * @brief inner CODEC ALC 最小增益设置
 * 
 */
typedef enum
{
    //!ALC设置PGA的增益下限为-18dB
    INNER_CODEC_ALC_PGA_MIN_GAIN__18dB = 0,
    //!ALC设置PGA的增益下限为-12dB
    INNER_CODEC_ALC_PGA_MIN_GAIN__12dB = 1,
    //!ALC设置PGA的增益下限为-6dB
    INNER_CODEC_ALC_PGA_MIN_GAIN__6dB  = 2,
    //!ALC设置PGA的增益下限为0dB
    INNER_CODEC_ALC_PGA_MIN_GAIN_0dB = 3,
    //!ALC设置PGA的增益下限为6dB
    INNER_CODEC_ALC_PGA_MIN_GAIN_6dB = 4,
    //!ALC设置PGA的增益下限为12dB
    INNER_CODEC_ALC_PGA_MIN_GAIN_12dB = 5,
    //!ALC设置PGA的增益下限为18dB
    INNER_CODEC_ALC_PGA_MIN_GAIN_18dB = 6,
    //!ALC设置PGA的增益下限为24dB
    INNER_CODEC_ALC_PGA_MIN_GAIN_24dB = 7,
}inner_codec_alc_pga_min_gain_t;


typedef enum
{
    INNER_CODEC_ALC_LEVEL_0dB  = 0x7fff,
    INNER_CODEC_ALC_LEVEL__0_5dB  = 0x78d6,
    INNER_CODEC_ALC_LEVEL__1dB  = 0x7213,
    INNER_CODEC_ALC_LEVEL__1_5dB  = 0x6bb1,
    INNER_CODEC_ALC_LEVEL__2dB  = 0x65ab,
    INNER_CODEC_ALC_LEVEL__2_5dB  = 0x5ffb,
    INNER_CODEC_ALC_LEVEL__3dB  = 0x5a9d,
    INNER_CODEC_ALC_LEVEL__3_5dB  = 0x558b,
    INNER_CODEC_ALC_LEVEL__4dB  = 0x50c2,
    INNER_CODEC_ALC_LEVEL__4_5dB  = 0x4c3e,
    INNER_CODEC_ALC_LEVEL__5dB  = 0x47fa,
    INNER_CODEC_ALC_LEVEL__5_5dB  = 0x43f3,
    INNER_CODEC_ALC_LEVEL__6dB  = 0x4026,
    INNER_CODEC_ALC_LEVEL__6_5dB  = 0x3c8f,
    INNER_CODEC_ALC_LEVEL__7dB  = 0x392c,
    INNER_CODEC_ALC_LEVEL__7_5dB  = 0x35f9,
    INNER_CODEC_ALC_LEVEL__8dB  = 0x32f4,
    INNER_CODEC_ALC_LEVEL__8_5dB  = 0x301b,
    INNER_CODEC_ALC_LEVEL__9dB  = 0x2d6a,
    INNER_CODEC_ALC_LEVEL__9_5dB  = 0x2adf,
    INNER_CODEC_ALC_LEVEL__10dB  = 0x2879,
    INNER_CODEC_ALC_LEVEL__10_5dB  = 0x2636,
    INNER_CODEC_ALC_LEVEL__11dB  = 0x2412,
    INNER_CODEC_ALC_LEVEL__11_5dB  = 0x220e,
    INNER_CODEC_ALC_LEVEL__12dB  = 0x2026,
    INNER_CODEC_ALC_LEVEL__12_5dB  = 0x1e5a,
    INNER_CODEC_ALC_LEVEL__13dB  = 0x2026,
    INNER_CODEC_ALC_LEVEL__13_5dB  = 0x1b0d,
    INNER_CODEC_ALC_LEVEL__14dB  = 0x1989,
    INNER_CODEC_ALC_LEVEL__14_5dB  = 0x181c,
    INNER_CODEC_ALC_LEVEL__15dB  = 0x16c2,
    INNER_CODEC_ALC_LEVEL__15_5dB  = 0x157c,
    INNER_CODEC_ALC_LEVEL__16dB  = 0x1449,
    INNER_CODEC_ALC_LEVEL__16_5dB  = 0x1326,
    INNER_CODEC_ALC_LEVEL__17dB  = 0x1214,
    INNER_CODEC_ALC_LEVEL__17_5dB  = 0x1111,
    INNER_CODEC_ALC_LEVEL__18dB  = 0x101d,
    INNER_CODEC_ALC_LEVEL__18_5dB  = 0x0f36,
    INNER_CODEC_ALC_LEVEL__19dB  = 0x0e5c,
    INNER_CODEC_ALC_LEVEL__19_5dB  = 0x0d8e,
}inner_codec_alc_level_t;


/**
 * @brief inner CODEC ADC配置
 * 
 */
typedef struct 
{
    //!ADC L 输入模式选择
    inner_codec_input_mode_t   codec_adc_input_mode_l;
    //!ADC R 输入模式选择
    inner_codec_input_mode_t   codec_adc_input_mode_r;
    //!MIC L 增益设置
    inner_codec_mic_amplify_t  codec_adc_mic_amp_l;
    //!MIC R 增益设置
    inner_codec_mic_amplify_t  codec_adc_mic_amp_r;
    //!不使能ALC的情况下，PGA L 的固定增益
    float pga_gain_l;
    //!不使能ALC的情况下，PGA R 的固定增益
    float pga_gain_r;
    //!左声道数字增益
    float dig_gain_l;
    //!右声道数字增益
    float dig_gain_r;
}inner_codec_adc_config_t;


/**
 * @brief inner CODEC ALC设置结构体类型
 * 
 */
typedef struct 
{
    //!ALC hold time设置
    inner_codec_alc_hold_time_t holdtime;
    //!ALC decay time设置
    inner_codec_alc_decay_time_t decaytime;
    //!ALC attack time设置
    inner_codec_alc_attack_time_t attacktime;
    //!ALC设置PGA的上限设置
    inner_codec_alc_pga_max_gain_t max_pga_gain;
    //!ALC设置PGA的下限设置
    inner_codec_alc_pga_min_gain_t min_pga_gain;
    //!ALC采样率设置
    inner_codec_samplerate_t  samplerate;
    //!ALC模式设置
    inner_codec_alc_mode_t alcmode;
    //!不使能ALC的情况下，PGA的增益设置
    float pga_gain;
    //!ALC max level设置
    inner_codec_alc_level_t max_level;
    //!ALC min level设置
    inner_codec_alc_level_t min_level;
}inner_codec_alc_config_t;



/**
 * @brief inner CODEC ALC通道选择
 * 
 */
typedef enum
{
    //!ALC左通道
    INNER_CODEC_ALC_CHA_R_ONLY = 0,
    //!ALC右通道
    INNER_CODEC_ALC_CHA_L_ONLY,
    //!ALC两个通道
    INNER_CODEC_ALC_CHA_STEREO_ONLY,
    //!ALC关闭
    INNER_CODEC_ALC_CHA_NONE,
}inner_codec_alc_cha_sel_t;


/**
 * @brief inner CODEC简易版ALC配置
 * 
 */
typedef struct 
{
    //!ALC设置PGA增益上限设置
    inner_codec_alc_pga_max_gain_t alc_maxgain;    
    //!ALC设置PGA增益下限设置
    inner_codec_alc_pga_min_gain_t alc_mingain;   
    //!ALC max level设置
    inner_codec_alc_level_t max_level;   
    //!ALC min level设置
    inner_codec_alc_level_t min_level;
	//!ALC ALC的采样率设置
    inner_codec_samplerate_t sample_rate;
    //!使用CI130X的ALC控制inner CODEC还是使用其本身的ALC控制
    inner_cedoc_gate_t use_ci130x_alc;
    //!ALC开关
    inner_cedoc_gate_t alc_gate;
    //!ALC attack time设置
    inner_codec_alc_attack_time_t attack_time;
    //!ALC decay time设置
    inner_codec_alc_decay_time_t decay_time;
    //!ALC hold time设置
    inner_codec_alc_hold_time_t hold_time;
    //!noise gate 开关
    inner_cedoc_gate_t noise_gate;
    //!过顶保护开关
    inner_cedoc_gate_t fast_decrece_87_5;
    //!过零检测开关
    inner_cedoc_gate_t zero_cross;
    //!噪声门限阈值
    inner_codec_noise_gate_threshold_t noise_gate_threshold;
    //!基本的数字增益
    uint16_t digt_gain;
}inner_codec_alc_use_config_t;


typedef enum
{
    INNER_CODEC_MIC_BIAS_1_0 = 0,//1.0 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_1 = 1,//1.1 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_2 = 2,//1.2 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_3 = 3,//1.3 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_4 = 4,//1.4 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_5 = 5,//1.5 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_6 = 6,//1.6 * (AVDD/2)
    INNER_CODEC_MIC_BIAS_1_7 = 7,//1.7 * (AVDD/2)
}inner_codec_micbias_t;


void inner_codec_up_ibas_adc(void);
void inner_codec_up_ibas_dac(void);
void inner_codec_reset(void);
void inner_codec_power_up(inner_codec_current_t current);
void inner_codec_power_off(void);
void inner_codec_hp_filter_config(inner_cedoc_gate_t gate,inner_codec_highpass_cut_off_t Hz);
void inner_codec_adc_enable(inner_codec_adc_config_t *ADC_Config);
void inner_codec_adc_disable(inner_codec_cha_sel_t cha, inner_cedoc_gate_t EN);
void inner_codec_dac_enable(bool is_first_enable);
void inner_codec_dac_disable(void);
void inner_codec_alc_disable(inner_codec_cha_sel_t cha);
void inner_codec_adc_mode_set(inner_codec_mode_t mode,inner_codec_frame_1_2len_t frame_Len,
                        inner_codec_valid_word_len_t word_len,inner_codec_i2s_data_famat_t data_fram);
void inner_codec_dac_mode_set(inner_codec_mode_t mode,inner_codec_frame_1_2len_t frame_Len,
                        inner_codec_valid_word_len_t word_len,inner_codec_i2s_data_famat_t data_fram);
void inner_codec_left_alc_pro_mode_config(inner_codec_alc_config_t* ALC_Type);
void inner_codec_right_alc_pro_mode_config(inner_codec_alc_config_t* ALC_Type);
void inner_codec_left_alc_enable(inner_cedoc_gate_t gate,inner_codec_use_alc_control_pgagain_t is_alc_ctr_pga);
void inner_codec_right_alc_enable(inner_cedoc_gate_t gate,inner_codec_use_alc_control_pgagain_t is_alc_ctr_pga);
void inner_codec_micbias_set(inner_codec_micbias_t bias);
#endif


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

