/**
 * @file ci130x_codec.h
 * @brief CI130X inner codec驱动的头文件
 * @version 0.1
 * @date 2019-05-10
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __PDM_H
#define __PDM_H

#include "ci130x_system.h"
#include "ci_log.h"
#include <stdbool.h>
#include <math.h>


#define USE_CI110X_ALC 0

#ifdef __cplusplus
 extern "C" {
#endif



/**
 * @brief PDM预充电电流
 * 
 */
typedef enum
{
    PDM_CURRENT_I   = 0x80,
    PDM_CURRENT_2I  = 0x40,
    PDM_CURRENT_4I  = 0x20,
    PDM_CURRENT_8I  = 0x10,
    PDM_CURRENT_16I = 0x08,
    PDM_CURRENT_32I = 0x04,
    PDM_CURRENT_64I = 0x02,
    PDM_CURRENT_128I= 0x01,
        
}pdm_current_t;


/**
 * @brief PDM高通滤波器截止频率设置
 * 
 */
typedef enum
{
    //!PDM高通滤波器截止频率设置为20Hz
    PDM_HIGHPASS_CUT_OFF_20HZ = 0,
    //!PDM高通滤波器截止频率设置为245Hz
    PDM_HIGHPASS_CUT_OFF_245HZ = 1,
    //!PDM高通滤波器截止频率设置为612Hz
    PDM_HIGHPASS_CUT_OFF_612HZ  = 2, 
    //!PDM高通滤波器截止频率设置为1Hz
    PDM_HIGHPASS_CUT_OFF_1HZ  = 3, 
}pdm_highpass_cut_off_t;


/**
 * @brief PDM ALC判断信号选择
 * 
 */
typedef enum
{
    //!ALC的判断信号来自高通滤波器之前（同时也是数字增益之前）
    PDM_ALC_JUDGE_BEFOR_HIGHPASS = 0,
    //!ALC的判断信号来自高通滤波器之后（同时也是数字增益之后）
    PDM_ALC_JUDGE_AFTER_HIGHPASS = 1,
}pdm_alc_judge_t;


/**
 * @brief PDM ALC模式设置
 * 
 */
typedef enum
{
    //!ALC normal模式
    PDM_ALC_MODE_NORMAL = 0,
    //!ALC 模式为jackway1
    PDM_ALC_MODE_JACKWAY1 = 1,
    //!ALC 模式为jackway2
    PDM_ALC_MODE_JACKWAY2 = 2,
    //!ALC 模式为jackway3
    PDM_ALC_MODE_JACKWAY3 = 3,
}pdm_alc_mode_t;


/**
 * @brief PDM使用或者不适用PDM本身的ALC控制其模拟PGA的增益
 * 
 */
typedef enum
{
    //!不使用ALC控制PGA增益
    PDM_WONT_USE_ALC_CONTROL_PGA_GAIN = 0,
    //!使用ALC控制PGA增益
    PDM_USE_ALC_CONTROL_PGA_GAIN,
}pdm_use_alc_control_pgagain_t;



/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_codec ci130x_codec
 * @brief CI130X芯片PDM驱动
 * @{
 */


/**
 * @brief PDM 主从模式设置
 * 
 */
typedef enum
{
    //!PDM主模式
    PDM_MODE_MASTER = 3,
    //!PDM从模式
    PDM_MODE_SLAVE  = 0,      
}pdm_mode_t;


/**
 * @brief PDM数据窗口的宽度
 * 
 */
typedef enum
{
    //!数据窗口宽度为32bit
    PDM_FRAME_LEN_32BIT = 3,
    //!数据窗口宽度为24bit
    PDM_FRAME_LEN_24BIT = 2,
}pdm_frame_1_2len_t;


/**
 * @brief PDM有效数据宽度
 * 
 */
typedef enum
{
    //!有效数据宽度为32bit
    PDM_VALID_LEN_32BIT = 3,
    //!有效数据宽度为24bit
    PDM_VALID_LEN_24BIT = 2,
    //!有效数据宽度为20bit
    PDM_VALID_LEN_20BIT = 1,
    //!有效数据宽度为16bit
    PDM_VALID_LEN_16BIT = 0,
}pdm_valid_word_len_t;


/**
 * @brief PDM数据格式
 * 
 */
typedef enum
{
    //!inner codec的数据格式为PCM格式
    PDM_I2S_DATA_FORMAT_PCM_MODE = 3,
    //!inner codec的数据格式为标准I2S格式
    PDM_I2S_DATA_FORMAT_I2S_MODE = 2,
    //!inner codec的数据格式为左对齐格式，slave模式下不支持
    PDM_I2S_DATA_FORMAT_LJ_MODE  = 1,
    //!inner codec的数据格式为右对齐格式
    PDM_I2S_DATA_FORMAT_RJ_MODE  = 0,
}pdm_i2s_data_famat_t;


/**
 * @brief PDM MIC输入端查分或者单端模式设置
 * 
 */
typedef enum
{
    //!输入为差分输入模式
    PDM_INPUT_MODE_DIFF = 1,
    //!输入为单端输入模式
    PDM_INPUT_MODE_SINGGLE_ENDED = 2,
}pdm_input_mode_t;


/**
 * @brief PDM MIC增益配置
 * 
 */
typedef enum
{
    //!MIC增益为0dB
    PDM_MIC_AMP_0dB = 0,
    //!MIC增益为6dB
    PDM_MIC_AMP_6dB = 1,
    //!MIC增益为13dB
    PDM_MIC_AMP_13dB = 2,
    //!MIC增益为20dB
    PDM_MIC_AMP_20dB = 3,
}pdm_mic_amplify_t;


/**
 * @brief PDM采样率设置
 * 
 */
typedef enum
{
    //!采样率为96K
    PDM_SAMPLERATE_96K = 0,
    //!采样率为48K
    PDM_SAMPLERATE_48K = 1,
    //!采样率为44.1K
    PDM_SAMPLERATE_44_1K = 2,
    //!采样率为32K
    PDM_SAMPLERATE_32K = 3,
    //!采样率为24K
    PDM_SAMPLERATE_24K = 4,
    //!采样率为16K
    PDM_SAMPLERATE_16K = 5,
    //!采样率为12K
    PDM_SAMPLERATE_12K = 6,
    //!采样率为8K
    PDM_SAMPLERATE_8K = 7,
}pdm_samplerate_t;


/**
 * @brief PDM ALC hold time设置
 * 
 */
typedef enum
{
    //!ALC hold time 0
    PDM_ALC_HOLD_TIME_0MS = 0,
    //!ALC hold time 2ms
    PDM_ALC_HOLD_TIME_2MS = 1,
    //!ALC hold time 4ms
    PDM_ALC_HOLD_TIME_4MS = 2,
    //!ALC hold time 8ms
    PDM_ALC_HOLD_TIME_8MS = 3,
    //!ALC hold time 16ms
    PDM_ALC_HOLD_TIME_16MS = 4,
    //!ALC hold time 32ms
    PDM_ALC_HOLD_TIME_32MS = 5,
    //!ALC hold time 64ms
    PDM_ALC_HOLD_TIME_64MS = 6,
    //!ALC hold time 128ms
    PDM_ALC_HOLD_TIME_128MS = 7,
    //!ALC hold time 256ms
    PDM_ALC_HOLD_TIME_256MS = 8,
    //!ALC hold time 512ms
    PDM_ALC_HOLD_TIME_512MS = 9,
    //!ALC hold time 1024ms
    PDM_ALC_HOLD_TIME_1024mS = 10,
}pdm_alc_hold_time_t;


/**
 * @brief PDM ALC decay time配置
 * 
 */
typedef enum
{
    //!ALC decay time 500us
    PDM_ALC_DECAY_TIME_500US = 0,
    //!ALC decay time 1ms
    PDM_ALC_DECAY_TIME_1MS = 1,
    //!ALC decay time 2ms
    PDM_ALC_DECAY_TIME_2MS = 2,
    //!ALC decay time 4ms
    PDM_ALC_DECAY_TIME_4MS = 3,
    //!ALC decay time 8ms
    PDM_ALC_DECAY_TIME_8MS = 4,
    //!ALC decay time 16ms
    PDM_ALC_DECAY_TIME_16MS = 5,
    //!ALC decay time 32ms
    PDM_ALC_DECAY_TIME_32MS = 6,
    //!ALC decay time 64ms
    PDM_ALC_DECAY_TIME_64MS = 7,
    //!ALC decay time 128ms
    PDM_ALC_DECAY_TIME_128MS = 8,
    //!ALC decay time 256ms
    PDM_ALC_DECAY_TIME_256MS = 9,
    //!ALC decay time 512ms
    PDM_ALC_DECAY_TIME_512MS = 10,
}pdm_alc_decay_time_t;


/**
 * @brief PDM ALC attack time设置
 * 
 */
typedef enum
{
    //!ALC attack time 125us
    PDM_ALC_ATTACK_TIME_125US = 0,
    //!ALC attack time 250us
    PDM_ALC_ATTACK_TIME_250US = 1,
    //!ALC attack time 500us
    PDM_ALC_ATTACK_TIME_500US = 2,
    //!ALC attack time 1ms
    PDM_ALC_ATTACK_TIME_1MS = 3,
    //!ALC attack time 2ms
    PDM_ALC_ATTACK_TIME_2MS = 4,
    //!ALC attack time 4ms
    PDM_ALC_ATTACK_TIME_4MS = 5,
    //!ALC attack time 8ms
    PDM_ALC_ATTACK_TIME_8MS = 6,
    //!ALC attack time 16ms
    PDM_ALC_ATTACK_TIME_16MS = 7,
    //!ALC attack time 32ms
    PDM_ALC_ATTACK_TIME_32MS = 8,
    //!ALC attack time 64ms
    PDM_ALC_ATTACK_TIME_64MS = 9,
    //!ALC attack time 128ms
    PDM_ALC_ATTACK_TIME_128MS = 10,
}pdm_alc_attack_time_t;


/**
 * @brief PDM ALC noise gate门限
 * 
 */
typedef enum
{
    //!ALC噪声门限39dB
    PDM_NOISE_GATE_THRE_39dB = 0,
    //!ALC噪声门限45dB
    PDM_NOISE_GATE_THRE_45dB = 1,
    //!ALC噪声门限51dB
    PDM_NOISE_GATE_THRE_51dB = 2,
    //!ALC噪声门限57dB
    PDM_NOISE_GATE_THRE_57dB = 3,   
}pdm_noise_gate_threshold_t;


/**
 * @brief PDM全局开关
 * 
 */
typedef enum
{
    //!打开PDM
    PDM_GATE_ENABLE  =   1,
    //!关闭PDM
    PDM_GATE_DISABLE =   0,
}pdm_gate_t;



/**
 * @brief PDM ALC 最大增益设置
 * 
 */
typedef enum
{
    //!ALC设置PGA的增益上限为-13.5dB
    PDM_ALC_PGA_MAX_GAIN__13_5dB = 0,
    //!ALC设置PGA的增益上限为-7.5dB
    PDM_ALC_PGA_MAX_GAIN__7_5dB = 1,
    //!ALC设置PGA的增益上限为-1.5dB
    PDM_ALC_PGA_MAX_GAIN__1_5dB = 2,
    //!ALC设置PGA的增益上限为4.5dB
    PDM_ALC_PGA_MAX_GAIN_4_5dB  = 3,
    //!ALC设置PGA的增益上限为10.5dB
    PDM_ALC_PGA_MAX_GAIN_10_5dB = 4,
    //!ALC设置PGA的增益上限为16.5dB
    PDM_ALC_PGA_MAX_GAIN_16_5dB = 5,
    //!ALC设置PGA的增益上限为22.5dB
    PDM_ALC_PGA_MAX_GAIN_22_5dB = 6,
    //!ALC设置PGA的增益上限为28.5dB
    PDM_ALC_PGA_MAX_GAIN_28_5dB = 7,
}pdm_alc_pga_max_gain_t;



/**
 * @brief PDM ALC 最小增益设置
 * 
 */
typedef enum
{
    //!ALC设置PGA的增益下限为-18dB
    PDM_ALC_PGA_MIN_GAIN__18dB = 0,
    //!ALC设置PGA的增益下限为-12dB
    PDM_ALC_PGA_MIN_GAIN__12dB = 1,
    //!ALC设置PGA的增益下限为-6dB
    PDM_ALC_PGA_MIN_GAIN__6dB  = 2,
    //!ALC设置PGA的增益下限为0dB
    PDM_ALC_PGA_MIN_GAIN_0dB = 3,
    //!ALC设置PGA的增益下限为6dB
    PDM_ALC_PGA_MIN_GAIN_6dB = 4,
    //!ALC设置PGA的增益下限为12dB
    PDM_ALC_PGA_MIN_GAIN_12dB = 5,
    //!ALC设置PGA的增益下限为18dB
    PDM_ALC_PGA_MIN_GAIN_18dB = 6,
    //!ALC设置PGA的增益下限为24dB
    PDM_ALC_PGA_MIN_GAIN_24dB = 7,
}pdm_alc_pga_min_gain_t;


typedef enum
{
    PDM_ALC_LEVEL_0dB  = 0x7fff,
    PDM_ALC_LEVEL__0_5dB  = 0x78d6,
    PDM_ALC_LEVEL__1dB  = 0x7213,
    PDM_ALC_LEVEL__1_5dB  = 0x6bb1,
    PDM_ALC_LEVEL__2dB  = 0x65ab,
    PDM_ALC_LEVEL__2_5dB  = 0x5ffb,
    PDM_ALC_LEVEL__3dB  = 0x5a9d,
    PDM_ALC_LEVEL__3_5dB  = 0x558b,
    PDM_ALC_LEVEL__4dB  = 0x50c2,
    PDM_ALC_LEVEL__4_5dB  = 0x4c3e,
    PDM_ALC_LEVEL__5dB  = 0x47fa,
    PDM_ALC_LEVEL__5_5dB  = 0x43f3,
    PDM_ALC_LEVEL__6dB  = 0x4026,
    PDM_ALC_LEVEL__6_5dB  = 0x3c8f,
    PDM_ALC_LEVEL__7dB  = 0x392c,
    PDM_ALC_LEVEL__7_5dB  = 0x35f9,
    PDM_ALC_LEVEL__8dB  = 0x32f4,
    PDM_ALC_LEVEL__8_5dB  = 0x301b,
    PDM_ALC_LEVEL__9dB  = 0x2d6a,
    PDM_ALC_LEVEL__9_5dB  = 0x2adf,
    PDM_ALC_LEVEL__10dB  = 0x2879,
    PDM_ALC_LEVEL__10_5dB  = 0x2636,
    PDM_ALC_LEVEL__11dB  = 0x2412,
    PDM_ALC_LEVEL__11_5dB  = 0x220e,
    PDM_ALC_LEVEL__12dB  = 0x2026,
    PDM_ALC_LEVEL__12_5dB  = 0x1e5a,
    PDM_ALC_LEVEL__13dB  = 0x2026,
    PDM_ALC_LEVEL__13_5dB  = 0x1b0d,
    PDM_ALC_LEVEL__14dB  = 0x1989,
    PDM_ALC_LEVEL__14_5dB  = 0x181c,
    PDM_ALC_LEVEL__15dB  = 0x16c2,
    PDM_ALC_LEVEL__15_5dB  = 0x157c,
    PDM_ALC_LEVEL__16dB  = 0x1449,
    PDM_ALC_LEVEL__16_5dB  = 0x1326,
    PDM_ALC_LEVEL__17dB  = 0x1214,
    PDM_ALC_LEVEL__17_5dB  = 0x1111,
    PDM_ALC_LEVEL__18dB  = 0x101d,
    PDM_ALC_LEVEL__18_5dB  = 0x0f36,
    PDM_ALC_LEVEL__19dB  = 0x0e5c,
    PDM_ALC_LEVEL__19_5dB  = 0x0d8e,
}pdm_alc_level_t;


/**
 * @brief PDM ADC配置
 * 
 */
typedef struct 
{
    //!ADC L 输入模式选择
    pdm_input_mode_t   codec_adc_input_mode_l;
    //!ADC R 输入模式选择
    pdm_input_mode_t   codec_adc_input_mode_r;
    //!MIC L 增益设置
    pdm_mic_amplify_t  codec_adc_mic_amp_l;
    //!MIC R 增益设置
    pdm_mic_amplify_t  codec_adc_mic_amp_r;
    //!不使能ALC的情况下，PGA L 的固定增益
    float pga_gain_l;
    //!不使能ALC的情况下，PGA R 的固定增益
    float pga_gain_r;
}pdm_adc_config_t;


/**
 * @brief PDM ALC设置结构体类型
 * 
 */
typedef struct 
{
    //!ALC hold time设置
    pdm_alc_hold_time_t holdtime;
    //!ALC decay time设置
    pdm_alc_decay_time_t decaytime;
    //!ALC attack time设置
    pdm_alc_attack_time_t attacktime;
    //!ALC设置PGA的上限设置
    pdm_alc_pga_max_gain_t max_pga_gain;
    //!ALC设置PGA的下限设置
    pdm_alc_pga_min_gain_t min_pga_gain;
    //!ALC采样率设置
    pdm_samplerate_t  samplerate;
    //!ALC模式设置
    pdm_alc_mode_t alcmode;
    //!不使能ALC的情况下，PGA的增益设置
    float pga_gain;
    //!ALC max level设置
    pdm_alc_level_t max_level;   
    //!ALC min level设置
    pdm_alc_level_t min_level;
}pdm_alc_config_t;



/**
 * @brief PDM ALC通道选择
 * 
 */
typedef enum
{
    //!ALC左通道
    PDM_ALC_CHA_R_ONLY = 0,
    //!ALC右通道
    PDM_ALC_CHA_L_ONLY,
    //!ALC两个通道
    PDM_ALC_CHA_STEREO_ONLY,
    //!ALC关闭
    PDM_ALC_CHA_NONE,
}pdm_alc_cha_sel_t;


/**
 * @brief PDM简易版ALC配置
 * 
 */
typedef struct 
{
    //!ALC设置PGA增益上限设置
    pdm_alc_pga_max_gain_t alc_maxgain;    
    //!ALC设置PGA增益下限设置
    pdm_alc_pga_min_gain_t alc_mingain;   
    //!ALC max level设置
    pdm_alc_level_t max_level;   
    //!ALC min level设置
    pdm_alc_level_t min_level;
	//!ALC ALC的采样率设置
    pdm_samplerate_t sample_rate;
    //!使用CI130X的ALC控制PDM还是使用其本身的ALC控制
    pdm_gate_t use_ci130x_alc;
    //!ALC开关
    pdm_gate_t alc_gate;
    //!ALC attack time设置
    pdm_alc_attack_time_t attack_time;
    //!ALC decay time设置
    pdm_alc_decay_time_t decay_time;
    //!ALC hold time设置
    pdm_alc_hold_time_t hold_time;
    //!noise gate 开关
    pdm_gate_t noise_gate;
    //!过顶保护开关
    pdm_gate_t fast_decrece_87_5;
    //!过零检测开关
    pdm_gate_t zero_cross;
    //!噪声门限阈值
    pdm_noise_gate_threshold_t noise_gate_threshold;
    //!基本的数字增益
    uint16_t digt_gain;
}pdm_alc_use_config_t;


/**
 * @brief inner CODEC通道设置
 * 
 */
typedef enum
{
    //!CODEC ALC左通道
    PDM_LEFT_CHA = 0,
    //!CODEC ALC右通道
    PDM_RIGHT_CHA = 1,
}pdm_cha_sel_t;


void pdm_reset(void);
void pdm_power_up(pdm_current_t current);
void pdm_power_off(void);
void pdm_hightpass_config(pdm_gate_t gate,pdm_highpass_cut_off_t Hz);
void pdm_adc_enable(pdm_adc_config_t *ADC_Config);
void pdm_adc_disable(pdm_cha_sel_t cha, pdm_gate_t EN);
void pdm_dac_enable(void);
void pdm_dac_disable(pdm_cha_sel_t cha, pdm_gate_t EN);
void pdm_alc_disable(pdm_cha_sel_t cha,float ALC_Gain);
void pdm_adc_mode_set(pdm_mode_t mode,pdm_frame_1_2len_t frame_Len,
                        pdm_valid_word_len_t word_len,pdm_i2s_data_famat_t data_fram);
void pdm_dac_mode_set(pdm_mode_t mode,pdm_frame_1_2len_t frame_Len,
                        pdm_valid_word_len_t word_len,pdm_i2s_data_famat_t data_fram);
void pdm_dac_gain_set(int32_t l_gain, int32_t r_gain);
void pdm_left_alc_pro_mode_config(pdm_alc_config_t* ALC_Type);
void pdm_right_alc_pro_mode_config(pdm_alc_config_t* ALC_Type);
void pdm_left_alc_enable(pdm_gate_t gate,pdm_use_alc_control_pgagain_t is_alc_ctr_pga);
void pdm_right_alc_enable(pdm_gate_t gate,pdm_use_alc_control_pgagain_t is_alc_ctr_pga);
void pdm_noise_gate_set(pdm_cha_sel_t cha,pdm_noise_gate_threshold_t Threshold,pdm_gate_t gate);
void pdm_zero_cross_set(pdm_cha_sel_t cha, pdm_gate_t gate);
void pdm_87_5_fast_decrement_set(pdm_cha_sel_t cha, pdm_gate_t gate);
void pdm_alc_judge_sel(pdm_alc_judge_t judge);
void pdm_pga_gain_config_via_reg27_28(pdm_cha_sel_t cha,uint32_t gain);
pdm_mic_amplify_t pdm_get_mic_gain(pdm_cha_sel_t cha);
void pdm_set_mic_gain(pdm_cha_sel_t cha,pdm_mic_amplify_t gain);
void pdm_alc_left_config(pdm_alc_use_config_t* ALC_str);
void pdm_alc_right_config(pdm_alc_use_config_t* ALC_str);
void pdm_adc_dig_gain_set_left(uint8_t gain);
void pdm_adc_dig_gain_set_right(uint8_t gain);
void pdm_set_input_mode_left(pdm_input_mode_t mode);
void pdm_set_input_mode_right(pdm_input_mode_t mode);
void pdm_set_mic_gain_left(pdm_mic_amplify_t gain);
void pdm_set_mic_gain_right(pdm_mic_amplify_t gain);
uint8_t pdm_get_alc_gain_left(void);
uint8_t pdm_get_alc_gain_right(void);
void pdm_pga_gain_config_via_reg43_53(pdm_cha_sel_t cha,uint32_t gain);
void pdm_pga_gain_config_via_reg43_53_db(pdm_cha_sel_t cha,float gain_db);
void pdm_pga_gain_config_via_reg27_28_db(pdm_cha_sel_t cha,float gain_db);
void pdm_hpout_mute(void);
void pdm_hpout_mute_disable(void);
void pdm_set_sample_rate(pdm_samplerate_t samplerate);


void pdm_left_mic_reinit(void);
void pdm_right_mic_reinit(void);
/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

