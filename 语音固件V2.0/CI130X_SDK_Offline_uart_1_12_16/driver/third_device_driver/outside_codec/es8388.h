/**
 * @file es8388.h
 * @brief 
 * @version 0.1
 * @date 2019-03-28
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */


#ifndef __ES8388_H
#define __ES8388_H

#include <stdint.h>
#include "ci130x_system.h"


/**
 * @ingroup third_device_driver
 * @defgroup es8388 es8388
 * @brief es8388的驱动
 */

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup es8388
 * @{
 */

/**
 * @brief 8388通道选择
 * 
 */
typedef enum
{
    //!右声道
    ES8388_CHA_RIGHT = 0,
    //!左声道
    ES8388_CHA_LEFT = 1,
    //!双声道
    ES8388_CHA_STEREO = 2,
}es8388_cha_sel_t;


/**
 * @brief 8388主从模式设置
 * 
 */
typedef enum
{
    //!8388做从模式
    ES8388_MODE_SLAVE = 0,
    //!8388做主模式
    ES8388_MODE_MASTER = 1,
}es8388_mode_sel_t;


/**
 * @brief 8388数据宽度设置
 * 
 */
typedef enum
{
    //!8388 ADC数据宽度为24bit
    ES8388_ADC_DATA_LEN_24BIT =(0x0<<2),
    //!8388 ADC数据宽度为20bit
    ES8388_ADC_DATA_LEN_20BIT =(0x1<<2),
    //!8388 ADC数据宽度为18bit
    ES8388_ADC_DATA_LEN_18BIT =(0x2<<2),
    //!8388 ADC数据宽度为16bit
    ES8388_ADC_DATA_LEN_16BIT =(0x3<<2),
    //!8388 ADC数据宽度为32bit
    ES8388_ADC_DATA_LEN_32BIT =(0x4<<2),
}es8388_adc_data_len_t;


/**
 * @brief 8388数据格式设置
 * 
 */
typedef enum
{
    //!8388 ADC数据格式为标准IIS格式
    ES8388_ADC_DATA_FORMAT_I2S           =(0x0<<0),
    //!8388 ADC数据格式为左对齐格式
    ES8388_ADC_DATA_FORMAT_LEFT_JUSTIFY  =(0x1<<0),
    //!8388 ADC数据格式为右对齐格式
    ES8388_ADC_DATA_FORMAT_RIGHT_JUSTIFY =(0x2<<0),
    //!8388 ADC数据格式为PCM格式
    ES8388_ADC_DATA_FORMAT_PCM_JUSTIFY   =(0x3<<0),
}es8388_adc_data_format_t;


/**
 * @brief 8388 ADC input channal选择的结构体
 * 
 */
typedef struct 
{
    //!ADC 左通道输入模式选择
    uint8_t l_input_single_sel;
    //!ADC 右通道输入模式选择
    uint8_t r_input_single_sel;
    uint8_t ds_sel;  //
    uint8_t dsr;  //
    uint8_t dsl;  //
    uint8_t monomix;
    uint8_t tri;
}es8388_input_cha_t;   


/**
 * @brief 8388 ALC调节上限
 * 
 */
typedef enum
{
    //!8388 ALC调节增益上限为-6.5dB
    ES8388_ALC_MAXGAIN__6_5dB = (0<<3),
    //!8388 ALC调节增益上限为-0.5dB
    ES8388_ALC_MAXGAIN__0_5dB = (1<<3),
    //!8388 ALC调节增益上限为5.5dB
    ES8388_ALC_MAXGAIN_5_5dB =  (2<<3),
    //!8388 ALC调节增益上限为11.5dB
    ES8388_ALC_MAXGAIN_11_5dB = (3<<3),
    //!8388 ALC调节增益上限为17.5dB
    ES8388_ALC_MAXGAIN_17_5dB = (4<<3),
    //!8388 ALC调节增益上限为23.5dB
    ES8388_ALC_MAXGAIN_23_5dB = (5<<3),
    //!8388 ALC调节增益上限为29.5dB
    ES8388_ALC_MAXGAIN_29_5dB = (6<<3),
    //!8388 ALC调节增益上限为35.5dB
    ES8388_ALC_MAXGAIN_35_5dB = (7<<3), 
}es8388_alc_maxgain_t;//ALC最大增益调节上限


/**
 * @brief 8388 ALC调节下限
 * 
 */
typedef enum
{
    //!8388 ALC调节增益下限为-12dB
    ES8388_ALC_MINGAIN__12dB =  (0<<0),
    //!8388 ALC调节增益下限为-6dB
    ES8388_ALC_MINGAIN__6dB =   (1<<0),
    //!8388 ALC调节增益下限为0dB
    ES8388_ALC_MINGAIN_0dB =    (2<<0),
    //!8388 ALC调节增益下限为6dB
    ES8388_ALC_MINGAIN_6dB =    (3<<0),
    //!8388 ALC调节增益下限为12dB
    ES8388_ALC_MINGAIN_12dB =   (4<<0),
    //!8388 ALC调节增益下限为18dB
    ES8388_ALC_MINGAIN_18dB =   (5<<0),
    //!8388 ALC调节增益下限为24dB
    ES8388_ALC_MINGAIN_24dB =   (6<<0),
    //!8388 ALC调节增益下限为30dB
    ES8388_ALC_MINGAIN_30dB =   (7<<0),
}es8388_alc_mingain_t;//ALC最小增益调节下限


/**
 * @brief 8388 ALC目标幅值（dB为单位）
 * 
 */
typedef enum
{
    //!目标幅值:-16.5dB
    ES8388_ALC_LEVEL__16_5dB = (0<<4),
    //!目标幅值:-15dB
    ES8388_ALC_LEVEL__15dB   = (1<<4),
    //!目标幅值:-13.5dB
    ES8388_ALC_LEVEL__13_5dB = (2<<4),
    //!目标幅值:-12dB
    ES8388_ALC_LEVEL__12dB   = (3<<4),
    //!目标幅值:-10.5dB
    ES8388_ALC_LEVEL__10_5dB = (4<<4),
    //!目标幅值:-9dB
    ES8388_ALC_LEVEL__9dB    = (5<<4),
    //!目标幅值:-7.5dB
    ES8388_ALC_LEVEL__7_5dB  = (6<<4),
    //!目标幅值:-6dB
    ES8388_ALC_LEVEL__6dB    = (7<<4),
    //!目标幅值:-4.5dB
    ES8388_ALC_LEVEL__4_5dB  = (8<<4),
    //!目标幅值:-3dB
    ES8388_ALC_LEVEL__3dB    = (9<<4),
    //!目标幅值:-1.5dB
    ES8388_ALC_LEVEL__1_5dB  = (10<<4),
}es8388_alc_level_t;//ALC调节声音的目标幅度


/**
 * @brief 8388 ALC通道选择
 * 
 */
typedef enum
{
    //!8388 ALC关闭
    ES8388_ALC_OFF      = (0<<6),
    //!8388 ALC 只开启右通道
    ES8388_ALC_R_ONLY   = (1<<6),
    //!8388 ALC 只开启左通道
    ES8388_ALC_L_ONLY   = (2<<6),
    //!8388 ALC开启双通道
    ES8388_ALC_STEREO   = (3<<6),
}es8388_alc_cha_gate_t;


/**
 * @brief 8388 ALC配置
 * 
 */
typedef struct 
{
    //!8388 ALC 最大增益设置
    es8388_alc_maxgain_t alc_maxgain;    
    //!8388 ALC 最小增益设置
    es8388_alc_mingain_t alc_mingain;    
    //!8388 ALC 目标幅值设置
    es8388_alc_level_t alc_level;      
}es8388_alc_config_t;


typedef enum
{
    ES8388_NUM1 = 1,
    ES8388_NUM2 = 2,
    ES8388_NUM3 = 3,
}es8388_num_sel_t;


/**
 * @brief 8388 MIC增益
 * 
 */
typedef enum
{
    //!8388 MIC增益0dB
    ES8388_MICAMP_0dB = 0,
    //!8388 MIC增益3dB
    ES8388_MICAMP_3dB,
    //!8388 MIC增益6dB
    ES8388_MICAMP_6dB,
    //!8388 MIC增益9dB
    ES8388_MICAMP_9dB,
    //!8388 MIC增益12dB
    ES8388_MICAMP_12dB,
    //!8388 MIC增益15dB
    ES8388_MICAMP_15dB,
    //!8388 MIC增益18dB
    ES8388_MICAMP_18dB,
    //!8388 MIC增益21dB
    ES8388_MICAMP_21dB,
    //!8388 MIC增益24dB
    ES8388_MICAMP_24dB,
    //!8388 MIC增益27dB
    ES8388_MICAMP_27dB,
    //!8388 MIC增益30dB
    ES8388_MICAMP_30dB,
    //!8388 MIC增益33dB
    ES8388_MICAMP_33dB,
}es8388_mic_amp_t;


/**
 * @brief 8388 ADC数据极性选择，ENABLE和DISABLE的数据的相位有180度的差距
 * 
 */
typedef enum
{
    //!8388 ADC采集的数据相位不变
    ES8388_ADC_INV_DISABLE = 0,
    //!8388 ADC采集的数据相位调整180度
    ES8388_ADC_INV_ENABLE = 1,
}es8388_adc_inv_sel_t;


/**
 * @brief 8388 DAC数据宽度
 * 
 */
typedef enum
{
    //!8388 DAC数据宽度24bit
    ES8388_DAC_DATA_LEN_24BIT = (0x0<<3),
    //!8388 DAC数据宽度20bit
    ES8388_DAC_DATA_LEN_20BIT = (0x1<<3),
    //!8388 DAC数据宽度18bit
    ES8388_DAC_DATA_LEN_18BIT = (0x2<<3),
    //!8388 DAC数据宽度16bit
    ES8388_DAC_DATA_LEN_16BIT = (0x3<<3),
    //!8388 DAC数据宽度32bit
    ES8388_DAC_DATA_LEN_32BIT = (0x4<<3),
}es8388_dac_data_len_t;


/**
 * @brief 8388 DAC数据格式
 * 
 */
typedef enum
{
    //!8388 DAC数据格式为标准IIS格式
    ES8388_DAC_DATA_FORMAT_I2S           =(0x0<<1),
    //!8388 DAC数据格式为左对齐格式
    ES8388_DAC_DATA_FORMAT_LEFT_JUSTIFY  =(0x1<<1),
    //!8388 DAC数据格式为右对齐格式
    ES8388_DAC_DATA_FORMAT_RIGHT_JUSTIFY =(0x2<<1),
    //!8388 DAC数据格式为PCM格式
    ES8388_DAC_DATA_FORMAT_PCM_JUSTIFY   =(0x3<<1),
}es8388_dac_data_format_t;

//ADC
int32_t ES8388_ADC_data_format_set(es8388_num_sel_t num_8388,es8388_adc_data_len_t data_length,es8388_adc_data_format_t data_format);
int32_t ES8388_ADC_mic_amplify_set(es8388_num_sel_t num_8388,es8388_mic_amp_t l_amp,es8388_mic_amp_t r_amp);
int32_t ES8388_ALC_config(es8388_num_sel_t num_8388,es8388_alc_config_t* ALC_Type,es8388_alc_cha_gate_t ALC_gate);
int32_t ES8388_ADC_invl_config(es8388_num_sel_t num_8388,es8388_adc_inv_sel_t gate_l,es8388_adc_inv_sel_t gate_r);
void ES8388_aec_setting(es8388_num_sel_t num_8388);
void ES8388_alc_gate(es8388_num_sel_t num_8388,es8388_alc_cha_gate_t ALC_gate);
void ES8388_alc_max_gain_set(es8388_num_sel_t num_8388,es8388_alc_maxgain_t max_gain);

//DAC
int32_t ES8388_DAC_data_format_set(es8388_num_sel_t num_8388,es8388_dac_data_len_t data_length,es8388_dac_data_format_t data_format);
int32_t ES8388_dac_vol_set(es8388_num_sel_t num_8388,int32_t l1vol,int32_t r1vol,int32_t l2vol,int32_t r2vol);
void ES8388_DAC_mute(es8388_num_sel_t num_8388,FunctionalState cmd);
void ES8388_DAC_power(es8388_num_sel_t num_8388,es8388_cha_sel_t cha,FunctionalState cmd);

//ALL
int32_t ES8388_play_init(es8388_num_sel_t es8388_num);



#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

