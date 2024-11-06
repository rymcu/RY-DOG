/**
 * @file es8311.h
 * @brief 
 * @version 0.1
 * @date 2019-03-28
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */


#ifndef __ES8311_H
#define __ES8311_H

#include <stdint.h>
#include "ci130x_system.h"


/**
 * @ingroup third_device_driver
 * @defgroup es8311 es8311
 * @brief es8311的驱动
 */

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup es8311
 * @{
 */


/**
 * @brief 8311 ALC调节上限
 * 
 */
typedef enum
{
    //!8311 ALC调节增益上限为-6.5dB
    ES8311_ALC_MAXGAIN__6_5dB = (0<<3),
    //!8311 ALC调节增益上限为-0.5dB
    ES8311_ALC_MAXGAIN__0_5dB = (1<<3),
    //!8311 ALC调节增益上限为5.5dB
    ES8311_ALC_MAXGAIN_5_5dB =  (2<<3),
    //!8311 ALC调节增益上限为11.5dB
    ES8311_ALC_MAXGAIN_11_5dB = (3<<3),
    //!8311 ALC调节增益上限为17.5dB
    ES8311_ALC_MAXGAIN_17_5dB = (4<<3),
    //!8311 ALC调节增益上限为23.5dB
    ES8311_ALC_MAXGAIN_23_5dB = (5<<3),
    //!8311 ALC调节增益上限为29.5dB
    ES8311_ALC_MAXGAIN_29_5dB = (6<<3),
    //!8311 ALC调节增益上限为35.5dB
    ES8311_ALC_MAXGAIN_35_5dB = (7<<3), 
}es8311_alc_maxgain_t;//ALC最大增益调节上限


/**
 * @brief 8311 ALC调节下限
 * 
 */
typedef enum
{
    //!8311 ALC调节增益下限为-12dB
    ES8311_ALC_MINGAIN__12dB =  (0<<0),
    //!8311 ALC调节增益下限为-6dB
    ES8311_ALC_MINGAIN__6dB =   (1<<0),
    //!8311 ALC调节增益下限为0dB
    ES8311_ALC_MINGAIN_0dB =    (2<<0),
    //!8311 ALC调节增益下限为6dB
    ES8311_ALC_MINGAIN_6dB =    (3<<0),
    //!8311 ALC调节增益下限为12dB
    ES8311_ALC_MINGAIN_12dB =   (4<<0),
    //!8311 ALC调节增益下限为18dB
    ES8311_ALC_MINGAIN_18dB =   (5<<0),
    //!8311 ALC调节增益下限为24dB
    ES8311_ALC_MINGAIN_24dB =   (6<<0),
    //!8311 ALC调节增益下限为30dB
    ES8311_ALC_MINGAIN_30dB =   (7<<0),
}es8311_alc_mingain_t;//ALC最小增益调节下限


/**
 * @brief 8311 ALC目标幅值（dB为单位）
 * 
 */
typedef enum
{
    //!目标幅值:-30.1dB
    ES8311_ALC_LEVEL__30_1dB = 0,
    ES8311_ALC_LEVEL__24_1dB,
    ES8311_ALC_LEVEL__20_6dB,
    ES8311_ALC_LEVEL__18_1dB,
    ES8311_ALC_LEVEL__16_1dB,
    ES8311_ALC_LEVEL__14_5dB ,
    ES8311_ALC_LEVEL__13_2dB,
    ES8311_ALC_LEVEL__12dB,
    ES8311_ALC_LEVEL__11dB,
    ES8311_ALC_LEVEL__10_1dB,
    ES8311_ALC_LEVEL__9_3dB,
    ES8311_ALC_LEVEL__8_5dB,
    ES8311_ALC_LEVEL__7_8dB,
    ES8311_ALC_LEVEL__7_2dB,
    ES8311_ALC_LEVEL__6_6dB,
    ES8311_ALC_LEVEL__6dB,
}es8311_alc_level_t;//ALC调节声音的目标幅度


typedef enum
{
    ES8311_PGA_GAIN_0dB = 0,
    ES8311_PGA_GAIN_3dB,
    ES8311_PGA_GAIN_6dB,
    ES8311_PGA_GAIN_9dB,
    ES8311_PGA_GAIN_12dB,
    ES8311_PGA_GAIN_15dB,
    ES8311_PGA_GAIN_18dB,
    ES8311_PGA_GAIN_21dB,
    ES8311_PGA_GAIN_24dB,
    ES8311_PGA_GAIN_27dB,
    ES8311_PGA_GAIN_30dB,
}es8311_pga_gain_t;


typedef enum
{
    ES8311_ADC_GAIN_SCALE_UP_0dB = 0,
    ES8311_ADC_GAIN_SCALE_UP_6dB,
    ES8311_ADC_GAIN_SCALE_UP_12dB,
    ES8311_ADC_GAIN_SCALE_UP_18dB,
    ES8311_ADC_GAIN_SCALE_UP_24dB,
    ES8311_ADC_GAIN_SCALE_UP_30dB,
    ES8311_ADC_GAIN_SCALE_UP_36dB,
    ES8311_ADC_GAIN_SCALE_UP_42dB,
}es8311_adc_gain_scale_up_t;


/**
 * @brief 8311 ALC通道选择
 * 
 */
typedef enum
{
    //!8311 ALC关闭
    ES8311_ALC_OFF  = (0<<7),
    //!8311 ALC 只开启右通道
    ES8311_ALC_ON   = (1<<7),
}es8311_alc_gate_t;


/**
 * @brief 8311 ALC配置
 * 
 */
typedef struct 
{
    //!8311 ALC maxgain
    float max_gain;
    //!8311 ALC 目标幅值设置
    es8311_alc_level_t alc_max_level;      
    //!8311 ALC 目标幅值设置
    es8311_alc_level_t alc_min_level;      
}es8311_alc_config_t;


typedef enum
{
    ES8311_AUTO_MUTE_WS_42MS = 0,
    ES8311_AUTO_MUTE_WS_84MS,
    ES8311_AUTO_MUTE_WS_126MS,
    ES8311_AUTO_MUTE_WS_168MS,
    ES8311_AUTO_MUTE_WS_210MS,
    ES8311_AUTO_MUTE_WS_252MS,
    ES8311_AUTO_MUTE_WS_294MS,
    ES8311_AUTO_MUTE_WS_336MS,
    ES8311_AUTO_MUTE_WS_378MS,
    ES8311_AUTO_MUTE_WS_420MS,
    ES8311_AUTO_MUTE_WS_462MS,
    ES8311_AUTO_MUTE_WS_504MS,
    ES8311_AUTO_MUTE_WS_546MS,
    ES8311_AUTO_MUTE_WS_588MS,
    ES8311_AUTO_MUTE_WS_630MS,
    ES8311_AUTO_MUTE_WS_672MS,
}es8311_auto_mute_ws_t;


typedef enum
{
    ES8311_AUTOMUTE_NG__96dB = 0,
    ES8311_AUTOMUTE_NG__90dB,
    ES8311_AUTOMUTE_NG__84dB,
    ES8311_AUTOMUTE_NG__78dB,
    ES8311_AUTOMUTE_NG__72dB,
    ES8311_AUTOMUTE_NG__66dB,
    ES8311_AUTOMUTE_NG__60dB,
    ES8311_AUTOMUTE_NG__54dB,
    ES8311_AUTOMUTE_NG__51dB,
    ES8311_AUTOMUTE_NG__48dB,
    ES8311_AUTOMUTE_NG__45dB,
    ES8311_AUTOMUTE_NG__42dB,
    ES8311_AUTOMUTE_NG__39dB,
    ES8311_AUTOMUTE_NG__36dB,
    ES8311_AUTOMUTE_NG__33dB,
    ES8311_AUTOMUTE_NG__30dB,
}es8311_automute_ng_t;


typedef enum
{
    ES8311_AUTOMUTE_VOL_0dB = 0,
    ES8311_AUTOMUTE_VOL__4dB,
    ES8311_AUTOMUTE_VOL__8dB,
    ES8311_AUTOMUTE_VOL__12dB,
    ES8311_AUTOMUTE_VOL__16dB,
    ES8311_AUTOMUTE_VOL__20dB,
    ES8311_AUTOMUTE_VOL__24dB,
    ES8311_AUTOMUTE_VOL__28dB,
}es8311_automute_vol_t; 


typedef enum
{
    ES8311_AUTO_MUTE_DISABLE = 0,
    ES8311_AUTO_MUTE_ENABLE = 1,
}es8311_auto_mute_en_t;


typedef struct
{
    es8311_auto_mute_en_t auto_mute_en;
    es8311_automute_ng_t ng;
    es8311_automute_vol_t vol;
    es8311_auto_mute_ws_t ws;
}es8311_auto_mute_config_t;

void es8311_init(void);
void es8311_pga_gain(es8311_pga_gain_t gain);
void es8311_adc_gain_scale_up(es8311_adc_gain_scale_up_t scale_up);
int8_t es8311_auto_mute_set(es8311_auto_mute_config_t* str);
int8_t es8311_dac_vol_set(int8_t vol);
void es8311_i2c_init(void);
void es8311_dac_off(void);
void es8311_dac_on(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

