/**
 * @file ci130x_codec.c
 * @brief CI130X inner codec的驱动函数
 * @version 0.1
 * @date 2019-03-29
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
//TODO:在2代的基础上，改的3代的驱动，双声道ADC，单声道DAC

#include <math.h>
#include <string.h>
#include "ci130x_codec.h"
#include "ci130x_scu.h"
#include "ci130x_system.h"
#include "ci130x_gpio.h"
#include "ci_assert.h"
#include "romlib_runtime.h"
#include "sdk_default_config.h"

#define CODEC_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("CODEC Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

#define USE_CI130X_OWN_ALC 0//目前没有用到CI130X自带的ALC，为了省代码，用这个宏可以不编译alc.c



uint8_t get_pga_gain_reg_data_from_db(int32_t gain_db)
{
    if(28 < gain_db)
    {
        return 0x1f;
    }
    else if(-18.0f > gain_db)
    {
        return 0;
    }

    uint8_t reg_pga_gain = (uint8_t)(((float)(gain_db + 18)) / 1.5f);

    return (reg_pga_gain)&0x1f;
}


/**
 * @brief ADC电流源强制启动
 * 
 */
void inner_codec_up_ibas_adc(void)
{
	CODEC->reg23 |= 1<<6;
	_delay_10us_240M(100);
	CODEC->reg23 &= ~(1<<6);
	_delay_10us_240M(100);
}


/**
 * @brief DAC电流源强制启动
 * 
 */
void inner_codec_up_ibas_dac(void)
{
	CODEC->reg2a |= 1<<0;
	_delay_10us_240M(100);
	CODEC->reg2a &= ~(1<<0);
	_delay_10us_240M(100);
}


/**
 * @brief INNER CODEC reset
 * 
 */
void inner_codec_reset(void)
{
    CODEC->reg0 = 0;
    _delay_10us_240M(500);
    CODEC->reg0 = 0x3;
    _delay_10us_240M(500);
}

/**
 * @brief CODEC作为ADC的模式选择
 *
 * @param mode Master模式或者Slave模式；
 * @param frame_Len 数据宽度；
 * @param word_len 有效数据长度；
 * @param data_fram 数据格式
 */
void inner_codec_adc_mode_set(inner_codec_mode_t mode,inner_codec_frame_1_2len_t frame_Len,
                    inner_codec_valid_word_len_t word_len,inner_codec_i2s_data_famat_t data_fram)
{
    CODEC->reg3 &= ~((0x03<<4) | (0x03<<2));
    CODEC->reg3 |= ((mode<<4) | (frame_Len<<2));

    CODEC->reg2 &= ~((0x03<<5) | (0x03<<3));
    CODEC->reg2 |= ((word_len<<5) | (data_fram<<3));
}


/**
 * @brief CODEC作为DAC的模式选择（3代不打算开放配置了，本来都是内部使用）
 *
 * @param mode Master模式或者Slave模式；
 * @param frame_Len 数据宽度；
 * @param word_len 有效数据长度；
 * @param data_fram 数据格式
 */
void inner_codec_dac_mode_set(inner_codec_mode_t mode,inner_codec_frame_1_2len_t frame_Len,
                    inner_codec_valid_word_len_t word_len,inner_codec_i2s_data_famat_t data_fram)
{
    CODEC->reg4 = 0x10;
    CODEC->reg5 = 0x2;
}


/**
 * @brief CODEC模块上电
 *
 * @param current pre-charge/dis-charge的电流大小
 */
void inner_codec_power_up(inner_codec_current_t current)
{
    CODEC->reg2a &= ~((0x03<<4));
    CODEC->reg2a |= ((0x01<<4));
    CODEC->reg22 = 0x01;
    CODEC->reg23 |= (0x01<<5);
    CODEC->reg22 = 0xff;
    _delay_10us_240M(2000);
    CODEC->reg22 = current;
}


/**
 * @brief 关闭CODEC模块电源
 *
 */
void inner_codec_power_off(void)
{
    CODEC->reg22 = 0x01;
    CODEC->reg23 &= ~(0x01<<5);
    CODEC->reg22 = 0xff;
    _delay_10us_240M(500);
}


/**
 * @brief 配置CODEC模块的高通滤波器
 *
 * @param gate 高通滤波器开关
 * @param Hz 高通滤波器的截止频率
 */
void inner_codec_hp_filter_config(inner_cedoc_gate_t gate,inner_codec_highpass_cut_off_t Hz)
{
    CODEC->rega &= ~(0x07<<0);
    CODEC->rega |= ((!gate)<<2) | (Hz<<0);
    _delay_10us_240M(100);
}


/**
 * @brief 设置ADC的输入模式
 * 
 * @param mode 
 */
void inner_codec_set_input_mode(inner_codec_cha_sel_t cha,inner_codec_input_mode_t mode)
{
    //左:bit6，右:bit2
    uint8_t offset = (1 - cha)*4 + 2;
    CODEC->reg26 &= ~(0x03<<offset);
    CODEC->reg26 |= (mode<<offset);
}


/**
 * @brief 设置ADC的MIC增益
 * 
 * @param gain 
 */
static void inner_codec_set_mic_gain(inner_codec_cha_sel_t cha,inner_codec_mic_amplify_t gain)
{
    CODEC->pga_gain_reg[cha] &= ~(0x7 << 5);
    CODEC->pga_gain_reg[cha] |= (gain << 5);
    _delay_10us_240M(100);
}


/**
 * @brief 使能CODEC中的ADC:
 *        1.使用此函数之前先初始化CODEC_ADC_Config_TypeDef结构体并填写参数；
 *        2.ALCL_Gain和ALCR_Gain的上下限分别是28.5dB和-18dB，步长1.5。
 *
 * @param ADC_Config CODEC_ADC_Config_TypeDef结构体指针
 */
void inner_codec_adc_enable(inner_codec_adc_config_t *ADC_Config)
{
    CODEC->reg26 &= ~((0x03<<2)|(0x03<<6));
    CODEC->reg26 |= ((ADC_Config->codec_adc_input_mode_r<<2)|(ADC_Config->codec_adc_input_mode_l<<6));
    _delay_10us_240M(100);
    CODEC->reg24 |= ((0x01<<3)|(0x01<<7) | (0x01<<1)|(0x01<<5));
    _delay_10us_240M(100);
    CODEC->reg23 |= ((0x07<<0) | (0x07<<3));

    _delay_10us_240M(100);
    inner_codec_up_ibas_adc();
    CODEC->reg26 |= ((0x01<<0)|(0x01<<4));
    _delay_10us_240M(500);
    CODEC->reg26 |= ((0x01<<1)|(0x01<<5));
    _delay_10us_240M(500);

    CODEC->reg29 = 0xff;
    _delay_10us_240M(500);
    CODEC->reg24 |= ((0x01<<2)|(0x01<<6));
    _delay_10us_240M(100);

    inner_codec_set_mic_gain(INNER_CODEC_LEFT_CHA,ADC_Config->codec_adc_mic_amp_l);
    inner_codec_set_mic_gain(INNER_CODEC_RIGHT_CHA,ADC_Config->codec_adc_mic_amp_r);
    _delay_10us_240M(100); 
    CODEC->reg25 &= ~(0x0f<<0);//为ADC选择合适的偏置电流
    CODEC->reg25 |= (5<<0);
    _delay_10us_240M(100); 
    
    uint32_t reg_data = CODEC->pga_gain_reg[0];
    reg_data &= ~(0x1f<<0);
    reg_data |= get_pga_gain_reg_data_from_db((int32_t)ADC_Config->pga_gain_l);
    CODEC->pga_gain_reg[0]  = reg_data;
    _delay_10us_240M(100); 

    reg_data = CODEC->pga_gain_reg[1];
    reg_data &= ~(0x1f<<0);
    reg_data |= get_pga_gain_reg_data_from_db((int32_t)ADC_Config->pga_gain_r);
    CODEC->pga_gain_reg[1]  = reg_data;
    _delay_10us_240M(100); 
    // CODEC->reg24 |= ((0x01<<0)|(0x01<<4));
}




/**
 * @brief 调节CODEC ADC的数字增益（197为0dB，加减1为加减0.5dB增益）
 *
 * @param gain 左声道增益大小
 */
void inner_codec_adc_dig_gain_set(inner_codec_cha_sel_t cha,int32_t gain)
{
    int32_t gain_data_reg = 0;//((int32_t)(gain * 0.5f) + 197);
    if(gain > 28.5f)
    {
        gain_data_reg = 0xff;
    }
    else if(gain < -98.5f)
    {
        gain_data_reg = 0;
    }
    else
    {
        gain_data_reg = (int32_t)((float)gain * 2) + 197;
    }
    CODEC->adc_dig_gain_reg[cha] = (uint8_t)gain_data_reg;
}


/**
 * @brief 关闭CODEC中的ADC
 *
 * @param cha 左右通道、全部通道的选择；
 * @param EN 选择是否关闭ADC
 */
void inner_codec_adc_disable(inner_codec_cha_sel_t cha, inner_cedoc_gate_t EN)
{
    uint8_t offset = (1 - cha) * 4;
    if(EN)
    {
        CODEC->reg24 &= ~((uint32_t)0x07 << offset);
        CODEC->reg29 &= ~((uint32_t)0x0f << offset);
        CODEC->reg26 &= ~((uint32_t)0x03 << offset);
    }
}


/**
 * @brief 配置CODEC中的DAC增益,-39dB到7.5dB，步长1.5dB，真实为32档。
 *
 * @param
 * @param
 */
static void icodec_dac_gain_set(inner_codec_cha_sel_t cha,int32_t gain)
{
    float step = 100.0f/31.0f;
    uint32_t reg_l_gain = 0;
    if(gain >= 100)
    {
        gain = 100;
    }

    if(gain < 0)
    {
        gain = 0;
    }

    if(0 == gain)
    {
        //音量调为0的时候，数字增益也调为0
        CODEC->reg6 = 0;
    }
    else
    {
        //音量调不为0的时候，数字增益调为默认值
        CODEC->reg6 = 0xeb;
    }

    reg_l_gain = (uint32_t)( (float)gain/step );

    CODEC->reg2d  = reg_l_gain;
}


/**
 * @brief DAC数字增益设置（10dB -- -117dB，步进1dB）
 * 
 * @param dig_gain 
 */
void inner_codec_dac_dig_gain_set(int16_t dig_gain)
{
    if(dig_gain < -117)
    {
        CODEC->reg6 = 0;
        return;
    }
    else if(dig_gain > 10)
    {
        CODEC->reg6 = 0xff;
        return;
    }
    else
    {
        uint32_t reg_data = 0;
        reg_data = (uint32_t)(2*dig_gain + 235);
        CODEC->reg6 = reg_data;
    }
}


void inner_codec_dac_enable(bool is_first_enable)
{
    CODEC->reg2b &= ~((0x01<<2));
    CODEC->reg2a |= (0x01<<7);
    _delay_10us_240M(100);
    if(is_first_enable)
    {
        inner_codec_up_ibas_dac();
    }
    CODEC->reg2a &= ~((0x03<<4));
    CODEC->reg2a |= ((0x02<<4) | (0x01<<6));
    CODEC->reg2b |= ((0x0f<<4) | (0x03<<0));
    if(is_first_enable)
    {
        inner_codec_dac_dig_gain_set(0);
    }
    CODEC->reg2c = 0x1f;
    _delay_10us_240M(500);
    CODEC->reg2b |= ((0x01<<2));
}


/**
 * @brief 关闭DAC
 *
 * @param cha 声道选择
 * @param EN 开关
 */
void inner_codec_dac_disable(void)
{
    CODEC->reg2b &= ~((0x07<<0) | (0x0f<<4));
    CODEC->reg2a = (0x01<<4);
}


/**
 * @brief hpout mute的开关，不关DAC
 * 
 * @param en 
 */
static void inner_codec_hpout_is_mute(bool en)
{
    if(en)
    {
        CODEC->reg2b &= ~(1<<2);
    }
    else
    {
        CODEC->reg2b |= (1<<2);
    }
}


/**
 * @brief ALC判断的信号来自高通滤波器之前，还是之后
 *
 * @param judge 选择inner CODEC的判断信号来自ALC之前还是之后
 */
void inner_codec_alc_judge_sel(inner_codec_alc_judge_t judge)
{
    CODEC->rega  &= ~(0x01<<6);

    CODEC->rega  |= ((1&judge)<<6);
}


/**
 * @brief inner codec低通滤波器类型选择
 * 
 * @param mode 
 */
void inner_codec_lp_filter_sel(inner_codec_lp_filter_mode_t mode)
{
    uint32_t reg_data = CODEC->rega;
    reg_data &= ~(1<<7);
    reg_data |= (mode<<7);
    CODEC->rega = reg_data;
}


/**
 * @brief 
 * 
 * @param cha 使能ALC
 * @param gate 打开/关闭ALC
 * @param is_alc_ctr_pga 是否使用ALC控制PGA增益（这个gate是需要的，因为存在使能了ALC，但是不使用ALC控制PGA增益的情况）
 */
void inner_codec_alc_enable(inner_codec_cha_sel_t cha,inner_cedoc_gate_t gate,inner_codec_use_alc_control_pgagain_t is_alc_ctr_pga)
{
    uint32_t reg_tmp;

    uint8_t bit_offset = 4 + (1 - cha);
    reg_tmp = CODEC->rega;
    reg_tmp &= ~(0x01 << bit_offset);
    reg_tmp |= (is_alc_ctr_pga << bit_offset);
    CODEC->rega = reg_tmp;

    CODEC->alc_reg[cha].reg44 |= 1<<3;

    reg_tmp = CODEC->alc_reg[cha].reg49;
    reg_tmp &= ~(0x01 << 6);
    reg_tmp |= ((0x01&gate) << 6);
    CODEC->alc_reg[cha].reg49 = reg_tmp;
}


/**
 * @brief 关闭CODEC中的ALC，并切换到 pga_gain_reg 控制PGA增益
 *
 * @param cha 通道选择
 * @param ALC_Gain 关闭ALC之后，使用寄存器0x27控制ALC增益的大小
 */
void inner_codec_alc_disable(inner_codec_cha_sel_t cha)
{
    CODEC->alc_reg[cha].reg49 &= ~(0x01<<6);

    uint8_t bit_offset = 4 + (1 - cha);
    CODEC->rega  &= ~(0x01<<bit_offset);
}



void inner_codec_set_pga_gain(inner_codec_cha_sel_t cha,int32_t ALC_Gain)
{
    uint32_t reg_data = CODEC->pga_gain_reg[cha];
    reg_data &= ~(0x1f<<0);
    reg_data |= get_pga_gain_reg_data_from_db(ALC_Gain);
    CODEC->pga_gain_reg[cha]  = reg_data;
}


//ALC默认配置
#define METHOD_TO_CONTROL_GAIN_ATTACK   0
#define ALC_JACK_WAY    INNER_CODEC_ALC_MODE_JACKWAY2
#define ALC_HOLD_TIME   INNER_CODEC_ALC_HOLD_TIME_2MS
#define ALC_DECAY_TIME  INNER_CODEC_ALC_DECAY_TIME_2MS
#define ALC_ATTACK_TIME INNER_CODEC_ALC_ATTACK_TIME_2MS
#define ALC_MODE        0
#define ALC_ZERO_CROSS  INNER_CODEC_GATE_DISABLE
#define ALC_FAST_DECAY_87_5 INNER_CODEC_GATE_DISABLE
#define ALC_NOISE_GATE  INNER_CODEC_GATE_DISABLE
#define ALC_NOISE_GATE_SET  INNER_CODEC_NOISE_GATE_THRE_45dB
#define ALC_INIT_PGA_GAIN   0x1f
#define ALC_SAMPLE_RATE     INNER_CODEC_SAMPLERATE_16K
#define ALC_LOW_8_BIT_MAX_LEVEL     ((uint8_t)(INNER_CODEC_ALC_LEVEL__6dB >> 0))
#define ALC_HOGH_8_BIT_MAX_LEVEL    ((uint8_t)(INNER_CODEC_ALC_LEVEL__6dB >> 8))
#define ALC_LOW_8_BIT_MIN_LEVEL     ((uint8_t)(INNER_CODEC_ALC_LEVEL__7_5dB >> 0))
#define ALC_HOGH_8_BIT_MIN_LEVEL    ((uint8_t)(INNER_CODEC_ALC_LEVEL__7_5dB >> 8))
#define ALC_AGC_FUNC_SEL    ENABLE
#define ALC_PGA_MAX_GAIN    INNER_CODEC_ALC_PGA_MAX_GAIN_28_5dB
#define ALC_PGA_MIN_GAIN    INNER_CODEC_ALC_PGA_MIN_GAIN__18dB

#define ALC_DIG_GAIN    (197)

/**
 * @brief ALC的默认配置
 * 
 */
void alc_init_config(void)
{
    uint32_t alc_reg_init_config_data[10] = 
    {
        ((METHOD_TO_CONTROL_GAIN_ATTACK << 6) | (ALC_JACK_WAY << 4) | (ALC_HOLD_TIME << 0)),
        ((ALC_DECAY_TIME << 4) | (ALC_ATTACK_TIME << 0)),
        ((ALC_MODE << 7) | (ALC_ZERO_CROSS << 6) | (ALC_FAST_DECAY_87_5 << 4) | (ALC_NOISE_GATE << 3) | (ALC_NOISE_GATE_SET << 0)),
        ((ALC_ZERO_CROSS << 5) | (ALC_INIT_PGA_GAIN << 0)),
        ((1 << 3) | (ALC_SAMPLE_RATE << 0)),
        (ALC_LOW_8_BIT_MAX_LEVEL),
        (ALC_HOGH_8_BIT_MAX_LEVEL),
        (ALC_LOW_8_BIT_MIN_LEVEL),
        (ALC_HOGH_8_BIT_MIN_LEVEL),
        ((ALC_AGC_FUNC_SEL << 6) | (ALC_PGA_MAX_GAIN << 3) | (ALC_PGA_MIN_GAIN) << 0),
    };
    for(int i=0;i<2;i++)
    {
        // CODEC->adc_dig_gain_reg[i] = ALC_DIG_GAIN;
        uint32_t* reg_data_p = (uint32_t*)(uint32_t)(&(CODEC->alc_reg[i].reg40));
        for(int j=0;j<10;j++)
        {
            reg_data_p[j] = alc_reg_init_config_data[j];
        }
    }
}


/**
 * @brief 设置MICBIAS的大小
 * 
 * @param bias 
 */
void inner_codec_micbias_set(inner_codec_micbias_t bias)
{
    CODEC->reg23 &= ~(7<<0);
    CODEC->reg23 |= bias << 0;
}


#include "codec_manager.h"

#if INNER_CODEC_AUDIO_IN_USE_RESAMPLE
#define LPF_CONFIG  (INNER_CODEC_LP_FILTER_IIR)
#else
#define LPF_CONFIG  (INNER_CODEC_LP_FILTER_FIR)
#endif

#define ALC_JUDGE   (INNER_CODEC_ALC_JUDGE_AFTER_HIGHPASS)
#define ALC_CONTROL_PGA_L (INNER_CODEC_GATE_ENABLE)
#define ALC_CONTROL_PGA_R (INNER_CODEC_GATE_ENABLE)
#define HPF_GATE    (0)//0为打开，1为关闭
#define HPF_CONFIG  (INNER_CODEC_HIGHPASS_CUT_OFF_20HZ)

/**
 * @brief 包括CODEC整个的初始化，ADC、DAC的初始化
 * 
 * @param codec_hw_info 
 * @return int 
 */
int icodec_init(cm_codec_hw_info_t * codec_hw_info)
{
    scu_set_device_gate((uint32_t)CODEC,ENABLE);
    scu_set_device_reset((uint32_t)CODEC);
    scu_set_device_reset_release((uint32_t)CODEC);

    inner_codec_reset();
    inner_codec_power_up(INNER_CODEC_CURRENT_128I);
    inner_codec_dac_enable(true);

    //ADC主从模式设置、数据格式
    inner_codec_mode_t code_mode[] = {INNER_CODEC_MODE_SLAVE,INNER_CODEC_MODE_MASTER};
    inner_codec_i2s_data_famat_t codec_adc_df[] = {INNER_CODEC_I2S_DATA_FORMAT_I2S_MODE,
                                        INNER_CODEC_I2S_DATA_FORMAT_LJ_MODE,
                                        INNER_CODEC_I2S_DATA_FORMAT_RJ_MODE};
    #if (!AUDIO_IN_FROM_DMIC)
    inner_codec_adc_config_t CODEC_ADC_Config_Struct;

    CODEC_ADC_Config_Struct.codec_adc_input_mode_l = codec_hw_info->codec_gain.codec_adc_input_mode_l;
    CODEC_ADC_Config_Struct.codec_adc_input_mode_r = codec_hw_info->codec_gain.codec_adc_input_mode_r;
    
    CODEC_ADC_Config_Struct.codec_adc_mic_amp_l = codec_hw_info->codec_gain.codec_adc_mic_amp_l;
    CODEC_ADC_Config_Struct.codec_adc_mic_amp_r = codec_hw_info->codec_gain.codec_adc_mic_amp_r;

    //如果不使能ALC，PGA的增益设置
    CODEC_ADC_Config_Struct.pga_gain_l = codec_hw_info->codec_gain.pga_gain_l;
    CODEC_ADC_Config_Struct.pga_gain_r = codec_hw_info->codec_gain.pga_gain_r;
    //ADC初始化
    inner_codec_adc_enable(&CODEC_ADC_Config_Struct);

    //ADC增益设置

    
    CODEC->reg3 &= ~(0x03 << 4)  | (0x03 << 2);
    CODEC->reg3 |= (code_mode[codec_hw_info->input_iis.iis_mode_sel] << 4) | (INNER_CODEC_FRAME_LEN_32BIT << 2);

    CODEC->reg2 &= ~(0x03 << 3);
    CODEC->reg2 |= codec_adc_df[codec_hw_info->input_iis.iis_data_format] << 3;

    //滤波器设置
    inner_codec_hp_filter_config(INNER_CODEC_GATE_ENABLE, INNER_CODEC_HIGHPASS_CUT_OFF_20HZ);
    inner_codec_lp_filter_sel(INNER_CODEC_LP_FILTER_IIR);
    inner_codec_alc_judge_sel(INNER_CODEC_ALC_JUDGE_AFTER_HIGHPASS);
    CODEC->rega = (LPF_CONFIG << 7) | (ALC_JUDGE << 6) | (ALC_CONTROL_PGA_L << 5) 
            | (ALC_CONTROL_PGA_R << 4) | (1 << 3) | (HPF_GATE << 2) | (HPF_CONFIG << 0);

    //ALC设置
    alc_init_config();

    inner_codec_adc_dig_gain_set(INNER_CODEC_LEFT_CHA,(int)(codec_hw_info->codec_gain.dig_gain_l));
    inner_codec_adc_dig_gain_set(INNER_CODEC_RIGHT_CHA,(int)(codec_hw_info->codec_gain.dig_gain_r));
    #endif

    //DAC设置
    inner_codec_dac_mode_set(code_mode[codec_hw_info->input_iis.iis_mode_sel],
                            0,
                            0,
                            codec_adc_df[codec_hw_info->input_iis.iis_data_format]);
}


int icodec_start(io_direction_t io_dir)
{
    if(CODEC_OUTPUT == io_dir)
    {
        #if IF_JUST_CLOSE_HPOUT_WHILE_NO_PLAY
        inner_codec_hpout_is_mute(false);
        #else
        inner_codec_dac_enable(false);
        #endif
    }
    return 0;
}

int icodec_stop(io_direction_t io_dir)
{
    if(CODEC_OUTPUT == io_dir)
    {
        #if IF_JUST_CLOSE_HPOUT_WHILE_NO_PLAY
        inner_codec_hpout_is_mute(true);
        #else
        inner_codec_dac_disable();
        #endif
    }
    return 0;
}


/**
 * @brief 配置CODEC ADC或者DAC的采样率、数据位宽以及通道选择
 * 
 * @param audio_info 
 * @param io_dir 输入或输出（ADC或者DAC）
 * @return int 
 */
int icodec_config(cm_sound_info_t * audio_info, io_direction_t io_dir)
{
    if(CODEC_INPUT == io_dir)
    {
        //左右声道
        inner_cedoc_gate_t left_gate[] = {1,0,1,0};
        inner_cedoc_gate_t right_gate[] = {1,1,0,0};
        inner_codec_adc_disable(INNER_CODEC_LEFT_CHA, (inner_cedoc_gate_t)left_gate[audio_info->channel_flag]);   //ADC左通道关闭
        inner_codec_adc_disable(INNER_CODEC_RIGHT_CHA, (inner_cedoc_gate_t)right_gate[audio_info->channel_flag]);   //ADC右通道关闭
        
        //采样率
        const uint32_t audio_cap_sr[8][2] = {
            8000,INNER_CODEC_SAMPLERATE_8K,
            12000,INNER_CODEC_SAMPLERATE_12K,
            16000,INNER_CODEC_SAMPLERATE_16K,
            24000,INNER_CODEC_SAMPLERATE_24K,
            32000,INNER_CODEC_SAMPLERATE_32K,
            44100,INNER_CODEC_SAMPLERATE_44_1K,
            48000,INNER_CODEC_SAMPLERATE_48K,
            96000,INNER_CODEC_SAMPLERATE_96K,
        };
        uint32_t sr = audio_info->sample_rate;
        for(int i=0;i<8;i++)
        {
            if(audio_cap_sr[i][0] == sr)
            {
                CODEC->alc_reg[0].reg44 = (1<<3) | (audio_cap_sr[i][1] << 0);
                CODEC->alc_reg[1].reg44 = (1<<3) | (audio_cap_sr[i][1] << 0);
                break;
            }
        }
        
        //数据位宽
        inner_codec_valid_word_len_t valid_bit[] = {INNER_CODEC_VALID_LEN_16BIT,
                                                    INNER_CODEC_VALID_LEN_24BIT,
                                                    INNER_CODEC_VALID_LEN_32BIT,
                                                    INNER_CODEC_VALID_LEN_20BIT};
        CODEC->reg2 &= ~(0x03<<5);
        CODEC->reg2 |= (valid_bit[audio_info->sample_depth] << 5);
    }
    else
    {
        //DAC只有单声道，没有采样率选择
        //有效数据位宽配置
        inner_codec_valid_word_len_t valid_bit[] = {INNER_CODEC_VALID_LEN_16BIT,
                                                    INNER_CODEC_VALID_LEN_24BIT,
                                                    INNER_CODEC_VALID_LEN_32BIT,
                                                    INNER_CODEC_VALID_LEN_20BIT};
        CODEC->reg4 &= ~(3 << 5);
        CODEC->reg4 |= (valid_bit[audio_info->sample_depth] << 5);
    }
    return 0;
}


int icodec_ioctl(io_direction_t io_dir, uint32_t param0, uint32_t param1, uint32_t param2)
{
    cm_io_ctrl_cmd_t cmd = (cm_io_ctrl_cmd_t)param0;
    switch(cmd)
    {
    case CM_IOCTRL_SET_DAC_GAIN:    //输出音量控制
        icodec_dac_gain_set((inner_codec_cha_sel_t)(param1-1), param2);
        break;
        //这个增益暂时只设置PGA的增益
    case CM_IOCTRL_SET_ADC_GAIN:    //输入音量控制
        inner_codec_set_pga_gain((inner_codec_cha_sel_t)(param1-1), param2);
        break;
        //打开ALC
    case CM_IOCTRL_ALC_ENABLE:
        inner_codec_alc_enable((inner_codec_cha_sel_t)(param1-1),INNER_CODEC_GATE_ENABLE,INNER_CODEC_USE_ALC_CONTROL_PGA_GAIN);
        break;
        //关闭ALC，选择通道进行关闭
    case CM_IOCTRL_ALC_DISABLE:
        inner_codec_alc_disable((inner_codec_cha_sel_t)(param1-1));
        break;
        //关闭没用的DAC，ADC通道，降低功耗
    case CM_IOCTRL_DAC_ENABLE:
        
        break;
    case CM_IOCTRL_MUTE:
        
        break;
    default:
        break;
    }

    return 0;
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
