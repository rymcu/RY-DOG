/**
 * @file ci130x_pdm.c
 * @brief CI130X inner pdm的驱动函数
 * @version 0.1
 * @date 2019-03-29
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include "ci130x_pdm.h"
#include "ci130x_alc.h"
#include "ci130x_scu.h"
#include "ci130x_system.h"
#include "ci_log.h"
#include <stdlib.h>
#include <string.h>
#include "ci130x_core_eclic.h"
#include "romlib_runtime.h"


#define PDMC_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("PDMC Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

#define USE_CI130X_OWN_ALC 1//目前没有用到CI110x自带的ALC，为了省代码，用这个宏可以不编译alc.c

/**
 * @brief 当主频为168时，延时100us
 *
 * @param cnt 延迟0.1毫秒
 */
// static void _delay_10us_240M(unsigned int cnt)
// {
//     volatile unsigned int i,j;
//     for(i=0;i<cnt;i++)
//     {
//         for(j=0;j<4200;j++)
//         {
//         }
//     }
// }


/**
 * @brief INNER PDM reset
 * 
 */
void pdm_reset(void)
{
    PDM->reg0 = 0;
    _delay_10us_240M(500);
    PDM->reg0 = 0x3;
    _delay_10us_240M(500);
}

/**
 * @brief PDM作为ADC的模式选择
 *
 * @param mode Master模式或者Slave模式；
 * @param frame_Len 数据宽度；
 * @param word_len 有效数据长度；
 * @param data_fram 数据格式
 */
void pdm_adc_mode_set(pdm_mode_t mode,pdm_frame_1_2len_t frame_Len,
                    pdm_valid_word_len_t word_len,pdm_i2s_data_famat_t data_fram)
{
    PDM->reg3 &= ~(0x03<<4);
    PDM->reg3 |= (mode<<4);

    PDM->reg3 &= ~(0x03<<2);
    PDM->reg3 |= (frame_Len<<2);

    PDM->reg2 &= ~(0x03<<5);
    PDM->reg2 |= (word_len<<5);

    PDM->reg2 &= ~(0x03<<3);
    PDM->reg2 |= (data_fram<<3);
}


/**
 * @brief PDM作为DAC的模式选择
 *
 * @param mode Master模式或者Slave模式；
 * @param frame_Len 数据宽度；
 * @param word_len 有效数据长度；
 * @param data_fram 数据格式
 */
void pdm_dac_mode_set(pdm_mode_t mode,pdm_frame_1_2len_t frame_Len,
                    pdm_valid_word_len_t word_len,pdm_i2s_data_famat_t data_fram)
{
    PDM->reg3 &= ~(0x03<<6);
    PDM->reg3 |= (mode<<6);

    PDM->reg5 &= ~(0x03<<2);
    PDM->reg5 |= (frame_Len<<2);

    PDM->reg4 &= ~(0x03<<5);
    PDM->reg4 |= (word_len<<5);

    PDM->reg4 &= ~(0x03<<3);
    PDM->reg4 |= (data_fram<<3);
}


/**
 * @brief PDM模块上电
 *
 * @param current pre-charge/dis-charge的电流大小
 */
void pdm_power_up(pdm_current_t current)
{
    PDM->reg2a &= ~((0x03<<0)|(0x03<<4));
    PDM->reg2a |= ((0x01<<0)|(0x01<<4));
    PDM->reg22 = 0x01;
    PDM->reg23 |= (0x01<<5);
    PDM->reg22 = 0xff;
    _delay_10us_240M(500);
    PDM->reg22 = current;
}


/**
 * @brief 关闭PDM模块电源
 *
 */
void pdm_power_off(void)
{
    PDM->reg22 = 0x01;
    PDM->reg23 &= ~(0x01<<5);
    PDM->reg22 = 0xff;
    _delay_10us_240M(500);
}


/**
 * @brief 配置PDM模块的高通滤波器
 *
 * @param gate 高通滤波器开关
 * @param Hz 高通滤波器的截止频率
 */
void pdm_hightpass_config(pdm_gate_t gate,pdm_highpass_cut_off_t Hz)
{
    PDM->rega  |= (0x01<<6);

    PDM->rega &= ~(0x01<<2);
    PDM->rega |= ((!gate)<<2);
    PDM->rega &= ~(0x03<<0);
    if(PDM_GATE_ENABLE == gate)
    {
        PDM->rega |= Hz<<0;
    }
    _delay_10us_240M(500);
}


/**
 * @brief 设置左通道的输入模式
 * 
 * @param mode 
 */
void pdm_set_input_mode_left(pdm_input_mode_t mode)
{
    PDM->reg26 &= ~(0x03<<6);
    PDM->reg26 |= (mode<<6);
}


/**
 * @brief 设置右通道的输入模式
 * 
 * @param mode 
 */
void pdm_set_input_mode_right(pdm_input_mode_t mode)
{
    PDM->reg26 &= ~(0x03<<2);
    PDM->reg26 |= (mode<<2);
}


/**
 * @brief 设置左通道的MIC增益
 * 
 * @param gain 
 */
void pdm_set_mic_gain_left(pdm_mic_amplify_t gain)
{
    PDM->reg25 &= ~(3 << 6);
    PDM->reg25 |= (gain << 6);
    _delay_10us_240M(500);
}


/**
 * @brief 设置右通道的MIC增益
 * 
 * @param gain 
 */
void pdm_set_mic_gain_right(pdm_mic_amplify_t gain)
{
    PDM->reg25 &= ~(3 << 4);
    PDM->reg25 |= (gain << 4);
    _delay_10us_240M(500);
}


/**
 * @brief 使能PDM中的ADC:
 *        1.使用此函数之前先初始化PDM_ADC_Config_TypeDef结构体并填写参数；
 *        2.ALCL_Gain和ALCR_Gain的上下限分别是28.5dB和-18dB，步长1.5。
 *
 * @param ADC_Config PDM_ADC_Config_TypeDef结构体指针
 */
void pdm_adc_enable(pdm_adc_config_t *ADC_Config)
{
    float Reg_ALC_Gain = 0;

    PDM->reg26 &= ~((0x03<<2)|(0x03<<6));
    PDM->reg26 |= ((ADC_Config->codec_adc_input_mode_r<<2)|(ADC_Config->codec_adc_input_mode_l<<6));

    PDM->reg24 |= ((0x01<<3)|(0x01<<7));

    PDM->reg23 |= 0x07<<0;
    PDM->reg23 |= 0x01<<4;
    PDM->reg23 |= 0x01<<5;
    PDM->reg23 |= 0x01<<3;

    PDM->reg24 |= ((0x01<<1)|(0x01<<5));
    PDM->reg26 |= ((0x01<<0)|(0x01<<4));
    _delay_10us_240M(500);
    PDM->reg26 |= ((0x01<<1)|(0x01<<5));

    PDM->reg29 |= ((0x01<<2)|(0x01<<6));
    PDM->reg29 |= ((0x01<<1)|(0x01<<5));
    PDM->reg29 |= ((0x01<<0)|(0x01<<4));
    PDM->reg29 |= ((0x01<<3)|(0x01<<7));
    PDM->reg24 |= ((0x01<<2)|(0x01<<6));

    PDM->reg25 &= ~((0x03<<4)|(0x03<<6));
    PDM->reg25 |= ((ADC_Config->codec_adc_mic_amp_r<<4)|(ADC_Config->codec_adc_mic_amp_l<<6));
    _delay_10us_240M(500); 
    PDM->reg25 &= ~(0x0f<<0);
    PDM->reg25 |= (5<<0);

    if(28.5 < ADC_Config->pga_gain_l)
    {
        ADC_Config->pga_gain_l = 28.5;
    }
    else if(-18 > ADC_Config->pga_gain_l)
    {
        ADC_Config->pga_gain_l = -18;
    }
    Reg_ALC_Gain = (ADC_Config->pga_gain_l+18)/1.5;
    PDM->reg27  = (int)Reg_ALC_Gain;

    if(28.5 < ADC_Config->pga_gain_r)
    {
        ADC_Config->pga_gain_r = 28.5;
    }
    else if(-18 > ADC_Config->pga_gain_r)
    {
        ADC_Config->pga_gain_r = -18;
    }
    Reg_ALC_Gain = (ADC_Config->pga_gain_r+18)/1.5;
    PDM->reg28  = (int)Reg_ALC_Gain;
    PDM->reg24 |= ((0x01<<0)|(0x01<<4));
}


/**
 * @brief 调节CODEC ADC的数字增益（197为0dB，加减1为加减0.5dB增益）
 *
 * @param gain 左声道增益大小
 */
void pdm_adc_dig_gain_set(pdm_cha_sel_t cha,int32_t gain)
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
        gain_data_reg = (int32_t)((float)gain * 0.5f) + 197;
    }
    PDM->pdm_dig_gain[cha] = (uint8_t)gain_data_reg;
}


/**
 * @brief 关闭PDM中的ADC
 *
 * @param cha 左右通道、全部通道的选择；
 * @param EN 选择是否关闭ADC
 */
void pdm_adc_disable(pdm_cha_sel_t cha, pdm_gate_t EN)
{
    if((PDM_LEFT_CHA == cha)&&(PDM_GATE_ENABLE == EN))    //dis左通道的ADC
    {
        PDM->reg24 &= ~(0x01<<4);
        PDM->reg29 &= ~(0x01<<5);
        PDM->reg29 &= ~(0x01<<6);
        PDM->reg26 &= ~(0x01<<5);
        PDM->reg26 &= ~(0x01<<4);
        PDM->reg24 &= ~(0x01<<5);
        PDM->reg29 &= ~(0x01<<4);
        PDM->reg29 &= ~(0x01<<7);
        PDM->reg24 &= ~(0x01<<6);
    }

    else if((PDM_RIGHT_CHA == cha)&&(PDM_GATE_ENABLE == EN))    //dis左通道的ADC
    {
        
        PDM->reg24 &= ~(0x01<<0);
        PDM->reg29 &= ~(0x01<<1);
        PDM->reg29 &= ~(0x01<<2);
        PDM->reg26 &= ~(0x01<<1);
        PDM->reg26 &= ~(0x01<<0);
        PDM->reg24 &= ~(0x01<<1);
        PDM->reg29 &= ~(0x01<<0);
        PDM->reg29 &= ~(0x01<<3);
        PDM->reg24 &= ~(0x01<<2);
    }
}


/**
 * @brief 配置PDM中的DAC增益,-39dB到7.5dB，步长1.5dB，真实为32档。
 *
 * @param l_gain LeftADC中 HPDRV的增益，0--100；
 * @param r_gain RightADC中 HPDRV的增益，0--100。
 */
void pdm_dac_gain_set(int32_t l_gain,int32_t r_gain)
{
    float step = 100.0f/31.0f;
    uint32_t reg_l_gain = 0;
    uint32_t reg_r_gain = 0;
    if(l_gain >= 100)
    {
        l_gain = 100;
    }
    if(r_gain >= 100)
    {
        r_gain = 100;
    }

    if(l_gain < 0)
    {
        l_gain = 0;
    }
    if(r_gain < 0)
    {
        r_gain = 0;
    }

    reg_l_gain = (uint32_t)( (float)l_gain/step );
    reg_r_gain = (uint32_t)( (float)r_gain/step );

    PDM->reg2f  = reg_l_gain;
    PDM->reg30  = reg_r_gain;
}


/**
 * @brief 使能PDM中的DAC(-39 -- 7 dB)
 *
 * @param l_gain LeftADC中 HPDRV的增益，单位dB；
 * @param r_gain RightADC中 HPDRV的增益，单位dB。
 */
void pdm_dac_enable(void)
{
    PDM->reg2e &= ~((0x01<<2)|(0x01<<6));
    PDM->reg2a |= (0x01<<7);
    PDM->reg2a |= ((0x01<<2)|(0x01<<6));
    PDM->reg2a &= ~((0x03<<0)|(0x03<<4));
    PDM->reg2a |= ((0x02<<0)|(0x02<<4));
    PDM->reg2e |= ((0x01<<0)|(0x01<<4));
    PDM->reg2e |= ((0x01<<1)|(0x01<<5));
    PDM->reg2b |= ((0x01<<3)|(0x01<<7));
    PDM->reg2b |= ((0x01<<2)|(0x01<<6));
    PDM->reg2b |= ((0x01<<1)|(0x01<<5));
    PDM->reg2b |= ((0x01<<0)|(0x01<<4));
    PDM->reg2c = 0x1f;
    PDM->reg2d = 0x1f;
    _delay_10us_240M(500);
    PDM->reg2e |= ((0x01<<2)|(0x01<<6));
}


void pdm_hpout_mute(void)
{
    uint32_t reg_data = PDM->reg2e;
    reg_data &= ~((1<<6)|(1<<2));
    PDM->reg2e = reg_data;
}


void pdm_hpout_mute_disable(void)
{
    uint32_t reg_data = PDM->reg2e;
    reg_data |= ((1<<6)|(1<<2));
    PDM->reg2e = reg_data;
}


/**
 * @brief 关闭DAC
 *
 * @param cha 声道选择
 * @param EN 开关
 */
void pdm_dac_disable(pdm_cha_sel_t cha, pdm_gate_t EN)
{
    if((PDM_LEFT_CHA == cha)&&(PDM_GATE_ENABLE == EN))    //dis左通道的ADC
    {
        PDM->reg2e &= ~(0x01<<6);
        PDM->reg2e &= ~(0x01<<5);
        PDM->reg2e &= ~(0x01<<4);
        PDM->reg2b &= ~(0x01<<5);
        PDM->reg2b &= ~(0x01<<6);
        PDM->reg2b &= ~(0x01<<7);
        PDM->reg2a &= ~(0x03<<4);
        PDM->reg2a |= (0x01<<4);
        PDM->reg2a &= ~(0x01<<6);
        PDM->reg2b &= ~(0x01<<4);
    }

    else if((PDM_RIGHT_CHA == cha)&&(PDM_GATE_ENABLE == EN))    //dis右通道的ADC
    {
        PDM->reg2e &= ~(0x01<<2);
        PDM->reg2e &= ~(0x01<<1);
        PDM->reg2e &= ~(0x01<<0);
        PDM->reg2b &= ~(0x01<<1);
        PDM->reg2b &= ~(0x01<<2);
        PDM->reg2b &= ~(0x01<<3);
        PDM->reg2a &= ~(0x03<<0);
        PDM->reg2a |= (0x01<<0);
        PDM->reg2a &= ~(0x01<<2);
        PDM->reg2b &= ~(0x01<<0);
    }
}



/**
 * @brief ALC判断的信号来自高通滤波器之前，还是之后
 *
 * @param judge 选择PDM的判断信号来自ALC之前还是之后
 */
void pdm_alc_judge_sel(pdm_alc_judge_t judge)
{
    PDM->rega  &= ~(0x01<<6);

    PDM->rega  |= ((1&judge)<<6);
}



/**
 * @brief 向PDM reg27 reg28寄存器写PGA的增益
 *
 * @param cha 左通道或者右通道
 * @param gain PGA增益的大小
 */
void pdm_pga_gain_config_via_reg27_28(pdm_cha_sel_t cha,uint32_t gain)
{
    if(gain > 0x1f)
    {
        PDMC_CI_ASSERT(0,"gain err!\n");
    }
    if(PDM_RIGHT_CHA == cha)
    {
        PDM->reg28 = gain;
    }
    else if(PDM_LEFT_CHA == cha)
    {
        PDM->reg27 = gain;
    }
    else
    {
        PDM->reg27 = gain;
        PDM->reg28 = gain;
    }
}


void pdm_pga_gain_config_via_reg27_28_db(pdm_cha_sel_t cha,float gain_db)
{
    float gain = gain_db;
    if(28.5f < gain)
    {
        gain = 28.5f;
    }
    else if(-18.0f > gain)
    {
        gain = -18.0f;
    }
    uint32_t reg_gain = (uint32_t)((gain + 18.0f)/1.5f);
    pdm_pga_gain_config_via_reg27_28(cha,reg_gain);
}



/**
 * @brief 获取PDM MIC增益的大小
 *
 * @param cha 通道选择
 */
pdm_mic_amplify_t pdm_get_mic_gain(pdm_cha_sel_t cha)
{
    volatile uint32_t tmp = 0;
    pdm_mic_amplify_t mic_amp;
    tmp = PDM->reg25;
    if(PDM_LEFT_CHA == cha)
    {
        tmp = PDM->reg25 & (0x3 << 6);
        mic_amp = (pdm_mic_amplify_t)(tmp >> 6);
    }
    else if(PDM_RIGHT_CHA == cha)
    {
        tmp = PDM->reg25 & (0x3 << 4);
        mic_amp = (pdm_mic_amplify_t)((tmp >> 4)&0x3);
    }
    else
    {
        PDMC_CI_ASSERT(0,"cha err!\n");
    }
    return mic_amp;
}


/**
 * @brief 向PDM MIC增益控制寄存器写值
 *
 * @param cha 左通道或者右通道
 * @param gain MIC增益的大小
 */
void pdm_set_mic_gain(pdm_cha_sel_t cha,pdm_mic_amplify_t gain)
{
    if(PDM_LEFT_CHA == cha)
    {
        PDM->reg25 &= ~(3 << 6);
        PDM->reg25 |= (gain << 6);
    }
    else if(PDM_RIGHT_CHA == cha)
    {
        PDM->reg25 &= ~(3 << 4);
        PDM->reg25 |= (gain << 4);
    }
    else
    {
        PDM->reg25 &= ~(3 << 4);
        PDM->reg25 |= (gain << 4);
        PDM->reg25 &= ~(3 << 6);
        PDM->reg25 |= (gain << 6);
    }
    _delay_10us_240M(500);
}



/**
 * @brief PDM ALC配置左通道
 * 
 * @param ALC_str pdm_alc_use_config_t类型结构体指针
 * @param ALC_cha 需要开启的ALC通道配置
 */
void pdm_alc_left_config(pdm_alc_use_config_t* ALC_str)
{
    pdm_alc_config_t ALC_Config_str_l;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_Config_str_l,0,sizeof(pdm_alc_config_t));
    ALC_Config_str_l.attacktime = ALC_str->attack_time;
    ALC_Config_str_l.decaytime = ALC_str->decay_time;
    ALC_Config_str_l.holdtime = ALC_str->hold_time;
    ALC_Config_str_l.max_pga_gain = ALC_str->alc_maxgain;
    ALC_Config_str_l.min_pga_gain = ALC_str->alc_mingain;
    ALC_Config_str_l.pga_gain = 0;
    ALC_Config_str_l.max_level = ALC_str->max_level;
    ALC_Config_str_l.min_level = ALC_str->min_level;
    ALC_Config_str_l.alcmode = PDM_ALC_MODE_JACKWAY2;
    ALC_Config_str_l.samplerate = ALC_str->sample_rate;

    pdm_left_alc_pro_mode_config(&ALC_Config_str_l);
    
    pdm_noise_gate_set(PDM_LEFT_CHA,ALC_str->noise_gate_threshold,ALC_str->noise_gate);

    pdm_87_5_fast_decrement_set(PDM_LEFT_CHA,ALC_str->fast_decrece_87_5);

    pdm_zero_cross_set(PDM_LEFT_CHA,ALC_str->zero_cross);
    
    pdm_use_alc_control_pgagain_t pga_control = PDM_USE_ALC_CONTROL_PGA_GAIN;
    
    if(PDM_GATE_DISABLE == ALC_str->alc_gate)
    {//ALC关闭，一定不使用ALC控制PGA增益
        pga_control = PDM_WONT_USE_ALC_CONTROL_PGA_GAIN;
    }
    else if((PDM_GATE_ENABLE == ALC_str->alc_gate) && (PDM_GATE_ENABLE == ALC_str->use_ci130x_alc))
    {//PDM 打开，使用PDM自己的ALC，使用ALC控制
        pga_control = PDM_WONT_USE_ALC_CONTROL_PGA_GAIN;
    }
    else
    {
        pga_control = PDM_USE_ALC_CONTROL_PGA_GAIN;
    }

    pdm_left_alc_enable(ALC_str->alc_gate,pga_control);
    
    pdm_alc_judge_sel(PDM_ALC_JUDGE_AFTER_HIGHPASS);
    
    if(0 != ALC_str->digt_gain)
    {
        pdm_adc_dig_gain_set_left(ALC_str->digt_gain);
    }

    if(PDM_GATE_ENABLE == ALC_str->use_ci130x_alc)
    {
        #if USE_CI130X_OWN_ALC
        scu_set_device_gate(HAL_ALC_BASE,ENABLE);
        eclic_irq_enable(ALC_IRQn);

        alc_aux_config_t ALC_AUX_L_Config_Str;
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_AUX_L_Config_Str,0,sizeof(alc_aux_config_t));
        ALC_AUX_L_Config_Str.step_grd = ALC_STEP_GRD_0_5dB;
        ALC_AUX_L_Config_Str.step_cnt = ALC_STEP_CNT_3;
        ALC_AUX_L_Config_Str.db_offset = ALC_str->digt_gain - 36;
        ALC_AUX_L_Config_Str.step_tmr_atk = 6833*(1<<ALC_str->attack_time);//164*10000;  //162*1000对应1ms,20500对应125us,6833=20500/3
        ALC_AUX_L_Config_Str.step_tmr_dcy = 27333*(1<<ALC_str->decay_time);//164*10000;  //162*1000对应1ms
        ALC_AUX_L_Config_Str.cfg_addr = 0;//0x4003f020;
        ALC_AUX_L_Config_Str.min_db_alert = 0x0d;//
        ALC_AUX_L_Config_Str.max_db_alert = 0x1e;//只有当PDM输出的PGA的增益值大于（>）设定的值的时候，才会触发这两个中断
        ALC_AUX_L_Config_Str.atk_limit_num = 3;
        ALC_AUX_L_Config_Str.dcy_limit_num = 3;

        alc_aux_left_config(ALC_PDM,&ALC_AUX_L_Config_Str);

        /*配置ALC的中断*/
        alc_aux_int_t ALC_AUX_Int_str = {0};
        ALC_AUX_Int_str.crs_down_min_db_int_en = DISABLE;
        ALC_AUX_Int_str.crs_up_max_db_int_en = DISABLE;
        ALC_AUX_Int_str.gain_change_int_en = DISABLE;
        ALC_AUX_Int_str.gain_in_change_int_en = DISABLE;
        ALC_AUX_Int_str.atk_limit_int_en = DISABLE;
        ALC_AUX_Int_str.dcy_limit_int_en = DISABLE;

        alc_aux_intterupt_config(ALC_PDM,&ALC_AUX_Int_str,ALC_Left_Cha);

        /*配置ALC的全局寄存器*/
        alc_aux_globle_config_t ALC_AUX_Glb_Str;
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_AUX_Glb_Str,0,sizeof(alc_aux_globle_config_t));
        ALC_AUX_Glb_Str.left_gain_sel = ALC_L_GAIN_FROM_PDM_L;
        ALC_AUX_Glb_Str.right_gain_sel = ALC_R_GAIN_FROM_PDM_R;
        ALC_AUX_Glb_Str.detect_num = 40;//ALC(AHB)/MCLK*2为最佳，只在选择ALC_DETECT_MD_NUM模式，此值有效，最多6位2进制
        ALC_AUX_Glb_Str.detect_mode = ALC_DETECT_MD_GAIN_CHANGE;//ALC_DETECT_MD_GAIN_CHANGE;//ALC_DETECT_MD_NUM;

        alc_aux_globle_config(ALC_PDM,&ALC_AUX_Glb_Str);
        alc_aux_left_cha_en(ALC_PDM,ENABLE);
        alc_aux_globle_enable(ALC_PDM,ENABLE);  
        #else
        PDMC_CI_ASSERT(0,"please open the define\n");
        #endif
    }
}



/**
 * @brief PDM ALC配置左通道
 * 
 * @param ALC_str pdm_alc_use_config_t类型结构体指针
 * @param ALC_cha 需要开启的ALC通道配置
 */
void pdm_alc_right_config(pdm_alc_use_config_t* ALC_str)
{
    pdm_alc_config_t ALC_Config_str_r;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_Config_str_r,0,sizeof(pdm_alc_config_t));
    ALC_Config_str_r.attacktime = ALC_str->attack_time;
    ALC_Config_str_r.decaytime = ALC_str->decay_time;
    ALC_Config_str_r.holdtime = ALC_str->hold_time;
    ALC_Config_str_r.max_pga_gain = ALC_str->alc_maxgain;
    ALC_Config_str_r.min_pga_gain = ALC_str->alc_mingain;
    ALC_Config_str_r.pga_gain = 0;
    ALC_Config_str_r.max_level = ALC_str->max_level;
    ALC_Config_str_r.min_level = ALC_str->min_level;
    ALC_Config_str_r.alcmode = PDM_ALC_MODE_JACKWAY2;
    ALC_Config_str_r.samplerate = ALC_str->sample_rate;

    pdm_right_alc_pro_mode_config(&ALC_Config_str_r);
    
    pdm_noise_gate_set(PDM_RIGHT_CHA,ALC_str->noise_gate_threshold,ALC_str->noise_gate);

    pdm_87_5_fast_decrement_set(PDM_RIGHT_CHA,ALC_str->fast_decrece_87_5);

    pdm_zero_cross_set(PDM_RIGHT_CHA,ALC_str->zero_cross);

    pdm_use_alc_control_pgagain_t pga_control = PDM_USE_ALC_CONTROL_PGA_GAIN;

    if(PDM_GATE_DISABLE == ALC_str->alc_gate)
    {//ALC关闭，一定不使用ALC控制PGA增益
        pga_control = PDM_WONT_USE_ALC_CONTROL_PGA_GAIN;
    }
    else if((PDM_GATE_ENABLE == ALC_str->alc_gate) && (PDM_GATE_ENABLE == ALC_str->use_ci130x_alc))
    {//PDM 打开，使用PDM自己的ALC，使用ALC控制
        pga_control = PDM_WONT_USE_ALC_CONTROL_PGA_GAIN;
    }
    else
    {
        pga_control = PDM_USE_ALC_CONTROL_PGA_GAIN;
    }

    pdm_right_alc_enable(ALC_str->alc_gate,pga_control);

    pdm_alc_judge_sel(PDM_ALC_JUDGE_AFTER_HIGHPASS);
    
    if(0 != ALC_str->digt_gain)
    {
        pdm_adc_dig_gain_set_right(ALC_str->digt_gain);
    }

    if(PDM_GATE_ENABLE == ALC_str->use_ci130x_alc)
    {
        #if USE_CI130X_OWN_ALC
        scu_set_device_gate(HAL_ALC_BASE,ENABLE);
        eclic_irq_enable(ALC_IRQn);

        alc_aux_config_t ALC_AUX_R_Config_Str;
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_AUX_R_Config_Str,0,sizeof(alc_aux_config_t));
        ALC_AUX_R_Config_Str.step_grd = ALC_STEP_GRD_0_5dB;
        ALC_AUX_R_Config_Str.step_cnt = ALC_STEP_CNT_3;
        ALC_AUX_R_Config_Str.db_offset = ALC_str->digt_gain - 36;       //0dB
        ALC_AUX_R_Config_Str.step_tmr_atk = 6833*(1<<ALC_str->attack_time);//164*10000;  //162*1000对应1ms,20500对应125us,6833=20500/3
        ALC_AUX_R_Config_Str.step_tmr_dcy = 27333*(1<<ALC_str->decay_time);//164*10000;  //162*1000对应1ms
        ALC_AUX_R_Config_Str.cfg_addr = 0;//0x4003f024;
        ALC_AUX_R_Config_Str.min_db_alert = 0x0d;//
        ALC_AUX_R_Config_Str.max_db_alert = 0x1e;//只有当PDM输出的PGA的增益值大于（>）设定的值的时候，才会触发这两个中断
        ALC_AUX_R_Config_Str.atk_limit_num = 3;
        ALC_AUX_R_Config_Str.dcy_limit_num = 3;

        alc_aux_right_config(ALC_PDM,&ALC_AUX_R_Config_Str);

        /*配置ALC的中断*/
        alc_aux_int_t ALC_AUX_Int_str = {0};
        ALC_AUX_Int_str.crs_down_min_db_int_en = DISABLE;
        ALC_AUX_Int_str.crs_up_max_db_int_en = DISABLE;
        ALC_AUX_Int_str.gain_change_int_en = DISABLE;
        ALC_AUX_Int_str.gain_in_change_int_en = DISABLE;
        ALC_AUX_Int_str.atk_limit_int_en = DISABLE;
        ALC_AUX_Int_str.dcy_limit_int_en = DISABLE;

        alc_aux_intterupt_config(ALC_PDM,&ALC_AUX_Int_str,ALC_Right_Cha);

        /*配置ALC的全局寄存器*/
        alc_aux_globle_config_t ALC_AUX_Glb_Str;
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&ALC_AUX_Glb_Str,0,sizeof(alc_aux_globle_config_t));
        ALC_AUX_Glb_Str.left_gain_sel = ALC_L_GAIN_FROM_PDM_L;
        ALC_AUX_Glb_Str.right_gain_sel = ALC_R_GAIN_FROM_PDM_R;
        ALC_AUX_Glb_Str.detect_num = 40;//ALC(AHB)/MCLK*2为最佳，只在选择ALC_DETECT_MD_NUM模式，此值有效，最多6位2进制
        ALC_AUX_Glb_Str.detect_mode = ALC_DETECT_MD_GAIN_CHANGE;//ALC_DETECT_MD_GAIN_CHANGE;//ALC_DETECT_MD_NUM;

        alc_aux_globle_config(ALC_PDM,&ALC_AUX_Glb_Str);
        alc_aux_right_cha_en(ALC_PDM,ENABLE);
        alc_aux_globle_enable(ALC_PDM,ENABLE);   
        #else
        PDMC_CI_ASSERT(0,"please open the define\n");
        #endif
    }
}



//ALC默认配置
#define METHOD_TO_CONTROL_GAIN_ATTACK   0
#define ALC_JACK_WAY    PDM_ALC_MODE_JACKWAY2
#define ALC_HOLD_TIME   PDM_ALC_HOLD_TIME_2MS
#define ALC_DECAY_TIME  PDM_ALC_DECAY_TIME_2MS
#define ALC_ATTACK_TIME PDM_ALC_ATTACK_TIME_2MS
#define ALC_MODE        0
#define ALC_ZERO_CROSS  PDM_GATE_DISABLE
#define ALC_FAST_DECAY_87_5 PDM_GATE_DISABLE
#define ALC_NOISE_GATE  PDM_GATE_DISABLE
#define ALC_NOISE_GATE_SET  PDM_NOISE_GATE_THRE_45dB
#define ALC_INIT_PGA_GAIN   0x1f
#define ALC_SAMPLE_RATE     PDM_SAMPLERATE_16K
#define ALC_LOW_8_BIT_MAX_LEVEL     ((uint8_t)(PDM_ALC_LEVEL__6dB >> 0))
#define ALC_HOGH_8_BIT_MAX_LEVEL    ((uint8_t)(PDM_ALC_LEVEL__6dB >> 8))
#define ALC_LOW_8_BIT_MIN_LEVEL     ((uint8_t)(PDM_ALC_LEVEL__7_5dB >> 0))
#define ALC_HOGH_8_BIT_MIN_LEVEL    ((uint8_t)(PDM_ALC_LEVEL__7_5dB >> 8))
#define ALC_AGC_FUNC_SEL    ENABLE
#define ALC_PGA_MAX_GAIN    PDM_ALC_PGA_MAX_GAIN_28_5dB
#define ALC_PGA_MIN_GAIN    PDM_ALC_PGA_MIN_GAIN__18dB

/**
 * @brief ALC的默认配置
 * 
 */
static void pdm_alc_init_config(void)
{
    const uint32_t alc_reg_init_config_data[10] = 
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
        uint32_t* reg_data_p = (uint32_t*)(uint32_t)(&(PDM->alc_reg[i].reg40));
        for(int j=0;j<10;j++)
        {
            reg_data_p[j] = alc_reg_init_config_data[j];
        }
    }
}

//经测试，PDM依然存在零飘的问题
#define ALC_JUDGE   (PDM_ALC_JUDGE_AFTER_HIGHPASS)
#define ALC_CONTROL_PGA_L (PDM_GATE_ENABLE)
#define ALC_CONTROL_PGA_R (PDM_GATE_ENABLE)
#define HPF_GATE    (!PDM_GATE_ENABLE)//0为打开，1为关闭
#define HPF_CONFIG  (PDM_HIGHPASS_CUT_OFF_20HZ)


static void pdm_adc_init_config(void)
{
    //reg0 -- rega
    const uint8_t pdm_init_reg_data_1[] = 
    {
        0x3,0x86,0x10,0xe,
        0x50,0xe,0x22,0x1,
        0xc5,0xc5
    };

    uint32_t* reg_data_p = (uint32_t*)(uint32_t)(&(PDM->reg0));
    for(int j=0;j<sizeof(pdm_init_reg_data_1);j++)
    {
        reg_data_p[j] = (uint32_t)pdm_init_reg_data_1[j];
    }

    PDM->rega = (0 << 7) | (ALC_JUDGE << 6) | (ALC_CONTROL_PGA_L << 5) 
            | (ALC_CONTROL_PGA_R << 4) | (1 << 3) | (HPF_GATE << 2) | (HPF_CONFIG << 0);

    //reg21 -- reg30
    const uint8_t pdm_init_reg_data_2[] = 
    {
        0x0,0x1,0x3f,
        0xff,0x5,0x33,0x1f,
        0x1f,0xff,0x11,
    };

    reg_data_p = (uint32_t*)(uint32_t)(&(PDM->reg21));
    for(int j=0;j<sizeof(pdm_init_reg_data_2);j++)
    {
        reg_data_p[j] = (uint32_t)pdm_init_reg_data_2[j];
    }
}


static void pdm_aux_alc_init_config(void)
{
    //reg0 -- rega
    const uint32_t pdm_aux_alc_init_reg_data_1[] = 
    {
        0x0001a007,
        0x00008820,0x0001ab10,0x0001ab14,0x00000000,
        0x03031e0d,
    };

    uint32_t* reg_data_p = (uint32_t*)(uint32_t)(&(ALC_PDM->GLB_CTRL));
    for(int j=0;j<(sizeof(pdm_aux_alc_init_reg_data_1)/4);j++)
    {
        reg_data_p[j] = (uint32_t)pdm_aux_alc_init_reg_data_1[j];
    }

    //reg21 -- reg30
    const uint32_t pdm_aux_alc_init_reg_data_2[] = 
    {
        0x00008820,0x0001ab10,0x0001ab14,0x00000000,
	    0x03031e0d,
    };

    reg_data_p = (uint32_t*)(uint32_t)(&(ALC_PDM->RIGHT_CTRL));
    for(int j=0;j<(sizeof(pdm_aux_alc_init_reg_data_2)/4);j++)
    {
        reg_data_p[j] = (uint32_t)pdm_aux_alc_init_reg_data_2[j];
    }
}



int pdm_init(void)
{
    extern void pad_config_for_pdm(void);
    // extern void pad_config_for_iis(void);
    pad_config_for_pdm();
    // pad_config_for_iis();

    scu_set_device_gate((uint32_t)PDM,ENABLE);
    scu_set_device_gate(HAL_ALC_BASE,ENABLE);
    scu_set_device_reset((uint32_t)PDM);
    scu_set_device_reset_release((uint32_t)PDM);
    scu_set_device_reset(HAL_ALC_BASE);
    scu_set_device_reset_release(HAL_ALC_BASE);

    pdm_adc_init_config();
    //ALC设置
    pdm_alc_init_config();
    pdm_adc_dig_gain_set(PDM_LEFT_CHA,0);
    pdm_adc_dig_gain_set(PDM_RIGHT_CHA,0);
    *(volatile uint32_t*)(HAL_PDM_BASE + 0x260) |= (1<<0) | (1<<1);

    pdm_aux_alc_init_config();
}


void pdm_left_mic_reinit(void)
{
    uint32_t reg_data = PDM->reg24;
    reg_data &= ~(1<<6);
    PDM->reg24 = reg_data;
    _delay_10us_240M(300);
    reg_data |= (1<<6);
    PDM->reg24 = reg_data;
}


void pdm_right_mic_reinit(void)
{
    uint32_t reg_data = PDM->reg24;
    reg_data &= ~(1<<2);
    PDM->reg24 = reg_data;
    _delay_10us_240M(300);
    reg_data |= (1<<2);
    PDM->reg24 = reg_data;
}
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
