#include "ci130x_scu.h"
#include "ci130x_gpio.h"
#include "ci130x_dpmu.h"
#include "ci130x_iis.h"
#include "audio_play_process.h"
#include "sdk_default_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ci130x_codec.h"
#include "codec_manager.h"

int g_pa_pin_valid_level;        //记录功放使能控制引脚的有效电平   1: 高电平，0: 低电平





/**
 * @brief 引脚复用配置为UART功能
 *
 * @param UARTx UART组 : UART0, UART1
 */
void pad_config_for_uart(UART_TypeDef *UARTx)
{
    /*配置UART引脚*/
    // if (UARTx == UART0)
    // {
    //     dpmu_set_io_reuse(PB5,SECOND_FUNCTION);
	//     dpmu_set_io_reuse(PB6,SECOND_FUNCTION);
	//     dpmu_set_io_pull(PB6,DPMU_IO_PULL_UP);  //RX需开启上拉
    // }
    // else if (UARTx == UART1)
    // {
    //     dpmu_set_io_reuse(PB7,SECOND_FUNCTION);
    //     dpmu_set_io_reuse(PC0,SECOND_FUNCTION);
	//     dpmu_set_io_pull(PC0,DPMU_IO_PULL_UP);  //RX需开启上拉
    // }
    // else if (UARTx == UART2)
    // {
    //     dpmu_set_io_reuse(PB1,THIRD_FUNCTION);
    //     dpmu_set_io_reuse(PB2,THIRD_FUNCTION);
    //     dpmu_set_io_pull(PB2,DPMU_IO_PULL_UP);  //RX需开启上拉
    // }
}


/**
 * @brief 引脚复用配置为IIS功能
 *
 */
void pad_config_for_iis(void)
{
    #if USE_IIS1_OUT_PRE_RSLT_AUDIO
    /*配置IIS引脚*/
    // IOResue_FUNCTION IISx_IO_reuse = SECOND_FUNCTION;
	// dpmu_set_io_reuse(PA2,IISx_IO_reuse);
    // dpmu_set_io_reuse(PA3,IISx_IO_reuse);
    // dpmu_set_io_reuse(PA4,IISx_IO_reuse);
    // dpmu_set_io_reuse(PA5,IISx_IO_reuse);
    // dpmu_set_io_reuse(PA6,IISx_IO_reuse);
    #endif
}


/**
 * @brief 引脚复用配置为IIS功能
 *
 */
void pad_config_for_pdm(void)
{
    /*配置PDM引脚*/
    // IOResue_FUNCTION IISx_IO_reuse = FIFTH_FUNCTION;
	// dpmu_set_io_reuse(PB7,IISx_IO_reuse);
    // dpmu_set_io_reuse(PC0,IISx_IO_reuse);
}


/**
 * @brief 开启功放使能
 *
 */
void power_amplifier_on(void)
{
    /*配置功放PA使能*/
    // if (g_pa_pin_valid_level)
    // {
    //     gpio_set_output_high_level(PD,pin_2);
    // }
    // else
    // {
    //     gpio_set_output_low_level(PD,pin_2);
    // }
}

/**
 * @brief 关闭功放使能
 *
 */
void power_amplifier_off(void)
{
    /*配置功放PA关闭*/
    // if (g_pa_pin_valid_level)
    // {
    //     gpio_set_output_low_level(PD,pin_2);
    // }
    // else
    // {
    //     gpio_set_output_high_level(PD,pin_2);
    // }
}

/**
 * @brief 引脚复用配置为GPIO,用于控制功放使能
 *
 */
void pad_config_for_power_amplifier(void)
{
    /*配置功放PA引脚功能*/
    // scu_set_device_gate(PD,ENABLE);

    // dpmu_set_io_direction(PD2,DPMU_IO_DIRECTION_INPUT);
    // dpmu_set_io_pull(PD2,DPMU_IO_PULL_DISABLE);
	// dpmu_set_io_reuse(PD2,FIRST_FUNCTION);
    // gpio_set_input_mode(PD,pin_2);
    // uint8_t pa_default_level = gpio_get_input_level(PD,pin_2);
    // dpmu_set_io_direction(PD2,DPMU_IO_DIRECTION_OUTPUT);
    // gpio_set_output_mode(PD,pin_2);

    // if(pa_default_level == 0)
    // {
    //     //PA脚如果是下拉的，PA有效电平就是高电平
    //     g_pa_pin_valid_level = 1;
    // }
    // else
    // {
    //     g_pa_pin_valid_level = 0;
    // }

    // #if (PLAYER_CONTROL_PA) 
    // power_amplifier_off();
    // #else
    // power_amplifier_on();
    // #endif
}

#if USE_IIC_PAD
/**
 * @brief 引脚复用配置为IIC功能
 *
 */
void pad_config_for_i2c(void)
{
    /*配置IIC引脚*/
    // dpmu_set_io_reuse(PB7,THIRD_FUNCTION);
    // dpmu_set_io_reuse(PC0,THIRD_FUNCTION);
}
#endif //USE_IIC_PAD

#if USE_COLOR_LIGHT
/**
  * @功能:引脚复用配置为color light功能
  * @
  */
void pad_config_for_color_light(void)
{
    /*配置color三色灯引脚功能*/
    // pwm_init_t pwm_config;

    // scu_set_device_gate(HAL_PA_BASE,ENABLE);
    // scu_set_device_gate(HAL_PB_BASE,ENABLE);
    // scu_set_device_gate(HAL_PC_BASE,ENABLE);

    // /*R*/
    // dpmu_set_io_reuse(PA2,FIFTH_FUNCTION);
    // scu_set_device_gate(HAL_PWM0_BASE,ENABLE);
    
	// pwm_config.clk_sel = 0;
    // pwm_config.freq = 10000;
    // pwm_config.duty = 500;
    // pwm_config.duty_max = 1000;
    // pwm_init(PWM0,pwm_config);
    // pwm_set_restart_md(PWM0, 1); /*重新计数生效模式：1，等待正在输出的PWM波完成以后才生效*/
    // pwm_start(PWM0);
    // dpmu_set_io_reuse(PA2,FIFTH_FUNCTION);/*pwm pad enable*/
    // color_light_r_pwm_name = PWM0;

    // /*G*/
    // dpmu_set_io_reuse(PB0,SECOND_FUNCTION);
    // scu_set_device_gate(HAL_PWM1_BASE,ENABLE);
    
	// pwm_config.clk_sel = 0;
    // pwm_config.freq = 10000;
    // pwm_config.duty = 500;
    // pwm_config.duty_max = 1000;
    // pwm_init(PWM1,pwm_config);
    // pwm_set_restart_md(PWM1, 1); /*重新计数生效模式：1，等待正在输出的PWM波完成以后才生效*/
    // pwm_start(PWM1);
    // dpmu_set_io_reuse(PB0,SECOND_FUNCTION);/*pwm pad enable*/
    // color_light_r_pwm_name = PWM1;

    // /*B*/
    // dpmu_set_io_reuse(PC2,FORTH_FUNCTION);
    // scu_set_device_gate(HAL_PWM2_BASE,ENABLE);
    
	// pwm_config.clk_sel = 0;
    // pwm_config.freq = 10000;
    // pwm_config.duty = 500;
    // pwm_config.duty_max = 1000;
    // pwm_init(PWM2,pwm_config);
    // pwm_set_restart_md(PWM2, 1); /*重新计数生效模式：1，等待正在输出的PWM波完成以后才生效*/
    // pwm_start(PWM2);
    // dpmu_set_io_reuse(PC2,FORTH_FUNCTION);/*pwm pad enable*/
    // color_light_r_pwm_name = PWM2;
}
#endif //USE_COLOR_LIGHT

#if USE_NIGHT_LIGHT
/**
  * @功能:night light功能关闭
  * @
  */
void night_light_disable(void)
{
    /*配置night灯关闭*/
    // dpmu_set_io_reuse(PB3,FIRST_FUNCTION); /*gpio function*/
    // dpmu_set_io_direction(PB3,DPMU_IO_DIRECTION_OUTPUT);
    // gpio_set_output_mode(PB, pin_3);
    // gpio_set_output_level_single(PB, pin_3, 0);
}

/**
  * @功能:night light功能打开
  * @
  */
void night_light_enable(void)
{
    /*配置night灯开启*/
    // dpmu_set_io_reuse(PB3,SECOND_FUNCTION);/*pwm pad enable*/
}

/**
  * @功能:引脚复用配置为night light功能
  * @
  */
void pad_config_for_night_light(void)
{
    /*配置night灯引脚功能*/
    // pwm_init_t pwm_config;

    // scu_set_device_gate(HAL_PB_BASE,ENABLE);

    // dpmu_set_io_reuse(PB3,SECOND_FUNCTION);
    // scu_set_device_gate(HAL_PWM4_BASE,ENABLE);

    // pwm_config.clk_sel = 0;
    // pwm_config.freq = 25000;
    // pwm_config.duty = 0;
    // pwm_config.duty_max = 1000;
    // pwm_init(PWM4,pwm_config);
    // pwm_set_restart_md(PWM4, 0); /*重新计数生效模式：0，重新计数立即生效*/
    // pwm_start(PWM4);
    // night_light_w_pwm_name = PWM4;
    // night_light_enable();
}
#endif //USE_NIGHT_LIGHT

#if USE_BLINK_LIGHT
/**
  * @功能:blink light电平设置
  * @
  */
void blink_light_on_off(uint8_t on_off)
{
    /*配置blink灯电平*/
    // gpio_set_output_level_single(PB, pin_4,on_off);
}

/**
  * @功能:引脚复用配置为blink light功能
  * @
  */
void pad_config_for_blink_light(void)
{
    /*配置blin灯引脚功能*/
    // scu_set_device_gate(PB,ENABLE);
    // dpmu_set_io_direction(PB4,DPMU_IO_DIRECTION_OUTPUT);
    // dpmu_set_io_reuse(PB4,FIRST_FUNCTION);/*gpio function*/
    // gpio_set_output_mode(PB, pin_4);
    // gpio_set_output_level_single(PB, pin_4,0);
}
#endif //USE_BLINK_LIGHT

#if USE_VAD_LIGHT
/**
  * @功能:vad light功能关闭
  * @
  */
void vad_light_on(void)
{
    /*配置vad灯开启*/
    // gpio_set_output_level_single(PB, pin_7,1);
}

/**
  * @功能:vad light功能打开
  * @
  */
void vad_light_off(void)
{ 
    /*配置vad灯关闭*/
    // gpio_set_output_level_single(PB, pin_7,0);
}

/**
  * @功能:引脚复用配置为vad light功能
  * @
  */
void pad_config_for_vad_light(void)
{
    /*配置vad灯引脚功能*/
    // scu_set_device_gate(PB,ENABLE);
    // dpmu_set_io_direction(PB7,DPMU_IO_DIRECTION_OUTPUT);
    // dpmu_set_io_reuse(PB7,FIRST_FUNCTION);/*gpio function*/
    // gpio_set_output_mode(PB, pin_7);
    // gpio_set_output_level_single(PB, pin_7,0);
}
#endif //USE_VAD_LIGHT

/**
 * @brief 1、选择晶振作为时钟源还是RC作为时钟源
 * 
 */
void board_clk_source_set(void)
{   
    dpmu_unlock_cfg_config();
    #if USE_EXTERNAL_CRYSTAL_OSC
    dpmu_set_src_source(DPMU_SRC_USE_OUTSIDE_OSC);
    #else
    dpmu_set_src_source(DPMU_SRC_USE_INNER_RC);
    #endif
    dpmu_lock_cfg_config();
}


/******************************默认采音播音的CODEC配置*******************************/
/**
 * @brief 配置参数表
 *
 */
const cm_codec_hw_info_t host_mic_hw_info = 
{
    #if AUDIO_PLAYER_ENABLE
    .IICx = IIC_NULL,
    .output_iis.IISx = IIS1,
    .output_iis.iis_mode_sel = IIS_MASTER,//IIS_SLAVE,//IIS_MASTER,
    .output_iis.over_sample = IIS_MCLK_FS_256,
    #if !INNER_CODEC_AUDIO_IN_USE_RESAMPLE
    .output_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_OSC_OR_INEER_RC,
    #else
    .output_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_IPCORE,
    #endif
    .output_iis.mclk_out_en = IIS_MCLK_OUT,
    .output_iis.iis_data_format = IIS_DF_IIS,
    .output_iis.sck_lrck_ratio = IIS_SCK_LRCK_64,
    .output_iis.tx_cha = IIS_TX_CHANNAL_TX0,
    .output_iis.scklrck_out_en = IIS_SCKLRCK_OUT,
    #endif

    .input_iis.IISx = IIS1,
    .input_iis.iis_mode_sel = IIS_MASTER,
    .input_iis.over_sample = IIS_MCLK_FS_256,
    #if !INNER_CODEC_AUDIO_IN_USE_RESAMPLE
    .input_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_OSC_OR_INEER_RC,
    #else
    .input_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_IPCORE,
    #endif
    .input_iis.mclk_out_en = IIS_MCLK_OUT,
    .input_iis.iis_data_format = IIS_DF_IIS,
    .input_iis.sck_lrck_ratio = IIS_SCK_LRCK_64,
    .input_iis.rx_cha = IIS_RX_CHANNAL_RX0,
    .input_iis.outside_mclk_fre = 0,
    .input_iis.scklrck_out_en = IIS_SCKLRCK_MODENULL,//IIS_SCKLRCK_OUT,

#if (MIC_DIFF_SINGLE == 0)
    .codec_gain.codec_adc_input_mode_l = INNER_CODEC_INPUT_MODE_DIFF,
#else 
    .codec_gain.codec_adc_input_mode_l = INNER_CODEC_INPUT_MODE_SINGGLE_ENDED,
#endif 
    .codec_gain.codec_adc_input_mode_r = INNER_CODEC_INPUT_MODE_DIFF,
    .codec_gain.codec_adc_mic_amp_l = INNER_CODEC_MIC_AMP_12dB,
    .codec_gain.codec_adc_mic_amp_r = INNER_CODEC_MIC_AMP_12dB,
    .codec_gain.pga_gain_l = 28.5f,
    .codec_gain.pga_gain_r = 28.5f,
#if (MIC_DIFF_SINGLE == 0)
    .codec_gain.dig_gain_l = 1.0f,
#else
    .codec_gain.dig_gain_l = 6.0f,//数字增益配置，该值加/减 1 ,数字增益加/减 1dB
#endif 
    .codec_gain.dig_gain_r = 1.0f,
    .codec_if = 
    {
        .codec_init = icodec_init, //codec初始化函数
        .codec_config = icodec_config,  //codec配置函数
        .codec_start = icodec_start,  
        .codec_stop = icodec_stop,
        .codec_ioctl = icodec_ioctl,  //codec控制功能函数
    }
}; 

/**
 * @brief 播音配置参数表
 *
 */
audio_format_info_t audio_format_info = 
{
    .samprate = 16000,  //播音采样率
    .nChans = 2,         //通道数
    .out_min_size = 1152,   //帧长度
};

/**
 * @brief 录音配置参数表
 *
 */
const cm_sound_info_t host_mic_sound_info = {
    .sample_rate = 32000,  //录音采样率
    #if (2 == HOST_CODEC_CHA_NUM) //使用aec时需要开启双通道 mic和ref通道
    .channel_flag = 3,          //bit[0] = 1选择左声道，bit[1]=1选择右声道，可以用或运算组合
	#else
    .channel_flag = 1,          //bit[0] = 1选择左声道，bit[1]=1选择右声道，可以用或运算组合
    #endif
    .sample_depth = IIS_DW_16BIT,  //数据宽度
};

/**
 * @brief 录音codec注册
 *
 */
void audio_in_codec_registe()
{
    /*还应该加入codec power up的操作*/
    cm_reg_codec(HOST_MIC_RECORD_CODEC_ID, (cm_codec_hw_info_t*)&host_mic_hw_info);
    uint32_t block_size = 0;
    if(3 == host_mic_sound_info.channel_flag)
    {
        block_size = AUDIO_CAP_POINT_NUM_PER_FRM * sizeof(int16_t) * 2 * 2;/*32K采样率*/
    }
    else
    {
        block_size = AUDIO_CAP_POINT_NUM_PER_FRM * sizeof(int16_t) * 2;/*32K采样率*/
    }
    //配置录音PCM buffer
    // cm_record_buffer_info_t record_buffer_info;
    cm_pcm_buffer_info_t record_buffer_info_str;
    record_buffer_info_str.record_buffer_info.block_num = AUDIO_IN_BUFFER_NUM;
    record_buffer_info_str.record_buffer_info.block_size = block_size;//576;
    record_buffer_info_str.record_buffer_info.buffer_size = record_buffer_info_str.record_buffer_info.block_size * record_buffer_info_str.record_buffer_info.block_num;
    record_buffer_info_str.record_buffer_info.pcm_buffer = (void*)pvPortMalloc(record_buffer_info_str.record_buffer_info.buffer_size);
    cm_config_pcm_buffer(HOST_MIC_RECORD_CODEC_ID, CODEC_INPUT, &record_buffer_info_str);
    //配置录音音频格式
    cm_config_codec(HOST_MIC_RECORD_CODEC_ID, CODEC_INPUT, &host_mic_sound_info);

    #if 1/*内部ADC探针模式*/
    // Scu_IIS_PAD_DataConfig(PAD_IIS_DATA_FROM_CODEC_ADC);
    // Scu_IIS_PAD_DataConfig(PAD_IIS_DATA_FROM_PDM);
    #endif

    pad_config_for_power_amplifier();

    // extern int pdm_init(void);
    // pdm_init();
    // while(1);
}   

