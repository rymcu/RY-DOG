/**
 * @file audio_play_device.c
 * @brief  播放器底层设备
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include "audio_play_device.h"
#include "audio_play_api.h"
#include "audio_play_process.h"
#include <string.h>
#include "audio_play_os_port.h"
#include "ci130x_codec.h"
#include "sdk_default_config.h"
#include "FreeRTOS.h"
#include "codec_manager.h"
#include "romlib_runtime.h"
#include "board.h"


extern int cm_start_codec(int codec_index, io_direction_t io_dir);      //TODO HSL
extern int cm_stop_codec(int codec_index, io_direction_t io_dir);       //TODO HSL
extern int cm_set_codec_mute(int codec_index, io_direction_t io_dir, int channel_flag, FunctionalState en);     //TODO HSL
extern int cm_set_codec_dac_gain(int codec_index, cm_cha_sel_t cha, int gain);       //TODO HSL
extern void cm_get_pcm_buffer(int codec_index,uint32_t* ret_buf,uint32_t wait_tick);
extern int cm_write_codec(int codec_index, void * pcm_buffer,uint32_t wait_tick);

int sg_play_device_index = PLAY_CODEC_ID;
static int32_t g_audio_play_gain = 0;


/**
 * @brief buf释放回调函数
 * 
 * @param xHigherPriorityTaskWoken 中断退出OS调度任务标志
 */
void send_one_buf_done_event(void * xHigherPriorityTaskWoken)
{
    /* 设置buf done 事件标志 */
    audio_play_event_group_set_bits_isr(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_ONE_BUF_DONE,xHigherPriorityTaskWoken ); 
}


/**
 * @brief 硬件超时回调函数
 * 
 * @param xHigherPriorityTaskWoken 中断退出OS调度任务标志
 */
void audio_play_hw_timeout(void * xHigherPriorityTaskWoken)
{
    /* 设置buf done 事件标志 */
    audio_play_event_group_set_bits_isr(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_PLAY_TIMEOUT,xHigherPriorityTaskWoken ); 
}


/**
 * @brief 播放硬件声卡预初始化
 * 
 */

void audio_play_hw_per_init(void)
{
    /* 声卡硬件配置注册 */
    
}


/**
 * @brief 调节播放音量
 * 
 * @param gain 音量(0--100)
 */
void audio_play_set_vol_gain(int32_t gain)
{
    g_audio_play_gain = gain;

    cm_set_codec_dac_gain(sg_play_device_index, 0, gain);
}


/**
 * @brief 获取当前播放音量
 * 
 * @return int32_t 音量(0--100)
 */
int32_t audio_play_get_vol_gain(void)
{
    return g_audio_play_gain;
}


/**
 * @brief 设置静音
 * 
 * @param is_mute 是否静音
 */
void audio_play_set_mute(bool is_mute)
{
    if(is_mute)
    {
        // audio_play_card_force_mute(sg_play_device_num,ENABLE);
    }
    else
    {
        // audio_play_card_force_mute(sg_play_device_num,DISABLE);
    }
}


/**
 * @brief pa、da控制，可选择是否控制功放
 * 
 * @param cmd pa使能或失能
 * 
 */
void audio_play_hw_pa_da_ctl(FunctionalState cmd,bool is_control_pa)
{
    typedef enum
    {
        CODEC_INPUT,            /*!< 输入 */
        CODEC_OUTPUT,           /*!< 输出 */
    }io_direction_t;
    //PA的操作需要在DAC关闭的情况下进行，不然会影响采音
    if(ENABLE == cmd)
    {
        icodec_start(CODEC_OUTPUT);
        if(is_control_pa)
        {
            power_amplifier_on();
        }
    }
    else
    {
        if(is_control_pa)
        {
            power_amplifier_off();
        }
        icodec_stop(CODEC_OUTPUT);
    }
}


/**
 * @brief 声卡驱动开始播放
 * 
 */
void audio_play_hw_start(FunctionalState pa_cmd, audio_format_info_t *audio_format_info)
{
	//播放之前，关ALC
    static uint32_t last_hw_samplerate = 0; 
    static uint32_t last_hw_block_size = 0;     
    
    // 配置声卡
    cm_sound_info_t sound_info;

    static uint32_t pre_sample_rate = 0;
    static uint32_t pre_nChans = 0;

    sound_info.sample_rate = audio_format_info->samprate;
    sound_info.channel_flag = (audio_format_info->nChans == 2) ? 3:1;
    sound_info.sample_depth = IIS_DW_16BIT;

    
    if((pre_sample_rate != audio_format_info->samprate) || (pre_nChans != audio_format_info->nChans))
    {
        pre_sample_rate = sound_info.sample_rate;
        pre_nChans = audio_format_info->nChans;
        cm_config_codec(sg_play_device_index, CODEC_OUTPUT, &sound_info);
    }
    
    cm_start_codec(sg_play_device_index, CODEC_OUTPUT);
    cm_set_codec_mute(sg_play_device_index, CODEC_OUTPUT, 3, DISABLE);
    //只要DAC的开关不会影响采音，就可以直接开关DAC，而不用开关HP
    /* 启动PA */
    if(ENABLE == pa_cmd)
    {
    }
    else
    {
        //只要DAC的开关不会影响采音，就可以直接开关DAC，而不用开关HP
    }
}

/**
 * @brief 声卡驱动停止播放
 * 
 */
void audio_play_hw_stop(FunctionalState pa_cmd)
{
	//播放结束，开ALC
    cm_set_codec_mute(sg_play_device_index, CODEC_OUTPUT, 3, ENABLE);

     /* 停止声卡硬件 */
    cm_stop_codec(sg_play_device_index, CODEC_OUTPUT);
}


/**
 * @brief 向硬件写入音频数据
 * 
 */
int32_t audio_play_hw_write_data(void* pcm_buf,uint32_t buf_size)
{
    uint32_t p = 0;
    cm_get_pcm_buffer(sg_play_device_index,&p,portMAX_DELAY);
    if(0 == p)
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)p, pcm_buf, buf_size);
    cm_write_codec(sg_play_device_index,(void*)p,portMAX_DELAY);
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 清理写入硬件的音频数据
 * 
 */
int32_t audio_play_hw_clean_data(void)
{
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 查询音频数据消息是否发满
 * 
 */
int32_t audio_play_hw_queue_is_full(void)
{
    return cm_get_codec_empty_buffer_number(sg_play_device_index, CODEC_OUTPUT);
}


/**
 * @brief 查询硬件待处理的消息个数
 * 
 */
int32_t audio_play_hw_get_queue_remain_nums(void)
{
    return cm_get_codec_busy_buffer_number(sg_play_device_index, CODEC_OUTPUT);
}


#if AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_HARD
void audio_play_hw_fix_pop_issue_hard(void)
{
    extern int32_t get_cur_dac_l_gain(void);
    extern int32_t get_cur_dac_r_gain(void);
    int32_t dac_l_gain = get_cur_dac_l_gain();
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*95/100,dac_l_gain*95/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*90/100,dac_l_gain*90/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*85/100,dac_l_gain*85/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*80/100,dac_l_gain*80/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*60/100,dac_l_gain*60/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*40/100,dac_l_gain*40/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,dac_l_gain*20/100,dac_l_gain*20/100);
    audio_play_task_delay(pdMS_TO_TICKS(2));
    audio_play_card_set_vol_gain(sg_play_device_num,0,0);
}


void audio_play_hw_fix_pop_issue_hard_done(void)
{
    audio_play_set_vol_gain(audio_play_get_vol_gain());
}
#endif


#if AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_SOFT
void audio_play_hw_fix_pop_issue_soft(void)
{
    int32_t timeout = pdMS_TO_TICKS(100);
    audio_play_card_set_pause_flag(AUDIO_PLAY_CARD_PAUSE_DATA_DECREACE_1);
    ci_logdebug(LOG_AUDIO_PLAY,"dp%d\n",AUDIO_PLAY_CARD_PAUSE_DATA_DECREACE_1);
    while(AUDIO_PLAY_CARD_PAUSE_END != audio_play_card_get_pause_flag())
    {
        audio_play_task_delay(2);
        timeout -= 2;
        if(timeout <= 0)
        {
            /* 超时跳出 */
            break;
        }
    }
}


void audio_play_hw_fix_pop_issue_soft_done(void)
{
    audio_play_card_set_pause_flag(AUDIO_PLAY_CARD_PAUSE_NULL);
}   
#endif
