/**
 * @file audio_play_api.c
 * @brief  播放器接口函数
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include "audio_play_api.h"
#include "audio_play_process.h"
#include "get_play_data.h"
#include "ci130x_system.h"
#include "ci130x_core_misc.h"
#include <string.h>
#include "ci_log.h"
#include "romlib_runtime.h"

/*! majar version number */
#define AUDIO_PLAY_VERSION              2L  
/*! minor version number */
#define AUDIO_PLAY_SUBVERSION           8L 
/*! revise version number */ 
#define AUDIO_PLAY_REVISION             3L  


/**
 * @brief 获取播放器组件版本号
 * 
 * @param version 版本号buf，3byte
 * @retval RETURN_OK 获取成功
 * @retval RETURN_ERR 获取失败
 */
int32_t get_audio_play_version(char version[3])
{
    version[0] = AUDIO_PLAY_VERSION;
    version[1] = AUDIO_PLAY_SUBVERSION;
    version[2] = AUDIO_PLAY_REVISION;

    return RETURN_OK;
}


/**
 * @brief 暂停当前的audio播放新的audio
 * @note 目前仅能记录历史mp3暂停状态1条，如需增加需扩展g_wave_total_lens和get_play_data.c中data_list
 * @param dir 文件路径
 * @param name 文件名
 * @param offset 播放偏移 
 * @param decoder_name 解码器名称 
 * @param end_paly_callback 结束播放回调函数指针 
 */
void pause_audio_play_audio(const char *dir,const char *name,int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;
    int32_t ret = RETURN_ERR;
    do{
        uint32_t state = get_audio_play_state();

        ci_logdebug(LOG_AUDIO_PLAY,"##### 以前的状态 %d\n ",state);
        if(AUDIO_PLAY_STATE_IDLE != state)
        {
            play_msg.decoder_name = decoder_name;
            play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_SD;
            play_msg.audio_play_end_hook = end_paly_callback;
            play_msg.offset = offset;
            play_msg.sd_data.dir = dir;
            play_msg.sd_data.name = name;
            play_msg.audio_device_arg.pa_cmd = ENABLE;
            audio_play_queue_overwrite(audio_play_queue, &play_msg);
            audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_PAUSE);  
            audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);       
            ret = RETURN_OK;
        }
        else
        {
            ret = play_audio(dir,name,0,decoder_name,end_paly_callback);
        }
    }while(ret == RETURN_ERR);
}


/**
 * @brief 播放最近一次被打断的mp3
 * 
 * @param end_paly_callback 结束播放回调函数指针 
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int32_t continue_history_play(CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;
    
    if(RETURN_OK != continue_data_valid())
    {
        return RETURN_ERR;
    }
    
    if(AUDIO_PLAY_STATE_IDLE != get_audio_play_state())
    {
        return RETURN_ERR;        
    }

    play_msg.decoder_name = "history";
    play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_CONTINUE;
    play_msg.audio_device_arg.pa_cmd = ENABLE;
    play_msg.audio_play_end_hook = end_paly_callback;

    audio_play_queue_overwrite(audio_play_queue, &play_msg);
    audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);   
    return RETURN_OK;
}


/**
 * @brief 从sd卡或网络播放音乐
 * @note 可以指定解码器或不指定解码器，如果不指定解码器音频且文件头不是标准头会因找不到解码器而无法播放
 * 
 * @param dir_or_url 播放路径
 * @param name 文件名称
 * @param offset 播放偏移 
 * @param decoder_name 解码器名称 
 * @param end_paly_callback 结束播放回调函数指针 
 * 
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int32_t play_audio(const char *dir_or_url,const char *name,int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;

    #if AUDIO_PLAY_USE_NET
    /* 判断是网络地址还是文件系统地址 */
    if(dir_or_url[0] != '/')
    {
        play_msg.audio_play_end_hook = end_paly_callback;
        play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_NET;
        play_msg.net_data.url = dir_or_url;
    }
    else
    #endif
    {
        uint32_t dir_len = MASK_ROM_LIB_FUNC->newlibcfunc.strlen_p(dir_or_url);
        uint32_t name_len = MASK_ROM_LIB_FUNC->newlibcfunc.strlen_p(name);

        if((dir_len >= AUDIO_FILE_MAX_NAME_SIZE)||(name_len >= AUDIO_FILE_MAX_NAME_SIZE))
        {
            return RETURN_ERR;
        }
        
        play_msg.audio_play_end_hook = end_paly_callback;
        play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_SD;
        play_msg.sd_data.dir = dir_or_url;
        play_msg.sd_data.name = name;
    }
    play_msg.offset = offset;
    play_msg.audio_device_arg.pa_cmd = ENABLE;
    play_msg.decoder_name = decoder_name;

    if(AUDIO_PLAY_STATE_IDLE == get_audio_play_state())
    {
        audio_play_queue_overwrite(audio_play_queue, &play_msg);//覆盖写入，用于一个的队列
        audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);             
        return RETURN_OK;
    }

    return RETURN_ERR;
}


/**
 * @brief 播放ADPCM
 * 
 * @param data_addr 播报词在flash内地址
 * @param end_paly_callback 结束播放回调函数指针 
 * 
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int32_t play_prompt(uint32_t data_addr, uint32_t data_addr_num, CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;
    
    play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_FLASH;
    play_msg.flash_data.data_addr_num = data_addr_num;
    play_msg.flash_data.arg = data_addr;
    play_msg.audio_play_end_hook = end_paly_callback;
    play_msg.decoder_name = "prompt";
    play_msg.offset = 0;
    play_msg.audio_device_arg.pa_cmd = DISABLE;
    if(AUDIO_PLAY_STATE_IDLE != get_audio_play_state())
    {
        return RETURN_ERR;
    }

    audio_play_queue_overwrite(audio_play_queue, &play_msg);
    audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);   
    return RETURN_OK;
}


/**
 * @brief 暂停当前的audio播放adpcm
 * 
 * @param data_addr 播报词在flash内地址
 * @param end_paly_callback 结束播放回调函数指针 
 */
void pause_audio_play_prompt(uint32_t data_addr, uint32_t data_addr_num, CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;
    int32_t ret = RETURN_ERR;
    do{
        uint32_t state = get_audio_play_state();

        ci_logdebug(LOG_AUDIO_PLAY,"##### 以前的状态 %d\n ",state);
        if(AUDIO_PLAY_STATE_IDLE != state)
        {
            play_msg.decoder_name = "prompt";
            play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_FLASH;
            play_msg.offset = 0;
            play_msg.flash_data.data_addr_num = data_addr_num;
            play_msg.flash_data.arg = data_addr;
            play_msg.audio_device_arg.pa_cmd = DISABLE;
            play_msg.audio_play_end_hook = end_paly_callback;
            audio_play_queue_overwrite(audio_play_queue, &play_msg);
            audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_PAUSE);  
            audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);       
            ret = RETURN_OK;
        }
        else
        {
            ret = play_prompt(data_addr,data_addr_num,end_paly_callback);
        }
    }while(ret == RETURN_ERR);
}


/**
 * @brief 播放来自外部的数据源音频
 * 
 * 用于播放外部数据源音频，需要调用外部数据流写入原始数据  
 * 
 * @note 可以指定解码器或不指定解码器，如果不指定解码器音频且文件头不是标准头会因找不到解码器而无法播放
 * 
 * @param decoder_name 解码器名称 
 * @param end_paly_callback 结束播放回调函数指针 
 * 
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int32_t play_with_outside(int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;

    play_msg.audio_play_end_hook = end_paly_callback;
    play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_OUTSIDE;
    play_msg.decoder_name = decoder_name;
    play_msg.offset = offset;
    play_msg.audio_device_arg.pa_cmd = ENABLE;

    if(AUDIO_PLAY_STATE_IDLE == get_audio_play_state())
    {
        audio_play_queue_overwrite(audio_play_queue, &play_msg);//覆盖写入，用于一个的队列
        audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);             
        return RETURN_OK;
    }

    return RETURN_ERR;
}


/**
 * @brief 播放来自外部的数据源音频
 * 
 * 用于播放外部数据源音频，需要调用外部数据流写入原始数据  
 * 
 * @note 可以指定解码器或不指定解码器，如果不指定解码器音频且文件头不是标准头会因找不到解码器而无法播放
 * 
 * @param decoder_name 解码器名称 
 * @param end_paly_callback 结束播放回调函数指针 
 * 
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int32_t play_with_outside_v2(int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback)
{
    audio_play_msg_t play_msg;

    play_msg.audio_play_end_hook = end_paly_callback;
    play_msg.play_data_type = AUDIO_PLAY_DATA_TYPE_OUTSIDE_V2;
    play_msg.decoder_name = decoder_name;
    play_msg.offset = offset;
    play_msg.audio_device_arg.pa_cmd = ENABLE;

    if(AUDIO_PLAY_STATE_IDLE == get_audio_play_state())
    {
        audio_play_queue_overwrite(audio_play_queue, &play_msg);//覆盖写入，用于一个的队列
        audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_START);             
        return RETURN_OK;
    }

    return RETURN_ERR;
}


/**
 * @brief 停止播放
 * 
 * @param end_paly_callback 结束播放回调函数指针 
 * @retval RETURN_OK 停止成功
 * @retval RETURN_ERR 停止失败（可能已经停止或未进行播放）
 */
int32_t stop_play(CALLBACK_END_PLAY end_paly_callback,void *xHigherPriorityTaskWoken)
{ 
    ci_logdebug(LOG_AUDIO_PLAY,"##### 以前的状态 %d\n ",get_audio_play_state());
    if(AUDIO_PLAY_STATE_IDLE == get_audio_play_state())
    {
        return RETURN_ERR;
    }

    if(end_paly_callback != NULL)
    {
        if(RETURN_ERR == fastset_audio_end_callback(end_paly_callback))
        {
            return RETURN_ERR;
        }
    }
    if(0 != check_curr_trap())
    {
        audio_play_event_group_set_bits_isr(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_STOP,xHigherPriorityTaskWoken);  
    }
    else
    {
        audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_STOP);  
    }
    return RETURN_OK;
}


/**
 * @brief 暂停播放
 * 
 * @param end_paly_callback 结束播放回调函数指针 
 * @retval RETURN_OK 暂停成功
 * @retval RETURN_ERR 暂停失败（可能已经停止或未进行播放）
 */
int32_t pause_play(CALLBACK_END_PLAY end_paly_callback,void *xHigherPriorityTaskWoken)
{ 
    ci_logdebug(LOG_AUDIO_PLAY,"##### 以前的状态 %d\n ",get_audio_play_state());
    if(AUDIO_PLAY_STATE_IDLE == get_audio_play_state())
    {
        return RETURN_ERR;
    }
    if(end_paly_callback != NULL)
    {
        if(RETURN_ERR == fastset_audio_end_callback(end_paly_callback))
        {
            return RETURN_ERR;
        }
    }
    if(0 != check_curr_trap())
    {
        audio_play_event_group_set_bits_isr(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_PAUSE,xHigherPriorityTaskWoken);  
    }
    else
    {
        audio_play_event_group_set_bits(audio_play_cmd_event_group,AUDIO_PLAY_EVENT_PAUSE);  
    }
    return RETURN_OK;
}

/**
 * @brief 获取当前的播放进度，
 * @note  在stop后仍可以获取最近一次播放结束位置，重新start会从0重新计算
 * 
 * @return uint32_t 当前播放进度偏移
 */
uint32_t get_play_offset(void)
{
    return get_curr_offset();
}


/**
 * @brief 获取读入数据缓冲器进度，
 * @note  在stop后仍可以获取最近一次播放结束位置，重新start会从0重新计算
 * 
 * @return uint32_t 当前数据加载进度偏移
 */
uint32_t get_data_load_offset(void)
{
    #if AUDIO_PLAY_USE_NET
    return get_net_download_offset();
    #else
    // TODO: 目前不支持非网络数据的缓冲
    return 0;
    #endif
}
