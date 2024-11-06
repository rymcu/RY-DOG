/**
 * @file audio_play_api.h
 * @brief  播放器接口函数
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _AUDIO_PLAY_API_H_
#define _AUDIO_PLAY_API_H_

#include "audio_play_process.h"
#include "audio_play_config.h"
#include "audio_play_os_port.h"
#include "sdk_default_config.h"


enum
{
    AUDIO_PLAY_CB_STATE_UNKNOWN_ERR               = -99,
    AUDIO_PLAY_CB_STATE_DECODER_MEM_ERR           = -5,
    AUDIO_PLAY_CB_STATE_PARSE_MP3_MEM_ERR         = -4,
    AUDIO_PLAY_CB_STATE_PARSE_M4A_MEM_ERR         = -3,
    AUDIO_PLAY_CB_STATE_PARSE_FILE_LEASTDATA_ERR  = -2,
    AUDIO_PLAY_CB_STATE_INTERNAL_ERR              = -1,
    AUDIO_PLAY_CB_STATE_DONE                      = 0,
    AUDIO_PLAY_CB_STATE_PAUSE                     = 1,
    AUDIO_PLAY_CB_STATE_PAUSE_BEFORE_THRESHOLD    = 2,
    AUDIO_PLAY_CB_STATE_PAUSE_AFTER_THRESHOLD     = 3,
    AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD            = 4,
};

/**
 * @ingroup audio_player
 * @defgroup audio_play_api 音频播放器API
 * @brief 音频播放器组件API
 * @{
 */

int32_t get_audio_play_version(char version[3]);

int32_t play_prompt(uint32_t data_addr,uint32_t data_addr_num,CALLBACK_END_PLAY end_paly_callback);
void pause_audio_play_prompt(uint32_t data_addr,uint32_t data_addr_num,CALLBACK_END_PLAY end_paly_callback);
int32_t stop_play(CALLBACK_END_PLAY end_paly_callback,void *xHigherPriorityTaskWoken);
int32_t pause_play(CALLBACK_END_PLAY end_paly_callback,void *xHigherPriorityTaskWoken);
int32_t play_audio(const char *dir_or_url,const char *name,int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback);
void pause_audio_play_audio(const char *dir,const char *name,int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback);
int32_t continue_history_play(CALLBACK_END_PLAY end_paly_callback);

uint32_t get_play_offset(void);
uint32_t get_data_load_offset(void);
void audio_play_set_vol_gain(int32_t gain);
int32_t audio_play_get_vol_gain(void);
void audio_play_set_mute(bool is_mute);
void audio_play_hw_pa_da_ctl(FunctionalState cmd,bool is_control_pa);
void set_play_speed(float play_speed);
#if AUDIO_PLAYER_FIX_OFFSET_ISSUE
int32_t get_near_offset_data(uint8_t *src);
#endif

#if AUDIO_PLAY_USE_OUTSIDE
int32_t play_with_outside(int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback);
void outside_clear_stream(audio_play_os_stream_t outside_stream,audio_play_os_sem_t outside_end_sem);
void outside_send_end_sem(audio_play_os_sem_t outside_end_sem);
int32_t outside_write_stream(audio_play_os_stream_t outside_stream, uint32_t addr, uint32_t size, bool is_block);
void outside_init_stream(audio_play_os_stream_t *outside_stream, audio_play_os_sem_t *outside_end_sem, uint32_t full_size);
void outside_destroy_stream(audio_play_os_stream_t outside_stream, audio_play_os_sem_t outside_end_sem);
void set_curr_outside_handle(audio_play_os_stream_t outside_stream, audio_play_os_sem_t outside_end_sem);
#endif

#if AUDIO_PLAY_USE_OUTSIDE_V2
int32_t play_with_outside_v2(int offset,const char *decoder_name,CALLBACK_END_PLAY end_paly_callback);
void outside_v2_clear_stream(audio_play_os_queue_t outside_msg,audio_play_os_sem_t outside_end_sem);
void outside_v2_send_end_sem(audio_play_os_sem_t outside_end_sem);
int32_t outside_v2_write_msg(audio_play_os_queue_t outside_msg, uint32_t addr, bool is_block);
void outside_v2_init_msg(audio_play_os_queue_t *outside_msg, audio_play_os_sem_t *outside_end_sem, uint32_t msg_count);
void outside_v2_destroy_stream(audio_play_os_queue_t outside_msg, audio_play_os_sem_t outside_end_sem);
void set_curr_outside_v2_handle(audio_play_os_queue_t outside_msg, audio_play_os_sem_t outside_end_sem);
#endif

/** @} */

#endif /* _AUDIO_PLAY_API_H_ */
