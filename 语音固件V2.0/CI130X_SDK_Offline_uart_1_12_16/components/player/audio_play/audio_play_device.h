/**
 * @file audio_play_device.h
 * @brief  播放器底层设备
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef _AUDIO_PLAY_DEVICE_H_
#define _AUDIO_PLAY_DEVICE_H_

#include <stdint.h>
#include "audio_play_config.h"
#include "ci130x_system.h"
#include "audio_play_process.h"


/**
 * @ingroup audio_player
 * @defgroup audio_play_device 播放器底层声卡接口
 * @brief 音频播放器组件声卡移植层
 * @{
 */

void audio_play_hw_per_init(void);
void audio_play_hw_start(FunctionalState pa_cmd, audio_format_info_t *audio_format_info);
void audio_play_hw_stop(FunctionalState pa_cmd);
int32_t audio_play_hw_write_data(void* pcm_buf,uint32_t buf_size);
int32_t audio_play_hw_queue_is_full(void);
int32_t audio_play_hw_get_queue_remain_nums(void);
int32_t audio_play_hw_clean_data(void);
#if AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_HARD
void audio_play_hw_fix_pop_issue_hard(void);
void audio_play_hw_fix_pop_issue_hard_done(void);
#endif
#if AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_SOFT
void audio_play_hw_fix_pop_issue_soft(void);
void audio_play_hw_fix_pop_issue_soft_done(void);
#endif

/** @} */

#endif /* _AUDIO_PLAY_DEVICE_H_ */ 
