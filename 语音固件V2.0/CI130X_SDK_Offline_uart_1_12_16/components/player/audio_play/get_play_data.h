/**
 * @file get_play_data.h
 * @brief 用于获取播放数据，提供从SD卡，flash，串口等读入数据
 * @version V2.0.1
 * @date 2018.07.17
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef _GET_PLAY_DATA_H_
#define _GET_PLAY_DATA_H_

#include "sdk_default_config.h"
#include "audio_play_config.h"

/**
 * @ingroup audio_player
 * @defgroup audio_play_get_play_data 播放器数据读入任务
 * @brief 音频播放器组件数据读入任务
 * @{
 */

/*! 数据请求消息数量 */
#define GET_PLAY_DATA_QUEUE_COUNT                  (1)
/*! 数据buf个数 */
#define GET_PLAY_DATA_BUFF_COUNT                   (2)
/*! 数据buf大小 */
#define GET_PLAY_DATA_BUFF_SIZE                    (576U)

/**
 * @brief 数据标志
 * 
 */
enum 
{
    AUDIO_DATD_BUFF_NORMAL = 0x0,        /*!< 普通数据               */
    AUDIO_DATD_BUFF_END = 0x1,           /*!< 数据结束               */
    AUDIO_DATD_BUFF_ERR = 0x2,           /*!< 数据出错               */
};

/**
 * @brief 数据获取task状态机
 * 
 */
typedef enum
{
    GET_PLAY_DATA_STATE_IDLE,            /*!< 空闲状态               */
    GET_PLAY_DATA_STATE_START,           /*!< 启动状态               */
    GET_PLAY_DATA_STATE_BUSY,            /*!< 忙碌状态               */
    GET_PLAY_DATA_STATE_STOP,            /*!< 停止状态               */
} get_play_data_state_t;

extern void task_get_play_data_init(void);
extern void task_get_play_data(void *pvParameters);
extern int32_t get_net_data(const char *url, uint32_t offset);
extern int32_t get_flash_data(uint32_t addr, uint32_t num);
extern int32_t get_sd_data(const char *dir,const char *name,uint32_t offset);
extern int32_t get_outside_data(uint32_t offset);
extern int32_t get_outside_v2_data(uint32_t offset);
extern int32_t receive_data(uint32_t *addr,uint32_t *size,uint32_t *flag);
extern void finsh_read_data(void);
extern int32_t pause_read_data(void);
extern int32_t continue_read_data(void);
extern int32_t get_pause_read_data_offset(void);
extern int32_t continue_data_valid(void);
extern get_play_data_state_t get_data_task_state(void);
extern uint32_t get_curr_offset(void);
#if AUDIO_PLAY_USE_NET
extern uint32_t get_net_download_offset(void);
#endif

/** @} */

#endif /*_GET_PLAY_DATA_H_*/
