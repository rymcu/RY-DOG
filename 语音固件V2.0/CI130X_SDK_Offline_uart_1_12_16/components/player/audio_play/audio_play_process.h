/**
 * @file audio_play_process.h
 * @brief  播放器
 * @version 2.0.2
 * @date 2019-03-15
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef _AUDIO_PLAY_PROCESS_H_
#define _AUDIO_PLAY_PROCESS_H_

#include <string.h>
#include <stdlib.h>
#include "sdk_default_config.h"
#include "audio_play_os_port.h"
#include "audio_play_config.h"
#include "ci130x_system.h"

/**
 * @ingroup audio_player
 * @defgroup audio_play_process 播放器主任务
 * @brief 音频播放器组件主任务
 * @{
 */

/**
 * @brief 播放数请求类型
 * 
 */
typedef enum
{
    AUDIO_PLAY_DATA_TYPE_FLASH = 1,     /*!< 播放源来自flash      */
    AUDIO_PLAY_DATA_TYPE_SD = 2,        /*!< 播放源来自sd卡       */
    AUDIO_PLAY_DATA_TYPE_NET = 3,       /*!< 播放源来自网络       */
    AUDIO_PLAY_DATA_TYPE_OUTSIDE = 4,   /*!< 播放源来自外部输入   */
    AUDIO_PLAY_DATA_TYPE_OUTSIDE_V2 = 5,/*!< 播放源来自外部输入   */

    AUDIO_PLAY_DATA_TYPE_CONTINUE = 99, /*!< 播放继续历史播放     */
}play_data_type_t;

/**
 * @brief 播放事件标志 
 */
enum
{
    AUDIO_PLAY_EVENT_NONE         =  (1<<0),    /*!< 无                  */
    AUDIO_PLAY_EVENT_ONE_BUF_DONE =  (1<<4),    /*!< buff数据播放完毕     */
    AUDIO_PLAY_EVENT_PAUSE        =  (1<<5),    /*!< 暂停播放             */
    AUDIO_PLAY_EVENT_RESUME       =  (1<<6),    /*!< 保留                 */
    AUDIO_PLAY_EVENT_STOP         =  (1<<7),    /*!< 停止播放             */
    AUDIO_PLAY_EVENT_PLAY_DONE    =  (1<<8),    /*!< 保留                 */
    AUDIO_PLAY_EVENT_START        =  (1<<9),    /*!< 启动播放             */
    AUDIO_PLAY_EVENT_PLAY_TIMEOUT =  (1<<10),   /*!< 超时                 */
    AUDIO_PLAY_EVENT_PREV         =  (1<<11),   /*!< 保留                 */
};

/**
 * @brief 结束播放回调函数
 * 
 */
typedef void(*CALLBACK_END_PLAY)(int32_t arg);


/**
 * @brief 播放请求消息数据结构
 * 
 */
typedef struct
{
    const char *decoder_name;                 /*!< 解码器类型                */
    play_data_type_t play_data_type;          /*!< 数据源类型                */
    /** @brief 音频数据请求 */
    union
    {
        /** @brief sd卡音频数据请求 */
        struct
        {   
            const char *dir;                  /*!< 路径位置                   */
            const char *name;                 /*!< 文件名称                   */
        }sd_data;                             /*!< sd卡数据                   */
        /** @brief 网络音频数据请求 */
        struct
        {   
            const char *url;                  /*!< 网络url                    */
        }net_data;                            /*!< 网络数据                   */
        /** @brief Flash音频数据请求 */
        struct
        {
            union
            {
                uint32_t arg;                 /*!< 参数                       */
                uint32_t data_addr;           /*!< 播报词地址                 */
                uint32_t data_addr_list;      /*!< 播报词地址序列             */
            };
            uint32_t data_addr_num;           /*!< 播报词地址数量             */
        }flash_data;                          /*!< flash数据                  */
        uint32_t outside_data;                /*!< 外部数据                   */
    };
    int offset;                               /*!< 起始偏移                   */
    struct
    {  
        FunctionalState pa_cmd;               /*!< 本次播放是否开关PA         */
    }audio_device_arg;                        /*!< 播放设备硬件参数           */
    CALLBACK_END_PLAY audio_play_end_hook;    /*!< play end callbak           */
}audio_play_msg_t;


/**
 * @brief audio play process 任务状态
 * 
 */
typedef enum
{
    AUDIO_PLAY_STATE_IDLE,     /*!< 播放空闲       */
    AUDIO_PLAY_STATE_START,    /*!< 播放开始       */
    AUDIO_PLAY_STATE_PLAYING,  /*!< 播放中         */
    AUDIO_PLAY_STATE_STOP,     /*!< 播放停止       */
} audio_play_state_t;


/**
 * @brief 音频信息数据结构
 * 
 */
typedef struct
{
    uint32_t samprate;          /*!< 采样率               */
    uint8_t nChans;             /*!< 通道数               */
    int32_t out_min_size;       /*!< 帧长度               */
}audio_format_info_t;


/***************************************************************************
                    extern 
****************************************************************************/
extern audio_play_os_event_group_t audio_play_cmd_event_group;
extern audio_play_os_queue_t audio_play_queue;
extern audio_play_os_task_t audio_play_task_handle;

/***************************************************************************
                    extern function
****************************************************************************/
extern void task_audio_play(void *pvParameters);
extern int32_t audio_play_init(void);
#if AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
extern int32_t read_weakup_voice_to_ram(uint32_t wakeup_voice_flash_addr);
#endif
extern void set_play_will_finish_threshold_data_size(int size);
extern uint32_t get_audio_play_state(void);
extern void get_audio_format_info(audio_format_info_t *format);
extern int32_t fastset_audio_end_callback(CALLBACK_END_PLAY callback);
extern uint32_t get_decode_sync_word_offset(void);
extern void clean_decode_sync_word_offset(void);

/** @} */

#endif /* _AUDIO_PLAY_PROCESS_H_ */
