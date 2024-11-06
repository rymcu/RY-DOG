/**
 * @file audio_play_os_port.h
 * @brief  播放器组件OS移植层
 * @version 1.0
 * @date 2019-05-08
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef AUDIO_PLAY_OS_PORT_H
#define AUDIO_PLAY_OS_PORT_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "audio_play_config.h"

/**
 * @ingroup audio_player
 * @defgroup audio_play_os_port 播放器OS移植层
 * @brief 音频播放器组件OS移植层
 * @{
 */

/**
 * @brief 任务入口函数句柄类型
 * 
 */
typedef void (*audio_play_os_task_fun_t)( void * );

/**
 * @brief 任务句柄类型
 * 
 */
typedef void* audio_play_os_task_t;

/**
 * @brief 信号量句柄类型
 * 
 */
typedef void* audio_play_os_sem_t;

/**
 * @brief 队列句柄类型
 * 
 */
typedef void* audio_play_os_queue_t;

/**
 * @brief 数据流句柄类型
 * 
 */
typedef void* audio_play_os_stream_t;

/**
 * @brief 事件标志组类型
 * 
 */
typedef void* audio_play_os_event_group_t;

/**
 * @brief 错误类型
 * 
 */
typedef enum
{
    AUDIO_PLAY_OS_SUCCESS = 0,   /*!< 无错误    */
    AUDIO_PLAY_OS_FAIL = -1,     /*!< 未知错误  */
}audio_play_os_state_t;

extern const uint32_t audio_play_os_block;
extern const uint32_t audio_play_os_unblock;

/************ task ***************/
audio_play_os_state_t audio_play_task_create(audio_play_os_task_fun_t pxTaskCode, const char *const pcName, const uint16_t usStackDepth,
                                  void *const pvParameters, uint32_t uxPriority, audio_play_os_task_t* pxCreatedTask);
void audio_play_task_delete(audio_play_os_task_t xTaskToDelete);
void audio_play_task_suspend(audio_play_os_task_t xTaskToSuspend);
void audio_play_task_resume(audio_play_os_task_t xTaskToResume);
void audio_play_task_delay(const uint32_t xTicksToDelay);
/********* semaphore *************/
audio_play_os_sem_t audio_play_semaphore_create_binary(void);
void audio_play_semaphore_delete(audio_play_os_sem_t xSemaphore);
audio_play_os_state_t audio_play_semaphore_give(audio_play_os_sem_t xQueue);
audio_play_os_state_t audio_play_semaphore_take(audio_play_os_sem_t xSemaphore, uint32_t xTicksToWait);
/*********** queue ***************/
audio_play_os_queue_t audio_play_queue_create(const uint32_t uxQueueLength, const uint32_t uxItemSize);
audio_play_os_state_t audio_play_queue_send(audio_play_os_queue_t xQueue, const void *const pvItemToQueue, uint32_t xTicksToWait);
audio_play_os_state_t audio_play_queue_peek(audio_play_os_queue_t xQueue, void *const pvBuffer, uint32_t xTicksToWait);
audio_play_os_state_t audio_play_queue_overwrite(audio_play_os_queue_t xQueue, const void *const pvItemToQueue);
audio_play_os_state_t audio_play_queue_delete(audio_play_os_queue_t xQueue);
audio_play_os_state_t audio_play_queue_receive(audio_play_os_queue_t xQueue, void *const pvBuffer, uint32_t xTicksToWait);
audio_play_os_state_t audio_play_queue_reset(audio_play_os_queue_t xQueue);
audio_play_os_state_t audio_play_queue_is_full(audio_play_os_queue_t xQueue);
audio_play_os_state_t audio_play_queue_is_empty(audio_play_os_queue_t xQueue);
/******** stream_buffer **********/
audio_play_os_stream_t audio_play_stream_buffer_create(size_t xBufferSizeBytes, size_t xTriggerLevelBytes);
void audio_play_stream_buffer_delete(audio_play_os_stream_t xStreamBuffer);
size_t audio_play_stream_buffer_send(audio_play_os_stream_t xStreamBuffer, const void *pvTxData, size_t xDataLengthBytes,
                                     uint32_t xTicksToWait);
size_t audio_play_stream_buffer_receive(audio_play_os_stream_t xStreamBuffer, void *pvRxData, size_t xBufferLengthBytes,
                                        uint32_t xTicksToWait);
audio_play_os_state_t audio_play_stream_buffer_reset(audio_play_os_stream_t xStreamBuffer);
audio_play_os_state_t audio_play_stream_buffer_is_empty(audio_play_os_stream_t xStreamBuffer);
size_t audio_play_stream_buffer_get_spaces_size(audio_play_os_stream_t xStreamBuffer);
/******** event_group ************/
audio_play_os_event_group_t audio_play_event_group_create(void);
uint32_t audio_play_event_group_wait_bits(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToWaitFor,
                                             const audio_play_os_state_t xClearOnExit, const audio_play_os_state_t xWaitForAllBits, uint32_t xTicksToWait);
uint32_t audio_play_event_group_set_bits(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToSet);
audio_play_os_state_t audio_play_event_group_set_bits_isr(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToSet,
                                               void *pxHigherPriorityTaskWoken);
/************ heap ***************/
void *audio_play_malloc(size_t xWantedSize);
void *audio_play_calloc(size_t nmemb, size_t size);
void audio_play_free(void *pv);
void *audio_play_realloc(void *pv, size_t size);

/** @} */

#endif /* AUDIO_PLAY_OS_PORT_H */
