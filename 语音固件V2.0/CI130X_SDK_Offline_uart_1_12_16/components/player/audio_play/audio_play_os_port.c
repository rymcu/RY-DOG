/**
 * @file audio_play_os_port.c
 * @brief  播放器组件OS移植层
 * @version 1.0
 * @date 2019-05-08
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#include "audio_play_os_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"


/*! 阻塞等待delay */
const uint32_t audio_play_os_block = portMAX_DELAY;
const uint32_t audio_play_os_unblock = 0;


/**
 * @brief 任务创建
 * 
 * @param pxTaskCode 任务函数入口
 * @param pcName 任务名称
 * @param usStackDepth 栈大小
 * @param pvParameters 参数
 * @param uxPriority 优先级
 * @param pxCreatedTask 任务句柄指针
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 创建成功
 * @retval AUDIO_PLAY_OS_FAIL 创建失败
 */
audio_play_os_state_t audio_play_task_create(audio_play_os_task_fun_t pxTaskCode,
                                           const char *const pcName,
                                           const uint16_t usStackDepth,
                                           void *const pvParameters,
                                           uint32_t uxPriority,
                                           audio_play_os_task_t *pxCreatedTask)
{
    if (pdFALSE == xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, (TaskHandle_t *)pxCreatedTask))
    {
        return AUDIO_PLAY_OS_FAIL;
    }

    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 删除任务
 * 
 * @param xTaskToDelete 任务句柄
 */
void audio_play_task_delete(audio_play_os_task_t xTaskToDelete)
{
    vTaskDelete(xTaskToDelete);
}


/**
 * @brief 挂起任务
 * 
 * @param xTaskToSuspend 任务句柄
 */
void audio_play_task_suspend(audio_play_os_task_t xTaskToSuspend)
{
    vTaskSuspend(xTaskToSuspend);
}


/**
 * @brief 恢复任务
 * 
 * @param xTaskToResume 任务句柄
 */
void audio_play_task_resume(audio_play_os_task_t xTaskToResume)
{
    vTaskResume(xTaskToResume);
}


/**
 * @brief 任务休眠
 * 
 * @param xTicksToDelay 休眠tick数
 */
void audio_play_task_delay(const uint32_t xTicksToDelay)
{
    vTaskDelay(xTicksToDelay);
}


/**
 * @brief 创建二值信号量
 * 
 * @return audio_play_os_sem_t 信号量句柄
 */
audio_play_os_sem_t audio_play_semaphore_create_binary(void)
{
    return xSemaphoreCreateBinary();
}


/**
 * @brief 删除信号量
 * 
 * @param xSemaphore 信号量句柄
 */
void audio_play_semaphore_delete(audio_play_os_sem_t xSemaphore)
{
    vSemaphoreDelete(xSemaphore);
}


/**
 * @brief 发送信号量
 * 
 * @param xSemaphore 信号量句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 发送成功
 * @retval AUDIO_PLAY_OS_FAIL 发送失败
 */
audio_play_os_state_t audio_play_semaphore_give(audio_play_os_sem_t xSemaphore)
{
    if (pdFALSE == xSemaphoreGive(xSemaphore))
    {
        return AUDIO_PLAY_OS_FAIL;
    }

    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 获取信号量
 * 
 * @param xSemaphore 信号量句柄 
 * @param xTicksToWait 等待tick数
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 获取成功
 * @retval AUDIO_PLAY_OS_FAIL 获取失败
 */
audio_play_os_state_t audio_play_semaphore_take(audio_play_os_sem_t xSemaphore, uint32_t xTicksToWait)
{
    if (pdFALSE == xSemaphoreTake(xSemaphore, xTicksToWait))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 创建队列
 * 
 * @param uxQueueLength 消息大小
 * @param uxItemSize 队列长度
 * @return audio_play_os_queue_t 队列句柄
 */
audio_play_os_queue_t audio_play_queue_create(const uint32_t uxQueueLength, const uint32_t uxItemSize)
{
    return xQueueCreate(uxQueueLength, uxItemSize);
}


/**
 * @brief 发送队列消息
 * 
 * @param xQueue 队列句柄
 * @param pvItemToQueue 消息
 * @param xTicksToWait 等待tick数
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 发送成功
 * @retval AUDIO_PLAY_OS_FAIL 发送失败
 */
audio_play_os_state_t audio_play_queue_send(audio_play_os_queue_t xQueue, const void *const pvItemToQueue, uint32_t xTicksToWait)
{
    if (pdFALSE == xQueueSend(xQueue, pvItemToQueue, xTicksToWait))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 覆盖写入队列消息
 * 
 * @param xQueue 队列句柄
 * @param pvItemToQueue 消息
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 写入成功
 * @retval AUDIO_PLAY_OS_FAIL 写入失败
 */
audio_play_os_state_t audio_play_queue_overwrite(audio_play_os_queue_t xQueue, const void *const pvItemToQueue)
{
    if (pdFALSE == xQueueOverwrite(xQueue, pvItemToQueue))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 复制队列头部消息
 * 
 * @param xQueue 队列句柄
 * @param pvBuffer 消息buff
 * @param xTicksToWait 等待tick数
 *  
 * @retval AUDIO_PLAY_OS_SUCCESS 复制成功
 * @retval AUDIO_PLAY_OS_FAIL 复制失败
 */
audio_play_os_state_t audio_play_queue_peek(audio_play_os_queue_t xQueue, void *const pvBuffer, uint32_t xTicksToWait)
{
    if (pdFALSE == xQueuePeek(xQueue, pvBuffer, xTicksToWait))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 接受队列消息
 * 
 * @param xQueue 队列句柄
 * @param pvBuffer 消息buff
 * @param xTicksToWait 等待tick数
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 接受成功
 * @retval AUDIO_PLAY_OS_FAIL 接受失败
 */
audio_play_os_state_t audio_play_queue_receive(audio_play_os_queue_t xQueue, void *const pvBuffer, uint32_t xTicksToWait)
{
    if (pdFALSE == xQueueReceive(xQueue, pvBuffer, xTicksToWait))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 复位队列
 * 
 * @param xQueue 队列句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 复位成功
 * @retval AUDIO_PLAY_OS_FAIL 复位失败
 */
audio_play_os_state_t audio_play_queue_reset(audio_play_os_queue_t xQueue)
{
    if (pdFALSE == xQueueReset(xQueue))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 查询队列是否满
 * 
 * @param xQueue 队列句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 队列已满
 * @retval AUDIO_PLAY_OS_FAIL 队列未满
 */
audio_play_os_state_t audio_play_queue_is_full(audio_play_os_queue_t xQueue)
{
    if (pdFALSE == xQueueGenericIsFull(xQueue))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 销毁队列
 * 
 * @param xQueue 队列句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 队列销毁成功
 * @retval AUDIO_PLAY_OS_FAIL 队列销毁失败
 */
audio_play_os_state_t audio_play_queue_delete(audio_play_os_queue_t xQueue)
{
    vQueueDelete(xQueue);

    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 查询队列是否为空
 * 
 * @param xQueue 队列句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 队列已空
 * @retval AUDIO_PLAY_OS_FAIL 队列未空
 */
audio_play_os_state_t audio_play_queue_is_empty(audio_play_os_queue_t xQueue)
{
    if (pdFALSE == xQueueGenericIsEmpty(xQueue))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 创建数据缓存流
 * 
 * @param xBufferSizeBytes 缓冲区大小 
 * @param xTriggerLevelBytes 触发深度
 * @return audio_play_os_stream_t 缓存流句柄
 */
audio_play_os_stream_t audio_play_stream_buffer_create(size_t xBufferSizeBytes, size_t xTriggerLevelBytes)
{
    return xStreamBufferCreate(xBufferSizeBytes, xTriggerLevelBytes);
}


/**
 * @brief 删除数据缓存流
 * 
 * @param xStreamBuffer 缓存流句柄
 */
void audio_play_stream_buffer_delete(audio_play_os_stream_t xStreamBuffer)
{
    vStreamBufferDelete(xStreamBuffer);
}


/**
 * @brief 向缓存流写入数据
 * 
 * @param xStreamBuffer 缓存流句柄
 * @param pvTxData 数据地址
 * @param xDataLengthBytes 数据大小 
 * @param xTicksToWait 等待tick数
 * @return size_t 成功写入大小
 */
size_t audio_play_stream_buffer_send(audio_play_os_stream_t xStreamBuffer,
                                     const void *pvTxData,
                                     size_t xDataLengthBytes,
                                     uint32_t xTicksToWait)
{
    return xStreamBufferSend(xStreamBuffer, pvTxData, xDataLengthBytes, xTicksToWait);
}


/**
 * @brief 查询缓冲流中的剩余空间大小
 * @param xStreamBuffer 缓存流句柄
 * 
 * @return size_t 剩余空间大小
 */
size_t audio_play_stream_buffer_get_spaces_size(audio_play_os_stream_t xStreamBuffer)
{
    return xStreamBufferSpacesAvailable(xStreamBuffer);
}


/**
 * @brief 从缓存流获取数据
 * 
 * @param xStreamBuffer 缓存流句柄
 * @param pvRxData 数据buf地址
 * @param xBufferLengthBytes 数据大小
 * @param xTicksToWait 等待tick数
 * @return size_t 读出大小
 */
size_t audio_play_stream_buffer_receive(audio_play_os_stream_t xStreamBuffer,
                                        void *pvRxData,
                                        size_t xBufferLengthBytes,
                                        uint32_t xTicksToWait)
{
    return xStreamBufferReceive(xStreamBuffer, pvRxData, xBufferLengthBytes, xTicksToWait);
}


/**
 * @brief 缓存流复位
 * 
 * @param xStreamBuffer 缓存流句柄 
 * @retval AUDIO_PLAY_OS_SUCCESS 复位成功
 * @retval AUDIO_PLAY_OS_FAIL 复位失败
 */
audio_play_os_state_t audio_play_stream_buffer_reset(audio_play_os_stream_t xStreamBuffer)
{
    if (pdFALSE == xStreamBufferReset(xStreamBuffer))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 查询缓存流是否为空
 * 
 * @param xStreamBuffer 缓存流句柄
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 缓存流已空
 * @retval AUDIO_PLAY_OS_FAIL 缓存流不空
 */
audio_play_os_state_t audio_play_stream_buffer_is_empty(audio_play_os_stream_t xStreamBuffer)
{
    if (pdFALSE == xStreamBufferIsEmpty(xStreamBuffer))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}


/**
 * @brief 申请内存malloc
 * 
 * @param xWantedSize 内存大小
 * @return void* 内存地址
 */
void *audio_play_malloc(size_t xWantedSize)
{
    return pvPortMalloc(xWantedSize);
}


/**
 * @brief 申请内存calloc
 * 
 * @param nmemb 内存大小
 * @param size 类型大小
 * @return void* 内存地址 
 */
void *audio_play_calloc(size_t nmemb, size_t size)
{
    return pvPortCalloc(nmemb, size);
}


/**
 * @brief 重新分配内存
 * 
 * @param pv 内存地址
 * @param size 新的大小
 * @return void* 内存地址
 */
void *audio_play_realloc(void *pv, size_t size)
{
    return pvPortRealloc(pv,size);
}


/**
 * @brief 释放内存
 * 
 * @param pv 内存地址
 */
void audio_play_free(void *pv)
{
    vPortFree(pv);
}

/**
 * @brief 创建事件标志组
 * 
 * @return audio_play_os_event_group_t 事件标志组句柄
 */
audio_play_os_event_group_t audio_play_event_group_create(void)
{
    return xEventGroupCreate();
}


/**
 * @brief 等待事件标志组bit置位
 * 
 * @param xEventGroup 事件标志组句柄
 * @param uxBitsToWaitFor 等待bit位
 * @param xClearOnExit 是否退出时清除bit位
 * @param xWaitForAllBits 是否等待所有要等待的位均发生
 * @param xTicksToWait 等待tick数
 * @return uint32_t 等待到的bit位
 */
uint32_t audio_play_event_group_wait_bits(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToWaitFor, const audio_play_os_state_t xClearOnExit, const audio_play_os_state_t xWaitForAllBits, uint32_t xTicksToWait)
{
    BaseType_t clear_on_exit = pdTRUE,wait_all_bit = pdTRUE;
    if(xClearOnExit == AUDIO_PLAY_OS_FAIL)
    {
        clear_on_exit = pdFALSE;
    }
    if(xWaitForAllBits == AUDIO_PLAY_OS_FAIL)
    {
        wait_all_bit = pdFALSE;
    }
    return xEventGroupWaitBits(xEventGroup, uxBitsToWaitFor, clear_on_exit, wait_all_bit, xTicksToWait);
}


/**
 * @brief 设置事件标志组bit位
 * 
 * @param xEventGroup 事件标志组句柄
 * @param uxBitsToSet 要设置的bit位
 * @return uint32_t 设置的bit位
 */
uint32_t audio_play_event_group_set_bits(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToSet)
{
    return xEventGroupSetBits(xEventGroup, uxBitsToSet);
}


/**
 * @brief 设置事件标志组bit位（中断安全）
 * 
 * @param xEventGroup 事件标志组句柄
 * @param uxBitsToSet 要设置的bit位
 * @param pxHigherPriorityTaskWoken 任务切换标志指针
 * 
 * @retval AUDIO_PLAY_OS_SUCCESS 设置成功
 * @retval AUDIO_PLAY_OS_FAIL 设置失败
 */
audio_play_os_state_t audio_play_event_group_set_bits_isr(audio_play_os_event_group_t xEventGroup, const uint32_t uxBitsToSet, void *pxHigherPriorityTaskWoken)
{
    if (pdFALSE == xEventGroupSetBitsFromISR(xEventGroup, uxBitsToSet, (BaseType_t *)pxHigherPriorityTaskWoken))
    {
        return AUDIO_PLAY_OS_FAIL;
    }
    return AUDIO_PLAY_OS_SUCCESS;
}
