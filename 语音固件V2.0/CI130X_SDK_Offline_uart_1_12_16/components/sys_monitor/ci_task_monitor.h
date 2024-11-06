/**
 * @file ci_task_monitor.h
 * @brief  系统监控组件
 *
 * 负责监视控制加入监控队列里的每一个任务，以防有任务出现异常情况，
 * 避免因异常未及时处理而造成的损失。
 *
 * @version 0.1
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "ci130x_iwdg.h"
#include "ci130x_scu.h"
#include "sdk_default_config.h"
#include "ci_log.h"
#include <math.h>

/**
 * @addtogroup sys_monitor
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

void monitor_creat(void (*call_back)(void));
void join_monitor(uint8_t* id,uint32_t time_ms, TaskHandle_t handle);
void exit_monitor(uint8_t id);
void task_alive(uint8_t id);
void task_monitor(void *pvparameters);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
