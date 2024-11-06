/**
 * @file ci_system_info.c
 * @brief 获取堆栈、任务等信息接口
 * @version 1.0
 * @date 2019-02-21
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include "ci_system_info.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>
#include "ci_log.h"

#if defined(__GNUC__)
#include <malloc.h>
#endif

/**
 * @brief 获取HEAP使用信息
 * 
 */
void get_mem_status(void)
{
    #if defined(__GNUC__)
	extern int get_heap_bytes_remaining_size(void);
	struct mallinfo mi = mallinfo();/*可以查看动态分配内存的总大小*/
    ci_logdebug(LOG_SYS_INFO,"HEAP alloc: %d free %d\n",mi.uordblks, mi.fordblks + get_heap_bytes_remaining_size());
    #endif
}


/**
 * @brief 获取freertos heap使用信息
 * 
 */
void get_fmem_status(void)
{
    size_t fheap_meminfo = xPortGetFreeHeapSize();
    ci_logdebug(LOG_SYS_INFO,"freeRTOS HEAP alloc: %d free:%d\n",configTOTAL_HEAP_SIZE-fheap_meminfo,fheap_meminfo);
}

/**
 * @brief 获取任务状态
 * 
 */
void get_task_status(void)
{
    int task_curr_num = uxTaskGetNumberOfTasks();
    char *task_info = (char *)pvPortMalloc( task_curr_num * sizeof( TaskStatus_t ) );

    vTaskList(task_info);
    ci_logdebug(LOG_SYS_INFO,"TaskName\tStatus\tPRI\tStack\tTaskNumber\r\n");
    for(int i=0;i < task_curr_num*sizeof( TaskStatus_t ) ; i+=UART_LOG_BUFF_SIZE)
    {
        ci_logdebug(LOG_SYS_INFO,"%s",task_info+i);
    }
    vPortFree(task_info);
}

#if 0
/**
 * @brief 获取任务运行时间
 * @note 需要设置高精度定时器
 * 
 */
void get_task_runtime(void)
{
    int task_curr_num = uxTaskGetNumberOfTasks();
    char *task_info = (char *)pvPortMalloc( task_curr_num * sizeof( TaskStatus_t ) );

    vTaskGetRunTimeStats(task_info);
    ci_logdebug(LOG_SYS_INFO,"%s\n\r",task_info);

    vPortFree(task_info);
}
#endif
