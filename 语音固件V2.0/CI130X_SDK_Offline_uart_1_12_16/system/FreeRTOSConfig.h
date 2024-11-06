/*
 * FreeRTOS Kernel V10.1.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/
#ifndef __ASSEMBLER__
#include "platform_config.h"
#include "ci_assert.h"
extern char __FREERTOSHEAP_SIZE;
// #include "ci_debug_config.h"
#if CONFIG_SYSTEMVIEW_EN
#include "SEGGER_SYSVIEW_FreeRTOS.h"
#endif 
#endif


#define configUSE_PREEMPTION                            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION         0
#define configUSE_TICKLESS_IDLE                         0
#define configCPU_CLOCK_HZ                              ((uint32_t)get_ipcore_clk())
#define configRTC_CLOCK_HZ						        ((uint32_t)get_systick_clk())
#define configTICK_RATE_HZ                              ( ( uint32_t ) 500)
#define configMAX_PRIORITIES                            ( 6 )  //0 - 3 共6等级，idle独占0，Tmr_svc独占3
#define configMINIMAL_STACK_SIZE                        ( ( unsigned short ) 160 )
#define configMAX_TASK_NAME_LEN                         16
#define configUSE_16_BIT_TICKS                          0
#define configIDLE_SHOULD_YIELD                         1
#define configUSE_TASK_NOTIFICATIONS                    1
#define configUSE_MUTEXES                               1
#define configUSE_RECURSIVE_MUTEXES                     0
#define configUSE_COUNTING_SEMAPHORES                   1
#define configQUEUE_REGISTRY_SIZE                       10
#define configUSE_QUEUE_SETS                            0
#define configUSE_TIME_SLICING                          1
#define configUSE_NEWLIB_REENTRANT                      0
#define configENABLE_BACKWARD_COMPATIBILITY             1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         5

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION                 0
#define configSUPPORT_DYNAMIC_ALLOCATION                1
#define configTOTAL_HEAP_SIZE                  	        ((int)&__FREERTOSHEAP_SIZE)
#define configAPPLICATION_ALLOCATED_HEAP                0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                             0
#define configUSE_TICK_HOOK                             0
#define configCHECK_FOR_STACK_OVERFLOW                  0
#define configUSE_MALLOC_FAILED_HOOK                    0
#define configUSE_DAEMON_TASK_STARTUP_HOOK              0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS                   0
#define configUSE_TRACE_FACILITY                        1
#define configUSE_STATS_FORMATTING_FUNCTIONS            1

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 1

/* Software timer related definitions. */
#define configUSE_TIMERS                                1
#define configTIMER_TASK_PRIORITY                       ( configMAX_PRIORITIES - 1 ) //Tmr_svc 独占最高优先级
#define configTIMER_QUEUE_LENGTH                        5
#define configTIMER_TASK_STACK_DEPTH                    configMINIMAL_STACK_SIZE

/* Interrupt nesting behaviour configuration. */
#define configPRIO_BITS       		                    (3UL)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0x1
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	0x6
#define configKERNEL_INTERRUPT_PRIORITY                 ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Define to trap errors during development. */
//#define configASSERT( x )                               CI_ASSERT(x, "FreeRTOS ASSERT\n")

/* A header file that defines trace macro can be included here. */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE               1024

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                        1
#define INCLUDE_uxTaskPriorityGet                       1
#define INCLUDE_vTaskDelete                             1
#define INCLUDE_vTaskSuspend                            1
#define INCLUDE_xResumeFromISR                          1
#define INCLUDE_vTaskDelayUntil                         1
#define INCLUDE_vTaskDelay                              1
#define INCLUDE_xTaskGetSchedulerState                  1
#define INCLUDE_xTaskGetCurrentTaskHandle               1
#define INCLUDE_uxTaskGetStackHighWaterMark             1
#define INCLUDE_xTaskGetIdleTaskHandle                  1
#define INCLUDE_eTaskGetState                           0
#define INCLUDE_xEventGroupSetBitFromISR                1
#define INCLUDE_xTimerPendFunctionCall                  1
#define INCLUDE_xTaskAbortDelay                         0
#define INCLUDE_xTaskGetHandle                          1
#define INCLUDE_xTaskResumeFromISR                      1

#endif /* FREERTOS_CONFIG_H */
