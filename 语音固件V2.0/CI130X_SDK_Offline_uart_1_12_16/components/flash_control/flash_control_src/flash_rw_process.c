// /**
//  * @file flash_rw_process.c
//  * @brief 用于统一管理软件读写flash，避免同硬件读写flash冲突
//  * @version 2.0
//  * @date 2018-07-10
//  * 
//  * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
//  * 
//  */
// #include <stdbool.h>
// #include "flash_rw_process.h"
// // #include "nn_and_flash_manage.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"


// // #include "asr_api.h"

// #include "ci130x_spiflash.h"
// #include "audio_play_process.h"

// static bool before_asr_run = true;
// SemaphoreHandle_t dnn_cpu_mode_semaphore = NULL;
// SemaphoreHandle_t dnn_dnn_mode_semaphore = NULL;
// SemaphoreHandle_t flash_ctl_xSemaphore = NULL;

// /**
//  * @brief 设置asr将要启动，之后的flash操作需要通知到dnn
//  * 
//  */
// void set_asr_run_flag(void)
// {
//     before_asr_run = false;
// }

// /**
//  * @brief 设置asr停止，之后的flash操作不需要通知到dnn
//  * 
//  */
// void set_asr_stop_flag(void)
// {
//     before_asr_run = true;
// }


// /**
//  * @brief 初始化flash管理任务
//  * @note this init before task run,maybe other task used these resource
//  * 
//  */
// int32_t flash_ctl_init(void)
// {
//     mprintf("%s %d\n",__FUNCTION__,__LINE__);
//     flash_ctl_xSemaphore = xSemaphoreCreateMutex();
//     if(NULL == flash_ctl_xSemaphore)
//     {
//         ci_logerr(LOG_FLASH_CTL,"flash_ctl_xSemaphore creat fail\r\n");   
//         return RETURN_ERR;
//     }
    
//     dnn_cpu_mode_semaphore = xSemaphoreCreateBinary();
//     if(NULL == dnn_cpu_mode_semaphore)
//     {
//         ci_logerr(LOG_FLASH_CTL,"dnn_cpu_mode_semaphore creat fail\r\n");   
//         return RETURN_ERR;
//     }
//     dnn_dnn_mode_semaphore = xSemaphoreCreateBinary();
//     if(NULL == dnn_dnn_mode_semaphore)
//     {
//         ci_logerr(LOG_FLASH_CTL,"dnn_dnn_mode_semaphore creat fail\r\n");   
//         return RETURN_ERR;
//     }

//     return RETURN_OK;
// }


// /**
//  * @brief 请求控制flash
//  * 
//  * @retval RETURN_OK 请求成功
//  * @retval RETURN_ERR 请求失败
//  */
// int32_t requset_flash_ctl(void)
// {
//        mprintf("%s %d\n",__FUNCTION__,__LINE__);
//     if(before_asr_run)
//     {
//         return RETURN_OK;
//     }
    
//     xSemaphoreTake(flash_ctl_xSemaphore,portMAX_DELAY);
//     //ci_loginfo(CI_LOG_INFO,"get flash_ctl_xSemaphore\n");

// 	// extern audio_play_os_task_t audio_play_task_handle;
// 	// taskENTER_CRITICAL();
// 	// if(xTaskGetCurrentTaskHandle() != audio_play_task_handle)
// 	// {
// 	// 	vTaskSuspend((TaskHandle_t)audio_play_task_handle);
// 	// }
// 	// taskEXIT_CRITICAL();

// 	// if(-1 == send_requset_flash_msg_to_dnn())
// 	// {
// 	// 	CI_ASSERT(0, "send requset flash msg err!\n");
// 	// }
	
//     /* 等待DNN解除FLASH占用 */
//     if(pdPASS == xSemaphoreTake(dnn_cpu_mode_semaphore,pdMS_TO_TICKS(5000)))
//     {
//         return RETURN_OK;
//     }
//     else
//     {
//         CI_ASSERT(0,"take flash ctrl timeout!\n");
//         //return RETURN_ERR;
//     }
// }


// /**
//  * @brief 解除控制flash
//  * 
//  * @retval RETURN_OK 解除成功
//  * @retval RETURN_ERR 解除失败
//  */
// int32_t release_flash_ctl(void)
// {
//     int32_t ret = RETURN_ERR;
//     mprintf("%s %d\n",__FUNCTION__,__LINE__);
	
// 	if(before_asr_run)
//     {
//         return RETURN_OK;
//     }
	
// 	// if(-1 == send_release_flash_semaphore_to_dnn())
// 	// {
// 	// 	ret = RETURN_ERR;
// 	// }
//     // else
//     // {
//     //     ret = RETURN_OK;
//     // }
	
	
//     // 等待回到dnn模式
// 	if(pdPASS == xSemaphoreTake(dnn_dnn_mode_semaphore,pdMS_TO_TICKS(5000)))
//     {
// 		ret = RETURN_OK;
//     }
//     else
//     {
//         CI_ASSERT(0,"take flash ctrl timeout!\n");
//     }


// 	// extern audio_play_os_task_t audio_play_task_handle;
// 	// taskENTER_CRITICAL();
// 	// if(xTaskGetCurrentTaskHandle() != audio_play_task_handle)
// 	// {
// 	// 	vTaskResume((TaskHandle_t)audio_play_task_handle);
// 	// }
// 	// taskEXIT_CRITICAL();

//     xSemaphoreGive(flash_ctl_xSemaphore);
//     return ret;
// }



// /**
//  * @brief 请求读flash
//  * 
//  * @param buf 数据buff
//  * @param addr 数据地址
//  * @param size 数据大小
//  * @retval RETURN_OK 读取成功
//  * @retval RETURN_ERR 读取失败
//  */
// // int32_t post_read_flash(char *buf, uint32_t addr, uint32_t size)
// // {
// // 	int ret = RETURN_ERR;

// //     // requset_flash_ctl();
// //     mprintf("%s %d\n",__FUNCTION__,__LINE__);

// //     ci_logdebug(LOG_FLASH_CTL,"FLASH R Start!\n");
// //     ci_logdebug(LOG_FLASH_CTL,"addr = 0x%x ,buf = 0x%x ,size= 0x%x \n",addr,buf,size);
// //     // ret = flash_read(QSPI0, (unsigned int)buf,addr,size);
// //     MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)buf,0x50000000+addr,size);

// //     // release_flash_ctl();

// //     return ret;
// // }


// /**
//  * @brief 请求写flash
//  * 
//  * @param buf 数据buff
//  * @param addr 数据地址
//  * @param size 数据大小
//  * @retval RETURN_OK 读取成功
//  * @retval RETURN_ERR 读取失败
//  */
// int32_t post_write_flash(char *buf, uint32_t addr, uint32_t size)
// {
// 	int ret = RETURN_ERR;
//     mprintf("%s %d\n",__FUNCTION__,__LINE__);

//     requset_flash_ctl();

//     ci_logdebug(LOG_FLASH_CTL,"FLASH W Start!\n");
//     ci_logdebug(LOG_FLASH_CTL,"addr = 0x%x ,buf = 0x%x ,size= 0x%x \n",addr,buf,size);
//     ret = flash_write(QSPI0,addr,(uint32_t)buf, size);

//     release_flash_ctl();

//     return ret;
// }


// /**
//  * @brief 请求擦除flash
//  * 
//  * @param addr 数据地址
//  * @param size 数据大小
//  * @retval RETURN_OK 擦除成功
//  * @retval RETURN_ERR 擦除失败
//  */
// int32_t post_erase_flash(uint32_t addr, uint32_t size)
// {
// 	int ret = RETURN_ERR;
//     mprintf("%s %d\n",__FUNCTION__,__LINE__);
//     requset_flash_ctl();

//     ci_logdebug(LOG_FLASH_CTL,"FLASH E Start!\n");
//     ci_logdebug(LOG_FLASH_CTL,"addr = 0x%x, size= 0x%x \n",addr,size);
//     ret = flash_erase(QSPI0,addr,size);

//     release_flash_ctl();

//     return ret;
// }
