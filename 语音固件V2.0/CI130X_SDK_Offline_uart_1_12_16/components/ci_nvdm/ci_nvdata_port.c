/**
 * @file ci_nvdata_port.c
 * @brief  
 * @version 0.1
 * @date 2019-05-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include "ci_nvdata_port.h"
#include "ci_nvdata_manage.h"

#include "ci130x_spiflash.h"
#include "ci130x_uart.h"

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "flash_manage_outside_port.h"

static SemaphoreHandle_t cinv_mutex;


void cinv_port_mutex_creat(void)
{
    cinv_mutex = xSemaphoreCreateMutex();

    if (NULL == cinv_mutex) 
    {
        cinv_log_error("%s, error\n",__func__);
    }
    // cinv_log_debug("%s\n",__func__);
}


void cinv_port_mutex_take(void)
{
    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) 
    {
        if (xSemaphoreTake(cinv_mutex, portMAX_DELAY) == pdFALSE)
        {
            cinv_log_error("%s, error\n",__func__);
        }
        // cinv_log_debug("%s\n",__func__);
    }
}


void cinv_port_mutex_give(void)
{
    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) 
    {
        if (xSemaphoreGive(cinv_mutex) == pdFALSE) 
        {
            cinv_log_error("%s, error\n",__func__);
        }
        // cinv_log_debug("%s\n",__func__);
    }
}


void *cinv_port_malloc(uint32_t size)
{
    return pvPortMalloc(size);
}


void cinv_port_free(void *pdata)
{
    vPortFree(pdata);
}


/**
 * @brief
 *
 * @param address
 * @param buffer
 * @param length
 */
void cinv_port_flash_read(uint32_t address, uint8_t *buffer, uint32_t length)
{
    // flash_read(QSPI0, (uint32_t)buffer, address, length);
    // cinv_log_debug("cinv_port_flash_read: address = 0x%x, buffer = 0x%x, length = %d\n", address, (uint32_t)buffer, length);
    post_read_flash((char*)buffer, address, length);
}


void cinv_port_flash_write(uint32_t address, const uint8_t *buffer, uint32_t length)
{
    // flash_write(QSPI0, address, (uint32_t)buffer, length);
    post_write_flash((uint32_t)buffer, (char*)address, length);

    // cinv_log_debug("cinv_port_flash_write: address = 0x%x, buffer = 0x%x, length = %d\n", address, (uint32_t)buffer, length);
}


void cinv_port_flash_erase(uint32_t address)
{
    /*here must be casefull, erase size must be same as erase block size*/
    // flash_erase(QSPI0, address, CINV_FLASH_ERASE_BLOCK_SIZE);
    post_erase_flash(address, CINV_FLASH_ERASE_BLOCK_SIZE);

    cinv_log_debug("cinv_port_flash_erase: address = 0x%x\n", address);
}


/* also can do this in flash_read/flash_write */
void cinv_port_flash_protect(functional_state_t fun_en)
{
    if(F_ENABLE == fun_en)
    {
        
    }
    else
    {

    }
}


/*if power is low, maybe flash operate will cause error more than normal,or frequency need low*/
bool cinv_port_power_check(void)
{
    return true;
}


void cinv_port_flash_used_request(void)
{
    // if (0 != requset_flash_ctl())
    // {
    //     while(1);
    // }
}


void cinv_port_flash_used_release(void)
{
    // if(0 != release_flash_ctl())
    // {
    //     while(1);
    // }
}


