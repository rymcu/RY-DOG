/**
 * @file ci_nvdata_port.h
 * @brief  
 * @version 0.1
 * @date 2019-05-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */


#ifndef _CI_NVDATA_PORT_H_
#define _CI_NVDATA_PORT_H_

#include <stdbool.h>
#include "sdk_default_config.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum 
{
    F_DISABLE = 0, 
    F_ENABLE = !F_DISABLE
}functional_state_t;

void cinv_port_mutex_creat(void);
void cinv_port_mutex_take(void);
void cinv_port_mutex_give(void);

void *cinv_port_malloc(uint32_t size);
void cinv_port_free(void *pdata);

void cinv_port_flash_read(uint32_t address, uint8_t *buffer, uint32_t length);
void cinv_port_flash_write(uint32_t address, const uint8_t *buffer, uint32_t length);
void cinv_port_flash_erase(uint32_t address);

void cinv_port_flash_protect(functional_state_t fun_en);

void cinv_port_flash_used_request(void);

void cinv_port_flash_used_release(void);


bool cinv_port_power_check(void);


      
#if USE_STD_PRINTF
#define cinv_log_debug printf
// #define cinv_log_debug(fmt, args...) do{}while(0)
#define cinv_log_info printf
#define cinv_log_error printf
#else
// #define cinv_log_debug _printf
#define cinv_log_debug(fmt, args...) do{}while(0)
#define cinv_log_info _printf
#define cinv_log_error _printf
#endif
//#define cinv_assert(x) do{}while(0 == (x))
#define cinv_assert(x) do{}while(0)



#ifdef __cplusplus
}
#endif

#endif

