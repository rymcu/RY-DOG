/**
  ******************************************************************************
  * @file    flash_control_outside_port.h
  * @version V1.0.0
  * @date    2021.04.04
  * @brief 
  ******************************************************************************
  */

#ifndef __FLASH_CONTROL_OUTSIDE_PORT_H_
#define __FLASH_CONTROL_OUTSIDE_PORT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


uint32_t get_current_model_addr(uint32_t *p_dnn_addr, uint32_t *p_dnn_size, uint32_t *p_asr_addr, uint32_t *p_addr_size);
void is_ci_flash_data_info_inited(bool* state);

#ifdef __cplusplus
}
#endif

#endif
