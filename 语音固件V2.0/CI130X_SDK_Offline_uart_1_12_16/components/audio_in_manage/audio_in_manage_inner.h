/**
  ******************************************************************************
  * @file    audio_in_manage.h
  * @version V1.0.0
  * @date    2019.04.04
  * @brief 
  ******************************************************************************
  */

#ifndef _VOICE_IN_MANAGE_INNER_H_
#define _VOICE_IN_MANAGE_INNER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef void (*get_fft_rslt_fun_t)(float* ,int);
typedef void (*get_voice_energy_fun_t)(float);


typedef struct 
{
    int32_t cha_num;//通道数
    uint32_t channel_flag; /*!< 声道选择，bit[0] = 1选择左声道，bit[1]=1选择右声道，可以用或运算组合 */
    uint32_t addr1;//最多支持两个声道，两个声道的地址
    uint32_t addr2;
    uint32_t size; 
}audio_in_get_voice_t;


void audio_in_manage_inner_task(void* p);

void set_get_fft_rslt_fun_callback(get_fft_rslt_fun_t fun);

#ifdef __cplusplus
}
#endif

#endif
