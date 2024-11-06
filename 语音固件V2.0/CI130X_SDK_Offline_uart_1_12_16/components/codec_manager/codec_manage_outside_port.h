/**
  ******************************************************************************
  * @file    codec_manage_outside_port.h
  * @version V1.0.0
  * @date    2021.04.04
  * @brief 
  ******************************************************************************
  */

#ifndef __CODEC_MANAGE_OUTSIDE_PORT_H_
#define __CODEC_MANAGE_OUTSIDE_PORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "codec_manager.h"
#include "ci130x_system.h"

#ifdef __cplusplus
extern "C" {
#endif

int cm_write_codec(int codec_index, void * pcm_buffer,uint32_t wait_tick);
void cm_get_pcm_buffer(int codec_index,uint32_t* ret_buf,uint32_t wait_ticks);
int cm_release_pcm_buffer(int codec_index, io_direction_t io_dir, void * pcm_buffer);
int cm_set_codec_mute(int codec_index, io_direction_t io_dir, int channel_flag, FunctionalState en);
int cm_get_codec_empty_buffer_number(int codec_index, io_direction_t io_dir);
int cm_get_codec_busy_buffer_number(int codec_index, io_direction_t io_dir);
int cm_set_codec_alc(int codec_index, cm_cha_sel_t cha, FunctionalState alc_enable);
int cm_set_codec_adc_gain(int codec_index, cm_cha_sel_t cha, int gain);

void audio_pre_rslt_write_data(int16_t* left,int16_t* right);

#ifdef __cplusplus
}
#endif

#endif
