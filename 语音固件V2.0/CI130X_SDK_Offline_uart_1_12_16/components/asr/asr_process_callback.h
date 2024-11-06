/**
  ******************************************************************************
  * @文件    asr_process_callbak.h
  * @版本    V1.0.1
  * @日期    2019-3-15
  * @概要  asr 回调函数，VAD相关
  ******************************************************************************
  * @注意
  *
  * 版权归chipintelli公司所有，未经允许不得使用或修改
  *
  ******************************************************************************
  */ 

#ifndef _ASR_PROCESS_CALLBACK_H_
#define _ASR_PROCESS_CALLBACK_H_


#ifdef __cplusplus
extern "C" {
#endif
  
#include "command_info.h"


int vadstart_callback(unsigned int* pdata ,int line);
int vadprocess_callback(unsigned int* pdata ,int line);
int vadend_callback(unsigned int* pdata,int line);
int computevad_callback(int asrpcmbuf_addr,int pcm_byte_size, short asrfrmshift,unsigned int asrpcmbuf_start_addr,unsigned int asrpcmbuf_end_addr);
int set_pcm_vad_mark_flag(short *pcm_data,int frame_len);



#ifdef __cplusplus
}
#endif


#endif

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
