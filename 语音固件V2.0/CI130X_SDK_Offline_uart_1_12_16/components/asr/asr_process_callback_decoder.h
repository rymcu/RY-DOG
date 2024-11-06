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
  
typedef struct
{
    char* cmd_word;
    cmd_handle_t cmd_handle;
    unsigned int asrvoice_ptr;
    short confidence;
    short vocie_valid_frame_len;
    short voice_start_frame;
    short start_word_confidence;
    short end_word_confidence;
    short lowest_confidence;
    char cwsl_flag;
}callback_asr_result_type_t;


int asr_result_callback(callback_asr_result_type_t * asr);


#ifdef __cplusplus
}
#endif


#endif

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
