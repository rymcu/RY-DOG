/**
  ******************************************************************************
  * @文件    asr_process_callbak.c
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
#include <stdlib.h>
#include <stdio.h>

#include "sdk_default_config.h"
#include "platform_config.h"
#include "ci_log_config.h"
#include "asr_process_callback_decoder.h"
#include "system_msg_deal.h"
#include "status_share.h"
// #include "voiceprint_port.h"

#include "ci_log.h"


int send_asr_prediction_msg(uint32_t cmd_handle)
{
    return 0;
}

int send_asr_pre_cancel_msg(void)
{
    return 0;
}

int send_asr_pre_confirm_msg(uint32_t cmd_handle)
{
    return 0;
}

#if !USE_CWSL
void get_cwsl_threshold(unsigned char *wakeup_threshold,unsigned char *cmdword_threshold )
{}
#endif 

int asr_result_callback(callback_asr_result_type_t *asr)
{
    #if USE_CWSL
    extern int deal_cwsl_cmd(char * cmd_word,cmd_handle_t *cmd_handle,short *confidence,int unwakeup_flag,int *cwsl_flag);
    int unwakeup_flag = 0;
    if(SYS_STATE_UNWAKEUP == get_wakeup_state())
    {
        unwakeup_flag = 1;
    }
    int cwsl_flag ;
    int exit_flag = deal_cwsl_cmd(asr->cmd_word,&asr->cmd_handle,&asr->confidence,unwakeup_flag,&cwsl_flag);
    asr->cwsl_flag = cwsl_flag;
    if(exit_flag)
    {
        return 0;
    }
    #endif
    
    mprintf("send result:%s %d\n", asr->cmd_word, asr->confidence);

#if (!DEBUG_ASR_NOT_PLAY)
    if (INVALID_HANDLE != asr->cmd_handle)
    {
        #if (USE_AEC_MODULE && !ONE_SHOT_ENABLE)
        if (!(cmd_info_is_wakeup_word(asr->cmd_handle)))
        {
            if (ciss_get(CI_SS_INTERCEPT_ASR_OUT))
            {
                return 0;
            }
        }
        #endif

        sys_msg_t send_msg;
        send_msg.msg_type = SYS_MSG_TYPE_ASR;
        send_msg.msg_data.asr_data.asr_status = MSG_ASR_STATUS_GOOD_RESULT;
        send_msg.msg_data.asr_data.asr_cmd_handle = asr->cmd_handle;
        mprintf("asr->cmd_handle is %x\n",(asr->cmd_handle));
        send_msg.msg_data.asr_data.asr_score = asr->confidence;
        send_msg.msg_data.asr_data.asr_pcm_base_addr = asr->asrvoice_ptr;
        send_msg.msg_data.asr_data.asr_frames = asr->vocie_valid_frame_len;
        send_msg_to_sys_task(&send_msg, NULL);
    }
    else
    {
        mprintf("asr->cmd_handle is INVALID_HANDLE\n");
    }
#endif
    return 0;
}


int asr_lite_result_callback(char * words,short cfd)
{
    //mprintf("out2 :%s %d\n",words,cfd);
    /*add code for app*/
    
    return 0;
}


/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
