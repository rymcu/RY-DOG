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
#include "asr_process_callback.h"
//#include "system_msg_deal.h"
// #include "voiceprint_port.h"

#include "ci_log.h"
#include "status_share.h"

#define DEBUG_ASR_NOT_PLAY (0)
#define VAD_PCM_MARK_EN (1)


//#include "asr_pcm_buf.h"


int set_pcm_vad_mark_flag(short *pcm_data,int frame_len)
{
    #if VAD_PCM_MARK_EN
    static int vad_start_marked = 0;//vad三帧数一算，此处为避免重复输出3次标签
    static int vad_end_marked = 0;
    
    status_t vad_state = ciss_get(CI_SS_VAD_STATE);//asr_vad 获取vad_start时刻
    //mprintf("vad_state = %d\n",vad_state);
    if((CI_SS_VAD_START == vad_state)&&(0 == vad_start_marked))
    {
        vad_start_marked = 1;
    	for(int i = 0;i < frame_len;i++)
        {
            pcm_data[i] = 30000;        
        }
        vad_end_marked = 0;
        
    }
    
    if((CI_SS_VAD_END == vad_state)&&(0 == vad_end_marked))
    {
        vad_end_marked = 1;
        
    	for(int i = 0;i < frame_len;i++)
        {
            pcm_data[i] = -30000;        
        }
        vad_start_marked = 0;
       
    }
    #endif

    return 0;
}

#define DEBUG_WRITE_VAD_FILE (0)

#if DEBUG_WRITE_VAD_FILE /*debug used*/
#include "ff.h"

static uint32_t mid_log_flag = 0;
static uint32_t voice_file_index = 1;

static FIL fp_result = NULL;

char voice_file_name[20];

void vad_result_open_new_file(void)
{
    FRESULT res;

    if (NULL != fp_result.fs)
    {
        ci_logerr(LOG_AUDIO_IN, "already %d\n", res);
        vPortEnterCritical();
        while (1)
            ;
    }

    MASK_ROM_LIB_FUNC->newlibcfunc.sprintf_p(voice_file_name, "vad_result_%d.pcm", voice_file_index);
    voice_file_index++;

    ci_logdebug(LOG_AUDIO_IN, "file new %s\n", voice_file_name);

    res = f_open(&fp_result, voice_file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        ci_logerr(LOG_AUDIO_IN, "fw open err %d\n", res);
        vPortEnterCritical();
        while (1)
            ;
    }

    ci_logdebug(LOG_AUDIO_IN, "file new done\n");
}

void vad_result_write_to_file(void *buf, uint32_t size)
{
    uint32_t lr;
    FRESULT res;

    if (NULL == fp_result.fs)
    {
        ci_logerr(LOG_AUDIO_IN, "file error %d\n", res);
        vPortEnterCritical();
        while (1)
            ;
    }

    res = f_write(&fp_result, (void *)buf, size, &lr);
    res = FR_OK;
    lr = size;
    if ((res != FR_OK) || (lr != size))
    {
        ci_logerr(LOG_AUDIO_IN, "f_write err %d\n", res);
    }
}

void vad_result_close_file(void)
{
    if (NULL == fp_result.fs)
    {
        ci_logerr(LOG_AUDIO_IN, "already\n");
        vPortEnterCritical();
        while (1)
            ;
    }

    f_close(&fp_result);
}

#endif


/*******************************************************************************
    *功能：vad start 函数调用
    *参数：pdata 数据指针  pdata[0]:vadstart-addr,pdata[1]:帧数
    *返回：0
    *注意：none
*******************************************************************************/
int vadstart_callback(unsigned int *pdata, int line)
{
    ciss_set(CI_SS_CWSL_OUTPUT_FLAG, 0);     //清除上异常CWSL识别输出标记。
    return 0;
}

/*******************************************************************************
    *功能：vad process 函数调用
    *参数：pdata 数据指针 pdata[0]:vadcur-addr,pdata[1]:frms
    *返回：0
    *注意：none
*******************************************************************************/
int vadprocess_callback(unsigned int *pdata, int line)
{
    return 0;
}

extern void vad_light_off(void);
/*******************************************************************************
    *功能：vad end 函数调用
    *参数：pdata 数据指针  pdata[0]:vadend-addr
            pdata[1]:vad end结束类型 0 硬件 vad end, 1 超时  vad end, 2 asr
             3,系统(释放asr,暂停asr等)
    *返回：0
    *注意：none
*******************************************************************************/
int vadend_callback(unsigned int *pdata, int line)
{
    // vad_light_off();
    return 0;
}


int computevad_callback(int asrpcmbuf_addr, int pcm_byte_size, short asrfrmshift, unsigned int asrpcmbuf_start_addr, unsigned int asrpcmbuf_end_addr)
{
    return 0;
}


/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
