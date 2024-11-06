#ifndef __CI130X_AUDIO_PRE_RSLT_OUT_H
#define __CI130X_AUDIO_PRE_RSLT_OUT_H

#include "codec_manager.h"
#include "ci130x_gpio.h"
#include "ci130x_dpmu.h"
#include "ci130x_iis.h"
#include "ci130x_scu.h"
#include <stdbool.h>
#ifdef __cplusplus
 extern "C" {
#endif


typedef struct
{
    //!codec的主从模式
    // iis_mode_sel_t mode;
    //!数据格式
    // iis_data_format_t data_format;
    //!block size（发送多少字节的数据来一次中断，在中断中需填写下一帧发送的数据的起始地址）
    int32_t block_size;
    //!SCK和LRCK时钟频率的比值
    // IIS_SCK_LRCK_WID_t sck_lrck_rate;
}audio_pre_rslt_out_init_t;


void audio_pre_rslt_out_play_card_init(void);
void audio_pre_rslt_write_data(int16_t* rslt,int16_t* origin);
void audio_pre_rslt_stop(void);
void audio_pre_rslt_start(void);
/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/





