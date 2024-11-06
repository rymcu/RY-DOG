

#include "codec_manage_outside_port.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "ci130x_nuclear_com.h"
#include "ci130x_system_ept.h"
#include "ci130x_system.h"


//para[0]表示的是需要调用的函数编号，asr_dnn_rpmsg_ept_num_t中定义
//para[1]，直到后面的一些参数都是传递的函数参数
static void codec_manage_ept_cal_serve(char* para,uint32_t size)
{
    nuclear_com_t str;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void*)&str,0,sizeof(str));
    str.src_ept_num = 0;
    str.dst_ept_num = codec_manage_serve_inner_ept_num;//修改
    str.data_p = (void*)para;
    str.data_len = size;
    nuclear_com_send(&str,0xfffff);
}


int cm_start_codec(int codec_index, io_direction_t io_dir)
{
    uint32_t para[3];
    para[0] = cm_start_codec_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)io_dir;
    
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return 0;
}


int cm_set_codec_mute(int codec_index, io_direction_t io_dir, int channel_flag, FunctionalState en)
{
    uint32_t para[5];
    para[0] = cm_set_codec_mute_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)io_dir;
    para[3] = (uint32_t)channel_flag;
    para[4] = (uint32_t)en;
    
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return 0;
}


int cm_stop_codec(int codec_index, io_direction_t io_dir)
{
    uint32_t para[3];
    para[0] = (uint32_t)cm_stop_codec_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)io_dir;
    
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return 0;
}


int cm_get_codec_empty_buffer_number(int codec_index, io_direction_t io_dir)
{
    int ret;
    uint32_t para[4];
    para[0] = (uint32_t)cm_get_codec_empty_buffer_number_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)io_dir;
    para[3] = (uint32_t)&ret;
    
    codec_manage_ept_cal_serve((char*)para, (uint32_t)(sizeof(para)));
    return ret;
}


int cm_get_codec_busy_buffer_number(int codec_index, io_direction_t io_dir)
{
    int ret;
    uint32_t para[4];
    para[0] = cm_get_codec_busy_buffer_number_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)io_dir;
    para[3] = (uint32_t)&ret;
    
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return ret;
}


int cm_set_codec_alc(int codec_index, cm_cha_sel_t cha, FunctionalState alc_enable)
{
    uint32_t para[4];
    para[0] = cm_set_codec_alc_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)cha;
    para[3] = (uint32_t)alc_enable;
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
}


int cm_set_codec_adc_gain(int codec_index, cm_cha_sel_t cha, int gain)
{
    uint32_t para[4];
    para[0] = cm_set_codec_adc_gain_ept_num;
    para[1] = (uint32_t)codec_index;
    para[2] = (uint32_t)cha;
    para[3] = (uint32_t)gain;
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
}



void audio_pre_rslt_write_data(int16_t* left,int16_t* right)
{
    uint32_t para[3];
    para[0] = audio_pre_rslt_write_data_ept_num;
    para[1] = (uint32_t)left;
    para[2] = (uint32_t)right;
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
}


void *cinn_malloc_in_host_sram(int xWantedSize) 
{
    uint32_t ret;
    uint32_t para[3];
    para[0] = cinn_malloc_in_host_sram_ept_num;
    para[1] = (uint32_t)xWantedSize;
    para[2] = (uint32_t)&ret;
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return ret;
}


void cinn_free_in_host_sram( void *pv )
{
    uint32_t ret;
    uint32_t para[2];
    para[0] = cinn_free_in_host_sram_ept_num;
    para[1] = (uint32_t)pv;
    codec_manage_ept_cal_serve((char*)para,sizeof(para));
    return ret;
}



void codec_manage_nuclear_com_outside_port_init(void)
{
    
}

