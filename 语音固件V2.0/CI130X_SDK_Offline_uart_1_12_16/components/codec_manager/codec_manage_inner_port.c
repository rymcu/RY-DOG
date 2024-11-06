

#include "codec_manage_inner_port.h"
#include "ci_assert.h"
#include "ci130x_nuclear_com.h"
#include "codec_manager.h"
#include "ci130x_system.h"
#include "ci130x_audio_pre_rslt_out.h"

#define CHECK_PARA_NUM  (0)

#define CMI_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("CMI Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

#define CODEC_MANAGE_INNER_PORT_MAX_PARA_NUM    (6)
uint32_t codec_manage_inner_msg_buf[CODEC_MANAGE_INNER_PORT_MAX_PARA_NUM] = {0};

int32_t codec_manage_inner_serve_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    // if(payload_len > sizeof(codec_manage_inner_msg_buf))
    // {
    //     // mprintf("payload_len = %d\n",payload_len);
    //     CMI_CI_ASSERT(0,"\n");
    // }
    // MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)codec_manage_inner_msg_buf,(void*)payload,payload_len);
    uint32_t* data = (uint32_t*)payload;
    
    #if THIS_MODULE_PRINT_OPEN
    mprintf("NN I %d\n",data[0]);
    #endif
    switch(data[0])
    {
        case audio_pre_rslt_write_data_ept_num:
        {
            #if CHECK_PARA_NUM
            if(12 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                CMI_CI_ASSERT(0,"\n");
            }
            #endif
            //模块内部实现
            int16_t* left = (int16_t*)data[1];
            int16_t* right = (int16_t*)data[2];
            void audio_pre_rslt_write_data(int16_t* left,int16_t* right);
            audio_pre_rslt_write_data(left,right);
            break;
        }
        case cm_set_codec_adc_gain_ept_num:
        {
            #if CHECK_PARA_NUM
            if(16 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                CMI_CI_ASSERT(0,"\n");
            }
            #endif
            int codec_index = (int)data[1];
            cm_cha_sel_t cha = (cm_cha_sel_t)data[2];
            int gain = (int)data[3];
            cm_set_codec_adc_gain(codec_index,cha,gain);
            break;
        }
        case cm_set_codec_alc_ept_num:
        {
            #if CHECK_PARA_NUM
            if(16 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                CMI_CI_ASSERT(0,"\n");
            }
            #endif
            int codec_index = (int)data[1];
            cm_cha_sel_t cha = (cm_cha_sel_t)data[2];
            FunctionalState alc_enable = (FunctionalState)data[3];
            cm_set_codec_alc(codec_index,cha,alc_enable);
            break;
        }
        case cinn_malloc_in_host_sram_ept_num:
        {
            #if CHECK_PARA_NUM
            if(12 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                CMI_CI_ASSERT(0,"\n");
            }
            #endif
            size_t size = (int)data[1];
            uint32_t* ret_addr = (uint32_t)data[2];
            extern void *decoder_port_malloc(int size);
            uint32_t ret = (uint32_t)decoder_port_malloc(size);
            *ret_addr = ret;
            break;
        }
        case cinn_free_in_host_sram_ept_num:
        {
            {
            #if CHECK_PARA_NUM
            if(8 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                CMI_CI_ASSERT(0,"\n");
            }
            #endif
            void* pp = (void*)data[1];
            extern void decoder_port_free(void *pp);
            decoder_port_free(pp);
            break;
        }
        }
        default:
            break;
    }
    return 0;
}


void codec_manage_inner_port_init(void)
{
    nuclear_com_registe_serve(codec_manage_inner_serve_cb,codec_manage_serve_inner_ept_num);
}












