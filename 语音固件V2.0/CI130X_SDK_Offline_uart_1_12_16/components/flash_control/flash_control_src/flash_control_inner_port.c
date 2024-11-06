

#include "flash_control_inner_port.h"
#include "ci_assert.h"
#include "ci130x_nuclear_com.h"


#define CHECK_PARA_NUM  (0)

#define FCI_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("FCI Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

#define FLASH_CONTROL_INNER_PORT_MAX_PARA_NUM    (6)
uint32_t flash_control_inner_msg_buf[FLASH_CONTROL_INNER_PORT_MAX_PARA_NUM] = {0};

int32_t flash_control_inner_serve_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    // if(payload_len > sizeof(flash_control_inner_msg_buf))
    {
        // mprintf("payload_len = %d\n",payload_len);
        // FCI_CI_ASSERT(0,"\n");
    }
    // MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)flash_control_inner_msg_buf,(void*)payload,payload_len);
    uint32_t* data = (uint32_t*)payload;

    bool state;
    extern void is_ci_flash_data_info_inited(bool* state);
    is_ci_flash_data_info_inited(&state);
    if(!state)
    {
        if(data[0] != is_ci_flash_data_info_inited_ept_num)
        {
            FCI_CI_ASSERT(0,"didn't inti\n");
        }
    }
    

    #if THIS_MODULE_PRINT_OPEN
    mprintf("NN I %d\n",data[0]);
    #endif
    switch(data[0])
    {
        //nuclear com修改第二步，在inner端添加实现这个接口需要的操作
        case get_current_model_addr_ept_num:
        {
            #if CHECK_PARA_NUM
            if(20 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FCI_CI_ASSERT(0,"\n");
            }
            #endif
            //模块内部实现
            uint32_t* p_dnn_addr = (uint32_t*)data[1];
            uint32_t* p_dnn_size = (uint32_t*)data[2];
            uint32_t* p_asr_addr = (uint32_t*)data[3];
            uint32_t* p_addr_size = (uint32_t*)data[4];
            get_current_model_addr(p_dnn_addr,p_dnn_size,p_asr_addr,p_addr_size);
            break;
        }
        case is_ci_flash_data_info_inited_ept_num:
        {
            #if CHECK_PARA_NUM
            if(8 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FCI_CI_ASSERT(0,"\n");
            }
            #endif
            //模块内部实现
            bool* state = (bool*)data[1];
            is_ci_flash_data_info_inited(state);
            break;
        }
        default:
            break;
    }
    return 0;
}


void flash_control_inner_port_init(void)
{
    nuclear_com_registe_serve(flash_control_inner_serve_cb,flash_control_serve_inner_ept_num);
}












