

#include "flash_control_outside_port.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "ci130x_nuclear_com.h"
#include "ci130x_system_ept.h"



//para[0]表示的是需要调用的函数编号，asr_dnn_rpmsg_ept_num_t中定义
//para[1]，直到后面的一些参数都是传递的函数参数
static void flash_control_ept_cal_serve(char* para,uint32_t size)
{
    nuclear_com_t str;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void*)&str,0,sizeof(str));
    str.src_ept_num = 0;
    str.dst_ept_num = flash_control_serve_inner_ept_num;//修改
    str.data_p = (void*)para;
    str.data_len = size;
    nuclear_com_send(&str,0xfffff);
}


//TEST OK
//没解码一帧数据就会调一次
//用于检测check_dnn_outbuf_isoverlap
//可优化
//nuclear com修改第一步，添加需要的接口
// uint32_t get_current_model_addr(uint32_t *p_dnn_addr, uint32_t *p_dnn_size, uint32_t *p_asr_addr, uint32_t *p_addr_size)
// {
//     //大概70us完成
//     #if 1
//     #if THIS_MODULE_PRINT_OPEN
//     mprintf("A %d:%d\n",__LINE__,set_g_asrtop_cursegment_decoderinbuf_nums_cal_ept_num);
//     #endif
//     uint32_t para[5];
//     para[0] = get_current_model_addr_ept_num;
//     para[1] = (uint32_t)p_dnn_addr;
//     para[2] = (uint32_t)p_dnn_size;
//     para[3] = (uint32_t)p_asr_addr;
//     para[4] = (uint32_t)p_addr_size;
    
//     flash_control_ept_cal_serve((char*)para,sizeof(para));
//     #if THIS_MODULE_PRINT_OPEN
//     mprintf("A END %d\n",__LINE__);
//     #endif
//     #else
//     extern void set_g_asrtop_cursegment_decoderinbuf_nums(int p);
//     set_g_asrtop_cursegment_decoderinbuf_nums(p);
//     #endif
// }


void is_ci_flash_data_info_inited(bool* state)
{
    #if 1
    #if THIS_MODULE_PRINT_OPEN
    mprintf("A %d:%d\n",__LINE__,is_ci_flash_data_info_inited_ept_num);
    #endif
    uint32_t para[2];
    para[0] = is_ci_flash_data_info_inited_ept_num;
    para[1] = (uint32_t)state;
    
    flash_control_ept_cal_serve((char*)para,sizeof(para));
    #if THIS_MODULE_PRINT_OPEN
    mprintf("A END %d\n",__LINE__);
    #endif
    #else
    extern void set_g_asrtop_cursegment_decoderinbuf_nums(int p);
    set_g_asrtop_cursegment_decoderinbuf_nums(p);
    #endif
}


void flash_control_nuclear_com_outside_port_init(void)
{
    
}

