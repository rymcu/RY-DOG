/**
 * @file status_share_bnpu.c
 * @brief 示例程序
 * @version 1.0.0
 * @date 2021-12-16
 *
 * @copyright Copyright (c) 2021  Chipintelli Technology Co., Ltd.
 *
 */
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h" 
#include "event_groups.h"
#include "status_share.h"
#include "ci130x_system_ept.h"
#include "ci130x_nuclear_com.h"


#ifdef STATUS_SHARE_MODULE
static status_t* status_list_mirror = NULL;
#endif


static void other_func_ept_cal_serve_rpmsg(char* para,uint32_t size)
{
    nuclear_com_t str;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void*)&str,0,sizeof(str));
    str.src_ept_num = 0;
    str.dst_ept_num = other_func_inner_serve_ept_num;
    str.data_p = (void*)para;
    str.data_len = size;
    nuclear_com_send(&str,0xfffff);
}


/**
 * @brief 信息共享模块初始化.
 */
void ciss_init(void)
{
    uint32_t true_status_list_addr = 0;
    uint32_t para[2];
    para[0] = ciss_init_ept_num;
    para[1] = (uint32_t)&true_status_list_addr;
    other_func_ept_cal_serve_rpmsg((char*)para,sizeof(para));
    status_list_mirror = (status_t*)true_status_list_addr;
    ci_loginfo(LOG_STATUS_SHARE, "status_list_mirror addr = %08x\n",(uint32_t)status_list_mirror);
}

/**
 * @brief 读取状态信息.
 * 
 * @param id 状态信息标识，指定要读取的状态.
 * @return 要读取的状态值.
 */
status_t ciss_get(status_id_t id)
{
    if (id < EM_STATUS_NUM)
    {
        return status_list_mirror[id];
    }
    else
    {
        return INVALID_STATUS;
    }
}

/**
 * @brief 设置状态信息，如果状态有变化且是可等待状态，发送状态等待事件标志位.
 * 
 * @param id 状态信息标识，指定要设置的状态.
 * @param value 要设置的状态值.
 */
void ciss_set(status_id_t id, status_t value)
{
    if (id < EM_STATUS_NUM)
    {
        if (status_list_mirror[id] != value)
        {
            status_list_mirror[id] = value;
        }
    }
}





