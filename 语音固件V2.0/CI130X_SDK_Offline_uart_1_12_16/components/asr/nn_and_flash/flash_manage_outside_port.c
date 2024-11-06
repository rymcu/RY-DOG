
#include "nn_and_flash_manage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
#include "flash_manage_outside_port.h"
#include "ci130x_nuclear_com.h"
#include <string.h>
#include <stdlib.h>
#include "ci130x_core_misc.h"
#include "status_share.h"
#include "romlib_runtime.h"


#define FMO_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("FMO Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }



typedef enum
{
    FLASH_REQ_READ = 1<<0,
    FLASH_REQ_WRITE = 1<<1,
    FLASH_REQ_ERASE = 1<<2,
    FLASH_REQ_READ_UNIQUE_ID = 1<<3,
    FLASH_REQ_MAX_NUM = 1<<4,
}flash_req_typt_t;


static EventGroupHandle_t flash_req_op_done_event_group = NULL;

SemaphoreHandle_t flash_ctl_xSemaphore = NULL;


static void wait_flash_sem(void)
{
    if(pdFAIL == xSemaphoreTake(flash_ctl_xSemaphore,pdMS_TO_TICKS(3000)))
    {
        FMO_CI_ASSERT(0,"\n");
    }
}


static void give_flash_sem(void)
{
    ciss_set(CI_SS_FLASH_HOST_STATE,CI_SS_FLASH_IDLE);
    xSemaphoreGive(flash_ctl_xSemaphore);
}


static void flash_manage_req_wait_op_done_sem(flash_req_typt_t op_type)
{
    EventBits_t ret = xEventGroupWaitBits(flash_req_op_done_event_group, op_type, pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
    if(ret != op_type)
    {
        //等待flash信号量超时
        // mprintf("op_type = %d\n",op_type);
        FMO_CI_ASSERT(0,"\n");
    }
}


static void flash_manage_ept_cal_serve_rpmsg(char* para,uint32_t size)
{
    nuclear_com_t str;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void*)&str,0,sizeof(str));
    str.src_ept_num = 0;
    str.dst_ept_num = flash_manage_serve_inner_ept_num;
    str.data_p = (void*)para;
    str.data_len = size;
    nuclear_com_send(&str,0xfffff);
}


void is_flash_power_off(bool* state)
{
    uint32_t para[2];
    para[0] = is_flash_power_off_ept_num;
    para[1] = (uint32_t)state;
    flash_manage_ept_cal_serve_rpmsg((char*)para,sizeof(para));
}


static void flash_manage_give_op_done_sem(flash_req_typt_t op_type)
{//只能在中断中调用，也就是mailbox通知中断里面
    BaseType_t xTaskWokenByReceive = pdFALSE;
    if(check_curr_trap())
    {
        xEventGroupSetBitsFromISR(flash_req_op_done_event_group, (op_type), &xTaskWokenByReceive);
        portEND_SWITCHING_ISR(xTaskWokenByReceive);
    }
    else
    {
        xEventGroupSetBits(flash_req_op_done_event_group, (op_type));
    }
}


uint32_t xip_read_flash(uint8_t* buf,uint32_t addr,uint32_t size)
{
    uint32_t tmp;
    uint32_t end_addr = addr + size;
    //mprintf("addr:0x%x, size = %d\n",addr, size);
    uint32_t pre_num = addr & 3;
    if(pre_num)
    {
        tmp = *(uint32_t*)(addr & 0xFFFFFFFC);
        uint32_t real_size = 4 - pre_num;
        if(size < real_size)
        {
            real_size = size;
        }
        memcpy(buf,((uint8_t*)&tmp) + pre_num, real_size);
        addr += real_size;
        buf += real_size; 
    }

    int n = (end_addr - (uint32_t)addr)/4;

    for(int i = 0; i < n; i++)
    {
        *(uint32_t*)buf = *(uint32_t*)addr;
        buf += 4;
        addr += 4;
    }

    if(end_addr & 3)
    {
        tmp = *(uint32_t*)addr;
        memcpy(buf,(uint8_t*)&tmp,end_addr & 3);
    }

    return size;
}

static volatile char tmp[32];
// #pragma GCC optimize("O0")
int32_t post_read_flash(char *buf, uint32_t addr, uint32_t size)
{
    wait_flash_sem();
    ciss_set(CI_SS_FLASH_HOST_STATE,CI_SS_FLASH_READ);
    if(addr >= FLASH_CPU_READ_BASE_ADDR)
    {
        addr -= FLASH_CPU_READ_BASE_ADDR;
    }
    // mprintf("buf:0x%x, addr:0x%x, size = %d\n", buf, addr, size);

#if 0 
    //解决卡死问题
    if(addr & 3)
    {
        uint32_t read_size = 4 - (addr & 3);
        memcpy(tmp, FLASH_CPU_READ_BASE_ADDR + (addr & (~0x3)) - 1, read_size + 1);
    }
    memcpy(buf, FLASH_CPU_READ_BASE_ADDR + addr, size);
#else
    
    xip_read_flash(buf,FLASH_CPU_READ_BASE_ADDR+addr,size);
#endif
    give_flash_sem();
    return RETURN_OK;
}


int32_t post_erase_flash(uint32_t addr, uint32_t size)
{
    if((addr % 4096) || (size % 4096))
    {
         CI_ASSERT(0,"Erase addr and size must be 4096 aligned.\n");
    }
    wait_flash_sem();
    ciss_set(CI_SS_FLASH_HOST_STATE,CI_SS_FLASH_ERASE);
    //nuclear send里面有信号量保护，所以外面不用
    uint32_t para[4];
    para[0] = post_erase_flash_ept_num;
    para[1] = addr;
    para[2] = size;
    // mprintf("HOST req flash erase 1\n");
    flash_manage_ept_cal_serve_rpmsg((char*)para,sizeof(para));
    flash_manage_req_wait_op_done_sem(FLASH_REQ_ERASE);
    // mprintf("HOST req flash erase done\n");
    give_flash_sem();
    return RETURN_OK;
}


int32_t post_write_flash(char *buf, uint32_t addr, uint32_t size)
{
    wait_flash_sem();
    ciss_set(CI_SS_FLASH_HOST_STATE,CI_SS_FLASH_WRITE);
    //nuclear send里面有信号量保护，所以外面不用
    uint32_t para[4];
    para[0] = post_write_flash_ept_num;
    para[1] = (uint32_t)buf;
    para[2] = addr;
    para[3] = size;
    // mprintf("HOST req flash write 1\n");
    flash_manage_ept_cal_serve_rpmsg((char*)para,sizeof(para));
    flash_manage_req_wait_op_done_sem(FLASH_REQ_WRITE);
    // mprintf("HOST req flash write done\n");
    give_flash_sem();
    return RETURN_OK;
}


int32_t post_spic_read_unique_id(uint8_t* unique_id)
{
    wait_flash_sem();
    ciss_set(CI_SS_FLASH_HOST_STATE,CI_SS_FLASH_READ_UNIQUE_ID);
    uint32_t para[2];
    para[0] = post_spic_read_unique_id_ept_num;
    para[1] = (uint32_t)unique_id;
    // mprintf("HOST req flash read uniq id 1\n");
    flash_manage_ept_cal_serve_rpmsg((char*)para,sizeof(para));
    flash_manage_req_wait_op_done_sem(FLASH_REQ_READ_UNIQUE_ID);
    // mprintf("HOST req flash read uniq id done\n");
    give_flash_sem();
    return RETURN_OK;
}


void flash_manage_outside_port_init(void)
{
    flash_req_op_done_event_group = xEventGroupCreate();
    if(NULL == flash_req_op_done_event_group)
    {
        FMO_CI_ASSERT(0,"\n");
    }
}


#define FLASH_MANAGE_MAX_PARA_NUM_OUTSIDE    (6)
static uint32_t flash_manage_nuclear_com_outside_buf[FLASH_MANAGE_MAX_PARA_NUM_OUTSIDE];

static int32_t flash_manage_nuclear_com_outside_serve_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    // MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)flash_manage_nuclear_com_outside_buf,(void*)payload,payload_len);
    uint32_t* data = (uint32_t*)payload;

    switch(data[0])
    {
        case post_read_flash_op_done_ept_num:
        {
            #if NUCLER_COM_JUDGE_PARA_NUM
            if(4 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FMO_CI_ASSERT(0,"\n");
            }
            #endif
            flash_manage_give_op_done_sem(FLASH_REQ_READ);
            break;
        }
        case post_erase_flash_op_done_ept_num:
        {
            #if NUCLER_COM_JUDGE_PARA_NUM
            if(4 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FMO_CI_ASSERT(0,"\n");
            }
            #endif
            flash_manage_give_op_done_sem(FLASH_REQ_ERASE);
            break;
        }
        case post_write_flash_op_done_ept_num:
        {
            #if NUCLER_COM_JUDGE_PARA_NUM
            if(4 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FMO_CI_ASSERT(0,"\n");
            }
            #endif
            flash_manage_give_op_done_sem(FLASH_REQ_WRITE);
            break;
        }
        case post_spic_read_unique_id_op_done_ept_num:
        {
            #if NUCLER_COM_JUDGE_PARA_NUM
            if(4 != payload_len)
            {
                mprintf("payload_len = %d\n",payload_len);
                FMO_CI_ASSERT(0,"\n");
            }
            #endif
            flash_manage_give_op_done_sem(FLASH_REQ_READ_UNIQUE_ID);
            break;
        }
        default:
            break;
    }
    return 0;
}



void flash_manage_nuclear_com_outside_port_init(void)
{
    flash_ctl_xSemaphore = xSemaphoreCreateMutex();
    if(NULL == flash_ctl_xSemaphore)
    {
        FMO_CI_ASSERT(0,"\n");  
    }
    give_flash_sem();
    flash_manage_outside_port_init();
    nuclear_com_registe_serve(flash_manage_nuclear_com_outside_serve_cb,flash_manage_serve_outside_ept_num);
}








