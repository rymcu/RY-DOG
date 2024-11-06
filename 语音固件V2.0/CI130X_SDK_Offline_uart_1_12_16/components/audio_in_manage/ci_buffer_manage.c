
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sdk_default_config.h"
#include "ci_log.h"
#include <stdbool.h>
#include "ci_buffer_manage.h"
#include "FreeRTOS.h"
#include "task.h"


#define CI_BUFFER_MAG_PRINT_ENABLE  (0)//debug的打印


/**************************************************************************
                    typedef
****************************************************************************/

/**************************************************************************
                    global
****************************************************************************/

/**************************************************************************
                    function
****************************************************************************/


static void ci_buffer_manage_enter_critcal(void)
{

}


static void ci_buffer_manage_exit_critcal(void)
{

}


/**
 * @brief buffer manage初始化
 * 
 * @param buffer_info_p buffer信息的结构体指针
 * @param frm_num 需要初始化的buffer有多少帧（buffer大小 = write_frm_size * frm_num）
 * @param write_frm_size 写入buffer的一帧的大小
 * @param read_frm_size 读buffer时的一帧的大小
 * @return uint32_t 0:初始化失败
 *                  0xffffffff:这个buffer初始化过
 *                  其他:buffer的起始地址
 */
// #pragma GCC optimize("O0")
uint32_t ci_buffer_init(buffer_manage_t* buffer_info_p,buffer_manage_init_t* init_p)
{   
    ci_buffer_manage_enter_critcal();
    if(buffer_info_p->is_inited)
    {
        ci_buffer_manage_exit_critcal();
        ci_logwarn(CI_LOG_DEBUG,"inited : 0x%x",buffer_info_p->buffer_id);
        return 0xffffffff;
    }
    uint16_t frm_num = init_p->write_frm_num;
    uint16_t write_frm_size = init_p->write_frm_size;

    /*这个buffer的总大小，buffer大小应该被读写帧长整除*/
    int32_t buffer_size = frm_num * write_frm_size;
    #if CHECK_BUFFER_SIZE
    if(buffer_size % buffer_info_p->read_frm_size)
    {
        CI_ASSERT(0,"buffer size err\n");
    }
    #endif

    uint32_t buffer_addr = init_p->buffer_start_addr;
    if(0 == buffer_addr)
    {
        CI_ASSERT(0,"\n");
    }

    buffer_info_p->buffer_start_addr = init_p->buffer_start_addr;
    buffer_info_p->buffer_size = buffer_size;
    buffer_info_p->buffer_end_addr = buffer_addr + buffer_size;
    buffer_info_p->write_frm_size = write_frm_size;
    buffer_info_p->read_frm_size = init_p->read_frm_size;
    buffer_info_p->is_inited = true;
    buffer_info_p->valid_buffer_size = 0;
    buffer_info_p->w_r_diff_size = 0;
    buffer_info_p->write_ptr = buffer_addr;
    buffer_info_p->read_ptr = buffer_addr;
    buffer_info_p->buffer_id = init_p->buffer_id;

    ci_buffer_manage_exit_critcal();

    #if CI_BUFFER_MAG_PRINT_ENABLE
    mprintf("buffer id[0x%x] init done\n",buffer_info_p->buffer_id);
    #endif
    return buffer_addr;
}


/*判断是否写爆了，有时候需要判断，有时候不用判断
更新buffer有效大小
更新写指针
更新有效帧数

注意buffer回环*/
//这个函数不做什么buffer满了的判断，你让它写它就写，覆盖就覆盖
// #pragma GCC optimize("O0")
uint32_t ci_buffer_get_write_ptr(buffer_manage_t* buffer_info_p,uint32_t src_addr,
                                uint16_t frm_num,bool is_cpy_to_buffer)
{
    ci_buffer_manage_enter_critcal();

    #if 0//不判断帧数是否为1，与cpy模式的关系
    if(!is_cpy_to_buffer)
    {//如果是非cpy模式，每次写入帧数必须为1
        if(1 != frm_num)
        {
            CI_ASSERT(0,"\n");
        }
    }
    #endif

    if(buffer_info_p->is_writing)
    {//不允许连续写两次
        CI_ASSERT(0,"\n");
    }
    buffer_info_p->is_writing = true;

    uint16_t write_size = buffer_info_p->write_frm_size * frm_num;
    int32_t w_r_diff_size_will = buffer_info_p->w_r_diff_size + write_size;

    uint32_t buffer_start_addr = buffer_info_p->buffer_start_addr;
    int32_t buffer_size = buffer_info_p->buffer_size;
    uint32_t buffer_end_addr = buffer_info_p->buffer_end_addr;

    uint32_t write_ptr = buffer_info_p->write_ptr;

    #if 1
    //读写指针的差异超过了buffer总大小，表示buffer写爆了
    if(w_r_diff_size_will > buffer_size)
    {
        ci_buffer_manage_id_t id = buffer_info_p->buffer_id;
        // while(1)
        {
            #if CI_BUFFER_MAG_PRINT_ENABLE
            ci_logerr(CI_LOG_DEBUG,"buffer full : 0x%x",id);
            #endif
            write_ptr = 0;
            // CI_ASSERT(0,"\n");
            // vTaskDelay(pdMS_TO_TICKS(100));
        }
        buffer_info_p->w_r_diff_size = buffer_size;
    }
    else
    #endif
    {
        buffer_info_p->ask_write_frm_num = frm_num;
        uint32_t write_ptr_will = write_ptr + write_size;

        //判断是否回环
        if(write_ptr_will >= buffer_end_addr)
        {
            if(is_cpy_to_buffer)
            {//不允许要写的数据回环
                uint32_t dst = write_ptr;
                uint32_t src = src_addr;
                uint32_t size = buffer_end_addr - write_ptr;
                memcpy((void*)dst,(void*)src,size);
                dst = buffer_start_addr;
                src = src_addr + size;
                size = write_size - size;
                memcpy((void*)dst,(void*)src,size);
            }
            else
            {
                ci_buffer_manage_id_t buffer_id = buffer_info_p->buffer_id;
                ci_logwarn(CI_LOG_DEBUG,"buffer rollback : 0x%x",buffer_id);
            }
            uint32_t over_size = write_ptr_will - buffer_end_addr;
            buffer_info_p->write_ptr = buffer_start_addr + over_size;
        }
        else
        {
            if(is_cpy_to_buffer)
            {
                memcpy((void*)write_ptr,(void*)src_addr,write_size);
            }
            buffer_info_p->write_ptr = write_ptr_will;
        }
    }
    #if CI_BUFFER_MAG_PRINT_ENABLE
    //写的buffer是第多少个
    uint32_t buffer_num = (write_ptr - buffer_info_p->buffer_start_addr) / buffer_info_p->write_frm_size;
    mprintf("write_ptr[0x%x] = [%d] 0x%x\n",buffer_info_p->buffer_id,buffer_num,write_ptr);
    #endif
    ci_buffer_manage_exit_critcal();
    return write_ptr;
}


// #pragma GCC optimize("O0")
void ci_buffer_write_done(buffer_manage_t* buffer_info_p,bool is_judge_buffer_overflow)
{
    ci_buffer_manage_enter_critcal();
    if(!(buffer_info_p->is_writing))
    {//只有在调用了ci_buffer_write_get_ptr之后，才能调用本函数ci_buffer_write_done，否则报错
        CI_ASSERT(0,"\n");
    }
    buffer_info_p->is_writing = false;
    uint16_t frm_num = buffer_info_p->ask_write_frm_num;
    uint16_t write_size = buffer_info_p->write_frm_size * frm_num;
    
    int32_t buffer_size = buffer_info_p->buffer_size;

    int w_r_diff_size_will_be = buffer_info_p->w_r_diff_size + write_size;
    
    /*如果写读数据的差异，比buffer的大小还要大，表示buffer写爆了*/
    if(w_r_diff_size_will_be > buffer_size)
    {
        if(is_judge_buffer_overflow)
        {
            ci_buffer_manage_id_t buffer_id = buffer_info_p->buffer_id;
            ci_logerr(CI_LOG_DEBUG,"buffer full : 0x%x",buffer_id);
        }
        w_r_diff_size_will_be = buffer_size;
    }

    buffer_info_p->w_r_diff_size = w_r_diff_size_will_be;

    buffer_info_p->valid_buffer_size += write_size;
    if(buffer_info_p->valid_buffer_size > buffer_size)
    {
        buffer_info_p->valid_buffer_size = buffer_size;
    }

    #if CI_BUFFER_MAG_PRINT_ENABLE
    mprintf("valid_buffer_size[0x%x] = %d\n",buffer_info_p->buffer_id,buffer_info_p->valid_buffer_size);
    #endif

    #if CI_BUFFER_MAG_PRINT_ENABLE
    mprintf("w_r_diff_size[0x%x] = %d\n",buffer_info_p->buffer_id,buffer_info_p->w_r_diff_size);
    #endif

    buffer_info_p->ask_write_frm_num = 0;
    ci_buffer_manage_exit_critcal();
}


// #pragma GCC optimize("O0")
uint32_t ci_buffer_get_read_ptr(buffer_manage_t* buffer_info_p,uint16_t frm_num,
                                bool is_cpy_to_buffer,uint32_t dst_addr)
{
    ci_buffer_manage_enter_critcal();
    uint32_t ret = 0;
    if(buffer_info_p->is_reading)
    {
        CI_ASSERT(0,"\n");
    }
    buffer_info_p->is_reading = true;
    buffer_info_p->ask_read_frm_num = frm_num;
    uint16_t read_frm_size = buffer_info_p->read_frm_size;
    uint32_t wanted_read_size = frm_num * read_frm_size;
    uint32_t w_r_diff_size = buffer_info_p->w_r_diff_size;
    if(wanted_read_size > w_r_diff_size)
    {
        ci_logerr(CI_LOG_DEBUG,"not enough data to read : 0x%x",buffer_info_p->buffer_id);
    }

    uint32_t read_ptr = buffer_info_p->read_ptr;
    ret = read_ptr;

    uint32_t buffer_start_addr = buffer_info_p->buffer_start_addr;
    int32_t buffer_size = buffer_info_p->buffer_size;
    uint32_t buffer_end_addr = buffer_info_p->buffer_end_addr;
    uint32_t now_read_ptr = read_ptr + wanted_read_size;
    if(now_read_ptr >= buffer_end_addr)
    {
        if(is_cpy_to_buffer)
        {
            uint32_t dst = dst_addr;
            uint32_t src = read_ptr;
            uint32_t size = buffer_end_addr - read_ptr;
            memcpy((void*)dst,(void*)src,size);
            dst = dst_addr + size;
            src = buffer_start_addr;
            size = wanted_read_size - size;
            memcpy((void*)dst,(void*)src,size);
        }
        now_read_ptr = buffer_start_addr + (now_read_ptr - buffer_end_addr);
    }
    else
    {
        if(is_cpy_to_buffer)
        {
            memcpy((void*)dst_addr,(void*)read_ptr,wanted_read_size);
        }
    }
    buffer_info_p->read_ptr = now_read_ptr;

    #if CI_BUFFER_MAG_PRINT_ENABLE
    //写的buffer是第多少个
    uint32_t buffer_num = (read_ptr - buffer_info_p->buffer_start_addr) / buffer_info_p->read_frm_size;
    mprintf("get read_ptr[0x%x] = [%d] 0x%x\n",buffer_info_p->buffer_id,buffer_num,read_ptr);
    #endif

    #if CI_BUFFER_MAG_PRINT_ENABLE
    // mprintf("next read_ptr[0x%x] = 0x%x\n",buffer_info_p->buffer_id,buffer_info_p->read_ptr);
    #endif

    ci_buffer_manage_exit_critcal();
    return ret;
}


// #pragma GCC optimize("O0")
void ci_buffer_read_done(buffer_manage_t* buffer_info_p)
{
    ci_buffer_manage_enter_critcal();
    if(!buffer_info_p->is_reading)
    {
        CI_ASSERT(0,"\n");
    }
    buffer_info_p->is_reading = false;
    uint16_t frm_num = buffer_info_p->ask_read_frm_num;
    uint16_t read_size = buffer_info_p->read_frm_size * frm_num;
    buffer_info_p->w_r_diff_size -= read_size;
    #if CI_BUFFER_MAG_PRINT_ENABLE
    mprintf("w_r_diff_size[0x%x] = %d\n",buffer_info_p->buffer_id,buffer_info_p->w_r_diff_size);
    #endif
    buffer_info_p->ask_read_frm_num = 0;
    ci_buffer_manage_exit_critcal();
}


//读指针回退 frn_num 帧
/**
 * @brief buffer读指针回退 frn_num 帧
 * 
 * @param buffer_info_p 
 * @param frn_num 需要回退的帧数
 */
// #pragma GCC optimize("O0")
void ci_buffer_read_ptr_back(buffer_manage_t* buffer_info_p,uint16_t frn_num)
{
    ci_buffer_manage_enter_critcal();
    uint32_t w_r_diff_size = buffer_info_p->w_r_diff_size;
    uint16_t read_frm_size = buffer_info_p->read_frm_size;
    int32_t buffer_size =buffer_info_p->buffer_size;

    /*回退之后，相当于有效的数据量增加了*/ 
    w_r_diff_size += (read_frm_size * frn_num);
    
    /*没有那么多的有效数据去回退*/
    if(w_r_diff_size < buffer_info_p->valid_buffer_size)
    {
        w_r_diff_size = buffer_info_p->valid_buffer_size;
    }

    if(w_r_diff_size > buffer_size)
    {
        /*buffer里还有很多数据没有被读走，现在又要回退，肯定是达不到回退的目的的*/
        ci_buffer_manage_id_t id = buffer_info_p->buffer_id;
        ci_logerr(CI_LOG_DEBUG,"buffer read too slow : 0x%x",id);
    }
    else
    {
        uint32_t buffer_start_addr = buffer_info_p->buffer_start_addr;
        uint32_t buffer_end_addr = buffer_info_p->buffer_end_addr;
        buffer_info_p->w_r_diff_size = w_r_diff_size;
        uint32_t read_ptr_wanted = buffer_info_p->read_ptr - w_r_diff_size;
        if(read_ptr_wanted < buffer_start_addr)
        {
            read_ptr_wanted = buffer_end_addr - (buffer_start_addr - read_ptr_wanted);
        }
        buffer_info_p->read_ptr = read_ptr_wanted;
    }

    #if CI_BUFFER_MAG_PRINT_ENABLE
    mprintf("w_r_diff_size[0x%x] = %d\n",buffer_info_p->buffer_id,buffer_info_p->w_r_diff_size);
    #endif

    #if CI_BUFFER_MAG_PRINT_ENABLE
    uint32_t buffer_num = (buffer_info_p->read_ptr - buffer_info_p->buffer_start_addr) / buffer_info_p->read_frm_size;
    mprintf("read_ptr[0x%x] = [%d] %x\n",buffer_info_p->buffer_id,buffer_num,buffer_info_p->read_ptr);
    #endif

    ci_buffer_manage_exit_critcal();
}


/**
 * @brief 重置buffer状态，buffer的静态参数不用重置，只重置有效大小，读、写指针
 * 
 * @param buffer_info_p 
 */
// #pragma GCC optimize("O0")
void ci_buffer_state_reset(buffer_manage_t* buffer_info_p)
{
    ci_buffer_manage_enter_critcal();
    buffer_info_p->valid_buffer_size = 0;
    buffer_info_p->w_r_diff_size = 0;
    buffer_info_p->write_ptr = buffer_info_p->buffer_start_addr;
    buffer_info_p->read_ptr = buffer_info_p->buffer_start_addr;
    ci_buffer_manage_exit_critcal();
}

