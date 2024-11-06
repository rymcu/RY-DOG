
#ifndef __CI_BUFFER_MANAGE_H
#define __CI_BUFFER_MANAGE_H

#include <stdint.h>


#define CHECK_BUFFER_SIZE   (0)


#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
    CI_PCM_BUFFER_ID = 0x66,
    CI_FE_BUFFER_ID = 0x77,
    CI_NN_OUT_BUFFER_ID = 0x88,
}ci_buffer_manage_id_t;


typedef struct 
{
    uint32_t buffer_start_addr;         //buffer的起始地址
    uint32_t buffer_end_addr;
    int32_t buffer_size;                //buffer的大小
    int32_t valid_buffer_size;          //buffer当前有效大小，会一直累加（最大为buffer的大小），只会在buffer reset的时候清零
    volatile int32_t w_r_diff_size;     //写、读数据的差值
    uint32_t write_ptr;                 //buffer写指针
    uint32_t read_ptr;                  //buffer读指针
    uint16_t write_frm_size;            //写buffer的帧大小
    uint16_t read_frm_size;             //读buffer的帧大小
    uint16_t ask_write_frm_num;         //请求写入的帧数
    uint16_t ask_read_frm_num;          //请求读出的帧数
    ci_buffer_manage_id_t buffer_id;
    bool is_inited;                     //这个buffer是否被初始化过
    bool is_writing;
    bool is_reading;
}buffer_manage_t;



typedef struct 
{
    uint32_t buffer_start_addr;         //buffer的起始地址
    uint16_t write_frm_num;             //写buffer的帧数是多少
    uint16_t write_frm_size;            //写buffer的帧大小
    uint16_t read_frm_size;             //读buffer的帧大小
    ci_buffer_manage_id_t buffer_id;
}buffer_manage_init_t;


uint32_t ci_buffer_init(buffer_manage_t* buffer_info_p,buffer_manage_init_t* init_p);

uint32_t ci_buffer_get_write_ptr(buffer_manage_t* buffer_info_p,uint32_t src_addr,
                                uint16_t frm_num,bool is_cpy_to_buffer);
void ci_buffer_write_done(buffer_manage_t* buffer_info_p,bool is_judge_buffer_overflow);

uint32_t ci_buffer_get_read_ptr(buffer_manage_t* buffer_info_p,uint16_t frm_num,
                                bool is_cpy_to_buffer,uint32_t dst_addr);
void ci_buffer_read_done(buffer_manage_t* buffer_info_p);

void ci_buffer_read_ptr_back(buffer_manage_t* buffer_info_p,uint16_t frn_num);

void ci_buffer_state_reset(buffer_manage_t* buffer_info_p);

#ifdef __cplusplus
}
#endif

#endif
