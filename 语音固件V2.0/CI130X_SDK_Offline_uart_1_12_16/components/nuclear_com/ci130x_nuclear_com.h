#ifndef _CI130X_NUCLEAR_COM_H_
#define _CI130X_NUCLEAR_COM_H_

#include "stdint.h"
#include "ci130x_system_ept.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef int32_t (*nuclear_com_serve_cb_t)(void *payload, uint32_t payload_len, uint32_t src, void *priv);


typedef struct 
{
    uint32_t    trans_flag_addr;//另外一边相应之后，将这个地址置为1，表示相应完成，
                                //这个元素不用在接口调用的地方填，可以不关心
    uint32_t    src_ept_num;//从哪个节点发送的请求
    uint32_t    dst_ept_num;//需要通知哪个节点
    void*       data_p;//数据的指针
    uint32_t    data_len;//数据的长度
}nuclear_com_t;


void nuclear_com_send(nuclear_com_t* msg_p,uint32_t time_out);
void nuclear_com_registe_serve(nuclear_com_serve_cb_t cb,mailbox_serve_ept_t ept_num);
void nuclear_com_init(void);


/******************NEW API*****************************************/


#define REMOTE_CALL(X) rc_caller_##X                // 用于声明远端调用

#define RC_MAX_PARAMETER_NUM        8               // 远端调用接口的最大参数数量
#define DECLEAR_REMOTE_FUNC_ID(X)   RC_##X##_ID     // 通过函数名定义远端调用接口ID
#define GET_REMOTE_FUNC_ID(X)       RC_##X##_ID     // 通过函数名获取远端调用接口ID


#if CORE_ID == 0
#define RC_HOST_CALLER(X)           rc_caller_##X
#define RC_HOST_CALLEE(X)           rc_callee_##X
#define RC_BNPU_CALLER(X)           not_used_caller_##X
#define RC_BNPU_CALLEE(X)           not_used_callee_##X
#else
#define RC_HOST_CALLER(X)           not_used_caller_##X
#define RC_HOST_CALLEE(X)           not_used_callee_##X
#define RC_BNPU_CALLER(X)           rc_caller_##X
#define RC_BNPU_CALLEE(X)           rc_callee_##X
#endif

typedef void (*rc_ret_callback_t)(uint32_t);


void rc_init();





#ifdef __cplusplus
}
#endif

#endif


