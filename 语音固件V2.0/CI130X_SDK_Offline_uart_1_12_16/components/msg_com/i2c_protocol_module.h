/**
 * @file i2c_protocol_module.h
 * @brief CI130X芯片IIC通讯协议模块头文件
 * @version 0.1
 * @date 2019-11-9
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _I2C_PROTOCOL_MODULE_
#define _I2C_PROTOCOL_MODULE_

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
                            include
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "command_info.h"
#include "ci130x_iic.h"

/*-----------------------------------------------------------------------------
                            define
-----------------------------------------------------------------------------*/ 
#define  IIC_PROTOCOL_CMD_ID       1             /* 1：默认开启命令词ID的IIC协议 */
#define  IIC_PROTOCOL_SEC_ID       0             /* 0：默认关闭语义ID的IIC协议 */

#define  IIC_PROTOCOL_DEBUG        0             /* 默认关闭打印 */

#define  IIC_WAKEUP_WORD_CMDID     0x01          /* 唤醒词命令词ID ：1个字节*/
#define  IIC_WAKEUP_WORD_SECID     0x01e05501    /* 唤醒词语义ID ：4个字节*/

#if IIC_PROTOCOL_CMD_ID
#define  IIC_SEND_CMDID_LEN        3       /* slave发给master的命令词id数据个数*/
#define  I2C_RECV_STATUS_CMDID     0x02    /* 接收命令词ID*/
#define  IIC_RECV_CMDID_LEN        4       /* master发给slave的命令词id播报数据个数*/
#endif

#if IIC_PROTOCOL_SEC_ID
#define  IIC_SEND_SECID_LEN        6       /* slave发给master的语义id数据个数*/
#define  I2C_RECV_STATUS_SECID     0x03    /* 接收语义ID*/
#define  IIC_RECV_SECID_LEN        7       /* master发给slave的语义id播报数据个数*/
#endif

#define  I2C_RECV_STATUS_REG       0x01    /* 接收寄存器地址*/
#define  I2C_RECV_STATUS_CHECK_SUM 0x04    /* 接收校验和*/
#define  I2C_RECV_STATUS_END       0x05    /* 接收数据尾*/

/*-----------------------------------------------------------------------------
                            extern
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
                        struct / enum / union
-----------------------------------------------------------------------------*/
#if IIC_PROTOCOL_CMD_ID 
typedef enum 
{
    I2C_REG_CMDID      = 0x02,     /* 请求得到命令词ID的寄存器地址 */
    I2C_REG_PLAY_CMDID = 0x03,     /* 用命令词ID播放Audio的寄存器地址 */
}i2c_cmdid_reg_t;

/* 命令词id消息 */
#pragma pack(1)
typedef struct
{
    uint8_t reg;                   /*寄存器地址*/
    uint8_t cmd_id;                /*命令词ID：8位*/
    uint8_t check_sum;             /*校验和：reg累加cmd_id*/
    uint8_t end;                   /*数据尾：0x5a*/
}i2c_cmdid_msg_t;
#pragma pack()
#endif /*#if IIC_PROTOCOL_CMD_ID*/

#if IIC_PROTOCOL_SEC_ID
typedef enum 
{
    I2C_REG_SECID      = 0x04,     /* 请求得到语义ID的寄存器地址 */
    I2C_REG_PLAY_SECID = 0x05,     /* 用语义ID播放Audio的寄存器地址 */
}i2c_secid_reg_t;

/* 语义id消息 */
#pragma pack(1)
typedef struct        
{   
    uint8_t reg;                   /*寄存器地址*/
    uint8_t sec_id[4];             /*语义ID：32位*/
    uint8_t check_sum;             /*校验和：reg累加语义ID各个字节*/
    uint8_t end;                   /*数据尾：0x5a*/
}i2c_secid_msg_t;
#pragma pack()
#endif /*#if IIC_PROTOCOL_SEC_ID*/

#pragma pack(1)
typedef struct        
{   
    union
    {
         #if IIC_PROTOCOL_CMD_ID 
         i2c_cmdid_msg_t  cmdid_msg;
         #endif
         
         #if IIC_PROTOCOL_SEC_ID
         i2c_secid_msg_t  secid_msg;
         #endif
    };
}sys_msg_i2c_data_t;
#pragma pack()

/*-----------------------------------------------------------------------------
                        function declare
-----------------------------------------------------------------------------*/
void i2c_communicate_init(void);
void i2c_sleep_deal();
void i2c_product_send_msg(cmd_handle_t cmd_handle);
uint32_t userapp_deal_i2c_msg(sys_msg_i2c_data_t *msg);
uint32_t i2c_send_data(volatile uint8_t *reg, uint8_t *data);
uint32_t i2c_recv_secid_packet(uint8_t data);
uint32_t i2c_recv_cmdid_packet(uint8_t data);


#ifdef __cplusplus
}
#endif


#endif
