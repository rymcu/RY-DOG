/**
 * @file ci130x_iic.h
 * @brief CI110X芯片IIC模块的驱动程序头文件
 * @version 0.1
 * @date 2019-10-28
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef __CI130X_IIC_H_
#define __CI130X_IIC_H_

#include <stdbool.h>
#include "ci130x_system.h"
#include "ci130x_scu.h"
#include "ci130x_core_eclic.h"
#include "ci130x_core_misc.h"

#ifdef __cplusplus
    extern "C"{
#endif

/**
 * @brief IIC控制器定义
 */
typedef enum
{
    IIC0 = HAL_IIC0_BASE,
    IIC_NULL = 0,
}iic_base_t;

typedef enum 
{
    LONG_TIME_OUT  = 0X5FFFFF,
    SHORT_TIME_OUT = 0XFFFF,
}IIC_TimeOut;

typedef enum 
{
    IIC_ACKTYPE_ERR = -1,
    IIC_ACKTYPE_ACK  = 0,
    IIC_ACKTYPE_NACK = 1,
}IIC_AckType;

typedef enum 
{
    IIC_SENDSTATE_OK  = 0,
    IIC_SENDSTATE_STOP = 1,
    IIC_SENDSTATE_NOTIFY = 2,
}IIC_SendStateType;

typedef enum 
{
    IIC_M_WRITE  = 0,
    IIC_M_READ = 1,
}iic_multi_transmission_type;

typedef bool (*master_send_cb_t)(char* data, IIC_SendStateType state, IIC_AckType previous_ack);
typedef bool (*master_recv_cb_t)(char data, bool stop);
typedef bool (*slave_send_cb_t)(char* data, IIC_SendStateType state, IIC_AckType previous_ack);
typedef bool (*slave_recv_cb_t)(char data, bool stop);

typedef struct
{
    char *buf;
    int size;
    iic_multi_transmission_type flag;
    union{
        int read_size;
        int write_size;
    };
}multi_transmission_msg;


/****************************************** 常用接口 **********************************************************/
void iic_polling_init(iic_base_t base, uint32_t speed, uint32_t slaveaddr, IIC_TimeOut timeout);
int32_t iic_master_polling_send(iic_base_t base, uint16_t addr, const char *buf, int32_t count, uint8_t *last_ack_flag);
int32_t iic_master_polling_recv(iic_base_t base, uint16_t addr, char *buf, int32_t count);
int32_t iic_master_multi_transmission(iic_base_t base, uint16_t addr, multi_transmission_msg *msg, int msg_count);
/*************************************************************************************************************/
/****************************************** 其他接口 **********************************************************/
int32_t iic_slave_polling_send(iic_base_t base, const char *buf, int32_t count, uint8_t *last_ack_flag);
int32_t iic_slave_polling_recv(iic_base_t base, char *buf, int32_t count);
void iic_interrupt_init(iic_base_t base, uint32_t speed, uint32_t slaveaddr, IIC_TimeOut timeout);
int32_t iic_master_interrupt_send(iic_base_t base, uint16_t addr, master_send_cb_t master_send_cb);
int32_t iic_master_interrupt_recv(iic_base_t base, uint16_t addr, master_recv_cb_t master_recv_cb);
int32_t iic_slave_interrupt_send(iic_base_t base, slave_send_cb_t slave_send_cb);
int32_t iic_slave_interrupt_recv(iic_base_t base, slave_recv_cb_t slave_recv_cb);
/*************************************************************************************************************/
void IIC_IRQHandler(iic_base_t base);
/*************************************************************************************************************/

/****************************************** 兼容旧驱动接口 ***************************************************/
int32_t i2c_master_only_send(char slave_ic_address, const char *buf, int32_t count);
int32_t i2c_master_send_recv(char slave_ic_address, char *buf, int32_t send_len, int32_t rev_len);
int32_t i2c_master_only_recv(char slave_ic_address, char *buf, int32_t rev_len);
/*************************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*__CI130X_IIC_H_*/
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/