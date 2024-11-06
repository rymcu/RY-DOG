/**
 * @file i2c_protocol_module.c
 * @brief CI130X芯片IIC通讯协议程序
 * @version 0.1
 * @date 2020-07-20
 * 
 * @copyright Copyright (c) 2020  Chipintelli Technology Co., Ltd.
 * 
 */
/*-----------------------------------------------------------------------------
                            header
-----------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "i2c_protocol_module.h"
#include "ci130x_iic.h"
#include "platform_config.h"
#include "system_msg_deal.h"
#include "command_info.h"
#include "prompt_player.h"
#include "ci_log.h"
#include "sdk_default_config.h"
#include "romlib_runtime.h"

/* 该宏打开时，才能实现IIC通信协议 */
#if MSG_USE_I2C_EN

/*-----------------------------------------------------------------------------
                            define
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
                            extern
-----------------------------------------------------------------------------*/
extern void  resume_voice_in(void);       /* 恢复语音输入功能 */
extern void  pause_voice_in(void);        /* 停止语音输入功能 */
extern void set_state_enter_wakeup(uint32_t exit_wakup_ms);      /* 设置退出唤醒时间 */

/*-----------------------------------------------------------------------------
                        struct / enum / union
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
                            global
-----------------------------------------------------------------------------*/

#if IIC_PROTOCOL_CMD_ID
uint8_t send_cmdid_packet[IIC_SEND_CMDID_LEN] = {0};    /*存储slave发送给master的命令词ID数据包：cmd_id，check_sum，end */
uint8_t cmdid_msg_t[3] = {0};          /*存储临时计算的值*/
i2c_cmdid_msg_t  recv_cmdid_packet;    /*存储master发给slave的命令词ID数据包：reg，cmd_id，check_sum，end */
#endif 

#if IIC_PROTOCOL_SEC_ID
uint32_t secid_count = 0;              /*接收语义ID的字节个数*/
uint8_t send_secid_packet[IIC_SEND_SECID_LEN] = {0};    /*存储slave发送给master的语义ID数据包：sec_id[4]，check_sum，end */
uint8_t secid_msg_t[6] = {0};          /*存储临时计算的值*/
i2c_secid_msg_t  recv_secid_packet;    /*存储master发给slave的语义ID数据包：reg，sec_id[4]，check_sum，end */
#endif 

volatile uint32_t send_count = 0;      /*用于回复数据个数计数*/
uint8_t IIC_END_DATA = 0x5a;           /*数据包尾*/
volatile uint8_t i2c_recv_status = I2C_RECV_STATUS_REG;  /*初始数据接收状态标志*/

uint8_t i2c0_reg = 0xFF;               /* IICx作为salve 时，通信协议寄存器地址 */
static uint32_t i2c0_recv_count;              /* IICx作为salve 时，接收数据个数 */

/*-----------------------------------------------------------------------------
                            declare
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
                            function
-----------------------------------------------------------------------------*/

/**
 * @brief 发送消息给系统处理
 *
 * @param *msg : IIC系统消息
 */
static uint32_t i2c_port_send_packet_rev_msg(sys_msg_i2c_data_t *msg)
{
    sys_msg_t send_msg;
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    send_msg.msg_type = SYS_MSG_TYPE_I2C;                             
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&send_msg.msg_data), msg, sizeof(sys_msg_i2c_data_t));
    
    xResult = send_msg_to_sys_task(&send_msg,&xHigherPriorityTaskWoken);

    if((xResult != pdFAIL)&&(pdTRUE == xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken;
}

/**
 * @brief IIC协议播报回调函数
 *
 * @param cmd_handle : 命令词句柄
 */
static void i2c_play_voice_callback(cmd_handle_t cmd_handle)
{
    resume_voice_in();   
}

/**
 * @brief 生成带校验，发送给master的cmd_id数据包
 *
 * @param cmd_handle : 命令词句柄
 */
#if IIC_PROTOCOL_CMD_ID
static uint32_t i2c_product_send_cmdid_msg(cmd_handle_t cmd_handle)
{
    uint8_t cmd_id = cmd_info_get_command_id(cmd_handle);
    
    cmdid_msg_t[0] = cmd_id;    /*仅支持非0的ID*/
    cmdid_msg_t[1] = ( I2C_REG_CMDID + cmdid_msg_t[0] );   
    cmdid_msg_t[2] = IIC_END_DATA;
    
    #if IIC_PROTOCOL_DEBUG
    ci_loginfo(CI_LOG_DEBUG,"[*i2c product cmdid_msg*] : cmd_id-[%0.2x],check_sum-[%0.2x],end-[%0.2x]\n\n",
               cmdid_msg_t[0],cmdid_msg_t[1],cmdid_msg_t[2]);
    #endif
    
    return RETURN_OK;
}

/**
 * @brief 接收带校验的命令词ID数据包
 *
 * @param data : 接收到的字节
 */
uint32_t i2c_recv_cmdid_packet(uint8_t data)
{
    switch(i2c_recv_status)
    {
        /*接收寄存器地址*/
        case I2C_RECV_STATUS_REG:
        {          
             if(data == I2C_REG_PLAY_CMDID)
             {
                 recv_cmdid_packet.reg = I2C_REG_PLAY_CMDID;          
                 i2c_recv_status = I2C_RECV_STATUS_CMDID;
             }
             else
             {
                 i2c_recv_status = I2C_RECV_STATUS_REG;
             } 
             break;
        }
        /*接收命令词ID*/
        case I2C_RECV_STATUS_CMDID:
        {           
             if(data != 0x00)
             {
                recv_cmdid_packet.cmd_id = data;
                i2c_recv_status = I2C_RECV_STATUS_CHECK_SUM;
             }
             else
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
             }
             break;  
        }
        /*接收校验和*/
        case I2C_RECV_STATUS_CHECK_SUM:
        {       
             recv_cmdid_packet.check_sum = data;
             i2c_recv_status = I2C_RECV_STATUS_END;
             break;
        }
        /*接收数据尾巴*/
        case I2C_RECV_STATUS_END:
        {          
             if(data == IIC_END_DATA)
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
                
                /*收到master回复的正确数据，需将发送的数据缓存清除*/
                #if IIC_PROTOCOL_SEC_ID
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_secid_packet, 0, sizeof(send_secid_packet));
                #endif
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_cmdid_packet, 0, sizeof(send_cmdid_packet));
                
                recv_cmdid_packet.end = data;
                sys_msg_i2c_data_t i2c_msg;
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&i2c_msg.cmdid_msg), &recv_cmdid_packet, sizeof(i2c_cmdid_msg_t));
                i2c_port_send_packet_rev_msg(&i2c_msg);
             }
             else
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
             }
             break; 
        } 
        default :
             i2c_recv_status = I2C_RECV_STATUS_REG;
             break;
    }

    return RETURN_OK;
}

/**
 * @brief 处理接收到的cmd_id消息
 * 
 * @param msg : 消息
 * 
 */
static uint32_t i2c_deal_cmdid_msg(sys_msg_i2c_data_t *i2c_msg)
{
    i2c_cmdid_msg_t msg ;

    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&msg), &(i2c_msg->cmdid_msg), sizeof(i2c_cmdid_msg_t));

    #if IIC_PROTOCOL_DEBUG
    ci_loginfo(CI_LOG_DEBUG,"[*i2c_deal_cmdid_msg*]: reg-[%0.2x], cmd_id-[%d], check_sum-[%0.2x], end-[%0.2x]\n\n",
                                                     msg.reg,   msg.cmd_id,   msg.check_sum,   msg.end);
    #endif
    
    if((PLAY_ENTER_WAKEUP_EN == 0) && (PLAY_OTHER_CMD_EN == 0))    /*如果屏蔽了主动播报，这里就被动播报*/
    {
        if((msg.cmd_id == IIC_WAKEUP_WORD_CMDID) || (msg.cmd_id == 0xd0))   /*唤醒词到来*/
        {
            enter_wakeup_deal(EXIT_WAKEUP_TIME ,INVALID_HANDLE);
            msg.cmd_id = IIC_WAKEUP_WORD_CMDID;   /*统一用id [1] 播报唤醒词*/
        }
        else  /*其他词到来*/
        {
        set_state_enter_wakeup(EXIT_WAKEUP_TIME);
        }
        
        /*停止录音*/
        pause_voice_in();
        prompt_play_by_cmd_id(msg.cmd_id, -1, i2c_play_voice_callback,true);
        ci_loginfo(CI_LOG_DEBUG,"i2c协议[命令词ID]被动播报成功！\n");
    }
    else
    {
        ci_loginfo(CI_LOG_DEBUG,"i2c协议[命令词ID]被动播报失败！！！请修改user_config.h屏蔽唤醒词、命令词的主动播报！！！\n");
    }

    return RETURN_OK;     
}
#endif /*#if IIC_PROTOCOL_CMD_ID*/

/**
 * @brief 生成带校验，发送给master的semantic_id数据包
 *
 * @param cmd_handle : 命令词句柄
 */
#if IIC_PROTOCOL_SEC_ID
static uint32_t i2c_product_send_secid_msg(cmd_handle_t cmd_handle)
{
    uint32_t sec_id = cmd_info_get_semantic_id(cmd_handle); 
    
    secid_msg_t[0] = (uint8_t)sec_id;     /*仅支持非0的ID*/
    secid_msg_t[1] = (uint8_t)(sec_id >> 8); 
    secid_msg_t[2] = (uint8_t)(sec_id >> 16); 
    secid_msg_t[3] = (uint8_t)(sec_id >> 24 ); 
    secid_msg_t[4] = (I2C_REG_SECID + secid_msg_t[0] + secid_msg_t[1] + secid_msg_t[2] + secid_msg_t[3]);
    secid_msg_t[5] = IIC_END_DATA;  
    
    #if IIC_PROTOCOL_DEBUG
    ci_loginfo(CI_LOG_DEBUG,"[*i2c product secid_msg*]: sec_id-[%0.2x%0.2x%0.2x%0.2x],check_sum-[%0.2x],end-[%0.2x]\n\n",
               secid_msg_t[3],secid_msg_t[2],secid_msg_t[1],secid_msg_t[0],secid_msg_t[4],secid_msg_t[5]);
    #endif
    
    return RETURN_OK;
}

/**
 * @brief 接收带校验的语义ID数据包
 *
 * @param data : 接收到的字节
 */
uint32_t i2c_recv_secid_packet(uint8_t data)
{
    switch(i2c_recv_status)
    {
        /*接收寄存器地址*/
        case I2C_RECV_STATUS_REG:
        {        
             if(data == I2C_REG_PLAY_SECID)
             {
                 recv_secid_packet.reg = data;
                 secid_count = 0;
                 i2c_recv_status = I2C_RECV_STATUS_SECID;
             }
             else
             {
                 i2c_recv_status = I2C_RECV_STATUS_REG;
             }
             break;
        }
        /*接收语义ID*/
        case I2C_RECV_STATUS_SECID:
        {       
             if(data != 0x00)      /*暂时只支持非全0语义ID*/
             {
                recv_secid_packet.sec_id[secid_count++] = data;
                if(secid_count == 4)
                {
                    secid_count = 0;
                    i2c_recv_status = I2C_RECV_STATUS_CHECK_SUM; 
                } 
             }
             else
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
             }
             break;  
        }
        /*接收校验和*/
        case I2C_RECV_STATUS_CHECK_SUM:
        {               
             recv_secid_packet.check_sum = data;
             i2c_recv_status = I2C_RECV_STATUS_END;  
             break;
        }  
        /*接收数据尾*/
        case I2C_RECV_STATUS_END:
        {            
             if(data == IIC_END_DATA)
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
                
                /*收到master回复的正确数据，需将发送的数据缓存清除*/
                #if IIC_PROTOCOL_CMD_ID
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_cmdid_packet, 0, sizeof(send_cmdid_packet));
                #endif
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_secid_packet, 0, sizeof(send_secid_packet));
                
                recv_secid_packet.end = data;
                sys_msg_i2c_data_t i2c_msg;
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&i2c_msg.secid_msg), &recv_secid_packet, sizeof(i2c_secid_msg_t));
                i2c_port_send_packet_rev_msg(&i2c_msg);
             }
             else
             {
                i2c_recv_status = I2C_RECV_STATUS_REG;
             }
             break;  
        }
        default :
             i2c_recv_status = I2C_RECV_STATUS_REG;
             break;
    }

    return RETURN_OK;
}

/**
 * @brief 处理接收到的semantic_id消息
 * 
 * @param msg : i2c消息
 * 
 */
static uint32_t i2c_deal_secid_msg(sys_msg_i2c_data_t *i2c_msg)
{
    i2c_secid_msg_t msg ; 
    uint32_t sec_id;

    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&msg), &(i2c_msg->secid_msg), sizeof(i2c_secid_msg_t));

    sec_id  = *((uint32_t *)(&msg.sec_id[0]));    
    
    #if IIC_PROTOCOL_DEBUG
    ci_loginfo(CI_LOG_DEBUG,"[*i2c_deal_secid_msg*]: reg-[%0.2x], sec_id-[%0.2x%0.2x%0.2x%0.2x], check_sum-[%0.2x], end-[%0.2x]\n\n",
               msg.reg, msg.sec_id[3], msg.sec_id[2], msg.sec_id[1], msg.sec_id[0], msg.check_sum, msg.end);
    #endif
    
    if((PLAY_ENTER_WAKEUP_EN == 0) && (PLAY_OTHER_CMD_EN == 0))    /*如果屏蔽了主动播报,就被动播报*/
    {
        if(sec_id == IIC_WAKEUP_WORD_SECID)   /*唤醒词到来*/
        {
            enter_wakeup_deal(EXIT_WAKEUP_TIME ,INVALID_HANDLE);
        }
        else  /*其他词到来*/
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME);
        }
        
        /*停止录音*/
        pause_voice_in();
        prompt_play_by_cmd_id(sec_id, -1, i2c_play_voice_callback,true);
        
        ci_loginfo(CI_LOG_DEBUG,"i2c协议[语义ID]被动播报成功！\n");
    }
    else
    {
        ci_loginfo(CI_LOG_DEBUG,"i2c协议[语义ID]被动播报失败！！！请修改user_config.h屏蔽唤醒词、命令词的主动播报！！！\n");
    }

    return RETURN_OK;     
}
#endif /*#if IIC_PROTOCOL_SEC_ID*/

/**
 * @brief 作为salve时，在中断里回复数据给master
 *
 * @param I2Cx : IIC编号用来寻址寄存器
 * @param reg : 通信协议寄存器地址
 */
uint32_t i2c_send_data(volatile uint8_t *reg, uint8_t *data)
{
    switch(*reg){
        /*发送命令词id信息给master*/
        #if IIC_PROTOCOL_CMD_ID
        case I2C_REG_CMDID:
            *data = send_cmdid_packet[send_count++];
            if(send_count == IIC_SEND_CMDID_LEN)
            {
                send_count = 0;
            }
            break;
        #endif

        /*发送语义id信息给master*/
        #if IIC_PROTOCOL_SEC_ID
        case I2C_REG_SECID:
        	*data = send_secid_packet[send_count++];
            if(send_count == IIC_SEND_SECID_LEN)
            {
                send_count = 0;
            }
            break;
        #endif

        /*不支持的寄存器地址，回复00*/
        default :
        {
        	*data = 0x00;
            break;
        }  
    }

    return RETURN_OK;
}


/**
 * @brief 处理IIC通信的系统消息
 */
uint32_t userapp_deal_i2c_msg(sys_msg_i2c_data_t *i2c_msg)
{
    uint8_t reg = *((uint8_t *)i2c_msg);
    
    switch(reg)
    {
        /*用命令词ID被动播报*/
        #if IIC_PROTOCOL_CMD_ID
        case I2C_REG_PLAY_CMDID:  
            i2c_deal_cmdid_msg(i2c_msg);
            break;
        #endif
        
        /*用语义ID被动播报*/
        #if IIC_PROTOCOL_SEC_ID
        case I2C_REG_PLAY_SECID:  
            i2c_deal_secid_msg(i2c_msg);
            break;
        #endif

        default:
            break;
    }
    
    return RETURN_OK; 
}

/**
 * @brief 退出唤醒后，清除发送给master的数据包
 */
void i2c_sleep_deal()
{
    #if IIC_PROTOCOL_CMD_ID
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_cmdid_packet, 0, sizeof(send_cmdid_packet));
    #endif
    
    #if IIC_PROTOCOL_SEC_ID
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(send_secid_packet, 0, sizeof(send_secid_packet));
    #endif
}

/**
 * @brief 将计算得到的存储到发送缓存
 */
void i2c_save_send_msg(void)
{
    #if IIC_PROTOCOL_CMD_ID
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(send_cmdid_packet), cmdid_msg_t, sizeof(send_cmdid_packet));
    #endif
    
    #if IIC_PROTOCOL_SEC_ID
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(send_secid_packet), secid_msg_t, sizeof(send_secid_packet));
    #endif
}

/**
 * @brief 生成发送给master的数据包
 * 
 * @param cmd_handle : 命令词句柄
 */
void i2c_product_send_msg(cmd_handle_t cmd_handle)
{
    #if IIC_PROTOCOL_CMD_ID
    i2c_product_send_cmdid_msg(cmd_handle);
    #endif
    
    #if IIC_PROTOCOL_SEC_ID    
    i2c_product_send_secid_msg(cmd_handle);
    #endif
    
    i2c_save_send_msg();
}

/**
 * @brief slave接收回调函数
 * 
 * @param data : 数据
 * @param stop : 是否有stop
 */
bool i2c_slave_recv_callback(char data, bool stop)
{
    if(!stop)
    {
        if(i2c0_recv_count == 0)       /*第1个字节为寄存器地址*/
        {
            i2c0_reg = data;    /*从接收寄存器读数据*/
            i2c_recv_status = I2C_RECV_STATUS_REG;
            send_count = 0;
        }

        #if IIC_PROTOCOL_CMD_ID
        if(i2c0_reg == I2C_REG_PLAY_CMDID)
        {
            i2c_recv_cmdid_packet(data);  /*接收命令ID播报消息*/
            i2c0_recv_count ++;              /*接收数据个数计数*/
        }
        #endif

        #if IIC_PROTOCOL_SEC_ID
        if(i2c0_reg == I2C_REG_PLAY_SECID)
        {
            i2c_recv_secid_packet(data);  /*接收语义ID播报消息*/
            i2c0_recv_count ++;              /*接收数据个数计数*/
        }
        #endif
        return true;
    }
    else
    {
        send_count = 0;
        i2c_recv_status = I2C_RECV_STATUS_REG; 
        i2c0_recv_count = 0;
        i2c0_reg = 0xFF;
        return false;
    }
}

/**
 * @brief slave发送回调函数
 * 
 * @param data : 数据指针
 * @param state : 发送状态
 * @param previous_ack : 当前ack
 */
bool i2c_slave_send_callback(char *data, IIC_SendStateType state, IIC_AckType previous_ack)
{
    if(previous_ack != IIC_ACKTYPE_ERR){
        //mprintf("previous_ack %d\n",previous_ack);
    }
    if(state == IIC_SENDSTATE_OK)  //状态正常
    {
        i2c_send_data(&i2c0_reg,data);
        return true;
    }
    else if(state == IIC_SENDSTATE_STOP)
    {
        //本次发送将已停止
        send_count = 0;
        i2c_recv_status = I2C_RECV_STATUS_REG; 
        i2c0_recv_count = 0;
        i2c0_reg = 0xFF;
        return true;
    }
    else
    {
        return true;
    }
}

/**
 * @brief IIC通信协议初始化，引脚、模式
 */
void i2c_communicate_init(void)
{
    /*引脚初始化*/
    pad_config_for_i2c();
    /*IIC初始化为SLAVE模式，7位设备地址0x64*/
    iic_interrupt_init(IIC0,100,0x64,SHORT_TIME_OUT);
    /*注册中断发送函数*/
    iic_slave_interrupt_send(IIC0,i2c_slave_send_callback);
    /*注册中断接收函数*/
    iic_slave_interrupt_recv(IIC0,i2c_slave_recv_callback);
}

#endif   /* #if MSG_USE_I2C_EN */

