#ifndef __USER_MSG_DEAL_H__
#define __USER_MSG_DEAL_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 按命令词ID响应asr消息处理
 * 
 * @param asr_msg 
 * @param cmd_handle 
 * @param cmd_id 
 * @return uint32_t 
 */
uint32_t deal_asr_msg_by_cmd_id(sys_msg_asr_data_t *asr_msg, cmd_handle_t cmd_handle, uint16_t cmd_id);

/**
 * @brief 按语义ID响应asr消息处理
 * 
 * @param asr_msg 
 * @param cmd_handle 
 * @param semantic_id 
 * @return uint32_t 
 */
uint32_t deal_asr_msg_by_semantic_id(sys_msg_asr_data_t *asr_msg, cmd_handle_t cmd_handle, uint32_t semantic_id);

/**
 * @brief 用户自定义消息处理
 * 
 * @param msg 
 * @return uint32_t 
 */
uint32_t deal_userdef_msg(sys_msg_t *msg);

/**
 * @brief 按键消息处理
 * 
 * @param msg 
 * @return uint32_t 
 */
void userapp_deal_key_msg(sys_msg_key_data_t  *key_msg);

#ifdef __cplusplus
}
#endif

static int power_on_flag = 1; //未上电前为1，上电之后为0

typedef struct
{
    uint16_t wakeup_cmd_id;            /* 唤醒词的命令词id  */
    uint8_t wakeup_net;                /* 唤醒网络         */
}wakeup_data;

extern uint8_t get_wakeup_net(void);
uint8_t check_cmd_id_is_wake_up(uint16_t cmd_id);
typedef struct
{
    uint16_t cmd_id;        /* 命令词id       */
    char     pro_buf[24];       /* 命令词发送协议  */    
    uint8_t  pro_len;       /* 协议长度       */
    uint8_t  paly_type;     /* 主被动播报     */
}send_cmd_pro_t;

typedef struct
{
    uint16_t cmd_id;        /* 命令词id       */
    char     pro_buf[24];   /* 命令词接收协议  */    
    uint8_t  pro_len;       /* 协议长度       */
    uint8_t  paly_type;     /* 音量控制，1最大，2最小，3增大，4减小*/
}recv_cmd_pro_t;

typedef enum
{
    PLAY_MAXIMUM_VOLUME = 1,
    PLAY_MINIMUM_VOLUME,
    PLAY_TURN_UP_VOLUME,
    PLAY_TURN_DOWN_VOLUME,
}voice_type_t;

typedef struct
{
    uint8_t  net_num;           /* 网络序号             */
    char     pro_buf[24];       /* 命令词发送协议        */    
    uint8_t  pro_len;           /* 协议长度             */
    uint16_t cmd_id;            /* 切换指定网络命令词id  */
}wakeup_net_pro_t;

#endif

