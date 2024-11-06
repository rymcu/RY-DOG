

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#include "ci130x_uart.h"
#include "command_info.h"
#include "sdk_default_config.h"


#ifndef _VOICE_MODULE_UART_PROTOCOL_
#define _VOICE_MODULE_UART_PROTOCOL_

#if (UART_PROTOCOL_VER == 2)

#ifdef __cplusplus
extern "C"
{
#endif


#define VMUP_PROTOCOL_VERSION                1
#define VMUP_PROTOCOL_SUBVERSION             0
#define VMUP_PROTOCOL_REVISION               0


#define VMUP_MSG_DATA_MAX_SIZE (20)

#pragma pack(1)
typedef struct
{
    uint16_t header;
    uint16_t data_length;
    uint8_t msg_type;
    uint8_t msg_cmd;
    uint8_t msg_seq;
    uint8_t msg_data[VMUP_MSG_DATA_MAX_SIZE];
    /*uint16_t chksum; send add auto*/
    /*uint8_t tail; send add auto*/
}sys_msg_com_data_t;


#pragma pack()

/*header*/
#define VMUP_MSG_HEAD_LOW  (0xA5)
#define VMUP_MSG_HEAD_HIGH (0xFC)
#define VMUP_MSG_HEAD   ((VMUP_MSG_HEAD_HIGH<<8)|VMUP_MSG_HEAD_LOW)


/*tail*/
#define VMUP_MSG_TAIL   (0xFB)


/*msg_type*/
#define VMUP_MSG_TYPE_CMD_UP   (0xA0)
#define VMUP_MSG_TYPE_CMD_DOWN (0xA1)
#define VMUP_MSG_TYPE_ACK      (0xA2)
#define VMUP_MSG_TYPE_NOTIFY   (0xA3)


/*msg_cmd*/
#define VMUP_MSG_CMD_ASR_RESULT    (0x91)           //报告语音识别结果
#define VMUP_MSG_CMD_PLAY_VOICE    (0x92)           //播放本地播报音
#define VMUP_MSG_CMD_GET_FLASHUID  (0x93)           //读取FLASH的序列号
#define VMUP_MSG_CMD_GET_VERSION   (0x94)           //读取版本号
#define VMUP_MSG_CMD_RESET_MODULE  (0x95)           //复位语音模块
#define VMUP_MSG_CMD_SET_CONFIG    (0x96)           //设置
#define VMUP_MSG_CMD_ENTER_OTA_MODE (0x97)          //进入升级模式
#define VMUP_MSG_CMD_NOTIFY_STATUS (0x9A)           //事件通知
#define VMUP_MSG_CMD_ACK_COMMON    (0xAA)
/* !!! if user want add please add form VMUP_MSG_CMD_USER_START*/
#define VMUP_MSG_CMD_USER_START    (0xB0)


/*msg_data  msg_cmd:VMUP_MSG_CMD_PLAY_VOICE*/
#define VMUP_MSG_DATA_PLAY_START   (0x80)
#define VMUP_MSG_DATA_PLAY_PAUSE   (0x81)
#define VMUP_MSG_DATA_PLAY_RESUME  (0x82)
#define VMUP_MSG_DATA_PLAY_STOP    (0x83)

#define VMUP_MSG_DATA_PLAY_BY_VOICEID (0x90)
#define VMUP_MSG_DATA_PLAY_BY_SEMANTIC_ID   (0x91)
#define VMUP_MSG_DATA_PLAY_BY_CMD_ID   (0x92)


/*msg_data  msg_cmd:VMUP_MSG_CMD_GET_VERSION*/
#define VMUP_MSG_DATA_VER_PROTOCOL   (0x80)         // 串口协议版本号
#define VMUP_MSG_DATA_VER_SDK        (0x81)         // SDK版本号
#define VMUP_MSG_DATA_VER_ASR        (0x82)         // ASR组件版本号
#define VMUP_MSG_DATA_VER_PREPROCESS (0x83)         // 语音预处理算法版本号
#define VMUP_MSG_DATA_VER_PLAYER     (0x84)         // 播放器版本号
#define VMUP_MSG_DATA_VER_APP        (0x8A)         // 应用程序版本号


/*msg_data  msg_cmd:VMUP_MSG_CMD_NOTIFY_STATUS*/
#define VMUP_MSG_DATA_NOTIFY_POWERON     (0xB0)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPENTER (0xB1)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPEXIT  (0xB2)
#define VMUP_MSG_DATA_NOTIFY_PLAYSTART   (0xB3)
#define VMUP_MSG_DATA_NOTIFY_PLAYEND     (0xB4)


/*msg_data msg_cmd:VMUP_MSG_CMD_SET_CONFIG*/
#define VMUP_MSG_CMD_SET_VOLUME        (0x80)
#define VMUP_MSG_CMD_SET_ENTERWAKEUP   (0x81)
#define VMUP_MSG_CMD_SET_PRT_MID_RST   (0x82)
#define VMUP_MSG_CMD_SET_MUTE          (0x83)
#define VMUP_MSG_CMD_SET_NEEDACK       (0x90)
#define VMUP_MSG_CMD_SET_NEEDSTRING    (0x91)


/*ACK error code*/
#define VMUP_MSG_ACK_ERR_NONE          (0x0)
#define VMUP_MSG_ACK_ERR_CHECKSUM      (0xff)
#define VMUP_MSG_ACK_ERR_NOSUPPORT     (0xfe)


void vmup_receive_packet(uint8_t receive_char);
void vmup_communicate_init(void);
void vmup_send_notify(uint8_t notify_event);
void userapp_deal_cmd(sys_msg_com_data_t *msg);
void userapp_deal_com_msg(sys_msg_com_data_t *msg);
void userapp_send_com_test(void);
void vmup_send_asr_result_cmd(cmd_handle_t cmd_handle, uint8_t asr_score);



#ifdef __cplusplus
}
#endif

#endif //#if (UART_PROTOCOL_VER == 2)

#endif

