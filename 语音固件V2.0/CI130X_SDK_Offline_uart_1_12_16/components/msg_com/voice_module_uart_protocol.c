


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "sdk_default_config.h"

#include "voice_module_uart_protocol.h"

#include "system_msg_deal.h"
#include "command_info.h"
#include "ci130x_spiflash.h"
#include "ci130x_dpmu.h"
#include "prompt_player.h"
#include "audio_play_api.h"
// #include "flash_rw_process.h"
#include "asr_api.h"
#include "firmware_updater.h"
#include <string.h>
#include <stdlib.h>
#include "romlib_runtime.h"
#include "flash_manage_outside_port.h"
#include "baudrate_calibrate.h"

#if (UART_PROTOCOL_VER == 2)

#define TIMEOUT_ONE_PACKET_INTERVAL (1000)/*ms, in this code, it should be bigger than portTICK_PERIOD_MS */
#define MAX_DATA_RECEIVE_PER_PACKET (80)/*???*/


#define VMUP_PACKET_MIN_SIZE (8)
#define VMUP_PACKET_MAX_SIZE (VMUP_MSG_DATA_MAX_SIZE + VMUP_PACKET_MIN_SIZE)


uint8_t glb_send_sequence;

extern void  resume_voice_in(void);
extern void  pause_voice_in(void);
extern void set_state_enter_wakeup(uint32_t exit_wakup_ms);


/********************************************************************
                     port function
********************************************************************/
static SemaphoreHandle_t send_packet_mutex;

#define log_debug(fmt, args...) do{}while(0)
#if USE_STD_PRINTF
#define log_info printf
#define log_error printf
#else
#define log_info _printf
#define log_error _printf
#endif
#define cinv_assert(x) do{}while(0)

static uint32_t baud_sync_req;

static void vmup_port_mutex_creat(void)
{
    send_packet_mutex = xSemaphoreCreateMutex();

    if (NULL == send_packet_mutex) 
    {
        log_error("%s, error\n",__func__);
    }
    log_debug("%s\n",__func__);
}


static void vmup_port_mutex_take(void)
{
    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) 
    {
        if (xSemaphoreTake(send_packet_mutex, portMAX_DELAY) == pdFALSE)
        {
            log_error("%s, error\n",__func__);
        }
        log_debug("%s\n",__func__);
    }
}


static void vmup_port_mutex_give(void)
{
    if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) 
    {
        if (xSemaphoreGive(send_packet_mutex) == pdFALSE) 
        {
            log_error("%s, error\n",__func__);
        }
        log_debug("%s\n",__func__);
    }
}


int vmup_port_send_packet_rev_msg(sys_msg_com_data_t *msg)
{
    sys_msg_t send_msg;
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    send_msg.msg_type = SYS_MSG_TYPE_COM;
    if(sizeof(sys_msg_com_data_t) > sizeof(send_msg.msg_data))
    {
        CI_ASSERT(0,"\n");
    }                             
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)(&send_msg.msg_data), msg, sizeof(sys_msg_com_data_t));
    
    xResult = send_msg_to_sys_task(&send_msg,&xHigherPriorityTaskWoken);

    if((xResult != pdFAIL)&&(pdTRUE == xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken;
}


TickType_t last_time;
static bool vmup_port_timeout_one_packet(void)
{
    TickType_t now_time;
    TickType_t timeout;
    
    now_time = xTaskGetTickCountFromISR();

    timeout = (now_time - last_time);/*uint type, so overflow just used - */

    last_time = now_time;

    if(timeout > TIMEOUT_ONE_PACKET_INTERVAL/portTICK_PERIOD_MS) /*also as timeout = timeout*portTICK_PERIOD_MS;*/
    {
        return true;
    }
    else
    {
        return false;
    }
}


static uint16_t vmup_port_checksum(uint16_t init_val, uint8_t * data, uint16_t length)
{
    uint32_t i;
    uint16_t chk_sum = init_val;
    for(i=0; i<length; i++)
    {
        chk_sum += data[i];
    }

    return chk_sum;
}


static int vmup_port_send_char(uint8_t ch)
{
    UartPollingSenddata((UART_TypeDef *)UART_PROTOCOL_NUMBER, ch);
    return RETURN_OK;
}

static void vump_uart_irq_handler(void)
{
    /*发送数据*/
    if (((UART_TypeDef*)UART_PROTOCOL_NUMBER)->UARTMIS & (1UL << UART_TXInt))
    {
        UART_IntClear((UART_TypeDef*)UART_PROTOCOL_NUMBER,UART_TXInt);
    }
    /*接受数据*/
    if (((UART_TypeDef*)UART_PROTOCOL_NUMBER)->UARTMIS & (1UL << UART_RXInt))
    {
        //here FIFO DATA must be read out
        vmup_receive_packet(UART_RXDATA((UART_TypeDef*)UART_PROTOCOL_NUMBER));
        UART_IntClear((UART_TypeDef*)UART_PROTOCOL_NUMBER,UART_RXInt);
    }

    UART_IntClear((UART_TypeDef*)UART_PROTOCOL_NUMBER,UART_AllInt);
}


static int vmup_port_hw_init(void)
{
#if HAL_UART0_BASE == UART_PROTOCOL_NUMBER
    __eclic_irq_set_vector(UART0_IRQn, (int32_t)vump_uart_irq_handler);
#elif (HAL_UART1_BASE == UART_PROTOCOL_NUMBER)
    __eclic_irq_set_vector(UART1_IRQn, (int32_t)vump_uart_irq_handler);
#else
    __eclic_irq_set_vector(UART2_IRQn, (int32_t)vump_uart_irq_handler);
#endif
    UARTInterruptConfig((UART_TypeDef *)UART_PROTOCOL_NUMBER, UART_PROTOCOL_BAUDRATE);
    return RETURN_OK;
}

/********************************************************************
                     receive function
********************************************************************/
/*receive use state machine method, so two char can arrive different time*/
typedef enum 
{
    REV_STATE_HEAD0   = 0x00,
    REV_STATE_HEAD1   = 0x01,
    REV_STATE_LENGTH0 = 0x02,
    REV_STATE_LENGTH1 = 0x03,
    REV_STATE_TYPE    = 0x04,
    REV_STATE_CMD     = 0x05,
    REV_STATE_SEQ     = 0x06,
    REV_STATE_DATA    = 0x07,
    REV_STATE_CKSUM0  = 0x08,
    REV_STATE_CKSUM1  = 0x09,
    REV_STATE_TAIL    = 0x0a,
}vmup_receive_state_t;

sys_msg_com_data_t recever_packet;
static uint8_t rev_state = REV_STATE_HEAD0;
static uint16_t length0 = 0, length1 = 0;
static uint16_t chk_sum0 = 0, chk_sum1 = 0;
static uint16_t data_rev_count = 0;


void vmup_receive_packet(uint8_t receive_char)
{
    if(true == vmup_port_timeout_one_packet())
    {
        rev_state = REV_STATE_HEAD0;
    }

    switch(rev_state)
    {
        case REV_STATE_HEAD0:
            if(VMUP_MSG_HEAD_LOW == receive_char)
            {
                rev_state = REV_STATE_HEAD1;
            }
            else
            {
                rev_state = REV_STATE_HEAD0;
            }
            break;
        case REV_STATE_HEAD1:
            if(VMUP_MSG_HEAD_HIGH == receive_char)
            {
                rev_state = REV_STATE_LENGTH0;
                recever_packet.header = VMUP_MSG_HEAD;
            }
            else
            {
                if(VMUP_MSG_HEAD_LOW != receive_char)
                {
                    rev_state = REV_STATE_HEAD0;
                }
            }
            break;
        case REV_STATE_LENGTH0:
            length0 = receive_char;
            rev_state = REV_STATE_LENGTH1;
            break;
        case REV_STATE_LENGTH1:
            length1 = receive_char;
            length1 <<= 8;
            length1 += length0;
            if(length1 <= (VMUP_MSG_DATA_MAX_SIZE-10))
            {
                recever_packet.data_length = length1;
                rev_state = REV_STATE_TYPE;
            }
            else
            {
                rev_state = REV_STATE_HEAD0;
            }
            break;
        case REV_STATE_TYPE:
            recever_packet.msg_type = receive_char;
            rev_state = REV_STATE_CMD;
            break;
        case REV_STATE_CMD:
            recever_packet.msg_cmd = receive_char;
            rev_state = REV_STATE_SEQ;
            break;
        case REV_STATE_SEQ:
            recever_packet.msg_seq = receive_char;
            if(length1 > 0)
            {
                rev_state = REV_STATE_DATA;
                data_rev_count = 0;
            }
            else
            {
                rev_state = REV_STATE_CKSUM0;
            }
            break;
        case REV_STATE_DATA:
            recever_packet.msg_data[data_rev_count++] = receive_char;
            if(data_rev_count == length1)
            {
                rev_state = REV_STATE_CKSUM0;
            }
            break;
        case REV_STATE_CKSUM0:
            chk_sum0 = receive_char;
            rev_state = REV_STATE_CKSUM1;
            break;
        case REV_STATE_CKSUM1:
        {
            uint16_t packet_chk_sum;
            chk_sum1 = receive_char;
            chk_sum1 <<= 8;
            chk_sum1 += chk_sum0;
            /*recever_packet->chksum = chk_sum1; just used as judgement*/
            packet_chk_sum = vmup_port_checksum(0, (uint8_t*)&recever_packet.msg_type, 3);
            packet_chk_sum = vmup_port_checksum(packet_chk_sum, recever_packet.msg_data, recever_packet.data_length);            
            if(chk_sum1 == packet_chk_sum)
            {
                rev_state = REV_STATE_TAIL;
            }
            else
            {
                rev_state = REV_STATE_HEAD0;
            }
            break;
        }
        case REV_STATE_TAIL:
            if(receive_char == VMUP_MSG_TAIL)/*receive ok*/
            {
                /*recever_packet->tail = receive_char; just used as judgement*/
                vmup_port_send_packet_rev_msg(&recever_packet);
            }
            else
            {
                data_rev_count = 0;
            }
            rev_state = REV_STATE_HEAD0;
            break;
        default:
            rev_state = REV_STATE_HEAD0;
            break;
    }
    
}


/********************************************************************
                     send function
********************************************************************/
static int vmup_send_packet(sys_msg_com_data_t * msg)
{
    uint16_t chk_sum;
    uint8_t *buf = (uint8_t*)msg;
    int i;
    
    if(msg == NULL)
    {
        return RETURN_ERR;
    }        

    vmup_port_mutex_take();
        
    /*check sum*/
    chk_sum = vmup_port_checksum(0, (uint8_t*)&msg->msg_type, 3);
    chk_sum = vmup_port_checksum(chk_sum, msg->msg_data, msg->data_length);
    
    /*header and data*/    
    for(i = 0;i < msg->data_length + 7; i++)
    {
        vmup_port_send_char( buf[i] );
    }

    vmup_port_send_char( chk_sum&0xff );
    vmup_port_send_char( (chk_sum>>8)&0xff );

    /*tail*/
    vmup_port_send_char( VMUP_MSG_TAIL );

    vmup_port_mutex_give();

    return RETURN_OK;
}

#if UART_BAUDRATE_CALIBRATE
void send_baudrate_sync_req(void)
{
    sys_msg_com_data_t msg;
    uint32_t *sendid = (uint32_t *)msg.msg_data;

    memset(msg.msg_data,0,8);
    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 7;
    msg.msg_type = VMUP_MSG_TYPE_CMD_UP;
    msg.msg_cmd = VMUP_MSG_CMD_ASR_RESULT;
    msg.msg_seq = glb_send_sequence++;
    
    *sendid = baud_sync_req;
    msg.msg_data[6] = 0;

    vmup_send_packet(&msg);
}
#endif


void vmup_communicate_init(void)
{
    vmup_port_hw_init();
    vmup_port_mutex_creat();

    #if UART_BAUDRATE_CALIBRATE
    cmd_handle_t cmd_handle = cmd_info_find_command_by_string("<baud_sync_req>");
    baud_sync_req = cmd_info_get_command_id(cmd_handle);
    ci_loginfo(CI_LOG_INFO, "baud_sync_req:%08x\n", baud_sync_req);

    baudrate_calibrate_init(UART_PROTOCOL_NUMBER, UART_PROTOCOL_BAUDRATE, send_baudrate_sync_req);          // 初始化波特率校准
    #endif
}


void vmup_send_notify(uint8_t notify_event)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 2;
    msg.msg_type = VMUP_MSG_TYPE_NOTIFY;
    msg.msg_cmd = VMUP_MSG_CMD_NOTIFY_STATUS;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = 0;
    msg.msg_data[1] = notify_event;

    vmup_send_packet(&msg);
}


void vmup_send_ack_common_err(uint8_t sequence, uint8_t cmd, uint8_t err_no)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 2;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_ACK_COMMON;
    msg.msg_seq = sequence;
    msg.msg_data[0] = err_no;
    msg.msg_data[1] = cmd;
    vmup_send_packet(&msg);
}         


void vmup_send_asr_result_cmd(cmd_handle_t cmd_handle, uint8_t asr_score)
{
    sys_msg_com_data_t msg;
    uint32_t *sendid = (uint32_t *)msg.msg_data;

    memset(msg.msg_data,0,8);
    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 7;
    msg.msg_type = VMUP_MSG_TYPE_CMD_UP;
    msg.msg_cmd = VMUP_MSG_CMD_ASR_RESULT;
    msg.msg_seq = glb_send_sequence++;
    *sendid = cmd_info_get_semantic_id(cmd_handle);
    msg.msg_data[6] = asr_score;

    vmup_send_packet(&msg);
}


void vmup_send_asr_result_ack(uint8_t sequence)
{
    sys_msg_com_data_t msg;
    
    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_ASR_RESULT;
    msg.msg_seq = sequence;
    msg.msg_data[0] = VMUP_MSG_ACK_ERR_NONE;
    vmup_send_packet(&msg);
}


void vmup_send_play_voice_cmd(uint8_t play_control, uint8_t play_setting, uint32_t play_id)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 6;
    msg.msg_type = VMUP_MSG_TYPE_CMD_DOWN;
    msg.msg_cmd = VMUP_MSG_CMD_PLAY_VOICE;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = play_control;
    msg.msg_data[1] = play_setting;
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&msg.msg_data[2], &play_id, sizeof(uint32_t));

    vmup_send_packet(&msg);
}


void vmup_send_play_voice_ack(uint8_t sequence,uint8_t status)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_PLAY_VOICE;
    msg.msg_seq = sequence;
    msg.msg_data[0] = status;

    vmup_send_packet(&msg);
}


void vmup_send_unique_id_cmd(void)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_CMD_DOWN;
    msg.msg_cmd = VMUP_MSG_CMD_GET_FLASHUID;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = 0x80;

    vmup_send_packet(&msg);
}


void vmup_send_unique_id_ack(uint8_t sequence,uint8_t *unique_id)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 16;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_GET_FLASHUID;
    msg.msg_seq = sequence;
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(msg.msg_data, unique_id, 16);

    vmup_send_packet(&msg);
}


void vmup_send_get_ver_cmd(uint8_t ver_type)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_CMD_DOWN;
    msg.msg_cmd = VMUP_MSG_CMD_GET_VERSION;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = ver_type;

    vmup_send_packet(&msg);
}


void vmup_send_get_ver_ack(uint8_t sequence,uint8_t *ver)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 4;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_GET_VERSION;
    msg.msg_seq = sequence;
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(msg.msg_data, ver, 4);

    vmup_send_packet(&msg);
}


void vmup_send_reset_cmd(void)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 5;
    msg.msg_type = VMUP_MSG_TYPE_CMD_DOWN;
    msg.msg_cmd = VMUP_MSG_CMD_RESET_MODULE;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = 'r';
    msg.msg_data[1] = 'e';
    msg.msg_data[2] = 's';
    msg.msg_data[3] = 'e';
    msg.msg_data[4] = 't';

    vmup_send_packet(&msg);
}


void vmup_send_reset_ack(uint8_t sequence)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_RESET_MODULE;
    msg.msg_seq = sequence;
    msg.msg_data[0] = VMUP_MSG_ACK_ERR_NONE;
    vmup_send_packet(&msg);
}


void vmup_send_setting_cmd(uint8_t set_type,uint32_t set_value)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 5;
    msg.msg_type = VMUP_MSG_TYPE_CMD_DOWN;
    msg.msg_cmd = VMUP_MSG_CMD_SET_CONFIG;
    msg.msg_seq = glb_send_sequence++;
    msg.msg_data[0] = set_type;
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&msg.msg_data[1], &set_value, sizeof(set_value));

    vmup_send_packet(&msg);
}


void vmup_send_setting_ack(uint8_t sequence, uint8_t err_no)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_SET_CONFIG;
    msg.msg_seq = sequence;
    msg.msg_data[0] = err_no;
    vmup_send_packet(&msg);
}

void vmup_send_enter_ota_ack(uint8_t sequence, uint8_t err_no)
{
    sys_msg_com_data_t msg;

    msg.header = VMUP_MSG_HEAD;
    msg.data_length = 1;
    msg.msg_type = VMUP_MSG_TYPE_ACK;
    msg.msg_cmd = VMUP_MSG_CMD_ENTER_OTA_MODE;
    msg.msg_seq = sequence;
    msg.msg_data[0] = err_no;
    vmup_send_packet(&msg);
}

/********************************************************************
                     user add function
********************************************************************/

void play_voice_callback(cmd_handle_t cmd_handle)
{
    resume_voice_in();
    
    vmup_send_notify(VMUP_MSG_DATA_NOTIFY_PLAYEND);
}

void userapp_deal_cmd(sys_msg_com_data_t *msg)
{
    switch(msg->msg_cmd)
    {
        case VMUP_MSG_CMD_PLAY_VOICE:
        {
            uint32_t id;
            uint8_t play_control;
            uint8_t play_setting;
            id = *((uint32_t *)(&msg->msg_data[2]));
            play_control = *((uint8_t *)(msg->msg_data));
            play_setting = *((uint8_t *)(&msg->msg_data[1]));
            if(VMUP_MSG_DATA_PLAY_START == play_control)
            {
                vmup_send_notify(VMUP_MSG_DATA_NOTIFY_PLAYSTART);

                /*play voice*/
                if (VMUP_MSG_DATA_PLAY_BY_SEMANTIC_ID == play_setting)
                {
                    pause_voice_in();
                    prompt_play_by_semantic_id(id, -1, play_voice_callback, false);
					update_awake_time();
                }
                else if (VMUP_MSG_DATA_PLAY_BY_CMD_ID == play_setting)
                { 
                    pause_voice_in();
                    prompt_play_by_cmd_id(id,-1,play_voice_callback,false);
					update_awake_time();
                }
                else if (VMUP_MSG_DATA_PLAY_BY_VOICEID == play_setting)
                {
                }
                vmup_send_play_voice_ack(msg->msg_seq,play_control);
            }
            else if (VMUP_MSG_DATA_PLAY_STOP == play_control)
            {
                stop_play(NULL,NULL);
            }
            else if (VMUP_MSG_DATA_PLAY_PAUSE == play_control)
            {
                pause_play(NULL,NULL);
            }
            else if(VMUP_MSG_DATA_PLAY_RESUME == play_control)
            {
                continue_history_play(NULL);
            }
            break;
        }
        case VMUP_MSG_CMD_GET_FLASHUID:
        {
            uint8_t flash_unique_id[20];
            /*get flash id*/
            // requset_flash_ctl();
            post_spic_read_unique_id((uint8_t*)flash_unique_id);
            // release_flash_ctl();        
            vmup_send_unique_id_ack(msg->msg_seq, flash_unique_id);
            break;
        }
        case VMUP_MSG_CMD_GET_VERSION:
        {
            uint8_t ver_type;
            uint8_t ver[4];
            ver_type = *((uint8_t *)msg->msg_data);
            ver[0] = 'v';
            switch(ver_type)
            {
            case VMUP_MSG_DATA_VER_PROTOCOL:
                ver[1] = VMUP_PROTOCOL_VERSION;
                ver[2] = VMUP_PROTOCOL_SUBVERSION;
                ver[3] = VMUP_PROTOCOL_REVISION;
                break;
            case VMUP_MSG_DATA_VER_SDK:             // SDK版本号
                ver[1] = SDK_VERSION;
                ver[2] = SDK_SUBVERSION;
                ver[3] = SDK_REVISION;
                break;
            case VMUP_MSG_DATA_VER_ASR:             // ASR组件版本号
                {
                    char *p;
                    int i;
                    char asr_ver_buf[100]={0};
                    get_asr_sys_verinfo(asr_ver_buf);
                    for (i = 0;i < 3;i++)
                    {
                        p = strrchr(asr_ver_buf, '.');
                        ver[3 - i] = atoi(p+1);
                        *p = '\0';
                    }
                }
                break;
            case VMUP_MSG_DATA_VER_PREPROCESS:      // 语音预处理算法版本号 //TODO
                ver[1] = 'x';
                ver[2] = 'x';
                ver[3] = 'x';
                break;
            case VMUP_MSG_DATA_VER_PLAYER:          // 播放器版本号
                get_audio_play_version((char*)&ver[1]);
                break;
            case VMUP_MSG_DATA_VER_APP:             // 应用程序版本号
                ver[1] = USER_VERSION_MAIN_NO;
                ver[2] = USER_VERSION_SUB_NO;
                ver[3] = 0;
                break;
            default:
			break;
            }
            vmup_send_get_ver_ack(msg->msg_seq, ver);
            break;
        }
        case VMUP_MSG_CMD_RESET_MODULE:
            vmup_send_reset_ack(msg->msg_seq);
            UartPollingSenddone((UART_TypeDef*)UART_PROTOCOL_NUMBER);
            //Scu_SoftwareRst_System();
            dpmu_software_reset_system_config();
            break;
        case VMUP_MSG_CMD_SET_CONFIG:
        {
            uint8_t set_type;
            set_type = *((uint8_t *)msg->msg_data);
            if(VMUP_MSG_CMD_SET_VOLUME == set_type)
            {
                /*set volume*/
                audio_play_set_vol_gain(100*msg->msg_data[1]/20);
                vmup_send_setting_ack(msg->msg_seq, VMUP_MSG_ACK_ERR_NONE);
            }
            else if(VMUP_MSG_CMD_SET_ENTERWAKEUP == set_type)
            {
                /*
                sys_msg_t send_msg;
                send_msg.msg_type = SYS_MSG_TYPE_CMD_INFO;
                send_msg.msg_data.cmd_info_data.cmd_info_status = MSG_CMD_INFO_STATUS_POST_CHANGE_ASR_NORMAL_WORD;
                send_msg_to_sys_task(&send_msg, NULL);
                set_state_enter_wakeup(EXIT_WAKEUP_TIME); */
                enter_wakeup_deal(EXIT_WAKEUP_TIME ,INVALID_HANDLE);
            }
            else if (VMUP_MSG_CMD_SET_MUTE == set_type)
            {
                if (msg->msg_data[1] == 0)
                {
                    prompt_player_enable(ENABLE);
                }
                else
                {
                    prompt_player_enable(DISABLE);
                }
            }
            break;
        }
        default:
            vmup_send_ack_common_err(msg->msg_seq,msg->msg_cmd,VMUP_MSG_ACK_ERR_NOSUPPORT);
            break;
    }

}


void userapp_deal_com_msg(sys_msg_com_data_t *msg)
{    

    switch(msg->msg_type)
    {
        case VMUP_MSG_TYPE_CMD_DOWN:
            userapp_deal_cmd(msg);
            break;
        #if UART_BAUDRATE_CALIBRATE
        case VMUP_MSG_TYPE_CMD_UP:
            if (msg->msg_cmd == VMUP_MSG_CMD_ASR_RESULT)
            {
                uint32_t id = *((uint32_t *)(msg->msg_data));
                if (id == baud_sync_req)
                {
                    baudrate_calibrate_set_ack();
                }
            }
            break;
        #else
        case VMUP_MSG_TYPE_CMD_UP:
            break;  
        #endif
        case VMUP_MSG_TYPE_ACK:
            break;
        case VMUP_MSG_TYPE_NOTIFY:
            break;  
        default:
            vmup_send_ack_common_err(msg->msg_seq,msg->msg_cmd,VMUP_MSG_ACK_ERR_NOSUPPORT);
            break;
    }
}


void userapp_send_com_test(void)
{

    vmup_send_asr_result_cmd(0,26);

    vmup_send_get_ver_cmd(VMUP_MSG_DATA_VER_PROTOCOL);

    vmup_send_play_voice_cmd(VMUP_MSG_DATA_PLAY_START, VMUP_MSG_DATA_PLAY_BY_VOICEID, 32);

    vmup_send_unique_id_cmd();

    vmup_send_reset_cmd();

    vmup_send_setting_cmd(VMUP_MSG_CMD_SET_VOLUME,15);

}

#endif //(UART_PROTOCOL_VER == 2)
