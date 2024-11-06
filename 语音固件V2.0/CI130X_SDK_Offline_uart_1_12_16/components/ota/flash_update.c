 /**
   ******************************************************************************
   * @file	  flash_update.c 
   * @version V1.0.0
   * @date	  2017.12
   * @brief 
   ******************************************************************************
   **/
#include "flash_update.h"
#include "ci130x_timer.h"
#include "ci130x_spiflash.h"
#include "ci130x_uart.h"
#include "ci130x_scu.h"
#include "ci_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "sdk_default_config.h"

//#include "ci130x_loader.h"

/**************************************************************************
                    type define
****************************************************************************/

#define RECIEVE_BUFFER_COUNT    3


//HEAD and TAIL
#define MSG_HEAD_S 0xa5
#define MSG_HEAD_E 0x0f
#define MSG_HEAD 0x0FA5
#define MSG_TAIL 0xFF

#define SRAM_START_ADDR         0x1FFB0000
#define MAX_SRAM_CODE_SIZE      (1024*(480-32))
#define TCM_START_ADDR          ICACHE_TCM_S
#define TCM_SIZE                (ICACHE_TCM_E - ICACHE_TCM_S + 1)

#define UPDATE_ERASE_64K        (64 * 1024)
#define UPDATE_ERASE_4K         (4 * 1024)

typedef enum 
{
    STATE_HEAD0 = 0x00,
    STATE_HEAD1 = 0x01,
    STAT_LENGTH0 = 0x02,
    STAT_LENGTH1 = 0x03,
    STATE_TYPE = 0x04,
    STATE_CMD = 0x05,
    STATE_NUMBER = 0x06,
    STATE_DATA = 0x07,
    STATE_CRC0 = 0x08,
    STATE_CRC1 = 0x09,
    STATE_TAIL = 0x0a,
}uart_receive_state_t;

#define UPDATE_DEAL_PER_PACKET (4096)
#define MAX_DATA_RECEIVE_PER_PACKET (UPDATE_DEAL_PER_PACKET+4)/*because buffer address is fixed,so now don't big than 8k*/

#define CHAR_BUF_TO_INT_LITTILE(buf) ((buf)[0]|((buf)[1]<<8)|((buf)[2]<<16)|((buf)[3]<<24))

/**************************************************************************
                    fucntion declaration
****************************************************************************/


/**************************************************************************
                    type define
****************************************************************************/
typedef struct recv_buffer_struct
{
    Data_t message;
    struct recv_buffer_struct * next;
    uint8_t * data_buffer;
}recv_buffer_t;


//标志位、计数值
uint32_t success_flag = 0,write_block_addr = 0,write_block_size = 0,offset = 0;
static uint16_t write_block_crc = 0,write_block_crc0 = 0;

//接收、发送数据包
Data_p Send_Box = NULL;
Data_t send_message;
recv_buffer_t rcv_buffer[RECIEVE_BUFFER_COUNT];
recv_buffer_t * p_recv_buffer = NULL;
recv_buffer_t * full_buffer_chain = NULL;
recv_buffer_t * idle_buffer_chain = NULL;

FileConfig_Struct FileConfigInfo;
uint8_t UpdateState;

//#pragma data_alignment = 4
static uint8_t * send_data_buf;

//#pragma data_alignment = 4
static uint8_t * read_buf_for_crc;

recv_buffer_t * get_buffer(recv_buffer_t ** queue)
{
    recv_buffer_t * ret = *queue;
    if (ret)
    {
        *queue = ret->next;
    }
    return ret;
}


void append_buffer(recv_buffer_t ** queue, recv_buffer_t * p_buffer)
{
    recv_buffer_t * p = *queue;

    p_buffer->next = NULL;
    if (p != NULL)
    {
        while(p->next != NULL)
        {
            p = p->next;
        }            
        p->next = p_buffer;
    }
    else
    {
        *queue = p_buffer;
    }    
}


int32_t flash_update_buf_init(void)
{
    Send_Box = &send_message;
    Send_Box->data = send_data_buf;
    
    send_data_buf = pvPortMalloc(UPDATE_DEAL_PER_PACKET);
    read_buf_for_crc = pvPortMalloc(UPDATE_DEAL_PER_PACKET);

    for (int i = 0;i < RECIEVE_BUFFER_COUNT;i++)
    {
        rcv_buffer[i].data_buffer = pvPortMalloc(MAX_DATA_RECEIVE_PER_PACKET);
        rcv_buffer[i].message.data = rcv_buffer[i].data_buffer;
        rcv_buffer[i].next = idle_buffer_chain;
        idle_buffer_chain = &rcv_buffer[i];
    }
    
    p_recv_buffer = get_buffer(&idle_buffer_chain);
    
    return RETURN_OK;
}


static int32_t flash_erase_function(spic_base_t spic,uint32_t addr,uint32_t size)
{
    /* other erase */
    uint32_t current_addr = addr;
    uint32_t current_size = size;
    uint32_t total = size;
    do
    {
        if((0 == (current_addr % UPDATE_ERASE_64K)) && (size >= UPDATE_ERASE_64K))
        {
            current_size = UPDATE_ERASE_64K;
        }
        else if((0 == (current_addr % UPDATE_ERASE_4K)) && (size >= UPDATE_ERASE_4K))
        {
            current_size = UPDATE_ERASE_4K;
        }
        else
        {
            return RETURN_ERR;
        }
        
        if(RETURN_ERR == flash_erase(spic,current_addr,current_size))
        {
            return RETURN_ERR;
        }
        size -= current_size;
        current_addr += current_size;
        send_notify_progress_packet(0,total - size,total);
    }
    while(size > 0);
    return RETURN_OK;
}


//发送函数(轮询方式)
void send_func(void)
{
    //发送
    uint16_t i = 0;
    
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->msg_head);
    UartPollingSenddata(OTA_UPDATE_PORT,(Send_Box->msg_head)>>8);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->length);
    UartPollingSenddata(OTA_UPDATE_PORT,(Send_Box->length)>>8);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->type);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->cmd);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->number);
    while(i != Send_Box->length)
    {
        UartPollingSenddata(OTA_UPDATE_PORT,*(Send_Box->data+i));
        i++;
    }
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->crc);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->crc>>8);
    UartPollingSenddata(OTA_UPDATE_PORT,Send_Box->msg_tail);
}


//接收函数(中断方式)
static uint8_t rev_state = STATE_HEAD0;
static uint16_t length0 = 0,length1 = 0;
static uint16_t crc0 = 0,crc1 = 0;
static uint16_t data_rev_count;

void receive_func(uint8_t receive_char)
{    
    switch(rev_state)
    {
        case STATE_HEAD0://包头验证
            if(receive_char == MSG_HEAD_S)
            {
                if (p_recv_buffer == NULL)
                {
                    p_recv_buffer = get_buffer(&idle_buffer_chain);
                }
                
                if (p_recv_buffer)
                {
                    rev_state = STATE_HEAD1;
                }
            }
            else
            {
                rev_state = STATE_HEAD0;
            }
            break;
        case STATE_HEAD1:
            if(receive_char == MSG_HEAD_E)
            {
                rev_state = STAT_LENGTH0;
                p_recv_buffer->message.msg_head = MSG_HEAD;
            }
            else
            {
                if(MSG_HEAD_S != receive_char)
                {
                    rev_state = STATE_HEAD0;
                }
            }
            break;
        case STAT_LENGTH0://包大小验证
            length0 = receive_char;
            rev_state = STAT_LENGTH1;
            break;
        case STAT_LENGTH1:
            length1   = receive_char;
            length1 <<= 8;
            length1  += length0;
            if(length1 <= MAX_DATA_RECEIVE_PER_PACKET)
            {
                p_recv_buffer->message.length = length1;
                rev_state = STATE_TYPE;
            }
            else
            {
                rev_state = STATE_HEAD0;
            }
            break;
        case STATE_TYPE:
            p_recv_buffer->message.type = receive_char;
            rev_state = STATE_CMD;
            break;
        case STATE_CMD:
            p_recv_buffer->message.cmd = receive_char;
            rev_state = STATE_NUMBER;
            break;
        case STATE_NUMBER:
            p_recv_buffer->message.number = receive_char;
            if(length1>0)
            {
                rev_state = STATE_DATA;
                data_rev_count = 0;
            }
            else
            {
                rev_state = STATE_CRC0;
            }
            break;
        case STATE_DATA:
            p_recv_buffer->message.data[data_rev_count++] = receive_char;
            if(data_rev_count == length1)
            {
                rev_state = STATE_CRC0;
            }
            break;
        case STATE_CRC0:
            crc0 = receive_char;
            rev_state = STATE_CRC1;
            break;
        case STATE_CRC1:
            crc1 = receive_char;
            crc1 <<= 8;
            crc1 += crc0;
            p_recv_buffer->message.crc = crc1;
            if(crc1 == crc_func(crc_func(0,(uint8_t*)&p_recv_buffer->message.type,3),p_recv_buffer->message.data,p_recv_buffer->message.length))
            {
                rev_state = STATE_TAIL;
            }
            else
            {
                rev_state = STATE_HEAD0;
            }
            break;
        case STATE_TAIL:
            if(receive_char == MSG_TAIL)
            {
                p_recv_buffer->message.msg_tail = receive_char;
                append_buffer(&full_buffer_chain, p_recv_buffer);
                p_recv_buffer = get_buffer(&idle_buffer_chain);
            }
            else
            {
                data_rev_count = 0;
            }
            rev_state = STATE_HEAD0;
            break;
        default:
            bootloader_assert(0);
            break;
    }
    
}


//数据包处理函数
void Resolution_func()
{
    recv_buffer_t * p_buffer = get_buffer(&full_buffer_chain);
    Data_t * rev_msg = &p_buffer->message;
    
    //收到ACK确认包
    if(rev_msg->type == MSG_TYPE_ACK)
    {
        if(rev_msg->cmd == MSG_CMD_UPDATE_WRITE)
        {   
            uint32_t temp_offset = offset;

            temp_offset += (rev_msg->length - 4);
            if((write_block_size - temp_offset) >= UPDATE_DEAL_PER_PACKET)
            {
                send_req_update_write_packet(temp_offset,UPDATE_DEAL_PER_PACKET);
            }
            else if(((write_block_size - temp_offset) < UPDATE_DEAL_PER_PACKET)&&((write_block_size - temp_offset) > 0))
            {
                send_req_update_write_packet(temp_offset,write_block_size - temp_offset);
            }

            //写spiflash
            flash_write(FLASH_SPI_PORT,write_block_addr+offset,(unsigned int)(rev_msg->data+4),rev_msg->length-4);
            flash_read(FLASH_SPI_PORT,(unsigned int)read_buf_for_crc,write_block_addr+offset,rev_msg->length-4);
            write_block_crc0 = crc_func(write_block_crc0,read_buf_for_crc,rev_msg->length - 4);
            offset += (rev_msg->length - 4);

            if(offset >= write_block_size)
            {
                send_req_update_block_write_done_packet();
            }
        }
    }
    //收到CMD命令包
    else if(rev_msg->type == MSG_TYPE_CMD)
    {
        if(rev_msg->cmd == MSG_CMD_GET_INFO)
        {
            send_ack_get_info_packet();
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_CHECK_READY)
        {
            send_ack_update_check_ready_packet();
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_BLOCK_INFO)
        {
            //保存 起始地址 和 大小 CRC
            write_block_addr = *(uint32_t*)(rev_msg->data);
            write_block_size = *(uint32_t*)(&rev_msg->data[4]);
            write_block_crc = *(uint16_t*)(&rev_msg->data[8]);
            offset = 0,write_block_crc0 = 0;
            UpdateState = 0;

            flash_read(FLASH_SPI_PORT,(uint32_t)&FileConfigInfo,FILECONFIG_SPIFLASH_START_ADDR,sizeof(FileConfigInfo));
            send_ack_update_block_info_packet();
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_ERA)
        {
            send_ack_update_era_packet();
            if(RETURN_OK != flash_erase_function(FLASH_SPI_PORT,write_block_addr,write_block_size))
            {
                bootloader_assert(0);
                /*need notify error*/
            }            
            /******************************************************/
            send_req_update_write_packet(0,UPDATE_DEAL_PER_PACKET);
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_VERIFY)
        {
            //保存 起始地址 和 大小 CRC
            write_block_addr = *(uint32_t*)(rev_msg->data);
            write_block_size = *(uint32_t*)(&rev_msg->data[4]);
            write_block_crc = *(uint16_t*)(&rev_msg->data[8]);

            bootloader_logdebug("0x%x-------0x%x\n",write_block_crc,write_block_crc0);
            if(write_block_crc == write_block_crc0)
            {
                set_update_complete_status();
                send_ack_update_verify_packet(1);
                success_flag = 0xff;
            }
            else
            {
                send_ack_update_verify_packet(0);
                success_flag = 0;
            }
        }
        else if(rev_msg->cmd == MSG_CMD_TRY_FAST_BD)
        {
            int flag_TXFE;
            int timeout = 100000;
            send_ack_try_fast_bd_packet();
            do{
                flag_TXFE = UART_FLAGSTAT(OTA_UPDATE_PORT, UART_TXFE);
                timeout--;
            }while((!flag_TXFE) && timeout > 0);
            uint32_t bd_rate = CHAR_BUF_TO_INT_LITTILE(rev_msg->data);
            //改变波特率
            UARTInterruptConfig(OTA_UPDATE_PORT,(UART_BaudRate)bd_rate);
        }
        else if(rev_msg->cmd == MSG_CMD_TRY_FAST_BD_TEST)
        {
            uint32_t size = CHAR_BUF_TO_INT_LITTILE(rev_msg->data);
            send_ack_try_fast_bd_test_packet(size);
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_READ) 
        {
            uint32_t ReadStartAddr, ReadSize; 
            ReadStartAddr = *(uint32_t*)(rev_msg->data);
            ReadSize = *(uint32_t*)(&rev_msg->data[4]);
            
            send_ack_update_read_packet(ReadStartAddr,ReadSize);
        }
        else if(rev_msg->cmd == MSG_CMD_UPDATE_COMPLETE)
        {
            send_ack_update_complet_packet(); 
            UpdateState = 1;
        }        
        else if (rev_msg->cmd == MSG_CMD_SYS_RST)
        {
            int flag_TXFE;
            int timeout = 100000;
            send_ack_system_reset();
            do{
                flag_TXFE = UART_FLAGSTAT(OTA_UPDATE_PORT, UART_TXFE);
                timeout--;
            }while((!flag_TXFE) && timeout > 0);
            //Scu_SoftwareRst_System();
            dpmu_software_reset_system_config();
        }        
    }
    
    append_buffer(&idle_buffer_chain, p_buffer);    //put buffer back to idle queue
}


/**************************************************************************
                    crc function
****************************************************************************/
static const uint16_t crc16tab_ccitt[256]= 
{
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};


//CRC校验函数
uint16_t crc_func(uint16_t crc,uint8_t * buf,uint32_t len)
{
    uint32_t counter;
    for( counter = 0; counter < len; counter++)
    crc = (crc<<8) ^ crc16tab_ccitt[((crc>>8) ^ *(uint8_t *)buf++)&0x00FF];
    return crc;
}


void make_packet(uint8_t type, uint8_t cmd, uint8_t number, uint8_t *data, uint16_t data_length)
{
    Send_Box->msg_head = MSG_HEAD;
    Send_Box->length = data_length;
  	Send_Box->type = type;
    Send_Box->cmd = cmd;
    Send_Box->number = number;
    Send_Box->data = data;
    Send_Box->crc = crc_func(crc_func(0,&Send_Box->type,3),Send_Box->data,Send_Box->length);
    Send_Box->msg_tail = MSG_TAIL;
}


void send_req_update_req_packet(void)
{
    make_packet(MSG_TYPE_REQ, MSG_CMD_UPDATE_REQ, 0, send_data_buf, 0);    
    send_func();
}

void send_req_update_write_packet(uint32_t offset,uint32_t size)
{
    *((uint32_t*)&send_data_buf[0]) = offset;
    *((uint32_t*)&send_data_buf[4]) = size;
    make_packet(MSG_TYPE_REQ, MSG_CMD_UPDATE_WRITE, 0, send_data_buf, 8);
    send_func();
}

void send_req_update_write_packet_ex(uint32_t index,uint32_t offset,uint32_t size)
{
    *((uint32_t*)&send_data_buf[0]) = index;
    *((uint32_t*)&send_data_buf[4]) = offset;
    *((uint32_t*)&send_data_buf[8]) = size;    
    make_packet(MSG_TYPE_REQ, MSG_CMD_UPDATE_WRITE, 0, send_data_buf, 12);
  	send_func();
}

void send_req_update_block_write_done_packet(void)
{
    make_packet(MSG_TYPE_REQ, MSG_CMD_UPDATE_BLOCK_WRITE_DONE, 0, send_data_buf, 0);    
    send_func();
}

void send_ack_system_reset(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_SYS_RST, 0, send_data_buf, 0);  
    send_func();
}

/**************************************************************************
      ACK              
****************************************************************************/
void send_ack_get_info_packet(void)
{
    unsigned int FileConfigInfoLen;
    FileConfigInfoLen = sizeof(FileConfig_Struct);

    send_data_buf[0] = BOOT_LOADER_NEW_RELEASE;
    send_data_buf[1] = BOOT_LOADER_NEW_MINOR;
    send_data_buf[2] = BOOT_LOADER_NEW_MAJOR;
    send_data_buf[3] = 0;

    //读spiflash Send_Box->data[4]~[7]
    spic_read_jedec_id(FLASH_SPI_PORT,&send_data_buf[4]);
    //Read File configuration
    flash_read(FLASH_SPI_PORT,(uint32_t)&FileConfigInfo,FILECONFIG_SPIFLASH_START_ADDR,FileConfigInfoLen);
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&send_data_buf[8],(char*)&FileConfigInfo,FileConfigInfoLen);
    spic_read_unique_id(FLASH_SPI_PORT, &send_data_buf[8+FileConfigInfoLen]);
    make_packet(MSG_TYPE_ACK, MSG_CMD_GET_INFO, 0, send_data_buf, MSG_LEN_GET_INFO+FileConfigInfoLen + UNIQUE_ID_LENGTH);

    send_func();
}

void send_ack_update_check_ready_packet(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_CHECK_READY, 0, send_data_buf, 0);
    send_func();
}

void send_ack_update_block_info_packet(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_BLOCK_INFO, 0, send_data_buf, 0);
    send_func();
}

void send_ack_update_era_packet(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_ERA, 0, send_data_buf, 0);
    send_func();
}

void send_ack_update_verify_packet(uint8_t verify)
{
    send_data_buf[0] = verify;
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_VERIFY, 0, send_data_buf, 1);
    send_func();
}

void send_ack_try_fast_bd_packet(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_TRY_FAST_BD, 0, send_data_buf, 0);
    send_func();
}

void send_ack_try_fast_bd_test_packet(uint32_t size)
{
    send_data_buf[0] = 0x1;
    for(uint32_t i = 1;i < (size+1);i++)
    {
        send_data_buf[i] = 0xf0;
    }
    make_packet(MSG_TYPE_ACK, MSG_CMD_TRY_FAST_BD_TEST, 0, send_data_buf, size + 1);
  	send_func();
}

void send_ack_update_read_packet(uint32_t StartAddr, uint32_t Size)
{
    *((uint32_t*)&send_data_buf[0]) = StartAddr;
    flash_read(FLASH_SPI_PORT,(uint32_t)&send_data_buf[4],StartAddr,Size);
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_READ, 0, send_data_buf, Size + sizeof(uint32_t));
    send_func();
}

void send_ack_update_complet_packet(void)
{
    make_packet(MSG_TYPE_ACK, MSG_CMD_UPDATE_COMPLETE, 0, send_data_buf, 0);
    send_func();
}

void send_notify_progress_packet(int index, int current, int total)
{
    MASK_ROM_LIB_FUNC->newlibcfunc.sprintf_p((char*)send_data_buf, "%d:%d/%d", index,current,total);
    make_packet(MSG_TYPE_NOTIFY, MSG_CMD_UPDATE_PROGRESS, 0, send_data_buf, MASK_ROM_LIB_FUNC->newlibcfunc.strlen_p((char*)send_data_buf)+1);
    send_func();
}

int32_t get_update_state(void)
{
    if(1 == UpdateState)
    {
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}

void set_update_complete_status(void)
{
    unsigned int FileConfigInfoLen;
    unsigned char FileUpdateFlag;
    FileUpdateFlag = 0x00;
    FileConfigInfoLen = sizeof(FileConfig_Struct);

    flash_read(FLASH_SPI_PORT,(uint32_t)&FileConfigInfo,FILECONFIG_SPIFLASH_START_ADDR,FileConfigInfoLen);
    if(write_block_addr == FileConfigInfo.UserCode1StartAddr)
    {
        FileConfigInfo.UserCode1CompltStatus = USER_CODE_AREA_STA_OK;
        FileConfigInfo.UserCode2CompltStatus = USER_CODE_AREA_STA_OLD;
        FileUpdateFlag = 0x01;
    }
    else if(write_block_addr == FileConfigInfo.UserCode2StartAddr)
    {
        FileConfigInfo.UserCode2CompltStatus = USER_CODE_AREA_STA_OK;
        FileConfigInfo.UserCode1CompltStatus = USER_CODE_AREA_STA_OLD;
        FileUpdateFlag = 0x01;
    }
    else if(write_block_addr == FileConfigInfo.ASRCMDModelStartAddr)
    {
        FileConfigInfo.ASRCMDModelCompltStatus = CMPT_STATUS_OK;
        FileUpdateFlag = 0x01;
    }
    else if(write_block_addr == FileConfigInfo.DNNModelStartAddr)
    {
        FileConfigInfo.DNNModelCompltStatus = CMPT_STATUS_OK;
        FileUpdateFlag = 0x01;
    }
    else if(write_block_addr == FileConfigInfo.VoicePlayingStartAddr)
    {
        FileConfigInfo.VoicePlayingCompltStatus = CMPT_STATUS_OK;
        FileUpdateFlag = 0x01;
    }
    else if(write_block_addr == FileConfigInfo.UserFileStartAddr)
    {
        FileConfigInfo.UserFileCompltStatus = CMPT_STATUS_OK;
        FileUpdateFlag = 0x01;
    }
    if(0x01 == FileUpdateFlag)
    {
        flash_write(FLASH_SPI_PORT,FILECONFIG_SPIFLASH_START_ADDR,(unsigned int)&FileConfigInfo,FileConfigInfoLen);
    }
}

int32_t have_a_new_message(void)
{
    if(full_buffer_chain != NULL)
    {
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}



