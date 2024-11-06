 /**
   ******************************************************************************
   * @file	  flash_update.h 
   * @version V1.0.0
   * @date	  2017.12
   * @brief 
   ******************************************************************************
   **/
#ifndef __FLASH_UPDATE__
#define __FLASH_UPDATE__

#include "stdint.h"
#include "ci130x_spiflash.h"

/**************************************************************************
                    debug macro
****************************************************************************/
//#define BOOTLOADER_DEBUG_BY_UART
//#define SRAM_DebugMode                  //Need switch *.icf between xxx_SRAM.icf & xxx_SDRAM.icf files in project path

#define FLASH_SPI_PORT          QSPI0

#ifdef BOOTLOADER_DEBUG_BY_UART
enum BOOTLOADER_LOG_LEVEL
{
    BOOTLOADER_LOG_ERROR = 0,
    BOOTLOADER_LOG_WARN,
    BOOTLOADER_LOG_DEBUG,
    BOOTLOADER_LOG_INFO,
};


#define BOOTLOADER_DEBUG_LEVEL BOOTLOADER_LOG_INFO

/*need open SecureCRT ansi color*/
#define bootloader_loginfo(fmt, args...) do { if (BOOTLOADER_DEBUG_LEVEL>=BOOTLOADER_LOG_INFO) mprintf(fmt, ## args); } while(0)
#define bootloader_logdebug(fmt, args...) do { if (BOOTLOADER_DEBUG_LEVEL>=BOOTLOADER_LOG_DEBUG) mprintf(fmt, ## args); } while(0)
#define bootloader_logwar(fmt, args...) do { if (BOOTLOADER_DEBUG_LEVEL>=BOOTLOADER_LOG_WARN) mprintf("\x1b[35m"fmt"\x1b[0m", ## args); } while(0)
#define bootloader_logerr(fmt, args...) do { if (BOOTLOADER_DEBUG_LEVEL>=BOOTLOADER_LOG_ERROR) mprintf("\x1b[31m"fmt"\x1b[0m", ## args); } while(0)

#define bootloader_assert(x) if( ( x ) == 0 ) { while(1); }
#else
#define bootloader_loginfo(fmt, args...) do{}while(0)
#define bootloader_logdebug(fmt, args...) do{}while(0)
#define bootloader_logwar(fmt, args...) do{}while(0)
#define bootloader_logerr(fmt, args...) do{}while(0)
#define bootloader_assert(x) do{}while(0)
#endif


/*updater vesion define*/
#define BOOT_LOADER_NEW_STR     "V20102"
#define BOOT_LOADER_NEW_MAJOR   0x02
#define BOOT_LOADER_NEW_MINOR   0x01
#define BOOT_LOADER_NEW_RELEASE 0x02


#define USER_CODE_AREA_STA_UPDATE       0xFC
#define USER_CODE_AREA_STA_OK           0xF0
#define USER_CODE_AREA_STA_OLD          0xC0

/**************************************************************************
                    type define
****************************************************************************/
//#define DEBUG

//TYPE
#define MSG_TYPE_CMD	                    0xA0
#define MSG_TYPE_REQ	                    0xA1
#define MSG_TYPE_ACK	                    0xA2
#define MSG_TYPE_NOTIFY	                    0xA3
//CMD
#define MSG_CMD_UPDATE_REQ                  0x03
#define MSG_CMD_GET_INFO                    0x04
#define MSG_CMD_UPDATE_CHECK_READY          0x05
#define MSG_CMD_UPDATE_BLOCK_INFO           0x06
#define MSG_CMD_UPDATE_ERA                  0x07
#define MSG_CMD_UPDATE_WRITE                0x08
#define MSG_CMD_UPDATE_BLOCK_WRITE_DONE     0x09
#define MSG_CMD_UPDATE_VERIFY               0x0A
#define MSG_CMD_TRY_FAST_BD                 0x0b
#define MSG_CMD_TRY_FAST_BD_TEST            0x0c
#define MSG_CMD_UPDATE_READ                 0x0d
#define MSG_CMD_UPDATE_COMPLETE             0x0e
#define MSG_CMD_UPDATE_EXTERNAL_DEV         0x10
#define MSG_CMD_UPDATE_PROGRESS             0x11

#define MSG_CMD_TESTCMD                     0xA0        //测试命令
#define MSG_CMD_SYS_RST                     0xA1        //复位命令

//LENGTH
#define MSG_LEN_UPDATE_REQ                  0x00    //0x01
#define MSG_LEN_GET_INFO                    0x08
#define MSG_LEN_UPDATE_CHECK_READY          0x00
#define MSG_LEN_UPDATE_BLOCK_INFO           0x00
#define MSG_LEN_UPDATE_ERA                  0x00
#define MSG_LEN_UPDATE_WRITE                0x08
#define MSG_LEN_UPDATE_BLOCK_WRITE_DONE     0x00
#define MSG_LEN_UPDATE_VERIFY               0x01
#define MSG_LEN_TRY_FAST_BD                 0x04
#define MSG_LEN_TRY_FAST_BD_TEST            0x00    //Input size
#define MSG_LEN_UPDATE_READ                 0x00    //Input size +4
#define MSG_LEN_UPDATE_COMPLETE             0x01

#define FILECONFIG_SPIFLASH_START_ADDR      (0x8000)
#define FILECONFIG_SPIFLASH_SIZE            (4096)

#define USERCODE_MAX_SIZE                   (1024*448)
#define USERCODE_PER_SIZE                   (4096)

#define CMPT_STATUS_FAIL                    (0xFF)
#define CMPT_STATUS_OK                      (0x00)

#define UNIQUE_ID_LENGTH                    16

//接收，发送数据结构
typedef struct
{
    uint16_t msg_head;
    uint16_t length;
    uint8_t type;
    uint8_t cmd;
    uint8_t number;
    uint8_t *data;
    uint16_t crc;
    uint8_t msg_tail;
}Data_t,*Data_p; 


#pragma pack(1)
typedef struct
{
    uint32_t ManufacturerID;    //32Bit
    uint32_t ProductID[2];      //64Bit (MAC Address)
//    
    uint32_t HWName[16];        //String
    uint32_t HWVersion;         //Vm.n.x.y
    uint32_t SWName[16];        //String,Exporting to Packet file name
    uint32_t SWVersion;         //Vm.n.x.y
//    
    uint32_t BootLoaderVersion; //Vm.n.x.y
    uint8_t Reserve[14];        //一代用于updater,现在updater放FLASH中了，这些字段空间保留
//    
    uint32_t UserCode1Version;
    uint32_t UserCode1StartAddr;
    uint32_t UserCode1Size;
    uint32_t UserCode1CRC;
    uint8_t UserCode1CompltStatus;
//    
    uint32_t UserCode2Version;
    uint32_t UserCode2StartAddr;
    uint32_t UserCode2Size;
    uint32_t UserCode2CRC;
    uint8_t UserCode2CompltStatus;
//    
    uint32_t ASRCMDModelVersion;
    uint32_t ASRCMDModelStartAddr;
    uint32_t ASRCMDModelSize;
    uint32_t ASRCMDModelCRC;
    uint8_t ASRCMDModelCompltStatus;
//    
    uint32_t DNNModelVersion;
    uint32_t DNNModelStartAddr;
    uint32_t DNNModelSize;
    uint32_t DNNModelCRC;
    uint8_t DNNModelCompltStatus;
//    
    uint32_t VoicePlayingVersion;
    uint32_t VoicePlayingStartAddr;
    uint32_t VoicePlayingSize;
    uint32_t VoicePlayingCRC;
    uint8_t VoicePlayingCompltStatus;
//
    uint32_t UserFileVersion;
    uint32_t UserFileStartAddr;
    uint32_t UserFileSize;
    uint32_t UserFileCRC;
    uint8_t UserFileCompltStatus;
//
    uint32_t ConsumerDataStartAddr;
    uint32_t ConsumerDataSize;
    
    uint16_t PartitionTableChecksum;
}FileConfig_Struct; 
#pragma pack()


//发送函数
void send_func(void);

//接收函数
void receive_func(uint8_t receive_char);

//数据包处理函数
void Resolution_func();

//CRC校验函数
uint16_t crc_func(uint16_t crc,uint8_t * buf,uint32_t len);

int32_t flash_update_buf_init(void);

void send_req_update_req_packet(void);
void send_req_update_write_packet(uint32_t offset,uint32_t size);
void send_req_update_write_packet_ex(uint32_t index,uint32_t offset,uint32_t size);
void send_req_update_block_write_done_packet(void);

void send_ack_get_info_packet(void);
void send_ack_update_check_ready_packet(void);
void send_ack_update_block_info_packet(void);
void send_ack_update_era_packet(void);
void send_ack_update_verify_packet(uint8_t verify);
void send_ack_try_fast_bd_packet(void);
void send_ack_try_fast_bd_test_packet(uint32_t size);
void send_ack_update_read_packet(uint32_t StartAddr, uint32_t Size);
void send_ack_update_complet_packet(void);
void send_ack_update_extern_dev_packet(void);
void send_ack_system_reset(void);
void send_notify_progress_packet(int index, int current, int total);
int32_t get_update_state(void);
void set_update_complete_status(void);

int32_t check_req_ack(void);
int32_t have_a_new_message(void);
int32_t check_req_recv(void);


#endif
