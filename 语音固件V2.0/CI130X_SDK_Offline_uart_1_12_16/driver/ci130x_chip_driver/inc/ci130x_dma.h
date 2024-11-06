/**
 * @file ci130x_dma.h
 * @brief dma驱动
 * @version 1.0
 * @date 2018-05-29
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */


#ifndef __CI100X_DMAC_H
#define __CI100X_DMAC_H

#include "ci130x_system.h"
#include "ci130x_dtrflash.h"


#ifdef __cplusplus
extern "C"{ 
#endif 


#define GDMA_SDRAM_ADDR  (0x70000000UL)
#define GDMA_SDRAM_SIZE	 (16*1024*1024)
  
#define GDMA_CSRAM_ADDR  (0x1FFF8000UL)
#define GDMA_CSRAM_SIZE	 (32*1024)
  
#define GDMA_SRAM0_ADDR  (0x1FFE8000UL)
#define GDMA_SRAM0_SIZE  (64*1024) 
  
#define GDMA_SRAM1_ADDR  (0x20000000UL)
#define GDMA_SRAM1_SIZE  (64*1024)  

#define GDMA_PCMRAM_ADDR  (0x20020000UL)
#define GDMA_PCMRAM_SIZE  (16*1024)   

#define GDMA_FFTRAM_ADDR  (0x200FF800UL)
#define GDMA_FFTRAM_SIZE  (2*1024)      
  

typedef enum
{
	DMACChannel0 = 0,
	DMACChannel1 = 1,
	DMACChannel2 = 2,
	DMACChannel3 = 3,
	DMACChannel4 = 4,
	DMACChannel5 = 5,
	DMACChannel6 = 6,
	DMACChannel7 = 7,
	DMACChannelALL = 9,
}DMACChannelx;

typedef enum
{
	DMAC_AHBMaster1 = 0,
	DMAC_AHBMaster2 = 1,
}DMAC_AHBMasterx;

typedef enum
{
	LittleENDIANMODE = 0,
	BigENDIANMODE	= 1,
}ENDIANMODE;

typedef enum
{
	INCREMENT = 1,
	NOINCREMENT = 0,
}INCREMENTx;

typedef enum
{
	TRANSFERWIDTH_8b = 0,
	TRANSFERWIDTH_16b = 1,
	TRANSFERWIDTH_32b = 2,
}TRANSFERWIDTHx;

typedef enum
{
	BURSTSIZE1 = 0,
	BURSTSIZE4 = 1,
	BURSTSIZE8 = 2,
	BURSTSIZE16 = 3,
	BURSTSIZE32 = 4,
	BURSTSIZE64 = 5,
	BURSTSIZE128 = 6,
	BURSTSIZE256 = 7,
}BURSTSIZEx;
/*Programmable DMA burst size. You can programme the DMA burst size to
transfer data more efficiently. The burst size is usually set to half the size of the
FIFO in the peripheral.
Busrts do not cross the 1KB address boundary*/

typedef enum
{
	DMAC_ACCESS_USERMODE = 0,
	DMAC_ACCESS_PRIVILEGEDMODE = 1,
}DMAC_ACCESS_MODE;

typedef enum
{
	BUFFERABLE = 1,
	NONBUFFERABLE = 0,
}BUFFERABLEx;

typedef enum
{
	CACHEABLE = 1,
	NONCACHEABLE = 0,
}CACHEABLEx;

typedef enum
{
	CHANNELINTMASK_ITC = 15,
	CHANNELINTMASK_IE = 14,
}CHANNELINTMASKx;

typedef enum
{
	M2M_DMA = 0,
	M2P_DMA = 1,
	P2M_DMA = 2,
	SP2DP_DMA = 3,
	SP2DP_DP = 4,
	M2P_P = 5,
	P2M_P = 6,
	SP2DP_SP = 7,
}DMAC_FLOWCTRL;

typedef enum
{
	DMAC_Peripherals_SPI0 = 0,
	DMAC_Peripherals_SPI1_RX = 1,
	DMAC_Peripherals_SPI1_TX = 2,
	DMAC_Peripherals_QSPI2 = 3,
	DMAC_Peripherals_UART0_RX = 4,
	DMAC_Peripherals_UART0_TX = 5,
	DMAC_Peripherals_UART1_RX = 6,
	DMAC_Peripherals_UART1_TX = 7,
	DMAC_Peripherals_UART2_RX = 8,
	DMAC_Peripherals_UART2_TX = 9,
}DMAC_Peripherals;

typedef struct
{
	FunctionalState TCInt;
	CACHEABLEx		CacheAble;
	BUFFERABLEx		BufferAble;
	DMAC_ACCESS_MODE		AccessMode;
	INCREMENTx 		DestInc;
	INCREMENTx		SrcInc;
	DMAC_AHBMasterx DestMaster;
	DMAC_AHBMasterx SrcMaster;
	TRANSFERWIDTHx	DestTransferWidth;
	TRANSFERWIDTHx	SrcTransferWidth;
	BURSTSIZEx		DestBurstSize;
	BURSTSIZEx		SrcBurstSize;
	unsigned short 	TransferSize;
}LLI_Control;

#if SPIC_DMA_MODEFI
typedef struct
{
    DMAC_FLOWCTRL flowctrl;
    BURSTSIZEx busrtsize;
    TRANSFERWIDTHx transferwidth;
}dma_config_temp;

#else
typedef struct
{
    DMAC_FLOWCTRL flowctrl;
    BURSTSIZEx busrtsize;
    TRANSFERWIDTHx transferwidth;
    uint32_t srcaddr;
    uint32_t destaddr;
    uint32_t transfersize;
}dma_config_t;
#endif //#if SPIC_DMA_MODEFI

/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_dma ci130x_dma
 * @brief CI130X芯片dma驱动
 * @{
 */

/**
 * @brief dma完成回调函数指针
 * 
 */
typedef void (*dma_callback_func_ptr_t)(void); 

void clear_dma_translate_flag(DMACChannelx dmachannel);
int wait_dma_translate_flag(DMACChannelx dmachannel,uint32_t timeout);
void dma_with_os_int(void);
void dma_without_os_int(void);
void dma_int_event_group_init(void);
void dma_irq_handler(void);

int DMAC_IntStatus(DMACChannelx dmachannel);
int DMAC_IntTCStatus(DMACChannelx dmachannel);
void DMAC_IntTCClear(DMACChannelx dmachannel);
int DMAC_IntErrorStatus(DMACChannelx dmachannel);
void DMAC_IntErrorClear(DMACChannelx dmachannel);
int DMAC_RawIntTCStatus(DMACChannelx dmachannel);
int DMAC_RawErrorIntStatus(DMACChannelx dmachannel);
int DMAC_ChannelEnableStatus(DMACChannelx dmachannel);
void DMAC_SoftwareBurstReq(DMACChannelx dmachannel);
void DMAC_SoftwareSingleReq(DMACChannelx dmachannel);
void DMAC_SoftwareLastBurstReq(DMACChannelx dmachannel);
void DMAC_SoftwareLastSingleReq(DMACChannelx dmachannel);
void DMAC_Config(DMAC_AHBMasterx dmamaster,ENDIANMODE endianmode);
void DMAC_EN(FunctionalState cmd);
void DMAC_ChannelSoureAddr(DMACChannelx dmachannel,unsigned int addr);
void DMAC_ChannelDestAddr(DMACChannelx dmachannel,unsigned int addr);
void DMAC_ChannelLLI(DMACChannelx dmachannel,unsigned int dmalli,DMAC_AHBMasterx dmamaster);
void DMAC_ChannelSourceConfig(DMACChannelx dmachannel,INCREMENTx inc,\
 	DMAC_AHBMasterx dmamaster,TRANSFERWIDTHx  transferwidth,BURSTSIZEx brustsize);
void DMAC_ChannelDestConfig(DMACChannelx dmachannel,INCREMENTx inc,\
 	DMAC_AHBMasterx dmamaster,TRANSFERWIDTHx  transferwidth,BURSTSIZEx brustsize);
void DMAC_ChannelTCInt(DMACChannelx dmachannel,FunctionalState cmd);
void DMAC_ChannelProtectionConfig(DMACChannelx dmachannel,DMAC_ACCESS_MODE access_mode,\
	BUFFERABLEx bufferable,CACHEABLEx cacheable);
void DMAC_ChannelTransferSize(DMACChannelx dmachannel,unsigned short size);
unsigned int DMAC_ChannelCurrentTransferSize( DMACChannelx dmachannel );
void DMAC_ChannelDisable(DMACChannelx dmachannel);
void DMAC_ChannelEnable(DMACChannelx dmachannel);
void DMAC_ChannelInterruptMask(DMACChannelx dmachannel,CHANNELINTMASKx \
	channelintmask,FunctionalState cmd);
void DMAC_ChannelPowerDown(DMACChannelx dmachannel,FunctionalState cmd);
void DMAC_ChannelHalt(DMACChannelx dmachannel,FunctionalState cmd);
void DMAC_ChannelConfig(DMACChannelx dmachannel,char destperiph,char srcperiph ,DMAC_FLOWCTRL flowctrl);
void DMAC_ChannelLock(DMACChannelx dmachannel,FunctionalState cmd);
void DMAC_M2MConfig(DMACChannelx dmachannel,unsigned int srcaddr,unsigned int destaddr,unsigned int bytesize,DMAC_AHBMasterx master);

void DMAC_M2P_P2M_advance_config(DMACChannelx dmachannel,DMAC_Peripherals periph,\
  DMAC_FLOWCTRL flowctrl,unsigned int srcaddr,unsigned int destaddr,\
	unsigned int bytesize,TRANSFERWIDTHx datawidth,BURSTSIZEx burstsize,DMAC_AHBMasterx master);
	
void DMAC_M2P_P2MConfig(DMACChannelx dmachannel,DMAC_Peripherals periph,DMAC_FLOWCTRL flowctrl,\
	unsigned int srcaddr,unsigned int destaddr,unsigned int bytesize);
void DMAC_P2PConfig(DMACChannelx dmachannel,DMAC_Peripherals srcperiph,DMAC_Peripherals destperiph,unsigned int bytesize);
#if SPIC_DMA_MODEFI
void spic_dma_stady_init(DMACChannelx channel);
void spic_dma_transfer_config(uint32_t srcaddr, uint32_t destaddr, uint32_t transfersize);
#else
void spic_dma_config(DMACChannelx channel,dma_config_t* config);
#endif
unsigned int DMAC_LLIContrlreturn(LLI_Control* lli_ctrl);
void DMAC_ChannelLLIControl(DMACChannelx dmachannel,unsigned int val);

/** @} */

#ifdef __cplusplus
}
#endif 

#endif /*__CI130X_DMA_H*/ 
