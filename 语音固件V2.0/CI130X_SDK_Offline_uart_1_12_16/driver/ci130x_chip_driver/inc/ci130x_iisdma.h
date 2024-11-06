/**
 * @file ci130x_iisdma.h
 * @brief 二代芯片IISDMA底层驱动接口的头文件
 * @version 0.1
 * @date 2019-05-10
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#ifndef __IISDMA_H
#define __IISDMA_H


#include "ci130x_system.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_iisdma ci130x_iisdma
 * @brief CI130X芯片iisdma驱动
 * @{
 */


/**
 * @brief IISDMA TX或者RX选择
 *
 */
typedef enum
{
	//!使能IISDMA TX
	IISxDMA_TX_EN 	=1,
	//!使能IISDMA RX
	IISxDMA_RX_EN	=0,
}IISDMA_TXRX_ENx;


/**
 * @brief IISDMA中不同通道的优先级设置
 *
 */
typedef enum
{
	//!IIS0>IIS1>IIS2
	IISDMA_Priority0 =0,
	//!IIS0>IIS2>IIS1
    IISDMA_Priority1,
	//!IIS1>IIS0>IIS2
	IISDMA_Priority2,
	//!IIS1>IIS2>IIS0
	IISDMA_Priority3,
	//!IIS2>IIS0>IIS1
	IISDMA_Priority4,
	//!IIS2>IIS1>IIS0
	IISDMA_Priority5,
	//!IIS0>IIS1>IIS2
	IISDMA_Priority6,
}IISDMA_Priorityx;


/**
 * @brief 每个IISDMA对应的通道
 *
 */
typedef enum
{
	//!通道0
	IISDMACha0=0,
	//!通道1
	IISDMACha1=1,
	//!通道2
	IISDMACha2=2,
}IISDMAChax;


/**
 * @brief IISDMA单次搬运的数据大小
 *
 */
typedef enum
{
	//!单次搬运16byte的数据
	IISDMA_TXRXSINGLESIZE16bytes =1,
	//!单次搬运32byte的数据
	IISDMA_TXRXSINGLESIZE32bytes =2,
	//!单次搬运48byte的数据
	IISDMA_TXRXSINGLESIZE48bytes =3,
	//!单次搬运64byte的数据
	IISDMA_TXRXSINGLESIZE64bytes =4,
	//!单次搬运80byte的数据
	IISDMA_TXRXSINGLESIZE80bytes =5,
	//!单次搬运96byte的数据
	IISDMA_TXRXSINGLESIZE96bytes =6,
	/*...*/
	//!单次搬运128byte的数据
	IISDMA_TXRXSINGLESIZE128bytes =8,
	//!单次搬运496byte的数据
	IISDMA_TXRXSINGLESIZE496bytes =31,
}IISDMA_TXRXSingleSIZEx;


/**
 * @brief IISDMA搬运了多少次之后，来地址绕回中断
 *
 */
typedef enum
{
	//!搬运1次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX1RollbackADDR 	=0,
	//!搬运2次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX2RollbackADDR	=1,
	//!搬运3次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX3RollbackADDR	=3,
	/*...*/
	//!搬运16次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX16RollbackADDR	=15,
	//!搬运32次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX32RollbackADDR	=31,
	//!搬运40次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX40RollbackADDR   =39,
	//!搬运64次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
    IISDMA_RXTX64RollbackADDR   =63,
	//!搬运512次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX512RollbackADDR  =511,
	//!搬运1024次出现写绕回，若开启地址绕回中断，将产生地址绕回中断
	IISDMA_RXTX1024RollbackADDR	=1023,
}IISDMA_RXTXxRollbackADDR;


/**
 * @brief IISDMA传输多少次之后，来传输完成中断
 *
 */
typedef enum
{
	//!搬运1次之后，若开启传输完成中断，将产生传输完成中断
	IISDMA_RX1Interrupt 	=0,
	//!搬运2次之后，若开启传输完成中断，将产生传输完成中断
	IISDMA_RX2Interrupt		=1,
	//!搬运3次之后，若开启传输完成中断，将产生传输完成中断
	IISDMA_RX3Interrupt		=2,
	//!搬运8次之后，若开启传输完成中断，将产生传输完成中断
    IISDMA_RX8Interrupt     =7,
	//!搬运8次之后，若开启传输完成中断，将产生传输完成中断
    IISDMA_RX16Interrupt    =15,
	//!搬运8次之后，若开启传输完成中断，将产生传输完成中断
    IISDMA_RX20Interrupt    =19,

	/*.........*/
	//!搬运32次之后，若开启传输完成中断，将产生传输完成中断
	IISDMA_RX32Interrupt	=31,
}IISDMA_RXxInterrupt;



/**
 * @brief TADDR0或者TADDR1选择
 *
 */
typedef enum
{
	//!TXADDR0
	IISDMA_TXAAD_Sel_ADDR0=0,
	//!TXADDR1
	IISDMA_TXAAD_Sel_ADDR1=1,
}IISDMA_TXADDR_Sel;



void IISDMA_DMICModeConfig(IISDMA_TypeDef* IISDMAx, FunctionalState cmd);
void IISDMA_ChannelENConfig(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, \
                           IISDMA_TXRX_ENx iisdma_txrx_sel, FunctionalState cmd);
void IISDMA_ADDRRollBackINT(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,\
                           IISDMA_TXRX_ENx iisdma_txrx_sel,FunctionalState cmd);
void IISDMA_ChannelIntENConfig(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,\
                              IISDMA_TXRX_ENx iisdma_txrx_sel,FunctionalState cmd);
void IISDMA_EN(IISDMA_TypeDef* IISDMAx, FunctionalState cmd);
void IISxDMA_RADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int rxaddr);
void IISxDMA_RNUM(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXxInterrupt iisrxtointerrupt ,\
                 IISDMA_RXTXxRollbackADDR rollbacktimes,IISDMA_TXRXSingleSIZEx rxsinglesize);
void IISxDMA_TADDR0(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int txaddr0);
void IISxDMA_TNUM0(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXTXxRollbackADDR rollbackaddr,\
                  IISDMA_TXRXSingleSIZEx txsinglesize);
void IISxDMA_TADDR1(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int txaddr1);
void IISxDMA_TNUM1(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXTXxRollbackADDR rollbackaddr,\
                  IISDMA_TXRXSingleSIZEx txsinglesize);
void IISDMA_PriorityConfig(IISDMA_TypeDef* IISDMAx, IISDMA_Priorityx iisdma_priority);
void IISDMA_INT_All_Clear(IISDMA_TypeDef* IISDMAx);
void IISDMA_INT_Clear(IISDMA_TypeDef* IISDMAx,unsigned int tmp);
int IISDMA_ADDRRollBackSTATE(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_TXRX_ENx iisdma_txrx_sel);
int CHECK_IISDMA_DATABUSBUSY(IISDMA_TypeDef* IISDMAx);
void IISDMA_RXCompleteClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx);
void IISxDMA_TXADDRRollbackInterruptClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,\
                                         IISDMA_TXADDR_Sel txrestart_addr);
void IISDMA_RXTXClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_TXRX_ENx iisdmarxtx);
unsigned int IISxDMA_RXADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx);
int IISDMA_TX_ADDR_Get(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx);
uint32_t Get_IISxDMA_RADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx);
void i2s_dma_straight_to_vad(uint32_t,uint32_t block_times,uint32_t endroll_addr);

typedef struct
{
	IISDMA_TypeDef*              IISDMAx;
	uint32_t                  block_size;
	/*---------------------------------*/
	IISDMA_TXRXSingleSIZEx tx_single_size;
	IISDMAChax                       TxChax;
	uint8_t                    iisdma_tx;
	uint32_t        iisdma_tx_data_addr0;
	uint32_t        iisdma_tx_block_num0;
	uint32_t        iisdma_tx_data_addr1;
	uint32_t        iisdma_tx_block_num1;
	/*---------------------------------*/
	IISDMA_TXRXSingleSIZEx rx_single_size;
	IISDMAChax                       RxChax;
	uint8_t                    iisdma_rx;
	uint32_t         iisdma_rx_data_addr;
	uint32_t         iisdma_rx_block_num;
}iisdma_config_t,*iisdma_config_p;

typedef struct
{
	IISDMAChax cha;
	uint32_t addr;
}iis_data_t;

void iisdma_config(iisdma_config_p config);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /*__IISDMA_H*/
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
