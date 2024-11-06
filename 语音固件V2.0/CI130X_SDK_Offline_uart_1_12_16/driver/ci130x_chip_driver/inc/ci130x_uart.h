/**
 * @file ci130x_uart.h
 * @brief  UART驱动文件
 * @version 0.1
 * @date 2019-10-25
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <ci130x_system.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_uart ci130x_uart
 * @brief CI130X芯片UART驱动
 * @{
 */

#define UART0_DMA_ADDR 0x61000000 /*!< UART0 DMA地址 */
#define UART1_DMA_ADDR 0x62000000 /*!< UART1 DMA地址 */
#define UART2_DMA_ADDR 0x63000000 /*!< UART2 DMA地址 */

/**
 * @brief UART接收错误标志寄存器定义
 */
typedef enum
{
    UART_FramingError    = 0,  /*!< 帧错误 */
    UART_ParityError     = 1,  /*!< 奇偶校验错误 */
    UART_BreakError      = 2,  /*!< break 错误 */
    UART_OverrunError    = 3,  /*!< 运行超时错误 */
}UART_ERRORFLAG;

/**
 * @brief UART标志寄存器定义
 */
typedef enum
{
    UART_CTS     = 0,  /*!< 发送清除标志 */
    UART_DSR     = 1,  /*!< 数据准备就绪标志 */
    UART_DCD     = 2,  /*!< 数据载波检查标志 */
    UART_RI      = 3,  /*!< 响铃指示信号 */
    UART_BUSY    = 4,  /*!< UART忙标志 */
    UART_RXFF    = 5,  /*!< 接收FIFO满标志 */
    UART_RXFE    = 6,  /*!< 接收FIFO空标志 */
    UART_TXFF    = 7,  /*!< 发送FIFO满标志 */
    UART_TXFE    = 8,  /*!< 发送FIFO空标志 */
    UART_EOC     = 9,  /*!< 完成当前传输标志 */
    UART_EDF     = 10, /*!< 数据错误标志 */
}UART_FLAGStatus;

/**
 * @brief UART波特率定义
 */
typedef enum
{
    UART_BaudRate2400       = 2400,    /*!< 2400Bps */
    UART_BaudRate4800       = 4800,    /*!< 4800Bps */
    UART_BaudRate9600       = 9600,    /*!< 9600Bps */
    UART_BaudRate19200      = 19200,   /*!< 19200Bps */
    UART_BaudRate38400      = 38400,   /*!< 38400Bps */
    UART_BaudRate57600      = 57600,   /*!< 57600Bps */
    UART_BaudRate115200     = 115200,  /*!< 115200Bps */
    UART_BaudRate230400     = 230400,  /*!< 230400Bps */
    UART_BaudRate380400     = 380400,  /*!< 380400Bps */
    UART_BaudRate460800     = 460800,  /*!< 460800Bps */
    UART_BaudRate921600     = 921600,  /*!< 921600Bps */
    UART_BaudRate1M         = 1000000, /*!< 1MBps */
    UART_BaudRate2M         = 2000000, /*!< 2MBps */
    UART_BaudRate3M         = 3000000, /*!< 3MBps */
}UART_BaudRate;


/**
 * @brief UART每帧有效数据位定义
 */
typedef enum
{
    UART_WordLength_5b    = 0, /*!< 5Bit有效位 */
    UART_WordLength_6b    = 1, /*!< 6Bit有效位 */
    UART_WordLength_7b    = 2, /*!< 7Bit有效位 */
    UART_WordLength_8b    = 3, /*!< 8Bit有效位 */
}UART_WordLength;

/**
 * @brief UART停止位定义
 */
typedef enum
{
    UART_StopBits_1        = 0, /*!< 1Bit停止位 */
    UART_StopBits_1_5      = 1, /*!< 1.5Bit停止位 */
    UART_StopBits_2        = 2, /*!< 2Bit停止位 */
}UART_StopBits;

/**
 * @brief UART奇偶校验定义
 */
typedef enum
{
    UART_Parity_No         = 0x0, /*!< 无奇偶校验 */
    UART_Parity_Odd        = 0x01, /*!< 奇校验 */
    UART_Parity_Even       = 0x03, /*!< 偶校验 */
}UART_Parity;

/**
 * @brief UART总线数据传输模式定义
 */
typedef enum
{
    UART_Byte    = 1, /*!< 以Byte模式向TxFIFO发送数据 */
    UART_Word    = 0, /*!< 以Word模式向TxFIFO发送数据 */
}UART_ByteWord;

/**
 * @brief UART控制寄存器定义
 */
typedef enum
{
    UART_CTSHardwareFlowCtrl    = 15, /*!< CTS硬件流控制信号控制位 */
    UART_RTSHardwareFlowCtrl    = 14, /*!< RTS硬件流控制信号控制位 */
    UART_Out2_RI                = 13, /*!<  */
    UART_Out1_DCD               = 12, /*!<  */
    UART_RTS                    = 11, /*!< 请求发送 */
    UART_DTR                    = 10, /*!< 数据传输就绪控制位 */
    UART_RXE                    = 9,  /*!< UART允许接收控制位 */
    UART_TXE                    = 8,  /*!< UART允许发送控制位 */
    UART_LoopBackE              = 7,  /*!< UART回环模式控制位 */
    UART_NCED                   = 1,  /*!< 忽略错误数据标志控制位 */
}UART_CRBitCtrl;

/**
 * @brief UART Tx/RxFIFO触发深度定义
 */
typedef enum
{
    UART_FIFOLevel1_8      = 0, /*!< 2Word(RX/TXFIFO均可配置) */
    UART_FIFOLevel1_4      = 1, /*!< 4Word(RX/TXFIFO均可配置) */
    UART_FIFOLevel1_2      = 2, /*!< 8Word(RX/TXFIFO均可配置) */
    UART_FIFOLevel3_4      = 3, /*!< 12Word(RX/TXFIFO均可配置) */
    UART_FIFOLevel7_8      = 4, /*!< 14Word(RX/TXFIFO均可配置) */
    UART_FIFOLevel1        = 5, /*!< 1Byte(仅RxFIFO可配置) */
    UART_FIFOLevel2        = 6, /*!< 2Byte(仅RxFIFO可配置) */
}UART_FIFOLevel;

/**
 * @brief UART中断标志定义
 */
typedef enum
{
    UART_ErrorDataInt        = 11, /*!< 数据错误中断标志 */
    UART_OverrunErrorInt     = 10, /*!< 运行超时错误中断标志 */
    UART_BreakErrorInt       = 9,  /*!< break 错误中断标志 */
    UART_ParityErrorInt      = 8,  /*!< 奇偶校验错误中断标志 */
    UART_FramingErrorInt     = 7,  /*!< 帧错误中断标志 */
    UART_RXTimeoutInt        = 6,  /*!< 接收超时中断标志 */
    UART_TXInt               = 5,  /*!< 发送中断标志 */
    UART_RXInt               = 4,  /*!< 接收中断标志 */
    UART_DSRModemInt         = 3,  /*!< 数据载波检查中断标志 */
    UART_DCDModemInt         = 2,  /*!< 数据准备就绪中断标志 */
    UART_CTSModemInt         = 1,  /*!< 发送清除中断标志 */
    UART_RIModemInt          = 0,  /*!< 响铃中断标志 */
    UART_AllInt              = 12, /*!< 操作所有中断标志 */
}UART_IntMask;

/**
 * @brief UART 定义
 */
typedef enum
{
    UART_RXDMA        = 0, /*!< UART接收DMA使能信号 */
    UART_TXDMA        = 1, /*!< UART发送DMA使能信号 */
}UART_TXRXDMA;


void UartPollingSenddata(UART_TypeDef* UARTx, char ch);
char UartPollingReceiveData(UART_TypeDef* UARTx);
void UARTPollingConfig(UART_TypeDef* UARTx, UART_BaudRate uartbaudrate);
void UART_IntMaskConfig(UART_TypeDef* UARTx,UART_IntMask intmask,FunctionalState cmd);
void UARTInterruptConfig(UART_TypeDef* UARTx,UART_BaudRate bd);
void UARTDMAConfig(UART_TypeDef* UARTx, UART_BaudRate uartbaudrate);
int UART_MaskIntState(UART_TypeDef* UARTx,UART_IntMask intmask);
void UART_IntClear(UART_TypeDef* UARTx,UART_IntMask intmask);
unsigned char UART_RXDATA(UART_TypeDef* UARTx);
int UART_ERRORSTATE(UART_TypeDef* UARTx,UART_ERRORFLAG uarterrorflag);
void UART_TXDATAConfig(UART_TypeDef* UARTx,unsigned int val);
int UART_FLAGSTAT(UART_TypeDef* UARTx,UART_FLAGStatus uartflag);
int UART_BAUDRATEConfig(UART_TypeDef* UARTx,UART_BaudRate uartbaudrate);
void UART_FIFOClear(UART_TypeDef* UARTx);
int UART_LCRConfig(UART_TypeDef* UARTx,UART_WordLength wordlength,\
          UART_StopBits uartstopbits,UART_Parity uartparity);
int UART_TXFIFOByteWordConfig(UART_TypeDef* UARTx,UART_ByteWord uarttxfifobit);
void UART_EN(UART_TypeDef* UARTx,FunctionalState cmd);
void UART_CRConfig(UART_TypeDef* UARTx,UART_CRBitCtrl crbitctrl,FunctionalState cmd);
void UART_RXFIFOConfig(UART_TypeDef* UARTx,UART_FIFOLevel fifoleve);
void UART_TXFIFOConfig(UART_TypeDef* UARTx,UART_FIFOLevel fifoleve);
void UART_IntMaskConfig(UART_TypeDef* UARTx,UART_IntMask intmask,FunctionalState cmd);
int UART_RawIntState(UART_TypeDef* UARTx,UART_IntMask intmask);
int UART_MaskIntState(UART_TypeDef* UARTx,UART_IntMask intmask);
void UART_IntClear(UART_TypeDef* UARTx,UART_IntMask intmask);
void UART_TXRXDMAConfig(UART_TypeDef* UARTx,UART_TXRXDMA uartdma);
void UART_TimeoutConfig(UART_TypeDef* UARTx,unsigned short time);
void UartPollingSenddone(UART_TypeDef* UARTx);
void UART_DMAByteWordConfig(UART_TypeDef* UARTx, FunctionalState cmd);
void UartSetCLKBaseBaudrate(UART_TypeDef *UARTx,UART_BaudRate uartbaudrate);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /*__UART_H*/
