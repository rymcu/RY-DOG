/**
 * @file ci130x_uart.c
 * @brief  UART驱动文件
 * @version 0.1
 * @date 2019-10-25
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include "ci130x_uart.h"
#include "ci130x_scu.h"
#include "ci130x_core_eclic.h"
#include "ci_log.h"
#include "platform_config.h"
#include "board.h"
#include "ci130x_dpmu.h"
#include "ci130x_dma.h"

int UART_CLK = 0; /*!< UART 时钟 */

/**
 * @brief 特殊情况下，数据传输master端不能发出stop。一段时间后，后续数据传输继续进行，该模块能恢复继续传输。
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param cmd ENABLE : 使能UART; DISABLE : 禁用UART
 */
void UART_NO_STOP_EN(UART_TypeDef *UARTx,FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        UARTx->UARTCR &= ~(1 << 2);
    }
    else
    {
        UARTx->UARTCR |= (1 << 2);
    }
}

/**
 * @brief UART非阻塞模式接收数据(使用时需确保接收FIFO不为空)
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @retval char 接收到的数据
 */
unsigned char UART_RXDATA(UART_TypeDef *UARTx)
{
    return UARTx->UARTRdDR & 0xFF;
}

/**
 * @brief UART错误标志状态
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param uarterrorflag 错误标志
 * @retval 0 无错误标志
 * @retval 非0 有对应的错误标志
 */
int UART_ERRORSTATE(UART_TypeDef *UARTx, UART_ERRORFLAG uarterrorflag)
{
    return UARTx->UARTRdDR & (1 << (uarterrorflag + 8));
}

/**
 * @brief UART非阻塞模式发送数据(使用时需确保发送FIFO不为满)
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param val 需发送数据
 */
void UART_TXDATAConfig(UART_TypeDef *UARTx, unsigned int val)
{
    UARTx->UARTWrDR = val;
}

/**
 * @brief 读 UART 标志寄存器
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param uartflag UART状态标志位
 * @retval 0 无错误标志
 * @retval 非0 有对应的错误标志
 */
int UART_FLAGSTAT(UART_TypeDef *UARTx, UART_FLAGStatus uartflag)
{
    return UARTx->UARTFlag & (1 << uartflag);
}

/**
 * @brief UART的波特率配置
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param uartbaudrate 波特率
 * @retval RETURN_OK 参数正确
 * @retval PARA_ERROR 参数错误
 */
int UART_BAUDRATEConfig(UART_TypeDef *UARTx, UART_BaudRate uartbaudrate)
{
    unsigned int tmp;

    UartSetCLKBaseBaudrate(UARTx, uartbaudrate);

    UARTx->UARTIBrd = (UART_CLK / (16 * uartbaudrate));
    tmp = (UART_CLK % (16 * uartbaudrate)) * 8 / uartbaudrate;
    tmp = (tmp >> 1) + (tmp & 0x1);
    UARTx->UARTFBrd = tmp;
    if (tmp > 63) /*reg UARTFBrd is [0:5]*/
    {
        UARTx->UARTIBrd += 1;
        UARTx->UARTFBrd = 0;
    }
    return RETURN_OK;
}

/**
 * @brief 清除 UART FIFO数据
 *
 * @param UARTx UART组:UART0,UART1，UART2
 */
void UART_FIFOClear(UART_TypeDef *UARTx)
{
    UARTx->UARTLCR |= (1 << 4);
}

/**
 * @brief 配置UART的数据位、停止位、奇偶校验位
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param wordlength 每帧有效数据个数，UART_WordLength_5b，...,UART_WordLength_8b
 * @param uartstopbits 停止位，UART_StopBits_1，UART_StopBits_1_5，UART_StopBits_2
 * @param uartparity 奇偶校验选择
 * @retval RETURN_OK 参数正确
 * @retval PARA_ERROR 参数错误
 */
int UART_LCRConfig(UART_TypeDef *UARTx, UART_WordLength wordlength,
                   UART_StopBits uartstopbits, UART_Parity uartparity)
{
    UARTx->UARTLCR &= ~(0x7F << 0);
    UARTx->UARTLCR |= (wordlength << 5) | (uartstopbits << 2) | (uartparity << 0);
    return RETURN_OK;
}

/**
 * @brief 设置UART的发送FIFO数据位宽
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param uarttxfifobit UART_Byte/UART_Word ,以byte/word方式向TXFIFO发送数据
 * @retval RETURN_OK 参数正确
 * @retval PARA_ERROR 参数错误
 */
int UART_TXFIFOByteWordConfig(UART_TypeDef *UARTx, UART_ByteWord uarttxfifobit)
{
    if (UART_Word == uarttxfifobit)
    {
        UARTx->UARTLCR &= ~(1 << 8);
    }
    else
    {
        UARTx->UARTLCR |= (1 << 8);
    }
    return RETURN_OK;
}

/**
 * @brief UART 使能控制
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param cmd ENABLE : 使能UART; DISABLE : 禁用UART
 */
void UART_EN(UART_TypeDef *UARTx, FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        UARTx->UARTCR &= ~(1 << 0);
    }
    else
    {
        UARTx->UARTCR |= (1 << 0);
    }
}

/**
 * @brief UART 信号使能控制
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param crbitctrl 控制信号选择
 * @param cmd ENABLE : 使能; DISABLE : 禁用
 */
void UART_CRConfig(UART_TypeDef *UARTx, UART_CRBitCtrl crbitctrl, FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        UARTx->UARTCR &= ~(1 << crbitctrl);
    }
    else
    {
        UARTx->UARTCR |= (1 << crbitctrl);
    }
}

/**
 * @brief UART 接收FIFO 触发深度选择
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param fifoleve FIFO触发深度选择
 */
void UART_RXFIFOConfig(UART_TypeDef *UARTx, UART_FIFOLevel fifoleve)
{
    UARTx->UARTFIFOlevel &= ~(0x7 << 3);
    UARTx->UARTFIFOlevel |= (fifoleve << 3);
}

/**
 * @brief UART 发送FIFO 触发深度选择
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param fifoleve FIFO触发深度选择
 */
void UART_TXFIFOConfig(UART_TypeDef *UARTx, UART_FIFOLevel fifoleve)
{
    UARTx->UARTFIFOlevel &= ~(0x7 << 0);
    UARTx->UARTFIFOlevel |= (fifoleve << 0);
}

/**
 * @brief UART 中断屏蔽设置
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param intmask 需要屏蔽的中断控制位
 * @param cmd ENABLE : 屏蔽; DISABLE : 不屏蔽
 */
void UART_IntMaskConfig(UART_TypeDef *UARTx, UART_IntMask intmask, FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        UARTx->UARTMaskInt &= ~(1 << intmask);
    }
    else
    {
        UARTx->UARTMaskInt |= (1 << intmask);
    }
    if (UART_AllInt == intmask)
    {
        UARTx->UARTMaskInt = (cmd != ENABLE) ? 0 : 0xFFF;
    }
}

/**
 * @brief UART 原始中断(中断屏蔽前)状态
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param intmask 需要查询中断状态控制位
 * @retval 0 查询中断位无原始中断标志
 * @retval 非0 查询中断位有原始中断标志
 */
int UART_RawIntState(UART_TypeDef *UARTx, UART_IntMask intmask)
{
    return UARTx->UARTRIS & (1 << intmask);
}

/**
 * @brief UART 屏蔽后的中断状态
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param intmask 需要查询中断状态控制位
 * @retval 0 查询中断位无原始中断标志
 * @retval 非0 查询中断位有原始中断标志
 */
int UART_MaskIntState(UART_TypeDef *UARTx, UART_IntMask intmask)
{
    return UARTx->UARTMIS & (1 << intmask);
}

/**
 * @brief UART 清除中断标志
 *
 * @param UARTx UART组:UART0,UART1，UART2
 * @param intmask 需要清除中断状态控制位
 */
void UART_IntClear(UART_TypeDef *UARTx, UART_IntMask intmask)
{
    if (UART_AllInt != intmask)
    {
        UARTx->UARTICR |= (1 << intmask);
    }
    else
    {
        UARTx->UARTICR = 0xFFF;
    }
}

/**
 * @brief UART DMA 发送/接收控制使能
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 * @param uartdma UART_RXDMA : 接收dma使能; UART_TXDMA : 发送dma使能
 */
void UART_TXRXDMAConfig(UART_TypeDef *UARTx, UART_TXRXDMA uartdma)
{
    UARTx->UARTDMACR |= (1 << uartdma);
}

/**
 * @brief UART  超时设置
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 * @param time 超时大小 : 发送或接收Bit位时间(1/Baudrate) Default:32,Max:1023
 */
void UART_TimeoutConfig(UART_TypeDef *UARTx, unsigned short time)
{
    UARTx->UARTTimeOut = time & 0x3FF;
}

/**
 * @brief UART DMA Byte/word 传输模式设置(仅DMA模式下配置)
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 * @param cmd ENABLE : UART DMA Byte传输模式; DISABLE : UART DMA Word传输模式
 */
void UART_DMAByteWordConfig(UART_TypeDef *UARTx, FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        UARTx->BYTE_HW_MODE &= ~(1 << 0);
    }
    else
    {
        UARTx->BYTE_HW_MODE |= (1 << 0);
    }
}

/**
 * @brief UART 查询方式发送一个字节数据
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 * @param ch 发送的字符
 */
void UartPollingSenddata(UART_TypeDef *UARTx, char ch)
{
    while (UARTx->UARTFlag & (0x1 << 7))
        ;
    UARTx->UARTWrDR = (unsigned int)ch;
}

/**
 * @brief UART 查询方式接收一个字节数据
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 * @retval char 接收到的字符
 */
char UartPollingReceiveData(UART_TypeDef *UARTx)
{
    while (UARTx->UARTFlag & (0x1 << 6))
        ;
    return UARTx->UARTRdDR & 0xff;
}

/**
 * @brief 等待 UART 发送完毕
 *
 * @param UARTx UART组 : UART0,UART1，UART2
 */
void UartPollingSenddone(UART_TypeDef *UARTx)
{
    while (0 == (UARTx->UARTFlag & (0x1 << 8)))
        ;
}

/**
 * @brief 根据波特率配置相应的外设时钟,并打开外设时钟
 *
 * @param UARTx UART组 : UART0,UART1
 * @param uartbaudrate 需设置的UART波特率
 */
void UartSetCLKBaseBaudrate(UART_TypeDef *UARTx,UART_BaudRate uartbaudrate)
{
    scu_set_device_gate((unsigned int)UARTx,DISABLE);
    scu_set_div_parameter((unsigned int)UARTx,2);
    UART_CLK = get_ahb_clk()/2;
    scu_set_device_gate((unsigned int)UARTx, ENABLE);
}


/**
 * @brief UART 查询模式初始化
 *
 * @param UARTx UART组 : UART0,UART1
 */
void UARTPollingConfig(UART_TypeDef *UARTx, UART_BaudRate uartbaudrate)
{
    scu_set_device_reset((unsigned int)UARTx);
    scu_set_device_reset_release((unsigned int)UARTx);
    
    // UartSetCLKBaseBaudrate(UARTx, uartbaudrate);

    pad_config_for_uart(UARTx);
    
    UART_EN(UARTx, DISABLE);
    UART_CRConfig(UARTx, UART_TXE, ENABLE);
    UART_CRConfig(UARTx, UART_RXE, ENABLE);

    UART_BAUDRATEConfig(UARTx, uartbaudrate);

    UART_TXFIFOByteWordConfig(UARTx, UART_Byte);
    UART_LCRConfig(UARTx, UART_WordLength_8b, UART_StopBits_1, UART_Parity_No);
    UART_RXFIFOConfig(UARTx, UART_FIFOLevel1_4);
    UART_TXFIFOConfig(UARTx, UART_FIFOLevel1_8);

    UART_IntClear(UARTx, UART_AllInt);
    UART_IntMaskConfig(UARTx, UART_AllInt, ENABLE);
    UART_FIFOClear(UARTx);
    UART_CRConfig(UARTx, UART_NCED, DISABLE);
    UART_EN(UARTx, ENABLE);
}

/**
 * @brief UART 中断模式初始化
 *
 * @param UARTx UART组 : UART0,UART1
 * @param bd 发送/接收波特率
 */
void UARTInterruptConfig(UART_TypeDef *UARTx, UART_BaudRate bd)
{
    IRQn_Type Uart_IRQ_Num = UART2_IRQn;
    if (UART0 == UARTx)
    {
        Uart_IRQ_Num = UART0_IRQn;
    }
    else if (UART1 == UARTx)
    {
        Uart_IRQ_Num = UART1_IRQn;
    }
    scu_set_device_reset((unsigned int)UARTx);
    scu_set_device_reset_release((unsigned int)UARTx);
    UART_BAUDRATEConfig(UARTx, bd);

    pad_config_for_uart(UARTx);

    UART_EN(UARTx, DISABLE); 


    UART_EN(UARTx, DISABLE);
    UART_CRConfig(UARTx, UART_TXE, ENABLE);
    UART_CRConfig(UARTx, UART_RXE, ENABLE);
    UART_IntClear(UARTx, UART_AllInt);
    UART_IntMaskConfig(UARTx, UART_AllInt, ENABLE);
    UART_TXFIFOByteWordConfig(UARTx, UART_Byte);
    UART_LCRConfig(UARTx, UART_WordLength_8b, UART_StopBits_1, UART_Parity_No);
    UART_RXFIFOConfig(UARTx, UART_FIFOLevel1);
    UART_TXFIFOConfig(UARTx, UART_FIFOLevel1_8);
    UART_FIFOClear(UARTx);
    UART_CRConfig(UARTx, UART_NCED, ENABLE);
    /*if TX FIFO is empty,then TX int coming 
    打开中断马上进去*/
    UART_IntMaskConfig(UARTx, UART_TXInt, ENABLE);
    /*NO_STOP_EN功能，需开启UART_FramingErrorInt中断*/
    //UART_IntMaskConfig(UARTx, UART_FramingErrorInt, DISABLE);
    //UART_NO_STOP_EN(UARTx,ENABLE);
    UART_IntMaskConfig(UARTx, UART_RXInt, DISABLE);
    UART_EN(UARTx, ENABLE);
    eclic_irq_enable(Uart_IRQ_Num);
}

/**
 * @brief UART DMA模式初始化
 *
 * @param UARTx UART组 : UART0,UART1
 */
void UARTDMAConfig(UART_TypeDef *UARTx, UART_BaudRate uartbaudrate)
{
    pad_config_for_uart(UARTx);

    // scu_set_device_reset((unsigned int)HAL_GDMA_BASE);
    // scu_set_device_reset_release((unsigned int)HAL_GDMA_BASE);
    scu_set_device_gate((unsigned int)HAL_GDMA_BASE, ENABLE);
    eclic_irq_enable(DMA_IRQn);
    clear_dma_translate_flag(DMACChannel1);

    scu_set_device_reset((unsigned int)UARTx);
    scu_set_device_reset_release((unsigned int)UARTx);

    UART_BAUDRATEConfig(UARTx,uartbaudrate); // LT, fpga's APB 10MHz, 921600 error   

    UART_EN(UARTx, DISABLE);

    UART_CRConfig(UARTx, UART_TXE, ENABLE);
    UART_CRConfig(UARTx, UART_RXE, ENABLE);    
    UART_TXFIFOByteWordConfig(UARTx, UART_Byte); // LT, when RXFIFOConfig byte or half-word mode, 2018.11.15
    UART_LCRConfig(UARTx, UART_WordLength_8b, UART_StopBits_1, UART_Parity_No);
    UART_RXFIFOConfig( UARTx, UART_FIFOLevel1 ); // LT, byte mode

    UART_TXFIFOConfig(UARTx, UART_FIFOLevel1_2);

    UART_DMAByteWordConfig(UARTx, ENABLE);

    UART_IntClear(UARTx, UART_AllInt);
    UART_IntMaskConfig(UARTx, UART_AllInt, ENABLE);
    UART_FIFOClear(UARTx);
    UART_CRConfig(UARTx, UART_NCED, ENABLE);
    UART_TXRXDMAConfig(UARTx, UART_RXDMA);
    UART_TXRXDMAConfig(UARTx, UART_TXDMA);
    UART_EN(UARTx, ENABLE);
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
