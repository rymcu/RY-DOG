/**
 * @file ci130x_iis.h
 * @brief 三代芯片IIS底层驱动接口头文件
 * @version 0.1
 * @date 2021-07-05
 * 
 * @copyright Copyright (c) 2021  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CI130X_IIS_H
#define __CI130X_IIS_H

#include "ci130x_system.h"
#include "ci130x_iisdma.h" 
#include "ci130x_scu.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif 

/**
 * @brief IIS控制器定义
 */
typedef enum
{
    IIS0 = HAL_IIS0_BASE,
    IIS1 = HAL_IIS1_BASE,
    IIS2 = HAL_IIS2_BASE,
}iis_base_t;

/**
 * @brief IIS数据宽度
 * 
 */
typedef enum
{
    IIS_DW_16BIT = 0,
    IIS_DW_24BIT = 1,
    IIS_DW_32BIT = 2,
    IIS_DW_20BIT = 3,
}iis_data_width_t;

/**
 * @brief IIS 数据格式（标准IIS格式，左对齐格式，右对齐格式）
 * 
 */
typedef enum
{
    IIS_DF_IIS = 0,   /*!< IIS格式标准 */
    IIS_DF_MSB = 1,   /*!< 左对齐(MSB)格式标准 */
    IIS_DF_LSB = 2,   /*!< 右对齐(LSB)格式标准 */
}iis_data_format_t;

/**
 * @brief IIS 声道选择（单声道还是双声道）
 * 
 */
typedef enum
{
    IIS_SC_STEREO = 0, /*!< 双声道 */
    IIS_SC_MONO   = 1, /*!< 单声道 */
}iis_sound_channel_t;

/**
 * @brief IIS TX 发送通道选择
 * 
 */
typedef enum
{
    IIS_TX_CHANNAL_TX0 = 0,
    IIS_TX_CHANNAL_TX1 = 1,
}iis_tx_channal_t;

/**
 * @brief IIS TX 发送FIFO触发等级配置
 * 
 */
typedef enum
{
    IIS_TX_FIFO_1D2 = 0,    /*!< 发送FIFO触发深度1/2空（16个word） */
    IIS_TX_FIFO_1D4 = 1,    /*!< 发送FIFO触发深度1/4空（8个word） */
}iis_txfifo_trig_t;

/**
 * @brief IIS RX 接收通道
 * 
 */
typedef enum
{
    IIS_RX_CHANNAL_RX0 = 0,
    IIS_RX_CHANNAL_RX1 = 1,
    IIS_RX_CHANNAL_RX2 = 2,
}iis_rx_channal_t;

/**
 * @brief IIS RX 接收FIFO触发等级配置
 * 
 */
typedef enum
{
    IIS_RX_FIFO_1D4 = 0,    /*!< 接收FIFO触发深度1/4满（32个word） */
    IIS_RX_FIFO_1D8 = 1,    /*!< 接收FIFO触发深度1/8满（16个word） */
    IIS_RX_FIFO_1D16 = 2,   /*!< 接收FIFO触发深度1/16满（8个word） */
    IIS_RX_FIFO_1D32 = 3,   /*!< 接收FIFO触发深度1/32满（4个word） */
}iis_rxfifo_trig_t;

/**
 * @brief IIS 总线上SCK与LRCK的比例关系
 * 
 */
typedef enum
{
    IIS_SCK_LRCK_32 = 0,    /*!< 只有16bit才能配置*/
    IIS_SCK_LRCK_64 = 1,    /*!< */
}iis_sck_lrck_t;

typedef enum
{
    IIS_ENABLE = 1,     /*!<使能*/
    IIS_DISABLE = 0,    /*!<不使能 */
}iis_cmd_t;

/**
 * @brief IIS TX初始化结构体
 * 
 */
typedef struct
{
    iis_sck_lrck_t sck_lrck;          /*!< 总线上SCK与LRCK的比例关系 */
    iis_tx_channal_t cha;             /*!< TX 发送通道选择 */
    iis_txfifo_trig_t txfifo_trig;    /*!< TX 发送FIFO触发等级配置 */
    iis_data_width_t txch_dw;         /*!< TX 数据宽度*/
    uint32_t txch_copy;               /*!< TX 通道复制功能，单通道数据复制到双通道送出（只有配置单声道才能配置）*/
    iis_data_format_t tx_df;          /*!< IIS 数据格式（标准IIS格式，左对齐格式，右对齐格式）*/
    iis_sound_channel_t tx_sc;        /*!< IIS 声道选择（单声道还是双声道）*/
    uint32_t tx_merge;                /*!< 双声道merge在一起（只有配置双声道时才能配置，且数据宽度必须为16bit），1 enable; 0 disable*/
    uint32_t tx_swap;                 /*!< 发送时左右声道数据对调（单声道下左声道发送变为右声道），1 enable; 0 disable*/
}iis_tx_config_t,*iis_tx_config_p;

/**
 * @brief IIS RX初始化结构体
 * 
 */
typedef struct
{
    iis_sck_lrck_t sck_lrck;          /*!< 总线上SCK与LRCK的比例关系 */
    iis_rx_channal_t cha;             /*!< RX 接收通道选择 */
    iis_rxfifo_trig_t rxfifo_trig;    /*!< RX 接收FIFO触发等级配置 */
    iis_data_width_t rxch_dw;         /*!< RX 数据宽度*/
    iis_data_format_t rx_df;          /*!< IIS 数据格式（标准IIS格式，左对齐格式，右对齐格式）*/
    iis_sound_channel_t rx_sc;        /*!< IIS 声道选择（单声道还是双声道）*/
    uint32_t rx_merge;                /*!< 两个 16bit merge在一起（数据宽度为16bit才能merge），1 enable; 0 disable */
    uint32_t rx_swap;                 /*!< 接收时左右声道数据对调（单声道下左声道发送变为右声道），1 enable; 0 disable */
}iis_rx_config_t,*iis_rx_config_p;

/**
 * @brief IIS TX初始化结构体
 * 
 */
typedef struct
{
    //!IISDMA传输地址回卷中断产生，需要IISDMA搬运多少次
    IISDMA_RXTXxRollbackADDR rollbackaddr0size;
    //!IISDMA传输地址回卷中断产生，需要IISDMA搬运多少次
    IISDMA_RXTXxRollbackADDR rollbackaddr1size;
    //!IISDMA单次搬运的数据大小
    IISDMA_TXRXSingleSIZEx tx0singlesize;
    //!IISDMA单次搬运的数据大小
    IISDMA_TXRXSingleSIZEx tx1singlesize;
    //!IISDMACha
    IISDMAChax iisdmacha;
}IIS_DMA_TXInit_Typedef;


/**
 * @brief IIS RX初始化结构体
 * 
 */
typedef struct
{
    //!RX的地址设置
    unsigned int rxaddr;
    //!IISDMA搬运多少次之后，产生传输完成中断
    IISDMA_RXxInterrupt rxinterruptsize;
    //!IISDMA搬运多少次之后，产生地址回卷中断
    IISDMA_RXTXxRollbackADDR rollbackaddrsize;
    //!IISDMA单次传输数据大小
    IISDMA_TXRXSingleSIZEx rxsinglesize;
    //!IISDMACha
    IISDMAChax iisdmacha;
}IIS_DMA_RXInit_Typedef;

void iis_init(void);

void iis_tx_enable(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd);
void iis_tx_l_mute(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd);
void iis_tx_r_mute(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd);
void iis_tx_chk(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd);
void iis_tx_config(uint32_t iis_base,iis_tx_config_p tx_cfg);
void iis_tx_same(uint32_t iis_base,FunctionalState cmd);

void iis_rx_enable(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd);
void iis_rx_mute(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd);
void iis_rx_chk(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd);
void iis_rx_dma_chk(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd);
void iis_rx_config(uint32_t iis_base,iis_rx_config_p rx_cfg);
void iis_rx_cha_merge(uint32_t iis_base,uint8_t enable_rx0,uint8_t enable_rx1,uint8_t enable_rx2);

void iis_int_handler(uint32_t iis_base);

void IISx_TXDMA_Init(IIS_DMA_TXInit_Typedef* IISDMA_Str);
void IISx_RXDMA_Init(IIS_DMA_RXInit_Typedef* IISDMA_Str);

#ifdef __cplusplus
}
#endif 

#endif /*__IIS_H*/

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

