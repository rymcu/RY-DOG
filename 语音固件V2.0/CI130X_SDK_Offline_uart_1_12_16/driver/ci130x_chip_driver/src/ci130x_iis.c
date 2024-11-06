/**
 * @file ci130x_iis.c
 * @brief 二代芯片IIS底层驱动接口
 * @version 0.1
 * @date 2021-07-05
 * 
 * @copyright Copyright (c) 2021  Chipintelli Technology Co., Ltd.
 * 
 */

#include "ci130x_iis.h"
#include "ci_assert.h"
#include "FreeRTOS.h"

#define IIS_CHK_TX_NUM (5)
#define IIS_CHK_RX_NUM (5)
#define IIS_CHK_DMA_NUM (5)

typedef volatile struct iis_register
{
    union/*!< 属性:RW 偏移:0x00 位宽:32 功能:中断查询寄存器 */
    {
        struct
        {
            uint32_t sck_rx0_int:           1;/*!< 功能:接收通道0 SCK时钟CHK中断标志位，写1清除 */
            uint32_t lrck_rx0_int:          1;/*!< 功能:接收通道0 LRCK时钟CHK中断标志位，写1清除 */
            uint32_t dma_rx0_int:           1;/*!< 功能:dma_rx0_req检测的中断标志位，写1清除 */
            uint32_t sck_tx0_int:           1;/*!< 功能:发送通道0 SCK时钟CHK中断标志位，写1清除 */
            uint32_t lrck_tx0_int:          1;/*!< 功能:发送通道0 LRCK时钟CHK中断标志位，写1清除 */
            uint32_t tx0_fifo_over_int:     1;/*!< 功能:tx0_fifo上溢出中断标志位，写1清除 */
            uint32_t tx0_fifo_down_int:     1;/*!< 功能:tx0_fifo下溢出中断标志位，写1清除 */
            uint32_t rx0_fifo_over_int:     1;/*!< 功能:rx0_fifo上溢出中断标志位，写1清除 */
            uint32_t rx0_fifo_down_int:     1;/*!< 功能:rx0_fifo下溢出中断标志位，写1清除 */
            uint32_t sck_rx1_int:           1;/*!< 功能:接收通道1 SCK时钟CHK中断标志位，写1清除 */
            uint32_t lrck_rx1_int:          1;/*!< 功能:接收通道1 LRCK时钟CHK中断标志位，写1清除 */
            uint32_t dma_rx1_int:           1;/*!< 功能:dma_rx1_req检测的中断标志位，写1清除 */
            uint32_t sck_tx1_int:           1;/*!< 功能:发送通道1 SCK时钟CHK中断标志位，写1清除 */
            uint32_t lrck_tx1_int:          1;/*!< 功能:发送通道1 LRCK时钟CHK中断标志位，写1清除 */
            uint32_t tx1_fifo_over_int:     1;/*!< 功能:tx1_fifo上溢出中断标志位，写1清除 */
            uint32_t tx1_fifo_down_int:     1;/*!< 功能:tx1_fifo下溢出中断标志位，写1清除 */
            uint32_t rx1_fifo_over_int:     1;/*!< 功能:rx1_fifo上溢出中断标志位，写1清除 */
            uint32_t rx1_fifo_down_int:     1;/*!< 功能:rx1_fifo下溢出中断标志位，写1清除 */
            uint32_t sck_rx2_int:           1;/*!< 功能:接收通道2 SCK时钟CHK中断标志位，写1清除 */
            uint32_t lrck_rx2_int:          1;/*!< 功能:接收通道2 LRCK时钟CHK中断标志位，写1清除 */
            uint32_t dma_rx2_int:           1;/*!< 功能:dma_rx2_req检测的中断标志位，写1清除 */
            uint32_t rx2_fifo_over_int:     1;/*!< 功能:rx2_fifo上溢出中断标志位，写1清除 */
            uint32_t rx2_fifo_down_int:     1;/*!< 功能:rx2_fifo下溢出中断标志位，写1清除 */
            uint32_t rr0_err_int:           1;/*!< 功能:接收通道0 数据使能信号产生太快中断标志位，写1清除 */
            uint32_t rr1_err_int:           1;/*!< 功能:接收通道1 数据使能信号产生太快中断标志位，写1清除 */
            uint32_t rr2_err_int:           1;/*!< 功能:接收通道2 数据使能信号产生太快中断标志位，写1清除 */
            uint32_t tt0_err_int:           1;/*!< 功能:发送通道0 数据使能信号产生太快中断标志位，写1清除 */
            uint32_t tt1_err_int:           1;/*!< 功能:发送通道1 数据使能信号产生太快中断标志位，写1清除 */
            uint32_t reserved:              4;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_int0;
    uint32_t reserved_0[3];/*!< 属性:-- 偏移:0x04、0x08、0x0C 位宽:32 功能:保留 */
    union/*!< 属性:RW 偏移:0x10、0x14 位宽:32 功能:发送通道控制寄存器 */
    {
        struct
        {
            uint32_t tx_en:                 1;/*!< 功能:发送通道使能：1 = 使能，0 = 失能 */
            uint32_t reserved_0:            2;/*!< 功能:保留 */
            uint32_t txfifo_trig:           1;/*!< 功能:发送FIFO触发等级配置：1 = 1/4空(8)，0 = 1/2空(16) */
            uint32_t txch_dw:               3;/*!< 功能:发送数据宽度：000 = 16bit，001 = 24bit，010 = 32bit，011 = 20bit */
            uint32_t txch_l_mute:           1;/*!< 功能:TX L声道数据静音 */
            uint32_t txch_r_mute:           1;/*!< 功能:TX R声道数据静音 */
            uint32_t txch_copy:             1;/*!< 功能:TX通道复制功能，单通道数据复制到双通道送出（只有tx_mono为1时才能配置） */
            uint32_t tx_sck_lrck:           1;/*!< 功能:IIS总线上SCK与LRCK的比例关系（master和slave都有效）：1----SCK = 64*LRCK，0----SCK = 32*LRCK(只有16bit才能配) */
            uint32_t tx_df:                 2;/*!< 功能:数据传输格式：00：IIS， 01：左对齐， 10 ：右对齐 */
            uint32_t tx_mono:               1;/*!< 功能:单双声道选择，1：单声道，0：双声道 */
            uint32_t tx_merge:              1;/*!< 功能:双声道merge在一起（只有tx_mono为0时才能配置，且必须为16bit） */
            uint32_t tx_merge_mono:         1;/*!< 功能:单声道merge在一起（只有tx_mono为1时才能配置，且必须为16bit） */
            uint32_t tx_swap:               1;/*!< 功能:发送时左右声道数据对调（单声道下左声道发送变为右声道） */
            uint32_t tx_same:               1;/*!< 两路通道时是否发同一数据源的数据（做环绕立体音效果），高有效（此时两个通道必须使用相同的IIS时钟和配置），只有TX1有这个功能 */
            uint32_t reserved:             14;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_tx_ctrl[2];
    uint32_t reserved_2[2];/*!< 属性:-- 偏移:0x18、0x1C 位宽:32 功能:保留 */
    union/*!< 属性:RW 偏移:0x20、0x24、0x28 位宽:32 功能:接收通道0、1、2控制寄存器 */
    {
        struct
        {
            uint32_t rx_en:                 1;/*!< 功能:接收通道使能，高有效 */
            uint32_t reserved_2:            3;/*!< 功能:保留 */
            uint32_t rxfifo_trig:           2;/*!< 功能:接收FIFO触发等级配置：00 = 1/4满：32，01 = 1/8满: 16，10 = 1/16满: 8，11 = 1/32满: 4 */
            uint32_t rxch_dw:               3;/*!< 功能:数据源数据宽度：000 = 16bit，001 = 24bit，010 = 32bit，011 = 20bit */
            uint32_t reserved_1:            7;/*!< 功能:保留 */
            uint32_t rx_mono:               1;/*!< 功能:单双声道选择，0： stereo（双声道），1： mono（单声道） */
            uint32_t rx_swap:               1;/*!< 功能:数据对调使能，1 = 使能，0 = 失能 */
            uint32_t reserved_0:            6;/*!< 功能:保留 */
            uint32_t rx_merge:              1;/*!< 功能:双通道的两个16bit的数据合并在一起（rx_mono 必须为0才能配置，且必须为16bit） */
            uint32_t rx_merge_mono:         1;/*!< 功能:单声道的两个16bit的数据合并在一起（rx_mono 必须为1才能配置，且必须为16bit） */
            uint32_t rx_mute:               1;/*!< 功能:RX通道静音使能：1：静音，RX输出为0，0：正常模式 */
            uint32_t rx_sck_lrck:           1;/*!< 功能:IIS总线上SCK与LRCK的比例关系（master和slave都有效）：1----SCK = 64*LRCK，0----SCK = 32*LRCK(只有16bit才能配) */
            uint32_t rx_df:                 2;/*!< 功能:数据传输格式：00：IIS，01：左对齐，10：右对齐 */
            uint32_t rx_ch_merge:           1;/*!< 功能:是否参与merge(不能只配置一路ch_merge有效) */
            uint32_t reserved:              1;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_rx_ctrl[3];
    uint32_t reserved_3;/*!< 属性:-- 偏移:0x2C 位宽:32 功能:保留 */
    union/*!< 属性:RW 偏移:0x30 位宽:32 功能:装载使能控制寄存器 */
    {
        struct
        {
            uint32_t rx_load_en:            1;/*!< 功能:数据接收通道配置信息装载使能信号，高有效（先读取该寄存器全部为0才能配置）；即使配置接收通道开启或者关闭，该位没有配置也不会生效 */
            uint32_t reserved_0:            9;/*!< 功能:保留 */
            uint32_t tx_load_en:            1;/*!< 功能:数据发送通道配置信息装载使能信号，高有效（先读取该寄存器全部为0才能配置）；即使配置发送通道开启或者关闭，该位没有配置也不会生效 */
            uint32_t reserved:             21;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_load_ctrl;
    union/*!< 属性:RW 偏移:0x34、0x38、0x3C 位宽:32 功能:RX0、RX1、RX2 DMA请求检测寄存器 */
    {
        struct
        {
            uint32_t chk_dma_num:          16;/*!< 功能:counter的阈值 */
            uint32_t chk_dma_en:            1;/*!< 功能:RX_DMA_REQ请求检测使能 */
            uint32_t reserved:             15;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_chken_dma_rx[3];
    union/*!< 属性:RW 偏移:0x40、0x44、0x48 位宽:32 功能:RX0、RX1、RX2 IIS时钟检测寄存器 */
    {
        struct
        {
            uint32_t chk_rx_en:             1;/*!< 功能:IIS的接收时钟（lrck与sck共用）检测使能 */
            uint32_t chk_rx_num:           20;/*!< 功能:IIS的接收时钟（lrck与sck共用）检测counter的阈值 */
            uint32_t reserved:             11;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_chken_rx[3];
    uint32_t reserved_4;/*!< 属性:-- 偏移:0x4C 位宽:32 功能:保留 */
    union/*!< 属性:RW 偏移:0x50、0x54 位宽:32 功能:TX0、TX1 IIS时钟检测寄存器 */
    {
        struct
        {
            uint32_t chk_tx_en:             1;/*!< 功能:IIS的发送时钟（lrck与sck共用）检测使能 */
            uint32_t chk_tx_num:           20;/*!< 功能:IIS的发送时钟（lrck与sck共用）检测counter的阈值 */
            uint32_t reserved:             11;/*!< 功能:保留 */
        };
        uint32_t val;
    }iis_chken_tx[2];
}iis_register_t,*iis_register_p;


void iis_wait_load_ctrl(uint32_t iis_base)
{
    iis_register_p iis = (iis_register_p)iis_base;
    volatile uint32_t iis_timeout = 1000000;
    while(iis->iis_load_ctrl.val)
    {
        iis_timeout--;
        if(iis_timeout == 0)
        {
            // CI_ASSERT(0,"iis_load_ctrl timeout!\n");
            iis_timeout = 1000000;
            mprintf("iis_load_ctrl timeout!\n");
        }
    }
}

/**
 * @brief IIS发送使能
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS发送通道
 * @param cmd 是否使能
 */
void iis_tx_enable(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis_wait_load_ctrl(iis_base);
    if(cmd == ENABLE)
    {
        iis->iis_tx_ctrl[cha].tx_en = 1;
    }
    else
    {
        iis->iis_tx_ctrl[cha].tx_en = 0;
    }
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS接收使能
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS接收通道
 * @param cmd 是否使能
 */
void iis_rx_enable(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis_wait_load_ctrl(iis_base);
    if(cmd == ENABLE)
    {
        iis->iis_rx_ctrl[cha].rx_en = 1;
    }
    else
    {
        iis->iis_rx_ctrl[cha].rx_en = 0;
    }
    iis->iis_load_ctrl.rx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS发送模式，左声道静音
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS发送通道
 * @param cmd 是否静音
 */
void iis_tx_l_mute(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    if(cmd == ENABLE)
    {
        iis->iis_tx_ctrl[cha].txch_l_mute = 1;
    }
    else
    {
        iis->iis_tx_ctrl[cha].txch_l_mute = 0;
    }
}

/**
 * @brief IIS发送模式，右声道静音
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS发送通道
 * @param cmd 是否静音
 */
void iis_tx_r_mute(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    if(cmd == ENABLE)
    {
        iis->iis_tx_ctrl[cha].txch_r_mute = 1;
    }
    else
    {
        iis->iis_tx_ctrl[cha].txch_r_mute = 0;
    }
}

/**
 * @brief IIS发送模式，时钟检测
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS发送通道
 * @param cmd 是否使能时钟检测
 */
void iis_tx_chk(uint32_t iis_base,iis_tx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis->iis_chken_tx[cha].chk_tx_num = IIS_CHK_TX_NUM;
    if(cmd == ENABLE)
    {
        iis->iis_chken_tx[cha].chk_tx_en = 1;
    }
    else
    {
        iis->iis_chken_tx[cha].chk_tx_en = 0;
    }
}

/**
 * @brief IIS发送模式，配置
 *
 * @param iis_base IIS控制器基地址
 * @param tx_cfg IIS发送配置信息
 */
void iis_tx_config(uint32_t iis_base,iis_tx_config_p tx_cfg)
{
    iis_register_p iis = (iis_register_p)iis_base;
    uint8_t mono_flag = 0;
    uint8_t merge_flag = 0;
    if(IIS_SC_MONO == tx_cfg->tx_sc)
    {
        mono_flag = 1;
    }
    if(IIS_DW_16BIT == tx_cfg->txch_dw)
    {
        merge_flag = 1;
    }
    iis_wait_load_ctrl(iis_base);

    iis->iis_tx_ctrl[tx_cfg->cha].tx_en = 0;
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
    //////////////////////////////////////
    iis->iis_tx_ctrl[tx_cfg->cha].txfifo_trig = tx_cfg->txfifo_trig;
    iis->iis_tx_ctrl[tx_cfg->cha].txch_dw = tx_cfg->txch_dw;
    if(mono_flag)
    {
        iis->iis_tx_ctrl[tx_cfg->cha].txch_copy = tx_cfg->txch_copy;
    }
    iis->iis_tx_ctrl[tx_cfg->cha].tx_sck_lrck = tx_cfg->sck_lrck;
    iis->iis_tx_ctrl[tx_cfg->cha].tx_df = tx_cfg->tx_df;
    iis->iis_tx_ctrl[tx_cfg->cha].tx_mono = tx_cfg->tx_sc;
    if(merge_flag)
    {
        if(mono_flag)
        {
            //单声道两个 16bit merge
            iis->iis_tx_ctrl[tx_cfg->cha].tx_merge_mono = tx_cfg->tx_merge;
            iis->iis_tx_ctrl[tx_cfg->cha].tx_merge = 0;
        }
        else
        {
            //双声道两个 16bit merge
            iis->iis_tx_ctrl[tx_cfg->cha].tx_merge = tx_cfg->tx_merge;
            iis->iis_tx_ctrl[tx_cfg->cha].tx_merge_mono = 0;
        }
    }
    iis->iis_tx_ctrl[tx_cfg->cha].tx_swap = tx_cfg->tx_swap;
    //////////////////////////////////////
    //iis->iis_tx_ctrl[tx_cfg->cha].tx_en = 1;
    iis->iis_load_ctrl.tx_load_en = 1;
 
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS发送模式，立体环绕音使能
 *
 * @param iis_base IIS控制器基地址
 * @param cmd 是否使能立体环绕音
 */
void iis_tx_same(uint32_t iis_base,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis_wait_load_ctrl(iis_base);
    iis->iis_tx_ctrl[0].tx_en = 0;
    iis->iis_tx_ctrl[1].tx_en = 0;
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
    if(cmd == ENABLE)
    {
        if(iis->iis_tx_ctrl[0].val != iis->iis_tx_ctrl[1].val)
        {
            mprintf("IIS TX0与TX1配置不相同，无法配置为环绕立体音效\n");
            return;
        }
        iis->iis_tx_ctrl[1].tx_same = 1;
    }
    else
    {
        iis->iis_tx_ctrl[1].tx_same = 0;
    }
    iis->iis_tx_ctrl[0].tx_en = 1;
    iis->iis_tx_ctrl[1].tx_en = 1;
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS接收模式，静音
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS接收通道
 * @param cmd 是否使能静音
 */
void iis_rx_mute(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    if(cmd == ENABLE)
    {
        iis->iis_rx_ctrl[cha].rx_mute = 1;
    }
    else
    {
        iis->iis_rx_ctrl[cha].rx_mute = 0;
    }
}

/**
 * @brief IIS接收模式，时钟检测
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS接收通道
 * @param cmd 是否使能时钟检测
 */
void iis_rx_chk(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis->iis_chken_rx[cha].chk_rx_num = IIS_CHK_RX_NUM;
    if(cmd == ENABLE)
    {
        iis->iis_chken_rx[cha].chk_rx_en = 1;
    }
    else
    {
        iis->iis_chken_rx[cha].chk_rx_en = 0;
    }
}

/**
 * @brief IIS接收模式，发送DMA请求检测
 *
 * @param iis_base IIS控制器基地址
 * @param cha IIS发送通道
 * @param cmd 是否检测DAM请求
 */
void iis_rx_dma_chk(uint32_t iis_base,iis_rx_channal_t cha,FunctionalState cmd)
{
    iis_register_p iis = (iis_register_p)iis_base;
    iis->iis_chken_dma_rx[cha].chk_dma_num = IIS_CHK_DMA_NUM;
    if(cmd == ENABLE)
    {
        iis->iis_chken_dma_rx[cha].chk_dma_en = 1;
    }
    else
    {
        iis->iis_chken_dma_rx[cha].chk_dma_en = 0;
    }
}

/**
 * @brief IIS接收配置
 *
 * @param iis_base IIS控制器基地址
 * @param tx_cfg IIS接收配置信息
 */
void iis_rx_config(uint32_t iis_base,iis_rx_config_p rx_cfg)
{
    iis_register_p iis = (iis_register_p)iis_base;
    uint8_t mono_flag = 0;
    uint8_t merge_flag = 0;
    if(IIS_SC_MONO == rx_cfg->rx_sc)
    {
        mono_flag = 1;
    }
    if(IIS_DW_16BIT == rx_cfg->rxch_dw)
    {
        merge_flag = 1;
    }
    iis_wait_load_ctrl(iis_base);

    iis->iis_rx_ctrl[rx_cfg->cha].rx_en = 0;
    iis->iis_load_ctrl.rx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
    //////////////////////////////////////
    iis->iis_rx_ctrl[rx_cfg->cha].rxfifo_trig = rx_cfg->rxfifo_trig;
    iis->iis_rx_ctrl[rx_cfg->cha].rxch_dw = rx_cfg->rxch_dw;
    iis->iis_rx_ctrl[rx_cfg->cha].rx_sck_lrck = rx_cfg->sck_lrck;
    iis->iis_rx_ctrl[rx_cfg->cha].rx_df = rx_cfg->rx_df;
    iis->iis_rx_ctrl[rx_cfg->cha].rx_mono = rx_cfg->rx_sc;
    if(merge_flag)
    {
        if(mono_flag)
        {
            //单声道两个 16bit merge
            iis->iis_rx_ctrl[rx_cfg->cha].rx_merge_mono = rx_cfg->rx_merge;
        }
        else
        {
            //双声道两个 16bit merge
            iis->iis_rx_ctrl[rx_cfg->cha].rx_merge = rx_cfg->rx_merge;
        }
    }
    iis->iis_rx_ctrl[rx_cfg->cha].rx_swap = rx_cfg->rx_swap;
    //////////////////////////////////////
    //iis->iis_rx_ctrl[rx_cfg->cha].rx_en = 1;
    iis->iis_load_ctrl.rx_load_en = 1;
    
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS接收模式，接收通道merge
 *
 * @param iis_base IIS控制器基地址
 * @param enable_rx0 发送通道0是否merge
 * @param enable_rx1 发送通道1是否merge
 * @param enable_rx2 发送通道2是否merge
 */
void iis_rx_cha_merge(uint32_t iis_base,uint8_t enable_rx0,uint8_t enable_rx1,uint8_t enable_rx2)
{
    iis_register_p iis = (iis_register_p)iis_base;
    uint32_t config_temp = 0;
    if(enable_rx0)
    {
        config_temp = iis->iis_rx_ctrl[0].val;
    }
    if(enable_rx1)
    {
        if(config_temp)
        {
            if(config_temp != iis->iis_rx_ctrl[1].val)
            {
                return;
            }
        }
        else
        {
            config_temp = iis->iis_rx_ctrl[1].val;
        }
    }
    if(enable_rx2)
    {
        if(config_temp)
        {
            if(config_temp != iis->iis_rx_ctrl[2].val)
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
    iis_wait_load_ctrl(iis_base);
    iis->iis_rx_ctrl[0].rx_en = 0;
    iis->iis_rx_ctrl[1].rx_en = 0;
    iis->iis_rx_ctrl[2].rx_en = 0;
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
    if(enable_rx0)
    {
        iis->iis_rx_ctrl[0].rx_ch_merge = 1;
    }
    else
    {
        iis->iis_rx_ctrl[0].rx_ch_merge = 0;
    }
    if(enable_rx1)
    {
        iis->iis_rx_ctrl[1].rx_ch_merge = 1;
    }
    else
    {
        iis->iis_rx_ctrl[1].rx_ch_merge = 0;
    }
    if(enable_rx2)
    {
        iis->iis_rx_ctrl[2].rx_ch_merge = 1;
    }
    else
    {
        iis->iis_rx_ctrl[2].rx_ch_merge = 0;
    }
    iis->iis_rx_ctrl[0].rx_en = 1;
    iis->iis_rx_ctrl[1].rx_en = 1;
    iis->iis_rx_ctrl[2].rx_en = 1;
    iis->iis_load_ctrl.tx_load_en = 1;
    iis_wait_load_ctrl(iis_base);
}

/**
 * @brief IIS中断处理
 *
 * @param iis_base IIS控制器基地址
 */
void iis_int_handler(uint32_t iis_base)
{
    iis_register_p iis = (iis_register_p)iis_base;

#if 0    
    if(iis->iis_int0.sck_rx0_int)
    {
        iis->iis_int0.sck_rx0_int = 1;
    }
    if(iis->iis_int0.lrck_rx0_int)
    {
        iis->iis_int0.lrck_rx0_int = 1;
    }
    if(iis->iis_int0.dma_rx0_int)
    {
        iis->iis_int0.dma_rx0_int = 1;
    }
    if(iis->iis_int0.sck_tx0_int)
    {
        iis->iis_int0.sck_tx0_int = 1;
    }
    if(iis->iis_int0.lrck_tx0_int)
    {
        iis->iis_int0.lrck_tx0_int = 1;
    }
    if(iis->iis_int0.tx0_fifo_over_int)
    {
        iis->iis_int0.tx0_fifo_over_int = 1;
    }
    if(iis->iis_int0.tx0_fifo_down_int)
    {
        iis->iis_int0.tx0_fifo_down_int = 1;
    }
    if(iis->iis_int0.rx0_fifo_over_int)
    {
        iis->iis_int0.rx0_fifo_over_int = 1;
    }
    if(iis->iis_int0.rx0_fifo_down_int)
    {
        iis->iis_int0.rx0_fifo_down_int = 1;
    }
    if(iis->iis_int0.sck_rx1_int)
    {
        iis->iis_int0.sck_rx1_int = 1;
    }
    if(iis->iis_int0.lrck_rx1_int)
    {
        iis->iis_int0.lrck_rx1_int = 1;
    }
    if(iis->iis_int0.dma_rx1_int)
    {
        iis->iis_int0.dma_rx1_int = 1;
    }
    if(iis->iis_int0.sck_tx1_int)
    {
        iis->iis_int0.sck_tx1_int = 1;
    }
    if(iis->iis_int0.lrck_tx1_int)
    {
        iis->iis_int0.lrck_tx1_int = 1;
    }
    if(iis->iis_int0.tx1_fifo_over_int)
    {
        iis->iis_int0.tx1_fifo_over_int = 1;
    }
    if(iis->iis_int0.tx1_fifo_down_int)
    {
        iis->iis_int0.tx1_fifo_down_int = 1;
    }
    if(iis->iis_int0.rx1_fifo_over_int)
    {
        iis->iis_int0.rx1_fifo_over_int = 1;
    }
    if(iis->iis_int0.rx1_fifo_down_int)
    {
        iis->iis_int0.rx1_fifo_down_int = 1;
    }
    if(iis->iis_int0.sck_rx2_int)
    {
        iis->iis_int0.sck_rx2_int = 1;
    }
    if(iis->iis_int0.lrck_rx2_int)
    {
        iis->iis_int0.lrck_rx2_int = 1;
    }
    if(iis->iis_int0.dma_rx2_int)
    {
        iis->iis_int0.dma_rx2_int = 1;
    }
    if(iis->iis_int0.rx2_fifo_over_int)
    {
        iis->iis_int0.rx2_fifo_over_int = 1;
    }
    if(iis->iis_int0.rx2_fifo_down_int)
    {
        iis->iis_int0.rx2_fifo_down_int = 1;
    }
    if(iis->iis_int0.rr0_err_int)
    {
        iis->iis_int0.rr0_err_int = 1;
    }
    if(iis->iis_int0.rr1_err_int)
    {
        iis->iis_int0.rr1_err_int = 1;
    }
    if(iis->iis_int0.rr2_err_int)
    {
        iis->iis_int0.rr2_err_int = 1;
    }
    if(iis->iis_int0.tt0_err_int)
    {
        iis->iis_int0.tt0_err_int = 1;
    }
    if(iis->iis_int0.tt1_err_int)
    {
        iis->iis_int0.tt1_err_int = 1;
    }
#else
    volatile uint32_t int_val = iis->iis_int0.val;
    iis->iis_int0.val = int_val;
#endif
}

/**
 * @brief IISDMA0中断处理函数
 * 
 */
void IIS_DMA_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    volatile unsigned int tmp;
    tmp = IISDMA0->IISDMASTATE;

    extern void cm_input_interrupt_handler(IISDMAChax dma_channel, BaseType_t * xHigherPriorityTaskWoken);
    extern void cm_output_interrupt_handler(IISDMAChax dma_channel, BaseType_t * xHigherPriorityTaskWoken);
    // mprintf("tmp:0x%08x", tmp);

    if (tmp & (1 << (1 + IISDMACha0*2 + IISxDMA_RX_EN)))//iis0 rx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha0*2 + IISxDMA_RX_EN)));
        cm_input_interrupt_handler(IISDMACha0, &xHigherPriorityTaskWoken);
    }

    if (tmp & (1 << (1 + IISDMACha0*2 + IISxDMA_TX_EN)))//iis0 tx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha0*2 + IISxDMA_TX_EN)));
        // cm_output_interrupt_handler(IISDMACha0, &xHigherPriorityTaskWoken);
        // mprintf("i:%d\n", __LINE__);
    }

    if (tmp & (1 << (1 + IISDMACha1*2 + IISxDMA_RX_EN)))//iis1 rx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha1*2 + IISxDMA_RX_EN)));
        cm_input_interrupt_handler(IISDMACha1, &xHigherPriorityTaskWoken);
    }

    if (tmp & (1 << (1 + IISDMACha1*2 + IISxDMA_TX_EN)))//iis1 tx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha1*2 + IISxDMA_TX_EN)));
        // cm_output_interrupt_handler(IISDMACha1, &xHigherPriorityTaskWoken);
        // mprintf("i:%d\n", __LINE__);
    }

    if (tmp & (1 << (1 + IISDMACha2*2 + IISxDMA_RX_EN)))//iis2 rx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha2*2 + IISxDMA_RX_EN)));
        cm_input_interrupt_handler(IISDMACha2, &xHigherPriorityTaskWoken);
    }

    if (tmp & (1 << (1 + IISDMACha2*2 + IISxDMA_TX_EN)))//iis2 tx end
    {
        IISDMA_INT_Clear(IISDMA0,(1 << (1 + IISDMACha2*2 + IISxDMA_TX_EN)));
        // cm_output_interrupt_handler(IISDMACha2, &xHigherPriorityTaskWoken);
        // mprintf("i:%d\n", __LINE__);
    }

	if (tmp & (0x1 << 7)) //iis0 rx BusMatrix完成一次地址轮循
	{
		IISDMA_INT_Clear(IISDMA0,(0x1 << 7));
	}
	if (tmp & (0x1 << 8)) //iis1 rx BusMatrix完成一次地址轮循
	{
		IISDMA_INT_Clear(IISDMA0,(0x1 << 8));
	}
	if (tmp & (0x1 << 9)) //iis2 rx BusMatrix完成一次地址轮循
	{
		IISDMA_INT_Clear(IISDMA0,(0x1 << 9));
	}
	if (tmp & (0x1 << 10)) //iis0 tx BusMatrix完成一次地址轮循
	{
        cm_output_interrupt_handler(IISDMACha0, &xHigherPriorityTaskWoken);
		IISDMA_INT_Clear(IISDMA0,(0x1 << 10));
        // mprintf("i:%d\n", __LINE__);
	}
	if (tmp & (0x1 << 11)) //iis1 tx BusMatrix完成一次地址轮循
	{
        cm_output_interrupt_handler(IISDMACha1, &xHigherPriorityTaskWoken);
		IISDMA_INT_Clear(IISDMA0,(1 << 11));
        // mprintf("i:%d\n", __LINE__);
	}
	if (tmp & (0x1 << 12)) //iis2 tx BusMatrix完成一次地址轮循
	{
        cm_output_interrupt_handler(IISDMACha2, &xHigherPriorityTaskWoken);
		IISDMA_INT_Clear(IISDMA0,(1 << 12));
        // mprintf("i:%d\n", __LINE__);
	}

    // if( pdFALSE != xHigherPriorityTaskWoken )
    {
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}

void IISx_RXDMA_Init(IIS_DMA_RXInit_Typedef* IISDMA_Str)
{
    IISDMAChax iisdmacha = IISDMA_Str->iisdmacha;  
    
    IISDMA_ChannelENConfig(IISDMA0,iisdmacha,IISxDMA_RX_EN,DISABLE);
    IISDMA_ADDRRollBackINT(IISDMA0,iisdmacha,IISxDMA_RX_EN,DISABLE);
    IISDMA_ChannelIntENConfig(IISDMA0,iisdmacha,IISxDMA_RX_EN,DISABLE);

    /*接受地址位rxaddr*/
    IISxDMA_RADDR(IISDMA0,iisdmacha,IISDMA_Str->rxaddr);
    IISxDMA_RNUM(IISDMA0,iisdmacha,IISDMA_Str->rxinterruptsize,IISDMA_Str->rollbackaddrsize,IISDMA_Str->rxsinglesize);

    /*打开IIS的DMA通道*/
    IISDMA_EN(IISDMA0,ENABLE);
}

void IISx_TXDMA_Init(IIS_DMA_TXInit_Typedef* IISDMA_Str)
{
    IISxDMA_TNUM0(IISDMA0,IISDMA_Str->iisdmacha,IISDMA_Str->rollbackaddr0size,IISDMA_Str->tx0singlesize);
    IISxDMA_TNUM1(IISDMA0,IISDMA_Str->iisdmacha,IISDMA_Str->rollbackaddr1size,IISDMA_Str->tx1singlesize);
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/ 

