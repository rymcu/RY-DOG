/**
 * @file ci130x_iisdma.c
 * @brief 二代芯片IISDMA底层驱动接口
 * @version 0.1
 * @date 2019-05-10
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include "ci130x_iisdma.h"
#include "ci_log.h"


#define IISDC_CI_ASSERT(x,msg)                                                                                                    \
    if( ( x ) == 0 )                                                                                                        \
    {                                                                                                                       \
        mprintf("%s",msg);                                                                                                   \
        mprintf("IISDC Line:%d\n",__LINE__);                                                                                   \
        while(1)  asm volatile ("ebreak");                                                                                  \
    }

/**
 * @brief IISDMA接收数据使能
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param cmd 使能或者禁止
 */
void IISDMA_DMICModeConfig(IISDMA_TypeDef* IISDMAx, FunctionalState cmd)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(cmd != ENABLE)
    {
        IISDMA->IISDMACTRL &=~(0x3<<20);
    }
    else
    {
        IISDMA->IISDMACTRL |= (0x3<<20);
    }
}


/**
 * @brief IISDMA某个通道的发送或者接收使能
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisdma_txrx_sel IISDMA输入或者输出模式选择
 * @param cmd 使能或者禁止
 */
void IISDMA_ChannelENConfig(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,IISDMA_TXRX_ENx iisdma_txrx_sel, FunctionalState cmd)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    uint32_t iisdma_ctrl_reg = IISDMA->IISDMACTRL;
    // if((iisx >= 3)||(iisdma_txrx_sel >= 3))
    // {
    //     IISDC_CI_ASSERT(0,"iisx or iisdma_txrx_sel err!");
    // }
    if(cmd!=ENABLE)
    {
        iisdma_ctrl_reg &=~(1<<((13+iisx*2+iisdma_txrx_sel) & 0xff));   //
    }
    else
    {
        iisdma_ctrl_reg |=(1<<((13+iisx*2+iisdma_txrx_sel) & 0xff)); //
    }
    IISDMA->IISDMACTRL = iisdma_ctrl_reg;
}


/**
 * @brief IISDMA发送/接收传输完成，BusMatrix需要进行地址切换的中断使能
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisdma_txrx_sel IISDMA输入或者输出模式选择
 * @param cmd 使能或者禁止
 */
void IISDMA_ADDRRollBackINT(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,IISDMA_TXRX_ENx iisdma_txrx_sel,FunctionalState cmd)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    uint32_t iisdma_ctrl_reg = IISDMA->IISDMACTRL;
    if((iisx >= 3)||(iisdma_txrx_sel >= 3))
    {
        IISDC_CI_ASSERT(0,"iisx or iisdma_txrx_sel err!");
    }
    if(cmd!=ENABLE)
    {
        iisdma_ctrl_reg  &= ~(1<<(6+iisx+3*(iisdma_txrx_sel)));
    }
    else
    {
        iisdma_ctrl_reg  |= (1<<(6+iisx+3*(iisdma_txrx_sel)));
    }
    IISDMA->IISDMACTRL = iisdma_ctrl_reg;
}


/**
 * @brief IISDMA发送/接收通道数据传输完成一次中断使能
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisdma_txrx_sel IISDMA输入或者输出模式选择
 * @param cmd 使能或者禁止
 */
void IISDMA_ChannelIntENConfig(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,IISDMA_TXRX_ENx iisdma_txrx_sel,FunctionalState cmd)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if((iisx >= 3)||(iisdma_txrx_sel >= 3))
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    if(cmd!=ENABLE)
    {
        IISDMA->IISDMACTRL &= ~(1<<(iisx*2+iisdma_txrx_sel));
    }
    else
    {
        IISDMA->IISDMACTRL |= (1<<(iisx*2+iisdma_txrx_sel));
    }
}


/**
 * @brief IISDMA使能控制
 *
 * @param IISDMAx IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param cmd 使能或者禁止
 */
void IISDMA_EN(IISDMA_TypeDef* IISDMAx, FunctionalState cmd)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    uint32_t iisdma_ctrl_reg = IISDMA->IISDMACTRL;
    
    if(cmd != ENABLE)
    {
        iisdma_ctrl_reg &=~(1<<19);
    }
    else
    {
        iisdma_ctrl_reg |=(1<<19);
    }
    IISDMA->IISDMACTRL = iisdma_ctrl_reg;
}


/**
 * @brief IISDMA接收地址配置
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param rxaddr 接收地址
 */
void IISxDMA_RADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int rxaddr)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISxDMA[iisx].IISxDMARADDR = rxaddr;
}


/**
 * @brief IISDMA接收数据长度配置
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisrxtointerrupt 接收通道完成（iisrxtointerrupt+1）次请求，产生传输完成中断
 * @param rollbacktimes 接收通道完成（rollbacktimes +1）次传输后，传输地址跳转到RADDR，并产生地址切换中断
 * @param rxsinglesize 接收通道每次请求发送的数据大小 （16*rxsinglesize）字节
 */
void IISxDMA_RNUM(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXxInterrupt iisrxtointerrupt,
                IISDMA_RXTXxRollbackADDR rollbacktimes,IISDMA_TXRXSingleSIZEx rxsinglesize)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if((iisx>= 3))
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISxDMA[iisx].IISxDMARNUM=  ((iisrxtointerrupt & 0x1f)<<16)|
            ((rollbacktimes & 0x3ff)<<6)|((rxsinglesize & 0x1f)<<0);
}


/**
 * @brief IISDMA TADDR0 发送地址寄存器配置
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param txaddr0 TADDR0地址
 */
void IISxDMA_TADDR0(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int txaddr0)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISxDMA[iisx].IISxDMATADDR0 = txaddr0;
}


/**
 * @brief
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param rollbackaddr 发送通道完成（rollbackaddr +1）次传输后，传输地址跳转到TADDR1，并产生传输地址切换中断
 * @param txsinglesize 发送通道每次请求发送的数据大小 （16*txsinglesize）字节
 */
void IISxDMA_TNUM0(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXTXxRollbackADDR rollbackaddr,
                    IISDMA_TXRXSingleSIZEx txsinglesize)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISxDMA[iisx].IISxDMATNUM0 = ((rollbackaddr & 0X3FF) << 6)|
        ((txsinglesize & 0X1F) << 0);
}


/**
 * @brief IISDMA TADDR1 发送地址寄存器配置
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param txaddr1 TADDR1地址
 */
void IISxDMA_TADDR1(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, unsigned int txaddr1)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }	
    IISDMA->IISxDMA[iisx].IISxDMATADDR1 = txaddr1;
}


/**
 * @brief
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param rollbackaddr 发送通道完成（rollbackaddr +1）次传输后，传输地址跳转到TADDR0，并产生地址切换中断
 * @param txsinglesize 发送通道每次请求发送的数据大小 （16*txsinglesize）字节
 */
void IISxDMA_TNUM1(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_RXTXxRollbackADDR rollbackaddr,
                    IISDMA_TXRXSingleSIZEx txsinglesize)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }	
    IISDMA->IISxDMA[iisx].IISxDMATNUM1 = ((rollbackaddr & 0X3FF) <<6)|
        ((txsinglesize & 0X1F)<<0);
}


/**
 * @brief IISDMA优先级设置
 *
 * @param IISDMAx IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisdma_priority 配置优先级响应顺序
 */
void IISDMA_PriorityConfig(IISDMA_TypeDef* IISDMAx, IISDMA_Priorityx iisdma_priority)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    IISDMA->IISDMAPTR = iisdma_priority;
}


/**
 * @brief IISDMA所有中断状态清除
 *
 * @param IISDMAx IISDMAx CI130X：IISDMA0和IISDMA1可选
 */
void IISDMA_INT_All_Clear(IISDMA_TypeDef* IISDMAx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    IISDMA->IISDMACLR = 0x1;
}


/**
 * @brief IISDMA单独清除某个通道的中断
 *
 * @param IISDMAx IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param tmp 需要清除的中断位
 */
void IISDMA_INT_Clear(IISDMA_TypeDef* IISDMAx,unsigned int tmp)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    IISDMA->IISDMACLR = tmp;
}


/**
 * @brief 返回IISDMA TX正在搬运的地址（TADDR0或者TADDR1）
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @return int 0：搬运的是TADDR0的数据
 *             1：搬运的是TADDR1的数据
 */
int IISDMA_TXAddrCheck(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    return IISDMA->IISDMASTATE & (1<<(16+iisx));
}


/**
 * @brief IISDMA发送/接收传输通道完成一次地址切换的状态
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisdma_txrx_sel IISDMA输入或者输出模式选择
 * @return int 非0：通道传输完成一次地址切换；
 *               0：通道传输未完成一次地址切换
 */
int IISDMA_ADDRRollBackSTATE(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_TXRX_ENx iisdma_txrx_sel)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    return IISDMA->IISDMASTATE & (1<<(7+iisx+iisdma_txrx_sel*3));
}


/**
 * @brief IISDMA数据总线busy状态
 *
 * @param IISDMAx IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @return int 0：空闲
 *           非0：数据正在传输
 */
int CHECK_IISDMA_DATABUSBUSY(IISDMA_TypeDef* IISDMAx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    return IISDMA->IISDMASTATE & 0x1;
}


/**
 * @brief 清除IISDMA接收通道完成传输的计数
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 */
void IISDMA_RXCompleteClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISDMAIISCLR |=(1<<(12+iisx));
}


/**
 * @brief 清除IISDMA发送内部的传输次数，并选择下一次传输开始的地址为TADDR0或者TADDR1
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param txrestart_addr 下一次传输的起始地址TADDR0或者TADDR1
 */
//此段代码对时序要求高，两个bit需同时拉
void IISxDMA_TXADDRRollbackInterruptClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx,IISDMA_TXADDR_Sel txrestart_addr)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    uint32_t iisdma_iis_clr_reg = IISDMA->IISDMAIISCLR;
    
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    //IISDMA->IISDMAIISCLR &=~(0x3<<(6+iisx*2));
    
    //IISDMA->IISDMAIISCLR |= (txrestart_addr & 0x1)<<(6+iisx*2);
    iisdma_iis_clr_reg |= (1<<(7+iisx*2)) | ((txrestart_addr & 0x1)<<(6+iisx*2));
    IISDMA->IISDMAIISCLR = iisdma_iis_clr_reg;
}


/**
 * @brief 清除IISDMA接收/发送的传输次数，清除之后，发送和接收的地址为当前配置的地址
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @param iisdmarxtx RX或者TX选择
 */
void IISDMA_RXTXClear(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx, IISDMA_TXRX_ENx iisdmarxtx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    if(iisx >= 3)
    {
        IISDC_CI_ASSERT(0,"iisx err!\n");
    }
    IISDMA->IISDMAIISCLR |= (1 << (iisx*2+iisdmarxtx));
}


/**
 * @brief IISDMA接收通道地址配置
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @return unsigned int
 */
unsigned int IISxDMA_RXADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx)
{
    IISDMA_TypeDef* IISDMA = IISDMAx;
    return IISDMA->IISDMARADDR[iisx & 0x3];
}


/**
 * @brief 返回当前TX正在传输的地址为TADDR0还是TADDR1
 *
 * @param IISDMAx CI130X：IISDMA0和IISDMA1可选
 * @param iisx CI130X：IISCha0、IISCha1、IISCha2可选
 * @return int 0：当前正在传输TADDR0
 *           非0：当前正在传输TADDR1
 */
int IISDMA_TX_ADDR_Get(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx)
{
    return (IISDMAx->IISDMASTATE & (1 << (16+iisx)) );
}


/**
 * @brief 返回当前IISDMA数据接收，存储的首地址
 *
 * @param IISDMAx IISDMA0和IISDMA1可选
 * @param iisx IIS1、IIS2、IIS3可选
 * @return uint32_t 当前IISDMA数据接收，存储的首地址
 */
uint32_t Get_IISxDMA_RADDR(IISDMA_TypeDef* IISDMAx, IISDMAChax iisx)
{
    uint32_t rxaddr = 0;
    IISDMA_TypeDef* IISDMA = IISDMAx;
    rxaddr = IISDMA->IISxDMA[iisx].IISxDMARADDR;
    return rxaddr;
}


/**
 * @brief IIS直通模式配置
 *
 * @param iis CI130X：IIS1、IIS2、IIS3可选
 * @param block_times 传输阈值设置
 * @param endroll_addr 接收地址回卷的最后一个地址
 */
void i2s_dma_straight_to_vad(uint32_t iis,uint32_t block_times,uint32_t endroll_addr)
{
    int32_t vad_sel = 0;
    IISDMA_TypeDef* iisdma = IISDMA0;
    if(HAL_IIS0_BASE == iis)
    {
        //vad_sel = 0;
        //IISDC_CI_ASSERT(0,"para_err\n");
    }
    else if(HAL_IIS1_BASE == iis)
    {
        vad_sel = 0;
    }
    else if(HAL_IIS2_BASE == iis)
    {
        vad_sel = 1;
        iisdma = IISDMA0;
    }

    iisdma->RX_VAD_CTRL = (0 << 24)|(1 << 16)|(vad_sel << 8)|(block_times << 0);
    iisdma->RX_LAST_ADDR = endroll_addr;
    iisdma->IIS_END_NUM_EN = (1 << vad_sel);
}

#include "ci130x_scu.h"
void iisdma_config(iisdma_config_p config)
{
	scu_set_device_gate((unsigned int)config->IISDMAx,ENABLE);
	uint32_t tx_temp0 = config->block_size/(16*config->tx_single_size)*config->iisdma_tx_block_num0;
	uint32_t tx_temp1 = config->block_size/(16*config->tx_single_size)*config->iisdma_tx_block_num1;
	uint32_t rx_temp = config->block_size/(16*config->rx_single_size)*config->iisdma_rx_block_num;
	IISDMA_EN(config->IISDMAx,DISABLE);
    IISDMA_INT_All_Clear(config->IISDMAx);
    
	if(config->iisdma_tx)
	{
        IISxDMA_TXADDRRollbackInterruptClear(config->IISDMAx,config->TxChax,IISDMA_TXAAD_Sel_ADDR0);
        IISDMA_RXTXClear(config->IISDMAx,config->TxChax,IISxDMA_TX_EN);
		IISDMA_ChannelENConfig(config->IISDMAx,config->TxChax,IISxDMA_TX_EN,DISABLE);
		IISDMA_ADDRRollBackINT(config->IISDMAx,config->TxChax,IISxDMA_TX_EN,ENABLE);
		IISDMA_ChannelIntENConfig(config->IISDMAx,config->TxChax,IISxDMA_TX_EN,DISABLE);
		IISxDMA_TADDR0(config->IISDMAx,config->TxChax,config->iisdma_tx_data_addr0);
		IISxDMA_TNUM0(config->IISDMAx,config->TxChax,tx_temp0-1,config->tx_single_size);
		IISxDMA_TADDR1(config->IISDMAx,config->TxChax,config->iisdma_tx_data_addr1);
		IISxDMA_TNUM1(config->IISDMAx,config->TxChax,tx_temp1-1,config->tx_single_size);
		IISDMA_ChannelENConfig(config->IISDMAx,config->TxChax,IISxDMA_TX_EN,ENABLE);
	}
	if(config->iisdma_rx)
	{
        IISDMA_RXCompleteClear(config->IISDMAx,config->RxChax);
        IISDMA_RXTXClear(config->IISDMAx,config->RxChax,IISxDMA_RX_EN);
		IISDMA_ChannelENConfig(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,DISABLE);
		IISDMA_ADDRRollBackINT(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,DISABLE);
		IISDMA_ChannelIntENConfig(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,DISABLE);
		IISxDMA_RADDR(config->IISDMAx,config->RxChax,config->iisdma_rx_data_addr);
		IISxDMA_RNUM(config->IISDMAx, config->RxChax, (rx_temp/config->iisdma_rx_block_num)-1, rx_temp - 1, config->rx_single_size);
		IISDMA_ChannelIntENConfig(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,ENABLE);
        IISDMA_ChannelENConfig(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,ENABLE);
        // IISDMA_ADDRRollBackINT(config->IISDMAx,config->RxChax,IISxDMA_RX_EN,ENABLE);
	}
	IISDMA_EN(config->IISDMAx,ENABLE);

}


/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
