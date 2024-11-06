/**
 * @file ci130x_dma.c
 * @brief dma驱动
 * @version 1.0
 * @date 2018-05-29
 *
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 *
 */

#include "ci130x_dma.h"
#include <stdint.h>

#define DMA_CHANNEL_NUMBER 2

#define CALL_CALLBACK(func) \
    if (func != NULL)       \
    {                       \
        func();             \
    }

dma_callback_func_ptr_t g_dma_channel_callback[DMA_CHANNEL_NUMBER] = {NULL};
volatile int g_dma_translate_ok[DMA_CHANNEL_NUMBER] = {0};

void set_dma_int_callback(DMACChannelx dmachannel, dma_callback_func_ptr_t func)
{
    if (dmachannel < DMA_CHANNEL_NUMBER)
    {
        g_dma_channel_callback[dmachannel] = func;
    }
}

#if !DRIVER_OS_API
/**
 * @brief 清除dma中断标志
 *
 * @param dmachannel dma通道
 */
void clear_dma_translate_flag(DMACChannelx dmachannel)
{
    if (dmachannel < DMA_CHANNEL_NUMBER)
    {
        g_dma_translate_ok[dmachannel] = 0;
    }
}

/**
 * @brief 等待dma中断标志
 *
 * @param dmachannel dma通道
 * @param timeout 超时时间
 * @return int 0，等到标志；非0，超时
 */
int wait_dma_translate_flag(DMACChannelx dmachannel, uint32_t timeout)
{
    int ret = RETURN_ERR;
    if (dmachannel < DMA_CHANNEL_NUMBER)
    {
        while ((!g_dma_translate_ok[dmachannel]) && (--timeout))
            ;
        g_dma_translate_ok[dmachannel] = 0;
        if (0 == timeout)
        {
            ret = RETURN_ERR;
        }
        else
        {
            ret = RETURN_OK;
        }
    }
    return ret;
}

/**
 * @brief 使用全局变量设置中断触发状态
 *
 */
void dma_without_os_int(void)
{
    if (DMAC_IntTCStatus(DMACChannel0))
    {
        DMAC->DMACIntTCClear = (1 << 0);
        g_dma_translate_ok[DMACChannel0] = 1;
        CALL_CALLBACK(g_dma_channel_callback[DMACChannel0]);
    }
    if (DMAC_IntTCStatus(DMACChannel1))
    {
        DMAC->DMACIntTCClear = (1 << 1);
        g_dma_translate_ok[DMACChannel1] = 1;
        CALL_CALLBACK(g_dma_channel_callback[DMACChannel1]);
    }
}
#else
#include "FreeRTOS.h"
#include "event_groups.h"
static EventGroupHandle_t dma_int_event_group = NULL;
/**
 * @brief dma事件中断标志组初始化
 *
 */
void dma_int_event_group_init(void)
{
    dma_int_event_group = xEventGroupCreate();
    if (dma_int_event_group == NULL)
    {
        mprintf("error\n");
    }
}

/**
 * @brief 清除dma中断标志
 *
 * @param dmachannel dma通道
 */
void clear_dma_translate_flag(DMACChannelx dmachannel)
{
    xEventGroupClearBits(dma_int_event_group, (1 << dmachannel));
}

/**
 * @brief 等待dma中断标志
 *
 * @param dmachannel dma通道
 * @param timeout 超时时间
 * @return int 0，等到标志；非0，超时
 */
int wait_dma_translate_flag(DMACChannelx dmachannel, uint32_t timeout)
{
    EventBits_t uxBits;
    UBaseType_t currPri = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    uxBits = xEventGroupWaitBits(dma_int_event_group, (1 << dmachannel), pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout));
    vTaskPrioritySet(NULL, currPri);
    if (0 != (uxBits & (1 << dmachannel)))
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief 使用事件标志组设置中断触发状态
 *
 */
void dma_with_os_int(void)
{
    static portBASE_TYPE dma_xHigherPriorityTaskWoken = pdFALSE;
    int reg = DMAC->DMACIntTCStatus;
    xEventGroupSetBitsFromISR(dma_int_event_group, reg & 0xf, &dma_xHigherPriorityTaskWoken);
    if (reg & (1 << DMACChannel0))
    {
        CALL_CALLBACK(g_dma_channel0_callback);
    }
    DMAC->DMACIntTCClear = reg;
}
#endif

void dma_irq_handler(void)
{
#if !DRIVER_OS_API
    dma_without_os_int();
#else
    dma_with_os_int();
#endif
}

/**
 * @brief DMA通道屏蔽后的中断状态(只要有通道传输完成中断或者通道传输错误中断的一个)
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return int 0，该通道无中断；非0，该通道有中断
 */
int DMAC_IntStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACIntStatus & (1 << dmachannel);
}

/**
 * @brief DMA通道传输完成中断状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return int 0，该通道无传输完成中断；非0，该通道有传输完成中断
 */
int DMAC_IntTCStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACIntTCStatus & (1 << dmachannel);
}

/**
 * @brief 清除DMA通道传输完成中断状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_IntTCClear(DMACChannelx dmachannel)
{
    if (DMACChannelALL == dmachannel)
    {
        DMAC->DMACIntTCClear = 0xff;
    }
    else
    {
        DMAC->DMACIntTCClear |= (1 << dmachannel);
    }
}

/**
 * @brief DMA通道传输错误中断状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return 0，该通道无传输错误中断；非0，该通道有传输错误中断
 */
int DMAC_IntErrorStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACIntErrorStatus & (1 << dmachannel);
}

/**
 * @brief 清除DMA通道传输错误中断状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_IntErrorClear(DMACChannelx dmachannel)
{
    if (DMACChannelALL == dmachannel)
    {
        DMAC->DMACIntErrClr = 0xff;
    }
    else
    {
        DMAC->DMACIntErrClr |= (1 << dmachannel);
    }
}

/**
 * @brief DMA通道原始的(屏蔽前)中断状态(只要有通道传输完成中断或者通道传输错误中断的一个)
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return 0，该通道无中断；非0，该通道有中断
 */
int DMAC_RawIntTCStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACRawIntTCStatus & (1 << dmachannel);
}

/**
 * @brief DMA通道传输错误原始(屏蔽前)中断状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return 0，该通道无传输错误中断；非0，该通道有传输错误中断
 */
int DMAC_RawErrorIntStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACRawIntErrorStatus & (1 << dmachannel);
}

/**
 * @brief DMA通道使能状态
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @return 0，该通道没有被使能；非0，该通道已使能
 */
int DMAC_ChannelEnableStatus(DMACChannelx dmachannel)
{
    return DMAC->DMACEnbldChns & (1 << dmachannel);
}

/**
 * @brief DMA通道软件突发传输请求
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_SoftwareBurstReq(DMACChannelx dmachannel)
{
    DMAC->DMACSoftBReq |= (1 << dmachannel);
}

/**
 * @brief DMA通道软件单次传输请求
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_SoftwareSingleReq(DMACChannelx dmachannel)
{
    DMAC->DMACSoftSReq |= (1 << dmachannel);
}

/**
 * @brief DMA通道软件最后一次突发传输请求
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_SoftwareLastBurstReq(DMACChannelx dmachannel)
{
    DMAC->DMACSoftLBReq |= (1 << dmachannel);
}

/**
 * @brief DMA通道软件最后一次单次传输请求
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_SoftwareLastSingleReq(DMACChannelx dmachannel)
{
    DMAC->DMACSoftLSReq |= (1 << dmachannel);
}

/**
 * @brief DMA控制器AHB MASTER 字节端序的配置
 *
 * @param dmamaster  DMAC_AHBMaster1，DMAC_AHBMaster2
 * @param endianmode 	LittleENDIANMODE 小端，低字节存放低地址；BigENDIANMODE，大端，低字节存放高地址
 */
void DMAC_Config(DMAC_AHBMasterx dmamaster, ENDIANMODE endianmode)
{
    DMAC->DMACConfiguration &= ~((uint32_t)1 << (1 + dmamaster));
    DMAC->DMACConfiguration |= (endianmode << (1 + dmamaster));
}

/**
 * @brief DMA控制器使能
 *
 * @param cmd  ENABLE,使能；DISABLE,禁止
 */
void DMAC_EN(FunctionalState cmd)
{
    if (cmd != DISABLE)
    {
        DMAC->DMACConfiguration |= (1 << 0);
    }
    else
    {
        for (int i = 0; i < DMA_CHANNEL_NUMBER; i++)
        {
            if (DMAC_ChannelEnableStatus((DMACChannelx)i))
            {
                DMAC_ChannelDisable((DMACChannelx)i);
            }
        }
        DMAC->DMACConfiguration &= ~(1 << 0);
    }
}

/**
 * @brief DMA通道源地址配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param addr 源地址
 */
void DMAC_ChannelSoureAddr(DMACChannelx dmachannel, unsigned int addr)
{
    DMAC->DMACChannel[dmachannel].DMACCxSrcAddr = addr;
}

/**
 * @brief DMA通道目标地址配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param addr 目标地址
 */
void DMAC_ChannelDestAddr(DMACChannelx dmachannel, unsigned int addr)
{
    DMAC->DMACChannel[dmachannel].DMACCxDestAddr = addr;
}

/**
 * @brief DMA通道 LLI 配置
 *
 * @param dmalli 下一个LLI结构体的首地址，(该地址必须是4字节边界对齐),LLI结构体必须依次包含：源地址、目标地址、下一个LLI的首地址、通道控制寄存器
 * @param dmamaster 加载LLI的AHB MASTER
 */
void DMAC_ChannelLLI(DMACChannelx dmachannel, unsigned int dmalli, DMAC_AHBMasterx dmamaster)
{
    DMAC->DMACChannel[dmachannel].DMACCxLLI = ((dmalli >> 2) << 2) | (dmamaster << 0);
}

/**
 * @brief DMA通道源的相关配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param inc 源地址是否增长，INCREMENT，增长；NOINCREMENT，不增长
 * @param dmamaster 源  搬运使用的DMA AHB master接口，
 * @param transferwidth 源 传输数据的位宽
 * @param brustsize 源 突发传输的大小
 */
void DMAC_ChannelSourceConfig(DMACChannelx dmachannel, INCREMENTx inc,
                              DMAC_AHBMasterx dmamaster, TRANSFERWIDTHx transferwidth, BURSTSIZEx brustsize)
{
    DMAC->DMACChannel[dmachannel].DMACCxControl &= ~((1 << 26) | (1 << 24) |
                                                     (0x7 << 18) | (0x7 << 12));
    DMAC->DMACChannel[dmachannel].DMACCxControl |=
        (inc << 26) | (dmamaster << 24) | (transferwidth << 18) | (brustsize << 12);
}

/**
 * @brief DMA通道目标的相关配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param inc 目标地址是否增长，INCREMENT，增长；NOINCREMENT，不增长
 * @param dmamaster 目标  搬运使用的DMA AHB master接口，
 * @param transferwidth 目标 传输数据的位宽
 * @param brustsize 目标 突发传输的大小
 */
void DMAC_ChannelDestConfig(DMACChannelx dmachannel, INCREMENTx inc,
                            DMAC_AHBMasterx dmamaster, TRANSFERWIDTHx transferwidth, BURSTSIZEx brustsize)
{
    DMAC->DMACChannel[dmachannel].DMACCxControl &= ~((1 << 27) | (1 << 25) |
                                                     (0x7 << 21) | (0x7 << 15));
    DMAC->DMACChannel[dmachannel].DMACCxControl |=
        (inc << 27) | (dmamaster << 25) | (transferwidth << 21) | (brustsize << 15);
}

/**
 * @brief DMA通道传输完成中断使能配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param cmd ENABLE，中断使能；DISABLE，中断禁止
 */
void DMAC_ChannelTCInt(DMACChannelx dmachannel, FunctionalState cmd)
{
    if (cmd != ENABLE)
    {
        DMAC->DMACChannel[dmachannel].DMACCxControl &= ~(1 << 31);
    }
    else
    {
        DMAC->DMACChannel[dmachannel].DMACCxControl |= (unsigned int)(1 << 31);
    }
}

/**
 * @brief DMA通道保护位的设置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param access_mode  访问模式，DMAC_ACCESS_USERMODE 用户模式；DMAC_ACCESS_PRIVILEGEDMODE，特权模式
 * @param bufferable 缓冲使能设置，NONBUFFERABLE 无缓冲；BUFFERABLE 缓冲使能
 * @param cacheable cache缓存使能设置，CACHEABLE cache使能；NONCACHEABLE 无cache
 */
void DMAC_ChannelProtectionConfig(DMACChannelx dmachannel, DMAC_ACCESS_MODE access_mode,
                                  BUFFERABLEx bufferable, CACHEABLEx cacheable)
{
    DMAC->DMACChannel[dmachannel].DMACCxControl &= ~(0x7 << 28);
    DMAC->DMACChannel[dmachannel].DMACCxControl |= (((cacheable << 2) | (bufferable << 1) | (access_mode << 0)) << 28);
}

/**
 * @brief DMA通道传输大小配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param size 传输大小(1~4095)，如果为0，DMA不会进行数据传输
 */
void DMAC_ChannelTransferSize(DMACChannelx dmachannel, unsigned short size)
{
    volatile unsigned int tmp = 0;
    size &= 0xFFFUL;

    tmp = DMAC->DMACChannel[dmachannel].DMACCxControl;
    tmp &= ~(0xFFF << 0);
    tmp |= (size << 0);
    DMAC->DMACChannel[dmachannel].DMACCxControl = tmp;
}

/**
 * @brief  当前DMA通道，还剩余多少数据尚未传输，以字节为单位
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 *
 * @return  剩余的，尚未传输的数据长度
 */
unsigned int DMAC_ChannelCurrentTransferSize(DMACChannelx dmachannel)
{
    unsigned int tmp = 0;

    tmp = DMAC->DMACChannel[dmachannel].DMACCxControl;
    return tmp & 0xFFF;
}

/**
 * @brief DMA通道传输禁止
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_ChannelDisable(DMACChannelx dmachannel)
{
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= (1 << 18);
    while (DMAC->DMACChannel[dmachannel].DMACCxConfiguration & (1 << 17))
        ;
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~(1 << 0);
}

/**
 * @brief DMA通道传输使能
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 */
void DMAC_ChannelEnable(DMACChannelx dmachannel)
{
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~(1 << 18);
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= (1 << 0);
}

/**
 * @brief DMA通道中断屏蔽配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param channelintmask CHANNELINTMASK_ITC 传输完成中断；CHANNELINTMASK_IE 传输错误中断
 * @param cmd ENABLE,屏蔽；DISABLE,不屏蔽
 */
void DMAC_ChannelInterruptMask(DMACChannelx dmachannel, CHANNELINTMASKx channelintmask, FunctionalState cmd)
{
    if (cmd != DISABLE)
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~((uint32_t)1 << channelintmask);
    }
    else
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= ((uint32_t)1 << channelintmask);
    }
}

/**
 * @brief DMA通道其他相关配置
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param destperiph 目标外设
 * @param srcperiph  源外设
 * @param flowctrl 流控类型选择
 */
void DMAC_ChannelConfig(DMACChannelx dmachannel, char destperiph,
                        char srcperiph, DMAC_FLOWCTRL flowctrl)
{
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~((0x7 << 11) | (0xF << 6) | (0xF << 1));
    DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= ((flowctrl << 11) | (destperiph << 6) | (srcperiph << 1));
}

/**
 * @brief DMA通道传输锁
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param cmd  ENABLE 传输锁住；DISABLE 传输不锁
 */
void DMAC_ChannelLock(DMACChannelx dmachannel, FunctionalState cmd)
{
    if (cmd != DISABLE)
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= (1 << 16);
    }
    else
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~(1 << 16);
    }
}

/**
 * @brief DMA通道传输(中停)Halt
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param cmd  ENABLE 传输被Halt；DISABLE 传输不Halt
 */
void DMAC_ChannelHalt(DMACChannelx dmachannel, FunctionalState cmd)
{
    if (cmd != DISABLE)
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= (1 << 18);
    }
    else
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~(1 << 18);
    }
}

/**
 * @brief DMA通道传输Power控制
 *
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param cmd  ENABLE Power关闭；DISABLE Power打开
 */
void DMAC_ChannelPowerDown(DMACChannelx dmachannel, FunctionalState cmd)
{
    if (cmd != DISABLE)
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration |= (1 << 19);
    }
    else
    {
        DMAC->DMACChannel[dmachannel].DMACCxConfiguration &= ~(0x1FFF << 19);
    }
}

/**
 * @brief DMA通道传输 内存到内存
 *
 * @note 必须给出合法地址, 实际的物理地址
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param srcaddr  源地址
 * @param destaddr 目标地址
 * @param bytesize 传输的字节数
 */
void DMAC_M2MConfig(DMACChannelx dmachannel, unsigned int srcaddr,
                    unsigned int destaddr, unsigned int bytesize, DMAC_AHBMasterx master)
{
    DMAC_ChannelDisable(dmachannel);
    DMAC_ChannelPowerDown(dmachannel, DISABLE);
    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(master, LittleENDIANMODE);
    DMAC_Config(master, LittleENDIANMODE);
    DMAC_EN(ENABLE);

    DMAC_ChannelSoureAddr(dmachannel, srcaddr);
    DMAC_ChannelDestAddr(dmachannel, destaddr);

    DMAC_ChannelLLI(dmachannel, 0, master);

    DMAC_ChannelProtectionConfig(dmachannel, DMAC_ACCESS_USERMODE, NONBUFFERABLE, NONCACHEABLE);
    DMAC_ChannelTCInt(dmachannel, ENABLE);
    DMAC_ChannelSourceConfig(dmachannel, INCREMENT, master,
                             TRANSFERWIDTH_32b, BURSTSIZE16);
    DMAC_ChannelDestConfig(dmachannel, INCREMENT, master,
                           TRANSFERWIDTH_32b, BURSTSIZE16);
    DMAC_ChannelTransferSize(dmachannel, bytesize / 4 + ((bytesize % 4) ? 1 : 0));

    DMAC_ChannelHalt(dmachannel, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_IE, ENABLE);
    DMAC_ChannelConfig(dmachannel, 0, 0, M2M_DMA);
    DMAC_ChannelLock(dmachannel, DISABLE);
    DMAC_ChannelEnable(dmachannel);
}

/**
 * @brief DMA通道传输 内存到外设，外设到内存
 * @note 不支持外设到外设,必须给出合法地址, 实际的物理地址
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param periph 外设
 *			DMAC_Peripherals_SPI0     QSPI 控制器(发送和接收同一个)
 *			DMAC_Peripherals_SPI1_RX  SPI接收
 *			DMAC_Peripherals_SPI1_TX  SPI发送
 *			DMAC_Peripherals_UART0_RX UART0 接收
 *			DMAC_Peripherals_UART0_TX UART0 发送
 *			DMAC_Peripherals_UART1_RX UART1 接收
 *			DMAC_Peripherals_UART1_TX UART1 发送
 * @param flowctrl  传输方向，M2P_DMA:内存到外设;P2M_DMA 外设到内存
 * @param destaddr 目标地址
 * @param bytesize 传输的字节数
 */
void DMAC_M2P_P2M_advance_config(DMACChannelx dmachannel, DMAC_Peripherals periph,
                                 DMAC_FLOWCTRL flowctrl, unsigned int srcaddr, unsigned int destaddr,
                                 unsigned int bytesize, TRANSFERWIDTHx datawidth, BURSTSIZEx burstsize, DMAC_AHBMasterx master)
{
    //modified by hhx 1024支持UART通过DMA进行byte传输
    INCREMENTx srcinc = INCREMENT, destinc = INCREMENT;

    if (M2P_DMA == flowctrl || M2P_P == flowctrl)
    {
        destinc = NOINCREMENT;
    }
    else if (P2M_DMA == flowctrl || P2M_P == flowctrl)
    {
        srcinc = NOINCREMENT;
    }
    DMAC_ChannelDisable(dmachannel);
    DMAC_ChannelPowerDown(dmachannel, DISABLE);

    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(master, LittleENDIANMODE);
    DMAC_Config(master, LittleENDIANMODE);
    DMAC_EN(ENABLE);

    DMAC_ChannelSoureAddr(dmachannel, srcaddr);
    DMAC_ChannelDestAddr(dmachannel, destaddr);

    DMAC_ChannelLLI(dmachannel, 0, master);

    DMAC_ChannelProtectionConfig(dmachannel, DMAC_ACCESS_USERMODE, NONBUFFERABLE, NONCACHEABLE);
    DMAC_ChannelTCInt(dmachannel, ENABLE);
    DMAC_ChannelSourceConfig(dmachannel, srcinc, master,
                             datawidth, burstsize);
    DMAC_ChannelDestConfig(dmachannel, destinc, master,
                           datawidth, burstsize);
    DMAC_ChannelTransferSize(dmachannel, bytesize / (1 << datawidth) + ((bytesize % (1 << datawidth)) ? 1 : 0));

    DMAC_ChannelHalt(dmachannel, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_IE, ENABLE);

    DMAC_ChannelConfig(dmachannel, periph, periph, flowctrl);
    DMAC_ChannelLock(dmachannel, DISABLE);
    DMAC_ChannelEnable(dmachannel);
}

/**
 * @brief DMA通道传输 内存到外设，外设到内存
 * @note 不支持外设到外设,必须给出合法地址, 实际的物理地址
 * @param dmachannel DMA通道 DMACChannel0~7
 * @param periph 外设
 *			DMAC_Peripherals_SPI0     QSPI 控制器(发送和接收同一个)
 *			DMAC_Peripherals_SPI1_RX  SPI接收
 *			DMAC_Peripherals_SPI1_TX  SPI发送
 *			DMAC_Peripherals_UART0_RX UART0 接收
 *			DMAC_Peripherals_UART0_TX UART0 发送
 *			DMAC_Peripherals_UART1_RX UART1 接收
 *			DMAC_Peripherals_UART1_TX UART1 发送
 * @param flowctrl  传输方向，M2P_DMA:内存到外设;P2M_DMA 外设到内存
 * @param destaddr 目标地址
 * @param bytesize 传输的字节数
 */
void DMAC_M2P_P2MConfig(DMACChannelx dmachannel, DMAC_Peripherals periph,
                        DMAC_FLOWCTRL flowctrl, unsigned int srcaddr, unsigned int destaddr,
                        unsigned int bytesize)
{

    INCREMENTx srcinc = INCREMENT, destinc = INCREMENT;

    if (M2P_DMA == flowctrl || M2P_P == flowctrl)
    {
        destinc = NOINCREMENT;
    }
    else if (P2M_DMA == flowctrl || P2M_P == flowctrl)
    {
        srcinc = NOINCREMENT;
    }

    DMAC_ChannelDisable(dmachannel);
    DMAC_ChannelPowerDown(dmachannel, DISABLE);

    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(DMAC_AHBMaster1, LittleENDIANMODE);
    DMAC_Config(DMAC_AHBMaster2, LittleENDIANMODE);
    DMAC_EN(ENABLE);

    DMAC_ChannelSoureAddr(dmachannel, srcaddr);
    DMAC_ChannelDestAddr(dmachannel, destaddr);

    DMAC_ChannelLLI(dmachannel, 0, DMAC_AHBMaster1);

    DMAC_ChannelProtectionConfig(dmachannel, DMAC_ACCESS_USERMODE, NONBUFFERABLE, NONCACHEABLE);
    DMAC_ChannelTCInt(dmachannel, ENABLE);
    DMAC_ChannelSourceConfig(dmachannel, srcinc, DMAC_AHBMaster1,
                             TRANSFERWIDTH_32b, BURSTSIZE8);
    DMAC_ChannelDestConfig(dmachannel, destinc, DMAC_AHBMaster2,
                           TRANSFERWIDTH_32b, BURSTSIZE8);
    DMAC_ChannelTransferSize(dmachannel, bytesize / 4 + ((bytesize % 4) ? 1 : 0));

    DMAC_ChannelHalt(dmachannel, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_IE, ENABLE);

    DMAC_ChannelConfig(dmachannel, periph, periph, flowctrl);
    DMAC_ChannelLock(dmachannel, DISABLE);
    DMAC_ChannelEnable(dmachannel);
}

void DMAC_ChannelLLIControl(DMACChannelx dmachannel, unsigned int val)
{
    DMAC->DMACChannel[dmachannel].DMACCxControl = val;
}

void DMAC_LLIConfig(DMACChannelx dmachannel, DMAC_LLI *dmac_lli)
{

    //	DMAC_AHBMasterx srcmaster,destmaster;
    //	INCREMENTx srcinc,destinc;

    DMAC_ChannelDisable(dmachannel);
    DMAC_ChannelPowerDown(dmachannel, DISABLE);

    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(DMAC_AHBMaster1, LittleENDIANMODE);
    DMAC_Config(DMAC_AHBMaster2, LittleENDIANMODE);
    DMAC_EN(ENABLE);

    DMAC_ChannelLLIControl(dmachannel, dmac_lli->Control);

    DMAC_ChannelSoureAddr(dmachannel, dmac_lli->SrcAddr);
    DMAC_ChannelDestAddr(dmachannel, dmac_lli->DestAddr);

    DMAC_ChannelLLI(dmachannel, (unsigned int)dmac_lli->NextLLI, DMAC_AHBMaster2);

    DMAC_ChannelHalt(dmachannel, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(dmachannel, CHANNELINTMASK_IE, ENABLE);

    DMAC_ChannelConfig(dmachannel, 0, 0, M2M_DMA); /*m2m*/
    DMAC_ChannelLock(dmachannel, DISABLE);
    DMAC_ChannelEnable(dmachannel);
}

unsigned int DMAC_LLIContrlreturn(LLI_Control *lli_ctrl)
{
    if (!lli_ctrl)
    {
        return 0;
    }

    return (lli_ctrl->TCInt << 31) |
           (lli_ctrl->CacheAble << 30) |
           (lli_ctrl->BufferAble << 29) |
           (lli_ctrl->AccessMode << 28) |
           (lli_ctrl->DestInc << 27) |
           (lli_ctrl->SrcInc << 26) |
           (lli_ctrl->DestMaster << 25) |
           (lli_ctrl->SrcMaster << 24) |
           (lli_ctrl->DestTransferWidth << 21) |
           (lli_ctrl->SrcTransferWidth << 18) |
           (lli_ctrl->DestBurstSize << 15) |
           (lli_ctrl->SrcBurstSize << 12) |
           (lli_ctrl->TransferSize << 0);
}

char *srcbuf[] = {"#0 hello IAR6.7354657657667667562343343432", "#1 chipintelli",
                  "#2 XB3567i7654324567890876543245678909876",
                  "#3 hello HMC6457908-=09876543", "#4 DMA tail657890-0098765", "#5 LLI DMA546789", "#6 LAST LLI", "#7 smteef fer",
                  "#8 3r45t54gntrjhnt", "#9 43ergnjgnjndf", "#10 weregnjdfgb"};
char destbuf[15][200] = {0};

void DMAC_LLItest(DMACChannelx dmachannel)
{
    LLI_Control lli_ctrl = {
        .TCInt = ENABLE,
        .CacheAble = NONCACHEABLE,
        .BufferAble = NONBUFFERABLE,
        .AccessMode = DMAC_ACCESS_USERMODE,
        .DestInc = INCREMENT,
        .SrcInc = INCREMENT,
        .DestMaster = DMAC_AHBMaster1,
        .SrcMaster = DMAC_AHBMaster2,
        .DestTransferWidth = TRANSFERWIDTH_32b,
        .SrcTransferWidth = TRANSFERWIDTH_32b,
        .DestBurstSize = BURSTSIZE4,
        .SrcBurstSize = BURSTSIZE4,
        .TransferSize = 40,
    };
    DMAC_LLI lli[20] = {
        [0] = {
            .SrcAddr = (unsigned int)srcbuf[0],
            .DestAddr = (unsigned int)destbuf[0],
            .NextLLI = (unsigned int)&lli[1],
            .Control = 0,
        },
        [1] = {
            .SrcAddr = (unsigned int)srcbuf[1],
            .DestAddr = (unsigned int)destbuf[1],
            .NextLLI = (unsigned int)&lli[2],
            .Control = 0,
        },
        [2] = {
            .SrcAddr = (unsigned int)srcbuf[2],
            .DestAddr = (unsigned int)destbuf[2],
            .NextLLI = (unsigned int)&lli[3],
            .Control = 0,
        },
        [3] = {
            .SrcAddr = (unsigned int)srcbuf[3],
            .DestAddr = (unsigned int)destbuf[3],
            .NextLLI = (unsigned int)&lli[4],
            .Control = 0,
        },
        [4] = {
            .SrcAddr = (unsigned int)srcbuf[4],
            .DestAddr = (unsigned int)destbuf[4],
            .NextLLI = (unsigned int)&lli[5],
            .Control = 0,
        },
        [5] = {
            .SrcAddr = (unsigned int)srcbuf[5],
            .DestAddr = (unsigned int)destbuf[5],
            .NextLLI = (unsigned int)&lli[6],
            .Control = 0,
        },
        [6] = {
            .SrcAddr = (unsigned int)srcbuf[6],
            .DestAddr = (unsigned int)destbuf[6],
            .NextLLI = (unsigned int)&lli[7],
            .Control = 0,
        },
        [7] = {
            .SrcAddr = (unsigned int)srcbuf[7],
            .DestAddr = (unsigned int)destbuf[7],
            .NextLLI = (unsigned int)&lli[8],
            .Control = 0,
        },
        [8] = {
            .SrcAddr = (unsigned int)srcbuf[8],
            .DestAddr = (unsigned int)destbuf[8],
            .NextLLI = (unsigned int)&lli[9],
            .Control = 0,
        },
        [9] = {
            .SrcAddr = (unsigned int)srcbuf[9],
            .DestAddr = (unsigned int)destbuf[9],
            .NextLLI = (unsigned int)&lli[10],
            .Control = 0,
        },
        [10] = {
            .SrcAddr = (unsigned int)srcbuf[10],
            .DestAddr = (unsigned int)destbuf[10],
            .NextLLI = 0,
            .Control = 0,
        },
        [11] = {0},
    };

    int tmp = DMAC_LLIContrlreturn(&lli_ctrl);

    for (int i = 0; i < 12; i++)
    {
        lli[i].Control = tmp;
    }
#if 0

    eclic_irq_enable(DMA_IRQn);

    scu_set_device_gate((unsigned int)DMA,ENABLE);
#endif
    DMAC_LLIConfig(dmachannel, &lli[0]);
}

/**
 * @brief
 *
 */
void SMT_DMA_LLItest(void)
{
    DMAC_LLItest(DMACChannel0);
}

/**
 * @brief
 *
 */
void GDMA_Traversal(void)
{
    //scu_set_device_gate((unsigned int)DMA,ENABLE);
    //DMAC_LLIConfig();
}

#if SPIC_DMA_MODEFI
/**
 * @brief 配置DMA固定初始化
 *
 * @param channel
 * @param config
 */
void spic_dma_stady_init(DMACChannelx channel)
{
    DMAC_ChannelDisable(channel);
    DMAC_ChannelPowerDown(channel, DISABLE);
    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(DMAC_AHBMaster1, LittleENDIANMODE);
    DMAC_Config(DMAC_AHBMaster2, LittleENDIANMODE);
    DMAC_EN(ENABLE);
    DMAC_ChannelLLI(channel, 0, DMAC_AHBMaster1);
    DMAC_ChannelProtectionConfig(channel, DMAC_ACCESS_USERMODE, NONBUFFERABLE,NONCACHEABLE);
    DMAC_ChannelTCInt(channel, ENABLE);
    DMAC_ChannelHalt(channel, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_IE, ENABLE);
    DMAC_ChannelLock(channel, DISABLE);
}

/**
 * @brief 配置DMA
 *
 * @param channel
 * @param config
 */
void spic_dma_config_temp(DMACChannelx channel, dma_config_temp *config)
{
    INCREMENTx srcinc, destinc;
    if (M2P_DMA == config->flowctrl)
    {
        srcinc = INCREMENT;
        destinc = NOINCREMENT;
    }
    if (P2M_DMA == config->flowctrl)
    {
        srcinc = NOINCREMENT;
        destinc = INCREMENT;
    }
    DMAC_ChannelSourceConfig(channel, srcinc, DMAC_AHBMaster2,config->transferwidth,config->busrtsize);
    DMAC_ChannelDestConfig(channel, destinc, DMAC_AHBMaster1,config->transferwidth,config->busrtsize);
    DMAC_ChannelConfig(channel, DMAC_Peripherals_SPI0, DMAC_Peripherals_SPI0,config->flowctrl);
}

void spic_dma_transfer_config(uint32_t srcaddr, uint32_t destaddr, uint32_t transfersize)
{
    extern DMACChannelx dmachanel;
    DMAC_ChannelSoureAddr(dmachanel, srcaddr);
    DMAC_ChannelDestAddr(dmachanel, destaddr);
    DMAC_ChannelTransferSize(dmachanel, transfersize);
    DMAC_ChannelEnable(dmachanel);
}
#else

void spic_dma_config(DMACChannelx channel, dma_config_t *config)
{
    INCREMENTx srcinc = INCREMENT, destinc = INCREMENT;
    if (M2P_DMA == config->flowctrl || M2P_P == config->flowctrl)
    {
        destinc = NOINCREMENT;
    }
    else if (P2M_DMA == config->flowctrl || P2M_P == config->flowctrl)
    {
        srcinc = NOINCREMENT;
    }
    DMAC_ChannelDisable(channel);
    DMAC_ChannelPowerDown(channel, DISABLE);
    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(DMAC_AHBMaster1, LittleENDIANMODE);
    DMAC_Config(DMAC_AHBMaster2, LittleENDIANMODE);
    DMAC_EN(ENABLE);
    DMAC_ChannelSoureAddr(channel, config->srcaddr);
    DMAC_ChannelDestAddr(channel, config->destaddr);
    DMAC_ChannelLLI(channel, 0, DMAC_AHBMaster1);
    DMAC_ChannelProtectionConfig(channel, DMAC_ACCESS_USERMODE, NONBUFFERABLE,
                                 NONCACHEABLE);
    DMAC_ChannelTCInt(channel, ENABLE);
    DMAC_ChannelSourceConfig(channel, srcinc, DMAC_AHBMaster2,
                             config->transferwidth,
                             config->busrtsize);
    DMAC_ChannelDestConfig(channel, destinc, DMAC_AHBMaster1,
                           config->transferwidth,
                           config->busrtsize);
    DMAC_ChannelTransferSize(channel, config->transfersize);
    DMAC_ChannelHalt(channel, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_IE, ENABLE);
    DMAC_ChannelConfig(channel, DMAC_Peripherals_SPI0, DMAC_Peripherals_SPI0,
                       config->flowctrl);
    DMAC_ChannelLock(channel, DISABLE);
    DMAC_ChannelEnable(channel);
}
#endif //#if SPIC_DMA_MODEFI

/**
 * @brief 配置DMA
 *
 * @param channel
 * @param src
 * @param dest
 * @param size
 */
void psram2ram_dma_config_fordnn(DMACChannelx channel, uint32_t src, uint32_t dest, uint32_t size)
{
    DMAC_ChannelDisable(channel);
    DMAC_ChannelPowerDown(channel, DISABLE);
    DMAC_IntTCClear(DMACChannelALL);
    DMAC_IntErrorClear(DMACChannelALL);
    DMAC_Config(DMAC_AHBMaster1, LittleENDIANMODE);
    DMAC_Config(DMAC_AHBMaster2, LittleENDIANMODE);
    DMAC_EN(ENABLE);
    DMAC_ChannelSoureAddr(channel, src);
    DMAC_ChannelDestAddr(channel, dest);
    DMAC_ChannelLLI(channel, 0, DMAC_AHBMaster1);
    DMAC_ChannelProtectionConfig(channel, DMAC_ACCESS_USERMODE, NONBUFFERABLE, NONCACHEABLE);
    DMAC_ChannelTCInt(channel, ENABLE);
    DMAC_ChannelSourceConfig(channel, INCREMENT, DMAC_AHBMaster1, TRANSFERWIDTH_32b, BURSTSIZE256);
    DMAC_ChannelDestConfig(channel, INCREMENT, DMAC_AHBMaster2, TRANSFERWIDTH_32b, BURSTSIZE256);
    DMAC_ChannelTransferSize(channel, size / 4);
    DMAC_ChannelHalt(channel, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_ITC, DISABLE);
    DMAC_ChannelInterruptMask(channel, CHANNELINTMASK_IE, ENABLE);
    DMAC_ChannelConfig(channel, 0, 0, M2M_DMA);
    DMAC_ChannelLock(channel, DISABLE);
    DMAC_ChannelEnable(channel);
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
