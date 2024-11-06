#include <string.h>
#include <stdlib.h>
#include "ci130x_core_eclic.h"
#include "ci130x_core_misc.h"
#include "platform_config.h"
#include "codec_manager.h"
#include "ci130x_iis.h"
#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "romlib_runtime.h"

#define ZERO_BUFFER_SIZE                 640

#define USE_CODEC  1        //FPGA调试不了CODEC，所以屏蔽

typedef struct
{
    QueueHandle_t busy_queue;                           // 使用中的buffer队列
    QueueHandle_t idle_queue;                           // 空闲的buffer队列
    uint32_t data_begin_addr;                           // 使用中的BUFFER的起始地址
    uint32_t data_end_addr;                             // 使用中的BUFFER的起始地址
    int32_t data_current_addr;                          // BUFFER中正在使用的block起始地址
    uint32_t last_data_begin_addr;                      // 在data_begin_addr切换后，保存还在硬件中播放的BUFFER地址
}play_buffer_queue_t;

typedef struct
{
    uint32_t write_addr;                                //写指针，用于记录正在被IISDMA使用的BUFFER地址
    uint32_t read_addr;                                 //读指针，用于记录下次读取数据的起始地址
    SemaphoreHandle_t recorded_count_sem;               //用于同步已录block数的计数信号量。
}record_buffer_queue_t;

typedef struct
{
    void (*callback_func)(void);                    //指针数组，用于保存每个CODEC的事件回调函数    
    cm_codec_hw_info_t codec_hw_info;               //用于保存CODEC接口（IIS,DMA,IIC等）相关配置。
    cm_sound_info_t play_sound_info;                //用于保存音频相关配置信息。
    cm_sound_info_t record_sound_info;              //用于保存音频相关配置信息。
    play_buffer_queue_t play_buffer_queue;          //DAC PCM buffer 缓存队列。
    cm_play_buffer_info_t play_buffer_info;         //播放音频buffer信息
    cm_record_buffer_info_t record_buffer_info;     //录音音频buffer信息
    record_buffer_queue_t record_buffer_queue;      //ADC PCM buffer 缓存队列。
}codec_info_t;

typedef struct
{
    codec_info_t codec_info[MAX_CODEC_NUM];             //用于保存所有codec相关信息
    char zero_buffer[ZERO_BUFFER_SIZE];                 //用于存放0数据的BUFFER, 主要用于最后一次中断处理之前，CODEC不出噪音
    char reg_flag[MAX_CODEC_NUM];                       //0:未注册; 1:已注册。
    char dma_channel_to_codec_index[MAX_CODEC_NUM+1];     //codec_index索引号与DMA channel的映射表，用于中断处理时，查找对应的codec
    SemaphoreHandle_t semaphore;
}codec_manage_t;

static codec_manage_t cm = {0};

extern unsigned char send_buf_addr[2048];

volatile uint8_t iis_pad_for_clk_input_flag = 0;

/**
 * @brief 标记IIS PAD方向
 *
 */
static int set_iis_pad_for_clk_input_flag(void)
{
    iis_pad_for_clk_input_flag = 1;
}

/**
 * @brief 获取IIS PAD方向
 *
 */
static int get_iis_pad_for_clk_input_flag(void)
{
    return iis_pad_for_clk_input_flag;
}

/**
 * @brief 初始化codec管理器
 *
 */
void cm_init()
{
    memset(&cm, 0, sizeof(cm));
    cm.semaphore = xSemaphoreCreateMutex();
}
 
static IISDMAChax get_iis_dma_channel(iis_base_t IISx)
{
    if (IIS0 == IISx)       //TODO HSL 配置CODEC索引号需要修改接口
    {
        return IISDMACha0;
    }
    else if (IIS1 == IISx)
    {
        return IISDMACha1;
    }
    else if(IIS2 == IISx)
    {
        return IISDMACha2;
    }
    else
    {
        CI_ASSERT(0,"\n");
    }
}

static void iis_clk_gate(IISNumx IISx, FunctionalState cmd)
{
    if(IISNum0 == IISx)
    {
        scu_set_device_gate(HAL_IIS0_BASE,cmd);
    }
    else if(IISNum1_RX == IISx)
    {
        // scu_set_device_gate(HAL_IIS1_BASE,cmd);
        // scu_set_device_gate(HAL_CODEC_BASE,cmd);
        scu_set_device_gate(IIS1_RX_GATE,cmd);
        scu_set_device_gate(CODEC_AD_GATE,cmd);
    }
    else if(IISNum1_TX == IISx)
    {
        // scu_set_device_gate(HAL_IIS1_BASE,cmd);
        // scu_set_device_gate(HAL_CODEC_BASE,cmd);
        scu_set_device_gate(IIS1_TX_GATE,cmd);
        scu_set_device_gate(CODEC_DA_GATE,cmd);
    }
    else if(IISNum2 == IISx)
    {
        scu_set_device_gate(HAL_IIS2_BASE,cmd);
        scu_set_device_gate(HAL_PDM_BASE,cmd);
    }
}

static int init_iis(cm_iis_info_t * p_iis_info, cm_iis_txrx_t txrx)
{    
    iis_base_t IISx = p_iis_info->IISx;
    IISDMAChax iis_dma_channel;
    iis_dma_channel = get_iis_dma_channel(IISx);

    #if 0
    //配置IIS
    if(CM_IIS_TX == txrx)
    {
        iis_tx_enable(IISx,p_iis_info->tx_cha,DISABLE);
    }
    else if(CM_IIS_RX == txrx)
    {
        (IISx,p_iis_info->rx_cha,DISABLE);
    }
    #endif

    //DMA初始化
    scu_set_device_gate((unsigned int)IISDMA0,ENABLE);
    //这两句不屏蔽导致开了语音前处理输出，什么IIS的中断都不来
    // IISDMA_DMICModeConfig(IISDMA0,DISABLE);
    // IISDMA_EN(IISDMA0, DISABLE);
    IISDMA_INT_All_Clear(IISDMA0);
    IISDMA_ChannelENConfig(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,DISABLE);
    IISDMA_ChannelENConfig(IISDMA0,iis_dma_channel,IISxDMA_TX_EN,DISABLE);
    IISDMA_ADDRRollBackINT(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,DISABLE);
    IISDMA_ADDRRollBackINT(IISDMA0,iis_dma_channel,IISxDMA_TX_EN,DISABLE);
    IISDMA_ChannelIntENConfig(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,DISABLE);
    // IISDMA_ChannelIntENConfig(IISDMA0,iis_dma_channel,IISxDMA_TX_EN,DISABLE);
    eclic_irq_enable((uint32_t)IIS_DMA_IRQn);

    return 0;
}

/**
 * @brief 注册codec
 *
 * @param codec_index CODEC注册的索引号，在访问codec_manager其他接口时，用此索引号指定要操作的CODEC。
 * @param codec_hw_info 保存了注册的CODEC的相关信息的结构体。
 * @note 索引号不能大于最大可注册数量(MAX_CODEC_NUM),也不能重复注册同一个索引号。
 * @return 注册是否成功，0：成功；其他值：失败，有可能指定的codec_index超出范围，或者已经被注册过了。
 */
int cm_reg_codec(int codec_index, cm_codec_hw_info_t * p_codec_hw_info)
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        return 1;
    }

    if (cm.reg_flag[codec_index])
    {
        return 2;
    }

    //保存CODEC信息结构
    cm.codec_info[codec_index].codec_hw_info = *p_codec_hw_info;
    cm.reg_flag[codec_index] = 1;

    //如果用到了IIC接口，初始化IIC接口
    if (p_codec_hw_info->IICx)
    {
        pad_config_for_i2c(); //配置IIC引脚
        iic_polling_init(p_codec_hw_info->IICx, 100, 0x10, LONG_TIME_OUT);
    }

    //IIS初始化
    pad_config_for_iis(); //配置IIS引脚
    if (p_codec_hw_info->input_iis.IISx)
    {
        init_iis(&(p_codec_hw_info->input_iis), CM_IIS_RX);
    }

    if (p_codec_hw_info->output_iis.IISx)
    {
        init_iis(&(p_codec_hw_info->output_iis), CM_IIS_TX);
    }

    if (p_codec_hw_info->input_iis.IISx == IIS0 || p_codec_hw_info->output_iis.IISx == IIS0)
    {
        cm.dma_channel_to_codec_index[0] = codec_index;
    }
    if (p_codec_hw_info->input_iis.IISx == IIS1 || p_codec_hw_info->output_iis.IISx == IIS1)
    {
        cm.dma_channel_to_codec_index[1] = codec_index;
    } 
    if (p_codec_hw_info->input_iis.IISx == IIS2 || p_codec_hw_info->output_iis.IISx == IIS2)
    {
        cm.dma_channel_to_codec_index[2] = codec_index;
    }
    
    #if USE_CODEC
    //初始化CODEC
    if (p_codec_hw_info->codec_if.codec_init)
    {
        p_codec_hw_info->codec_if.codec_init(p_codec_hw_info);
    }
    #endif

    return 0;
}


/**
 * @brief 注册指定CODEC的回调函数，用于异步调用方式，CODEC会在特定事件发生时，调用此函数与调用者通信。
 * @param codec_index CODEC注册的索引号，在访问codec_manager其他接口时，用此索引号指定要操作的CODEC。
 * @param callback_func CODEC事件回调函数。
 * @return 注册是否成功，0：成功；其他值：失败，有可能指定的codec_index超出范围。
 */
int cm_register_codec_callback(int codec_index, void (*callback_func)(void))
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        return 2;
    }
    cm.codec_info[codec_index].callback_func = callback_func;
    return 0;
}


static int config_iis_dma(codec_info_t * p_codec_info, io_direction_t io_dir)
{
    IISDMAChax iis_dma_channel;
    uint32_t clk;
    cm_iis_info_t * p_iis_info;
    cm_sound_info_t * sound_info;
    IISNumx device_select;

    if (CODEC_INPUT == io_dir)
    {
        p_iis_info = &(p_codec_info->codec_hw_info.input_iis);
        sound_info = &(p_codec_info->record_sound_info);
        if(IIS1 == p_iis_info->IISx)
        {
            device_select = IISNum1_RX;
        }
    }
    else
    {
        p_iis_info = &(p_codec_info->codec_hw_info.output_iis);
        sound_info = &(p_codec_info->play_sound_info);
        if(IIS1 == p_iis_info->IISx)
        {
            device_select = IISNum1_TX;
        }
    }

    if(IIS0 == p_iis_info->IISx)
    {
        device_select = IISNum0;
    }
    else if(IIS2 == p_iis_info->IISx)
    {
        device_select = IISNum2;
    }
    
    IIS_Clk_ConfigTypedef cap_iis_clk_str;
    if (AUDIO_PLAY_CLK_SOURCE_IPCORE == p_iis_info->clk_source)
    {
        clk = get_ipcore_clk();
    }
    else if (AUDIO_PLAY_CLK_SOURCE_EXT_OSC == p_iis_info->clk_source)
    {
        clk = get_osc_clk();
    }
    else if (AUDIO_PLAY_CLK_SOURCE_INTER_RC == p_iis_info->clk_source)
    {
        clk = get_src_clk(); 
    }
    else if(AUDIO_PLAY_CLK_SOURCE_OSC_OR_INEER_RC == p_iis_info->clk_source)
    {
        #if USE_EXTERNAL_CRYSTAL_OSC
        clk = get_osc_clk();
        #else
        clk = get_src_clk();
        #endif
    }
    else
    {
        clk = p_iis_info->outside_mclk_fre;         
    }

    static const int over_sample_table[] = {128, 192, 256, 384};
    float div_f = (float)clk/(float)sound_info->sample_rate/(float)over_sample_table[p_iis_info->over_sample];
    div_f = div_f + 0.5f;

    cap_iis_clk_str.device_select = device_select;
    cap_iis_clk_str.model_sel = p_iis_info->iis_mode_sel;
    cap_iis_clk_str.src_cfg.source = p_iis_info->clk_source;
    if(AUDIO_PLAY_CLK_SOURCE_OSC_OR_INEER_RC == p_iis_info->clk_source)
    {
        #if USE_EXTERNAL_CRYSTAL_OSC
        cap_iis_clk_str.src_cfg.source = IIS_SRC_SOURCE_EXT_OSC;
        #else
        cap_iis_clk_str.src_cfg.source = IIS_SRC_SOURCE_INTER_RC;
        #endif
    }
    cap_iis_clk_str.src_cfg.source_div = (int16_t)(div_f);
    cap_iis_clk_str.mclk_cfg.fs = p_iis_info->over_sample;    
    cap_iis_clk_str.mclk_cfg.sck_lrck = p_iis_info->sck_lrck_ratio;
    cap_iis_clk_str.mclk_mode = p_iis_info->mclk_out_en;
    cap_iis_clk_str.clk_mode = p_iis_info->scklrck_out_en;//TODO: 待修改
    
    //选择不同的时钟源SRC和MCLK
    if(IIS_MASTER == p_iis_info->iis_mode_sel)
    {
        if (CODEC_INPUT == io_dir)
        {
            cap_iis_clk_str.mclk_cfg.src = IIS_MCLK_SOURCE_SRC0;
            cap_iis_clk_str.mclk_cfg.mclk = IIS_MCLK_MCLK0;
            cap_iis_clk_str.clk_cfg = IIS_CLK_SOURCE_MCLK0;
        }
        else
        {
            cap_iis_clk_str.mclk_cfg.src = IIS_MCLK_SOURCE_SRC1;
            cap_iis_clk_str.mclk_cfg.mclk = IIS_MCLK_MCLK1;
            cap_iis_clk_str.clk_cfg = IIS_CLK_SOURCE_MCLK1;
        }

        if(get_iis_pad_for_clk_input_flag())
        {//如果IIS0（即IIS PAD）配置为slave，IIS PAD不能作为IIS1、IIS2的观测
            cap_iis_clk_str.clk_mode = IIS_SCKLRCK_MODENULL;
        }
    }
    else
    {
        set_iis_pad_for_clk_input_flag();
    }

    iis_clk_gate(device_select, DISABLE);
    iis_clk_config(&cap_iis_clk_str);
    iis_clk_gate(device_select, ENABLE);

    iis_dma_channel = get_iis_dma_channel(p_iis_info->IISx);

    if (CODEC_INPUT == io_dir)
    {
        // 配置DMA
        IIS_DMA_RXInit_Typedef IISDMARX_Init_Struct = {0};
        cm_record_buffer_info_t *p_record_buffer_info = &(p_codec_info->record_buffer_info);
        IISDMARX_Init_Struct.rxaddr = (uint32_t)p_record_buffer_info->pcm_buffer;
        IISDMARX_Init_Struct.rollbackaddrsize = (IISDMA_RXTXxRollbackADDR)(p_record_buffer_info->block_size / 64 * p_record_buffer_info->block_num - 1);
        IISDMARX_Init_Struct.rxinterruptsize = (IISDMA_RXxInterrupt)(p_record_buffer_info->block_size / 64 - 1);
        IISDMARX_Init_Struct.rxsinglesize = IISDMA_TXRXSINGLESIZE64bytes;
        IISDMARX_Init_Struct.iisdmacha = iis_dma_channel;
        IISx_RXDMA_Init(&IISDMARX_Init_Struct);

        // IIS配置
        iis_rx_config_t rx_cfg = {0};
        rx_cfg.cha = p_iis_info->rx_cha;   //接收通道
        rx_cfg.sck_lrck = p_iis_info->sck_lrck_ratio;   //IIS总线上SCK与LRCK的比例关系
        rx_cfg.rx_df = p_iis_info->iis_data_format;   //IIS格式标准
        rx_cfg.rxch_dw = sound_info->sample_depth;   //IIS数据宽度
        rx_cfg.rx_merge = IIS_ENABLE;   //两个 16bit merge
        rx_cfg.rx_swap = IIS_DISABLE;   //数据对调
        rx_cfg.rxfifo_trig = IIS_RX_FIFO_1D4; //接收FIFO触发深度1/4满（32个word）
        if(sound_info->channel_flag == 3)
        {
            rx_cfg.rx_swap = IIS_ENABLE;   //数据对调
            rx_cfg.rx_sc = IIS_SC_STEREO;   //双声道
        }
        else
        {
            rx_cfg.rx_sc = IIS_SC_MONO;    //单声道
            if(sound_info->channel_flag == 2)
            {
                rx_cfg.rx_swap = IIS_ENABLE;   //左声道数据对调到右声道
            }
        }
        iis_rx_config(p_iis_info->IISx,&rx_cfg);
    }
    else
    {
        // 配置DMA
        IIS_DMA_TXInit_Typedef IISDMATX_Init_Struct = {0};
        IISDMA_TXRXSingleSIZEx single_size = IISDMA_TXRXSINGLESIZE32bytes;//IISDMA_TXRXSINGLESIZE16bytes;
        cm_play_buffer_info_t * play_buf_info_t = &p_codec_info->play_buffer_info;
        
        IISDMATX_Init_Struct.rollbackaddr0size = (play_buf_info_t->block_size / (16 * single_size) - 1);
        IISDMATX_Init_Struct.tx0singlesize = single_size;
        IISDMATX_Init_Struct.rollbackaddr1size = (play_buf_info_t->block_size / (16 * single_size) - 1);
        IISDMATX_Init_Struct.tx1singlesize = single_size;
        IISDMATX_Init_Struct.iisdmacha = iis_dma_channel;
        IISx_TXDMA_Init(&IISDMATX_Init_Struct);
        

        // IIS配置
        iis_tx_config_t tx_cfg = {0};
        tx_cfg.cha = p_iis_info->tx_cha;   //接收通道0
        tx_cfg.sck_lrck = p_iis_info->sck_lrck_ratio;   //IIS总线上SCK与LRCK的比例关系
        tx_cfg.tx_df = p_iis_info->iis_data_format;   //IIS格式标准
        tx_cfg.txch_dw = sound_info->sample_depth;   //IIS数据宽度
        tx_cfg.tx_merge = IIS_ENABLE;   //两个 16bit merge
        tx_cfg.tx_swap = IIS_DISABLE;   //数据对调
        tx_cfg.txfifo_trig = IIS_TX_FIFO_1D2; //发送FIFO触发深度1/2空（16个word）
        if(sound_info->channel_flag == 3)
        {//双声道
            tx_cfg.tx_sc = IIS_SC_STEREO;
            tx_cfg.txch_copy = IIS_DISABLE;  //TX 通道复制功能
        }
        else
        {//单声道
            tx_cfg.tx_sc = IIS_SC_MONO;     
            tx_cfg.txch_copy = IIS_ENABLE;  //TX 通道复制功能
        }
        iis_tx_config(p_iis_info->IISx,&tx_cfg);
    }
}

// #pragma GCC optimize("O0")
int cm_config_pcm_buffer(int codec_index, io_direction_t io_dir, cm_pcm_buffer_info_t * pcm_buffer_info)
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        return 2;
    }

    codec_info_t * p_codec_info = &(cm.codec_info[codec_index]);
    
    play_buffer_queue_t * buffer_queue;
    if (CODEC_INPUT == io_dir)
    {
        p_codec_info->record_buffer_info = pcm_buffer_info->record_buffer_info;
        p_codec_info->record_buffer_queue.read_addr = (uint32_t)pcm_buffer_info->record_buffer_info.pcm_buffer;
        p_codec_info->record_buffer_queue.write_addr = (uint32_t)pcm_buffer_info->record_buffer_info.pcm_buffer;
        if (p_codec_info->record_buffer_queue.recorded_count_sem)
        {
            vSemaphoreDelete(p_codec_info->record_buffer_queue.recorded_count_sem);
        }
        p_codec_info->record_buffer_queue.recorded_count_sem = xSemaphoreCreateCounting(
                pcm_buffer_info->record_buffer_info.block_num, 0);
    }
    else
    {
        p_codec_info->play_buffer_info = pcm_buffer_info->play_buffer_info;
        buffer_queue = &(p_codec_info->play_buffer_queue);

        if (buffer_queue->idle_queue)
        {
            // if (p_codec_info->play_buffer_info.buffer_size > p_codec_info->play_buffer_info.buffer_size
            //     || p_codec_info->play_buffer_info.buffer_num > p_codec_info->play_buffer_info.buffer_num)
            {
                // if (buffer_queue->data_begin_addr)
                // {
                //     free((void*)buffer_queue->data_begin_addr);                
                // }
                // void * p_addr;
                // while(uxQueueMessagesWaiting(buffer_queue->idle_queue) > 0)
                // {                
                //     xQueueReceive(buffer_queue->idle_queue, &p_addr, portMAX_DELAY);
                //     free(p_addr);
                // }
                // while(uxQueueMessagesWaiting(buffer_queue->busy_queue) > 0)
                // {
                //     xQueueReceive(buffer_queue->busy_queue, &p_addr, portMAX_DELAY);
                //     free(p_addr);
                // }
                vQueueDelete(buffer_queue->idle_queue);
                vQueueDelete(buffer_queue->busy_queue);
                buffer_queue->idle_queue = 0;
                buffer_queue->busy_queue = 0;
            }
        }
        if (!buffer_queue->idle_queue)
        {
            buffer_queue->idle_queue = xQueueCreate(p_codec_info->play_buffer_info.buffer_num, sizeof(void*));
            buffer_queue->busy_queue = xQueueCreate(p_codec_info->play_buffer_info.buffer_num, sizeof(void*));
        }
        
        play_buffer_queue_t * play_buffer_queue_p = &(cm.codec_info[codec_index].play_buffer_queue);
        for(int i = 0;i < pcm_buffer_info->play_buffer_info.buffer_num;i++)
        {   
            //void *p = pvPortMalloc(p_codec_info->play_buffer_info.buffer_size);
            void *p = (void*)(((uint32_t)pcm_buffer_info->play_buffer_info.pcm_buffer) + pcm_buffer_info->play_buffer_info.buffer_size*i);
            if (!p)
            {
                ci_logerr(CI_LOG_ERROR, "not enough memory\n");
            }
            else
            {
                xQueueSend(play_buffer_queue_p->idle_queue, &p, portMAX_DELAY);
            }
        }

        play_buffer_queue_p->data_begin_addr = (uint32_t)&cm.zero_buffer[0];
        play_buffer_queue_p->data_current_addr = (uint32_t)&cm.zero_buffer[ZERO_BUFFER_SIZE];
        play_buffer_queue_p->data_end_addr = (uint32_t)&cm.zero_buffer[ZERO_BUFFER_SIZE];
    }
}

int cm_config_codec(int codec_index, io_direction_t io_dir, cm_sound_info_t * sound_info)
{
    cm_codec_hw_info_t * p_codec_hw_info;
    codec_info_t * p_codec_info;

    if (MAX_CODEC_NUM <= codec_index)
    {
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        return 2;
    }

    p_codec_info = &(cm.codec_info[codec_index]);    
    p_codec_hw_info = &(p_codec_info->codec_hw_info);

    if (CODEC_INPUT == io_dir)
    {
        p_codec_info->record_sound_info = *sound_info;
    }
    else
    {
        p_codec_info->play_sound_info = *sound_info;
    }

    if (p_codec_info->callback_func)
    {
        p_codec_info->callback_func();
    }

    if (p_codec_hw_info->codec_if.codec_stop)
    {
        p_codec_hw_info->codec_if.codec_stop(io_dir);
    }

    #if USE_CODEC
    // 配置CODEC
    if (p_codec_hw_info->codec_if.codec_config)
    {
        p_codec_hw_info->codec_if.codec_config(sound_info, io_dir);
    }
    #endif

    // 配置IIS    
    config_iis_dma(p_codec_info, io_dir);

    return 0;
}

int cm_start_codec(int codec_index, io_direction_t io_dir)
{
    codec_info_t * code_info_p = &(cm.codec_info[codec_index]);
    iis_base_t IISx;
    play_buffer_queue_t * play_buffer_queue = &(code_info_p->play_buffer_queue);

    xSemaphoreTake(cm.semaphore, portMAX_DELAY);
    if (code_info_p->codec_hw_info.codec_if.codec_start)
    {
        code_info_p->codec_hw_info.codec_if.codec_start(io_dir);
    }

    if (CODEC_INPUT == io_dir)
    {
        IISx = code_info_p->codec_hw_info.input_iis.IISx;
        IISDMAChax iis_dma_channel = get_iis_dma_channel(IISx);   
        IISDMA_ChannelENConfig(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,ENABLE);
        IISDMA_ADDRRollBackINT(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,ENABLE);
        IISDMA_ChannelIntENConfig(IISDMA0,iis_dma_channel,IISxDMA_RX_EN,ENABLE);
        IISDMA_EN(IISDMA0,ENABLE);

        iis_rx_enable(IISx,code_info_p->codec_hw_info.input_iis.rx_cha,ENABLE);
    }
    else
    {
        IISx = code_info_p->codec_hw_info.output_iis.IISx;
        IISDMAChax iis_dma_channel = get_iis_dma_channel(IISx);   
        BaseType_t xResult = xQueueReceive(play_buffer_queue->busy_queue, &(play_buffer_queue->data_begin_addr), 0);
        if (pdTRUE == xResult)
        {
            play_buffer_queue->data_end_addr = play_buffer_queue->data_begin_addr + code_info_p->play_buffer_info.buffer_size;
            ci_logdebug(LOG_AUDIO_PLAY, "get0,%x\n", play_buffer_queue->data_begin_addr);
        }
        else
        {
            play_buffer_queue->data_begin_addr = (uint32_t)&cm.zero_buffer[0];
            // play_buffer_queue->data_end_addr = (uint32_t)&cm.zero_buffer[ZERO_BUFFER_SIZE];
            play_buffer_queue->data_end_addr = ((uint32_t)&cm.zero_buffer[0])+code_info_p->play_buffer_info.block_size;
        }
        
        play_buffer_queue->data_current_addr = play_buffer_queue->data_begin_addr;

        IISxDMA_TADDR0(IISDMA0,iis_dma_channel,play_buffer_queue->data_current_addr);
        play_buffer_queue->data_current_addr += cm.codec_info[codec_index].play_buffer_info.block_size;
        if (play_buffer_queue->data_begin_addr < play_buffer_queue->data_end_addr)
        {
            IISxDMA_TADDR1(IISDMA0,iis_dma_channel,play_buffer_queue->data_current_addr);
            play_buffer_queue->data_current_addr += cm.codec_info[codec_index].play_buffer_info.block_size;
        }
        else
        {
            IISxDMA_TADDR1(IISDMA0,iis_dma_channel,(uint32_t)&cm.zero_buffer[0]);
        }
        play_buffer_queue->last_data_begin_addr = 0;
        IISxDMA_TXADDRRollbackInterruptClear(IISDMA0,iis_dma_channel,IISDMA_TXAAD_Sel_ADDR0);
        IISDMA_RXTXClear(IISDMA0,iis_dma_channel,IISxDMA_TX_EN);
        IISDMA_ChannelENConfig(IISDMA0,iis_dma_channel,IISxDMA_TX_EN,ENABLE);
        IISDMA_ADDRRollBackINT(IISDMA0,iis_dma_channel,IISxDMA_TX_EN,ENABLE);
        
        IISDMA_EN(IISDMA0,ENABLE);
        iis_tx_enable(IISx,code_info_p->codec_hw_info.output_iis.tx_cha,ENABLE);
    }

    xSemaphoreGive(cm.semaphore);
    return 0;
}

int cm_stop_codec(int codec_index, io_direction_t io_dir)
{
    xSemaphoreTake(cm.semaphore, portMAX_DELAY);
    if (CODEC_OUTPUT == io_dir)
    {
        void * pcm_buffer;
        BaseType_t rst;

        // 已写入busy queue的buffer转移到idle queue
        while((rst = xQueueReceive(cm.codec_info[codec_index].play_buffer_queue.busy_queue, &pcm_buffer, 5)) == pdTRUE)
        {
            xQueueSend(cm.codec_info[codec_index].play_buffer_queue.idle_queue, &pcm_buffer, 5);
        }

        // 等待硬件正在播放的buffer播放完成，转入idle queue
        while(uxQueueMessagesWaiting(cm.codec_info[codec_index].play_buffer_queue.idle_queue) != cm.codec_info[codec_index].play_buffer_info.buffer_num)
        {
            // mprintf("idle queue buffer count: %d\n", uxQueueMessagesWaiting(cm.codec_info[codec_index].play_buffer_queue.idle_queue));
            vTaskDelay(1);
        }

        codec_info_t * code_info_p = &(cm.codec_info[codec_index]);
        IISDMA_TypeDef* IISDMAx = IISDMA0;
        iis_base_t IISx = code_info_p->codec_hw_info.output_iis.IISx;
        IISDMAChax iis_dma_channel = get_iis_dma_channel(IISx);

        IISDMA_ChannelENConfig(IISDMAx,iis_dma_channel,IISxDMA_TX_EN,DISABLE);
        IISDMA_ADDRRollBackINT(IISDMAx,iis_dma_channel,IISxDMA_TX_EN,DISABLE);
        iis_tx_enable(IISx,code_info_p->codec_hw_info.output_iis.tx_cha,DISABLE);
        if (cm.codec_info[codec_index].codec_hw_info.codec_if.codec_stop)
        {
            cm.codec_info[codec_index].codec_hw_info.codec_if.codec_stop(io_dir);
        }
    }
    xSemaphoreGive(cm.semaphore);
    return 0;
}

int cm_read_codec(int codec_index, uint32_t * data_addr, uint32_t * data_size)
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        ci_logerr(CI_LOG_ERROR, "parameter error\n");
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        ci_logerr(CI_LOG_ERROR, "specified codec_index is not registered\n");
        return 2;
    }

    codec_info_t * code_info_p = &(cm.codec_info[codec_index]);
    record_buffer_queue_t * record_buffer_queue = &(code_info_p->record_buffer_queue);
    cm_record_buffer_info_t * record_buffer_info = &(code_info_p->record_buffer_info);

    // if (pdTRUE == xSemaphoreTake(record_buffer_queue->recorded_count_sem, portMAX_DELAY))
    if (pdTRUE == xSemaphoreTake(record_buffer_queue->recorded_count_sem, pdMS_TO_TICKS(300)))
    {
        portENTER_CRITICAL();
        *data_addr = record_buffer_queue->read_addr;
        *data_size = record_buffer_info->block_size;
        record_buffer_queue->read_addr += record_buffer_info->block_size;
        if (record_buffer_queue->read_addr >= (uint32_t)record_buffer_info->pcm_buffer + record_buffer_info->buffer_size)
        {
            // 读指针回环
            record_buffer_queue->read_addr = (uint32_t)record_buffer_info->pcm_buffer;
        }
        portEXIT_CRITICAL();
    }
}

int cm_write_codec(int codec_index, void * pcm_buffer,uint32_t wait_tick)
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        ci_logerr(CI_LOG_ERROR, "parameter error\n");
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        ci_logerr(CI_LOG_ERROR, "specified codec_index is not registered\n");
        return 2;
    }

    if (!cm.codec_info[codec_index].play_buffer_queue.idle_queue)
    {
        ci_logerr(CI_LOG_ERROR, "specified codec is not configered\n");
        return 3;
    }
    
    BaseType_t rst;
    if(check_curr_trap())
    {
        BaseType_t task_woken = pdFALSE;
        rst = xQueueSendFromISR(cm.codec_info[codec_index].play_buffer_queue.busy_queue, &pcm_buffer, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
    else
    {
        rst = xQueueSend(cm.codec_info[codec_index].play_buffer_queue.busy_queue, &pcm_buffer, wait_tick);
        ci_logdebug(LOG_AUDIO_PLAY, "write,%x\n", (uint32_t)pcm_buffer);
    }

    if (rst != pdTRUE)
    {
        mprintf("send msg err[%d]\n",codec_index);
    }
    return 0;
}

void cm_get_pcm_buffer(int codec_index,uint32_t* ret_buf,uint32_t wait_tick)
{
    void *ret = NULL;
    do
    {
        if (MAX_CODEC_NUM <= codec_index)
        {
            ci_logerr(CI_LOG_ERROR, "parameter error\n");
            break;
        }

        if (!cm.reg_flag[codec_index])
        {
            ci_logerr(CI_LOG_ERROR, "specified codec_index is not registered\n");
            break;
        }

        if (!cm.codec_info[codec_index].play_buffer_queue.idle_queue)
        {
            ci_logerr(CI_LOG_ERROR, "specified codec is not configered\n");
            break;
        }
        
        BaseType_t rst;
        if(check_curr_trap())
        {
            BaseType_t task_woken = pdFALSE;
            rst = xQueueReceiveFromISR(cm.codec_info[codec_index].play_buffer_queue.idle_queue, &ret, &task_woken);
            portYIELD_FROM_ISR(task_woken);
        }
        else
        {
            rst = xQueueReceive(cm.codec_info[codec_index].play_buffer_queue.idle_queue, &ret, wait_tick);
        }
        
        ci_logdebug(LOG_AUDIO_PLAY, "decode,%x\n", ret);
        if (rst != pdTRUE)
        {
            mprintf("no pcm buffer\n");
        }
    }while(0);

    *ret_buf = (uint32_t)ret;
}

int cm_release_pcm_buffer(int codec_index, io_direction_t io_dir, void * pcm_buffer)
{
    if (MAX_CODEC_NUM <= codec_index)
    {
        ci_logerr(CI_LOG_ERROR, "parameter error\n");
        return 1;
    }

    if (!cm.reg_flag[codec_index])
    {
        ci_logerr(CI_LOG_ERROR, "specified codec_index is not registered\n");
        return 2;
    }

    if (CODEC_OUTPUT == io_dir)
    {
        if (!cm.codec_info[codec_index].play_buffer_queue.idle_queue)
        {
            ci_logerr(CI_LOG_ERROR, "specified codec is not configered\n");
            return 3;
        }
        xQueueSend(cm.codec_info[codec_index].play_buffer_queue.idle_queue, &pcm_buffer, portMAX_DELAY);
        ci_logdebug(LOG_AUDIO_PLAY, "release,%x\n", pcm_buffer);
    }
    return 0;
}

/**
 * @brief 设置DAC的增益（0 —— 100）
 * 
 * @param codec_index CODEC编号
 * @param cha 通道 
 * @param gain 增益值 
 * @return int 
 */
int cm_set_codec_dac_gain(int codec_index, cm_cha_sel_t cha, int gain)
{
    if (cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl)
    {
        cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl(CODEC_OUTPUT, CM_IOCTRL_SET_DAC_GAIN, cha, gain);
    }
}


/**
 * @brief 设置ADC的gain，只开放PGA增益调节
 * 
 * @param codec_index 
 * @param cha 通道
 * @param gain 增益（-18dB —— 28dB）
 * @return int 
 */
int cm_set_codec_adc_gain(int codec_index, cm_cha_sel_t cha, int gain)
{
    if (cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl)
    {
        cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl(CODEC_OUTPUT, CM_IOCTRL_SET_ADC_GAIN, cha, gain);
    }
}


/**
 * @brief CODEC ADC ALC开关的接口
 * 
 * @param codec_index CODEC编号
 * @param cha 左声道或者右声道，不可以双声道
 * @param alc_enable ALC开关
 * @return int 
 */
int cm_set_codec_alc(int codec_index, cm_cha_sel_t cha, FunctionalState alc_enable)
{
    if (cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl)
    {
        if(ENABLE == alc_enable)
        {
            cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl(CODEC_INPUT, CM_IOCTRL_ALC_ENABLE, cha,0);
        }
        else
        {
            cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl(CODEC_INPUT, CM_IOCTRL_ALC_DISABLE, cha,0);
        }
    }
}


int cm_set_codec_dac_enable(int codec_index, int channel, FunctionalState en)
{
    if (cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl)
    {
        cm.codec_info[codec_index].codec_hw_info.codec_if.codec_ioctl(CODEC_OUTPUT, CM_IOCTRL_DAC_ENABLE, channel, en);
    }
}


int cm_get_codec_empty_buffer_number(int codec_index, io_direction_t io_dir)
{
    UBaseType_t count = -1;
    if (CODEC_OUTPUT == io_dir)
    {
        count = uxQueueMessagesWaiting(cm.codec_info[codec_index].play_buffer_queue.idle_queue);
    }
    return (int)(count > 0) ? (count - 1) : 0;
}

int cm_get_codec_busy_buffer_number(int codec_index, io_direction_t io_dir)
{
    UBaseType_t count = -1;
    if (CODEC_OUTPUT == io_dir)
    {
        count = uxQueueMessagesWaiting(cm.codec_info[codec_index].play_buffer_queue.idle_queue);
    }
    return cm.codec_info[codec_index].play_buffer_info.buffer_num - count;
}

int cm_set_codec_mute(int codec_index, io_direction_t io_dir, int channel_flag, FunctionalState en)
{
    uint32_t iis_base = (uint32_t)cm.codec_info[codec_index].codec_hw_info.input_iis.IISx;
    if(io_dir == CODEC_INPUT)
    {
        iis_rx_channal_t cha = cm.codec_info[codec_index].codec_hw_info.input_iis.rx_cha;
        iis_rx_mute(iis_base,cha,en);
    }   
    else
    {
        iis_tx_channal_t cha = cm.codec_info[codec_index].codec_hw_info.input_iis.tx_cha;
        if(3 == channel_flag)
        {   
            iis_tx_l_mute(iis_base,cha,en);
            iis_tx_r_mute(iis_base,cha,en);
        }
        else if(1 == channel_flag)
        {
            iis_tx_l_mute(iis_base,cha,en);
        }
        else if(2 == channel_flag)
        {
            iis_tx_r_mute(iis_base,cha,en);
        }
        else
        {
            mprintf("cha err\n");
        }
    }   
}

void cm_input_interrupt_handler(IISDMAChax dma_channel, BaseType_t * xHigherPriorityTaskWoken)
{
    int codec_index = cm.dma_channel_to_codec_index[dma_channel];
    codec_info_t * code_info_p = &(cm.codec_info[codec_index]);
    record_buffer_queue_t * record_buffer_queue = &(code_info_p->record_buffer_queue);
    cm_record_buffer_info_t * record_buffer_info = &(code_info_p->record_buffer_info);

    // 1个block写满了，向后移动一个block
    record_buffer_queue->write_addr += record_buffer_info->block_size;
    if (record_buffer_queue->write_addr >= (uint32_t)record_buffer_info->pcm_buffer + record_buffer_info->buffer_size)
    {
        // 最后一个block写满了，回环到buffer的第一个block
        record_buffer_queue->write_addr = (uint32_t)record_buffer_info->pcm_buffer;
    }

    // 如果写指针赶上读指针了，说明取数据太慢了，强制移动读指针，丢弃以前的数据。
    if (record_buffer_queue->write_addr == record_buffer_queue->read_addr)
    {
        record_buffer_queue->read_addr += record_buffer_info->block_size;
        if (record_buffer_queue->read_addr >= (uint32_t)record_buffer_info->pcm_buffer + record_buffer_info->buffer_size)
        {
            // 读指针回环
            record_buffer_queue->read_addr = (uint32_t)record_buffer_info->pcm_buffer;
        }
        // volatile uint32_t delay = 10000;
        // while(delay--);
        
        mprintf("record buffer overflow!\n");
    }
    else
    {
        xSemaphoreGiveFromISR(record_buffer_queue->recorded_count_sem, xHigherPriorityTaskWoken);
    }
}

__attribute__((weak)) void send_one_buf_done_event(void * xHigherPriorityTaskWoken){}


void cm_output_interrupt_handler(IISDMAChax dma_channel, BaseType_t * xHigherPriorityTaskWoken)
{    
    int codec_index = cm.dma_channel_to_codec_index[dma_channel];
    codec_info_t * code_info_p = &(cm.codec_info[codec_index]);
    play_buffer_queue_t * play_buffer_queue = &(code_info_p->play_buffer_queue);

    // 先把刚用过的buffer放入空闲队列
    if (play_buffer_queue->last_data_begin_addr)
    {
        xQueueSendFromISR(play_buffer_queue->idle_queue, &(play_buffer_queue->last_data_begin_addr), xHigherPriorityTaskWoken);
        play_buffer_queue->last_data_begin_addr = 0;
        ci_logdebug(LOG_AUDIO_PLAY, "idle,%x\n", play_buffer_queue->data_begin_addr);
        extern void send_one_buf_done_event(void * xHigherPriorityTaskWoken); 
        //启动了播报才能打开这个
        #if AUDIO_PLAYER_ENABLE
        send_one_buf_done_event(&xHigherPriorityTaskWoken);
        #endif
    }
    // buffer未播完的情况
    if (play_buffer_queue->data_current_addr >= play_buffer_queue->data_end_addr || play_buffer_queue->data_begin_addr == (uint32_t)cm.zero_buffer)
    {        
        if (play_buffer_queue->data_begin_addr != (uint32_t)&cm.zero_buffer[0])
        {
            play_buffer_queue->last_data_begin_addr = play_buffer_queue->data_begin_addr;
        }

        // 再看队列中是否还有buffer可以播放
        BaseType_t xResult = xQueueReceiveFromISR(play_buffer_queue->busy_queue, &(play_buffer_queue->data_begin_addr), xHigherPriorityTaskWoken);
        if (pdTRUE == xResult)
        {
            //队列里面有数据可以播放
            play_buffer_queue->data_end_addr = play_buffer_queue->data_begin_addr + code_info_p->play_buffer_info.buffer_size;
            ci_logdebug(LOG_AUDIO_PLAY, "play,%x\n", play_buffer_queue->data_begin_addr);
        }
        else
        {
            // 无数据可以播放，就播放0 buffer.
            play_buffer_queue->data_begin_addr = (uint32_t)cm.zero_buffer;
            play_buffer_queue->data_end_addr = play_buffer_queue->data_begin_addr + code_info_p->play_buffer_info.block_size;//(uint32_t)&cm.zero_buffer[ZERO_BUFFER_SIZE];
            ci_logdebug(LOG_AUDIO_PLAY, "zer,%x\n", play_buffer_queue->data_begin_addr);
        }
        play_buffer_queue->data_current_addr = play_buffer_queue->data_begin_addr;        
    }

    int used_buffer_index = IISDMA_TX_ADDR_Get(IISDMA0, dma_channel);
    if(0 == used_buffer_index)
    {
        IISxDMA_TADDR1(IISDMA0,dma_channel,play_buffer_queue->data_current_addr);        
    }
    else
    {
        IISxDMA_TADDR0(IISDMA0,dma_channel,play_buffer_queue->data_current_addr);
    }
    play_buffer_queue->data_current_addr += code_info_p->play_buffer_info.block_size;
}

#if 0
#include "ci130x_inner_codec.h"

#define IIS_OUT_INDEX               0
#define BLOCK_SIZE_TEST              (320)/*16ms perframe*/

const cm_codec_hw_info_t inner_codec_info = 
{
    .IICx = IIC_NULL,
    .output_iis.IISx = IIS1,
    .output_iis.iis_mode_sel = IIS_MASTER,
    .output_iis.over_sample = IIS_MCLK_FS_256,
    .output_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_INTER_RC,
    .output_iis.mclk_out_en = IIS_MCLK_OUT,
    .output_iis.iis_data_format = IIS_DF_IIS,
    .output_iis.sck_lrck_ratio = IIS_SCK_LRCK_64,
    .output_iis.tx_cha = IIS_TX_CHANNAL_TX0,
    .output_iis.scklrck_out_en = IIS_SCKLRCK_OUT,

    .input_iis.IISx = IIS1,
    .input_iis.iis_mode_sel = IIS_SLAVE,
    .input_iis.over_sample = IIS_MCLK_FS_256,
    .input_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_IPCORE,
    .input_iis.mclk_out_en = IIS_MCLK_OUT,
    .input_iis.iis_data_format = IIS_DF_IIS,
    .input_iis.sck_lrck_ratio = IIS_SCK_LRCK_64,
    .input_iis.rx_cha = IIS_RX_CHANNAL_RX0,
    .input_iis.scklrck_out_en = IIS_SCKLRCK_IN,

    .codec_if = 
    {
        #if USE_CODEC//TODO待修改CODEC驱动的接口
        .codec_init = icodec_init,
        .codec_config = icodec_config,
        .codec_start = icodec_start,
        .codec_stop = icodec_stop,
        .codec_ioctl = icodec_ioctl,
        #endif
    }
}; 



const cm_codec_hw_info_t iis_out_hw_info = 
{
    .IICx = IIC_NULL,
    .output_iis.IISx = IIS0,
    .output_iis.iis_mode_sel = IIS_MASTER,
    .output_iis.over_sample = IIS_MCLK_FS_256,
    .output_iis.clk_source = AUDIO_PLAY_CLK_SOURCE_IPCORE,
    .output_iis.mclk_out_en = IIS_MCLK_OUT,
    .output_iis.iis_data_format = IIS_DF_IIS,
    .output_iis.sck_lrck_ratio = IIS_SCK_LRCK_64,
    .output_iis.tx_cha = IIS_TX_CHANNAL_TX0,
    .codec_if = 
    {
        .codec_init = NULL,
        .codec_config = NULL,
        .codec_start = NULL,
        .codec_stop = NULL,
        .codec_ioctl = NULL,
    }
}; 

const cm_sound_info_t record_sound_info = {
    .sample_rate = 16000,
    .channel_flag = 3,          //双声道录制
    .sample_depth = IIS_DW_16BIT,
};


void record_task(void* p)
{   
    // 注册内部codec
    cm_reg_codec(0, (cm_codec_hw_info_t*)&inner_codec_info);
    // 注册IIS输出
    //cm_reg_codec(IIS_OUT_INDEX, (cm_codec_hw_info_t*)&iis_out_hw_info);
#if 0
    //配置录音PCM buffer
    cm_record_buffer_info_t record_buffer_info;
    record_buffer_info.block_num = 2;
    record_buffer_info.block_size = 320*4;//576;
    record_buffer_info.buffer_size = record_buffer_info.block_size * record_buffer_info.block_num;
    record_buffer_info.pcm_buffer = pvPortMalloc(record_buffer_info.buffer_size);
    cm_config_pcm_buffer(0, CODEC_INPUT, &record_buffer_info);
    //配置录音音频格式
    cm_config_codec(0, CODEC_INPUT, &record_sound_info);
    //开始录音
    cm_start_codec(0, CODEC_INPUT);
#endif
    //配置IIS输出PCM buffer
    cm_play_buffer_info_t pcm_buffer_info;
    pcm_buffer_info.block_num = 2;
    pcm_buffer_info.buffer_num = 4;
    pcm_buffer_info.block_size = 320*2;
    pcm_buffer_info.buffer_size = pcm_buffer_info.block_size * pcm_buffer_info.block_num;
    pcm_buffer_info.pcm_buffer = pvPortMalloc(pcm_buffer_info.buffer_size*pcm_buffer_info.buffer_num);
    // cm_config_pcm_buffer(IIS_OUT_INDEX, CODEC_OUTPUT, &pcm_buffer_info);//TODO err
    cm_config_pcm_buffer(IIS_OUT_INDEX, CODEC_OUTPUT, (cm_pcm_buffer_info_t*)&pcm_buffer_info);
    //配置IIS放音音频格式
    cm_sound_info_t play_sound_info = record_sound_info;
    cm_config_codec(IIS_OUT_INDEX, CODEC_OUTPUT, &play_sound_info);
    //开始放音
    cm_start_codec(IIS_OUT_INDEX, CODEC_OUTPUT);

    // cm_set_codec_dac_gain(0, 50, 50);
    while(1)
    {
        #if 0
        // 录音转放音
        uint32_t data_addr, data_size;
        cm_read_codec(0, &data_addr, &data_size);
        volatile uint32_t out_p = NULL;
        cm_get_pcm_buffer(1,&out_p,portMAX_DELAY);
        
        if (out_p)
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(out_p, data_addr, BLOCK_SIZE_TEST*2*2);
            cm_write_codec(1, out_p,portMAX_DELAY);
        }
        #endif

        volatile uint32_t out_p; 
        cm_get_pcm_buffer(0,&out_p,portMAX_DELAY);
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)out_p, (void*)send_buf_addr, BLOCK_SIZE_TEST*2*2);
        cm_write_codec(0, (void*)out_p,portMAX_DELAY);

        vTaskDelay(5000);
    }
}

void iis_test(void)
{
    xTaskCreate(record_task,"record_task",256,NULL,4,NULL);
}
#endif
