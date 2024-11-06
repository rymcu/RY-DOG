/**
 * @file audio_play_process.c
 * @brief  播放器
 * @version 2.8.3
 * @date 2019-03-15
 *       V2.8.3 增加功能提供音频播放达到一个临界值后释放一个信号回调函数
 *       V2.8.2 解决使用sd卡播放非根目录文件文件读取异常
 *       V2.8.1 解决IMAADPCM格式输出的size非16倍数导致iisdma无法正常输出(V2.6.0版本引入)
 *       V2.8.0 播放修改提示音解码器，增加支持兼容CI_FLAC文件格式，WAV文件支持8K采样率
 *       V2.7.0 播放修改提示音解码器，支持兼容标准48头WAV、IMAADPCM、CI_MP3、旧版CIADPCM文件格式
 *              修复AAC解码器暂停继续上下文错误的问题(V2.6.0版本引入)
 *       V2.6.1 修复标准IMA ADPCM格式解码器输出单位块size值不正确导致声卡获取的size出错问题
 *       V2.6.0 增加标准IMA ADPCM格式的命令词提示音
 *       V2.5.3 播放响应速度优化
 *       V2.5.2 声卡抗POP策略更新优化
 *       V2.5.1 增加设备接口调音量抗POP策略,主流程更新优化暂停响应速度
 *       V2.5.0 增加播放TTS后处理功能，目前支持倍速播放
 *       V2.4.7 修复一些解码器移植跳过错误帧配置问题
 *       V2.4.6 修复在数据阻塞时，处理停止事件无法快速响应
 *       V2.4.5 针对m4a头解析使用moov的size来动态申请内存解析
 *       V2.4.4 增加双声道音频混音到单声道
 *       V2.4.3 针对ci flash特性优化播放adpcm
 *       V2.4.2 修复m4a atom解析错误导致流程出错
 *       V2.4.1 部分错误处理机制完善，播放器控制返回出口提高代码易维护性
 *       V2.4.0 增加文件头解析逻辑，后续可以直接调用播放不需要预先声明文件类型
 *       V2.3.1 修复部分错误
 *       V2.3.0 播放器组件添加OS移植层，方便针对不同OS移植
 *       V2.2.0 抽象解码器框架到audio_play_decoder.c中，使用注册的方式添加解码器
 *       V2.1.1 根据解码器进度记录播放位置，以帮助精确暂停恢复，同时保证恢复时mp3解码器信息连续不丢失
 *       V2.1.0 api接口移动至audio_play_api.h、硬件相关配置移动到audio_play_device.h
 *       V2.0.2 数据获取彻底解耦，sd卡挂载、打开、保存、关闭均交给get_play_data.c
 *       V2.0.1 硬件iis、codec解耦，采用最新声卡驱动，单声道处理改用硬件方式
 *       V2.0.0 添加网络mp3播放功能，进一步将数据获取与解码器解耦
 *       V1.1.0 将audio date获取读取部分逻辑向get_play_data.c转移，添加本地mp3打断功能
 *       V1.0.0 音频播放器
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sdk_default_config.h"
#include "audio_play_process.h"
#include "audio_play_device.h"
#include "audio_play_api.h"
#include "audio_play_decoder.h"
#include "get_play_data.h"
#include "debug_time_consuming.h"
#include "ci130x_system.h"
#include "ci_log.h"
#include "sdk_default_config.h"
#include "parse_m4a_atom_containers_port.h"
#include "parse_m4a_atom_containers.h"
// #include "flash_rw_process.h"
#include "adpcmdec.h"
#include "ci_system_info.h"
#include "ci130x_core_misc.h"
#include "codec_manager.h"
#include "codec_manage_outside_port.h"
#include "romlib_runtime.h"
#include "FreeRTOS.h"
#include "task.h"
#include "status_share.h"


/**************************************************************************
                     define
****************************************************************************/
#define READBUF_SIZE            (576U) 

#if ON_LINE_SUPPORT
#define MAX_OUTPUTBUFF_SIZE     (4608U)
#else
#define MAX_OUTPUTBUFF_SIZE     (1152U)
#endif

#define MIN_OUTPUTBUFF_SIZE     (576U)
#define PCM_BUF_COUNT           (4)
#define MIN_VALID_DATA_SIZE     (20U)
#define TIMEOUT_CNT             (40000)


enum
{
    AUDIO_PLAY_FLAG_REL_END = 0,
    AUDIO_PLAY_FLAG_WILL_END = 1,
    AUDIO_PLAY_FLAG_DATA_END = 2,
    AUDIO_PLAY_FLAG_RESTART = 0xff,
};

/**************************************************************************
                    global
****************************************************************************/
/*! 播放事件标志组句柄 */
audio_play_os_event_group_t audio_play_cmd_event_group = NULL;
/*! 播放消息队列句柄 */
audio_play_os_queue_t audio_play_queue = NULL;
/*! 播放任务句柄 */
audio_play_os_task_t audio_play_task_handle;

extern int sg_play_device_index;

/**************************************************************************
                    static
****************************************************************************/
static audio_play_state_t audio_play_state = AUDIO_PLAY_STATE_IDLE;
static void *decoder_ops = NULL;
static audio_play_decoder_ops_t *curr_decoder_ops = NULL;
static audio_play_decoder_ops_t *history_decoder_ops = NULL;
static CALLBACK_END_PLAY g_end_paly_callback = NULL;
static bool end_paly_callback_flag = false;
static int32_t play_end_cb_flag = AUDIO_PLAY_CB_STATE_UNKNOWN_ERR;
extern audio_format_info_t audio_format_info;
static uint32_t audio_decode_offset = 0;
static uint32_t audio_read_offset = 0;
static uint32_t audio_play_output_size = 0;

/*for audio play card driver*/
static uint16_t *pcm_buf_base;
static uint32_t pcm_buf_wp = 0;
static uint16_t *decbuf = NULL;
uint8_t *wav_buf = NULL;

static uint8_t *data_in_buffer;
static uint32_t play_end_flag = AUDIO_PLAY_FLAG_RESTART;
static uint32_t no_use_size;
static uint32_t temp_wave_read_size = 0;
static uint32_t valid_data = GET_PLAY_DATA_BUFF_SIZE;
static bool valid_flag = false;
static uint32_t receive_timeout_ticks = 2;


static uint16_t pcm_buf_no_cache[MAX_OUTPUTBUFF_SIZE];


#define M4A_HEAD_SIZE  (GET_PLAY_DATA_BUFF_SIZE*20)
m4a_file_info_t m4a_file_info = {0};
uint8_t *bak_data = NULL;
mem_file_t *bak_data_fp = NULL;
void *decoder_config_arg = NULL;

#if AUDIO_PLAY_USE_SPEEDING_SPEECH
#include "sonic.h"
static sonicStream sonic_stream = NULL;
#define SONIC_SPEED               (1.0f)
#define SONIC_PITCH               (1.0f)
#define SONIC_RATE                (1.0f)
#define SONIC_VOLUME              (1.0f)
#define SONIC_EMULATECHORDPITCH   (0)
#define SONIC_QUALITY             (0)
#endif

static int play_will_finish_threshold_ms = 0; //ms
static int play_will_finish_threshold_data_size = 0;
static int play_pcm_size = 0; //TODO:
static int play_will_finish_curr_play_size = 0;
static bool play_will_finish = false;

#if AUDIO_PLAYER_FIX_OFFSET_ISSUE
static uint8_t near_offset_data[8] = {0};
static bool near_offset_data_valid = false;
#endif

/**************************************************************************
                    function
****************************************************************************/
/**
 * @brief 初始化播放任务所需资源
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
static int32_t task_audio_play_init_step(void)
{
    audio_play_decoder_ops_t *ops = (audio_play_decoder_ops_t *)decoder_ops;

    /* 初始化解码器 */
	#if !AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
    for(;ops != NULL;ops = ops->next_ops)
    {
        if(ops->init != NULL)
        {
            ops->init();
        }
    }
	#endif

    #if AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
    if(RETURN_OK != alloc_m4a_info(&m4a_file_info))
    {
        ci_logerr(LOG_AUDIO_PLAY,"malloc m4a_info error\n");
        return RETURN_ERR;
    }
    #endif
    
    #if AUDIO_PLAY_USE_SPEEDING_SPEECH
    /* 音频后处理器初始化 */
    /* 创建sonic_stream处理流并配置默认参数 */
    sonic_stream = sonicCreateStream(16000, 1);
    if(NULL == sonic_stream)
    {
        ci_logerr(LOG_AUDIO_PLAY,"malloc sonic_stream error\n");
        return RETURN_ERR;
    }
    sonicSetSpeed(sonic_stream, SONIC_SPEED);
    sonicSetPitch(sonic_stream, SONIC_PITCH);
    sonicSetRate(sonic_stream, SONIC_RATE);
    sonicSetVolume(sonic_stream, SONIC_VOLUME);
    sonicSetChordPitch(sonic_stream, SONIC_EMULATECHORDPITCH);
    sonicSetQuality(sonic_stream, SONIC_QUALITY);   
    #endif

    /*for source data used,include mp3 or adpcm raw data*/
    data_in_buffer = (uint8_t *)audio_play_malloc(sizeof(uint8_t)*READBUF_SIZE);
    if(NULL == data_in_buffer)
    {
        ci_logerr(LOG_AUDIO_PLAY,"malloc audio source data error\n");
        return RETURN_ERR;
    }

    /*decode output pcm data,if format is mp3:4 buffer for max,will delay 26*4 ms*/
// #if (CONFIG_DIRVER_BUF_USED_FREEHEAP_EN)
//     pcm_buf_base = (uint16_t*)audio_play_malloc(sizeof(uint16_t)*MIN_OUTPUTBUFF_SIZE*PCM_BUF_COUNT);
//     if(NULL == pcm_buf_base)
//     {
//         ci_logerr(LOG_AUDIO_PLAY,"alloc pcm buf error!\n");
//         return RETURN_ERR;
//     }
//     MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(pcm_buf_base,0,sizeof(uint16_t)*MIN_OUTPUTBUFF_SIZE*PCM_BUF_COUNT);
// #else
//     pcm_buf_base = pcm_buf_no_cache;
// #endif

    pcm_buf_wp = 0;
    ci_logdebug(LOG_AUDIO_PLAY,"pcm buffer base addr is 0x%p\n",pcm_buf_base);

    audio_play_hw_per_init();

    return RETURN_OK;
}


/**
 * @brief 获取最小单位的下一个pcm buff地址
 * 
 * @return uint16_t* pcm buff地址
 */
static uint16_t* get_pcm_buff_addr(void)
{
    pcm_buf_wp++;
    if(pcm_buf_wp >= PCM_BUF_COUNT)
    {
        pcm_buf_wp = 0;
    }
    return pcm_buf_base+MIN_OUTPUTBUFF_SIZE*pcm_buf_wp;
}


/**
 * @brief 获取PCM下一个缓冲区地址
 * 
 * @param num 获取buff个数
 * @return uint16_t* pcm buff地址
 * 
 * @note 在连续的数据流中num不应该改变，除非重新调用para_recover_default初始化参数
 */
static uint16_t* get_next_pcm_buff(uint32_t num)
{
    uint16_t * next_pcm_buff = NULL;

    CI_ASSERT(num > 0,"arg err!\n");
    
    while(num--)
    {
        next_pcm_buff = get_pcm_buff_addr();
    }
    
    return next_pcm_buff;
}


/**
 * @brief 检索解码器
 * 
 * @param decoder_name 解码器名称
 * @retval RETURN_OK 检索成功
 * @retval RETURN_ERR 检索失败
 */
static int32_t find_decoder_ops(const char* decoder_name)
{
    audio_play_decoder_ops_t *ops = (audio_play_decoder_ops_t *)decoder_ops;

    if(curr_decoder_ops != NULL)
    {
        if(0 ==  MASK_ROM_LIB_FUNC->newlibcfunc.strcmp_p(curr_decoder_ops->decoder_name, decoder_name))
        {
            /* 配置解码器模式 */
            ci_logdebug(LOG_AUDIO_PLAY,"found decoder!\n");
            #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
            if(curr_decoder_ops->init != NULL)
            {
                int ret = curr_decoder_ops->init();
                if(ret != RETURN_OK)
                {
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_DECODER_MEM_ERR;
                    return RETURN_ERR;
                }
            }
            #endif
            /* 解码器清理 */
            if(curr_decoder_ops->clear != NULL)
            {
                curr_decoder_ops->clear();
            }
            if(curr_decoder_ops->config != NULL  && decoder_config_arg != NULL)
            {
                curr_decoder_ops->config(decoder_config_arg);
            }
            return RETURN_OK;
        }
    }

    if((0 ==  MASK_ROM_LIB_FUNC->newlibcfunc.strcmp_p("history", decoder_name))&&(history_decoder_ops != NULL))
    {
        if(history_decoder_ops == NULL)
        {
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
            return RETURN_ERR;
        }

        curr_decoder_ops = history_decoder_ops;
        history_decoder_ops = NULL;
        #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
        if(curr_decoder_ops->init != NULL)
        {
            int ret = curr_decoder_ops->init();
            if(ret != RETURN_OK)
            {
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_DECODER_MEM_ERR;
                return RETURN_ERR;
            }
        }
        #endif
        /* 解码器清理 */
        if(curr_decoder_ops->clear != NULL)
        {
            curr_decoder_ops->clear();
        }
        /* 配置解码器模式 */
        if(curr_decoder_ops->load != NULL)
        {
            curr_decoder_ops->load();
        }
        ci_logdebug(LOG_AUDIO_PLAY,"load decoder ops is 0x%x\n",curr_decoder_ops);

        return RETURN_OK;
    }
    
    for(;ops != NULL;ops = ops->next_ops)
    {
        if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strcmp_p(ops->decoder_name, decoder_name))
        {
            curr_decoder_ops = ops;

            ci_logdebug(LOG_AUDIO_PLAY,"found decoder!\n");
            #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
            if(curr_decoder_ops->init != NULL)
            {
                int ret = curr_decoder_ops->init();
                if(ret != RETURN_OK)
                {
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_DECODER_MEM_ERR;
                    return RETURN_ERR;
                }
            }
            #endif
            /* 解码器清理 */
            if(curr_decoder_ops->clear != NULL)
            {
                curr_decoder_ops->clear();
            }
            /* 配置解码器模式 */
            if(curr_decoder_ops->config != NULL && decoder_config_arg != NULL)
            {
                curr_decoder_ops->config(decoder_config_arg);
            }
            return RETURN_OK;
        }
    }

    play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
    return RETURN_ERR;
}

/**
 * @brief 发送静音数据
 * 
 */
// void send_a_zero_buffer(void)
// {  
//     MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(decbuf,0,MIN_OUTPUTBUFF_SIZE*sizeof(uint16_t));
//     ci_logdebug(LOG_AUDIO_PLAY,"dec addr 0x%p\n",decbuf);

//     audio_format_info.out_min_size = 1152;
//     // audio_play_hw_write_data(decbuf, 2304*sizeof(short));
//     cm_write_codec(sg_play_device_index, decbuf,portMAX_DELAY);   //TODO HSL

//     // decbuf = get_next_pcm_buff(4);
//     decbuf = cm_get_pcm_buffer(sg_play_device_index,portMAX_DELAY);    //TODO HSL
// }


/**
 * @brief 设置播放器默认参数
 * 
 */
static void para_recover_default(void)
{
    receive_timeout_ticks = 2;//数据超时时间减短，加快启动播放速度
    audio_read_offset = 0;
    audio_decode_offset = 0;
    valid_data = GET_PLAY_DATA_BUFF_SIZE;
    pcm_buf_wp = 0;
    decbuf = pcm_buf_no_cache;
    // uint32_t ret_p = 0;
    // cm_get_pcm_buffer(sg_play_device_index,&ret_p,portMAX_DELAY);
    // decbuf = (void*)ret_p;
    // if(0 == decbuf)
    // {
    //     CI_ASSERT(0,"\n");
    // }
    decoder_config_arg = NULL;
    // MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(pcm_buf_base,0,sizeof(uint16_t)*MIN_OUTPUTBUFF_SIZE*PCM_BUF_COUNT);
    play_will_finish_threshold_data_size = 0;
    play_will_finish = false;
    play_will_finish_curr_play_size = 0;
    play_pcm_size = 0;
}


/**
 * @brief 检查是否有播放暂停/停止事件
 * 
 * @return play state
 */
static int32_t cheak_play_event(void)
{
    uint32_t uxBits;
    uxBits = audio_play_event_group_wait_bits(audio_play_cmd_event_group, AUDIO_PLAY_EVENT_PAUSE|AUDIO_PLAY_EVENT_STOP, AUDIO_PLAY_OS_SUCCESS, AUDIO_PLAY_OS_FAIL, audio_play_os_unblock); 
    if(0 != (uxBits&AUDIO_PLAY_EVENT_STOP)) //TODO: 优先响应stop事件,为什么???
    {
        return AUDIO_PLAY_CB_STATE_DONE;
    }
    else if(0 != (uxBits&AUDIO_PLAY_EVENT_PAUSE))
    {
        return AUDIO_PLAY_CB_STATE_PAUSE;
    }
    else
    {
        return AUDIO_PLAY_CB_STATE_UNKNOWN_ERR;
    }
}


/**
 * @brief 释放备份数据区
 * 
 */
static void free_bak_data(void)
{
    if((bak_data != NULL)||(bak_data_fp != NULL))
    {
        #if USE_AAC_DECODER
        mf_close(bak_data_fp);
        #endif
        audio_play_free(bak_data);
        bak_data_fp = NULL;
        bak_data = NULL;
    }
}


#if AUDIO_PLAY_USE_MIX_2_CHANS
#include <math.h>
/**
 * @brief 双声道混音到右声道
 * 
 * @param pcmbuf 原始数据
 * @param output_size 数据大小
 */
static void mix2chans(short *pcmbuf,uint32_t output_size)
{
    int16_t *src = (int16_t *)pcmbuf;
    int32_t value;

    for(int i = 0;i < output_size; i += 4)
    {
        #if 0 // 方法一
        /* 计算混音结果 */
        int src1 = (int)(*(src));
        int src2 = (int)(*(src+1));
        if ((src1 < 0) && (src2 < 0)) 
        {
            value = src1 + src2 +  (src1 * src2 / ((int)pow(2,15)-1));
        }
        else
        {
            value = src1 + src2 -  (src1 * src2 / ((int)pow(2,15)-1));
        }
        /* 混音结果存储在右声道 */
        *(src+1) = (int16_t)__SSAT(value,16);
        #endif
        
        #if 1 // 方法二
        /* 计算混音结果 */
        value = (int)(*(src)) + (int)(*(src+1));
        /* 混音结果存储在右声道 */
        *(src+1) = (int16_t)(value/2); 
        #endif
        
        #if 0 // 方法三
        /* 计算混音结果 */
        float src1 = (float)(*(src))/32767.0f;
        float src2 = (float)(*(src+1))/32767.0f;
        float sum;
        if ((src1 < 0) && (src2 < 0)) 
        {
            sum = src1 + src2 +  (src1 * src2);
        }
        else
        {
            sum = src1 + src2 -  (src1 * src2);
        }
        value = (int)(sum*32767.0f);
        /* 混音结果存储在右声道 */
        *(src+1) = (int16_t)value;
        #endif
        
        /* 不用管左声道 */
        //*(src) = 0;
        
        src++;
        src++;
    }
}
#endif


/**
 * @brief 数据处理调用解码器解码递交声卡驱动
 * 
 * @retval RETURN_OK 解码正常
 * @retval RETURN_ERR 解码失败
 */
// #pragma GCC optimize("O0")
static int32_t audio_play_decode_continue(void)
{
    int32_t get_frame_flag_timeout = TIMEOUT_CNT;
    uint32_t data_in_size = curr_decoder_ops->data_in_size;
    uint32_t wav_buf_addr;
    uint32_t rev_size;
    uint32_t rev_flag;
    static uint32_t decode_frame_count = 0;
    int skip_frame_count = 2;
    audio_format_info_t audio_format_info;
    
    if (audio_read_offset == 0)
    {
        decode_frame_count = 0;
    }

    while(--get_frame_flag_timeout > 0)
    {
        /* 若队列不满则存在空buf可以解码，否则返回 */
        if (decode_frame_count > skip_frame_count)
        {
            if(RETURN_OK == audio_play_hw_queue_is_full())
            {
                break;
            }
        }

        /* 有效数据配置，adpcm解码器需要，不影响mp3解码器 */
        ci_logverbose(LOG_AUDIO_PLAY,"valid_data is %d\n",valid_data);
        if(valid_data < data_in_size)
        {
            valid_flag = true;
            no_use_size = valid_data;
        }

        /* 调用解码器解码 */
        if(0 ==  curr_decoder_ops->decode(data_in_buffer,data_in_size,(short*)decbuf,&audio_play_output_size,&no_use_size,&audio_format_info))
        {
            decode_frame_count++;
            #if AUDIO_PLAY_USE_MIX_2_CHANS
            /* 如果音频为双声道，在此对数据进行混音 */
            if(audio_format_info.nChans == 2)
            {
                // mix2chans((short*)decbuf,audio_play_output_size);
            }
            #endif

            #if AUDIO_PLAYER_FIX_OFFSET_ISSUE
            /* 这里用来标记保存最近一次打断的数据值，用来给外部继续播放数据灌入做校验，如果数据源稳定则不需要打开该宏 */
            if(no_use_size  >= 8)
            {
                /* 存在有效的数据，保存数据 */
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(near_offset_data,data_in_buffer+data_in_size-no_use_size,8);
                near_offset_data_valid = true;
            }
            else
            {
                /* 不存在有效的数据 */
                near_offset_data_valid = false;
            }
            #endif            
            
            #if AUDIO_PLAY_USE_SPEEDING_SPEECH
            /* TTS音频变速功能如果打开，在这里进行输出音频后处理，目前采用sonic库，
             * 这种后处理会导致数据量增加或减小，处理流程如下
             */
            int samplesWritten = audio_play_output_size/2;  //单位为short型

            /* 根据采样率和通道数重配sonic_stream，重配会丢弃数据流里的历史数据，同一音频仅重配一次 */
            if(audio_format_info.samprate != sonicGetSampleRate(sonic_stream))
            {
                if(1 != sonicSetSampleRate(sonic_stream,audio_format_info.samprate))
                {
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                    ci_logerr(LOG_AUDIO_PLAY,"sonicSetSampleRate mem alloc err!\n");
                    return RETURN_ERR;
                }
            }
            if(audio_format_info.nChans != sonicGetNumChannels(sonic_stream))
            {
                if(1 != sonicSetNumChannels(sonic_stream,audio_format_info.nChans))
                {
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                    ci_logerr(LOG_AUDIO_PLAY,"sonicSetNumChannels mem alloc err!\n");
                    return RETURN_ERR;
                }
            }

            /* 写入音频到数据sonic_stream */
            if(1 != sonicWriteShortToStream(sonic_stream, (short *)decbuf, samplesWritten/audio_format_info.nChans))
            {
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                ci_logerr(LOG_AUDIO_PLAY,"sonicWriteShortToStream err!\n");
                return RETURN_ERR;
            }

            /* 检查sonic_stream中的音频数量，将音频数据写入播放设备驱动，直到数据不足一包 */
            while(sonicSamplesAvailable(sonic_stream) >= samplesWritten/audio_format_info.nChans)
            {
                if(audio_play_state == AUDIO_PLAY_STATE_PLAYING)
                {
                    /* 当前在播放状态，则一直等待硬件播放有可用的buf，将sonic_stream可用数据全部处理 */
                    while(RETURN_OK == audio_play_hw_queue_is_full())
                    {
                        /* 这里检查外部暂停/停止事件*/
                        /* FIXME:注意这里跳出播放，播放同步字稍微落后播放，对于下次继续播放可能重复播放,但影响不大 */
                        int32_t play_event_state = cheak_play_event();
                        if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != play_event_state)
                        {
                            play_end_cb_flag = play_event_state;
                            return RETURN_ERR;
                        }
                        audio_play_task_delay(pdMS_TO_TICKS(20));
                    }
                }
                else
                {
                    /* 如果在预解码阶段，硬件无可用buf则直接跳出，后续的数据暂留于sonic_stream，待开启播放后处理 */
                    /* TODO:这里检查下是否出现播放暂停后同步字进度稍微超前播放 */
                    if(RETURN_OK == audio_play_hw_queue_is_full())
                    {
                        break;
                    }
                }

                /* 读出数据，写入播放设备硬件，更新解码buf */
                sonicReadShortFromStream(sonic_stream, (short *)decbuf,samplesWritten/audio_format_info.nChans);
                // audio_play_hw_write_data(decbuf, audio_play_output_size); 
                cm_write_codec(sg_play_device_index, decbuf,portMAX_DELAY);
                // decbuf = get_next_pcm_buff((audio_play_output_size/2/MIN_OUTPUTBUFF_SIZE) + (audio_play_output_size%(2*MIN_OUTPUTBUFF_SIZE)?1:0));
                uint32_t ret_p;
                // cm_get_pcm_buffer(sg_play_device_index,decbuf,portMAX_DELAY);    //TODO HSL
                cm_get_pcm_buffer(sg_play_device_index,&ret_p,portMAX_DELAY);    //TODO HSL
                decbuf = (void*)ret_p;
                play_will_finish_curr_play_size += audio_play_output_size;
            }
            #else
            /* 不做后处理则可以直接写入硬件播放设备 */
            // TODO:多余的解码音频就丢弃了，不播放直接bypass，这个功能和带后处理的情景冲突不兼容这个功能
            (void)play_pcm_size;
            //if((play_pcm_size!=0)&&(play_will_finish_curr_play_size < play_pcm_size+1152 ))             
            if (decode_frame_count > skip_frame_count)
            {
                if (decode_frame_count == skip_frame_count+1)
                {
                    cm_pcm_buffer_info_t pcm_buffer_info;
                    pcm_buffer_info.play_buffer_info.block_num = 2;
                    pcm_buffer_info.play_buffer_info.buffer_num = PCM_BUF_COUNT;
                    pcm_buffer_info.play_buffer_info.block_size = audio_play_output_size/2;
                    pcm_buffer_info.play_buffer_info.buffer_size = pcm_buffer_info.play_buffer_info.block_size * pcm_buffer_info.play_buffer_info.block_num;
                    
                    static int last_pcm_buffer_total_size = 0;
                    static void* last_pcm_buffer = NULL;
                    int pcm_buffer_total_size = pcm_buffer_info.play_buffer_info.buffer_size*pcm_buffer_info.play_buffer_info.buffer_num;
                    if(last_pcm_buffer_total_size < pcm_buffer_total_size)
                    {
                        if (last_pcm_buffer != NULL)
                        {
                            vPortFree(last_pcm_buffer);
                            last_pcm_buffer = NULL;
                        }
                        last_pcm_buffer_total_size = pcm_buffer_total_size;
                    }
                    
                    if (last_pcm_buffer == NULL)
                    {
                        last_pcm_buffer = pvPortMalloc(pcm_buffer_total_size);
                        CI_ASSERT(last_pcm_buffer,"\n");
                    }
                    pcm_buffer_info.play_buffer_info.pcm_buffer = last_pcm_buffer;
                    cm_config_pcm_buffer(sg_play_device_index, CODEC_OUTPUT, &pcm_buffer_info);

                    uint32_t ret_buf;
                    cm_get_pcm_buffer(sg_play_device_index,&ret_buf,portMAX_DELAY);
                    memcpy(ret_buf, decbuf, audio_play_output_size);
                    cm_write_codec(sg_play_device_index, ret_buf,portMAX_DELAY);
                    audio_play_hw_start(DISABLE, &audio_format_info);    //TODO HSL 参数需要传进来
                }
                else
                {
                    cm_write_codec(sg_play_device_index, decbuf,portMAX_DELAY);
                }
                // decbuf = get_next_pcm_buff((audio_play_output_size/2/MIN_OUTPUTBUFF_SIZE) + (audio_play_output_size%(2*MIN_OUTPUTBUFF_SIZE)?1:0));
                uint32_t ret_p = 0;
                cm_get_pcm_buffer(sg_play_device_index,&ret_p,portMAX_DELAY);
                decbuf = (void*)ret_p;
            }
            
            play_will_finish_curr_play_size += audio_play_output_size;
            #endif
            ci_logverbose(LOG_AUDIO_PLAY,"play size %d %d\n",play_will_finish_curr_play_size,audio_play_output_size);
            if((play_will_finish_threshold_data_size!=0)&&(play_will_finish_curr_play_size >= play_will_finish_threshold_data_size )&& (!play_will_finish))
            {
                if(g_end_paly_callback != NULL)
                {
                    ci_loginfo(LOG_AUDIO_PLAY,"AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD 1\n");
                    g_end_paly_callback(AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD);
                }
                play_will_finish = true;
            }
            audio_decode_offset = audio_read_offset + data_in_size - no_use_size;
            get_frame_flag_timeout = TIMEOUT_CNT;
        }
        else
        {
            get_frame_flag_timeout = 0;
        }
        
        
        if(valid_data > data_in_size - no_use_size )
        {
            valid_data = valid_data - data_in_size + no_use_size;
        }
        else
        {
            valid_data = no_use_size;
        }
          
        /* 更新数据读入记录 */
        audio_read_offset += data_in_size - no_use_size;

        if(valid_flag&&(valid_data <= MIN_VALID_DATA_SIZE))
        {
            #if AUDIO_PLAY_USE_SPEEDING_SPEECH
            //TODO: 这里播放结束，如果仍处于预解码状态，sonic_stream中仍有数据，可能会造成数据丢失，尾音丢音
            sonicFlushStream(sonic_stream);
            while((0 != sonicSamplesAvailable(sonic_stream))&&(audio_play_state == AUDIO_PLAY_STATE_PLAYING))
            {
                while(RETURN_OK == audio_play_hw_queue_is_full())
                {
                    /* 这里检查外部暂停/停止事件*/
                    int32_t play_event_state = cheak_play_event();
                    if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != play_event_state)
                    {
                        play_end_cb_flag = play_event_state;
                        return RETURN_ERR;
                    }
                }
                int samplesWritten = audio_play_output_size/2;  //单位为short型
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(decbuf,0x0,audio_play_output_size);
                sonicReadShortFromStream(sonic_stream, (short *)decbuf,samplesWritten/audio_format_info.nChans);
                // audio_play_hw_write_data(decbuf, audio_play_output_size);
                // cm_write_codec(sg_play_device_index, decbuf,portMAX_DELAY);  //TODO HSL
                cm_write_codec(sg_play_device_index, decbuf,portMAX_DELAY);
                // decbuf = get_next_pcm_buff((audio_play_output_size/2/MIN_OUTPUTBUFF_SIZE) + (audio_play_output_size%(2*MIN_OUTPUTBUFF_SIZE)?1:0));
                uint32_t ret_p = 0;
                cm_get_pcm_buffer(sg_play_device_index,&ret_p,portMAX_DELAY); //TODO HSL
                decbuf = (void*)ret_p;
            }
            #endif

            if(AUDIO_PLAY_FLAG_DATA_END == play_end_flag)
            {
                finsh_read_data();
            }
            free_bak_data();
            ci_logdebug(LOG_AUDIO_PLAY,"valid_data %d\n",valid_data);
            play_end_flag = AUDIO_PLAY_FLAG_REL_END;
            get_frame_flag_timeout = TIMEOUT_CNT;
            break;
        }
        else
        {
            valid_flag = false;
        }
        
        /* FIXME:下面这段数据补充的代码，采用了拷贝的方式来针对变长帧处理，如果采用数据输入buf首端增加一个空白包，
         *       应该能完成未使用数据移动到上一buff头部以降低数据拷贝量(仅在循环尾端拷贝缓冲buf较多时更明显)，
         *       示例图如下：
         *          这里是空白buf      未使用数据               总共3个buf尾(不包括空白buf)
         *         |     ↓     |         |↓|                        ↓
         *         |____________|____________|____________|____________|
         *                      ↑        ↑           ↑
         *                  第一次buf头  下一次头    下一次尾
         *        这种方式空间换时间，并且对于交互get_play_data来说buf管理复杂度增加，如有必要可以考虑优化
         */
        /* 待解码buf数据补充 */
        if(no_use_size < data_in_size)
        {
            /* 拷贝剩余数据到buf起始位置 */
            MASK_ROM_LIB_FUNC->newlibcfunc.memmove_p(data_in_buffer,data_in_buffer+data_in_size-no_use_size,no_use_size);

            #if USE_AAC_DECODER
            /* 先检查bak_data_fp数据 */
            if(bak_data != NULL)
            {
                /* 读取bak_data_fp数据给data_in_buffer */
                int br =  mf_read(data_in_buffer+no_use_size,1,data_in_size-no_use_size,bak_data_fp);
                valid_data += br;
                if(data_in_size-no_use_size > br)
                {
                    free_bak_data();
                }
                /* 将读进来的数据视为未使用的数据，增加no_use_size大小 */
                no_use_size += br;
            }
            #endif
        }

        if(no_use_size < data_in_size)
        {
            /* 从循环缓冲buf读取数据 */
            if(GET_PLAY_DATA_BUFF_SIZE-temp_wave_read_size >= data_in_size-no_use_size)
            {
                /* 当前循环缓冲buf数据补充至待解码buf，达到data_in_size大小 */
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(data_in_buffer+no_use_size,(uint8_t *)(wav_buf+temp_wave_read_size),data_in_size-no_use_size);
            }
            else
            {
                /* 当前循环缓冲buf数据不足补充到data_in_size大小 */
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(data_in_buffer+no_use_size,(uint8_t *)(wav_buf+temp_wave_read_size),GET_PLAY_DATA_BUFF_SIZE-temp_wave_read_size);
                no_use_size += GET_PLAY_DATA_BUFF_SIZE-temp_wave_read_size;
                temp_wave_read_size = 0;

                if(AUDIO_PLAY_FLAG_WILL_END == play_end_flag)
                {
                    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_in_buffer+no_use_size,0x0,data_in_size-no_use_size);
                    ci_logdebug(LOG_AUDIO_PLAY,"play end flag! no_use_size is %d\n",no_use_size);
                    no_use_size = 0;
                }
                else
                {
                    /* 释放数据包 */
                    finsh_read_data();

                    /* 若最后一个数据包释放完毕 */
                    if(AUDIO_PLAY_FLAG_DATA_END == play_end_flag)
                    {
                        /* 准备结束播放 */
                        play_end_flag = AUDIO_PLAY_FLAG_WILL_END;
                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_in_buffer+no_use_size,0x0,data_in_size-no_use_size);
                        no_use_size = 0;
                    }
                    else
                    {
                        /* 获取新数据包 */
                        while(1)
                        {
                            if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
                            {
                                int32_t play_event_state = cheak_play_event();
                                if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != play_event_state)
                                {
                                    /* 数据超时了，外面请求暂停了 */
                                    ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout! receive pause/stop event!\n");
                                    play_end_cb_flag = play_event_state;
                                    return RETURN_ERR;
                                }
                                /* 否则重新接收 */
                                audio_play_task_delay(receive_timeout_ticks);
                            }
                            else
                            {
                                /* 成功接收 */
                                break;
                            }
                        }

                        if(rev_flag == AUDIO_DATD_BUFF_ERR)
                        {
                            /* 数据出错了 */
                            ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
                            play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                            return RETURN_ERR;
                        }

                        wav_buf = (uint8_t *)wav_buf_addr;

                        /* 如果收到的数据包是最后一包设置标志，在数据包释放时准备结束播放 */
                        if(rev_flag == AUDIO_DATD_BUFF_END)
                        {
                            ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
                            play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
                            valid_data += rev_size;
                        }
                        else
                        {
                            valid_data += GET_PLAY_DATA_BUFF_SIZE;
                        }
                        
                        /* 补充数据到data_in_size大小 */
                        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(data_in_buffer+no_use_size,(uint8_t *)(wav_buf+temp_wave_read_size),data_in_size-no_use_size);
                    }
                }
            }

            /* 数据补充完成，供解码器使用标志更新，准备下一帧解码 */
            temp_wave_read_size += data_in_size-no_use_size;
            no_use_size=data_in_size;
        }
    }

    /* 超时返回错误 */
    if(get_frame_flag_timeout > 0)
    {
        return RETURN_OK;
    }
    else
    {
        ci_logerr(LOG_AUDIO_PLAY,"decoder timeout or audio flie has err !!\n");
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
        return RETURN_ERR;
    }
}


/**
 * @brief 播放事件处理
 * 
 * @param uxBits 事件bit
 */
static void audio_play_playing_deal(uint32_t uxBits)
{
    if(0 != (uxBits&(AUDIO_PLAY_EVENT_STOP|AUDIO_PLAY_EVENT_PLAY_TIMEOUT|AUDIO_PLAY_EVENT_PAUSE)))
    {
        if(AUDIO_PLAY_FLAG_REL_END == play_end_flag)
        {
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_DONE;
        }
        else
        {
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_PAUSE;
            if(0 != (uxBits&AUDIO_PLAY_EVENT_PLAY_TIMEOUT))
            {
                ci_logerr(LOG_AUDIO_PLAY,"play timeout!\n");
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
            }
        }

        ci_logdebug(LOG_AUDIO_PLAY,"play_end_flag is %d\n",play_end_flag);
        pause_read_data();

        /* 请求暂停数据接收，保存状态 */
        if(0 != (uxBits&AUDIO_PLAY_EVENT_PAUSE))
        {
            /* 解码器现场保存 */
            if(curr_decoder_ops->save != NULL)
            {
                curr_decoder_ops->save();
                curr_decoder_ops->clear();
            }
            history_decoder_ops = curr_decoder_ops;
            ci_logdebug(LOG_AUDIO_PLAY,"save decoder ops is 0x%x\n",curr_decoder_ops);
        }

        free_bak_data();

        audio_play_state = AUDIO_PLAY_STATE_STOP;
        ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
        return;
    }
    if(0 != (uxBits&AUDIO_PLAY_EVENT_ONE_BUF_DONE))
    {
        if(AUDIO_PLAY_FLAG_REL_END == play_end_flag)
        {
            ci_logdebug(LOG_AUDIO_PLAY,"audio play done\n");
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_DONE;
            audio_play_state = AUDIO_PLAY_STATE_STOP; 
            ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
            return;
        }

        /* 
         * 1.decode to pcm
         * 2.send play buffer
         * 3.get next input buffer
         */
        if(RETURN_ERR == audio_play_decode_continue())
        {
            /* 请求暂停数据接收，保存状态 */
            if(play_end_cb_flag == AUDIO_PLAY_CB_STATE_PAUSE)
            {
                /* 解码器现场保存 */
                if(curr_decoder_ops->save != NULL)
                {
                    curr_decoder_ops->save();
                    curr_decoder_ops->clear();
                }
                history_decoder_ops = curr_decoder_ops;
                ci_logdebug(LOG_AUDIO_PLAY,"save decoder ops is 0x%x\n",curr_decoder_ops);
            }

            pause_read_data();
            
            free_bak_data();
            
            audio_play_state = AUDIO_PLAY_STATE_STOP; 
            ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
        }
    }
}


/**
 * @brief 启动播放
 * 
 * @retval RETURN_OK 解码正常
 * @retval RETURN_ERR 解码失败
 */
static int32_t audio_play_start(void)
{
    uint32_t data_in_size = curr_decoder_ops->data_in_size;
    uint32_t wav_buf_addr;
    uint32_t rev_size;
    uint32_t rev_flag;
    uint32_t start_offset = 0;

    valid_data = 0;
    valid_flag = false;
    temp_wave_read_size = 0;

    #if USE_AAC_DECODER
    if(bak_data != NULL)
    {
        int br =  mf_read(data_in_buffer,1,data_in_size,bak_data_fp);
        start_offset = br;
        valid_data += br;
        if(data_in_size > br)
        {
            free_bak_data();
        }
    }
    #endif
    
    if(play_end_flag == AUDIO_PLAY_FLAG_RESTART)
    {
        while(1)
        {
            if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
            {
                int32_t play_event_state = cheak_play_event();
                if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != play_event_state)
                {
                    /* 数据超时了，外面请求暂停了 */
                    ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout! receive pause/stop event!\n");
                    play_end_cb_flag = play_event_state;
                    return RETURN_ERR;
                }
                /* 否则重新接收 */
                audio_play_task_delay(receive_timeout_ticks);
            }
            else
            {
                /* 成功接收 */
                break;
            }
        }
        data_in_size = curr_decoder_ops->data_in_size;
        
        if(rev_flag == AUDIO_DATD_BUFF_ERR)
        {
            /* 数据出错了 */
            ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
            return RETURN_ERR;
        }
        wav_buf = (uint8_t *)wav_buf_addr;
        if(rev_flag == AUDIO_DATD_BUFF_END)
        {
            ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
            play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
            valid_data += rev_size;
        }
        else
        {
            valid_data += GET_PLAY_DATA_BUFF_SIZE;
        }
        if(start_offset < data_in_size)
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(data_in_buffer+start_offset,(uint8_t *)(wav_buf+temp_wave_read_size),data_in_size-start_offset);
            temp_wave_read_size += data_in_size-start_offset;
        }
    }
    else
    {
        temp_wave_read_size = GET_PLAY_DATA_BUFF_SIZE;
    }
    
    no_use_size = data_in_size;
    // decbuf = NULL;

    /* 
     * 1.decode to pcm
     * 2.send play buffer
     * 3.get next input buffer
     */
    return audio_play_decode_continue();
}


/**
 * @brief 向数据获取task发送请求数据消息
 * 
 * @param play_msg 播放请求消息
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t send_get_data_msg(audio_play_msg_t *play_msg)
{
    int32_t ret = RETURN_ERR;
    
        switch(play_msg->play_data_type)
        {
            case AUDIO_PLAY_DATA_TYPE_FLASH:
            {
                ci_logdebug(LOG_AUDIO_PLAY,"VoiceAddr is 0x%x\n", play_msg->flash_data.arg);
                ret = get_flash_data(play_msg->flash_data.arg, play_msg->flash_data.data_addr_num);
                break;
            }
        #if AUDIO_PLAY_USE_SD_CARD
            case AUDIO_PLAY_DATA_TYPE_SD:
            {
                ret = get_sd_data(play_msg->sd_data.dir,play_msg->sd_data.name,play_msg->offset);
                break;
            }
        #endif
        #if AUDIO_PLAY_USE_NET
            case AUDIO_PLAY_DATA_TYPE_NET:
            {
                ret = get_net_data(play_msg->net_data.url,play_msg->offset);
                break;
            }
        #endif /*AUDIO_PLAY_USE_NET*/
        #if AUDIO_PLAY_USE_OUTSIDE
            case AUDIO_PLAY_DATA_TYPE_OUTSIDE:
            {
                ret = get_outside_data(play_msg->offset);
                break;
            }
        #endif /* AUDIO_PLAY_USE_OUTSIDE */
        #if AUDIO_PLAY_USE_OUTSIDE_V2
            case AUDIO_PLAY_DATA_TYPE_OUTSIDE_V2:
            {
                ret = get_outside_v2_data(play_msg->offset);
                break;
            }
        #endif /* AUDIO_PLAY_USE_OUTSIDE */
            case AUDIO_PLAY_DATA_TYPE_CONTINUE:
            {
                /* 请求获取上一次播放暂停时的数据 */
                ret = continue_read_data();
                break;
            }
            default:
            {
                CI_ASSERT(0,"do not support msg!\n");
            }
        }
    
    return ret;
}

#if AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
/**
 * @brief 预处理处理m4a音频
 * 
 * @param bak_data_size
 * @retval RETURN_OK 解析正常
 * @retval RETURN_ERR 解析失败
 */
static int32_t parse_m4a(uint32_t bak_data_size)
{
    uint32_t wav_buf_addr;
    uint32_t rev_size;
    uint32_t rev_flag;
    uint32_t m4a_size = bak_data_size;
    uint32_t ftpy_size = (uint32_t)__REV(*(uint32_t *)(bak_data));
    uint32_t moov_size = (uint32_t)__REV(*(uint32_t *)(bak_data+ftpy_size))+ftpy_size;
    
    if(0 != MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)(bak_data+ftpy_size+4),"moov",4))
    {
        moov_size = GET_PLAY_DATA_BUFF_SIZE;
        ci_logwarn(LOG_AUDIO_PLAY,"not found moov size, try alloc %d to parse m4a\n",moov_size);
    }
    else
    {
        moov_size = (moov_size/4096+1)*4096;
        ci_loginfo(LOG_AUDIO_PLAY,"moov size alloc %d\n",moov_size);
    }
    
    PRINT_FMEM_STATUS();
    
    /* 重新申请解析文件头空间到m4a需要的大小 */
    bak_data = (uint8_t *)audio_play_realloc(bak_data,moov_size*sizeof(uint8_t));
    if(bak_data == NULL)
    {
        ci_logerr(LOG_AUDIO_PLAY,"mem alloc err! moov size is %d\n",moov_size);
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_M4A_MEM_ERR;
        return RETURN_ERR;
    }
    /* 请求音频数据moov_size */
    while((m4a_size < moov_size) && (play_end_flag == AUDIO_PLAY_FLAG_RESTART))
    {
        while(1)
        {
            if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
            {
                if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != cheak_play_event())
                {
                    /* 数据超时了，外面请求暂停了 */
                    ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout!\n");
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_FILE_LEASTDATA_ERR;
                    return RETURN_ERR;
                }
                /* 否则重新接收 */
                audio_play_task_delay(receive_timeout_ticks);
            }
            else
            {
                /* 成功接收 */
                break;
            }
        }
        if(rev_flag == AUDIO_DATD_BUFF_ERR)
        {
            /* 数据出错了 */
            ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
            return RETURN_ERR;
        }
        if(rev_flag == AUDIO_DATD_BUFF_END)
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data+m4a_size,(uint8_t *)wav_buf_addr,rev_size);
            m4a_size += rev_size;
            ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
            play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
            break;
        }
        else
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data+m4a_size,(uint8_t *)wav_buf_addr,GET_PLAY_DATA_BUFF_SIZE);
            m4a_size += GET_PLAY_DATA_BUFF_SIZE; 
            finsh_read_data();
        }
    }

    /* 打开数据备份区内存文件系统 */
    bak_data_fp = mf_open(bak_data,m4a_size,NULL);

    m4a_file_info.aac_stco_struct->mdatOffset = 0;

    /* 解析atom */
    collectMediaData(bak_data_fp,&m4a_file_info, m4a_size,0);

    ci_logwarn(LOG_AUDIO_PLAY,"mdatOffset is 0x%x!\n",m4a_file_info.aac_stco_struct->mdatOffset);
    if(m4a_file_info.aac_stco_struct->mdatOffset == 0)
    {
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
        return RETURN_ERR;
    }

    /* 解析完毕配置解码器配置 */
    decoder_config_arg = (void *)(&m4a_file_info);

    if(m4a_file_info.aac_stco_struct->mdatOffset >=  m4a_size)
    {
        int mdat_size = 0;
        
        ci_loginfo(LOG_AUDIO_PLAY,"mdatOffset need more data!\n");
        PRINT_FMEM_STATUS();
        
        mf_close(bak_data_fp);
        audio_play_free(bak_data);
        uint32_t rel_size = (m4a_file_info.aac_stco_struct->mdatOffset/4096+1)*4096 - m4a_size;
        
        bak_data = (uint8_t *)audio_play_malloc(rel_size*sizeof(uint8_t));
        if(bak_data == NULL)
        {
            ci_logerr(LOG_AUDIO_PLAY,"mem alloc err! mdatOffset size is %d\n",rel_size);
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_M4A_MEM_ERR;
            return RETURN_ERR;
        }
        while((mdat_size < rel_size) && (play_end_flag == AUDIO_PLAY_FLAG_RESTART))
        {
            while(1)
            {
                if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
                {
                    if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != cheak_play_event())
                    {
                        /* 数据超时了，外面请求暂停了 */
                        ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout!\n");
                        play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_FILE_LEASTDATA_ERR;
                        return RETURN_ERR;
                    }
                    /* 否则重新接收 */
                    audio_play_task_delay(receive_timeout_ticks);
                }
                else
                {
                    /* 成功接收 */
                    break;
                }
            }
            if(rev_flag == AUDIO_DATD_BUFF_ERR)
            {
                /* 数据出错了 */
                ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                return RETURN_ERR;
            }
            if(rev_flag == AUDIO_DATD_BUFF_END)
            {
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data+mdat_size,(uint8_t *)wav_buf_addr,rev_size);
                mdat_size += rev_size;
                ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
                play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
                break;
            }
            else
            {
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data+mdat_size,(uint8_t *)wav_buf_addr,GET_PLAY_DATA_BUFF_SIZE);
                mdat_size += GET_PLAY_DATA_BUFF_SIZE; 
                finsh_read_data();
            }
        }
        /* 打开数据备份区内存文件系统 */
        bak_data_fp = mf_open(bak_data,mdat_size,NULL);
        /* 更新请求数据的偏移地址到音频地址 */
        mf_seek(bak_data_fp, m4a_file_info.aac_stco_struct->mdatOffset-m4a_size, SEEK_SET);
    }
    else
    {
        /* 更新请求数据的偏移地址到音频地址 */
        mf_seek(bak_data_fp, m4a_file_info.aac_stco_struct->mdatOffset, SEEK_SET);
    }

    audio_read_offset = m4a_file_info.aac_stco_struct->mdatOffset;
    audio_decode_offset = audio_read_offset;
    
    return RETURN_OK;
}

/**
 * @brief 预处理处理mp3音频
 * 
 * @param bak_data_size
 * @retval RETURN_OK 解析正常
 * @retval RETURN_ERR 解析失败
 */
static int32_t parse_mp3(uint32_t bak_data_size)
{
    uint32_t mp3_size = bak_data_size;
    uint32_t wav_buf_addr;
    uint32_t rev_size;
    uint32_t rev_flag;
    
    /* 带ID3V2版本的mp3跳过id3 */
    uint32_t id3_size = bak_data[6] << 21 |  bak_data[7] << 14 | bak_data[8] << 7 | bak_data[9];
    if(id3_size < bak_data_size)
    {
        bak_data_fp = mf_open(bak_data,bak_data_size,NULL);
        mf_seek(bak_data_fp,id3_size,SEEK_SET);
    }
    else
    {
        /* 请求音频数据id3_size */
        while((mp3_size < id3_size) && (play_end_flag == AUDIO_PLAY_FLAG_RESTART))
        {
            while(1)
            {
                if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
                {
                    if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != cheak_play_event())
                    {
                        /* 数据超时了，外面请求暂停了 */
                        ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout!\n");
                        play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_FILE_LEASTDATA_ERR;
                        return RETURN_ERR;
                    }
                    /* 否则重新接收 */
                    audio_play_task_delay(receive_timeout_ticks);
                }
                else
                {
                    /* 成功接收 */
                    break;
                }
            }
            if(rev_flag == AUDIO_DATD_BUFF_ERR)
            {
                /* 数据出错了 */
                ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                return RETURN_ERR;
            }
            if(rev_flag == AUDIO_DATD_BUFF_END)
            {
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data,(uint8_t *)wav_buf_addr,rev_size);
                mp3_size += rev_size;
                ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
                play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
                break;
            }
            else
            {
                MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data,(uint8_t *)wav_buf_addr,GET_PLAY_DATA_BUFF_SIZE);
                mp3_size += GET_PLAY_DATA_BUFF_SIZE; 
                finsh_read_data();
            }
        }
        
        if(id3_size < (mp3_size-rev_size))
        {
            ci_logerr(LOG_AUDIO_PLAY,"parse mp3 err!\n");
            play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
            return RETURN_ERR;
        }

        /* 打开数据备份区内存文件系统 */
        bak_data_fp = mf_open(bak_data,rev_size,NULL);
        mf_seek(bak_data_fp,rev_size+id3_size-mp3_size,SEEK_SET);
    }
    
    audio_read_offset = id3_size;
    audio_decode_offset = audio_read_offset;
    
    return RETURN_OK;
}

flac_file_info_t flac_file_info;

/**
 * @brief 预处理处理flac音频
 * 
 * @param bak_data_size
 * @retval RETURN_OK 解析正常
 * @retval RETURN_ERR 解析失败
 */
static int32_t parse_flac(uint32_t bak_data_size)
{
    uint8_t *parse_ptr = bak_data+4;
    uint8_t endofmetate = 0;
    uint32_t seek_blocklength = 4;
    
    while(!endofmetate)
    {
        uint8_t blockhead =  (*(uint8_t *)(parse_ptr));
        endofmetate = blockhead&0x80;
        uint32_t blocklength=((uint32_t)parse_ptr[1]<<16)|((uint16_t)parse_ptr[2]<<8)|(parse_ptr[3]);//得到块大小
        if((blockhead&0x7f) == 0)
        {
            uint8_t *block_raw_data = (parse_ptr+4);
            flac_file_info.min_blocksize=((uint16_t)block_raw_data[0]<<8) |block_raw_data[1];					//最小块大小
            flac_file_info.max_blocksize=((uint16_t)block_raw_data[2]<<8) |block_raw_data[3];					//最大块大小
            flac_file_info.min_framesize=((uint32_t)block_raw_data[4]<<16)|((uint16_t)block_raw_data[5]<<8)|block_raw_data[6];//最小帧大小
            flac_file_info.max_framesize=((uint32_t)block_raw_data[7]<<16)|((uint16_t)block_raw_data[8]<<8)|block_raw_data[9];//最大帧大小
            flac_file_info.samplerate=((uint32_t)block_raw_data[10]<<12)|((uint16_t)block_raw_data[11]<<4)|((block_raw_data[12]&0xf0)>>4);//采样率
            flac_file_info.channels=((block_raw_data[12]&0x0e)>>1)+1;							//音频通道数
            flac_file_info.bps=((((uint16_t)block_raw_data[12]&0x01)<<4)|((block_raw_data[13]&0xf0)>>4))+1;	//采样位数16?24?32?
            flac_file_info.totalsamples=((uint32_t)block_raw_data[14]<<24)|((uint32_t)block_raw_data[15]<<16)|((uint16_t)block_raw_data[16]<<8)|block_raw_data[17];//一个声道的总采样数
            
        }
        //跳过这帧
        parse_ptr += blocklength+4;
        seek_blocklength += blocklength+4;
        /* TODO:不考虑帧头大于4K情况 */
        CI_ASSERT(seek_blocklength < bak_data_size , "unsupport flac!\n");
    }
    
    decoder_config_arg = (void *)&flac_file_info;

    mf_seek(bak_data_fp, seek_blocklength, SEEK_SET);
    
    return RETURN_OK;
}

/**
 * @brief 音频文件头预处理
 * 
 * @param play_msg 播放请求消息 
 */
static int32_t parse_file(audio_play_msg_t *play_msg)
{
    uint32_t wav_buf_addr;
    uint32_t rev_size;
    uint32_t rev_flag;
    int data_size;

    if(RETURN_ERR == send_get_data_msg(play_msg))
    {
        ci_logerr(LOG_AUDIO_PLAY,"send get audio data msg err!\n");
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
        return RETURN_ERR;
    }

    while(1)
    {
        if(RETURN_OK != receive_data(&wav_buf_addr,&rev_size,&rev_flag))
        {
            if(AUDIO_PLAY_CB_STATE_UNKNOWN_ERR != cheak_play_event())
            {
                /* 数据超时了，外面请求暂停了 */
                ci_logwarn(LOG_AUDIO_PLAY,"receive_data timeout!\n");
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_FILE_LEASTDATA_ERR;
                return RETURN_ERR;
            }
            /* 否则重新接收 */
            audio_play_task_delay(receive_timeout_ticks);
        }
        else
        {
            /* 成功接收 */
            break;
        }
    }
    if(rev_flag == AUDIO_DATD_BUFF_ERR)
    {
        /* 数据出错了 */
        ci_logerr(LOG_AUDIO_PLAY,"receive_data err!\n");
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
        return RETURN_ERR;
    }
    
    bak_data = (uint8_t *)audio_play_malloc(GET_PLAY_DATA_BUFF_SIZE*sizeof(uint8_t)); 
    if(NULL == bak_data)
    {
        ci_logerr(LOG_AUDIO_PLAY,"mem alloc err!\n");
        PRINT_FMEM_STATUS();
        play_end_cb_flag = AUDIO_PLAY_CB_STATE_PARSE_M4A_MEM_ERR;
        return RETURN_ERR;
    }
    data_size = (rev_flag == AUDIO_DATD_BUFF_END)?rev_size:GET_PLAY_DATA_BUFF_SIZE;
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(bak_data,(uint8_t *)wav_buf_addr,data_size);
    if(rev_flag == AUDIO_DATD_BUFF_END)
    {
        ci_logdebug(LOG_AUDIO_PLAY,"this is end buf!\n");
        play_end_flag = AUDIO_PLAY_FLAG_DATA_END;
    }
    else
    {
        finsh_read_data();
    }
    
    
    if(MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)bak_data,"RIFF",4)  == 0)
    {
        /* MS Wav 文件 */
        /* 目前不需要特殊处理，跳过44字节头即可 */
        bak_data_fp = mf_open(bak_data,data_size,NULL);
        mf_seek(bak_data_fp,44,SEEK_SET);
        play_msg->decoder_name = "ms_wav";
    }
    else if (MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)(bak_data+4),"ftyp",4)  == 0)
    {
        /* m4a文件 */
        /* m4a文件需要解析atom，需要重新申请realloc内存，并mf_open设置指针到音频数据起始地址，并配置aac解码器配置 */
        if(RETURN_OK == parse_m4a(data_size))
        {
            play_msg->decoder_name = "aac";
        }
        else
        {
            /* 解析错误则配置为解码器为空，直接跳过该音频 */
            return RETURN_ERR;
        }
    }
    else if (MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)(bak_data),"ID3",3)  == 0)
    {
        /* mp3文件 */
        if(RETURN_OK == parse_mp3(data_size))
        {
            play_msg->decoder_name = "mp3";
        }
        else
        {
            /* 解析错误则配置为解码器为空，直接跳过该音频 */
            return RETURN_ERR;
        }
    }
    else if (MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)bak_data,"fLaC",4)  == 0)
    {
        /* flac文件 */
        bak_data_fp = mf_open(bak_data,data_size,NULL);
        parse_flac(data_size);
        play_msg->decoder_name = "flac";
    }
    else
    {
        /* 未知格式不做额外处理，按照指定的解码器处理 */
        bak_data_fp = mf_open(bak_data,data_size,NULL);
    }

    return RETURN_OK;
}
#endif


/**
 * @brief 返回播放器task状态
 * 
 * @return uint32_t 播放处理任务状态
 */
uint32_t get_audio_play_state(void)
{
    return audio_play_state;
}

/**
 * @brief 返回当前音频信息
 * 
 * @param format 音频信息结构
 */
void get_audio_format_info(audio_format_info_t *format)
{
    format->samprate = audio_format_info.samprate;
    format->nChans = audio_format_info.nChans;
    format->out_min_size = audio_format_info.out_min_size;
}

/**
 * @brief 快速设置end回调函数
 * 
 * @param callback 播放结束回调函数句柄
 * @retval RETURN_OK 设置成功
 * @retval RETURN_ERR 设置失败
 */
int32_t fastset_audio_end_callback(CALLBACK_END_PLAY callback)
{                
    if(end_paly_callback_flag)
    {
        g_end_paly_callback = callback;
        ci_loginfo(LOG_AUDIO_PLAY,"change g_end_paly_callback\n");
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}


/**
 * @brief 返回当前播放位置
 * 
 * @return uint32_t 当前播放进度偏移
 */
uint32_t get_decode_sync_word_offset(void)
{
    return audio_decode_offset;
}


/**
 * @brief 清理当前播放位置
 * 
 */
void clean_decode_sync_word_offset(void)
{
    audio_decode_offset = 0;
}

/**
 * @brief 设置播放data发送will finsh信号阈值
 * 
 * @param size 
 */
void set_play_will_finish_threshold_data_size(int size)
{
    if(size > (AUDIO_PLAYER_THRESHOLD_DATA_SIZE)*2)
    {
        play_will_finish_threshold_data_size = size-(AUDIO_PLAYER_THRESHOLD_DATA_SIZE)*2;
    }
    else
    {
        play_will_finish_threshold_data_size = 0;
    }
    play_pcm_size = size;
}

/**
 * @brief 调用用户回调
 * 
 */
void do_call_callback_fun(void)
{
    if(g_end_paly_callback != NULL)
    {
        end_paly_callback_flag = false;
        if(play_end_cb_flag == AUDIO_PLAY_CB_STATE_PAUSE)
        {
            if(!play_will_finish)
            {
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_PAUSE_BEFORE_THRESHOLD;
            }
            else
            {
                play_end_cb_flag = AUDIO_PLAY_CB_STATE_PAUSE_AFTER_THRESHOLD;
            }
        }
        g_end_paly_callback(play_end_cb_flag);
    }
}

/**
 * @brief 注册解码器到解码器链表
 * 
 * @param registe_ops 解码器描述符
 */
void registe_decoder_ops(audio_play_decoder_ops_t *registe_ops)
{
    audio_play_decoder_ops_t *ops = (audio_play_decoder_ops_t *)decoder_ops;
    
    if(decoder_ops == NULL)
    {
        decoder_ops = registe_ops;
        return;
    }
    else if(decoder_ops == registe_ops)
    {
        ci_logdebug(LOG_AUDIO_PLAY,"decoder already registe!\n");
        return;
    }
    
    for(;ops->next_ops != NULL;ops = ops->next_ops)
    {
        if(ops->next_ops == registe_ops)
        {
            ci_logdebug(LOG_AUDIO_PLAY,"decoder already registe!\n");
            return;
        }
    }
    ops->next_ops = registe_ops;
}


/**
 * @brief 初始化播放任务
 * @note this init before task run,maybe other task used these resource
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
int32_t audio_play_init(void)
{
    #if AUDIO_PLAYER_ENABLE
    /* 注册prompt解码器 */
    #if USE_PROMPT_DECODER
    registe_decoder_ops(&prompt_decoder);  
    #endif
    
    /* 注册mp3解码器 */
    #if USE_MP3_DECODER
    registe_decoder_ops(&mp3_decoder);    
    #endif
    
    /* 注册aac解码器 */
    #if USE_AAC_DECODER
    registe_decoder_ops(&aac_decoder);    
    #endif
    
    /* 注册ms_wav解码器 */
    #if USE_MS_WAV_DECODER
    registe_decoder_ops(&ms_wav_decoder); 
    #endif

    /* 注册flac解码器 */
    #if USE_FLAC_DECODER
    registe_decoder_ops(&flac_decoder); 
    #endif
    #endif
    
    /*here only one msg queue,used overwrite function*/
    audio_play_queue = audio_play_queue_create(1, sizeof(audio_play_msg_t));
    if(NULL == audio_play_queue)
    {
        ci_logerr(LOG_AUDIO_PLAY,"audio_play_queue creat fail\r\n");   
        return RETURN_ERR;
    }
    
    audio_play_cmd_event_group = audio_play_event_group_create();
    if(audio_play_cmd_event_group == NULL)
    {
        ci_logerr(LOG_AUDIO_PLAY,"creat play event group error\n");
        return RETURN_ERR;
    }

    task_get_play_data_init();
    
    audio_play_task_create(task_audio_play, "task_audio_play", AUDIO_PLAYER_PROCTHREAD_STACK, NULL, 4, &audio_play_task_handle);
    audio_play_task_create(task_get_play_data, "task_get_play_data", AUDIO_PLAYER_DATATHREAD_STACK, NULL, 4, NULL);

    return RETURN_OK;
}


/**
 * @brief 设置播放倍速
 * 
 * @param play_speed 浮点型，1.0为无倍速，2.0为加速为2倍速，0.5为减速为1/2倍速
 */
void set_play_speed(float play_speed)
{
    #if AUDIO_PLAY_USE_SPEEDING_SPEECH
    sonicSetSpeed(sonic_stream, play_speed);
    #else
    ci_logerr(LOG_AUDIO_PLAY,"this operation is not supported, please open AUDIO_PLAY_USE_SPEEDING_SPEECH macro!\n");
    #endif
}


#if AUDIO_PLAYER_FIX_OFFSET_ISSUE
/**
 * @brief 获取最近一次被打断的数据相近的8个字节，用于解决应用程序可能存在的偏移不对齐问题
 * 
 * @retval RETURN_OK 获取成功
 * @retval RETURN_ERR 获取失败
 */
int32_t get_near_offset_data(uint8_t *src)
{
    if(near_offset_data_valid == true)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(src,near_offset_data,8);
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}
#endif

#include "ci130x_gpio.h"
/**
 * @brief 播放器调度任务
 * 
 * @param pvParameters 任务参数
 */
void task_audio_play(void *pvParameters)
{
    audio_play_msg_t play_msg = {0};
    uint32_t uxBits;   
    int32_t ret;

    /* 初始化播放任务所需的解码器、数据buff */
    if(RETURN_OK != task_audio_play_init_step())
    {
        CI_ASSERT(0,"task_audio_play_init err\r\n");
    }
    
    for(;;)
    {
        switch(audio_play_state)
        {
            case AUDIO_PLAY_STATE_IDLE:
            {
                uxBits = audio_play_event_group_wait_bits(audio_play_cmd_event_group, 0xFFFFFF, AUDIO_PLAY_OS_SUCCESS, AUDIO_PLAY_OS_FAIL, audio_play_os_block);
                ci_logdebug(LOG_AUDIO_PLAY,"cmd is 0x%x\n",uxBits);

                /* 接收播放请求消息 */
                if(0 == (uxBits&AUDIO_PLAY_EVENT_START))
                {
                    continue; 
                }
                if(AUDIO_PLAY_OS_SUCCESS != audio_play_queue_receive(audio_play_queue, &play_msg, 0))
                {
                    ci_logerr(LOG_AUDIO_PLAY,"play task Q_rec err\r\n");
                    continue;
                }
                else
                {
                    /* 配置播放类型，注册回调函数 */
                    g_end_paly_callback = play_msg.audio_play_end_hook;
                    end_paly_callback_flag = true;
                    play_end_cb_flag = AUDIO_PLAY_CB_STATE_UNKNOWN_ERR;
                    ci_logdebug(LOG_AUDIO_PLAY,"set g_end_paly_callback 0x%x\n",g_end_paly_callback);
                }

                ci_loginfo(LOG_AUDIO_PLAY,"play start\n");
                ci_logdebug(LOG_AUDIO_PLAY,"play type %d\n",play_msg.play_data_type);
                
                audio_play_state = AUDIO_PLAY_STATE_START;
                ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_PLAYING);
                break;
            }
            case AUDIO_PLAY_STATE_START:
            {
                // cm_pcm_buffer_info_t pcm_buffer_info;
                // pcm_buffer_info.play_buffer_info.block_num = 2;
                // pcm_buffer_info.play_buffer_info.buffer_num = 4;
                // pcm_buffer_info.play_buffer_info.block_size = 576;
                // pcm_buffer_info.play_buffer_info.buffer_size = pcm_buffer_info.play_buffer_info.block_size * pcm_buffer_info.play_buffer_info.block_num;
                
                // static bool is_pcm_buffer_inited = false;
                // if(!is_pcm_buffer_inited)
                // {
                //     pcm_buffer_info.play_buffer_info.pcm_buffer = pvPortMalloc(pcm_buffer_info.play_buffer_info.buffer_size*pcm_buffer_info.play_buffer_info.buffer_num);
                //     CI_ASSERT(pcm_buffer_info.play_buffer_info.pcm_buffer,"\n");
                //     cm_config_pcm_buffer(sg_play_device_index, CODEC_OUTPUT, &pcm_buffer_info);
                //     is_pcm_buffer_inited = true;
                // }

                /* 初始化播放参数 */
                para_recover_default();

                #if AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
                if((play_msg.play_data_type != AUDIO_PLAY_DATA_TYPE_CONTINUE)&&(0 !=  MASK_ROM_LIB_FUNC->newlibcfunc.strcmp_p("history", play_msg.decoder_name)))
                {
                    /* 处理文件头信息 */
                    if(RETURN_ERR == parse_file(&play_msg))
                    {
                        /* 错误！！！！解析文件格式出错 */
                        /* 此时已经开始数据读入了，需要停止数据读入关闭mf调用播放结束回调直接退回idle */
                        ci_logerr(LOG_AUDIO_PLAY,"parse file err!\n");
                        audio_play_state = AUDIO_PLAY_STATE_IDLE;
                        ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
                        pause_read_data();
                        free_bak_data();
                        do_call_callback_fun();
                        break;
                    }
                    /* 配置解码器 */
                    if(RETURN_ERR == find_decoder_ops(play_msg.decoder_name))
                    {
                        /* 错误！！！！找不到解码器 */
                        /* 此时已经开始数据读入了，需要停止数据读入关闭mf调用播放结束回调直接退回idle */
                        ci_logerr(LOG_AUDIO_PLAY,"parse not found decoder %s!\n",play_msg.decoder_name);
                        audio_play_state = AUDIO_PLAY_STATE_IDLE;
                        ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
                        pause_read_data();
                        free_bak_data();
                        do_call_callback_fun();
                        break;
                    }
                }
                else
                #endif
                {
                    /* 配置解码器 */
                    if(RETURN_ERR == find_decoder_ops(play_msg.decoder_name))
                    {
                        /* 错误！！！！找不到解码器 */
                        /* 此时已经开始数据并未开始读取，直接调用结束回调退回idle */
                        ci_logerr(LOG_AUDIO_PLAY,"history not found decoder %s!\n",play_msg.decoder_name);
                        audio_play_state = AUDIO_PLAY_STATE_IDLE;
                        ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
                        do_call_callback_fun();
                        break;
                    }
                    /* 请求数据 */
                    if(RETURN_ERR == send_get_data_msg(&play_msg))
                    {
                        /* 错误！！！！系统状态异常 */
                        /* 此时已经开始数据并未开始读取，直接调用结束回调退回idle */
                        ci_logerr(LOG_AUDIO_PLAY,"send get audio data msg err!\n");
                        audio_play_state = AUDIO_PLAY_STATE_IDLE;
                        ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
                        play_end_cb_flag = AUDIO_PLAY_CB_STATE_INTERNAL_ERR;
                        #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
                        if(curr_decoder_ops->destroy != NULL)
                        {
                            curr_decoder_ops->destroy();
                        }
                        #endif
                        do_call_callback_fun();
                        break;
                    }
                }
                
                /* 启动播放预解码 */
                ret = audio_play_start();
                if((RETURN_ERR == ret)/*||(decbuf == pcm_buf_base)*/)
                {
                    /* 错误！！！！解码数据有误，请检查数据是否正确 */
                    /* 此时已经开始数据读入了，需要停止数据读入关闭mf调用播放结束回调直接退回idle */
                    audio_play_state = AUDIO_PLAY_STATE_IDLE;
                    ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);
                    pause_read_data();

                    /* 请求暂停数据接收，保存状态 */
                    if(play_end_cb_flag == AUDIO_PLAY_CB_STATE_PAUSE)
                    {
                        /* 解码器现场保存 */
                        if(curr_decoder_ops->save != NULL)
                        {
                            curr_decoder_ops->save();
                            curr_decoder_ops->clear();
                        }      
                        history_decoder_ops = curr_decoder_ops;
                        ci_logdebug(LOG_AUDIO_PLAY,"save decoder ops is 0x%x\n",curr_decoder_ops);
                    }

                    free_bak_data();

                    #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
                    if(curr_decoder_ops->destroy != NULL)
                    {
                        curr_decoder_ops->destroy();
                    }
                    #endif
                    do_call_callback_fun();
                    if (decbuf != pcm_buf_no_cache)
                    {
                        cm_release_pcm_buffer(sg_play_device_index, CODEC_OUTPUT, decbuf);
                        decbuf = pcm_buf_no_cache;
                    }
                    break;
                }

                /* 启动声卡开始播放 */
                // ci_logdebug(LOG_AUDIO_PLAY,"play hw start\n");
                // audio_play_hw_start(play_msg.audio_device_arg.pa_cmd);

                receive_timeout_ticks = 50; //数据超时时间加长，减少cpu占用
                audio_play_state = AUDIO_PLAY_STATE_PLAYING;
                ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_PLAYING);
                break;
            }
            case AUDIO_PLAY_STATE_PLAYING:
            {
                // gpio_set_output_level_single(PA, pin_0, 0);
                uxBits = audio_play_event_group_wait_bits(audio_play_cmd_event_group, AUDIO_PLAY_EVENT_PAUSE|AUDIO_PLAY_EVENT_PLAY_TIMEOUT|AUDIO_PLAY_EVENT_STOP|AUDIO_PLAY_EVENT_ONE_BUF_DONE, AUDIO_PLAY_OS_SUCCESS, AUDIO_PLAY_OS_FAIL, audio_play_os_block); 
                /* 处理播放事件 */
                audio_play_playing_deal(uxBits);

                break;
            }
            case AUDIO_PLAY_STATE_STOP:
            {
                if (decbuf != pcm_buf_no_cache)
                {
                    cm_release_pcm_buffer(sg_play_device_index, CODEC_OUTPUT, decbuf);
                    decbuf = pcm_buf_no_cache; 
                }
                if((play_end_flag != AUDIO_PLAY_FLAG_REL_END)||(play_end_cb_flag == AUDIO_PLAY_CB_STATE_PAUSE))
                {
                    /* 如果是外部请求暂停，则声卡硬件立即停止并清理缓存 */                    
                    #if AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_HARD
                    /* 停止前通过递减调节硬件音量的方法解决pop音 */
                    audio_play_hw_fix_pop_issue_hard();
                    audio_play_hw_stop(play_msg.audio_device_arg.pa_cmd);
                    audio_play_hw_clean_data();
                    audio_play_hw_fix_pop_issue_hard_done();
                    #elif AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_SOFT
                    /* 停止前通过软件处理音频递减的方法解决pop音(数据处理加窗在驱动内完成) */
                    audio_play_hw_fix_pop_issue_soft();
                    audio_play_hw_stop(play_msg.audio_device_arg.pa_cmd);
                    audio_play_hw_clean_data();
                    audio_play_hw_fix_pop_issue_soft_done();
                    #else

                    /* 直接停止，可能导致pop音 */
                    audio_play_hw_stop(play_msg.audio_device_arg.pa_cmd);
                    audio_play_hw_clean_data();
                    #endif                    
                }
                else
                {
                    /* 否则等待数据播放完成，保证尾音不丢失，再停止硬件 */
                    while(0 != audio_play_hw_get_queue_remain_nums())
                    {
                        audio_play_task_delay(2);
                        continue;
                    }
                    audio_play_hw_stop(play_msg.audio_device_arg.pa_cmd);
                }

                play_end_flag = AUDIO_PLAY_FLAG_RESTART;

                /*清除解码缓存*/
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_in_buffer,0,READBUF_SIZE);

                #if AUDIO_PLAY_USE_SPEEDING_SPEECH
                /* 清理sonic_stream内部数据 */
                sonicCleanStream(sonic_stream);
                #endif

                audio_play_state = AUDIO_PLAY_STATE_IDLE;   
                ciss_set(CI_SS_PLAY_STATE,CI_SS_PLAY_STATE_IDLE);

                #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
                if(curr_decoder_ops->destroy != NULL)
                {
                    curr_decoder_ops->destroy();
                }
                #endif
                if(!play_will_finish)
                {
                    if(g_end_paly_callback != NULL)
                    {
                        ci_loginfo(LOG_AUDIO_PLAY,"AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD 2\n");
                        g_end_paly_callback(AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD);
                    }
                    play_will_finish = true;
                }
                /* 调用结束回调函数 */
                do_call_callback_fun();

                ci_loginfo(LOG_AUDIO_PLAY,"play end\n");
                break;
            }
            default:
                break;
        }
    }


    #pragma diag_suppress=Pe111
    /*or switch playing ,while in playing*/
    /*can't be here,just for code style*/
    if(curr_decoder_ops->destroy != NULL)
    {
        curr_decoder_ops->destroy();
    }
}
