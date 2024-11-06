/**
 * @file get_play_data.c
 * @brief 用于获取播放数据，提供从SD卡，flash，串口等读入数据
 * @version V2.0.3
 * @date 2018.07.17
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "get_play_data.h"
#include "audio_play_api.h"
#include "ci_log.h"
#include "ci_assert.h"
#include "sdk_default_config.h"
#if AUDIO_PLAY_USE_SD_CARD
#include "ff.h"
#endif
// #include "flash_rw_process.h"
#include "audio_play_os_port.h"
#include "adpcmdec.h"
#include "ci_voice_head.h"
#include "audio_play_decoder.h"
#include "ci130x_spiflash.h"
#include "FreeRTOS.h"
#include "romlib_runtime.h"
#include "flash_manage_outside_port.h"

#if AUDIO_PLAY_USE_NET
#include "lightcloud_http_client_ops.h"
#endif

#define WAIT_TIMEOUT       (100)

/**************************************************************************
                     define
****************************************************************************/
/**
 * @brief 数据源类型
 * 
 */
typedef enum
{
    GET_PLAY_DATA_NONE = 0,             /*!< 无                     */
    GET_PLAY_DATA_QSPI_FLASH = 1,       /*!< QSPI FLASHE            */
    GET_PLAY_DATA_QSPI_FLASH_LIST = 2,  /*!< QSPI FLASHE List       */
    GET_PLAY_DATA_SD_CARD = 3,          /*!< SD 卡                  */
    GET_PLAY_DATA_NET = 4,              /*!< 网络                   */
    GET_PLAY_DATA_OUTSIDE = 5,          /*!< 外部接口               */
    GET_PLAY_DATA_OUTSIDE_V2 = 6,       /*!< 外部接口V2             */
}hw_type;

/**
 * @brief 数据消息类型结构
 * 
 */
typedef struct
{
    uint32_t data_addr;    /*!< 数据buf地址              */
    uint32_t data_size;    /*!< 数据buf大小              */
    uint32_t data_flag;    /*!< buf标志                  */
}data_buf_msg_t;

/**
 * @brief 数据标识
 * 
 */
typedef enum
{
    NEW_DATA = 1,        /*!< 新数据               */
    CONTINUE_DATA = 2,   /*!< 历史缓冲数据         */
}get_data_flag_t;

/**
 * @brief 数据请求消息结构
 * 
 */
typedef struct
{
    hw_type type;                            /*!< 数据源类型           */
    union
    {
        struct
        {
            union
            {
                uint32_t arg;                /*!< 参数                 */
                uint32_t data_addr;          /*!< 地址                 */
                uint32_t data_addr_list;     /*!< 地址序列             */
            };
            uint32_t num;                    /*!< 数据大小             */
        }spi_flash_msg;                      /*!< flash请求消息        */
        struct
        {
            const char *dir;                 /*!< file dir             */
            const char *name;                /*!< file name            */
        }sd_card_msg;                        /*!< sd卡请求消息         */
        struct
        {
            const char *url;                 /*!< 网络URL              */
        }net_msg;                            /*!< 网络请求消息         */
    };
    uint32_t offset;                         /*!< 地址偏移             */
    get_data_flag_t flag;                    /*!< 请求标志             */
}get_play_data_msg_t;

#if AUDIO_PLAY_USE_NET
/**
 * @brief 网络下载线程参数
 * 
 */
typedef struct
{
    const char *url;    /*!< url               */
    int offset;         /*!< pos               */
}download_para_t;
#endif /* AUDIO_PLAY_USE_NET */

/**************************************************************************
                    static
****************************************************************************/
/*!  audio数据buf */
static uint8_t *data_buf = NULL;
/*!  数据请求队列 */
static audio_play_os_queue_t get_play_data_queue = NULL;
/*!  audio数据buf队列 */
static audio_play_os_queue_t data_buf_queue = NULL;
/*!  暂停事件信号 */
static audio_play_os_sem_t pause_data_event = NULL;
/*!  数据获取task状态 */
static get_play_data_state_t get_data_state = GET_PLAY_DATA_STATE_IDLE;
/*!  记录数据起始偏移 */
static uint32_t curr_offset = 0;
/*!  历史数据记忆表 */
static get_play_data_msg_t data_list =    
{
    .type = GET_PLAY_DATA_NONE
};
#if AUDIO_PLAY_USE_SD_CARD
/*!  文件描述符 */
static FIL fsrc;
#endif
/*!  数据源总大小 */
static uint32_t data_size = 0;

#if AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
static uint8_t *wakeup_voice_buf = NULL;
static uint32_t wakeup_voice_size = 0;
static uint32_t wakeup_voice_addr = 0;
static uint32_t wakeup_voice_threshold_data_size = 0;
static prompt_decoder_config wakeup_voice_config;
#endif

#if AUDIO_PLAY_USE_NET
/* 网络数据缓冲流最大数量 */
#define GET_PLAY_DATA_STREAM_COUNT     (2)
/*! 网络数据缓冲流 */
static audio_play_os_stream_t net_stream = NULL;
/*! 网络数据下载完成信号 */
static audio_play_os_sem_t download_end_sem = NULL;
/*! 网络http下载器描述符 */
static http_client_c *g_client = NULL;
/*! 数据下载量 */
static int s_len = 0;
/*! 网络初始化标志 */
static int net_client_is_ok = 0;
/*! 停止下载标志 */
static bool http_stop_flag = false;
/*! 下载任务句柄 */
audio_play_os_task_t http_download_task_handle;
#endif /* AUDIO_PLAY_USE_NET */

#if AUDIO_PLAY_USE_OUTSIDE
/*!  外部数据流描述符 */
static audio_play_os_stream_t g_outside_stream = NULL;
/*!  外部数据结束信号 */
static audio_play_os_sem_t g_outside_end_sem = NULL;
#endif /* AUDIO_PLAY_USE_OUTSIDE */
#if AUDIO_PLAY_USE_OUTSIDE_V2
/*!  外部数据消息述符 */
static audio_play_os_queue_t g_outside_v2_msg = NULL;
/*!  外部数据结束信号 */
static audio_play_os_sem_t g_outside_v2_end_sem = NULL;
#endif /* AUDIO_PLAY_USE_OUTSIDE_V2 */


/**************************************************************************
                    function
****************************************************************************/
#if AUDIO_PLAY_USE_NET
/**
 * @brief 网络http下载数据回调函数从这里获取下载数据写入net_stream流
 * 
 * @param p_user_ctx 
 * @param pos 数据标识
 * @param buf 数据地址
 * @param len 数据大小
 * @param type 类型
 * 
 * @retval RETURN_OK 操作正常
 * @retval RETURN_ERR 操作出错
 */
static int audio_play_http_download_data(void* p_user_ctx, e_data_pos pos, const char* buf, size_t len, const char* type)
{
    int sendbyte = 0;
    size_t xBytesSent = 0;
    
    if(http_stop_flag != true)
    {
        if(pos != DATA_LAST)
        {
            ci_logverbose(LOG_AUDIO_GET_DATA,"http load %d\n",len);
            /* 将网络数据下发到stream */
            do{
                ci_logverbose(LOG_AUDIO_GET_DATA,"net_stream is spaces 0x%d\n",audio_play_stream_buffer_get_spaces_size(net_stream));
                xBytesSent = audio_play_stream_buffer_send(net_stream,buf+sendbyte,len,pdMS_TO_TICKS(500));
                //xBytesSent = audio_play_stream_buffer_send(net_stream,buf+sendbyte,len,audio_play_os_unblock);
                sendbyte += xBytesSent;
                s_len += xBytesSent;
                len -= xBytesSent;
                /* FIXME: 这样相当于查询的方式检查暂停/停止的请求，这种方式查询太频繁会占用cpu过久，
                 * 太不频繁会导致无法快速响应暂停，目前配置500ms超时时间，考虑修改为一个暂停事件或信号的方式 */
                if(http_stop_flag == true)
                {
                    break;
                }
            }while(len != 0);
            
        }
        else
        {
            /* 全部下载完成 */
            audio_play_task_delay(20);
            s_len = 0;
            audio_play_semaphore_give(download_end_sem);
            ci_loginfo(LOG_AUDIO_GET_DATA,"download finsh!\n");
        }
    }
    return 0;
}

/**
 * @brief 网络http停止下载回调函数
 * 
 * @retval RETURN_OK 操作正常
 * @retval RETURN_ERR 操作出错
 */
static int audio_play_http_download_stop(void)
{
    if(http_stop_flag == true)
    {
        //http_stop_flag = false;
        ci_loginfo(LOG_AUDIO_GET_DATA,"download will stop!\n");
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
 * @brief 网络http下载描述符初始化，
 * @note 在第一次请求网络数据源时初始化，需要保证在请求网络数据前已完成tcp/ip协议栈初始化
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
static int32_t task_get_play_data_net_init(void)
{
    /* 初始化http连接 */
    g_client = cloud_create_http_client();
    if(NULL == g_client)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"duer_create_http_client() failed\n");
        return RETURN_ERR;
    }

    /* 注册处理函数 */
    cloud_http_client_reg_data_hdlr_cb(g_client,audio_play_http_download_data,NULL);
    cloud_http_client_reg_stop_notify_cb(g_client,audio_play_http_download_stop);

    /* 网络数据流初始化 */
    net_stream = audio_play_stream_buffer_create(GET_PLAY_DATA_BUFF_SIZE*GET_PLAY_DATA_BUFF_COUNT*5,GET_PLAY_DATA_BUFF_SIZE);
    if(NULL == net_stream)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"audio_play_stream_buffer_create() failed\n");
        return RETURN_ERR;
    }

    /* 下载完成信号初始化 */
    download_end_sem = audio_play_semaphore_create_binary();
    if(NULL == download_end_sem)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"download_end_sem creat fail\r\n");   
        return RETURN_ERR;
    }

    return RETURN_OK;
}


/**
 * @brief 创建下载网络数据线程
 * 
 * @param url 播放源url
 */
static void task_http_download(void *para)
{
    e_http_result ret = HTTP_FAILED;
    download_para_t * download_para = (download_para_t *)para;
    http_stop_flag = false;
    ret = cloud_http_client_get(g_client,download_para->url, download_para->offset);
    // TODO: 通过返回值能否判断所有下载失败的问题？
    if(ret != HTTP_OK)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"http download failed!\n");
        /* 因网络原因下载失败 */
        audio_play_task_delay(pdMS_TO_TICKS(500));
        audio_play_semaphore_give(download_end_sem);
    }
    ci_loginfo(LOG_AUDIO_GET_DATA,"task_http_download will exit! \n");
    audio_play_task_delete(NULL);
}
#endif /* AUDIO_PLAY_USE_NET */


/**
 * @brief 在请求暂停时，保存当前读取数据进度
 * 
 * @param msg 当前播放信息句柄
 */
static void save_audio_info(get_play_data_msg_t *msg)
{
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&data_list,0x0,sizeof(get_play_data_msg_t));
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&data_list,msg,sizeof(get_play_data_msg_t));

#if AUDIO_PLAY_USE_NET
    if(msg->type == GET_PLAY_DATA_NET)
    {
        http_stop_flag = true;
        vTaskDelay(200);
        while(AUDIO_PLAY_OS_FAIL == audio_play_stream_buffer_reset(net_stream))
        {
            vTaskDelay(20);
        }
        //audio_play_task_delete(http_download_task_handle);
        data_list.offset = get_decode_sync_word_offset()+msg->offset;
    }
    else
#endif
    {
        data_list.offset = get_decode_sync_word_offset()+msg->offset;
    }
}


/**
 * @brief 清理播放offset
 * 
 */
void clean_curr_offset(void)
{
    clean_decode_sync_word_offset();
    curr_offset = 0;
}



/**
 * @brief 检查ci voice头信息
 * 
 * @param wave_head 
 * @param wave_lens 
 * @param wave_addr 
 * @return int32_t 
 */
int32_t check_ci_voice_head(ci_voice_head_t *wave_head, uint32_t *wave_lens, uint32_t *head_lens,prompt_decoder_config *config,uint32_t *threshold_data_size)
{
    int32_t ret = -1;
    
    *threshold_data_size = 0;
    
    #if AUDIO_PLAY_SUPPT_MP3_PROMPT
    if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_ci_mp3.ID3,"ID3",3))
    {
        if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_ci_mp3.CI,"CI",2))
        {
            config->mode = CI_ADPCM_DECODER_MODE_MP3;
            *wave_lens = wave_head->wave_ci_mp3.file_size - 30;
            *head_lens = 30;
            *threshold_data_size = wave_head->wave_ci_mp3.pcm_size*2;
            ret = 0;
        }
    }
    else
    #endif
    #if AUDIO_PLAY_SUPPT_FLAC_PROMPT
    if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_ci_flac.FLAC,"fLaC",4))
    {
        if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_ci_flac.vorbis_comment.ci_flag_str,"filesize=",9))
        {
            config->mode = CI_ADPCM_DECODER_MODE_FLAC;
            char *info = wave_head->wave_ci_flac.streaminfo.info;
            config->flac.min_blocksize=((uint16_t)info[0]<<8) |info[1];					//最小块大小
            config->flac.max_blocksize=((uint16_t)info[2]<<8) |info[3];					//最大块大小
            config->flac.min_framesize=((uint32_t)info[4]<<16)|((uint16_t)info[5]<<8)|info[6];//最小帧大小
            config->flac.max_framesize=((uint32_t)info[7]<<16)|((uint16_t)info[8]<<8)|info[9];//最大帧大小
            config->flac.samplerate=((uint32_t)info[10]<<12)|((uint16_t)info[11]<<4)|((info[12]&0xf0)>>4);//采样率
            config->flac.channels=((info[12]&0x0e)>>1)+1;							//音频通道数
            config->flac.bps=((((uint16_t)info[12]&0x01)<<4)|((info[13]&0xf0)>>4))+1;	//采样位数16?24?32? 
            config->flac.totalsamples=((uint32_t)info[14]<<24)|((uint32_t)info[15]<<16)|((uint16_t)info[16]<<8)|info[17];//一个声道的总采样数
            
            char *filesize_str = wave_head->wave_ci_flac.vorbis_comment.ci_size_str;
            uint32_t size = 0;
            for(int i = 0;i < 8;i++)
            {
                size += (filesize_str[7-i] - '0')*powf(10.0f,i);
            }
            *wave_lens = size- 63;
            *head_lens = 63;
            *threshold_data_size = 0;//TODO:等待工具提供
            ret = 0;
        }
    }
    else
    #endif
    {
        if(wave_head->wave_pcm_44.wfmt.wav_formatag == 0x0011)
        {
            #if AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT
            //标准adpcm TODO:目前这种格式的文件还不支持开启AUDIO_PLAY_USE_QSPI_FLASH_LIST宏进行数据组合播报
            config->mode = CI_ADPCM_DECODER_MODE_IMAADPCM;
            if (0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_pcm_48.dchunk.data, "data", 4))
            {
                config->wav.head_size = 48;
                config->wav.nBlockAlign = wave_head->wave_pcm_48.wfmt.nBlockAlign;
                config->wav.nSamplesPerSec = wave_head->wave_pcm_48.wfmt.nSamplesPerSec;
                *wave_lens = wave_head->wave_pcm_48.dchunk.chunksize;
                *head_lens = 48;
                *threshold_data_size = (*wave_lens)*4;
                /* 目前只支持67的block大小 */
                if(config->wav.nBlockAlign == 67)
                {
                    ret = 0;
                }
            }
            else if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_adpcm.dchunk.data, "data", 4))
            {
                config->wav.head_size = 60;
                config->wav.nBlockAlign = wave_head->wave_adpcm.wfmt.nBlockAlign;
                config->wav.nSamplesPerSec = wave_head->wave_adpcm.wfmt.nSamplesPerSec;
                *wave_lens = wave_head->wave_adpcm.dchunk.chunksize;
                *head_lens = 60;
                *threshold_data_size = (*wave_lens)*4;
                /* 目前只支持67的block大小 */
                if(config->wav.nBlockAlign == 67)
                {
                    ret = 0;
                }
            }
            #endif
        }
        else
        {
            //旧版CI非标adpcm格式
            int fmt_chunk_size = wave_head->wave_pcm_44.wfmt.fmt_chunk_size;
            switch (fmt_chunk_size)
            {
                case 0x10:
                {
                    //旧版CI非标adpcm格式
                    config->mode = CI_ADPCM_DECODER_MODE_CIADPCM;
                    if (0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_pcm_44.dchunk.data, "data", 4))
                    {
                        *wave_lens = wave_head->wave_pcm_44.dchunk.chunksize/4;
                        *head_lens = 44;
                        *threshold_data_size = (*wave_lens)*4;
                        ret = 0;
                    }
                    break;
                }
                case 0x14:
                {
                    //CI标准wav格式
                    config->mode = CI_ADPCM_DECODER_MODE_WAV;
                    config->wav.nSamplesPerSec = wave_head->wave_pcm_48.wfmt.nSamplesPerSec;
                    if (0 == MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(wave_head->wave_pcm_48.dchunk.data, "data", 4))
                    {
                        *wave_lens = wave_head->wave_pcm_48.dchunk.chunksize;
                        *head_lens = 48;
                        *threshold_data_size = (*wave_lens);
                        ret = 0;
                    }
                    break;
                }
            }
        }
    }

    if (-1 == ret)
    {
        ci_logerr(LOG_AUDIO_PLAY, "wave fmt err!\n");
    }

    return ret;
}


#if AUDIO_PLAY_USE_QSPI_FLASH_LIST
int32_t process_flash_addr_list(char *data_buf, uint32_t size, uint32_t flash_addr, uint32_t *bytesLeft, uint32_t *threshold_data_size)
{
    prompt_decoder_config config;
    uint32_t wave_total_lens;
    uint32_t wave_head_size;

    post_read_flash((char *)data_buf, flash_addr, size);

    if(-1 == check_ci_voice_head((ci_voice_head_t *)data_buf,&wave_total_lens,&wave_head_size,&config,threshold_data_size))
    {
        return -1;
    }
    #if AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT
    prompt_decoder.config((void*)(&config));
    #endif
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf,0x0,wave_head_size);
    *data_buf = 's';
    *(data_buf+1) = 'p';
    *(data_buf+2) = 'l';
    *(data_buf+3) = 'c';
    if(wave_total_lens +wave_head_size > size)
    {
        *bytesLeft = wave_total_lens - size +wave_head_size;
        return size;
    }
    else
    {
        switch(config.mode)
        {
            case CI_ADPCM_DECODER_MODE_MP3:
            case CI_ADPCM_DECODER_MODE_FLAC:
            {
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf+wave_total_lens+wave_head_size,0x0,size-wave_total_lens-wave_head_size);
                break;
            }
        }
        *bytesLeft = 0;
        return wave_total_lens+wave_head_size;
    }
}
#endif


/**
 * @brief 获取播放数据任务
 * 
 * @param pvParameters task参数
 */
void task_get_play_data(void *pvParameters)
{
    #if AUDIO_PLAY_USE_QSPI_FLASH_LIST
    uint32_t *flash_addr_list = NULL;
    int32_t flash_addr_index = -1;
    uint32_t flash_addr_offset = 0;
    uint32_t threshold_list_data_size = 0;
    #endif
    #if AUDIO_PLAY_USE_NET
    download_para_t download_para;
    #endif
    #if AUDIO_PLAY_USE_SD_CARD
    FRESULT res;
    uint32_t br;
    #endif
    #if AUDIO_PLAY_USE_OUTSIDE_V2
    bool force_data_addr = false;
    #endif
    get_play_data_msg_t msg;
    uint32_t read_size = 0;
    data_buf_msg_t buf_msg;
    static uint32_t first_flag = 0;
    static uint8_t buf_index = 0;
    uint32_t msg_data_flag = 0;
    uint32_t msg_data_size = 0;
    uint32_t take_pause_data_event_timeout = 2;

    for(;;)
    {
        switch(get_data_state)
        {
            case GET_PLAY_DATA_STATE_IDLE:
            {
                if(AUDIO_PLAY_OS_SUCCESS != audio_play_queue_receive(get_play_data_queue, &msg, audio_play_os_block))
                {
                    ci_logerr(LOG_AUDIO_GET_DATA,"No get play data requests were received\r\n");
                    continue;/*no message*/
                }
                get_data_state = GET_PLAY_DATA_STATE_START;
                break;
            }
            case GET_PLAY_DATA_STATE_START:
            {
                clean_curr_offset();
                audio_play_queue_reset(data_buf_queue);

                if(CONTINUE_DATA == msg.flag)
                {
                    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&msg,&data_list,sizeof(get_play_data_msg_t));
                    data_list.type = GET_PLAY_DATA_NONE;
                }

                ci_logdebug(LOG_AUDIO_GET_DATA,"msg offset is %d\n",msg.offset);
                curr_offset = msg.offset;

                switch(msg.type)
                {
                    case GET_PLAY_DATA_QSPI_FLASH :
                    {
                        #if AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
                        if(msg.spi_flash_msg.data_addr == wakeup_voice_addr)
                        {
                            //如果该命令词地址已常驻内存，直接从内存拷贝
                            data_size = wakeup_voice_size;
                            set_play_will_finish_threshold_data_size(4096);//TODO
                            #if AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT
                            prompt_decoder.config((void*)(&wakeup_voice_config));
                            #endif
                            if(wakeup_voice_buf == NULL)
                            {
                                /* 抛出错误 */
                                ci_logerr(LOG_AUDIO_GET_DATA,"wakeup_voice_buf is NULL\n");
                                buf_msg.data_flag = AUDIO_DATD_BUFF_ERR;
                                audio_play_queue_send(data_buf_queue,&buf_msg,0);
                                ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                continue;
                            }
                        }
                        else
                        #endif
                        {
                            prompt_decoder_config config;
                            uint32_t wave_total_lens;
                            uint32_t wave_head_size;
                            uint32_t threshold_data_size;
                            /* 根据ci flash 特性需要提前读一笔数据,读出adpcm的大小 */
                            //post_read_flash((char *)(data_buf), msg.spi_flash_msg.data_addr, GET_PLAY_DATA_BUFF_SIZE);
							post_read_flash((char *)(data_buf), msg.spi_flash_msg.data_addr, GET_PLAY_DATA_BUFF_SIZE);
                            first_flag ++;
                            buf_index++;
                            /* 检查头，获取地址和大小*/
                            if(-1 == check_ci_voice_head((ci_voice_head_t *)data_buf,&wave_total_lens,&wave_head_size,&config,&threshold_data_size))
                            {
                                buf_msg.data_flag = AUDIO_DATD_BUFF_ERR;
                                audio_play_queue_send(data_buf_queue,&buf_msg,0);
                                ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                continue;
                            }
                            set_play_will_finish_threshold_data_size(threshold_data_size);
                            #if AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT
                            prompt_decoder.config((void*)(&config));
                            #endif

                            MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf,0x0,wave_head_size);
                            if(wave_total_lens +wave_head_size > GET_PLAY_DATA_BUFF_SIZE)
                            {
                                data_size = wave_total_lens - GET_PLAY_DATA_BUFF_SIZE +wave_head_size;
                            }
                            else
                            { 
                                switch(config.mode)
                                {
                                    case CI_ADPCM_DECODER_MODE_MP3:
                                    case CI_ADPCM_DECODER_MODE_FLAC:
                                    {
                                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf+wave_total_lens+wave_head_size,0x0,GET_PLAY_DATA_BUFF_SIZE-wave_total_lens-wave_head_size);
                                        break;
                                    }
                                }
                                /* 小于4K情况，这种方式优化性能，直接结束数据获取任务 */
                                buf_msg.data_flag = AUDIO_DATD_BUFF_END;
                                buf_msg.data_size = (wave_total_lens+wave_head_size);
                                buf_msg.data_addr = (uint32_t)(data_buf);
                                audio_play_queue_send(data_buf_queue,&buf_msg,0);
                                ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                continue;
                            }
                        }
                        break;
                    }
                    #if AUDIO_PLAY_USE_QSPI_FLASH_LIST
                    case GET_PLAY_DATA_QSPI_FLASH_LIST:
                    {
                        uint32_t flash_addr_bytesLeft;
                        int32_t flash_read_size = 0;
                        uint32_t threshold_data_size = 0;
                        flash_addr_list = (uint32_t *)msg.spi_flash_msg.data_addr_list;
                        flash_addr_index = -1;
                        first_flag ++;
                        buf_index++;
                        threshold_list_data_size = 0;
                        
                        do{
                            if(GET_PLAY_DATA_BUFF_SIZE-flash_read_size < 44)
                            {
                                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((char *)(data_buf+flash_read_size),0x0,GET_PLAY_DATA_BUFF_SIZE-flash_read_size);
                                flash_read_size = GET_PLAY_DATA_BUFF_SIZE;
                                flash_addr_bytesLeft = 0;
                                break;
                            }
                            flash_addr_index++;
                            int32_t br = process_flash_addr_list((char *)(data_buf+flash_read_size),GET_PLAY_DATA_BUFF_SIZE-flash_read_size,*(flash_addr_list+flash_addr_index),&flash_addr_bytesLeft,&threshold_data_size);
                            if(br == -1)
                            {
                                buf_msg.data_flag = AUDIO_DATD_BUFF_ERR;
                                audio_play_queue_send(data_buf_queue,&buf_msg,0);
                                ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                break;
                            }
                            threshold_list_data_size += threshold_data_size;
                            flash_read_size += br;
                            flash_addr_offset = br;
                            if(flash_addr_index+1 >= msg.spi_flash_msg.num)
                            {
                                set_play_will_finish_threshold_data_size(threshold_list_data_size);
                                break;
                            }
                        }
                        while(flash_read_size != GET_PLAY_DATA_BUFF_SIZE);

                        data_size = flash_addr_bytesLeft;
                        
                        if(buf_msg.data_flag != AUDIO_DATD_BUFF_ERR)
                        {
                            if(flash_read_size != GET_PLAY_DATA_BUFF_SIZE)
                            {
                                buf_msg.data_size = flash_read_size;
                                buf_msg.data_addr = (uint32_t)(data_buf);
                                buf_msg.data_flag = AUDIO_DATD_BUFF_END;
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                audio_play_queue_send(data_buf_queue,&buf_msg,0);
                                ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                                continue;
                            }
                        }
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_SD_CARD
                    case GET_PLAY_DATA_SD_CARD :
                    {
                        ci_loginfo(LOG_AUDIO_GET_DATA,"get mp3 name is %s\n",msg.sd_card_msg.name);
                        /* TODO: f_chdir函数在多线程使用时需要用户添加互斥锁保护，如果用户在其他线程也使用
                                    f_chdir，请将以下代码保护到锁中 */
                        /*********************************************************************************/
                        res = f_chdir(msg.sd_card_msg.dir);
                        if(FR_OK != res)
                        {
                            ci_logerr(LOG_AUDIO_GET_DATA,"chdir error!\n");
                            continue;
                        }
                        f_close(&fsrc);
                        res=f_open(&fsrc,msg.sd_card_msg.name,FA_OPEN_EXISTING|FA_READ);
                        if(FR_OK != res)
                        {
                            ci_logerr(LOG_AUDIO_GET_DATA,"file open error!\n");
                            continue;
                        }
                        f_chdir("/");
                        /*********************************************************************************/
                        data_size = f_size(&fsrc) - msg.offset;
                        f_lseek(&fsrc,msg.offset);
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_NET
                    case GET_PLAY_DATA_NET:
                    {
                        audio_play_stream_buffer_reset(net_stream);
                        audio_play_queue_reset((audio_play_os_queue_t)download_end_sem);
                        download_para.url = msg.net_msg.url;
                        download_para.offset = msg.offset;
                        data_size = 0xffffffff;
                        ci_loginfo(LOG_AUDIO_GET_DATA,"let's GET_PLAY_DATA_NET!!\n");
                        audio_play_task_create( task_http_download, "task_http_download", 512, &download_para, 3, &http_download_task_handle);
                        audio_play_task_delay(pdMS_TO_TICKS(500));
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_OUTSIDE
                    case GET_PLAY_DATA_OUTSIDE :
                    {
                        data_size = 0xffffffff;
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_OUTSIDE_V2
                    case GET_PLAY_DATA_OUTSIDE_V2:
                    {
                        data_size = 0xffffffff;
                        force_data_addr = true;
                        break;
                    }
                    #endif
                    default:
                    {
                        CI_ASSERT(0,"get data msg type err !\n");
                    }
                }          

                get_data_state = GET_PLAY_DATA_STATE_BUSY;
                break;
            }
            case GET_PLAY_DATA_STATE_BUSY:
            {
                /* 暂停保存逻辑 */
                /* take_pause_data_event_timeout在启动和播放中途采用不同是超时时间以兼顾启动响应速度和打断响应速度*/
                if (first_flag < GET_PLAY_DATA_BUFF_COUNT)
                {
                    take_pause_data_event_timeout = 2;
                }
                else
                {
                    take_pause_data_event_timeout = pdMS_TO_TICKS(20);
                }
                if(AUDIO_PLAY_OS_SUCCESS ==audio_play_semaphore_take(pause_data_event,take_pause_data_event_timeout))
                {                       
                    /* 收到数据获取暂停事件，保存数据读取进度信息 */
                    save_audio_info(&msg);

                    /* 进入停止状态，强制清除现有buf消息 */
                    get_data_state = GET_PLAY_DATA_STATE_STOP;
                    audio_play_queue_reset(data_buf_queue);
                    goto get_data_stop; //FIXME: 现在只能用goto解决这个问题了
                }

                /* 确保存在已经释放的空buf */
                if(AUDIO_PLAY_OS_SUCCESS == audio_play_queue_is_full(data_buf_queue))
                {
                    continue;
                }

                if(msg.type != GET_PLAY_DATA_NET)
                {
                    /* 更新data_size若数据读取完毕发出结束标志 */
                    if(data_size < GET_PLAY_DATA_BUFF_SIZE)
                    {
                        /* 文件结束最后一笔数据 */
                        read_size = data_size;
                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE),0x00,GET_PLAY_DATA_BUFF_SIZE);
                        get_data_state = GET_PLAY_DATA_STATE_STOP;
                        msg_data_flag = AUDIO_DATD_BUFF_END;
                        msg_data_size = read_size;
                    }
                    else
                    {
                        read_size = GET_PLAY_DATA_BUFF_SIZE;
                        data_size = data_size - GET_PLAY_DATA_BUFF_SIZE;
                        get_data_state = GET_PLAY_DATA_STATE_BUSY;
                        msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                        msg_data_size = GET_PLAY_DATA_BUFF_SIZE;
                    }
                }
                else
                {
                    /* 来自网络的数据size未知 */
                    read_size = GET_PLAY_DATA_BUFF_SIZE;
                }

                switch(msg.type)
                {
                    case GET_PLAY_DATA_QSPI_FLASH :
                    {
                        #if AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
                        if(msg.spi_flash_msg.data_addr == wakeup_voice_addr)
                        {
                            //如果该命令词地址已常驻内存，直接从内存拷贝
                            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE),(char *)(wakeup_voice_buf+GET_PLAY_DATA_BUFF_SIZE*first_flag),read_size);
                        }
                        else
                        #endif
                        {
                            /* 请求flash数据到buf */
                            /* 这里flash驱动会4K读取，最后一包数据时read_size虽然不是4K但仍读4K数据到buff*/
                            //post_read_flash((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE), msg.spi_flash_msg.data_addr+GET_PLAY_DATA_BUFF_SIZE*first_flag, read_size);
                            post_read_flash((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE), msg.spi_flash_msg.data_addr+GET_PLAY_DATA_BUFF_SIZE*first_flag, read_size);
                        }
                        break;
                    }
                    #if AUDIO_PLAY_USE_QSPI_FLASH_LIST
                    case GET_PLAY_DATA_QSPI_FLASH_LIST:
                    {
                        //post_read_flash((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE), *(flash_addr_list+flash_addr_index)+flash_addr_offset, read_size);
						post_read_flash((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE), *(flash_addr_list+flash_addr_index)+flash_addr_offset, read_size);
                        flash_addr_offset += read_size;
                        if((msg_data_flag == AUDIO_DATD_BUFF_END)&&(flash_addr_index+1 < msg.spi_flash_msg.num))
                        {
                            uint32_t flash_addr_bytesLeft = 0;
                            int32_t flash_read_size = read_size;
                            uint32_t threshold_data_size = 0;
                            
                            if(GET_PLAY_DATA_BUFF_SIZE-flash_read_size < 44)
                            {
                                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+flash_read_size),0x0,GET_PLAY_DATA_BUFF_SIZE-flash_read_size);
                                msg_data_size = GET_PLAY_DATA_BUFF_SIZE;
                                get_data_state = GET_PLAY_DATA_STATE_BUSY;
                                msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                                data_size = 0;
                            }
                            else
                            {

                                do{
                                    if(GET_PLAY_DATA_BUFF_SIZE-flash_read_size < 44)
                                    {
                                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+flash_read_size),0x0,GET_PLAY_DATA_BUFF_SIZE-flash_read_size);
                                        flash_read_size = GET_PLAY_DATA_BUFF_SIZE;
                                        flash_addr_bytesLeft = 0;
                                        break;
                                    }
                                    flash_addr_index++;
                                    int32_t br = process_flash_addr_list((char *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+flash_read_size),GET_PLAY_DATA_BUFF_SIZE-flash_read_size,*(flash_addr_list+flash_addr_index),&flash_addr_bytesLeft,&threshold_data_size);
                                    if(br == -1)
                                    {
                                        msg_data_flag = AUDIO_DATD_BUFF_ERR;
                                        break;
                                    }
                                    threshold_list_data_size += threshold_data_size;
                                    flash_addr_offset = br;
                                    flash_read_size += br;
                                    if(flash_addr_index+1 >= msg.spi_flash_msg.num)
                                    {
                                        set_play_will_finish_threshold_data_size(threshold_list_data_size);
                                        break;
                                    }
                                }
                                while(flash_read_size != GET_PLAY_DATA_BUFF_SIZE);

                                data_size = flash_addr_bytesLeft;
                                
                                if(msg_data_flag != AUDIO_DATD_BUFF_ERR)
                                {
                                    msg_data_size = flash_read_size;
                                    if(flash_read_size != GET_PLAY_DATA_BUFF_SIZE)
                                    {
                                        msg_data_flag = AUDIO_DATD_BUFF_END;
                                        data_size = read_size;
                                    }
                                    else
                                    {
                                        get_data_state = GET_PLAY_DATA_STATE_BUSY;
                                        msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                                    }
                                }
                            }
                        }

                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_SD_CARD
                    case GET_PLAY_DATA_SD_CARD :
                    {
                        /* 通过文件系统读取数据 */
                        f_read(&fsrc, (void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE), read_size, &br);
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_NET
                    case GET_PLAY_DATA_NET:
                    {
                        /* 从网络数据流中获取数据 */
                        size_t xBytesSent = 0;
                        uint32_t offset_size = 0; 
                        do{
                            xBytesSent = audio_play_stream_buffer_receive(net_stream,(void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+offset_size),read_size-offset_size,pdMS_TO_TICKS(20));
                            offset_size += xBytesSent;
                            ci_logverbose(LOG_AUDIO_GET_DATA,"offset_size is %d!\n",offset_size);
                            if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(pause_data_event,pdMS_TO_TICKS(20)))
                            {                       
                                /* 收到数据获取暂停事件，保存数据读取进度信息 */
                                save_audio_info(&msg);

                                /* 进入停止状态，强制清除现有buf消息 */
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                audio_play_queue_reset(data_buf_queue);
                                goto get_data_stop; //FIXME: 现在只能用goto解决这个问题了
                            }
                            if(offset_size != read_size)
                            {
                                /* 获取不到足够的数据则检查是否完成所有数据下载 */
                                /* 否则继续等待 */
                                if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(download_end_sem,0))
                                {
                                    /* 确认数据流完成了所有下载 */
                                    if(AUDIO_PLAY_OS_SUCCESS == audio_play_stream_buffer_is_empty(net_stream))
                                    {
                                        /* 文件结束最后一笔数据 */
                                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+offset_size),0x00,read_size-offset_size);
                                        get_data_state = GET_PLAY_DATA_STATE_STOP;
                                        data_size = read_size;
                                        ci_logdebug(LOG_AUDIO_GET_DATA,"Take end data %d!\n",offset_size);
                                        msg_data_flag = AUDIO_DATD_BUFF_END;
                                        msg_data_size = offset_size;
                                        break;
                                    }
                                    else
                                    {
                                        audio_play_semaphore_give(download_end_sem);
                                    }
                                }
                                
                            }
                            msg_data_size = offset_size;
                            msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                        }while(offset_size != read_size);
                        ci_logverbose(LOG_AUDIO_GET_DATA,"net_stream is spaces 0x%d\n",audio_play_stream_buffer_get_spaces_size(net_stream));
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_OUTSIDE
                    case GET_PLAY_DATA_OUTSIDE :
                    {
                        /* 从外部数据流中获取数据 */
                        size_t xBytesSent = 0;
                        uint32_t offset_size = 0; 
                        do{
                            xBytesSent = audio_play_stream_buffer_receive(g_outside_stream,(void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+offset_size),read_size-offset_size,pdMS_TO_TICKS(20));
                            offset_size += xBytesSent;
                            ci_logverbose(LOG_AUDIO_GET_DATA,"offset_size is %d!\n",offset_size);
                            if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(pause_data_event,pdMS_TO_TICKS(20)))
                            {                       
                                /* 收到数据获取暂停事件，保存数据读取进度信息 */
                                save_audio_info(&msg);

                                /* 进入停止状态，强制清除现有buf消息 */
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                audio_play_queue_reset(data_buf_queue);
                                goto get_data_stop; //FIXME: 现在只能用goto解决这个问题了
                            }
                            if(offset_size != read_size)
                            {
                                /* 获取不到足够的数据则检查是否完成所有数据下载 */
                                /* 否则继续等待 */
                                if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(g_outside_end_sem,0))
                                {
                                    /* 确认数据流完成了所有下载 */
                                    if(AUDIO_PLAY_OS_SUCCESS == audio_play_stream_buffer_is_empty(g_outside_stream))
                                    {
                                        /* 文件结束最后一笔数据 */
                                        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void *)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE+offset_size),0x00,read_size-offset_size);
                                        get_data_state = GET_PLAY_DATA_STATE_STOP;
                                        data_size = read_size;
                                        ci_logdebug(LOG_AUDIO_GET_DATA,"Take end data %d!\n",offset_size);
                                        msg_data_flag = AUDIO_DATD_BUFF_END;
                                        msg_data_size = offset_size;
                                        break;
                                    }
                                    else
                                    {
                                        audio_play_semaphore_give(g_outside_end_sem);
                                    }
                                }
                            }
                            msg_data_size = offset_size;
                            msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                        }while(offset_size != read_size);
                        break;
                    }
                    #endif
                    #if AUDIO_PLAY_USE_OUTSIDE_V2
                    case GET_PLAY_DATA_OUTSIDE_V2:
                    {
                        uint32_t rev_outside_msg;
                        audio_play_os_state_t ret = AUDIO_PLAY_OS_FAIL;
                        if(AUDIO_PLAY_OS_SUCCESS != audio_play_queue_is_empty(g_outside_v2_msg))
                        {
                            audio_play_queue_receive(g_outside_v2_msg,&rev_outside_msg,audio_play_os_unblock);
                        }

                        do
                        {
                            ret = audio_play_queue_peek(g_outside_v2_msg,&rev_outside_msg,pdMS_TO_TICKS(20));
                            msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                            if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(pause_data_event,pdMS_TO_TICKS(20)))
                            {                       
                                /* 收到数据获取暂停事件，保存数据读取进度信息 */
                                save_audio_info(&msg);

                                /* 进入停止状态，强制清除现有buf消息 */
                                get_data_state = GET_PLAY_DATA_STATE_STOP;
                                audio_play_queue_reset(data_buf_queue);
                                goto get_data_stop; //FIXME: 现在只能用goto解决这个问题了
                            }
                            if(AUDIO_PLAY_OS_SUCCESS != ret)
                            {
                                if(AUDIO_PLAY_OS_SUCCESS == audio_play_semaphore_take(g_outside_v2_end_sem,0))
                                {
                                    if(AUDIO_PLAY_OS_SUCCESS == audio_play_queue_is_empty(g_outside_v2_msg))
                                    {
                                        msg_data_flag = AUDIO_DATD_BUFF_END;
                                        break;
                                    }
                                    else
                                    {
                                        audio_play_semaphore_give(g_outside_v2_end_sem);
                                    }
                                    
                                }
                            }
                        }while(ret != AUDIO_PLAY_OS_SUCCESS);
                        buf_msg.data_addr = rev_outside_msg;   
                        msg_data_size = GET_PLAY_DATA_BUFF_SIZE;
                        break;
                    }
                    #endif
                }
                if(((GET_PLAY_DATA_BUFF_COUNT-1) == first_flag)||((read_size == data_size)&&((GET_PLAY_DATA_BUFF_COUNT-1) > first_flag)))
                {
                    //audio_play_task_suspend(audio_play_task_handle);
                    buf_msg.data_flag = AUDIO_DATD_BUFF_NORMAL;
                    for(int j = 0; j <= first_flag; j++)
                    {
                        if((msg_data_flag == AUDIO_DATD_BUFF_END)&&(j == first_flag))
                        {
                            buf_msg.data_flag = msg_data_flag;
                            buf_msg.data_size = msg_data_size;
                            msg_data_flag = AUDIO_DATD_BUFF_NORMAL;
                        }
                        #if AUDIO_PLAY_USE_OUTSIDE_V2
                        if(!force_data_addr)
                        {
                            buf_msg.data_addr = (uint32_t)(data_buf+j*GET_PLAY_DATA_BUFF_SIZE);
                        }
                        #else
                        buf_msg.data_addr = (uint32_t)(data_buf+j*GET_PLAY_DATA_BUFF_SIZE);
                        #endif
                        audio_play_queue_send(data_buf_queue,&buf_msg,0);
                        ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                    }
                    //audio_play_task_resume(audio_play_task_handle);
                }
                else if(GET_PLAY_DATA_BUFF_COUNT <= first_flag)
                {
                    buf_msg.data_flag = msg_data_flag;
                    buf_msg.data_size = msg_data_size;
                    #if AUDIO_PLAY_USE_OUTSIDE_V2
                    if(!force_data_addr)
                    {
                        buf_msg.data_addr = (uint32_t)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE);
                    }
                    #else
                    buf_msg.data_addr = (uint32_t)(data_buf+buf_index*GET_PLAY_DATA_BUFF_SIZE);
                    #endif
                    audio_play_queue_send(data_buf_queue,&buf_msg,0);
                    ci_logverbose(LOG_AUDIO_GET_DATA,"发送 wav_buf is 0x%x\n",buf_msg.data_addr);
                }
                first_flag ++;
                buf_index++;
                if(buf_index == GET_PLAY_DATA_BUFF_COUNT)
                {
                    buf_index = 0;
                }
                break;
            }
            case GET_PLAY_DATA_STATE_STOP:
            {
                /* 已经stop了，需要确认所有数据buf均被释放，如果这时候发现收到了暂停的请求则保存信息 */
                if(AUDIO_PLAY_OS_SUCCESS ==audio_play_semaphore_take(pause_data_event,pdMS_TO_TICKS(20)))
                {   
                    save_audio_info(&msg);
                }
                else if(AUDIO_PLAY_OS_SUCCESS != audio_play_queue_is_empty(data_buf_queue))
                {
                    continue;
                }
           get_data_stop:
                /* clean */
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf,0,sizeof(uint8_t)*GET_PLAY_DATA_BUFF_SIZE*GET_PLAY_DATA_BUFF_COUNT);
                audio_play_queue_reset(data_buf_queue);
                #if AUDIO_PLAY_USE_SD_CARD
                f_close(&fsrc);
                #endif
                #if AUDIO_PLAY_USE_OUTSIDE_V2
                force_data_addr = false;
                #endif
                buf_index = 0;
                first_flag = 0;
                data_size = 0;
                get_data_state = GET_PLAY_DATA_STATE_IDLE;
                break;
            }
        }
    }
}


#if AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
/**
 * @brief 读取唤醒词播报音频地址，存储在内存中
 * 
 * @param wakeup_voice_flash_addr 唤醒词播报音频在flash地址
 * 
 * @retval RETURN_OK 读取成功
 * @retval RETURN_ERR 读取失败
 */
int32_t read_weakup_voice_to_ram(uint32_t wakeup_voice_flash_addr)
{
    if(wakeup_voice_flash_addr != 0)
    {
        uint32_t wave_total_lens;
        uint32_t wave_head_size;
        if(wakeup_voice_buf != NULL)
        {
            vPortFree(wakeup_voice_buf);
            wakeup_voice_buf = NULL;
        }
        wakeup_voice_buf = (uint8_t *)pvPortMalloc(4096);
        if(wakeup_voice_buf == NULL)
        {
            CI_ASSERT(0,"malloc mem err\n");
        }
        //post_read_flash((char *)(wakeup_voice_buf), wakeup_voice_flash_addr, 4096);
		post_read_flash((char *)(wakeup_voice_buf), wakeup_voice_flash_addr, 4096);
        if(-1 == check_ci_voice_head((ci_voice_head_t *)wakeup_voice_buf,&wave_total_lens,&wave_head_size,&wakeup_voice_config,&wakeup_voice_threshold_data_size))
        {
            CI_ASSERT(0,"wakeup_voice_flash_addr err\n");
        }
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(wakeup_voice_buf,0x0,wave_head_size);
        if(wave_total_lens +wave_head_size > 4096)
        {
            wakeup_voice_buf = pvPortRealloc(wakeup_voice_buf,wave_total_lens +wave_head_size);
            if(wakeup_voice_buf == NULL)
            {
                CI_ASSERT(0,"malloc mem err\n");
            }
            //post_read_flash((char *)(wakeup_voice_buf+4096), wakeup_voice_flash_addr+4096, wave_total_lens+wave_head_size-4096);
			post_read_flash((char *)(wakeup_voice_buf+4096), wakeup_voice_flash_addr+4096, wave_total_lens+wave_head_size-4096);
        }
        else
        {
            switch(wakeup_voice_config.mode)
            {
                case CI_ADPCM_DECODER_MODE_MP3:
                case CI_ADPCM_DECODER_MODE_FLAC:
                {
                    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(data_buf+wave_total_lens+wave_head_size,0x0,4096-wave_total_lens-wave_head_size);
                    break;
                }
            }
        }
        wakeup_voice_size = wave_total_lens+wave_head_size;
        wakeup_voice_addr = wakeup_voice_flash_addr;
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}
#endif


/**
 * @brief 接收数据buf地址
 * 
 * @param addr buf地址
 * @param size 数据size
 * @param flag buf标志
 * 
 * @retval RETURN_OK 接收成功
 * @retval RETURN_ERR 接收失败
 */
int32_t receive_data(uint32_t *addr,uint32_t *size,uint32_t *flag)
{ 
    audio_play_os_state_t ret = AUDIO_PLAY_OS_FAIL;
    data_buf_msg_t buf_msg;

    ret = audio_play_queue_peek(data_buf_queue,&buf_msg,audio_play_os_unblock);
    if(ret == AUDIO_PLAY_OS_SUCCESS)
    {
        ci_logverbose(LOG_AUDIO_GET_DATA,"接收 wav_buf is 0x%x\n",buf_msg.data_addr);
        *addr = buf_msg.data_addr;
        *size = buf_msg.data_size;
        *flag = buf_msg.data_flag;
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
    
}


/**
 * @brief 释放数据buf
 * 
 */
void finsh_read_data(void)
{
    data_buf_msg_t buf_msg;

    /* 使用peek接受，使用received释放 */
    audio_play_queue_receive(data_buf_queue,&buf_msg,audio_play_os_block);
    ci_logverbose(LOG_AUDIO_GET_DATA,"释放 wav_buf is 0x%x\n",buf_msg.data_addr);
}


/**
 * @brief 请求暂停数据并保存记录
 * 
 * @retval RETURN_OK 数据有效
 * @retval RETURN_ERR 数据无效
 */
int32_t pause_read_data(void)
{
    int timeout = WAIT_TIMEOUT;

    audio_play_semaphore_give(pause_data_event);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            ci_logerr(LOG_AUDIO_GET_DATA,"audio player pause_read_data timeout!\n");   
            return RETURN_ERR;
        }
    }
    return RETURN_OK;
}


/**
 * @brief 历史数据有效性检查
 * 
 * @retval RETURN_OK 数据有效
 * @retval RETURN_ERR 数据无效
 */
int32_t continue_data_valid(void)
{
    if(data_list.type != GET_PLAY_DATA_NONE)
    {
        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }
}


/**
 * @brief 返回本模块任务状态
 * 
 * @return get_play_data_state_t 数据读入任务状态
 */
get_play_data_state_t get_data_task_state(void)
{
    return get_data_state;
}


#if AUDIO_PLAY_USE_NET
/**
 * @brief 返回网络数据下载offset
 * 
 * @return uint32_t 网络数据下载进度
 */
uint32_t get_net_download_offset(void)
{
    return s_len;
}
#endif /* AUDIO_PLAY_USE_NET */


/**
 * @brief 返回播放offset
 * 
 * @return uint32_t 播放进度
 */
uint32_t get_curr_offset(void)
{
    return curr_offset + get_decode_sync_word_offset();
}


#if AUDIO_PLAY_USE_OUTSIDE
/**
 * @brief 清理缓冲流
 * 
 * @param outside_stream 数据流描述符
 * @param outside_end_sem 数据结束信号描述符
 */
void outside_clear_stream(audio_play_os_stream_t outside_stream,audio_play_os_sem_t outside_end_sem)
{
    if(outside_stream != NULL)
    {
        audio_play_stream_buffer_reset(outside_stream);
    }
    if(outside_end_sem != NULL)
    {
        audio_play_queue_reset((audio_play_os_queue_t)outside_end_sem);
    }
}

/**
 * @brief 发送结束数据信号
 * 
 * @param outside_end_sem 数据流结束信号描述符
 */
void outside_send_end_sem(audio_play_os_sem_t outside_end_sem)
{
    audio_play_task_delay(20);
    audio_play_semaphore_give(outside_end_sem);
}


/**
 * @brief 向数据流写入数据
 * 
 * @param outside_stream 数据流描述符
 * @param addr 待写入数据地址
 * @param size 待写入数据大小
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t outside_write_stream(audio_play_os_stream_t outside_stream, uint32_t addr, uint32_t size, bool is_block)
{
    int sendbyte = 0;
    int xBytesSent;

    if(is_block || (audio_play_stream_buffer_get_spaces_size(outside_stream) >= size))
    {
        do{
            xBytesSent = audio_play_stream_buffer_send(outside_stream,(int *)(addr+sendbyte),size-sendbyte,audio_play_os_block);
            sendbyte += xBytesSent;
        }while(sendbyte < size);

        return RETURN_OK;
    }
    else
    {
        return RETURN_ERR;
    }

}


/**
 * @brief 初始化创建数据流、结束信号
 * 
 * @param outside_stream 数据流描述符
 * @param outside_end_sem 数据流结束信号描述符
 * @param full_size 数据缓冲流总大小
 */
void outside_init_stream(audio_play_os_stream_t *outside_stream, audio_play_os_sem_t *outside_end_sem, uint32_t full_size)
{
    *outside_stream = audio_play_stream_buffer_create(full_size,4096);
    if(NULL == *outside_stream)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"audio_play_stream_buffer_create() failed\n");
    }
    *outside_end_sem = audio_play_semaphore_create_binary();
    if(NULL == *outside_end_sem)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"outside_end_sem creat fail\r\n");   
    }
}

/**
 * @brief 销毁数据流、结束信号
 * 
 * @param outside_stream 数据流描述符
 * @param outside_end_sem 数据流结束信号描述符
 */
void outside_destroy_stream(audio_play_os_stream_t outside_stream, audio_play_os_sem_t outside_end_sem)
{
    audio_play_stream_buffer_delete(outside_stream);
    audio_play_semaphore_delete(outside_end_sem);
}

/**
 * @brief 设置当前使用的数据流、结束信号描述符
 * 
 * @param outside_stream 数据流描述符
 * @param outside_end_sem 数据流结束信号描述符
 */
void set_curr_outside_handle(audio_play_os_stream_t outside_stream, audio_play_os_sem_t outside_end_sem)
{
    g_outside_stream = outside_stream;
    g_outside_end_sem = outside_end_sem;
}
#endif /* AUDIO_PLAY_USE_OUTSIDE */


#if AUDIO_PLAY_USE_OUTSIDE_V2
/**
 * @brief 清理消息
 * 
 * @param outside_msg 数据消息描述符
 * @param outside_end_sem 数据结束信号描述符
 */
void outside_v2_clear_stream(audio_play_os_queue_t outside_msg,audio_play_os_sem_t outside_end_sem)
{
    if(outside_msg != NULL)
    {
        audio_play_queue_reset(outside_msg);
    }
    if(outside_end_sem != NULL)
    {
        audio_play_queue_reset((audio_play_os_queue_t)outside_end_sem);
    }
}

/**
 * @brief 发送结束数据信号
 * 
 * @param outside_end_sem 数据流结束信号描述符
 */
void outside_v2_send_end_sem(audio_play_os_sem_t outside_end_sem)
{
    audio_play_task_delay(20);
    audio_play_semaphore_give(outside_end_sem);
}


/**
 * @brief 向消息写入数据buff地址
 * 
 * @param outside_msg 数据流描述符
 * @param addr 待写入数据地址
 * @param is_block 是否阻塞写入
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t outside_v2_write_msg(audio_play_os_queue_t outside_msg, uint32_t addr, bool is_block)
{
    uint32_t msg = addr;
    if(is_block)
    {
        audio_play_queue_send(outside_msg,&msg,audio_play_os_block);
    }
    else
    {
        if(AUDIO_PLAY_OS_SUCCESS != audio_play_queue_send(outside_msg,&msg,audio_play_os_unblock))
        {
            return RETURN_ERR;
        }
    }
    return RETURN_OK;
}


/**
 * @brief 初始化创建消息、结束信号
 * 
 * @param outside_msg 数据buf地址消息描述符
 * @param outside_end_sem 数据流结束信号描述符
 * @param msg_count 数据buf消息个数
 */
void outside_v2_init_msg(audio_play_os_queue_t *outside_msg, audio_play_os_sem_t *outside_end_sem, uint32_t msg_count)
{
    *outside_msg = audio_play_queue_create(msg_count, sizeof(uint32_t));
    if(NULL == *outside_msg)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"outside_msg creat fail\r\n");   
    }
    *outside_end_sem = audio_play_semaphore_create_binary();
    if(NULL == *outside_end_sem)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"outside_end_sem creat fail\r\n");   
    }
}


void outside_v2_destroy_stream(audio_play_os_queue_t outside_msg, audio_play_os_sem_t outside_end_sem)
{
    audio_play_queue_delete(outside_msg);
    audio_play_semaphore_delete(outside_end_sem);
}


void set_curr_outside_v2_handle(audio_play_os_queue_t outside_msg, audio_play_os_sem_t outside_end_sem)
{
    g_outside_v2_msg = outside_msg;
    g_outside_v2_end_sem = outside_end_sem;
}
#endif


/**
 * @brief 获取audio数据任务预初始化
 * @note 请在OS运行前调用
 * 
 */
void task_get_play_data_init(void)
{
    data_buf = (uint8_t *)audio_play_malloc(sizeof(uint8_t)*GET_PLAY_DATA_BUFF_SIZE*GET_PLAY_DATA_BUFF_COUNT);
    if(NULL == data_buf)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"malloc err!\n");
    }

    get_play_data_queue = audio_play_queue_create(GET_PLAY_DATA_QUEUE_COUNT, sizeof(get_play_data_msg_t));
    if(NULL == get_play_data_queue)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"get_play_data_queue creat fail\r\n");   
    }

    data_buf_queue = audio_play_queue_create(GET_PLAY_DATA_BUFF_COUNT, sizeof(data_buf_msg_t));
    if(NULL == data_buf_queue)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"data_buf_queue creat fail\r\n");   
    }

    pause_data_event = audio_play_semaphore_create_binary();
    if(NULL == pause_data_event)
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"download_end_sem creat fail\r\n");   
    }
}


/**
 * @brief 请求获取flash音频数据
 * 
 * @param arg flash音频地址或地址列表指针
 * @param num 音频个数大小
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t get_flash_data(uint32_t arg, uint32_t num)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    if(num > 1)
    {
        msg.type = GET_PLAY_DATA_QSPI_FLASH_LIST;
        msg.spi_flash_msg.data_addr_list = arg;
        msg.spi_flash_msg.num = num;
    }
    else
    {
        msg.type = GET_PLAY_DATA_QSPI_FLASH;
        msg.spi_flash_msg.data_addr = arg;
    }
    msg.offset = 0;
    msg.flag = NEW_DATA;
    
    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}

#if AUDIO_PLAY_USE_SD_CARD
/**
 * @brief 请求获取sd卡音频数据
 * 
 * @param dir 音频文件地址
 * @param name 音频文件名称
 * @param offset 播放偏移
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t get_sd_data(const char *dir,const char *name,uint32_t offset)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    msg.type = GET_PLAY_DATA_SD_CARD;
    msg.sd_card_msg.dir = dir;
    msg.sd_card_msg.name = name;
    msg.offset = offset;
    msg.flag = NEW_DATA;
    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif


#if AUDIO_PLAY_USE_NET
/**
 * @brief 请求获取网络音频数据
 * 
 * @param url 音频源地址
 * @param offset 播放偏移
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t get_net_data(const char *url, uint32_t offset)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    if(net_client_is_ok == 0)
    {
        task_get_play_data_net_init();
        net_client_is_ok = 1;
    }

    msg.type = GET_PLAY_DATA_NET;
    msg.net_msg.url = url;
    msg.offset = offset;
    msg.flag = NEW_DATA;
    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif /* AUDIO_PLAY_USE_NET */


#if AUDIO_PLAY_USE_OUTSIDE
/**
 * @brief 请求获取外部音频数据
 * 
 * @param offset 播放偏移
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t get_outside_data(uint32_t offset)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    msg.type = GET_PLAY_DATA_OUTSIDE;
    msg.flag = NEW_DATA;
    msg.offset = offset;

    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif


#if AUDIO_PLAY_USE_OUTSIDE_V2
/**
 * @brief 请求获取外部音频数据
 * 
 * @param offset 播放偏移
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t get_outside_v2_data(uint32_t offset)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    msg.type = GET_PLAY_DATA_OUTSIDE_V2;
    msg.flag = NEW_DATA;
    msg.offset = offset;

    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif


/**
 * @brief 请求获取被打断的数据
 * 
 * @retval RETURN_OK 请求成功
 * @retval RETURN_ERR 请求失败
 */
int32_t continue_read_data(void)
{
    get_play_data_msg_t msg;
    int timeout = WAIT_TIMEOUT;

    msg.flag = CONTINUE_DATA;
    
    audio_play_queue_reset(data_buf_queue);
    while(get_data_state != GET_PLAY_DATA_STATE_IDLE)
    {
        audio_play_task_delay(pdMS_TO_TICKS(2));
        timeout--;
        if(timeout == 0)
        {
            return RETURN_ERR;
        }
    }
    audio_play_queue_reset((audio_play_os_queue_t)pause_data_event);
    
    if(AUDIO_PLAY_OS_FAIL == audio_play_queue_send(get_play_data_queue, &msg,0))
    {
        ci_logerr(LOG_AUDIO_GET_DATA,"send get_play_data_queue err\r\n");
        return RETURN_ERR;
    }
    return RETURN_OK;
}
