/**
 * @file audio_play_decoder.c
 * @brief  播放器音频解码器框架
 * @version 1.0
 * @date 2019-04-08
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */


#include "audio_play_decoder.h"
#include "ci130x_system.h"
#include "ci_log.h"
#include "audio_play_os_port.h"
#include "audio_play_config.h"
// #include "flash_rw_process.h"

// #include "mp3dec.h"
// #include "mp3common.h"

#include "romlib_runtime.h"

#define MP3_IN_SIZE   (576)

/* mp3解码器描述符 */
static HMP3Decoder mp3_dec_handle;
/* mp3帧信息 */
static MP3FrameInfo mp3FrameInfo; 
/* mp3现场保存描述符 */
#if AUDIO_PLAY_FIX_MP3_DECODER_ISSUE
static MP3SaveLastInfo save_mp3_dec_handle;
#endif

#if AUDIO_PLAYER_CONFIG_MP3_USE_OUTSIDE_MP3TABLE
static void *mp3tabBuff00 = NULL;
static void *mp3tabBuff01 = NULL;
static void *mp3tabBuff02 = NULL;
static void *mp3tabBuff1 = NULL;
static void *mp3tabBuff2 = NULL;
#endif
#include "ci_flash_data_info.h"

/**
 * @brief 解码器初始化
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
int32_t audio_play_mp3_init(void)
{
    #if AUDIO_PLAYER_CONFIG_MP3_USE_OUTSIDE_MP3TABLE
    if(mp3tabBuff00 == NULL)
    {
        static uint32_t mp3tabaddr1 = 0;
        static uint32_t mp3tabsize1= 0;

        if(mp3tabaddr1 == 0) {
			if(get_userfile_addr(60001, &mp3tabaddr1, &mp3tabsize1) != 0)
			{
				ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
				return RETURN_ERR;
			}
        }
        mp3tabBuff00 = malloc(3618);
        if(0 == mp3tabBuff00 )
		{
			ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
			return RETURN_ERR;
		}
        //post_read_flash((char *)(mp3tabBuff00), mp3tabaddr1, 3618);
		post_read_flash((char *)(mp3tabBuff00), mp3tabaddr1, 3618);

        mp3tabBuff01 = malloc(3456);
        if(0 == mp3tabBuff01 )
		{
			ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
			return RETURN_ERR;
		}
        //post_read_flash((char *)(mp3tabBuff01), mp3tabaddr1+3618, 3456);
        post_read_flash((char *)(mp3tabBuff01), mp3tabaddr1+3618, 3456);
		
        mp3tabBuff02 = malloc(1890);
        if(0 == mp3tabBuff02 )
		{
			ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
			return RETURN_ERR;
		}
        //post_read_flash((char *)(mp3tabBuff02), mp3tabaddr1+3618+3456, 1890);
		post_read_flash((char *)(mp3tabBuff02), mp3tabaddr1+3618+3456, 1890);
        config_mp3hufftabs_point(mp3tabBuff00,mp3tabBuff01,mp3tabBuff02);
    }
    if(mp3tabBuff1 == NULL)
    {
        static uint32_t mp3tabaddr2 = 0;
        static uint32_t mp3tabsize2= 0;
        if(mp3tabaddr2 == 0)
        {
			if(get_userfile_addr(60003, &mp3tabaddr2, &mp3tabsize2) != 0)
			{
				ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
				return RETURN_ERR;
			}
        }
        mp3tabBuff1 = malloc(2156);
        if(0 == mp3tabBuff1 )
        {
            ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
            return RETURN_ERR;
        }   
        //post_read_flash((char *)(mp3tabBuff1), mp3tabaddr2, 2156);
		post_read_flash((char *)(mp3tabBuff1), mp3tabaddr2, 2156);
        config_mp3trigtabs_point(mp3tabBuff1);
    }
    if(mp3tabBuff2 == NULL)
    {
    	static uint32_t mp3tabaddr3 = 0;
    	static uint32_t mp3tabsize3= 0;
    	if(mp3tabaddr3 == 0)
    	{
			if(get_userfile_addr(60002, &mp3tabaddr3, &mp3tabsize3) != 0)
			{
				ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
				return RETURN_ERR;
			}
    	}
        mp3tabBuff2 = malloc(1278);
        if(0 == mp3tabBuff2 )
        {
            ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
            return RETURN_ERR;
        }   
        //post_read_flash((char *)(mp3tabBuff2), mp3tabaddr3, 1278);
		post_read_flash((char *)(mp3tabBuff2), mp3tabaddr3, 1278);
        config_mp3tab_point(mp3tabBuff2);
    }
    #endif

    if(mp3_dec_handle == NULL)
    {
        mp3_dec_handle = MASK_ROM_LIB_FUNC->mp3func.MP3InitDecoder_p();	
        if(0 == mp3_dec_handle )
        {
            ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
            return RETURN_ERR;
        }   
    }
    
    #if AUDIO_PLAY_FIX_MP3_DECODER_ISSUE
    if(save_mp3_dec_handle == NULL)
    {
        save_mp3_dec_handle = (MP3SaveLastInfo)audio_play_malloc(MP3GetSaveLastInfoStructSize());
        if(NULL == save_mp3_dec_handle)
        {
            ci_logerr(LOG_AUDIO_PLAY,"mp3 buf malloc failed!\n");
            return RETURN_ERR;
        }
    }
    #endif
    
    return 0;
}

/**
 * @brief mp3解码器销毁
 * 
 * @retval RETURN_OK 销毁成功
 * @retval RETURN_ERR 销毁失败
 */
int32_t audio_play_mp3_destroy(void)
{
    if(mp3_dec_handle != NULL)
    {
        MASK_ROM_LIB_FUNC->mp3func.MP3FreeDecoder_p(mp3_dec_handle);
        mp3_dec_handle = NULL;
    }
    //#if AUDIO_PLAY_FIX_MP3_DECODER_ISSUE
    //audio_play_free(save_mp3_dec_handle);
    //#endif


    #if AUDIO_PLAYER_CONFIG_MP3_USE_OUTSIDE_MP3TABLE
    if(mp3tabBuff00 != NULL)
    {
        free(mp3tabBuff00);
        mp3tabBuff00 = NULL;
    }
    if(mp3tabBuff01 != NULL)
    {
        free(mp3tabBuff01);
        mp3tabBuff01 = NULL;
    }
    if(mp3tabBuff02 != NULL)
    {
        free(mp3tabBuff02);
        mp3tabBuff02 = NULL;
    }
    if(mp3tabBuff1 != NULL)
    {
        free(mp3tabBuff1);
        mp3tabBuff1 = NULL;
    }
    if(mp3tabBuff2 != NULL)
    {
        free(mp3tabBuff2);
        mp3tabBuff2 = NULL;
    }
    #endif

    return 0;
}

/**
 * @brief mp3解码器复位
 * 
 * @retval RETURN_OK 复位成功
 * @retval RETURN_ERR 复位失败
 */
int32_t audio_play_mp3_clear(void)
{
    MASK_ROM_LIB_FUNC->mp3func.MP3ClearLastDecodeMainData_p(mp3_dec_handle);
    return 0;
}

#if AUDIO_PLAY_FIX_MP3_DECODER_ISSUE
/**
 * @brief mp3解码器现场保存
 * 
 * @retval RETURN_OK 保存成功
 * @retval RETURN_ERR 保存失败
 */
int32_t audio_play_mp3_save(void)
{
    MASK_ROM_LIB_FUNC->mp3func.MP3SaveLastDecodeMainData_p(mp3_dec_handle,save_mp3_dec_handle);
    return 0;
}


/**
 * @brief mp3解码器现场恢复
 * 
 * @retval RETURN_OK 恢复成功
 * @retval RETURN_ERR 恢复失败
 */
int32_t audio_play_mp3_load(void)
{
    MASK_ROM_LIB_FUNC->mp3func.MP3LoadLastDecodeMainData_p(mp3_dec_handle,save_mp3_dec_handle);
    return 0;
}
#endif


/**
 * @brief mp3帧解码
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_mp3_decode(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    int32_t err;
    int32_t bytesLeft = (int32_t)in_size;
    uint8_t *readPtr = input;
    int32_t mp3_sync_offset;
    
    audio_format_info->out_min_size = 576;
    /* 寻找帧头 */
    mp3_sync_offset=MASK_ROM_LIB_FUNC->mp3func.MP3FindSyncWord_p(readPtr, bytesLeft);

    if(-1 == mp3_sync_offset)
    {
        /*sync not found*/
        /*mp3 sync byte 2 byte,just keep 2 byte for next*/
        *no_use_size = 2;
        ci_logdebug(LOG_AUDIO_PLAY,"res = %d!\n",mp3_sync_offset);
        return -1;
    }
    else
    {
        /*sync found*/
        /* 偏移至帧头 */
        readPtr   += mp3_sync_offset;  
        bytesLeft -= mp3_sync_offset; 
        
        /* 解码 */
        err = MASK_ROM_LIB_FUNC->mp3func.MP3Decode_p(mp3_dec_handle,&readPtr,&bytesLeft,output,0);
        /* 取的帧信息 */
        MASK_ROM_LIB_FUNC->mp3func.MP3GetLastFrameInfo_p(mp3_dec_handle, &mp3FrameInfo);       
        if(ERR_MP3_NONE != err )
        {   
            ci_logwarn(LOG_AUDIO_PLAY,"mp3_decord err %d,bad frame!,remove 2byte,detect next\r\n",err);
            *no_use_size = bytesLeft-2;   
            return -1;
        }
        else
        {
            if(MPEG1 == mp3FrameInfo.version)
            {
                /* 配置采样率 */
                audio_format_info->samprate = mp3FrameInfo.samprate;
                if((mp3FrameInfo.samprate > 96000)||(mp3FrameInfo.samprate < 8000))
                {
                    ci_logwarn(LOG_AUDIO_PLAY,"do not support samprate is %d!\n",mp3FrameInfo.samprate);
                    *no_use_size = bytesLeft-2;
                    return -1;
                }

                /* 单声道处理 */
                audio_format_info->nChans = mp3FrameInfo.nChans;
                if(1 == mp3FrameInfo.nChans)
                {
                    /* MPEG1 单声道一帧是1152-short,get_next_pcm_buff(2) */
                    *out_size = 1152*sizeof(short);
                }
                else
                {
                    /* MPEG1 双声道一帧是2304-short,get_next_pcm_buff(4) */
                    *out_size = 2304*sizeof(short);
                }
        
            }
            else if(MPEG2 == mp3FrameInfo.version || MPEG25 == mp3FrameInfo.version )
            { 
                /* 配置采样率 */
                audio_format_info->samprate = mp3FrameInfo.samprate;
                if((mp3FrameInfo.samprate > 96000)||(mp3FrameInfo.samprate < 8000))
                {
                    ci_logwarn(LOG_AUDIO_PLAY,"do not support samprate is %d!\n",mp3FrameInfo.samprate);
                    *no_use_size = bytesLeft-2;
                    return -1;
                }

                /* 单声道处理 */
                audio_format_info->nChans = mp3FrameInfo.nChans;
                if(1 == mp3FrameInfo.nChans)
                {
                    /* MPEG2 MPEG25 单声道一帧是576-short,get_next_pcm_buff(1) */
                    *out_size = 576*sizeof(short);
                }
                else
                {
                    /* MPEG2 MPEG25 双声道一帧是1152-short,get_next_pcm_buff(2) */
                    *out_size = 1152*sizeof(short);
                }
            }

            *no_use_size = bytesLeft;
            return 0;
        }  
    }
}


/**
 * @brief mp3解码器结构对象
 * 
 */
audio_play_decoder_ops_t mp3_decoder =
{
    .decoder_name = "mp3",
    .init = audio_play_mp3_init,
    .decode = audio_play_mp3_decode,
    #if AUDIO_PLAY_FIX_MP3_DECODER_ISSUE
    .save = audio_play_mp3_save,
    .load = audio_play_mp3_load,
    #else
    .load = NULL,
    .save = NULL,
    #endif
    .config = NULL,
    .clear = audio_play_mp3_clear,
    .destroy = audio_play_mp3_destroy,
    .data_in_size = MP3_IN_SIZE,
    .next_ops = NULL,
};


#include "flacdecoder.h"
#if 1
#include <stdlib.h>
#define FLAC_MALLOC malloc
#define FLAC_FREE free
#else
#include "freertos/FreeRTOS.h"
#define FLAC_MALLOC pvPortMalloc
#define FLAC_FREE   vPortFree
#endif

#define FLAC_IN_SIZE (4000)

FLACContext *fc = NULL; 

/**
 * @brief flac解码器初始化
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
int32_t audio_play_flac_init(void)
{
    if(fc == NULL)
    {
        fc = (FLACContext *)FLAC_MALLOC(sizeof(FLACContext)); 
        if(NULL == fc)
        {
            ci_logerr(LOG_AUDIO_PLAY,"flac decoder malloc failed!\n");
            return RETURN_ERR;
        }
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(fc,0,sizeof(FLACContext));//fc所有内容清零
    }

    return RETURN_OK;
}

/**
 * @brief flac解码器销毁
 * 
 * @retval RETURN_OK 解码器销毁成功
 * @retval RETURN_ERR 解码器销毁失败
 */
int32_t audio_play_flac_destroy(void)
{
    if(fc != NULL)
    {
        FLAC_FREE(fc);
        fc = NULL;
    }
    return RETURN_OK;
}

/**
 * @brief FLAC帧解码
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_flac_decode(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    int32_t err;
    int32_t bytesLeft = (int32_t)in_size;
    uint8_t *readPtr = input;
    int32_t flac_sync_offset = 0;

    audio_format_info->out_min_size = fc->min_blocksize;
    audio_format_info->samprate = fc->samplerate;
    audio_format_info->nChans = fc->channels;

    flac_sync_offset = flac_seek_frame(readPtr,bytesLeft,fc);
    if(flac_sync_offset < 0)
    {
        *no_use_size = 4;
        ci_logdebug(LOG_AUDIO_PLAY,"res = %d!\n",flac_sync_offset);
        return -1;
    }
    else
	{
        readPtr += flac_sync_offset;
        bytesLeft -= flac_sync_offset;
        err = flac_decode_frame16(fc,readPtr,bytesLeft,output);
        if(err!=0)
        {
            ci_logwarn(LOG_AUDIO_PLAY,"flac_decord err %d,bad frame!,remove 4byte,detect next\r\n",err);
            *no_use_size = bytesLeft-4;   
            return -1;
        }
        else
        {
            if(fc->channels == 1)
            {
                *out_size = fc->max_blocksize*2;
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(output+fc->blocksize,0x0,(fc->max_blocksize-fc->blocksize)*2);
            }
            else
            {
                *out_size = fc->max_blocksize*4;
                MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(output+fc->blocksize*2,0x0,(fc->max_blocksize-fc->blocksize)*4);
            }
   
            //TODO:需要处理这种现象
            CI_ASSERT(bytesLeft >= fc->gb.index/8, "flac decoder err!\n");
            
            *no_use_size = bytesLeft - fc->gb.index/8;
            return 0;
        }  
    }
}


/**
 * @brief 配置解码器模式
 * 
 * @param mode 模式参数
 * @retval RETURN_OK 配置正确
 * @retval RETURN_ERR 配置异常
 */
int32_t audio_play_flac_config( void *mode)
{
    flac_file_info_t *flac_file_info = (flac_file_info_t *)mode;

    if(flac_file_info != NULL)
    {
        fc->min_blocksize = flac_file_info->min_blocksize;
        fc->max_blocksize = flac_file_info->max_blocksize;
        fc->min_framesize = flac_file_info->min_framesize;
        fc->max_framesize = flac_file_info->max_framesize;
        fc->samplerate = flac_file_info->samplerate;
        fc->channels = flac_file_info->channels;
        fc->bps = flac_file_info->bps;
        fc->totalsamples = flac_file_info->totalsamples;
        
        fc->decoded0=FLAC_MALLOC(flac_file_info->max_blocksize*4);
        CI_ASSERT(fc->decoded0 != NULL , "malloc" );
        if(flac_file_info->channels == 2)
        {
            fc->decoded1=FLAC_MALLOC(flac_file_info->max_blocksize*4);
            CI_ASSERT(fc->decoded1 != NULL , "malloc" );
        }
    }
    return RETURN_OK;
}


int32_t audio_play_flac_clear(void)
{
    if(fc->decoded0 != NULL)
    {
        FLAC_FREE(fc->decoded0);
        fc->decoded0 = NULL;
    }
    if(fc->decoded1 != NULL)
    {
        FLAC_FREE(fc->decoded1);
        fc->decoded1 = NULL;
    }
    return RETURN_OK;
}


/**
 * @brief flac解码器结构对象
 * 
 */
audio_play_decoder_ops_t flac_decoder =
{
    .decoder_name = "flac",
    .init = audio_play_flac_init,
    .decode = audio_play_flac_decode,
    .save = NULL,
    .load = NULL,
    .config = audio_play_flac_config,
    .clear = audio_play_flac_clear,
    .destroy = audio_play_flac_destroy,
    .data_in_size = FLAC_IN_SIZE,
    .next_ops = NULL,
};


#define MS_WAV_IN_SIZE (1152)

static uint32_t wav_nSamplesPerSec = 16000;

/**
 * @brief MS WAV帧解码
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_ms_wav_decode(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    uint32_t relly_size = *no_use_size;
    
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(output,input,relly_size);
    #if !AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
    if(MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char *)input,"RIFF",4) == 0)
    {
        if(MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char *)(input+0x24),"data",4) == 0)
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(output,0x00,44);
        }
    }
    #endif
    *out_size = relly_size;
    *no_use_size = 0;
    if(*out_size < in_size)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(output+(*out_size)/sizeof(short),0x0,MS_WAV_IN_SIZE-(*out_size));
        *out_size = in_size;
    }
    audio_format_info->samprate = wav_nSamplesPerSec;
    audio_format_info->nChans = 1;
    audio_format_info->out_min_size = 576;

    return 0;
}


/**
 * @brief ms_wav解码器复位
 * 
 * @retval RETURN_OK 复位成功
 * @retval RETURN_ERR 复位失败
 */
int32_t audio_play_ms_wav_clear(void)
{
    wav_nSamplesPerSec = 16000;
    return 0;
}

/**
 * @brief MS WAV解码器结构对象
 * 
 */
audio_play_decoder_ops_t ms_wav_decoder =
{
    .decoder_name = "ms_wav",
    .init = NULL,
    .decode = audio_play_ms_wav_decode,
    .save = NULL,
    .load = NULL,
    .config = NULL,
    .clear = audio_play_ms_wav_clear,
    .destroy = NULL,
    .data_in_size = MS_WAV_IN_SIZE,
    .next_ops = NULL,
};


#include "adpcmdec.h"
#include "adpcm.h"

#define ADPCM_IN_SIZE (576)
#define IMAADPCM_IN_SIZE (1138) //1138=67*17-1 
#define PROMPT_MP3_IN_SIZE (324)//(1152)
#define PROMPT_FLAC_IN_SIZE (2304)

static uint32_t adpcmBlockAlign = 0;
static uint32_t adpcm_head_size = 0;
static uint32_t adpcm_nSamplesPerSec = 16000;

/**
 * @brief adpcm帧解码(用于CI旧版adpcm)
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_adpcm_decode(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    uint32_t relly_size = *no_use_size;
    
    de_codeadpcm_mono_to_mono((char*)input,relly_size,(short*)output);
    *out_size = relly_size*4;
    *no_use_size = 0;
    if(*out_size < in_size*4)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(output+(*out_size)/sizeof(short),0x0,ADPCM_IN_SIZE*4-(*out_size));
        *out_size = in_size*4;
    }
    audio_format_info->samprate = 16000;
    audio_format_info->nChans = 1;
    audio_format_info->out_min_size = ADPCM_IN_SIZE;

    return 0;
}


/**
 * @brief adpcm帧解码2(用于标准adpcm文件)
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_adpcm_decode2(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    uint32_t relly_size = *no_use_size;
    uint32_t output_block_size = 4*(adpcmBlockAlign-4)+2;
      
    if(adpcm_head_size != 0)
    {
        input += adpcm_head_size;
        relly_size -= adpcm_head_size;
        adpcm_head_size = 0;
    }

    audio_format_info->samprate = adpcm_nSamplesPerSec;
    audio_format_info->nChans = 1;
    /* 这里*8的原因是ci130x系列iisdma传输是burst 16，而adpcm输出size一定是奇数*2 */
    audio_format_info->out_min_size = 8*output_block_size;

    int block_num = relly_size / adpcmBlockAlign;
    int temp_num = IMAADPCM_IN_SIZE/adpcmBlockAlign;
    if(block_num != 0)
    {
        for(int i = 0; i < block_num; i++)
        {
            *output = *(short *)input;
            set_adpcm_decode_index((int16_t)(*(char*)(input+2)));
            set_adpcm_decode_predsample((int32_t)(*output));

            de_codeadpcm_mono_to_mono((char*)(input+4),adpcmBlockAlign-4,(short*)(output+1));
            
            input += adpcmBlockAlign;
            output += 2*(adpcmBlockAlign-4)+1;
        }
    }

    if(block_num != temp_num)
    {
        relly_size -= block_num*adpcmBlockAlign;
        if(relly_size > 0)
        {
            *output = *(short *)input;
            set_adpcm_decode_index((int16_t)(*(char*)(input+2)));
            set_adpcm_decode_predsample((int32_t)(*output));
            
            de_codeadpcm_mono_to_mono((char*)(input+4),relly_size-4,(short*)(output+1));

            MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void *)(output+2*(relly_size-4)+1),0x0,4*(adpcmBlockAlign-relly_size)+2*(2*(adpcmBlockAlign-4)+1)*(temp_num-block_num-1));
        }
        else
        {
            MASK_ROM_LIB_FUNC->newlibcfunc.memset_p((void *)(output),0x0,2*(2*(adpcmBlockAlign-4)+1)*(temp_num-block_num));
        }
        *out_size = temp_num*output_block_size;
        *no_use_size = 0;
    }
    else
    {
        *no_use_size = relly_size - block_num*adpcmBlockAlign;
        *out_size = block_num*output_block_size;
    }
    
    return 0;
}


/**
 * @brief 配置解码器模式
 * 
 * @param mode 模式参数
 * @retval RETURN_OK 配置正确
 * @retval RETURN_ERR 配置异常
 */
int32_t audio_play_prompt_config( void *mode)
{
    prompt_decoder_config *config = (prompt_decoder_config *)mode;

    adpcm_head_size = config->wav.head_size;

    if(config->mode == CI_ADPCM_DECODER_MODE_CIADPCM)
    {
        prompt_decoder.decode = audio_play_adpcm_decode;
        prompt_decoder.data_in_size = ADPCM_IN_SIZE;
        adpcmBlockAlign = 0;
    }
    else if(config->mode == CI_ADPCM_DECODER_MODE_IMAADPCM)
    {
        prompt_decoder.decode = audio_play_adpcm_decode2;
        adpcm_nSamplesPerSec = config->wav.nSamplesPerSec;
        adpcmBlockAlign = config->wav.nBlockAlign;
        prompt_decoder.data_in_size = IMAADPCM_IN_SIZE;
    }
    else if(config->mode == CI_ADPCM_DECODER_MODE_WAV)
    {
        prompt_decoder.decode = audio_play_ms_wav_decode;
        wav_nSamplesPerSec = config->wav.nSamplesPerSec;
        prompt_decoder.data_in_size = MS_WAV_IN_SIZE;
    }
    #if AUDIO_PLAY_SUPPT_MP3_PROMPT
    else if(config->mode == CI_ADPCM_DECODER_MODE_MP3)
    {
        prompt_decoder.decode = audio_play_mp3_decode;
        prompt_decoder.data_in_size = PROMPT_MP3_IN_SIZE;
    }
    #endif
    #if AUDIO_PLAY_SUPPT_FLAC_PROMPT
    else if(config->mode == CI_ADPCM_DECODER_MODE_FLAC)
    {
        prompt_decoder.decode = audio_play_flac_decode;
        audio_play_flac_clear();
        audio_play_flac_config(&(config->flac));
        prompt_decoder.data_in_size = PROMPT_FLAC_IN_SIZE;
    }
    #endif
    
    ci_loginfo(LOG_AUDIO_PLAY,"prompt type %d\n",config->mode);

    return RETURN_OK;
}


/**
 * @brief prompt解码器复位
 * 
 * @retval RETURN_OK 复位成功
 * @retval RETURN_ERR 复位失败
 */
int32_t audio_play_prompt_clear(void)
{
    ADPCM_Decode_clear();
    return 0;
}


/**
 * @brief 解码器初始化
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
int32_t audio_play_prompt_init(void)
{
    #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
    #if AUDIO_PLAY_SUPPT_MP3_PROMPT
    if(RETURN_ERR == audio_play_mp3_init())
    {
        return RETURN_ERR;
    }
    #endif
    #if AUDIO_PLAY_SUPPT_FLAC_PROMPT
    if(RETURN_ERR == audio_play_flac_init())
    {
        return RETURN_ERR;
    }
    #endif
    #endif

    return RETURN_OK;
}

/**
 * @brief 解码器销毁
 * 
 * @retval RETURN_OK 销毁成功
 * @retval RETURN_ERR 销毁失败
 */
int32_t audio_play_prompt_destroy(void)
{
    #if AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
    #if AUDIO_PLAY_SUPPT_MP3_PROMPT
    if(RETURN_ERR == audio_play_mp3_destroy())
    {
        return RETURN_ERR;
    }
    #endif
    #if AUDIO_PLAY_SUPPT_FLAC_PROMPT
    if(RETURN_ERR == audio_play_flac_destroy())
    {
        return RETURN_ERR;
    }
    #endif
    #endif

    return RETURN_OK;
}

/**
 * @brief adpcm解码器结构对象
 * 
 */
audio_play_decoder_ops_t prompt_decoder =
{
    .decoder_name = "prompt",
    .init = audio_play_prompt_init,
    .decode = audio_play_adpcm_decode,
    .save = NULL,
    .load = NULL,
    .config = audio_play_prompt_config,
    .clear = audio_play_prompt_clear,
    .destroy = audio_play_prompt_destroy,
    .data_in_size = ADPCM_IN_SIZE,
    .next_ops = NULL,
};


#include "aacdec.h"
#include "parse_m4a_atom_containers.h"

/* aac格式最大输入大小配置 */
#define AAC_IN_SIZE (2*AAC_MAINBUF_SIZE)

/* aac解码器描述符 */
static HAACDecoder aac_dec_handle;
/* aac帧信息 */
static AACFrameInfo aacFrameInfo; 
/* 保存m4a信息 */
static m4a_file_info_t save_m4a_file_info; 
/* 解码函数类型 */
typedef int32_t (*decode_fun)(uint8_t *, uint32_t, short *,uint32_t *, uint32_t *,audio_format_info_t *);
/* 保存aacdecode模式 */
decode_fun save_aacdecode = NULL;

/**
 * @brief aac解码器初始化
 * 
 * @retval RETURN_OK 初始化成功
 * @retval RETURN_ERR 初始化失败
 */
int32_t audio_play_aac_init(void)
{
    if(aac_dec_handle == NULL)
    {
        aac_dec_handle = AACInitDecoder();
        if(0 == aac_dec_handle )
        {
            ci_logerr(LOG_AUDIO_PLAY,"aac decoder malloc failed!\n");
            return RETURN_ERR;
        }
    }
    return 0;
}

/**
 * @brief aac解码器销毁
 * 
 * @retval RETURN_OK 解码器销毁成功
 * @retval RETURN_ERR 解码器销毁失败
 */
int32_t audio_play_aac_destroy(void)
{
    if(aac_dec_handle != NULL)
    {
        AACFreeDecoder(aac_dec_handle);
        aac_dec_handle = NULL;
    }
    return 0;
}


/**
 * @brief aac帧解码，用于adts流解码
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_aac_decode(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    int32_t err;
    int32_t bytesLeft = (int32_t)in_size;
    uint8_t *readPtr = input;
    int32_t aac_sync_offset = 0;
    
    audio_format_info->out_min_size = 1024;

    /* 寻找帧头 */
    aac_sync_offset=AACFindSyncWord(readPtr, bytesLeft);  

    if(-1 == aac_sync_offset)
    {
        /*sync not found*/
        /*aac sync byte 2 byte,just keep 2 byte for next*/
        *no_use_size = 2;
        ci_logdebug(LOG_AUDIO_PLAY,"res = %d!\n",aac_sync_offset);
        return -1;
    }
    else
    {
        /*sync found*/
        /* 偏移至帧头 */
        readPtr   += aac_sync_offset;  
        bytesLeft -= aac_sync_offset; 
        
        /* 解码 */
        err = AACDecode(aac_dec_handle,&readPtr,(int *)(&bytesLeft),output);
        *no_use_size = bytesLeft;

        /* 取的帧信息 */
        AACGetLastFrameInfo(aac_dec_handle, &aacFrameInfo);             
        if(ERR_AAC_NONE != err )
        {   
            ci_logwarn(LOG_AUDIO_PLAY,"aac_decord err %d,bad frame!,remove 2byte,detect next\r\n",err);
            *no_use_size = bytesLeft-2;   
            return -1;
        }
        else
        {
            audio_format_info->samprate = aacFrameInfo.sampRateCore;
            if((aacFrameInfo.sampRateCore > 96000)||(aacFrameInfo.sampRateCore < 8000))
            {
                ci_logwarn(LOG_AUDIO_PLAY,"do not support samprate is %d!\n",aacFrameInfo.sampRateCore);
                *no_use_size = bytesLeft-2;
                return -1;
            }
            audio_format_info->nChans = aacFrameInfo.nChans;
            *out_size = (aacFrameInfo.bitsPerSample/8)*aacFrameInfo.outputSamps;
            return 0;
        }  
    }
}


/**
 * @brief aac帧解码2，用于aac源数据解码
 * 
 * @param input 输入帧地址
 * @param in_size 输入长度
 * @param output 输出帧地址
 * @param out_size 输出帧大小
 * @param no_use_size 未使用的数据大小
 * @param audio_format_info 音频信息结构
 * 
 * @retval RETURN_OK 帧解码正确
 * @retval RETURN_ERR 帧解码异常
 */
int32_t audio_play_aac_decode_2(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info)
{
    int32_t err;
    int32_t bytesLeft = (int32_t)in_size;
    uint8_t *readPtr = input;
    
    audio_format_info->out_min_size = 1024;

    /* 解码 */
    err = AACDecode(aac_dec_handle,&readPtr,(int *)(&bytesLeft),output);
    *no_use_size = bytesLeft;

    /* 取的帧信息 */
    AACGetLastFrameInfo(aac_dec_handle, &aacFrameInfo);             
    if(ERR_AAC_NONE != err )
    {   
        ci_logwarn(LOG_AUDIO_PLAY,"aac_decord err %d,bad frame!, remove this frame, detect next\r\n",err);
        /* FIXME: decode_2没有同步字，所以出现错误帧是直接跳过还是返回多少未使用的data呢(⊙o⊙)? */
        *no_use_size = 0;
        //*no_use_size = bytesLeft-2;
        return -1;
    }
    else
    {
        audio_format_info->samprate = aacFrameInfo.sampRateCore;
        if((aacFrameInfo.sampRateCore > 96000)||(aacFrameInfo.sampRateCore < 8000))
        {
            ci_logwarn(LOG_AUDIO_PLAY,"do not support samprate is %d!\n",aacFrameInfo.sampRateCore);
            *no_use_size = bytesLeft-2;
            return -1;
        }
        audio_format_info->nChans = aacFrameInfo.nChans;
        *out_size = (aacFrameInfo.bitsPerSample/8)*aacFrameInfo.outputSamps;
        return 0;
    }  
}


/**
 * @brief 配置解码器模式
 * 
 * @param mode 模式参数
 * @retval RETURN_OK 配置正确
 * @retval RETURN_ERR 配置异常
 */
int32_t audio_play_aac_config( void *mode)
{
    m4a_file_info_t *m4a_file_info = (m4a_file_info_t *)mode;

    if(m4a_file_info != NULL)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(&save_m4a_file_info,m4a_file_info,sizeof(m4a_file_info_t));
        ci_logwarn(LOG_AUDIO_PLAY,"samprate is %d, numChannel is %d!\n",m4a_file_info->media_info->sound.timeScale,m4a_file_info->media_info->format.numChannel);

        aacFrameInfo.nChans = m4a_file_info->media_info->format.numChannel;
        aacFrameInfo.sampRateCore = m4a_file_info->media_info->sound.timeScale;
        aacFrameInfo.profile = 0;
        AACSetRawBlockParams(aac_dec_handle,0,&aacFrameInfo);
        aac_decoder.decode = audio_play_aac_decode_2;
    }
    return RETURN_OK;
}

/**
 * @brief aac解码器现场保存
 * 
 * @retval RETURN_OK 保存成功
 * @retval RETURN_ERR 保存失败
 */
int32_t audio_play_aac_save(void)
{
    save_aacdecode = aac_decoder.decode;
    return 0;
}


/**
 * @brief aac解码器现场恢复
 * 
 * @retval RETURN_OK 恢复成功
 * @retval RETURN_ERR 恢复失败
 */
int32_t audio_play_aac_load(void)
{
    if(save_aacdecode != NULL)
    {
        aac_decoder.decode = save_aacdecode;
        if(aac_decoder.decode == audio_play_aac_decode_2)
        {
            aacFrameInfo.nChans = save_m4a_file_info.media_info->format.numChannel;
            aacFrameInfo.sampRateCore = save_m4a_file_info.media_info->sound.timeScale;
            aacFrameInfo.profile = 0;
            AACSetRawBlockParams(aac_dec_handle,0,&aacFrameInfo);
        }
    }
   
    return 0;
}



/**
 * @brief aac解码器复位
 * 
 * @retval RETURN_OK 复位成功
 * @retval RETURN_ERR 复位失败
 */
int32_t audio_play_aac_clear(void)
{
    aac_decoder.decode = audio_play_aac_decode;    
    AACClearDecoder(aac_dec_handle);
    return RETURN_OK;
}

/**
 * @brief aac解码器结构对象
 * 
 */
audio_play_decoder_ops_t aac_decoder =
{
    .decoder_name = "aac",
    .init = audio_play_aac_init,
    .decode = audio_play_aac_decode,
    .load = audio_play_aac_load,
    .save = audio_play_aac_save,
    .clear = audio_play_aac_clear,
    .config = audio_play_aac_config,
    .destroy = audio_play_aac_destroy,
    .data_in_size = AAC_IN_SIZE,
    .next_ops = NULL,
};

