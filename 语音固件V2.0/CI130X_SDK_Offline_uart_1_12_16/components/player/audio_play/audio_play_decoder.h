/**
 * @file audio_play_decoder.h
 * @brief  播放器音频解码器框架
 * @version 1.0
 * @date 2019-04-09
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _AUDIO_PLAY_DECODER_H_
#define _AUDIO_PLAY_DECODER_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "audio_play_process.h"
#include "audio_play_config.h"


typedef enum
{
    CI_ADPCM_DECODER_MODE_CIADPCM = 0,
    CI_ADPCM_DECODER_MODE_IMAADPCM = 1,
    CI_ADPCM_DECODER_MODE_WAV = 2,
    CI_ADPCM_DECODER_MODE_MP3 = 3,
    CI_ADPCM_DECODER_MODE_FLAC = 4,
}ci_adpcm_decoder_mode_t;

typedef struct 
{
	int min_blocksize, max_blocksize;	//block的最小/最大采样数
	int min_framesize, max_framesize;	//最小/最大帧大小
	int samplerate, channels;			//采样率和通道数
	int bps;
	unsigned long totalsamples;
}flac_file_info_t;

typedef struct
{
    uint32_t head_size;
    uint32_t nBlockAlign;    //块对齐大小
    uint32_t nSamplesPerSec; //采样率
}wav_file_info_t;

typedef struct
{
    union{
        wav_file_info_t wav;
        flac_file_info_t flac;
    };
	ci_adpcm_decoder_mode_t mode;
} prompt_decoder_config;


/**
 * @ingroup audio_player
 * @defgroup audio_play_decoder 播放器解码器结构
 * @brief 音频播放器组件解码器移植层
 * @{
 */


/**
 * @brief mp3解码器功能选项
 * 
 * @note 开启这个选项可以在mp3切换mp3抗POP、解决小概率错误音频解码，但会增加约2K的内存申请
 */
#define AUDIO_PLAY_FIX_MP3_DECODER_ISSUE  (AUDIO_PLAYER_CONFIG_MP3_SAVE_DECODER_STATUS)

/**
 * @brief 解码器结构
 * 
 */
typedef struct audio_play_decoder_ops_s
{
    const char *decoder_name;                   /*!< 解码器名称       */
    int32_t (*init)(void);                      /*!< 解码器初始化     */
    int32_t (*decode)(uint8_t * input, uint32_t in_size, short *output,uint32_t *out_size, uint32_t *no_use_size,audio_format_info_t *audio_format_info); /*!< 帧解码       */
    int32_t (*save)(void);                      /*!< 保存解码器上下文 */
    int32_t (*load)(void);                      /*!< 恢复解码器上下文 */
    int32_t (*config)(void *mode);              /*!< 配置解码器模式   */
    int32_t (*clear)(void);                     /*!< 复位解码器       */
    int32_t (*destroy)(void);                   /*!< 销毁解码器       */
    uint32_t data_in_size;                      /*!< 帧大小           */
    struct audio_play_decoder_ops_s * next_ops; /*!< 解码器句柄       */
}audio_play_decoder_ops_t;


extern audio_play_decoder_ops_t prompt_decoder;
extern audio_play_decoder_ops_t mp3_decoder;
extern audio_play_decoder_ops_t aac_decoder;
extern audio_play_decoder_ops_t ms_wav_decoder;
extern audio_play_decoder_ops_t flac_decoder;

extern void registe_decoder_ops(audio_play_decoder_ops_t *registe_ops);

/** @} */

#endif /* _AUDIO_PLAY_DECODER_H_ */
