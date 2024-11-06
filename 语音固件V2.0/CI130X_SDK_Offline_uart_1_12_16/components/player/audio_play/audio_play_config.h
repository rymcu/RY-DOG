#ifndef _AUDIO_PLAY_CONFIG_H_
#define _AUDIO_PLAY_CONFIG_H_

#include "sdk_default_config.h"

/*! 播放器默认音量大小 */
#define AUDIO_PLAYER_CONFIG_DEFAULT_VOLUME             70

/*! mp3解码器保存上下文(消耗额外内存,在两首mp3互相切换时需要开启) */
#define AUDIO_PLAYER_CONFIG_MP3_SAVE_DECODER_STATUS    0
#define AUDIO_PLAYER_CONFIG_MP3_USE_OUTSIDE_MP3TABLE   0

/*! CI放音设备抗pop策略,这些策略目前仅有部分策略在云+端方案使用 */
#define AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_HARD  0  //播放暂停抗pop硬件方法
#define AUDIO_PLAYER_FIX_DEVICE_POP_ISSUE_SOFT  0  //播放暂停抗pop软件方法
#define AUDIO_PLAY_USE_INNER_CODEC_HPOUT_MUTE   0  //采样率改变时关闭HPOUT抗pop策略

/*! 播放器任务栈大小配置 */
#if AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
#define AUDIO_PLAYER_PROCTHREAD_STACK            (512)
#define AUDIO_PLAYER_DATATHREAD_STACK            (512)
#else
#define AUDIO_PLAYER_PROCTHREAD_STACK            (480)
#define AUDIO_PLAYER_DATATHREAD_STACK            (280)
#endif

/*!< sd卡音频文件名称长度 */
#define AUDIO_FILE_MAX_NAME_SIZE             (30) 

/*!< 剩余多少音频后去释放THRESHOLD回调函数 */
#define AUDIO_PLAYER_THRESHOLD_DATA_SIZE     (3200-576)

#endif /* _AUDIO_PLAY_CONFIG_H_ */
