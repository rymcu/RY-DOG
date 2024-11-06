#ifndef __STATUS_SHARE_H__
#define __STATUS_SHARE_H__

#include "ci_log.h"

typedef unsigned long status_t;

#define STATUS_SHARE_MODULE

#define INVALID_STATUS (((status_t)0) - 1) //无效状态值

#define LOG_STATUS_SHARE CI_LOG_WARN

typedef enum
{
    EM_STATUS_ID_START = 0, //普通状态起始ID，本模块内部使用，请不要修改。
    CI_SS_EXT_OSC_CLK,      //晶振频率
    CI_SS_SRC_CLK,          //系统时钟源频率
    CI_SS_AHB_CLK,          //AHB总线时钟频率
    CI_SS_APB_CLK,          //APB总线时钟频率
    CI_SS_IPCORE_CLK,       //IPCORE时钟频率
    CI_SS_SYSTEMTICK_CLK,   //OS tick 时钟频率
    CI_SS_VAD_STATE,        //VAD状态：1、VAD start；2、没有VAD start
    CI_SS_ASR_SYS_STATE,    //ASR系统状态：1、ASR启动；2、ASR未启动（后续可能会增加ASR状态）
    CI_SS_WAKING_UP_STATE,  //当前是否处于唤醒状态：1、处于唤醒状态；2、处于非唤醒状态
    CI_SS_CMD_STATE,        //当前命令词是唤醒词还是命令词：1、唤醒词；2、命令词
    CI_SS_CMD_SCORE,                //当前命令词的分数
    CI_SS_CMD_SCORE_CHA0,           //双通道命令词状态下，通道0分数
    CI_SS_CMD_SCORE_CHA1,           //双通道命令词状态下，通道1分数
    CI_SS_WAKING_UP_STATE_FOR_SSP,  //当前是否处于唤醒状态：1、处于唤醒状态；2、处于非唤醒状态
    CI_SS_CMD_STATE_FOR_SSP,        //当前命令词是唤醒词还是命令词：1、唤醒词；2、命令词
    CI_SS_PLAY_STATE,               //播报状态：1、未播报；2、播报（可能会增加，本地播报或者网络播报，播放什么格式等的状态）
    CI_SS_FLASH_HOST_STATE,
    CI_SS_AST_STATE,
    CI_SS_ASR_WARNING,
    CI_SS_FLASH_BNPU_STATE,
    CI_SS_MIC_VOICE_STATUE, //MIC输入的语音的状态：0、未启动；1、正常状态；2、mute状态
    CI_SS_ALC_STATE,        //ALC开启关闭状态：0、关闭；1、开启
    CI_SS_CWSL_OUTPUT_FLAG,  //CWSL是否有识别结果输出：0、无识别结果；1、已有识别结果输出
    CI_SS_INTERCEPT_ASR_OUT, //是否拦截ASR输出：0、不拦截；1、拦截
    CI_SS_CWSL_IN_REG,      //自学习是否在学习状态。
    CI_SS_PR_ADDR,          //某基地址 CWSL 用
    CI_SS_START_SLEEP_PROCESS,  //标记是否已开始退出唤醒处理。

    /*系统参数设置*/
    //decoder参数设置
    CI_SS_DECODER_MIN_ACTIVE,   //默认400，若节点小于8000，可尝试修改到300，不可随意更改
    CI_SS_DECODER_BEAM,         //默认9.5
    
    CI_SS_AEC_MUTE_STATE,       //开AEC时，是否先MUTE语音再播报

    //下面的值用于统计状态ID数量，本模块内部使用，请不要修改。
    EM_STATUS_NUM,
} status_id_t;

typedef enum
{
    CI_SS_VAD_IDLE = 0, //VAD状态检测为非人声
    CI_SS_VAD_START,
    CI_SS_VAD_ON, //VAD状态检测为人声
    CI_SS_VAD_END
} ci_ss_vad_state_t;

typedef enum
{
    CI_SS_ALC_OFF = 0, //ALC处于关闭状态
    CI_SS_ALC_UP,      //ALC处于开启状态
} ci_ss_alc_state_t;
typedef enum
{
    CI_SS_ASR_SYS_POWER_OFF = 0, //ASR还未启动
    CI_SS_ASR_SYS_STARTED_UP,    //ASR启动了
} ci_ss_asr_sys_state_t;

typedef enum
{
    CI_SS_NO_WAKEUP = 0, //当前系统处于非唤醒状态
    CI_SS_WAKEUPED,      //当前系统处于唤醒阶段
} ci_ss_wakeup_state_t;

typedef enum
{
    CI_SS_CMD_IS_NULL = 0,
    CI_SS_CMD_IS_WAKEUP, //当前的命令词是唤醒词
    CI_SS_CMD_IS_NORMAL,     //当前的命令词是非唤醒词
} ci_ss_cmd_state_t;

typedef enum
{
    CI_SS_PLAY_STATE_IDLE = 0, //当前未播报
    CI_SS_PLAY_STATE_PLAYING,  //当前处于播报状态
} ci_ss_play_state;

typedef enum
{
    CI_SS_FLASH_POWER_OFF = 0,
    CI_SS_FLASH_IDLE,
    CI_SS_FLASH_READ,
    CI_SS_FLASH_WRITE,
    CI_SS_FLASH_ERASE,
    CI_SS_FLASH_READ_UNIQUE_ID,
} ci_ss_flash_state;

typedef enum
{
    CI_SS_MIC_VOICE_NOT_START = 0,
    CI_SS_MIC_VOICE_NORMAL,
    CI_SS_MIC_VOICE_MUTE,
} ci_ss_mic_voice_state;

typedef enum
{
    CI_SS_AEC_MUTE_OFF = 0,
    CI_SS_AEC_MUTE_ON,
} ci_ss_aec_mute_state;

/**
 * @brief 信息共享模块初始化.
 */
void ciss_init(void);

/**
 * @brief 设置状态信息，如果状态有变化且是可等待状态，发送状态等待事件标志位.
 * 
 * @param id 状态信息标识，指定要设置的状态.
 * @param value 要设置的状态值.
 */
void ciss_set(status_id_t id, status_t value);

/**
 * @brief 读取状态信息.
 * 
 * @param id 状态信息标识，指定要读取的状态.
 * @return 要读取的状态值.
 */
status_t ciss_get(status_id_t id);



#endif
