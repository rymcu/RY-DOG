/**
 * @file sdk_default_config.h
 * @brief sdk配置文件
 * @version 1.0
 * @date 2019-08-08
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _SDK_DEFAULT_CONFIG_H_
#define _SDK_DEFAULT_CONFIG_H_

#if defined(CI_CONFIG_FILE)
#include CI_CONFIG_FILE
#endif  


/******************************** sdk config *********************************/
// SDK VER
#define SDK_VERSION                    1
#define SDK_SUBVERSION                 12
#define SDK_REVISION                   16
#define SDK_TYPE                       "Offline"

/******************************** 开发板选择 **********************************/
//板级支持配置
#ifndef BOARD_PORT_FILE
#define BOARD_PORT_FILE "CI-D03GS02S.c"
#endif

// //示例板级支持配置
// #ifndef BOARD_PORT_FILE
// #define BOARD_PORT_FILE            "board_port_xxx.c"
// #endif

/******************************** 灯控功能选择 **********************************/
// 使用VAD指示灯控
#ifndef USE_VAD_LIGHT
#define USE_VAD_LIGHT                   0       
#endif

// 使用BLINK闪烁灯控
#ifndef USE_BLINK_LIGHT
#define USE_BLINK_LIGHT                 0       
#endif

// 使用NIGHT小夜灯灯控
#ifndef USE_NIGHT_LIGHT
#define USE_NIGHT_LIGHT                 0       
#endif

// 使用COLOR三色彩灯灯控
#ifndef USE_COLOR_LIGHT
#define USE_COLOR_LIGHT                 0       
#endif

/******************************** 外设功能选择 **********************************/
// 使用IIC引脚配置
#ifndef USE_IIC_PAD
#define USE_IIC_PAD                     0       
#endif

// 退出唤醒时间
#ifndef EXIT_WAKEUP_TIME
#define EXIT_WAKEUP_TIME 30000
#endif

// volume set
#ifndef VOLUME_MAX
#define VOLUME_MAX                      7       //voice play max volume level
#endif
#ifndef VOLUME_MID
#define VOLUME_MID                      5       //voice play middle volume level
#endif
#ifndef VOLUME_MIN
#define VOLUME_MIN                      1       //voice play min volume level
#endif

#ifndef VOLUME_DEFAULT
#define VOLUME_DEFAULT                  5
#endif
//最大音量绝对值
#ifndef SET_MAX_VOLUME
#define SET_MAX_VOLUME                  75
#endif
//最小音量绝对值
#ifndef SET_MIN_VOLUME
#define SET_MIN_VOLUME                  10
#endif
// customer Ver (user define)
#ifndef USER_VERSION_MAIN_NO
#define USER_VERSION_MAIN_NO            1
#endif
#ifndef USER_VERSION_SUB_NO
#define USER_VERSION_SUB_NO             0
#endif
#ifndef USER_TYPE
#define USER_TYPE                       "CustomerAA"
#endif


// 使用COLOR三色彩灯灯控
#ifndef GS0XJ_BOARD_PA_AUTO
#define GS0XJ_BOARD_PA_AUTO             0      //GS0XJ系列板子PA是否开启自动检测功能。0、关闭（默认低电平有效）。1、开启。    
#endif


/* 串口调试LOG输出 */
#ifndef CONFIG_CI_LOG_UART
#define CONFIG_CI_LOG_UART                  HAL_UART0_BASE
#endif

/* 是否开启命令行控制台 */
#ifndef COMMAND_LINE_CONSOLE_EN
#define COMMAND_LINE_CONSOLE_EN         0
#endif

/* 驱动中断采用事件标准组配置为1，采用全局变量配置为0 */
#ifndef DRIVER_OS_API
#define DRIVER_OS_API                       0
#endif

#ifndef CONFIG_DIRVER_BUF_USED_FREEHEAP_EN
#define CONFIG_DIRVER_BUF_USED_FREEHEAP_EN  1
#endif

/* 在播放中是否暂停语音输入，一般来讲开启aec则不开启暂停语音输入否则开启 */
#ifndef PAUSE_VOICE_IN_WITH_PLAYING
#define PAUSE_VOICE_IN_WITH_PLAYING         1
#endif

/*是否允许关闭提示播报音 */
#ifndef PROMPT_CLOSE_EN
#define PROMPT_CLOSE_EN                     1
#endif

/*异常复位跳过上电播报 */
#ifndef EXCEPTION_RST_SKIP_BOOT_PROMPT
#define EXCEPTION_RST_SKIP_BOOT_PROMPT      1
#endif

/*默认模型分组ID，开机后第一次运行的识别环境 */
#ifndef DEFAULT_MODEL_GROUP_ID
#define DEFAULT_MODEL_GROUP_ID              0
#endif

/* 启用非唤醒状态切换模型 */
#ifndef USE_SEPARATE_WAKEUP_EN
#define USE_SEPARATE_WAKEUP_EN 1
#endif 

/* 启用降频模式 */
#ifndef USE_LOWPOWER_DOWN_FREQUENCY
#define USE_LOWPOWER_DOWN_FREQUENCY         0       
#endif

/* 启用低功耗模式 */
// #ifndef USE_LOWPOWER_OSC_FREQUENCY
// #define USE_LOWPOWER_OSC_FREQUENCY          0       //暂不能使用
// #endif

/*************************** 解码参数配置 ************************************/
#ifndef DECODER_MIN_ACTIVE
#define DECODER_MIN_ACTIVE  (400)   //默认400，若节点小于8000，可尝试修改到300；最小值200，最大值800
#endif

#ifndef DECODER_BEAM    
#define DECODER_BEAM    (9.5f)  //最小值3.5 最大值15.5
#endif

/*************************** 语音前端算法 config *****************************/

/*单MIC DENOISE*/
#ifndef USE_DENOISE_MODULE
#define USE_DENOISE_MODULE             0
#endif
/*双MIC DOA*/
#ifndef USE_DOA_MODULE
#define USE_DOA_MODULE                0
#endif
/*双MIC DEREVERB*/
#ifndef USE_DEREVERB_MODULE
#define USE_DEREVERB_MODULE            0
#endif

/*双MIC BEAMFORMING*/
#ifndef USE_BEAMFORMING_MODULE
#define USE_BEAMFORMING_MODULE          0
#endif

/*回声消除 AEC*/
#ifndef USE_AEC_MODULE
#define USE_AEC_MODULE                 0    
#endif

/*动态alc:ALC AUTO SWITCH*/
#ifndef USE_ALC_AUTO_SWITCH_MODULE
#define USE_ALC_AUTO_SWITCH_MODULE                0
#endif

/*是否codec录音只支持单个声道,需根据算法选择配合使用*/
#ifndef AUDIO_CAPTURE_USE_SINGLE_CHANNEL
#define AUDIO_CAPTURE_USE_SINGLE_CHANNEL   1
#endif


/*是否使用重采样*/
#ifndef INNER_CODEC_AUDIO_IN_USE_RESAMPLE
#define INNER_CODEC_AUDIO_IN_USE_RESAMPLE   1   //默认打开   0:不开重采样   1：打开重采样
#endif



/*根据codec使用的声道数来配置使用内存大小*/
#if !CONFIG_DIRVER_BUF_USED_FREEHEAP_EN
#if AUDIO_CAPTURE_USE_SINGLE_CHANNEL
    #define AUDUI_CAPTURE_NO_USE_MALLOC_BLOCK_SIZE (512U)
#else
    #define AUDUI_CAPTURE_NO_USE_MALLOC_BLOCK_SIZE (1024U)
#endif
#endif


/******************************** VP config **********************************/

//1:开启VP计算；0：关闭VP计算
// #ifndef USE_VP
// #define USE_VP                              0
// #endif

// #ifndef VP_TEMPLATE_NUM
// #define VP_TEMPLATE_NUM                     10
// #endif

/******************************** command word self-learning config **********************************/
#ifndef USE_CWSL
#define USE_CWSL                             0 //命令词自学习(command word self-learning)
#endif

#ifndef CICWSL_TOTAL_TEMPLATE
#define CICWSL_TOTAL_TEMPLATE                32 //可存储模板数量
#endif


/**************************** audio player config ****************************/
/* 注册prompt解码器 */
#ifndef USE_PROMPT_DECODER
#define USE_PROMPT_DECODER                   1
#endif

/* 注册mp3解码器 */
#ifndef USE_MP3_DECODER
#define USE_MP3_DECODER                     1
#endif

/* 注册m4a(aac)解码器 */
#ifndef USE_AAC_DECODER
#define USE_AAC_DECODER                     0
#endif

/* 注册ms wav解码器 */
#ifndef USE_MS_WAV_DECODER
#define USE_MS_WAV_DECODER                  0
#endif

/* 注册flac解码器 */
#ifndef USE_FLAC_DECODER
#define USE_FLAC_DECODER                    0
#endif

/* 命令词支持MP3格式音频，注意目前仅支持具有CI特殊头格式的MP3文件 */
#ifndef AUDIO_PLAY_SUPPT_MP3_PROMPT
#define AUDIO_PLAY_SUPPT_MP3_PROMPT         0
#endif
#if AUDIO_PLAY_SUPPT_MP3_PROMPT
#if !USE_MP3_DECODER
#error "if AUDIO_PLAY_SUPPT_MP3_PROMPT enabled please config USE_MP3_DECODER enable in your user_config.h"
#endif
#endif

/* 命令词支持FLAC格式音频，注意目前仅支持具有CI特殊头格式的FLAC文件 */
#ifndef AUDIO_PLAY_SUPPT_FLAC_PROMPT
#define AUDIO_PLAY_SUPPT_FLAC_PROMPT        0
#endif
#if AUDIO_PLAY_SUPPT_FLAC_PROMPT
#if !USE_FLAC_DECODER
#error "if AUDIO_PLAY_SUPPT_FLAC_PROMPT enabled please config USE_FLAC_DECODER enable in your user_config.h"
#endif
#endif

/* 命令词支持标准IMA ADPCM格式音频 */
#ifndef AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT
#define AUDIO_PLAY_SUPPT_IMAADPCM_PROMPT    1
#endif

/* 播放器底层缓冲区个数 */
#ifndef AUDIO_PLAY_BLOCK_CONT
    #define AUDIO_PLAY_BLOCK_CONT          (4)
#endif

/* 解码器内存使用动态分配 */
#ifndef AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM
#define AUDIO_PLAY_USE_DYNAMIC_DECODER_MEM  0
#endif

/* 数据组合播报 */
#ifndef AUDIO_PLAY_USE_QSPI_FLASH_LIST
#define AUDIO_PLAY_USE_QSPI_FLASH_LIST      0
#endif

/* 启用网络播放（需要lwip支持） */
#ifndef AUDIO_PLAY_USE_NET
#define AUDIO_PLAY_USE_NET                  0
#endif

/* 启用自定义外部数据源播放 */
#ifndef AUDIO_PLAY_USE_OUTSIDE
#define AUDIO_PLAY_USE_OUTSIDE              0
#endif

/* 启用自定义外部数据源播放 */
#ifndef AUDIO_PLAY_USE_OUTSIDE_V2
#define AUDIO_PLAY_USE_OUTSIDE_V2           0
#endif

/* 启用文件系统播放 */
#ifndef AUDIO_PLAY_USE_SD_CARD
#define AUDIO_PLAY_USE_SD_CARD              0
#endif

/*! 启用播放器双声道混音到右声道功能 */
#ifndef AUDIO_PLAY_USE_MIX_2_CHANS
#define AUDIO_PLAY_USE_MIX_2_CHANS                     0
#endif

/*! 自动音频识别文件头(消耗额外内存,播放m4a、flac、非单声道16Kwav音频格式时必须打开) */
#ifndef AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE
#define AUDIO_PLAYER_CONFIG_AUTO_PARSE_AUDIO_FILE      0
#endif

/*! 针对TTS音频倍速播放功能，注意仅支持加/减速播放TTS人声，不支持音乐 */
#ifndef AUDIO_PLAY_USE_SPEEDING_SPEECH
#define AUDIO_PLAY_USE_SPEEDING_SPEECH                 0
#endif


/*! 唤醒词播放词常驻内存 */
#ifndef AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM
#define AUDIO_PLAY_SUPPT_WAKEUP_VOICE_IN_RAM           0
#endif

/*! 用于解决应用程序可能存在的偏移不对齐问题 */
#ifndef AUDIO_PLAYER_FIX_OFFSET_ISSUE
#define AUDIO_PLAYER_FIX_OFFSET_ISSUE                  0
#endif


/****************************** 功放控制 config *******************************/

/*Whether the PA switch is controled by audio player*/
#ifndef PLAYER_CONTROL_PA
#define PLAYER_CONTROL_PA                              0
#endif


/****************************** 串口协议 config *******************************/
#ifndef MSG_COM_USE_UART_EN
#define MSG_COM_USE_UART_EN                0
#endif


#ifndef UART_PROTOCOL_NUMBER
#define UART_PROTOCOL_NUMBER	          (HAL_UART1_BASE)// HAL_UART0_BASE ~ HAL_UART2_BASE
#endif

#ifndef UART_PROTOCOL_BAUDRATE
#define UART_PROTOCOL_BAUDRATE            (UART_BaudRate9600)
#endif

#ifndef UART_PROTOCOL_VER
#define UART_PROTOCOL_VER                 2   //串口协议版本号，1：一代协议，2：二代协议，255：平台生成协议
#endif

#ifndef UART0_PAD_OPENDRAIN_MODE_EN
#define UART0_PAD_OPENDRAIN_MODE_EN 1
#endif

#ifndef UART1_PAD_OPENDRAIN_MODE_EN
#define UART1_PAD_OPENDRAIN_MODE_EN 1
#endif

#ifndef UART2_PAD_OPENDRAIN_MODE_EN
#define UART2_PAD_OPENDRAIN_MODE_EN        0   //UART引脚开启开漏模式，支持5V电平。0:关闭开漏模式；1：开启开漏模式。
#endif



#ifndef UART_BAUDRATE_CALIBRATE
#define UART_BAUDRATE_CALIBRATE             0               //是否使能波特率校准功能
#endif

#if UART_BAUDRATE_CALIBRATE

#ifndef BAUDRATE_SYNC_PERIOD
#define BAUDRATE_SYNC_PERIOD        300000       // 波特率同步周期，单位毫秒
#endif

#ifndef BAUDRATE_FAST_SYNC_PERIOD
#define BAUDRATE_FAST_SYNC_PERIOD   5000         // 一次波特率校准识别后，下一次同步周期，单位毫秒
#endif

// #ifndef UART_CALIBRATE_FIX_BAUDRATE
// #define UART_CALIBRATE_FIX_BAUDRATE         9600            //通信波特率
// #endif

// #ifndef UART_CALIBRATE_MAX_ERR_PROPORITION
// #define UART_CALIBRATE_MAX_ERR_PROPORITION  0.08            //允许的最大误差比8%
// #endif

// #ifndef UART_CALIBRATE_ARRAY_SIZE
// #define UART_CALIBRATE_ARRAY_SIZE           16              //最大数据缓存
// #endif

// #ifndef UART_CALIBRATE_IO_BASE
// #define UART_CALIBRATE_IO_BASE              PA              //校准波特率的IO口(RX引脚)
// #endif

// #ifndef UART_CALIBRATE_IO_PIN
// #define UART_CALIBRATE_IO_PIN               pin_2
// #endif

// #ifndef UART_CALIBRATE_IO_PAD
// #define UART_CALIBRATE_IO_PAD               PA2
// #endif

// #ifndef UART_CALIBRATE_IRQ
// #define UART_CALIBRATE_IRQ                  PA_IRQn
// #endif

// #ifndef UART_CALIBRATE_TIMER
// #define UART_CALIBRATE_TIMER                TIMER2
// #endif

// #ifndef UART_CALIBRATE_TIMER_BASE
// #define UART_CALIBRATE_TIMER_BASE           HAL_TIMER2_BASE
// #endif

 #endif


/***************************** flash加密 config ******************************/
/*flash防拷贝加密 */
#ifndef COPYRIGHT_VERIFICATION
#define COPYRIGHT_VERIFICATION          0 //1：使能flash加密校验，用于防止flash拷贝。需要在lds文件增加ROM段大小。0：禁止flash加密校验
#endif

#if (COPYRIGHT_VERIFICATION == 1)
#define ENCRYPT_DEFAULT                 0 //默认加密算法
#define ENCRYPT_USER_DEFINE             1 //用户自定义加密算法

#define ENCRYPT_ALGORITHM               ENCRYPT_DEFAULT //设置加密方式
#endif


/****************************** i2c communicate config *******************************/
#ifndef MSG_USE_I2C_EN
#define MSG_USE_I2C_EN                0                  /*使能IIC通信协议*/
#endif

#ifndef I2C_PROTOCOL_SPEED
#define I2C_PROTOCOL_SPEED            (100)              /*IIC传输速度*/
#endif


// 播报音配置
#ifndef PLAY_WELCOME_EN
#define PLAY_WELCOME_EN 0
#endif
#ifndef PLAY_ENTER_WAKEUP_EN
#define PLAY_ENTER_WAKEUP_EN 0
#endif
#ifndef PLAY_EXIT_WAKEUP_EN
#define PLAY_EXIT_WAKEUP_EN 0
#endif
#ifndef PLAY_OTHER_CMD_EN
#define PLAY_OTHER_CMD_EN 0
#endif

/******************************ASR config************************************/
#ifndef ADAPTIVE_THRESHOLD
#define ADAPTIVE_THRESHOLD                  1   //ASR 自适应阈值    =1 开启  =0 关闭
#endif

#ifndef ASR_SKIP_FRAME_CONFIG
#define ASR_SKIP_FRAME_CONFIG               0   //跳帧模式  0:关闭跳帧，1:只有命令词网络跳帧，2:所有模型都跳帧
#endif
#endif /*_SDK_DEFAULT_CONFIG_H_*/


#define FLASH_CPU_READ_BASE_ADDR	(0x50000000U)



#define VAD_SENSITIVITY_HIGH        1        // VAD 高灵敏度，适用于低音量语音,安静，或者各种噪声环境,但功耗偏高。
#define VAD_SENSITIVITY_MID         3        // VAD 中灵敏度，适用于安静及轻度噪声环境，功耗适中。
#define VAD_SENSITIVITY_LOW         9        // VAD 低灵敏度，仅适用于安静环境，降低误触发，也可以降低功耗。


#ifndef VAD_SENSITIVITY
#define VAD_SENSITIVITY         VAD_SENSITIVITY_HIGH    //用于配置VAD灵敏度
#endif



//语音相关的宏
/*********************** 使用外部IIS输入语音进行识别 *************************/
#ifndef AUDIO_IN_BUFFER_NUM
#define AUDIO_IN_BUFFER_NUM (4)
#endif


//使用IIS输出算法处理之后的语音
#ifndef USE_IIS1_OUT_PRE_RSLT_AUDIO
#define USE_IIS1_OUT_PRE_RSLT_AUDIO  (0)
#endif


//根据 HOST_MIC_RECORD_CODEC_ID 确定
// #ifndef MIC_FROM_WHICH_IIS
// #define MIC_FROM_WHICH_IIS  1//0：MIC接外部CODEC  1：MIC接内部CODEC 2：MIC使用PDM
// #endif


//根据 PLAY_CODEC_ID 确定
// #ifndef USE_OUTSIDE_CODEC_PLAY
// #define USE_OUTSIDE_CODEC_PLAY      0//使用外部codec进行播放
// #endif

#ifndef HOST_MIC_RECORD_CODEC_ID              
#define HOST_MIC_RECORD_CODEC_ID     1   //主MIC录音codec id: 0：主MIC接外部CODEC  1：主MIC接内部CODEC 2：主MIC使用PDM
#endif

#ifndef HOST_CODEC_CHA_NUM              
#define HOST_CODEC_CHA_NUM     1   //主COEDC的通道数，使用双麦算法和单MIC AEC的时候都应是2
#endif



#ifndef ASSIST_MIC_RECORD_CODEC_ID              
#define ASSIST_MIC_RECORD_CODEC_ID     2   //副MIC录音codec id: 0：副MIC接外部CODEC  1：副MIC接内部CODEC 2：副MIC使用PDM
#endif


#ifndef REF_RECORD_CODEC_ID              
#define REF_RECORD_CODEC_ID     0   //参考信号录音codec id: 0：参考信号接外部CODEC  1：参考信号接内部CODEC 2：参考信号使用PDM，
#endif                              //只在参考和MIC使用不一样的CODEC时使用

#if ((HOST_MIC_RECORD_CODEC_ID + ASSIST_MIC_RECORD_CODEC_ID + REF_RECORD_CODEC_ID) != 3)
#error "error\n"
#endif

#if (ASSIST_MIC_RECORD_CODEC_ID == HOST_MIC_RECORD_CODEC_ID)
#error "error\n"
#endif

#ifndef PLAY_CODEC_ID              
#define PLAY_CODEC_ID           1   //播音codec id  0：播报使用接外部CODEC  1：播报使用内部CODEC
#endif


#ifndef PLAY_PRE_AUDIO_CODEC_ID
#define PLAY_PRE_AUDIO_CODEC_ID      0  //播放预处理之后的语音的codec id  只能使用外部IIS，只能是0
#endif


#ifndef AUDIO_CAP_POINT_NUM_PER_FRM    
#define AUDIO_CAP_POINT_NUM_PER_FRM    (256)//采音每帧的点数  16K采样率下一帧的采样点数，160
#endif


#ifndef AUDIO_IN_FROM_DMIC
#define AUDIO_IN_FROM_DMIC                  0       //是否使用数字MIC输入音频
#endif

/**********I2S interface config**********/
#ifndef USE_I2S_INTERFACE_SCK_LRCK_32
#define USE_I2S_INTERFACE_SCK_LRCK_32              0//I2S 传输语音数据到WIFI时，SCK与LRCK的比值是否为32，否则为64
#endif


#ifndef IF_JUST_CLOSE_HPOUT_WHILE_NO_PLAY
#define IF_JUST_CLOSE_HPOUT_WHILE_NO_PLAY    0//没有播报的时候只关HP：0：没有播报的时候关DAC（省电，但是蓝牙播报的时候会干扰ADC）
#endif    


/************* 时钟源配置选择 *************************/
#ifndef USE_EXTERNAL_CRYSTAL_OSC
#define USE_EXTERNAL_CRYSTAL_OSC             1  //是否使用外部晶振。0：不使用外部晶振时钟；1：使用外部晶振时钟；默认使用外部晶振。
#endif


/************* 电源配置选择 *************************/
#ifndef USE_INNER_LDO3
#define USE_INNER_LDO3 1
#endif


/*! 在线应用相关支持的开关 */
#ifndef ON_LINE_SUPPORT
#define ON_LINE_SUPPORT                     0    // 0: 关闭在线应用相关支持；1: 开启在线应用相关支持。
#endif


#if USE_EXTERNAL_CRYSTAL_OSC
#undef UART_BAUDRATE_CALIBRATE
#define UART_BAUDRATE_CALIBRATE             0               //使用外部晶振时，不需要校准波特率。
#endif

#ifndef MIC_DIFF_SINGLE
#define MIC_DIFF_SINGLE                     0              //1，单端。0，差分（通用模块都是差分模式，省成本的模块为单端（MICN_L 接GND）时，需要配置为SINGLE）
#endif

#if (USE_LOWPOWER_DOWN_FREQUENCY)
#undef INNER_CODEC_AUDIO_IN_USE_RESAMPLE
#define INNER_CODEC_AUDIO_IN_USE_RESAMPLE   0
#endif

#ifndef ONE_SHOT_ENABLE
#define ONE_SHOT_ENABLE                     0              //1：开启oneshot功能。0：关闭oneshot功能
#endif