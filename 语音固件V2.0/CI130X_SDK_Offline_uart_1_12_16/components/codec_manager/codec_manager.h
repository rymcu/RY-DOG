#ifndef __CODEC_MANAGER_H__
#define __CODEC_MANAGER_H__

#include "ci130x_system.h"
#include "ci130x_iis.h"
#include "ci130x_iic.h"
#include "ci130x_codec.h"


#define MAX_CODEC_NUM                   2


typedef struct codec_hw_info_str cm_codec_hw_info_t;

/**
 * @brief 放音设备的过采样率
 * 
 */
typedef enum
{
	//!过采样率128
    AUDIO_PLAY_OVER_SAMPLE_128 = 128,
	//!过采样率192
    AUDIO_PLAY_OVER_SAMPLE_192 = 192,
	//!过采样率256
    AUDIO_PLAY_OVER_SAMPLE_256 = 256,
	//!过采样率384
    AUDIO_PLAY_OVER_SAMPLE_384 = 384,
}audio_play_card_over_sample_t;

//放音设备的时钟源选择
typedef enum
{
    //IIS时钟源为IPCORE
    AUDIO_PLAY_CLK_SOURCE_IPCORE = 0,
    //IIS时钟源为EXT_OSC
    AUDIO_PLAY_CLK_SOURCE_EXT_OSC = 1,
    //IIS时钟源为INTERNAL RC
    AUDIO_PLAY_CLK_SOURCE_INTER_RC = 2,
    //IIS时钟源为PAD_IN
    AUDIO_PLAY_CLK_SOURCE_PAD_IN = 3,
    //IIS时钟源为晶振或者内部RC
    AUDIO_PLAY_CLK_SOURCE_OSC_OR_INEER_RC = 0xff,
}audio_play_card_clk_source_t;

typedef enum
{
    CM_IIS_TX,              /*!< IIS TX */
    CM_IIS_RX,              /*!< IIS RX */
}cm_iis_txrx_t;

typedef enum
{
    CODEC_INPUT,            /*!< 输入 ADC */
    CODEC_OUTPUT,           /*!< 输出 DAC */
}io_direction_t;

typedef enum
{
    CM_IOCTRL_SET_DAC_GAIN,                     /*!< 设置输出增益, param1:左声道增益, param2:右声道增益 */
    CM_IOCTRL_SET_ADC_GAIN,                     /*!< 设置输入增益, param1:左声道增益, param2:右声道增益 */
    CM_IOCTRL_ALC_ENABLE,                       /*!< 使能ALC，param1:ALC总开关(ENABLE=开,DISABLE=关);param2:ALC是否控制PGA(ENABLE=控制,DISABLE=不控制) */
    CM_IOCTRL_ALC_DISABLE,                      /*!< 关闭ALC，param1:ALC总开关(ENABLE=开,DISABLE=关);param2:ALC是否控制PGA(ENABLE=控制,DISABLE=不控制) */
    CM_IOCTRL_DAC_ENABLE,                       /*!< 使能DAC，param1:0=左声道,1=右声道; param2:ENABLE=开,DISABLE=关 */
    CM_IOCTRL_MUTE,                             /*!< 使能mute，param1:0=左声道,1=右声道; param2:ENABLE=开,DISABLE=关 */
}cm_io_ctrl_cmd_t;


typedef enum
{
    CM_CHA_LEFT = 1,
    CM_CHA_RIGHT = 2,
    CM_CHA_TWO_CHA = 3,
}cm_cha_sel_t;

/**
 * @brief 音频信息结构体
 */
typedef struct
{
    uint32_t sample_rate;                       /*!< 采样率 */    
    iis_data_width_t sample_depth;              /*!< 采样深度,以bit为单位 */
    uint8_t channel_flag;                       /*!< 声道选择，bit[0] = 1选择左声道，bit[1]=1选择右声道，可以用或运算组合 */
}cm_sound_info_t;


/**
 * @brief 音频buffer信息结构体
 */
typedef struct
{
    void * pcm_buffer;                          /*!< 总buffer, 大小为(buffer_size*buffer_num) */
    uint16_t block_size;                        /*!< BUFFER块大小 以字节为单位*/
    uint16_t buffer_size;                       /*!< BUFFER大小 以字节为单位*/
    uint8_t block_num;                          /*!< BUFFER块数量 */
    uint8_t buffer_num;                         /*!< BUFFER数量 */    
}cm_play_buffer_info_t;

/**
 * @brief 音频buffer信息结构体
 */
typedef struct
{
    void * pcm_buffer;                          /*!< 总buffer, 大小为(buffer_size*buffer_num) */    
    uint16_t buffer_size;                       /*!< BUFFER大小 以字节为单位*/
    uint16_t block_size;                        /*!< BUFFER块大小 以字节为单位*/
    uint8_t block_num;                          /*!< BUFFER块数量 */
}cm_record_buffer_info_t;


typedef union
{
    cm_play_buffer_info_t play_buffer_info;
    cm_record_buffer_info_t record_buffer_info;
}cm_pcm_buffer_info_t;


/**
 * @brief CODEC操作接口
 */
typedef struct
{
    int (*codec_init)(cm_codec_hw_info_t*);     /*!< CODEC初始化接口 */
    int (*codec_config)(cm_sound_info_t * audio_info, io_direction_t io_dir); /*!< CODEC配置接口，可以重复配置，不用再初始化 */
    int (*codec_start)(io_direction_t);                /*!< CODEC启动接口 */
    int (*codec_stop)(io_direction_t);                 /*!< CODEC停止接口 */
    int (*codec_ioctl)(io_direction_t, uint32_t, uint32_t, uint32_t);  /*!< CODEC扩展控制接口 */
}cm_codec_interface_t;


typedef struct
{
    iis_base_t IISx;                            /*!< 指向CODEC使用到的IIS基地址 */

    IIS_Mode_Sel_t iis_mode_sel;                /*!< 指IIS工作模式：master or slave */
    IIS_Mclk_Fs_t over_sample;                  /*!< 过采样率 */
    IIS_Src_Source_t clk_source;                /*!< IIS时钟源 */
    IIS_Mclk_Mode_t mclk_out_en;                /*!< mclk是否从PAD输出 */
    iis_data_format_t iis_data_format;          /*!< IIS数据对齐方式 */
    IIS_Sck_Lrck_Wid_t sck_lrck_ratio;          /*!< SCK与采样率的比值 */
    IIS_SckLrck_Mode_t scklrck_out_en;          /*!< SCK/LRCK是否从PAD输出 */
    uint32_t outside_mclk_fre;                  /*!< 外部送入的MCLK的时钟频率，只在使用外部MCLK做IIS时钟源时有效 */
    union{
        iis_tx_channal_t tx_cha;                /*!< IIS发送tx通道选择 */
        iis_rx_channal_t rx_cha;                /*!< IIS接收rx通道选择 */
    };
}cm_iis_info_t;


/**
 * @brief CODEC相关信息结构体
 */
typedef struct codec_hw_info_str
{
    iic_base_t IICx;                            /*!< IIC基地址 */
    cm_iis_info_t input_iis;                    /*!< 输入IIS相关信息 */
    cm_iis_info_t output_iis;                   /*!< 输出IIS相关信息 */
    cm_codec_interface_t codec_if;              /*!< CODEC操作接口 */
    inner_codec_adc_config_t codec_gain;            /*!< CODEC增益初始设置 */
}cm_codec_hw_info_t;


extern void cm_init();

/**
 * @brief 注册codec
 *
 * @param codec_index CODEC注册的索引号，在访问codec_manager其他接口时，用此索引号指定要操作的CODEC。
 * @param codec_hw_info 保存了注册的CODEC的相关信息的结构体。
 * @note 索引号不能大于最大可注册数量(MAX_CODEC_NUM),也不能重复注册同一个索引号。
 * @return 注册是否成功，0：成功；其他值：失败，有可能指定的codec_index超出范围，或者已经被注册过了。
 */
extern int cm_reg_codec(int codec_index, cm_codec_hw_info_t * p_codec_hw_info);


/**
 * @brief 注册指定CODEC的回调函数，用于异步调用方式，CODEC会在特定事件发生时，调用此函数与调用者通信。
 * @param codec_index CODEC注册的索引号，在访问codec_manager其他接口时，用此索引号指定要操作的CODEC。
 * @param callback_func CODEC事件回调函数。
 * @return 注册是否成功，0：成功；其他值：失败，有可能指定的codec_index超出范围。
 */
extern int cm_register_codec_callback(int codec_index, void (*callback_func)(void));

extern int cm_config_pcm_buffer(int codec_index, io_direction_t io_dir, cm_pcm_buffer_info_t * pcm_buffer_info);

extern int cm_config_codec(int codec_index, io_direction_t io_dir, cm_sound_info_t * sound_info);

extern int cm_start_codec(int codec_index, io_direction_t io_dir);

extern int cm_stop_codec(int codec_index, io_direction_t io_dir);

extern int cm_read_codec(int codec_index, uint32_t * data_addr, uint32_t * data_size);

extern int cm_write_codec(int codec_index, void * pcm_buffer,uint32_t wait_tick);

extern void cm_get_pcm_buffer(int codec_index,uint32_t* ret_buf,uint32_t wait_tick);

extern int cm_release_pcm_buffer(int codec_index, io_direction_t io_dir, void * pcm_buffer);

extern int cm_set_codec_dac_gain(int codec_index, cm_cha_sel_t cha, int gain);

extern int cm_set_codec_adc_gain(int codec_index, cm_cha_sel_t cha, int gain);

extern int cm_set_codec_alc(int codec_index, cm_cha_sel_t cha, FunctionalState alc_enable);

extern int cm_set_codec_dac_enable(int codec_index, int channel, FunctionalState en);

extern int cm_get_codec_empty_buffer_number(int codec_index, io_direction_t io_dir);

extern int cm_get_codec_busy_buffer_number(int codec_index, io_direction_t io_dir);

int cm_set_codec_mute(int codec_index, io_direction_t io_dir, int channel_flag, FunctionalState en);

extern int icodec_init(cm_codec_hw_info_t * codec_hw_info);
extern int icodec_start(io_direction_t io_dir);
extern int icodec_config(cm_sound_info_t * audio_info, io_direction_t io_dir);
extern int icodec_stop(io_direction_t io_dir);
extern int icodec_ioctl(io_direction_t io_dir, uint32_t param0, uint32_t param1, uint32_t param2);

#endif

