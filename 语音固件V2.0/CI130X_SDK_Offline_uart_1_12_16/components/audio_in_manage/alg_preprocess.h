/**
  ******************************************************************************
  * @file    alg_preprocess.h
  * @version V1.0.0
  * @date    2019.04.04
  * @brief 
  ******************************************************************************
  */

#ifndef ALG_PREPROCESS_H
#define ALG_PREPROCESS_H

#include <stdbool.h>



typedef enum
{
  MICL = 0,
  MICR,
  REFL,
  REFR,
  DST1,
  DST2
} audio_channel_model;
typedef struct
{
  bool iis_out_enable;                   //是否通过iis输出16k音频信号。
  audio_channel_model iis_left_channel;  //左通道输出音频。
  audio_channel_model iis_right_channel; //右通道输出音频。
  bool vad_mark_enable;                  //是否附带vad标签，默认左通道输出vad标签。
  bool ssp_dst_cover_micl_enble;         //处理过后的音频dst覆盖原始micl数据，由cpu核传输到bnpu核,16k采样数据
} iis_out_audio_config_t;

typedef struct
{
  int frame_length;    //输入的数据帧长
  int mic_channel_num; //mic：麦克风信号的通道数
  int ref_channel_num; //ref：参考信号的通道数
//   short *mic_data;     //麦克风信号的数据地址
//   short *ref_data;     //参考信号的数据地址
  //mic_model_enum mic_model;
} audio_capture_t;

typedef void *(*func_init_t)(void *, void *);
typedef int (*func_process_t)(void *, void *);

typedef struct
{
  func_init_t func_init;       //ssp模块初始化函数
  func_process_t func_process; //ssp模块运行函数
  void *module_config;         //ssp模块配置参数结构体地址
  void *handle;                //ssp模块句柄
} ci_ssp_st;

//ssp_模块注册
typedef struct
{
    audio_capture_t* audio_capture;
	ci_ssp_st* ci_ssp;
	int module_num;                //ssp模块句柄
} ci_ssp_registe_t;

typedef struct ci_ssp_config_st {
	//ci_ssp_st hpf;
	ci_ssp_st alc_auto_switch;		//alc_动态alc
	ci_ssp_st stft;					      //fft
	ci_ssp_st aec;					      //AEC
	ci_ssp_st doa;					      //DOA
	ci_ssp_st dereverb;				    //DEREVERB
	ci_ssp_st bf;
	ci_ssp_st denoise;				    //DENOISE
	ci_ssp_st istft;				      //ifft
	ci_ssp_st iis_out_audio;		  //音频输出模块
}ci_ssp_config_t;


#ifdef __cplusplus
extern "C"
{
#endif
  /**
 * @brief stft短时傅里叶变换模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return stft_istft模块句柄,该句柄与音频处理数据流句柄复用.
 */
  void *ci_stft_module_init(void *module_config, void *wrapfft_audio_t);
  /**
 * @brief stft模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  int ci_stft_processing(void *handle, void *wrapfft_audio_t);
  /**
 * @brief istft短时傅里叶逆变换模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return stft_istft模块句柄.
 */
  void *ci_istft_module_init(void *module_config, void *wrapfft_audio_t);
  /**
 * @brief istft模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  int ci_istft_processing(void *handle, void *wrapfft_audio_t);
  /**
 * @brief denoise模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return denoise模块句柄.
 */
  void *ci_denoise_module_init(void *module_config, void *wrapfft_audio_t);
  /**
 * @brief denoise模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  int ci_denoise_processing(void *handle, void *wrapfft_audio_t);

  /**
* @brief aec模块初始化.
*
* @param module_config 模块参数配置.
* @param wrapfft_audio_t 音频处理数据流句柄.
* @return aec模块句柄.
*/
  void* ci_bf_module_init(void* module_config, void* wrapfft_audio_t);
  /**
 * @brief beamforming模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return beamforming模块句柄.
 */

  int ci_bf_processing(void *handle, void* wrapfft_audio_t);
  /**
 * @brief beamforming模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  void* ci_dereverb_module_init(void* module_config, void* wrapfft_audio_t);
  int ci_dereverb_processing(void* handle, void* wrapfft_audio_t);

  void* ci_doa_module_init(void* module_config, void* wrapfft_audio_t);
  int ci_doa_processing(void* handle, void* wrapfft_audio_t);

  void* ci_aec_module_init(void* handle, void* wrapfft_audio_t);
  /**
* @brief aec模块执行.
*
* @param module_config 模块参数配置.
* @param wrapfft_audio_t 音频处理数据流句柄.
* @return 返回运行状态.
*/
  int ci_aec_processing(void* handle, void* wrapfft_audio_t);

  /**
 * @brief 语音前端信号处理ssp模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return ssp模块句柄.
 */
  void ci_ssp_init();
  /**
 * @brief ssp模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  int ci_ssp_processing();
  /**
 * @brief 识别特征提取stft模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return ssp模块句柄.
 */
  void ci_asr_fe_stft_init(void);
  /**
 * @brief 识别特征提取asr_fe模块执行.
 * 
 * @param src_data 输入数据.
 * @param wrapfft_audio_t 输入帧长度.
 * @return 返回运行状态.
 */
  int ci_asr_fe_processing(short *src_data, int frame_length);
  /**
 * @brief iis_out_audio模块初始化.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return iis_out_audio模块句柄.
 */
  void *ci_iis_out_audio_module_init(void *module_config, void *wrapfft_audio_t);
  /**
 * @brief iis_out_audio模块执行.
 * 
 * @param module_config 模块参数配置.
 * @param wrapfft_audio_t 音频处理数据流句柄.
 * @return 返回运行状态.
 */
  int ci_iis_out_audio_processing(void *handle, void *wrapfft_audio_t);

  void *ci_alc_auto_switch_module_init(void *module_config, void *wrapfft_audio_t);

  int ci_alc_auto_switch_processing(void *handle, void *wrapfft_audio_t);
  void set_ssp_registe(audio_capture_t* audio_capture, ci_ssp_st* ci_ssp, int module_num);
#ifdef __cplusplus
}
#endif

#endif /* ALG_PREPROCESS_H */
