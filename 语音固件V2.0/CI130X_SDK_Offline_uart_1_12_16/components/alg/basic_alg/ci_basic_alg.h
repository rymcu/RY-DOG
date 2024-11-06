/**
  ******************************************************************************
  * @file    ci_basic_alg.h
  * @author  lt
  * @version V1.0.0
  * @date    2019.07.09
  * @brief 
  ******************************************************************************
  **/

#ifndef CI_BASIC_ALG_H
#define CI_BASIC_ALG_H


typedef enum
{
  DENOISE = 0,
  DEREVERB,
  BEAMFORMING
} alg_th_module;
typedef struct
{
  alg_th_module th_module;     //算法模块判断
  float start_Hz;              //算法模块处理起始频率 单位Hz
  float end_Hz;                //算法模块处理降噪结束频率 单位Hz
  float alg_throshold;         //门限的值
  int set_alg_thr_window_size; //判断窗长

} alg_thr_config_t;

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {  
    alg_thr_config_t*  alg_thr_config;
    float* sg_eng;
    float threshold;
    
    float default_alg_threshold;
    int decide_frame_len;
    int max_idx;
  } alg_th_s;
  //门限控制
  alg_th_s *alg_th_create(const alg_thr_config_t* alg_thr_config);
  bool ci_ssp_module_thr_control(alg_th_s *alg_th, float *fft);
  void reset_threshold_for_alc_switch(alg_th_s *alg_th);
  //动态alc相关接口
  void inner_codec_alc_disable_port(void);
  void inner_codec_left_alc_enable_port(void);
  int get_vad_idle_flag_port(void);

  //aec门限接口
  void* set_aec_gain(void* wrapfft_audio_t);
  bool ci_aec_application_enable(void *wrapfft_audio_t);
  bool get_aec_process_enable_flag();
#ifdef __cplusplus
}
#endif

#endif /* CI_BASIC_ALG_H */
