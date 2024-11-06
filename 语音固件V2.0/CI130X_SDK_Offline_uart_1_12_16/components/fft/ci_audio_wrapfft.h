#ifndef CI_AUDIO_WRAPFFT_H
#define CI_AUDIO_WRAPFFT_H
#include <stdbool.h>
#include "romlib_runtime.h"

#define HPF_CUT_OFF_FREQ (1) //fft_hpf的截止频率31.25*HPF_CUT_OFF_FREQ
typedef struct
{
    float real;
    float image;
} Complex;
typedef struct
{
    //初始化参数
    int sample_frequency;          //采样率
    int frame_size;                //窗长
    int frame_shift;               //帧移
    int fft_frm_size;              //fft输入有效长度的点数
    int fft_size;                  //fft的正频分量点数+直流分量点数
    int result_out_channel;        //ifft输出通道数
    
    int psd_compute_channel_num; //计算psd通道数；
    bool time_pre_emphasis_enable; //是否进行时域预加重，16k采样率时使用，32k采样率使用频域的预加重。
    bool downsampled_enable;       //是否进行32k-16k降采样处理
    bool fe_psd_enable;            //是否计算psd，以计算特征
    
} stft_istft_config_t;
typedef struct
{
    stft_istft_config_t *module_config;
    int mic_channel_num;
    int ref_channel_num;

    int iis_input_frame_len; //输入帧长度,32k采样320个采样点,16k采样160个采样点，该值的大小与stft中的帧移frame_shift大小一致！

    int frame_overlap_size;
    int fft_size; //用于后续算法模块
    short sg_last_pcm;
    short *mic[4];
    short *ref[2];

    short *frame_mic_data[4];
    short *frame_ref_data[2];

    short *dst[2]; //双通道输出使用

    float *fft_mic_out[4];
    float *fft_ref_out[2];
    //float *window;
    short *fix_window;
    float *ifft_result_buf;
    float *ifft_tmp_buf[2];
    float *adjust_gain;
    float *psd[2];
    float *fft_buf_tmp;

    riscv_rfft_fast_instance_f32 S;
} ci_wrapfft_audio;

#ifdef __cplusplus
extern "C"
{
#endif

    int ci_stft_istft_module_version(void);
    void *ci_stft_create(void *module_config, void *wrapfft_audio_t);
    void *ci_istft_create(void *module_config, void *wrapfft_audio_t);

    int ci_istft_deal(void *handle, void *wrapfft_audio);
    int ci_stft_deal(void *handle, void *wrapfft_audio);

    void ci_stft_istft_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* CI_AUDIO_WRAPFFT_H */
