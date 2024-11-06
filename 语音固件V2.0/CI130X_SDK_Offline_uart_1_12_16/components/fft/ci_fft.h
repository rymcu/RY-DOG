/**
 * @file ci_fft.h
 * @brief 
 * @version V1.0.0
 * @date 2019.07.09
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef CI_FFT_H
#define CI_FFT_H

#include "romlib_runtime.h"

/**
 * @addtogroup fft
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif


     /**
     * @brief  获取版本信息
     * 
     * @return int 版本id, 整数型，例如返回10000, 表示版本号为 1.0.0;
     */
    int ci_fft_version( void );


    /**
     * @brief  FFT初始化;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要考虑互斥机制，以保证该
     *         函数不会重入，并且该函数初始化后才能使用本模块的FFT/IFFT功能;<br>
     *         目前不支持ci_software_fft_w256_s128_init();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w512_s256_init( void );

    /**
     * @brief  FFT初始化;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要考虑互斥机制，以保证该
     *         函数不会重入，并且该函数初始化后才能使用本模块的FFT/IFFT功能;<br>
     *         目前不支持ci_software_fft_w256_s128_init();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w400_s160_init( void );

    /**
     * @brief  FFT初始化;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要考虑互斥机制，以保证该
     *         函数不会重入，并且该函数初始化后才能使用本模块的FFT/IFFT功能;<br>
     *         目前不支持ci_software_fft_w256_s128_init();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w256_s128_init( void );


    /**
     * @brief  FFT卸载;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要保证该
     *         函数不会重入，并且与模块的其它接口函数互斥;<br>
     *         目前不支持ci_software_fft_w256_s128_uninit();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w512_s256_uninit( void );

    /**
     * @brief  FFT卸载;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要保证该
     *         函数不会重入，并且与模块的其它接口函数互斥;<br>
     *         目前不支持ci_software_fft_w256_s128_uninit();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w400_s160_uninit( void );

    /**
     * @brief  FFT卸载;<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似
     *         多任务/线程环境下并不安全，用户需要保证该
     *         函数不会重入，并且与模块的其它接口函数互斥;<br>
     *         目前不支持ci_software_fft_w256_s128_uninit();
     *
     * @return int, 返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w256_s128_uninit( void );


    /**
     * @brief  软件FFT模块(注意该接口需要使用 2KB的 stack空间，用户必须确保不会出现stack overflow)
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         目前不支持 ci_software_fft_w256_s128();
     * 
     * @param audio_data  输入参数,  音频输入的数据（PCM格式，采样点位宽为16bit)<br>
     *                    例如，*_w512_s256, 512个采样点;
     * @param window_data 输入参数,  FFT需要的窗数据, 浮点类型;<br>
     *                    例如，*_w512_s256, 需要的窗的数据长度为512个;
     * @param result      输出参数, FFT的输出, 复数格式，共256个，complex0,complex1 ... complex255;<br>
     *                    复数的定义为typedef struct { float real; float image; } Complex;
     * @return int        返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w512_s256( const short *audio_data, const float *window_data, float *result );

    /**
     * @brief  软件FFT模块(注意该接口需要使用 2KB的 stack空间，用户必须确保不会出现stack overflow)
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         目前不支持 ci_software_fft_w256_s128();
     * 
     * @param audio_data  输入参数,  音频输入的数据（PCM格式，采样点位宽为16bit)<br>
     *                    例如，*_w512_s256, 512个采样点;
     * @param window_data 输入参数,  FFT需要的窗数据, 浮点类型;<br>
     *                    例如，*_w512_s256, 需要的窗的数据长度为512个;
     * @param result      输出参数, FFT的输出, 复数格式，共256个，complex0,complex1 ... complex255;<br>
     *                    复数的定义为typedef struct { float real; float image; } Complex;
     * @return int        返回值请查看枚举 enum _retval
     */
    int ci_software_fft_w400_s160( const short *audio_data, const float *window_data, float *result );  

    /**
     * @brief  软件FFT模块(注意该接口需要使用 2KB的 stack空间，用户必须确保不会出现stack overflow)
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         目前不支持 ci_software_fft_w256_s128();
     * 
     * @param audio_data  输入参数,  音频输入的数据（PCM格式，采样点位宽为16bit)<br>
     *                    例如，*_w512_s256, 512个采样点;
     * @param window_data 输入参数,  FFT需要的窗数据, 浮点类型;<br>
     *                    例如，*_w512_s256, 需要的窗的数据长度为512个;
     * @param result      输出参数, FFT的输出, 复数格式，共256个，complex0,complex1 ... complex255;<br>
     *                    复数的定义为typedef struct { float real; float image; } Complex;
     * @return int        返回值请查看枚举 enum _retval
     */  
    int ci_software_fft_w256_s128( const short *audio_data, const float *window_data, float *result );



    /**
     * @brief  软件IFFT<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         暂时不支持ci_software_ifft_w256_s128();
     * 
     * @param fft    输入参数, IFFT的输入, 复数格式，<br>
     *               例如，*_w512_s256, 共256个，complex0,complex1 ... complex255,
     *               复数的定义为typedef struct { float real; float image; } Complex;
     * @param result 输出参数, IFFT的输出，浮点格式，例如，*_w512_s256, 输出512个浮点数;
     * @return int   返回值请查看枚举 enum _retval
     */
    int ci_software_ifft_w512_s256( const float *fft, float *result );

    /**
     * @brief  软件IFFT<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         暂时不支持ci_software_ifft_w256_s128();
     * 
     * @param fft    输入参数, IFFT的输入, 复数格式，<br>
     *               例如，*_w512_s256, 共256个，complex0,complex1 ... complex255,
     *               复数的定义为typedef struct { float real; float image; } Complex;
     * @param result 输出参数, IFFT的输出，浮点格式，例如，*_w512_s256, 输出512个浮点数;
     * @return int   返回值请查看枚举 enum _retval
     */
    int ci_software_ifft_w400_s160( const float *fft, float *result );

    /**
     * @brief  软件IFFT<br>
     *         *_w512_s256_*: 表示帧长为 512, 帧移为 256，*_wx_sy, 类似;<br>
     *         暂时不支持ci_software_ifft_w256_s128();
     * 
     * @param fft    输入参数, IFFT的输入, 复数格式，<br>
     *               例如，*_w512_s256, 共256个，complex0,complex1 ... complex255,
     *               复数的定义为typedef struct { float real; float image; } Complex;
     * @param result 输出参数, IFFT的输出，浮点格式，例如，*_w512_s256, 输出512个浮点数;
     * @return int   返回值请查看枚举 enum _retval
     */
    int ci_software_ifft_w256_s128( const float *fft, float *result );

    int fft_dsp(float* fft_in,float *fft_out,int len);

    int ci_software_fft_w512( const short *audio_data, float *result );
    
    int ci_software_fft_init(riscv_rfft_fast_instance_f32 *S,int len);
	int ci_software_fft(riscv_rfft_fast_instance_f32 *S,float *in, float *result );
	int ci_software_ifft(riscv_rfft_fast_instance_f32 *S,float* fft, float* result);
   


#ifdef __cplusplus
}
#endif    
    
/** @} */

#endif /* CI_FFT_H */

