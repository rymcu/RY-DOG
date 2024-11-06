/**
 * @file ci_adapt_aec.h
 * @brief 
 * @version V1.0.0
 * @date 2019.07.09
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef CI_ADAPT_AEC_H
#define CI_ADAPT_AEC_H

/**
 * @addtogroup aec
 * @{
 */

typedef enum
{
    ENABLE_PLAYING_STATE_MODE=0,//根据播报状态进行aec控制
    COMPUTE_REF_AMPL_MODE,//根据参考幅值大小进行aec控制  
}aec_control_mode_t;
typedef struct
{
    //初始化参数
    int fft_size;//频域处理频点数
    int mic_channel_num;//麦克风信号通道数
    int ref_channel_num;//参考信号通道数
    
    int nlp_flag;//非线性处理模块选择模式，默认使用模式二， 0:不使用，1：使用模式一失真会更大，2：使用模式二失真要更小，3:先使用模式二后使用模式一
    int aggr_mode; //默认1，算法处理的效果等级：0，1，2，处理效果依次增强，算力内存也会依次增大
    
    float aec_gain;//增益数值
    float aec_enable_threshold;//门限值
    float aec_mic_div_ref_thr;//AEC门限基础值,参考与麦克风幅值比值

    int alc_off_codec_adc_gain_mic; 
    int alc_off_codec_adc_gain_ref;
    
    aec_control_mode_t aec_control_mode ;
}aec_config_t;

 
 
#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief       获得当前版本号
     * 
     * @return int  版本号id，例如10000表示1.00.00;
     */
    int ci_adapt_aec_version( void );

    /**
     * @brief                       创建模块
     * 
     * @param fft_size              默认设置512点的fft取对称256点; 
     * @param mic_channel_num       mic通道数默认为1;
     * @param ref_channel_num       参考信号的通道数默认为1;
     * @return void*                NULL:失败;其它:成功.
     */
    void* ci_aec_create(void* module_config);

    /**
     * @brief           释放模块
     * 
     * @param handle    模块句柄
     * @return          void
     */
    void* ci_adapt_aec_destroy( void *handle );

    
    /**
     * @brief                   AEC处理模块
     * @param handle            模块句柄;
    
     * @param fft_mic_in        mic信号的频域输入;
     * @param fft_ref_in        ref参考信号频域输入;
     * @param fft_out           AEC处理信号的频域输出;
     * @return int              0: AEC算法处理模块正常运行；其它：AEC算法处理过程出现异常.
     */
    int ci_adapt_aec_deal(void* handle, float** fft_mic_in, float** fft_ref_in, float** fft_out);
    void set_nlp_flag_api(int flag);
    void set_para_api(int alpha_l, int deta_d);
   
#ifdef __cplusplus
}
#endif

/** @} */


#endif /* CI_ADAPT_AEC_H */
