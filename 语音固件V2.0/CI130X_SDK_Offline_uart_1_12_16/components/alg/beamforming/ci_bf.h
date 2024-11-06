
#ifndef CI_BF_H
#define CI_BF_H

#define unit32_t     unsigned int

typedef struct
{
    //初始化参数
    int distance;               //设置麦克风间距
    int angle;                 //设置定向语音增强的角度
    int freq;                 //设置通道选取时的最低频点
    int frame_wkup;          //根据唤醒词长度设置唤醒词语音段帧长
    int frame_rt;           //根据唤醒词识别响应时间设置响应时间短帧长
    int wkup_result_thr;   //设置唤醒词最低分值
}bf_config_t;

#ifdef __cplusplus
extern "C" {
#endif
  
    /**
     * @brief  获取算法版本信息
     * @return int 版本id, 整数型，例如返回10000, 表示版本号为 1.0.0;
     */
    int ci_bf_version( void );

    /**
     * @brief  初始化当前算法模块
     * @return void*      若返回空NULL，则表示模块创建失败，否则表示创建成功;
     */
    void* ci_bf_create(void* module_config);
	
    /**
     * @brief  bf算法处理函数
     * @param handle 模块句柄，参数为来自ci_bf_create()的返回值,leftbuff,rightbuff分别为双麦左右通道经stft处理后的数据; 
     * @return int   返回0表示成功，否则表示失败;
     */
    int ci_bf_deal( void *handle,float *leftbuff,float *rightbuff, float **fftout_data);

    /**
     * @brief  获取bf算法处理后输出状态;
     * @return int，0表示为原始音频数据，1表示bf处理后的数据;
     */
    int get_bf_process_state(void);
  

   
  

#ifdef __cplusplus
}
#endif    

#endif