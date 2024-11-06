/**
 * @file ci_dereverb.h
 * @brief 
 * @version V1.0.0
 * @date 2021.01.04
 * 
 * @copyright Copyright (c) 2021 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef CI_DEREVERB_H
#define CI_DEREVERB_H

/**
 * @addtogroup dereverb
 * @{
 */

/*! 降混响起始频率*/  
#define START_HZ      (0)
/*! 降混响截止频率*/  
#define END_HZ        (8000)                       

typedef struct
{
    //初始化参数
    float startHz;
    float endHz;
}dereverb_config_t;


#ifdef __cplusplus
extern "C" {
#endif
    /**
     * @brief  获得当前版本号
     * 
     * @return int, 版本号id,例如10000表示1.00.00;
     */
    int ci_dereverb_version( void );

    /**
     * @brief        初始化模块
     * 
     * @return int   不等于NULL: 初始化成功, 其他：初始化存在异常 
     */
    void* ci_dereverb_create(void* module_config);
	
	/**
     * @brief        输出频率
     *
     * @return int   无
     */
	 
    /**
     * @brief        降混响运算
     *
     * @return int   不等于0: 运算失败, 其他：正常运算
     */
    int ci_dereverb_deal(void* handle, float** fft_in, float** fft_out);




#ifdef __cplusplus
}
#endif


#endif