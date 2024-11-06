/**
  ******************************************************************************
  * @file    ci_doa.h
  * @version V1.0.0
  * @date    2019.07.09
  * @brief 
  ******************************************************************************
  **/
#ifndef CI_DOA_H
#define CI_DOA_H
#include "../ci_alg_malloc.h"
/**
 * @addtogroup doa
 * @{
 */

typedef struct
{
    //初始化参数
    int distance;
    int min_frebin;
    int max_frebin;
    int samplerate;
}doa_config_t;

#ifdef __cplusplus
extern "C" {
#endif


     /**
     * @brief  获取算法版本信息
     * 
     * @return int 版本id, 整数型，例如返回10000, 表示版本号为 1.0.0;
     */
    int ci_doa_version( void );
    
    
    /**
     * @brief  初始化当前算法模块
     * 
     * @param distance    双麦间距, 只能是doa_distance_e枚举里的数值;<br>
     * @return void*      若返回空NULL，则表示模块创建失败，否则表示创建成功;
     */
    void* ci_doa_create( void* module_config);


    /**
    * @brief 释放当前算法模块
    * 
    * @param handle  模块句柄，参数为来自 ci_doa_create()的返回值;
    * @return int    返回0表示成功，否则表示失败;
    */
    int ci_doa_free( void  *handle );


    int ci_doa_deal(void *handle,float **fft_in );


    /**
     * @brief   复位本算法模块，为下次调用ci_doa_get()做准备；<br>
     *          我们只调用ci_doa_reset()一次, 然后调用ci_doa_deal()很多次，
     *          一直到我们可以从ci_doa_get()可以得到角度信息
     * 
     * @param handle  模块句柄，参数为来自 ci_doa_create()的返回值; 
     * @return int    返回0, 表示调用成功;返回其他值，表示失败; 
     */
    int ci_doa_reset( void *handle );


    /**
     * @brief   获取定向的结果, 该结果是连续多帧ci_doa_deal()处理后的定向结果；
     *          请结合ci_doa_reset()一起使用;  
     * 
     * @param handle  模块句柄，参数为来自 ci_doa_create()的返回值; 
     * @return int    返回-1, 表示当前帧没有角度信息输出;返回其他值(0~180)，表示当前帧计算得到的角度信息; 
     */
    int ci_doa_get( void *handle );


#ifdef __cplusplus
}
#endif

/** @} */

#endif /* CI_DOA_H */
