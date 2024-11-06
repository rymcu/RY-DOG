/**
  ******************************************************************************
  * @file    ci_doa_apply.h
  * @author  lt
  * @version V1.0.0
  * @date    2022.03.03
  * @brief 
  ******************************************************************************
  **/

#ifndef CI_DOA_APPLY_H
#define CI_DOA_APPLY_H

typedef enum
{
    DOA_MODEL_WAKE_UP = 0,
    DOA_MODEL_CMD,
} doa_apply_module;

typedef struct
{
    //初始化参数
    doa_apply_module doa_model;
    int rad_num;
    int count_lenth;
    int doa_resolution;
}doa_apply_config_t;


#ifdef __cplusplus
extern "C" {
#endif
    void* doa_apply_init(doa_apply_config_t doa_init_config);
    int ci_doa_deal_for_application(void *doa, void* doa_apply, float **fft_in, doa_apply_config_t doa_apply_config);



#ifdef __cplusplus
}

   
#endif
#endif