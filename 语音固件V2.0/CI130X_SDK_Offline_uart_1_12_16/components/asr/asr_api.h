/**
 * @file asr_api.h
 * @brief asr 系统外部接口使用及说明
 * @version V1.0.1
 * @date 2019-08-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __ASR_API_H
#define __ASR_API_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup asr
 * @{
 */

/**
 * @brief 配置 语音数据缓存信息给 asr 系统 
 * 
 * @param buf_base_ptr 语音缓存基地址
 * @param frm_nums 语音缓存帧数
 * @param frm_shift 语音缓存帧移
 * @return int 0：配置成功；-1：配置失败
 */
extern int asrtop_asrpcmbuf_mem_cfg(unsigned int buf_base_ptr,int frm_nums,int frm_shift);


/**
 * @brief asr 系统任务，信号量，队列创建，会配置 asr 默认参数
 * 
 * @return int 0 asr 系统任务创建成功；-1 asr 系统任务创建失败
 */
extern int asrtop_taskmanage_create(void);

/**
 * @brief asr 系统重启，第一次可直接调用 asrtop_asrtop_asr_system_start 系统的启动；
 * asr启动后，禁止单独调用，必须先调用 asrtop_asr_system_release 成功后，才可调用此函数。
 * 
 * @param lg_model_addr0 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 * 是否加载
 * @param lg_model_size0 语言模型大小
 * @param ac_model_addr 声学模型基地址，支持flash,PSRAM 映射地址，内部自行判断是否配置qspi
 * 对应dnn 模式参数
 * @param ac_model_size 声学模型大小
 * @param pdata pdata[0]: dnn输出位宽配置，1: 8bit,其他16bit(默认16bit),
 *              pdata[1]: 0: pdata[2],pdata[3] 参数无效，1：pdata[2],pdata[3],pdata[4] 参数有效，第二个网络语言模型参数
 *              pdata[2]: lg_model_addr1 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 *              pdata[3]: lg_model_size1 语言模型大小   
 *              pdata[4]: 创建启动的模型索引号
 *              pdata[5]~:预留
 * @return int 0 asr 系统加载模型启动成功；-1 asr 系统加载模型启动失败
 */
extern int asrtop_asr_system_start(unsigned int lg_model_addr0,unsigned int lg_model_size0,
    unsigned int ac_model_addr,unsigned int ac_model_size,void* pdata);


/**
 * @brief 释放 asr 系统，fe输出缓存，梅尔系数缓存，cmvn内部运算缓存,窗函数缓存，
 * dnn输出缓存，decoder用到的缓存 全部释放，释放成功后 asr 系统不处理语音(不影响
 * flash的使用)，必须调用 asrasrtop_asr_system_start恢复 asr的运行
 * 
 * @return int 0 asr 系统释放成功；-1 asr 系统释放失败
 */
extern int asrtop_asr_system_release(void);    
    
/**
 * @brief asr 系统模型创建，独立使用，模型创建不影响外部flash使用
 * 
 * @param lg_model_addr 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 * 是否加载
 * @param lg_model_size 语言模型大小
 * @param ac_model_addr 声学模型基地址，支持flash,PSRAM 映射地址，内部自行判断是否配置qspi
 * 对应dnn 模式参数
 * @param ac_model_size  声学模型大小
 * @param pdata pdata[0]: dnn输出位宽配置，1: 8bit,其他16bit(默认16bit),其他预留
 *              pdata[1]: 0: pdata[2],pdata[3] 参数无效，1：pdata[2],pdata[3],pdata[4] 参数有效，第二个网络语言模型参数
 *              pdata[2]: lg_model_addr1 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 *              pdata[3]: lg_model_size1 语言模型大小   
 *              pdata[4]: 创建启动的模型索引号
 *              pdata[5]~:预留            
 * @return int 0 创建成功；-1 创建失败
 */
extern int asrtop_asr_system_create_model(unsigned int lg_model_addr,unsigned int lg_model_size,
    unsigned int ac_model_addr,unsigned int ac_model_size,void* pdata);
    

/**
 * @brief 暂停 运行 asr 系统，暂停成功后，asr 系统内部余留语音相关信息会被清掉，
 * 不再处理语音（不影响flash的使用），直到 调用 asrtop_asr_system_continue
 * asr 系统 再运行，必须和asrtop_asr_system_continue 配对调用
 * 
 * @return int 0
 */
extern int asrtop_asr_system_pause(void);    
    
/**
 * @brief 继续 运行 asr 系统，必须调用 asrtop_asr_system_pause 成功后才能调用此函数
 * 禁止单独调用
 * @return int 0
 */
extern int asrtop_asr_system_continue(void);    

#if 0//可以通过信息共享模块获取此状态
/**
 * @brief 获取 asr 启动标志
 * 
 * @return int 0 启动未完成；1：启动完成
 */
extern int asrtop_is_startup_done(void);
#endif


/**
 * @brief Get the asrtop asrfrmshift 
 * 
 * @return int 帧移的长度，单位 PCM 点数
 */
extern int get_asrtop_asrfrmshift(void);

/**
 * @brief 发送请求 flash消息给dnn，必须和 释放配对使用
 * 
 * @return int  0成功，-1失败
 */
extern int send_requset_flash_msg_to_dnn(void);

/**
 * @brief 发送释放信号量给dnn ，只能flash 函数管理和请求flash 配对使用
 * 
 * @return int 0成功，-1失败
 */
extern int send_release_flash_semaphore_to_dnn(void);

/**
 * @brief 查询 asr 系统是否处于忙状态，处理语音
 * 
 * @return int 0 空闲，1忙
 */
extern int asrtop_sys_isbusy(void);

/**
 * @brief 获取 asr 系统 已经解码的帧数
 * 
 * @return short 帧数
 */
extern short asrtop_get_decode_pcm_finished_frame(void);

/**
 * @brief asrtop fe的 cmvn 迭代系数配置(调试用)
 * 
 * @param alpha 迭代系数(0~1)
 * @return int 0 配置成功,-1配置失败
 */
extern int asrtop_cmvn_update_weight_config(float alpha);

/**
 * @brief asrtop tdvad的 base_energy 系数 配置
 * 
 * @param base_energy 系数
 * @return int 0 配置成功,-1配置失败
 */
extern int asrtop_tdvad_base_energy_cfg(float base_energy); 


/**
 * @brief asr 系统模型内部语言模型网络索引切换
 * @param fst_idx 语言模型网络索引
 * @param pdata pdata[0]:dnn输出位宽配置，1: 8bit,其他16bit(默认16bit)
 *              pdata[2]~:预留 
 */
extern int asrtop_asr_switch_fst(int fst_idx,void* pdata);




/**
 * @brief asrtop tdvad的 vad end 条件帧数 配置
 * 
 * @param vadend_frames vad end 的帧数
 * @return int 0 配置成功,-1配置失败
 */
extern int asrtop_tdvad_vadend_frames_cfg(int vadend_frames); 

/**
 * @brief  asr识别打分模式配置
 * 
 * @param confidence_mode 配置模式，0 平均模式，-1 最大值模式 
 * @return int 0 配置成功,-1配置失败
 */
extern int asrtop_dynamic_confidence_mode_cfg(int confidence_mode);

/**
 * @brief  asr识别启动任务
 * 
 * @param 
 * @return 无
 */
extern void asr_system_startup_task(void* p);

/**
 * @brief  asr版本查询
 * 
 * @param version_buf 至少 80字节
 * @return 无
 */
int get_asr_sys_verinfo(char* version_buf);

/**
 * @brief  设置单个命令词的置信度和有效次数阈值
 * 
 * @param short confidence_thr（16-24）,short confidence_thr（>9）
 * @return 0
 */
 extern int set_asr_sigle_word_confidence_count_threshold(short confidence_thr, unsigned char valid_count_thr);
 /* @brief asr 系统模型创建，独立使用，模型创建不影响外部flash使用
 * 
 * @param lg_model_addr 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 * 是否加载
 * @param lg_model_size 语言模型大小
 * @param ac_model_addr 声学模型基地址，支持flash,PSRAM 映射地址，内部自行判断是否配置qspi
 * 对应dnn 模式参数
 * @param ac_model_size  声学模型大小
 * @param pdata pdata[0]: dnn输出位宽配置，1: 8bit,其他16bit(默认16bit)
 *              pdata[1]: 0: pdata[2],pdata[3] 参数无效，1：pdata[2],pdata[3],pdata[4] 参数有效，第二个网络语言模型参数
 *              pdata[2]: lg_model_addr1 语言模型基地址，支持flash，SRAM,PSRAM 映射的地址，内部会自行判断
 *              pdata[3]: lg_model_size1 语言模型大小   
 *              pdata[4]: 创建启动的模型索引号
 *              pdata[5]~:预留
 * @return int 0 创建成功；-1 创建失败
 */
int asrtop_asr_system_litecreate(unsigned int lg_model_addr,unsigned int lg_model_size,
    unsigned int ac_model_addr,unsigned int ac_model_size,void* pdata);

/**
 * @brief  
 * 
 * @param none
 * @return 0 成功，-1失败
 */    
extern int asr_dynamic_skip_close(void);

/**
 * @brief  
 *  必须 开启 USE_ASR_DY_SKP_EN 宏
 * @param none 
 * @return 0 成功，-1失败
 */
extern int asr_dynamic_skip_open(void);    
    
    
extern int dynmic_confidence_config(int min,int max,int step);
extern int dynmic_confidence_en_cfg(int en_cfg);
extern void asr_system_startup_task(void* p);

extern void decoder_port_inner_rpmsg_init(void);
extern void dnn_nuclear_com_outside_port_init(void);
extern void asr_top_nuclear_com_outside_port_init(void);
extern void vad_fe_nuclear_com_outside_port_init(void);
extern void codec_manage_inner_port_init(void);


/** @} */

#ifdef __cplusplus
}
#endif

#endif //__ASR_API_H

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/

