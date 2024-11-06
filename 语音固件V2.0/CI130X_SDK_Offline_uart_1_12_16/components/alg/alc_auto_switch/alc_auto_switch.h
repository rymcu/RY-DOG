/**
 * @file alc_auto_switch.h
 * @brief 
 * @version V1.0.0
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef ALC_AUTO_SWITCH_H
#define ALC_AUTO_SWITCH_H

typedef enum
{
      NON_STATIONARY_BAN = 0, //非稳态噪声严格控制，尽可能不进行alc_auto_switch操作
      STATIONARY_BAN,         //稳态噪声严格控制，尽可能不进行alc_auto_switch操作
      NORMAL,                 //正常模式，按设定的门限值控制
} alc_auto_switch_ban_mode_t;
typedef struct
{
      alc_auto_switch_ban_mode_t alc_auto_switch_ban_mode; //稳态与非稳态情况下分情况处理，可不进行alc_auto_switch操作
      int alc_auto_frame_size;                             //输入数据的采样点数，3代和2.5代为320，2代为256
      float thr_for_non_stationary_on;                     //非稳态噪声下对应的上门限，默认10000，//大概对应新闻alc on状态下的68dB。
      float thr_for_non_stationary_off;                    //非稳态噪声下对应的下门限，默认3000，//大概对应新闻alc off状态下的65dB。
      float thr_for_stationary_on;                         //稳态噪声下对应的上门限，默认7000   //大概对应稳态alc on状态下的65dB。即稳态下alc on的背景能量持续高于65dB，可以开启
      float thr_for_stationary_off;                        //稳态噪声下对应的下门限，默认1000     //大概对应稳态alc_off状态下的61~62dB。即稳态alc off的背景能力持续第一61~62dB，可以关闭
      int alc_off_codec_adc_gain;                          //alc关闭时pag增益默认20。
} alc_auto_switch_config_t;

/**
 * @addtogroup alc_auto_switch
 * @{
 */

/**
     * @brief  初始化函数
     *
     * @return void*, 返回指针;
     */

extern void *alc_auto_switch_create(void *module_config);
/**
     * @brief  获得当前版本号
     * 
     * @return int, 版本号id,例如10000表示1.00.00;
     */
extern int get_alc_auto_switch_version();
/**
      * @brief  调用函数
      * @param  hd handle
      * @return int, 运行成功返回0;
      */
extern int switch_alc_state_automatic(void *hd, short *pcm, int length);

/**
      * @brief  门限设置
      * @param  hd handle
      * @param  pcm_back_on     alc_on状态下进入alc off的背景能量门限值
      * @param  pcm_back_off    alc_off状态下进入alc on的背景能量门限值
      * @param  pcm_max_on      alc_on状态下进入alc off的最大能量门限值
      * @param  pcm_max_off     alc_off状态下进入alc on的最大能量门限值
      */
extern void set_th_for_alc_auto_switch(void *hd, float pcm_back_on, float pcm_back_off, float pcm_max_on, float pcm_max_off);

/**
     * @brief  门限设置
     * @param  hd handle
     * @param  pcm_back_on     alc_on状态下进入alc off的背景能量门限值
     * @param  pcm_back_off    alc_off状态下进入alc on的背景能量门限值
     */
extern void set_th_for_stationary(void *hd, float pcm_back_on, float pcm_back_off);

/**
      * @brief  门限设置
      * @param  hd handle
      * @param  pcm_max_on      alc_on状态下进入alc off的最大能量门限值
      * @param  pcm_max_off     alc_off状态下进入alc on的最大能量门限值
      */
extern void set_th_for_non_stationary(void *hd, float pcm_max_on, float pcm_max_off);

#endif //ALC_AUTO_SWITCH_H
