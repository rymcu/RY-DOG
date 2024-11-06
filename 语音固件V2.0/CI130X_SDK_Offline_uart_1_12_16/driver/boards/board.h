#ifndef __CI_BOARD_H__
#define __CI_BOARD_H__

#include "ci130x_system.h"
#include "ci130x_gpio.h"
#include "ci130x_pwm.h"

/*color light pwm config*/
extern pwm_base_t       color_light_r_pwm_name;
extern pwm_base_t       color_light_g_pwm_name;
extern pwm_base_t       color_light_b_pwm_name;

/*night light pwm config*/
extern pwm_base_t       night_light_w_pwm_name;

/**
 * @brief 引脚复用配置为UART功能
 *
 * @param UARTx UART组 : UART0, UART1
 */
void pad_config_for_uart(UART_TypeDef *UARTx);


/**
 * @brief 引脚复用配置为IIS功能
 *
 * @param IISx IIS组 : IIS1, IIS2
 */
void pad_config_for_iis(void);

/**
 * @brief 引脚复用配置为PDM功能
 *
 */
void pad_config_for_pdm(void);

/**
 * @brief 引脚复用配置为GPIO,用于控制功放使能
 *
 */
void pad_config_for_power_amplifier(void);

/**
 * @brief 开启功放使能
 *
 */
void power_amplifier_on(void);

/**
 * @brief 关闭功放使能
 *
 */
void power_amplifier_off(void);

/**
  * @功能:引脚复用配置为IIC功能
  * @
  */
void pad_config_for_i2c(void);

/**
  * @功能:引脚复用配置为color light功能
  * @
  */
void pad_config_for_color_light(void);

/**
  * @功能:night light功能关闭
  * @
  */
void night_light_disable(void);

/**
  * @功能:night light功能打开
  * @
  */
void night_light_enable(void);

/**
  * @功能:引脚复用配置为night light功能
  * @
  */
void pad_config_for_night_light(void);

/**
  * @功能:blink light电平设置
  * @
  */
void blink_light_on_off(uint8_t on_off);

/**
  * @功能:引脚复用配置为blink light功能
  * @
  */
void pad_config_for_blink_light(void);

/**
  * @功能:vad light功能关闭
  * @
  */
void vad_light_on(void);

/**
  * @功能:vad light功能打开
  * @
  */
void vad_light_off(void);

/**
  * @功能:引脚复用配置为vad light功能
  * @
  */
void pad_config_for_vad_light(void);

/**
 * @brief 录音codec注册
 *
 */
void audio_in_codec_registe();


/**
 * @brief 语音前处理使用IIS输出功能的初始化
 * 
 */
void audio_pre_rslt_out_codec_init(void);

#endif //__CI_BOARD_H__

