/**
  ******************************************************************************
  * @file    led_light_control.h 
  * @version V1.0.0
  * @date    2018.05.23
  * @brief
  ******************************************************************************
  **/

#ifndef _LED_LIGHT_CONTROL_H_
#define _LED_LIGHT_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @ingroup led
 * @defgroup blink_light 眨眼灯
 * @brief use CI130X PWM control LED for toys blink light
 * @{
 */
extern void blink_light_init(void);
extern void blink_light_on(void);
/** @} */  

/**
 * @ingroup led
 * @defgroup night_light 小夜灯
 * @brief use CI130X PWM control LED for night light
 * @{
 */
extern void night_light_init(void);
extern void night_light_set_brightness(uint32_t onoff,uint32_t val);
/** @} */  


#ifdef __cplusplus
}
#endif


#endif
  
