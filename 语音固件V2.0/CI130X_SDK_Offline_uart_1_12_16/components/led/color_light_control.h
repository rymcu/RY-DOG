/**
 * @file   color_light_control.h
 * @brief   A sample code use CI130X PWM control RGB or RGBW light, used HSV 
 *         color space.
 * @version V1.0.0
 * @date 2018-05-23
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef _COLOR_LIGHT_CONTROL_H_
#define _COLOR_LIGHT_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @ingroup led
 * @defgroup color_light 三色灯
 * @brief A sample code use CI130X PWM control RGB or RGBW light, used HSV color space
 * @{
 */

/**
 * @brief HSV color space
 * 
 */
typedef struct
{
    float h;        /*!< hue 0~360 */
    float s;        /*!< saturation  0.0 ~ 1.0 */
    float v;        /*!< brightness  0.0 ~ 1.0 */
}color_light_info_t;

extern void color_light_init(void);
extern void color_light_control(float h,float s,float v);
extern void color_light_set_level(float v);
extern void color_light_set_color(float h,float s);

/** @} */       

#ifdef __cplusplus
}
#endif


#endif
  
