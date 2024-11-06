/**
 * @file   color_light_control.c
 * @brief   A sample code use CI130X PWM control RGB or RGBW light, used HSV
 *         color space.
 * @version V1.0.0
 * @date 2018-05-23
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include <stdint.h>
#include "color_light_control.h"
#include "ci130x_pwm.h"
#include "ci130x_scu.h"
#include "ci130x_gpio.h"

#include "ci130x_uart.h"
#include "ci_log.h"
#include <math.h>
#include "ci130x_dpmu.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "board.h"


/**************************************************************************
                    type define
****************************************************************************/
#define PWM_MAX_CYCLE   1000

#define MAX3(a,b,c) (((a) > (b)) ? ( ((a) > (c)) ? (a) : (c) ) : ( ((b) > (c)) ? (b) : (c) ))
#define MIN3(a,b,c) (((a) > (b)) ? ( ((b) > (c)) ? (c) : (b) ) : ( ((a) > (c)) ? (c) : (a) ))

/**************************************************************************
                    function prototype
****************************************************************************/



/**************************************************************************
                    global
****************************************************************************/
static float current_color_h;
static float current_color_s;
static float current_color_v;

/*color light pwm config*/
pwm_base_t       color_light_r_pwm_name;
pwm_base_t       color_light_g_pwm_name;
pwm_base_t       color_light_b_pwm_name;

/**************************************************************************
                    color led
****************************************************************************/


/**
 * @brief initial, config IO and PWM used by light.
 *
 */
void color_light_init(void)
{
    // extern void pad_config_for_color_light(void);
    pad_config_for_color_light();
}


void HSV_to_RGB(float * r,float * g,float * b,float h,float s,float v)
{
    int i;
    float f,p,q,t;

    h /= 60; //sector 0 to 5
    i = (int)floor(h);
    f = h - i;   //factorial part of h
    p = v *(1 - s);
    q = v *(1 - s*f);
    t = v*(1 - s*(1-f));

    switch(i)
    {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default://case 5:
            *r = v;
            *g = p;
            *b = q;
            break;
   }
}


#if 1/*smple 1*/
void gamma_correct_RGB(float *R, float *G, float *B)
{
    if ( *R > 0.003f )
    {
        *R = (1.22f * (float)( powf(*R, ( 1/1.5f ) )) - 0.040f);
    }
    else
    {
        *R = 0;
    }

    if ( *G > 0.003f)
    {
        *G = (1.22f * (float)( powf(*G, ( 1/1.5f ) )) - 0.040f);
    }
    else
    {
        *G = 0;
    }

    if ( *B > 0.003f )
    {
        *B = (1.09f * (float)( powf(*B, ( 1/1.5f ) )) - 0.050f);
    }
    else
    {
        *B = 0;
    }
}
#else/*smple 2*/
void gamma_correct_RGB(float *R, float *G, float *B)
{
    if ( *R > 0.003f )
    {
        *R = ( ( powf(*R, 2.2f )) );
    }
    else
    {
        *R = 0;
    }

    if ( *G > 0.003f)
    {
        *G = (( powf(*G, 2.2f )));
    }
    else
    {
        *G = 0;
    }

    if ( *B > 0.003f )
    {
        *B = (( powf(*B, 2.2f )));
    }
    else
    {
        *B = 0;
    }
}
#endif


void temp_compensate_RGB(float *R, float *G, float *B)
{
}


void group_wavelen_compensate_RGB(float *R, float *G, float *B)
{
}


void group_lumin_compensate_RGB(float *R, float *G, float *B)
{
}

//#define USED_RGBW_LED_SHOW (0)

#ifdef USED_RGBW_LED_SHOW
static void RGB_to_RGBW( uint32_t *R, uint32_t *G, uint32_t *B, uint32_t *W )
{
  uint32_t w;
  if ( MAX3(*R,*G,*B) == 0 )
  {
    *W = 0;
    return;
  }
  w = (uint32_t)( ( (uint32_t)MIN3(*R,*G,*B) + MAX3(*R,*G,*B)  ) / ( MAX3(*R,*G,*B)  ) );
  *R *= w;
  *G *= w;
  *B *= w;
  *W = MIN3(*R,*G,*B);
  *W = (*W > PWM_MAX_CYCLE) ? PWM_MAX_CYCLE : *W;
  *R -= *W;
  *G -= *W;
  *B -= *W;
}
#endif

static uint32_t led_can_set0_flag = 0;
void light_pwm_set_duty (pwm_base_t channel, uint32_t promill)
{
#define MIN_PROMILL 0

    if(1==led_can_set0_flag)
    {

    }
    else
    {
        if(promill <= MIN_PROMILL)
        {
            promill = MIN_PROMILL;
        }
    }

    //promill = PWM_MAX_CYCLE - promill;/*invert*/
    pwm_set_duty(channel,promill,PWM_MAX_CYCLE);
}


static void light_update_color(void)
{
    float h,s,v;
    float f_R,f_G,f_B;
    uint32_t R,G,B;
    #ifdef USED_RGBW_LED_SHOW/*for RGBW light*/
    uint32_t W;
    #endif
    float Y,LinearY;

    h = current_color_h;
    s = current_color_s;
    v = current_color_v;

    /*par check*/
    h = h>=360.00 ? 0 : h;
    s = s>1.0 ? 1.0 : s;
    v= v>1.0 ? 1.0 : v;

    LinearY = v;

    #if 1/*gamma correct the level*/
    Y = powf( LinearY, (float)2.0 ) ;
    #else/*no gamma correct*/
    Y = LinearY;
    #endif

    #if 0/*correct some color*/
    if((h<270.00f)&&(h>210.00f))
    {
        h = powf((h-210.00f)/60.00f,2.5f) * 60.00f + 210.00f ;
    }
    #endif

    s = powf( s, (float)0.45 ) ;

    HSV_to_RGB(&f_R,&f_G,&f_B,h,s,Y);

    //color correction
    //gamma_correct_RGB(&f_R, &f_G, &f_B);
    temp_compensate_RGB(&f_R, &f_G, &f_B);
    group_wavelen_compensate_RGB(&f_R, &f_G, &f_B);
    group_lumin_compensate_RGB(&f_R, &f_G, &f_B);

    // truncate results exceeding 0..1
    f_R = (f_R > 1.0f ? 1.0f : (f_R < 0.0f ? 0.0f : f_R));
    f_G = (f_G > 1.0f ? 1.0f : (f_G < 0.0f ? 0.0f : f_G));
    f_B = (f_B > 1.0f ? 1.0f : (f_B < 0.0f ? 0.0f : f_B));

    R = (uint32_t)(f_R * (PWM_MAX_CYCLE-1));
    G = (uint32_t)(f_G * (PWM_MAX_CYCLE-1));
    B = (uint32_t)(f_B * (PWM_MAX_CYCLE-1));

    #ifdef USED_RGBW_LED_SHOW/*for RGBW light*/
    RGB_to_RGBW( &R, &G, &B, &W );//wwf

    /*gamma correct*/
    W = (uint32_t)(powf((float)W/PWM_MAX_CYCLE,(float)2.2) * PWM_MAX_CYCLE);
    #endif

    light_pwm_set_duty (color_light_r_pwm_name,R);
    light_pwm_set_duty (color_light_g_pwm_name,G);
    light_pwm_set_duty (color_light_b_pwm_name,B);
    #ifdef USED_RGBW_LED_SHOW/*for RGBW light*/
    //pwm_set_duty (WHITE_LED, W);//wwf
    #endif
}


/**
 * @brief set color, HSV all need.
 *
 * @param h : hue 0~360
 * @param s : saturation 0.0~1.0
 * @param v : value 0.0~1.0
 */
void color_light_control(float h,float s,float v)
{
    current_color_h = h;
    current_color_s = s;
    current_color_v = v;

    light_update_color();
}


/**
 * @brief set color, only update hue and saturation
 *
 * @param h : hue 0~360
 * @param s : saturation 0.0~1.0
 */
void color_light_set_color(float h,float s)
{
    current_color_h = h;
    current_color_s = s;

    light_update_color();
}


/**
 * @brief set clolr, only update value(lightness)
 *
 * @param v : value of HSV  0.0~1.0
 */
void color_light_set_level(float v)
{
    current_color_v = v;

    light_update_color();
}


