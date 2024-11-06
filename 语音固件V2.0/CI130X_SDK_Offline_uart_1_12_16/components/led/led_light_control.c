/**
  ******************************************************************************
  * @file    led_light_control.c
  * @version V1.0.0
  * @brief   A sample code use CI130X PWM control LED for night light, use CI130X
  *          PWM control LED for toys blink light
  * @date    2018.05.23
  * @brief
  ******************************************************************************
  **/

#include <stdint.h>
#include "led_light_control.h"
#include "ci130x_pwm.h"
#include "ci130x_scu.h"
#include "ci130x_gpio.h"
#include "board.h"
#include "ci130x_uart.h"
#include "ci130x_dpmu.h"
#include "ci_log.h"
#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/**************************************************************************
                    type define
****************************************************************************/
#define PWM_MAX_CYCLE   100

/*night light pwm config*/
pwm_base_t       night_light_w_pwm_name;

#define LED_ON  0
#define LED_OFF 1

typedef struct
{
    uint32_t led_onoff;
    uint32_t time_ms;
}led_show_state_t;



/**************************************************************************
                    function prototype
****************************************************************************/






/**************************************************************************
                    global
****************************************************************************/
xTimerHandle blink_led_timer = NULL;

static uint8_t led_flick_index = 0;

const led_show_state_t blink_eye[] =
{
    [0] = {LED_ON,600},
    [1] = {LED_OFF,500},
    [2] = {LED_ON,600},
    [3] = {LED_OFF,500},
    [4] = {LED_OFF,50},
};


/**************************************************************************
                    pwm led
****************************************************************************/
/**
 * @brief night initial, config PWM
 * 
 */
void night_light_init(void)
{
    // extern void pad_config_for_night_light(void);
    pad_config_for_night_light();
}


/**
 * @brief night light set brightness,
 * 
 * @param onoff 1:on , 0:off
 * @param val   0~100
 */
void night_light_set_brightness(uint32_t onoff,uint32_t val)
{
    float Y,Yin;
    if(0 == onoff)
    {
        pwm_set_duty((pwm_base_t)night_light_w_pwm_name,0,PWM_MAX_CYCLE);
        return;
    }

    if(val > 99)
    {
        val = 99;
    }

    Yin = val/100.00f;
    if(Yin > 1.0f)
    {
        Yin = 1.0f;
    }
    Y = powf( Yin, (float)2.0 ) ;
    Y = Y*PWM_MAX_CYCLE;
    // mprintf("Y is %f\n",Y);
    pwm_set_duty((pwm_base_t)night_light_w_pwm_name,(unsigned int)Y,PWM_MAX_CYCLE);
}


/**************************************************************************
                    blink led
****************************************************************************/
void SetLedAndTime(uint8_t on_off,uint16_t ms)
{
    // extern void blink_light_on_off(uint8_t on_off);
    blink_light_on_off(on_off);
    xTimerChangePeriod(blink_led_timer,pdMS_TO_TICKS(ms),0);
    xTimerStart(blink_led_timer,0);
}


void blink_led_timer_callback(TimerHandle_t p)
{
    SetLedAndTime(blink_eye[led_flick_index].led_onoff,blink_eye[led_flick_index].time_ms);
    led_flick_index++;

    //led_flick_index %= (sizeof(blink_eye)/sizeof(blink_eye));
    if(led_flick_index >= sizeof(blink_eye)/sizeof(led_show_state_t))
    {
        xTimerStop(blink_led_timer,0);
        led_flick_index = 0;
    }
}


/**
 * @brief blink light initial, config LED, and a RTOS soft timer.
 * 
 */
void blink_light_init(void)
{
    // extern void pad_config_for_blink_light(void);
    pad_config_for_blink_light();

    blink_led_timer = xTimerCreate("Blink_led_timer", pdMS_TO_TICKS(10),pdFALSE, (void *)0, blink_led_timer_callback);
    if(NULL == blink_led_timer)
    {
        mprintf("timer error!\n");
    }
}


/**
 * @brief blink light start one time, simulate blink eye one time, for toys use
 * 
 */
void blink_light_on(void)
{
    led_flick_index = 0;
    xTimerChangePeriod(blink_led_timer,pdMS_TO_TICKS(10),0);
    xTimerStart(blink_led_timer,0);
}

