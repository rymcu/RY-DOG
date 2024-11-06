#ifndef __USER_CONTROL_H__
#define __USER_CONTROL_H__

#include "ci130x_pwm.h"
#include "ci130x_gpio.h"
#include "ci130x_dpmu.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GPIO_control_init(void);

void pwm_control_init(void);

extern void user_pin_control_init(void);

#ifdef __cplusplus
}
#endif

#endif

