#include "user_control.h"

//GPIO init
void GPIO_control_init(void)
{
    ///tag-user-defined-gpio-init-by-pin-num-start

    ///"tag-user-defined-gpio-init-by-pin-num-end
}

// 输出高低电平
// void gpio_set_output_level_single(gpio_base_t gpio,gpio_pin_t pins,uint8_t level);

// 获取输入电平
// uint8_t gpio_get_input_level_single(gpio_base_t gpio,gpio_pin_t pins);


//pwn init
void pwm_control_init(void)
{
    pwm_init_t pwm_config;
    ///tag-user-defined-pwm-init-by-pin-num-start
    
    ///tag-user-defined-pwm-init-by-pin-num-end
}

void user_pin_control_init(void)
{
    GPIO_control_init();
    pwm_control_init();
}
