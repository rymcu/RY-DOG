/**
 * @file ci130x_timer.h
 * @brief  窗口看门狗驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#ifndef __TIMERWDT_H
#define __TIMERWDT_H

#include "ci130x_system.h"
#include "platform_config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef TWDG_CLK
#define TWDG_CLK   (get_apb_clk())
#endif

enum
{
    TWDG_DISABLE = 0,
    TWDG_ENABLE
};

enum
{
    TWDG_PCLK = 0,
    TWDG_EXT_CLK
};

enum
{
    TWDG_NORMAL_MODE = 0,
    TWDG_PRE_WARNING
};


typedef struct
{
    unsigned int scale;//分频系数，0和1表示不分频
    unsigned int lower;
    unsigned int upper;
    unsigned int mode;
}TWDG_InitTypeDef;

void TWDG_IRQHandler(void);

void TWDG_init( TWDG_InitTypeDef *initStruct );

void TWDG_disable(void);

unsigned int TWDG_counter(void);

void TWDG_service(void);

void TWDG_test(int mode,int ms_windowLowper,int ms_windowUpper);

#ifdef __cplusplus
}
#endif

#endif
/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
