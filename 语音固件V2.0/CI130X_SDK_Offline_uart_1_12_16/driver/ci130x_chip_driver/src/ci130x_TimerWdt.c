/**
 * @file ci130x_timer.c
 * @brief  窗口看门狗驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include "ci130x_TimerWdt.h"
#include "ci130x_scu.h"
#include "ci130x_dpmu.h"
#include "ci130x_core_eclic.h"
#include "ci_log.h"

/*TimerWatchDog结构体*/
typedef struct
{
    unsigned int glb_en     :1;         //watchdog timer enable
    unsigned int clk_sel    :1;         //时钟选择
    unsigned int mode       :1;         //工作模式选择
    unsigned int halt_en    :1;         //halt功能使能
    unsigned int srvWin_en  :1;         //window 功能开关，高有效
    unsigned int rest_en    :1;         //复位产生使能，高有效
    unsigned int int_en     :1;         //INT产生使能，高有效
    unsigned int reserved   :25;
}BF_TWDG_CONTROL;

typedef struct
{
    unsigned int scale      :16;        //分频系数，(0/1:不分频)
    unsigned int reserved   :16;
}BF_TWDG_SCALE;

typedef struct
{
    unsigned int cnt_ext    :1;         //counter计数越界
    unsigned int reserved   :31;
}BF_TWDG_STATUS;

typedef union
{
    unsigned int      ui;
    BF_TWDG_CONTROL   bf;
}_TWDG_CONTROL;

typedef union
{
    unsigned int      ui;
    BF_TWDG_SCALE     bf;
}_TWDG_SCALE;

typedef union
{
    unsigned int      ui;
    BF_TWDG_STATUS   bf;
}_TWDG_STATUS;

typedef struct
{
	volatile _TWDG_CONTROL   WDT_CTR;
	volatile _TWDG_SCALE 	WDT_SCALE;
	volatile unsigned int 	WDT_SRV;
	volatile unsigned int 	WDT_WLB;
	volatile unsigned int 	WDT_WUB;
	volatile unsigned int 	WDT_LOCK;
	volatile unsigned int 	WDT_COUNT;
	volatile _TWDG_STATUS 	WDT_STATUS;
}TWDG_TypeDef;
/*TimerWatchDog结构体end*/

/**
  * @功能:初始化配置时间看门狗
  * @注意:无
  * @参数:initStruct     - 初始化配置结构体
  * @返回值:无
  */
void TWDG_init(TWDG_InitTypeDef *initStruct)
{
    TWDG->WDT_LOCK              = 0x51acce55;
    TWDG->WDT_SCALE.bf.scale    = initStruct->scale;
    TWDG->WDT_WLB               = initStruct->lower;
    TWDG->WDT_WUB               = initStruct->upper;
    TWDG->WDT_CTR.bf.glb_en     = TWDG_ENABLE;
    TWDG->WDT_CTR.bf.clk_sel    = TWDG_PCLK;
    TWDG->WDT_CTR.bf.mode       = initStruct->mode;
    TWDG->WDT_CTR.bf.halt_en    = TWDG_ENABLE;
    TWDG->WDT_CTR.bf.srvWin_en  = TWDG_ENABLE;
    TWDG->WDT_CTR.bf.rest_en    = TWDG_ENABLE;
    TWDG->WDT_CTR.bf.int_en     = TWDG_ENABLE;
    TWDG->WDT_LOCK  = 0;
}


/**
  * @功能:关闭时间看门狗
  * @注意:无
  * @参数:initStruct     - 初始化配置结构体
  * @返回值:无
  */
void TWDG_disable(void)
{
    TWDG->WDT_LOCK          = 0x51acce55;
    TWDG->WDT_CTR.bf.glb_en = 0;
    TWDG->WDT_LOCK          = 0;
}


/**
  * @功能:获取counter值
  * @注意:无
  * @参数:initStruct     - 初始化配置结构体
  * @返回值:counter值
  */
unsigned int TWDG_counter(void)
{
    int value;

    TWDG->WDT_LOCK          = 0x51acce55;
    value   = TWDG->WDT_COUNT;
    TWDG->WDT_LOCK          = 0;

    return value;
}


/**
  * @功能:看门狗service
  * @注意:无
  * @参数:无
  * @返回值:无
  */
void TWDG_service()
{
    TWDG->WDT_LOCK  = 0x51acce55;
    TWDG->WDT_SRV   = 0xabadc0de;
    TWDG->WDT_LOCK  = 0;
    ci_logdebug(LOG_TWDT,"TWDG serviced!\n");
}

/****************************************************************************
** 函数  名： TWDG_test
** 功    能： 初始化时间看门狗
** 入口参数： mode       -  看门狗工作模式(TWDG_PRE_WARNING / TWDG_NORMAL_MODE)
**            ms_windowLowper     - 窗口下边界，单位ms
**            ms_windowUpper      - 窗口上边界，单位ms
**
** 返回  值： 无
**
****************************************************************************/
void TWDG_test(int mode,int ms_windowLowper,int ms_windowUpper)
{
	dpmu_twdg_reset_sysytem_config();     //要产生复位，必须配置SCU中，TWDG复位开关
	eclic_irq_enable(TWDG_IRQn);        //要产生中断，必须使能TWDG中断向量

    TWDG_InitTypeDef initStruct;
    initStruct.mode = mode; //TWDG_PRE_WARNING / TWDG_NORMAL_MODE
    initStruct.scale = 1;
    initStruct.scale = (initStruct.scale > 1) ? initStruct.scale : 1;
    initStruct.lower = (TWDG_CLK / (initStruct.scale * 1000)) * ms_windowLowper;
    initStruct.upper = (TWDG_CLK / (initStruct.scale * 1000)) * ms_windowUpper;

    TWDG_disable();
    TWDG_init(&initStruct);
}


/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
