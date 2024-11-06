/**
 * @file ci130x_iwdg.c
 * @brief  看门狗驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include "ci130x_iwdg.h"
#include "ci_assert.h"
#include "ci_log.h"


typedef struct
{
	volatile unsigned int wdogload;	/*!< offect:0x00;功能：计数初值寄存器 */
	volatile unsigned int wdogvalue;   /*!< offect:0x04;功能：计数值寄存器 */
	volatile unsigned int wdogcontrol;	/*!< offect:0x08;功能：控制寄存器 */
	volatile unsigned int wdogintclr;	/*!< offect:0x0c;功能：中断清除寄存器 */
	volatile unsigned int wdogris;		/*!< offect:0x10;功能：原始中断状态寄存器 */
	volatile unsigned int wdogmis;	    /*!< offect:0x14;功能：屏蔽中断状态寄存器 */
	volatile unsigned int wdogloaden;
}iwdg_register_t;

#define WDOGLOCK_REG    (*(volatile unsigned int *)(HAL_IWDG_BASE + 0x0c00))
#define IWDG_UNLOCK_MAGIC_NUMBER (0x1ACCE551)
#define IWDG_LOCK_MAGIC_NUMBER (1)

#define IWDG_WRITE_UNLOCK (WDOGLOCK_REG = IWDG_UNLOCK_MAGIC_NUMBER)
#define IWDG_WRITE_LOCK (WDOGLOCK_REG = IWDG_LOCK_MAGIC_NUMBER)

/**
 * @brief iwdg初始化
 *
 * @param base 外设基地址
 * @param init 初始化结构体
 */
void iwdg_init(iwdg_base_t base,iwdg_init_t init)
{
    /* 功能实现 */
    iwdg_register_t* iwdg_p = (iwdg_register_t*)base;
    IWDG_WRITE_UNLOCK;
    iwdg_p->wdogload = init.count;/* 设置计数值 */
    iwdg_p->wdogloaden = 0xffffffff;
    unsigned int temp = 0;
    temp = iwdg_p->wdogcontrol;
    temp &= ~(0x3 << 0);
    temp |= (init.irq << 0);
    temp |= (init.res << 1);
    iwdg_p->wdogcontrol = temp;/* 设置中断和复位 */
    IWDG_WRITE_LOCK;
}

/**
 * @brief 打开iwdg
 *
 * @param base 外设基地址
 */
void iwdg_open(iwdg_base_t base)
{
    /* 功能实现 */
    iwdg_register_t* iwdg_p = (iwdg_register_t*)base;
    IWDG_WRITE_UNLOCK;
    iwdg_p->wdogintclr = 0x1;/* 向此寄存器写任何值清除中断(或者说喂狗)，后重载计数值 */
    IWDG_WRITE_LOCK;
}

/**
 * @brief 关闭iwdg
 *
 * @param base 外设基地址
 */
void iwdg_close(iwdg_base_t base)
{
    /* 功能实现 */
    iwdg_register_t* iwdg_p = (iwdg_register_t*)base;
    IWDG_WRITE_UNLOCK;
    iwdg_p->wdogcontrol = 0;/* 设置禁止中断和复位 */
    IWDG_WRITE_LOCK;
}

/**
 * @brief 喂狗函数
 *
 * @param base 外设基地址
 */
void iwdg_feed(iwdg_base_t base)
{
    /* 功能实现 */
    iwdg_register_t* iwdg_p = (iwdg_register_t*)base;
    IWDG_WRITE_UNLOCK;
    iwdg_p->wdogintclr = 0x1;/* 向此寄存器写任何值清除中断(或者说喂狗)，后重载计数值。*/
    IWDG_WRITE_LOCK;
}


/*******************************************************************************
								中断处理函数
*******************************************************************************/

/**
 * @brief 中断处理函数
 *
 */
void iwdg_irqhander(void)
{
    /* 功能实现 */
    iwdg_register_t* iwdg_p = (iwdg_register_t*)IWDG;
    IWDG_WRITE_UNLOCK;
    iwdg_p->wdogintclr = 0x1;/* 向此寄存器写任何值清除中断(或者说喂狗)，后重载计数值。*/
    IWDG_WRITE_LOCK;
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/