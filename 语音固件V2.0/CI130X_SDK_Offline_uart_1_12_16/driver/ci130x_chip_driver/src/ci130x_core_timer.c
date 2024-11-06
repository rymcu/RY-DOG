/**
 * @file ci130x_core_timer.c
 * @brief 内核计数器定时器驱动
 * @version 1.0
 * @date 2019-11-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include "ci130x_core_eclic.h"

#include "ci130x_core_timer.h"


static uint32_t mtime_lo(void)
{
    return *(volatile uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME);
}


static uint32_t mtime_hi(void)
{
    return *(volatile uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME + 4);
}


static uint32_t __attribute__((noinline)) measure_cpu_freq(size_t n)
{
    uint32_t start_mtime, delta_mtime;
    extern uint32_t get_systick_clk(void);
    uint32_t mtime_freq = get_systick_clk();

    // Don't start measuruing until we see an mtime tick
    uint32_t tmp = mtime_lo();
    do
    {
        start_mtime = mtime_lo();
    } while (start_mtime == tmp);

    uint32_t start_mcycle = read_csr(mcycle);

    do
    {
        delta_mtime = mtime_lo() - start_mtime;
    } while (delta_mtime < n);

    uint32_t delta_mcycle = read_csr(mcycle) - start_mcycle;

    return (delta_mcycle / delta_mtime) * mtime_freq + ((delta_mcycle % delta_mtime) * mtime_freq) / delta_mtime;
}


uint32_t get_cpu_freq(void)
{
    uint32_t cpu_freq;

    // warm up
    measure_cpu_freq(1);
    // measure for real
    cpu_freq = measure_cpu_freq(100);

    return cpu_freq;
}


/**
 * @brief 获取当前timer计数值
 * 
 * @return uint64_t 当前timer计数值
 */
uint64_t get_timer_value(void)
{
    while (1)
    {
        volatile uint32_t hi = mtime_hi();
        volatile uint32_t lo = mtime_lo();
        if (hi == mtime_hi())
        {
            return ((uint64_t)hi << 32) | lo;
        }
    }
}

void clear_timer_value(void)
{
	*(volatile uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0x0;
	*(volatile uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME + 4) = 0x0;
}

/**
 * @brief 触发软中断
 * 
 */
void set_msip_int(void)
{
	*(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MSIP) |=0x01;
}


/**
 * @brief 清除软中断
 * 
 */
void clear_msip_int(void)
{
	*(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MSIP) &= ~0x01;
}


/**
 * @brief 软件发出内核复位请求
 * 
 */
void core_system_reset(void)
{
    *(volatile uint32_t *) (TIMER_CTRL_ADDR + TIMER_MSFTRST) = 0x80000a5f;
}


/**
 * @brief 设置内核timer暂停计数
 * 
 */
void set_timer_stop(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) |= (0x1 << 0);
}


/**
 * @brief 取消内核timer暂停计数（即继续计数）
 * 
 */
void clear_timer_stop(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) &= ~(0x1 << 0);
}


/**
 * @brief 开启内核timer计数中断自动清零
 * 
 */
void set_timer_cmpclrem(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) |= (0x1 << 1);
}


/**
 * @brief 关闭内核timer计数中断自动清零
 * 
 */
void clear_timer_cmpclrem(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) &= ~(0x1 << 1);
}


/**
 * @brief 设置内核timer时钟源为内核时钟
 * 
 */
void set_timer_clksrc(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) |= (0x1 << 2);
}


/**
 * @brief 设置内核timer时钟源为外部时钟
 * 
 */
void clear_timer_clksrc(void)
{
    *(volatile uint8_t *) (TIMER_CTRL_ADDR + TIMER_MTIMECTL) &= ~(0x1 << 2);
}
