/**
 * @file ci130x_core_misc.c
 * @brief 内核相关其他驱动
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

#include "ci130x_core_misc.h"
#include "ci130x_core_timer.h"

#include "ci130x_core_eclic.h"

/**
 * @brief 获取内核执行指令数
 * @note 需要使能mcountinhibit计数功能，该函数一般用于调试
 * 
 * @return uint64_t 执行指令数
 */
uint64_t get_instret_value(void)
{
    while (1)
    {
        uint32_t hi = read_csr(0xb82);
        uint32_t lo = read_csr(0xb02);
        if (hi == read_csr(0xb82))
        {
            return ((uint64_t)hi << 32) | lo;
        }
    }
}


/**
 * @brief 获取内核执行时钟周期数
 * @note 需要使能mcycle计数功能，该函数一般用于调试
 * 
 * @return uint64_t 执行时钟周期数
 */
uint64_t get_cycle_value(void)
{
    while (1)
    {
        uint32_t hi = read_csr(0xb80);
        uint32_t lo = read_csr(0xb00);
        if (hi == read_csr(0xb80))
        {
            return ((uint64_t)hi << 32) | lo;
        }
    }
}


/**
 * @brief 检查当前是否位于中断/异常状态
 * 
 * @retval 0 未处于中断/异常状态
 * @retval 1 处于中断/异常状态
 */
int32_t check_curr_trap(void)
{
	uint32_t csr_msubm = read_csr(0x7c4);//msubm
	if(csr_msubm & (0x3 << 6))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
