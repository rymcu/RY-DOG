/**
 * @file ci130x_init.c
 * @brief C环境启动预初始
 * @version 1.0.0
 * @date 2019-11-21
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "ci130x_core_eclic.h"
#include "ci130x_core_misc.h"
#include "ci130x_core_timer.h"
#include "ci130x_scu.h"
#include "ci130x_uart.h"
#include "ci130x_spiflash.h"
#include "platform_config.h"
#include "sdk_default_config.h"
#include "ci130x_dpmu.h"
#include "crc.h"
#include "ci_log.h"

//static void SystemInit(void);

void _init()
{
    #ifndef NO_RC_CLOCK_CONFIG
    extern char SHARE_SRAM_ADDR;
    extern char SHARE_SRAM_SIZE;
    memset(&SHARE_SRAM_ADDR,0x00,&SHARE_SRAM_SIZE);

	dpmu_rc_freq_sel(DPMU_RC_FREQ_12d288M);

    extern void board_clk_source_set(void);
    board_clk_source_set();

    #if USE_LOWPOWER_DOWN_FREQUENCY
    dpmu_pll_config(SRC_FREQUENCY_LOWPOWER, MAIN_FREQUENCY);
    set_src_clk(SRC_FREQUENCY_LOWPOWER);
    set_osc_clk(SRC_FREQUENCY_LOWPOWER);
    #else
    set_apb_clk(SRC_FREQUENCY_NORMAL/2);
    #if (USE_EXTERNAL_CRYSTAL_OSC == 0)
    load_freq_correct_factor();
    #endif
    dpmu_pll_config((uint32_t)(SRC_FREQUENCY_NORMAL/get_freq_factor()), MAIN_FREQUENCY);
    set_src_clk(SRC_FREQUENCY_NORMAL);
    set_osc_clk(SRC_FREQUENCY_NORMAL);
	#endif
    #endif

    /* ECLIC init */
    eclic_init(ECLIC_NUM_INTERRUPTS);
    eclic_mode_enable();

    disable_mcycle_minstret();

    //SystemInit();

    //init_clk_div();
    init_irq_pri();

    /* 设置中断优先级分组 */
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO0);

    /* 开启全局中断 */
    eclic_global_interrupt_enable();

    enable_mcycle_minstret();

#ifdef STDOUT_INTERFACE
#if (STDOUT_INTERFACE == HAL_UART0_BASE) || (STDOUT_INTERFACE == HAL_UART1_BASE) || (STDOUT_INTERFACE == HAL_UART2_BASE)
    UARTPollingConfig((UART_TypeDef*)STDOUT_INTERFACE, UART_BaudRate921600);
#endif
#endif
}

void _fini()
{
}

uint8_t LDO_trim_value_crc;
uint32_t efuse_writer;

void load_ldo_trim_value()
{
    scu_set_device_gate(HAL_EFUSE_BASE, ENABLE);
    efuse_clk_freq_sel(1);

    efuse_read_start();
    while(!efuse_check_idle());
    volatile uint32_t data = efuse_read_data(15);
    //uint32_t efuse_writer = data & (1<<22);
    efuse_writer = ((data >> 22) & 3);
    data = efuse_read_data(5);
    uint8_t LDO_trim_value[4];
    LDO_trim_value[0] = (data >> 12) & 0x0F;    //LDO1
    LDO_trim_value[1] = (data >> 17) & 0x1F;    //LDO2
    LDO_trim_value[2] = (data >> 22) & 0x0F;    //LDO3
    LDO_trim_value[3] = (data >> 26) & 0x1F;    //VREF
    // uint8_t LDO_trim_value_crc = crc8(0, LDO_trim_value, 4);
    LDO_trim_value_crc = crc8(0, LDO_trim_value, 4);
    
    if (LDO_trim_value_crc == (uint8_t)((efuse_read_data(4) >> 24) & 0xFF))
    {
        dpmu_unlock_cfg_config();
        if (efuse_writer == 1)
        {
            dpmu_ldo1_lv_set(LDO_trim_value[0]);
            dpmu_ldo2_lv_set(LDO_trim_value[1]);
            dpmu_config_update_en(0);
            dpmu_config_update_en(1);
        }
        else if (efuse_writer == 2)
        {
            uint8_t pmu_1_1 = efuse_read_data(4) & 0xFF;
            if (pmu_1_1 > LDO_trim_value[3])
            {
                dpmu_ldo1_lv_set(LDO_trim_value[0]);
                dpmu_ldo2_lv_set(LDO_trim_value[1]);
                dpmu_ldo3_lv_set(LDO_trim_value[2]);
                dpmu_config_update_en(0);
                dpmu_config_update_en(1);
                dpmu_config_update_en(2);
                dpmu_set_pmu_trim_value(LDO_trim_value[3]);
                dpmu_config_update_en(4);
            }
            else
            {
                dpmu_set_pmu_trim_value(LDO_trim_value[3]);
                dpmu_config_update_en(4);
                dpmu_ldo1_lv_set(LDO_trim_value[0]);
                dpmu_ldo2_lv_set(LDO_trim_value[1]);
                dpmu_ldo3_lv_set(LDO_trim_value[2]);
                dpmu_config_update_en(0);
                dpmu_config_update_en(1);
                dpmu_config_update_en(2);
            }
        }
        dpmu_lock_cfg_config();
    }
}

void print_ldo_info(void)
{
    if (LDO_trim_value_crc == (uint8_t)((efuse_read_data(4) >> 24) & 0xFF))
    {
        mprintf("EFUSE writer:");
        if (efuse_writer == 1)
        {
            mprintf("FT\n");
        }
        else if (efuse_writer == 2)
        {
            mprintf("CP\n");
        }
        else
        {
            mprintf("None\n");
        }
    }
    else
    {
        ci_logwarn(CI_LOG_WARN, "LDO trim value CRC error\n");
    }
}

void SystemInit(void)
{ 
    /*scu关闭时钟*/
    scu_unlock_clk_config();
    scu_unlock_reset_config();
    scu_unlock_system_config();
    SCU->AHB_CLKGATE_CFG &= ~(0x1F << 1);   //DSU不关
    SCU->APB0_CLKGATE_CFG &= ~(0x7FF << 0);   //CODEC不关
    SCU->APB1_CLKGATE_CFG &= ~(0x71B << 0);   //IIS0、IIS1、UART0不关
    scu_lock_clk_config();
    scu_lock_reset_config();
    scu_lock_system_config();

    dpmu_unlock_cfg_config();
    DPMU->AON_CLKGATE_CFG &= ~(0x1f << 0);    /*dpmu关闭时钟*/
    DPMU->PAD_FILTER_CFG_ADDR |= (0x1 << 10);  /*设置复位脚防抖动功能*/
    dpmu_lock_cfg_config();

    load_ldo_trim_value();
}

