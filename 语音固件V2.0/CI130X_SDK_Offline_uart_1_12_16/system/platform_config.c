/**
 * @file platform_config.c
 * @brief 统一初始化io、中断优先级等
 * @version 1.0
 * @date 2019-06-12
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include "platform_config.h"
#include "ci130x_system.h"
#include "ci130x_scu.h"
#include "ci130x_core_eclic.h"
#include "ci130x_gpio.h"
#include "sdk_default_config.h"
#include "romlib_runtime.h"
#include "board.h"
#include "ci130x_mailbox.h"
#include "status_share.h"
#include "ci130x_spiflash.h"


float g_freq_correct_factor = 1.0;
float load_freq_correct_factor()
{
    float ret = 1.0;
    uint8_t buffer[12];
    const spic_security_reg_t regs[3] = {SPIC_SECURITY_REG1, SPIC_SECURITY_REG2, SPIC_SECURITY_REG3};

    flash_init(QSPI0);
    for (int i = 0;i < 2;i++)
    {
        spic_read_security_reg(QSPI0, regs[i], buffer, 0, 12);
        float t = *(float*)&buffer[0];
        uint32_t check_value_i = ~*(uint32_t*)&buffer[4];
        float check_value_f = *(float*)&check_value_i;
        if ((check_value_f == t) && (t > 0.90) && (t < 1.10))
        {
            ret = t;
            break;
        }
        
    }
    eclic_irq_disable(DMA_IRQn);
    g_freq_correct_factor = ret;
    return ret;
}

/**
 * @brief 配置系统中断优先级
 * 
 */
void init_irq_pri(void)
{
    for (int i = TWDG_IRQn;i <= MAILBOX_IRQn;i++ )
    {
        eclic_irq_set_priority(i, 6, 0);
    }
}


/**
 * @brief 配置总线时钟
 * 
 */
void init_clk_div(void)
{   
    uint32_t main_frequency = dpmu_get_pll_frequency()/get_freq_factor();

    /* PLL 480M ip_core 240M */
    set_ipcore_clk(main_frequency);
    
    /* AHB 240M */
    set_ahb_clk(main_frequency);

    /* APB 120M */
    void set_apb_clk(uint32_t clk);
    set_apb_clk(main_frequency/2);

    /* SRC 12.288M */
    // void set_src_clk(uint32_t clk);
    // set_src_clk(SRC_FREQUENCY);

    /* 内核timer时钟 7.5M x 2(双边沿) */
    set_systick_clk(main_frequency/12);
}




/**
 * @brief 初始化系统
 * 
 */
void init_platform(void)
{    
    init_clk_div();
    init_irq_pri();
    mailbox_preinit();
    // #if (USE_EXTERNAL_CRYSTAL_OSC == 0 && UART_BAUDRATE_CALIBRATE == 0)
    // load_freq_correct_factor();
    // #endif
}

float get_freq_factor()
{
    return g_freq_correct_factor;
}

/**
 * @brief 获取ipcore时钟
 * 
 * @return uint32_t ipcore时钟
 */
uint32_t get_ipcore_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_IPCORE_CLK);
}


/**
 * @brief 获取AHB时钟
 * 
 * @return uint32_t AHB时钟
 */
uint32_t get_ahb_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_AHB_CLK);
}


/**
 * @brief 获取APB时钟
 * 
 * @return uint32_t APB时钟
 */
uint32_t get_apb_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_APB_CLK);
}


/**
 * @brief 获取systick时钟
 * 
 * @return uint32_t systick时钟
 */
uint32_t get_systick_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_SYSTEMTICK_CLK);
}





/**
 * @brief 获取osc时钟
 * 
 * @return uint32_t osc时钟
 */
uint32_t get_osc_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_EXT_OSC_CLK);
}


uint32_t get_src_clk(void)
{
    return (uint32_t)ciss_get(CI_SS_SRC_CLK);
}

/**
 * @brief 设置IPCORE时钟
 * 
 * @param clk IPCORE时钟
 */
void set_ipcore_clk(uint32_t clk)
{
    // ipcore_clk = clk;
    ciss_set(CI_SS_IPCORE_CLK, (status_t)clk);
}


/**
 * @brief 设置AHB时钟
 * 
 * @param clk AHB时钟
 */
void set_ahb_clk(uint32_t clk)
{
    // ahb_clk = clk;
    ciss_set(CI_SS_AHB_CLK, (status_t)clk);
}


/**
 * @brief 
 * 
 * @param clk APB时钟
 */
void set_apb_clk(uint32_t clk)
{
    // apb_clk = clk;
    ciss_set(CI_SS_APB_CLK, (status_t)clk);
}

/**
 * @brief 设置SRC时钟
 * 
 * @param clk SRC时钟
 */
void set_src_clk(uint32_t clk)
{
    // src_clk = clk;
    ciss_set(CI_SS_SRC_CLK, (status_t)clk);
}


/**
 * @brief 设置晶振时钟
 * 
 * @param clk 晶振时钟
 */
void set_osc_clk(uint32_t clk)
{
    // src_clk = clk;
    ciss_set(CI_SS_EXT_OSC_CLK, (status_t)clk);
}

/**
 * @brief 设置systick时钟
 * 
 * @param clk systick时钟
 */
void set_systick_clk(uint32_t clk)
{
    // systick_clk = clk;
    ciss_set(CI_SS_SYSTEMTICK_CLK, (status_t)clk);
}


/**
 * @brief 初始化 maskrom lib
 * 
 */
void maskrom_lib_init(void)
{
    //mprintf("maskrom lib mark %s!\n",MASK_ROM_LIB_FUNC->ci_lib_romruntime);
    //mprintf("maskrom lib ver %d!\n",MASK_ROM_LIB_FUNC->verison);
	MASK_ROM_LIB_FUNC->init_lib_romruntime_p();
	extern char *sbrk(int incr);
	extern void __malloc_lock(struct _reent *p);
	extern void __malloc_unlock(struct _reent *p);
	MASK_ROM_LIB_FUNC->newlibcfunc.NEWLib_Set_Func_p((void *)sbrk,(void *)__malloc_lock,(void *)__malloc_unlock);
	MASK_ROM_LIB_FUNC->mp3func.MP3Lib_Set_Func_p((void *)(MASK_ROM_LIB_FUNC->newlibcfunc.malloc_p),
												  (void *)(MASK_ROM_LIB_FUNC->newlibcfunc.free_p),
												  (void *)(MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p),
												  (void *)(MASK_ROM_LIB_FUNC->newlibcfunc.memmove_p),
												  (void *)(MASK_ROM_LIB_FUNC->newlibcfunc.memset_p));
}




