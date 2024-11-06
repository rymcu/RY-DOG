/**
 * @file ci110x_lowpower.c
 * @brief 电源管理驱动
 * @version 1.0
 * @date 2019-02-23
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#if 1

#include "ci130x_lowpower.h"
#include "ci130x_scu.h"
#include "ci_log.h"
#include "ci_assert.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "sdk_default_config.h"
#include "ci130x_uart.h"


typedef int32_t (*exit_low_power_mode)(void);
static exit_low_power_mode g_switch_to_normal = NULL;
static power_mode_t g_curr_power_mode = POWER_MODE_NORMAL;
static volatile bool g_scu_int = false;
static unsigned char backup_int[32];
static unsigned int clkgate_state[4] = {0};
typedef void(*lower_hook_fn)(void);
lower_hook_fn user_enter_lowpower = NULL;
lower_hook_fn user_exit_lowpower = NULL;

/**
 * @brief 保存外设时钟gata以及NVIC
 * 
 * @attention 不能重复保存调用push前必须调用pop，目前由power_mode_switch保证
 * 
 */
static void push_device_status(void)
{
	/*for(int i = 0; i < ECLIC_NUM_INTERRUPTS;i++)
	{
		backup_int[i]= *(volatile uint8_t *)(SOC_ECLIC_CTRL_ADDR+ECLIC_INT_IE_OFFSET+ (i+19) * 4);
		*(volatile uint8_t *)(SOC_ECLIC_CTRL_ADDR+ECLIC_INT_IE_OFFSET+ (i+19) * 4) = 0;
	}*/

    // Unlock_Ckconfig();
    // Unlock_Rstconfig();
    // clkgate_state[1] = SCU->PRE_CLKGATE0_CFG;
    // clkgate_state[2] = SCU->PRE_CLKGATE1_CFG;
}


/**
 * @brief 恢复外设时钟gata以及NVIC
 * 
 */
static void pop_device_status(void)
{
    /*  恢复系统状态 */
    // Unlock_Ckconfig();
    // Unlock_Rstconfig();
    // SCU->PRE_CLKGATE0_CFG = clkgate_state[1];
    // SCU->PRE_CLKGATE1_CFG = clkgate_state[2];
  
	/*for(int i = 0; i < ECLIC_NUM_INTERRUPTS;i++)
	{
		*(volatile uint8_t *)(SOC_ECLIC_CTRL_ADDR+ECLIC_INT_IE_OFFSET+ (i+19) * 4) = backup_int[i];
	}*/
}


/*************************正常全速模式 多命令词，全功能*********************************/
/**
 * @brief 切换到正常模式
 * 
 * @return int32_t 
 */
static int32_t switch_normal_mode(void)
{
    if(NULL != g_switch_to_normal)
    {
        if(RETURN_OK != g_switch_to_normal())
        {
            return RETURN_ERR; 
        }
        else
        {
            g_switch_to_normal = NULL;
        }
    }
    return RETURN_OK;
}
/***************************************************************************************/



/************************* 降频模式 仅保证识别功能 模式 *********************************/
static int32_t exit_down_frequency_mode(void)
{
    //恢复正常模式频率
    uint32_t main_freq = MAIN_FREQUENCY;
    dpmu_pll_config(SRC_FREQUENCY_LOWPOWER, main_freq);
    main_freq = dpmu_get_pll_frequency();
    set_ipcore_clk(main_freq);
    set_ahb_clk(main_freq);
    set_apb_clk(main_freq/2);

    // 重新配置并开启打印串口
    UARTPollingConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);

    //配置串口波特率
    #if MSG_COM_USE_UART_EN  
    //Scu_Setdiv_Parameter(UART_PROTOCOL_NUMBER,8);  
    #endif

    //配置IIS时钟，当前方案IIS使用RC时钟作为时钟源，所以不用配置。


    /* 恢复外设gata */
    pop_device_status();
    
    if(user_exit_lowpower != NULL)
    {
        user_exit_lowpower();
    }

    return RETURN_OK;
}


static int32_t switch_down_frequency_mode(void)
{
    if(g_curr_power_mode == POWER_MODE_NORMAL)
    {
    	/*for(int i = 0; i < ECLIC_NUM_INTERRUPTS;i++)
    	{
    		backup_int[i]= *(volatile uint8_t *)(SOC_ECLIC_CTRL_ADDR+ECLIC_INT_IE_OFFSET+ (i+19) * 4);
    	}*/

        /* 保存外设gata */
        // Unlock_Ckconfig();
        // Unlock_Rstconfig();
        // clkgate_state[1] = SCU->PRE_CLKGATE0_CFG;
        // clkgate_state[2] = SCU->PRE_CLKGATE1_CFG;
    }

    // if(g_curr_power_mode == POWER_MODE_OSC_FREQUENCY)
    // {
    //     /* 系统升频 */
    //     Scu_Setdiv_Parameter(M4_SYSTICK_BASE,12);  //90/12*2=15M

    //     #if !USE_IIS_PLL_TO_LOWPOWER
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS3_BASE,20);   //82/20=4.1M
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,ENABLE);
    //     iis3_div_para_change_to_half_of_180M_clk();
    //     iis1_div_para_change_to_half_of_180M_clk();
    //     #endif
    //     #if MSG_COM_USE_UART_EN  
    //     Scu_Setdiv_Parameter(UART_PROTOCOL_NUMBER,4);  
    //     #endif

    //     scu_pll_12d288_config(MAIN_FREQUENCY);
    //     set_ipcore_clk(MAIN_FREQUENCY/2);
    //     set_ahb_clk(MAIN_FREQUENCY/2);
    //     set_apb_clk(MAIN_FREQUENCY/4);
    // }


    //NVIC->ICER[0] = ~0x330; //除了VAD,DNN,IIS_DMA0，其他中断关闭
    /* TODO: 这里全关闭再使能会出错
    NVIC_EnableIRQ(IIS_DMA0_IRQn);
    NVIC_EnableIRQ(VAD_IRQn);
    NVIC_EnableIRQ(DNN_IRQn);*/

    /* TODO: 关闭不需要的外设时钟,DNN和FFT最好在不需要使用时关闭，那样这里就只需要留下iisdma0、vad、iis3、codec就可以了 */
    // Unlock_Ckconfig();
    // Unlock_Rstconfig();
    // SCU->PRE_CLKGATE0_CFG = PLL_CLKGATE0|DNN_CLKGATE0|QSPI_CLKGATE0|CODEC_CLKGATE0|IISDMA0_CLKGATE0;
    #if MSG_COM_USE_UART_EN  
    // if(UART_PROTOCOL_NUMBER == HAL_UART0_BASE) 
    // {
    //     SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1|UART0_CLKGATE1;
    //     //NVIC_EnableIRQ(UART0_IRQn);
    // }
    // else if(UART_PROTOCOL_NUMBER == HAL_UART1_BASE) 
    // {
    //     SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1|UART1_CLKGATE1;
    //     //NVIC_EnableIRQ(UART1_IRQn);
    // }
    // #else
    // SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1;
    #endif

    // if(g_curr_power_mode != POWER_MODE_OSC_FREQUENCY)
    {
        // 关闭外设
        scu_set_device_gate((unsigned int)UART0, DISABLE);

        /* 系统降频 */
        uint32_t main_freq = 140000000;//120000000;//140000000;
        dpmu_pll_config(SRC_FREQUENCY_LOWPOWER, main_freq);
        main_freq = dpmu_get_pll_frequency();
        set_ipcore_clk(main_freq);
        set_ahb_clk(main_freq);
        set_apb_clk(main_freq/2);

        
        // 重新配置并开启打印串口
        UARTPollingConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);

    //     /* 系统降频 */
    //     // scu_pll_12d288_config(MAIN_FREQUENCY);
    //     set_ipcore_clk(MAIN_FREQUENCY/2);
    //     set_ahb_clk(MAIN_FREQUENCY/2);
    //     set_apb_clk(MAIN_FREQUENCY/4);
    //     // Scu_Setdiv_Parameter(M4_SYSTICK_BASE,12);  //90/12*2=15M

    //     #if !USE_IIS_PLL_TO_LOWPOWER
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS3_BASE,20);   //82/20=4.1M
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,ENABLE);
    //     // iis3_div_para_change_to_half_of_180M_clk();
    //     // iis1_div_para_change_to_half_of_180M_clk();
    //     #endif
    //     #if MSG_COM_USE_UART_EN  
    //     // Scu_Setdiv_Parameter(UART_PROTOCOL_NUMBER,4);  
    //     #endif
    }

    // if(user_enter_lowpower != NULL)
    // {
    //     user_enter_lowpower();
    // }

    g_switch_to_normal = exit_down_frequency_mode;
    return RETURN_OK;
}
/***************************************************************************************/

#if 1
/******** PLL bypass 模式 仅保证VAD功能 需要在VAD START后切换到可以正常识别的模式 ******/
static int32_t exit_osc_frequency_mode(void)
{
    // Scu_Setdiv_Parameter(M4_SYSTICK_BASE,24);  //180/24*2=15M
    // #if MSG_COM_USE_UART_EN  
    // Scu_Setdiv_Parameter(UART_PROTOCOL_NUMBER,8);  
    // #endif
    // #if !USE_IIS_PLL_TO_LOWPOWER
    //     #if USE_OUTSIDE_IIS_TO_ASR
    //     //Scu_SetDeviceGate(HAL_IIS1_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS1_BASE,40);    //164/40=4.1M
    //     #else
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS3_BASE,40);    //164/40=4.1M
    //     #endif
    // #endif
    // scu_pll_12d288_config(MAIN_FREQUENCY*2);
    // set_ipcore_clk(MAIN_FREQUENCY);
    // set_ahb_clk(MAIN_FREQUENCY);
    // set_apb_clk(MAIN_FREQUENCY/2);
    // #if !USE_IIS_PLL_TO_LOWPOWER
    //     #if USE_OUTSIDE_IIS_TO_ASR
    //     #else
   
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,ENABLE);
    //     #endif
    // #endif
    
    
    // 关闭外设
        scu_set_device_gate((unsigned int)UART0, DISABLE);

        /* 系统降频 */
        uint32_t main_freq = MAIN_FREQUENCY;//140000000;
        dpmu_pll_config(SRC_FREQUENCY_LOWPOWER, main_freq);
        main_freq = dpmu_get_pll_frequency();
        set_ipcore_clk(main_freq);
        set_ahb_clk(main_freq);
        set_apb_clk(main_freq/2);

        
        // 重新配置并开启打印串口
        UARTPollingConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);

    /* 恢复外设gata */
    pop_device_status();
    
    if(user_exit_lowpower != NULL)
    {
        user_exit_lowpower();
    }

    return RETURN_OK;
}


static int32_t switch_osc_frequency_mode(void)
{
    // if(g_curr_power_mode == POWER_MODE_NORMAL)
    // {
    // 	/*for(int i = 0; i < ECLIC_NUM_INTERRUPTS;i++)
    // 	{
    // 		backup_int[i]= *(volatile uint8_t *)(SOC_ECLIC_CTRL_ADDR+ECLIC_INT_IE_OFFSET+ (i+19) * 4);
    // 	}*/

    //     /* 保存外设gata */
    //     Unlock_Ckconfig();
    //     Unlock_Rstconfig();
    //     clkgate_state[1] = SCU->PRE_CLKGATE0_CFG;
    //     clkgate_state[2] = SCU->PRE_CLKGATE1_CFG;
    // }

    // //NVIC->ICER[0] = ~0x230; //除了VAD,DNN,IIS_DMA0，其他中断关闭 TODO:实测发现关闭dnn中断会使唤醒率降低？？
    // /* TODO: 这里全关闭再使能会出错
    // NVIC_EnableIRQ(IIS_DMA0_IRQn);
    // NVIC_EnableIRQ(VAD_IRQn);*/
    
    // /* 关闭不需要的外设时钟,这里就只需要留下iisdma0、vad、iis3、codec就可以了 */
    // Unlock_Ckconfig();
    // Unlock_Rstconfig();
    // SCU->PRE_CLKGATE0_CFG = PLL_CLKGATE0|DNN_CLKGATE0|QSPI_CLKGATE0|CODEC_CLKGATE0|IISDMA0_CLKGATE0;
    // #if MSG_COM_USE_UART_EN  
    // if(UART_PROTOCOL_NUMBER == HAL_UART0_BASE) 
    // {
    //     SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1|UART0_CLKGATE1;
    //     //NVIC_EnableIRQ(UART0_IRQn);
    // }
    // else if(UART_PROTOCOL_NUMBER == HAL_UART1_BASE) 
    // {
    //     SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1|UART1_CLKGATE1;
    //     //NVIC_EnableIRQ(UART1_IRQn);
    // }
    // #else
    // SCU->PRE_CLKGATE1_CFG = MCLK3_CLKGATE1|IIS3_CLKGATE1|MCLK1_CLKGATE1|IIS1_CLKGATE1;
    // #endif

    // /* PLL bypass */
    // scu_pll_12d288_config(0);
    // set_ipcore_clk(6144000);
    // set_ahb_clk(6144000);
    // set_apb_clk(3072000);
    // Scu_Setdiv_Parameter(M4_SYSTICK_BASE,2); //6.144/2*2=6.144M
    // set_systick_clk(6144000);
    // #if !USE_IIS_PLL_TO_LOWPOWER
    //     #if USE_OUTSIDE_IIS_TO_ASR
    //     //Scu_SetDeviceGate(HAL_IIS1_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS1_BASE,2);   //6.144/2=3.072M
    //     //Scu_SetDeviceGate(HAL_IIS1_BASE,ENABLE);
    //     #else
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,DISABLE);
    //     //Scu_Setdiv_Parameter(HAL_IIS3_BASE,2);   //6.144/2=3.072M
    //     //Scu_SetDeviceGate(HAL_IIS3_BASE,ENABLE);
    //     #endif
    // #endif
    // #if MSG_COM_USE_UART_EN  
    // Scu_Setdiv_Parameter(UART_PROTOCOL_NUMBER,2);  //6.144/2=3.072M
    // //TODO: uart需要重新配置
    // #endif
    
    
    
    // 关闭外设
        scu_set_device_gate((unsigned int)UART0, DISABLE);

        /* 系统降频 */
        uint32_t main_freq = 60000000;//140000000;
        dpmu_pll_config(SRC_FREQUENCY_LOWPOWER, main_freq);
        main_freq = dpmu_get_pll_frequency();
        set_ipcore_clk(main_freq);
        set_ahb_clk(main_freq);
        set_apb_clk(main_freq/2);

        
        // 重新配置并开启打印串口
        UARTPollingConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);

    if(user_enter_lowpower != NULL)
    {
        user_enter_lowpower();
    }

    g_switch_to_normal = exit_osc_frequency_mode;
    return RETURN_OK;
}
/***************************************************************************************/
#endif

/**
 * @brief 注册进出低功耗模式切换用户扩展函数
 * 
 * @param enter_lowpower_fn 进入低功耗用户扩展函数指针
 * @param exit_lowpower_fn 退出低功耗用户扩展函数指针
 */
void register_lowpower_user_fn(void * enter_lowpower_fn,void * exit_lowpower_fn)
{
    user_enter_lowpower = (lower_hook_fn)enter_lowpower_fn;
    user_exit_lowpower = (lower_hook_fn)exit_lowpower_fn;
}

static uint32_t enter_critical(void)
{
    uint32_t  base_pri = eclic_get_mth();
    eclic_set_mth((0x6<<5)|0x1f); //0XDF
    return base_pri;
}

static void exit_critical(uint32_t base_pri)
{
    eclic_set_mth(base_pri);
}

/**
 * @brief 切换电源模式
 * 
 * @param power_mode 电源模式，可选参数：
 * @arg POWER_MODE_NORMAL            正常模式
 * @arg POWER_MODE_DOWN_FREQUENCY_EN 降频模式
 * @arg POWER_MODE_OSC_FREQUENCY_EN  晶振模式
 * @arg POWER_MODE_SLEEP             休眠模式
 * @arg POWER_MODE_DEEP_SLEEP        深度睡眠
 * 
 * @note 系统初始化完成后应处于normal正常模式
 * 
 * @return 切换前的电源模式，切换失败返回err
 * 
 */
power_mode_t power_mode_switch(power_mode_t power_mode)
{
    int32_t ret = RETURN_ERR;
    power_mode_t  last_power_mode = g_curr_power_mode;

    uint32_t  base_pri = enter_critical();

    if(power_mode != last_power_mode)
    {
        switch (power_mode)
        {
            case POWER_MODE_NORMAL:
                ci_loginfo(LOG_PM,"switch to NORMAL!\n");
                ret = switch_normal_mode();
                break;

        #if POWER_MODE_DOWN_FREQUENCY_EN
            case POWER_MODE_DOWN_FREQUENCY:
                ci_loginfo(LOG_PM,"switch to DOWN_FREQUENCY!\n");
                ret = switch_down_frequency_mode();
                break;
        #endif
        
        #if POWER_MODE_OSC_FREQUENCY_EN
            case POWER_MODE_OSC_FREQUENCY:
                ci_loginfo(LOG_PM,"switch to OSC_FREQUENCY!\n");
                ret = switch_osc_frequency_mode();
                break;
        #endif

        #if POWER_MODE_SLEEP_EN
            case POWER_MODE_SLEEP:
                ci_loginfo(LOG_PM,"switch to SLEEP!\n");
                ret = switch_sleep_mode();
                break;
        #endif

        #if POWER_MODE_DEEP_SLEEP_EN
            case POWER_MODE_DEEP_SLEEP:
                ci_loginfo(LOG_PM,"switch to DEEP_SLEEP!\n");
                ret = switch_deep_sleep_mode();
                break;
        #endif

            default:
                ci_logwarn(LOG_PM,"warr : switch to NORMAL!\n");
                break;
        }

        if(ret != RETURN_OK)
        {
            //ci_logerr(LOG_PM,"switch err!\n");
            g_curr_power_mode = POWER_MODE_NORMAL;
            last_power_mode = POWER_MODE_ERR;
        }
        else
        {
            g_curr_power_mode = power_mode;
            //ci_loginfo(LOG_PM,"switch ok!\n");
        }
    }

    exit_critical(base_pri);
    
    return last_power_mode;
}

/**
 * @brief 查询当前所处的模式
 * 
 * @return power_mode_t 当前所处的模式
 */
power_mode_t get_curr_power_mode(void)
{
    return g_curr_power_mode;
}


#if 0
/*************************功耗模式模板**************************************************/
/****************************************************************************************
切换频率流程:
    1.保存外设gate
    2.通过两种方式之一降低AHB时钟进而降低cpu时钟
        a.关闭PLL，采用外部晶振12M，此时AHB12M APB6M IP_CORE12M
        b.不关闭PLL，仅降低AHB分频，此时IP_CORE168M
    3.配置需要保留时钟的外设gate，并关闭不需要的外设gate
    4.配置相关外设分频以保证需要的外设工作正常
退出流程:
    1.恢复cpu时钟
    2.恢复外设gata
****************************************************************************************/
static int32_t exit_template_mode(void)
{
    /* 1.恢复CPU时钟 */


    /* 2.恢复外设gata */
    //pop_device_status();

    return RETURN_OK;
}

static int32_t switch_template_mode(void)
{
    /* 1.保存外设gata */
    push_device_status();

    /* 2.降低cpu时钟 */

    /* 3.配置外设gata */

    /* 4.配置外设分频及NVIC中断 */

    g_switch_to_normal = exit_template_mode;
    return RETURN_OK;
}
/***************************************************************************************/


/*******************************时钟查询打印 debug用************************************/
typedef enum
{
    CPU_CLK = 0,
    AHB_CLK,
    IP_CORE,
    APB_CLK,
    SYS_PLL,

    ASR_CLK,
    VAD_CLK,
    IIS_CLK,
    SPI_CLK,
    UART1_CLK,
}
clk_info_t;

/**
 * @brief 通过读SCU寄存器获取时钟
 * 
 * @param device 外设
 * @return uint32_t 时钟，单位HZ，返回0表示该模块目前OFF状态
 */
uint32_t Scu_GetClk(clk_info_t device)
{
    uint32_t CLK = 0;
    uint32_t cpu_clk, ahb_clk,sys_pll,ip_core,apb_clk;
    uint32_t reg[3];
    uint8_t pll_flag;

    Unlock_Ckconfig();
    Unlock_Rstconfig();

    reg[0] = SCU->SYS_CTRL;
    reg[1] = SCU->SYS_PLL_REG;
    reg[2] = SCU->CLKDIV_PAPAM0;
    pll_flag = (reg[0] & (0x1 << 3)) ? 1 : 0;
    sys_pll = 12000000 * (reg[1] & 0x7F) / ((reg[1] >> 8) & 0xf) / (0x1 << ((reg[1] >> 12) & 0x1) + (2 * ((reg[1] >> 13) & 0x1)));
    ahb_clk = pll_flag ? 12000000 : sys_pll / 2;
    ip_core = pll_flag ? 12000000 : sys_pll / 2;
    ahb_clk /= (reg[2] & 0x7f);
    apb_clk = ahb_clk/((reg[2]>>8) & 0x7f);
    cpu_clk = ahb_clk;
    sys_pll = pll_flag ? 0 : sys_pll;
  
    switch (device)
    {
        case CPU_CLK:
            CLK = cpu_clk;
            break;

        case AHB_CLK:
            CLK = ahb_clk;
            break;
        
        case IP_CORE:
            CLK = ip_core;
            break;

        case APB_CLK:
            CLK = apb_clk;
            break;

        case SYS_PLL:
            CLK = sys_pll;
            break;

        // TODO: 添加相关外设
        case ASR_CLK:
            break;
        case VAD_CLK:
            CLK = (SCU->PRE_CLKGATE0 >> 27) & 0x1 ? ahb_clk / ((SCU->CLKDIV_PAPAM1 >> 8) & 0x3f) : 0;
            break;
        case IIS_CLK:
            break;
        case SPI_CLK:
            break;
        case UART1_CLK:
            CLK = ( SCU->PRE_CLKGATE1 >> 17) & 0x1 ? ip_core / ((SCU->CLKDIV_PAPAM4 >> 0) & 0x3f) : 0;
            break;

        default:
            break;
    }

    return CLK;
}


#define CLKPRINTF(name,device)                                           \
    do{                                                                  \
        uint32_t _CLK = Scu_GetClk(device);                              \
        if(_CLK != 0)                                                    \
        {                                                                \
            ci_logdebug(LOG_PM,"%s:\t\t\t%d Hz\n",(name), _CLK);         \
        }                                                                \
        else                                                             \
        {                                                                \
            ci_logdebug(LOG_PM,"%s:\t\t\tOFF\n",(name));                 \
        }                                                                \
    }while(0)


/**
 * @brief 打印系统CLK
 * 
 * @param run_freq_only 1:仅打印系统时钟 0:打印全部外设时钟
 */
void printRunFrequency(int32_t run_freq_only)
{
    ci_logdebug(LOG_PM,"**********************clk monitor**************************\n");
    ci_logdebug(LOG_PM,"***********************************************************\n");
    ci_logdebug(LOG_PM,"CPU:\t\t\t%d Hz\n", Scu_GetClk(CPU_CLK));
    ci_logdebug(LOG_PM,"AHB:\t\t\t%d Hz\n", Scu_GetClk(AHB_CLK));
    ci_logdebug(LOG_PM,"APB:\t\t\t%d Hz\n", Scu_GetClk(APB_CLK));
    ci_logdebug(LOG_PM,"IP_C:\t\t\t%d Hz\n", Scu_GetClk(IP_CORE));
    CLKPRINTF("SYSPLL", SYS_PLL);
    if (!run_freq_only)
    {
        ci_logdebug(LOG_PM,"***********************************************************\n");
        CLKPRINTF("VAD", VAD_CLK);
        CLKPRINTF("UART1", UART1_CLK);
    }
    ci_logdebug(LOG_PM,"***********************************************************\n");
    ci_logdebug(LOG_PM,"\r\n");
}
/***************************************************************************************/
#endif

#endif
