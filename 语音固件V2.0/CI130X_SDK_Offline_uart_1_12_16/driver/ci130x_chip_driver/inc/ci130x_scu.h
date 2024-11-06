/**
 * @file ci130x_scu.h
 * @brief 三代芯片scu底层驱动接口头文件
 * @version 0.1
 * @date 2021-07-05
 * 
 * @copyright Copyright (c) 2021  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef CI130X_SCU_H
#define CI130X_SCU_H

#include "ci130x_system.h"
#include "sdk_default_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif 

// 注意，为了使CODE在多种采样率下都能得到整数倍分频，驱动中会自动对主频做一些调整（+/-4092MHz以内），所以实际运行频率肯能与这里配置的频率有差异。
// 准确频率可以通过调用 "dpmu_get_pll_frequency()" 函数获取。
#if USE_EXTERNAL_CRYSTAL_OSC
#define MAIN_FREQUENCY          240000000          /*外部晶振*/
#else
#define MAIN_FREQUENCY          200000000          /*内部晶振*/
#endif

#define SRC_FREQUENCY_NORMAL    12288000U
#define SRC_FREQUENCY_LOWPOWER  12288000U


typedef enum
{
    NMI_IRQ_IWGD    = 0x1,
    NMI_IRQ_WWGD    = 0x2,
    NMI_IRQ_EXT0    = 0x3,
    NMI_IRQ_EXT1    = 0x4,
    NMI_IRQ_TIMER0  = 0x5,
    NMI_IRQ_TIMER1  = 0x6,
    NMI_IRQ_UART0   = 0x7,
    NMI_IRQ_UART1   = 0x8,
    NMI_IRQ_UART2   = 0x9,
    NMI_IRQ_PA      = 0xA,
    NMI_IRQ_PB      = 0xB,
    NMI_IRQ_PC      = 0xC,
    NMI_IRQ_VDT     = 0xC,
    NMI_IRQ_V11_OK  = 0xE,
    NMI_IRQ_ADC     = 0xF,
}Nmi_Irq_t;

typedef enum
{
    SLEEPING_GATE    = 0,
    SLEEPDEEP_GATE   = 1,
    CPU_CORECLK_GATE = 2,
    STCLK_GATE       = 3,
    SRAM0_GATE       = 4,
    SRAM1_GATE       = 5,
    SRAM2_GATE       = 6,
    SRAM3_GATE       = 7,
    SRAM4_GATE       = 8,
    SRAM5_GATE       = 9,
    SRAM6_GATE       = 10,
    ROM_GATE         = 11,
}Sys_Clk_Gate_t;

typedef enum
{
    FIRST_FUNCTION = 0,
    SECOND_FUNCTION = 1,
    THIRD_FUNCTION = 2,
    FORTH_FUNCTION = 3,
    FIFTH_FUNCTION = 4,
    SIXTH_FUNCTION = 5,
}IOResue_FUNCTION;

typedef enum
{
    DIGITAL_MODE = 0,
    ANALOG_MODE = 1,
}ADIOResue_MODE;

typedef enum
{
/***-----PAD-----****************----analgo--****----1st-----****---2nd-----****----3rd----****----4th----****----5th----****----6th----****/
    PA0                  = 0,   /*  OSC_IN       PA_0            PWM5           ---            ---            ---            ---           */
    PA1                  = 1,   /*  OSC_OUT      PA_1            ---            ---            ---            ---            ---           */
    BOOT_SEL_0_PAD       = 2,   /*               BOOT_SEL_0      ---            ---            ---            ---            ---           */
    SPI0_CS_PAD          = 3,   /*               SPI0_CS         ---            ---            ---            ---            ---           */
    SPI0_D1_PAD          = 4,   /*               SPI0_D1         ---            ---            ---            ---            ---           */
    SPI0_D2_PAD          = 5,   /*               SPI0_D2         ---            ---            ---            ---            ---           */
    PA2                  = 6,   /*               PA_2            IIS0_SDI       IIC0_SDA       UART1_TX       PWM0           RC_CLK_V      */
    PA3                  = 7,   /*               PA_3            IIS0_LRCLK     IIC0_SCL       UART1_RX       PWM1           ---           */
    PA4                  = 8,   /*               PA_4            IIS0_SDO       ---            ---            PWM2           ---           */
    PA5                  = 9,   /*               PA_5            IIS0_SCLK      PDM_DAT        UART2_TX       PWM3           ---           */
    PA6                  = 10,  /*               PA_6            IIS0_MCLK      PDM_CLK        UART2_RX       PWM4           ---           */
    PA7                  = 11,  /*               PA_7            PWM0           UART1_TX       EXT_INT0       ---            ---           */
    PB0                  = 12,  /*               PB_0            PWM1           UART1_RX       EXT_INT1       ---            ---           */
    PB1                  = 13,  /*               PB_1            PWM2           UART2_TX       ---            ---            ---           */
    PB2                  = 14,  /*               PB_2            PWM3           UART2_RX       ---            ---            ---           */
    PB3                  = 15,  /*               PB_3            PWM4           IIC0_SDA       ---            ---            ---           */
    PB4                  = 16,  /*               PB_4            PWM5           IIC0_SCL       ---            ---            ---           */
    PB5                  = 17,  /*               PB_5            UART0_TX       IIC0_SDA       PWM1           ---            ---           */
    PB6                  = 18,  /*               PB_6            UART0_RX       IIC0_SCL       PWM2           UART2_TX       ---           */
    PB7                  = 19,  /*               PB_7            UART1_TX       IIC0_SDA       PWM3           PDM_DAT        ---           */
    PC0                  = 20,  /*               PC_0            UART1_RX       IIC0_SCL       PWM4           PDM_CLK        ---           */
    KEY_RSTN_PAD         = 21,  /*               KEY_RSTN        ---            ---            ---            ---            ---           */
    TEST_EN_PAD          = 22,  /*               TEST_EN         ---            ---            ---            ---            ---           */
    SPI0_D0_PAD          = 23,  /*               SPI0_D0         ---            ---            ---            ---            ---           */
    SPI0_CLK_PAD         = 24,  /*               SPI0_CLK        ---            ---            ---            ---            ---           */
    SPI0_D3_PAD          = 25,  /*               SPI0_D3         ---            ---            ---            ---            ---           */
	PC1                  = 26,  /*  AIN5         ---             PC_1           UART2_TX       PWM3           PDM_DAT        ---           */
	PC2                  = 27,  /*  AIN4         ---             PC_2           UART2_RX       PWM2           PDM_CLK        ---           */
	PC3                  = 28,  /*  AIN3         ---             PC_3           IIC0_SDA       PWM1           PDM_DAT        ---           */
	PC4                  = 29,  /*  AIN2         ---             PC_4           IIC0_SCL       PWM0           PDM_CLK        ---           */
	PC5                  = 30,  /*               PC_5            ---            ---            ---            ---            ---           */
    PD0                  = 31,  /*               PD_0            ---            ---            ---            ---            ---           */
    PD1                  = 32,  /*               PD_1            ---            ---            ---            ---            ---           */
    PD2                  = 33,  /*               PD_2            ---            ---            ---            ---            ---           */
    PD3                  = 34,  /*               PD_3            ---            ---            ---            ---            ---           */
    PD4                  = 35,  /*               PD_4            ---            ---            ---            ---            ---           */
    PD5                  = 36,  /*               PD_5            ---            ---            ---            ---            ---           */
}PinPad_Name;

/**
 * @brief IIS号选择
 *
 */
typedef enum
{
    //!IIS0
    IISNum0    = 0,
    //!IIS1_RX
    IISNum1_RX = 1,
    //!IIS1_TX
    IISNum1_TX = 2,
    //!IIS2
    IISNum2    = 3
}IISNumx;

/**
 * @brief IIS主从模式选择
 *
 */
typedef enum
{
    //!IIS做SLAVE
    IIS_SLAVE = 1,
    //!IIS做MASTER
    IIS_MASTER = 0,
}IIS_Mode_Sel_t;

/**
 * @brief IIS MCLK 输入/输出
 *
 */
typedef enum
{
    IIS_MCLK_MODENULL = 0,
    IIS_MCLK_IN = 1,
    IIS_MCLK_OUT = 2,
}IIS_Mclk_Mode_t;

/**
 * @brief IIS SCK和LRCK 输入/输出
 *
 */
typedef enum
{
    IIS_SCKLRCK_MODENULL = 0,
    IIS_SCKLRCK_IN = 1,
    IIS_SCKLRCK_OUT = 2,
}IIS_SckLrck_Mode_t;

/**
 * @brief QSPI控制器模式
 *
 */
typedef enum
{
    //!BOOT模式
    QSPI_MODE_BOOT = 1,
    //!正常模式
    QSPI_MODE_NOMAL = 0,
}Qspi_Mode_t;

/**
 * @brief DMA 中断 channel 选择
 *
 */
typedef enum
{
    //!channel 0的中断
    DMAINT_SEL_CHANNEL0   = 0,
    //!channel 1的中断
    DMAINT_SEL_CHANNEL1   = 1,
    //!两个channel 中断 
    DMAINT_SEL_CHANNELALL = 2,
}DmaInt_Sel_t;

/**
 * @brief DTRFLASH时钟来源选择
 *
 */
typedef enum
{
    //!PLL倍频前的时钟
    DTR_CLK_SEL_SRC_CLK = 0,
    //!PLL倍频后的时钟
    DTR_CLK_SEL_PLL_CLK  = 1,
}Dtr_Clk_Sel_t;

/**
 * @brief MCLK 时钟源SRC的 来源选择
 *
 */
typedef enum
{
    //!来源于 IPCORE
    IIS_SRC_SOURCE_IPCORE   = 0,
    //!来源于 EXT_OSC
    IIS_SRC_SOURCE_EXT_OSC  = 1,
    //!来源于 INTERNAL RC
    IIS_SRC_SOURCE_INTER_RC = 2,
    //!来源于 PAD IN
    IIS_SRC_SOURCE_PAD_IN   = 3,
}IIS_Src_Source_t;

/**
 * @brief IIS MCLK 过采样率选择
 *
 */
typedef enum
{
    //!过采样率128
    IIS_MCLK_FS_128   = 0,
    //!过采样率192
    IIS_MCLK_FS_192   = 1,
    //!过采样率256
    IIS_MCLK_FS_256   = 2,
    //!过采样率384
    IIS_MCLK_FS_384   = 3,
}IIS_Mclk_Fs_t;

/**
 * @brief IIS SCK和LRCK的频率关系比值
 *
 */
typedef enum
{
    //!SCK/LRCK=32
    IIS_SCK_LRCK_WID_32  = 0,
    //!SCK/LRCK=64
    IIS_SCK_LRCK_WID_64  = 1,
}IIS_Sck_Lrck_Wid_t;

/**
 * @brief MCLK 时钟来源选择
 *
 */
typedef enum
{
    //!来源于 SRC0
    IIS_MCLK_SOURCE_SRC0    = 0,
    //!来源于 SRC1
    IIS_MCLK_SOURCE_SRC1    = 1,
    //!来源于 SRC2
    IIS_MCLK_SOURCE_SRC2    = 2,
    //!来源于 PAD IN
    IIS_MCLK_SOURCE_PAD_IN  = 3,
}IIS_Mclk_Source_t;

/**
 * @brief MCLK 选择
 *
 */
typedef enum
{
    //!MCLK0
    IIS_MCLK_MCLK0    = 0,
    //!MCLK1
    IIS_MCLK_MCLK1    = 1,
    //!MCLK2
    IIS_MCLK_MCLK2    = 2,
}IIS_Mclk_t;

/**
 * @brief IIS SCK/LRCK输出来源
 *
 */
typedef enum
{
    //!MCLK0产生的SCK/LRCK
    IIS_CLK_SOURCE_MCLK0     = 0,
    //!MCLK1产生的SCK/LRCK
    IIS_CLK_SOURCE_MCLK1     = 1,
    //!MCLK2产生的SCK/LRCK
    IIS_CLK_SOURCE_MCLK2     = 2,
    //!PAD输入的SCK/LRCK
    IIS_CLK_SOURCE_PAD_IN    = 3,
    //!codec_ad_sck_in
    IIS_CLK_SOURCE_CODEC_AD  = 4,
    //!codec_da_sck_in
    IIS_CLK_SOURCE_CODEC_DA  = 5,
    //!pdm_sck_in
    IIS_CLK_SOURCE_PDM       = 6,
}IIS_Clk_Source_t;

/**
 * @brief codec dac IIS数据来源选择
 *
 */
typedef enum
{
    //!IIS1_TX
    CODEC_DAC_DATA_FROM_IIS1_TX   = 0,
    //!PAD_IN
    CODEC_DAC_DATA_FROM_PAD_IN    = 1,
    //!CODEC_ADC
    CODEC_DAC_DATA_FROM_CODEC_ADC = 2,
    //!PDM
    CODEC_DAC_DATA_FROM_PDM       = 3
}Codec_Dac_Data_Sel_t;

/**
 * @brief pad IIS输出数据来源选择
 *
 */
typedef enum
{
    //!IIS0_TX
    PAD_IIS_DATA_FROM_IIS0_TX   = 0,
    //!IIS1_TX
    PAD_IIS_DATA_FROM_IIS1_TX   = 1,
    //!CODEC_ADC
    PAD_IIS_DATA_FROM_CODEC_ADC = 2,
    //!PDM
    PAD_IIS_DATA_FROM_PDM       = 3
}Pad_IIS_Data_Sel_t;

/**
 * @brief IIS 输入输出选择
 *
 */
typedef enum
{
    //!输出
    IIS_CLK_MODE_OUTPUT    = 0,
    //!输入
    IIS_CLK_MODE_INPUT     = 1,
}IIS_Clk_Mode_t;

/**
 * @brief CODEC AD/DA 选择
 *
 */
typedef enum
{
    //!AD
    CODEC_CHANNEL_AD    = 0,
    //!DA
    CODEC_CHANNEL_DA    = 1,
}Codec_Channel_t;

/**
 * @brief IIS 时钟源 SRC配置结构体
 * 
 */
typedef struct
{
    IIS_Src_Source_t source;     //!SRC SOURCE 来源选择
    uint32_t source_div;         //!分频参数
}IIS_Src_Config_t;

/**
 * @brief IIS MCLK配置结构体
 * 
 */
typedef struct
{
    IIS_Mclk_t mclk;             //!MCLK 编号选择
    IIS_Mclk_Source_t src;       //!MCLK 来源 SRC 编号选择
    IIS_Mclk_Fs_t fs;            //!MCLK 过采样率选择
    IIS_Sck_Lrck_Wid_t sck_lrck; //!SCK/LRCK 比值
}IIS_Mclk_Config_t;

typedef struct
{
    IISNumx device_select;       //0：iis0 1：iis1_tx 2：iis_rx 3：iis2
    IIS_Mode_Sel_t model_sel;     //主从模式
    IIS_Src_Config_t src_cfg;    //时钟来源SRC配置
    IIS_Mclk_Config_t mclk_cfg;  //MCLK配置
    IIS_Clk_Source_t clk_cfg;    //sck/lrck来源
    IIS_Mclk_Mode_t mclk_mode;   //mclk从PAD输出/输入
    IIS_SckLrck_Mode_t clk_mode; //sck和lrck从PAD输出/输入
}IIS_Clk_ConfigTypedef;

/**
 * @brief 解锁系统控制寄存器
 */
void scu_unlock_system_config(void);

/**
 * @brief 解锁复位相关寄存器
 */
void scu_unlock_reset_config(void);

/**
 * @brief 解锁时钟相关寄存器
 */
void scu_unlock_clk_config(void);

/**
 * @brief 锁定系统控制寄存器
 */
void scu_lock_system_config(void);

/**
 * @brief 锁定时钟相关寄存器
 */
void scu_lock_clk_config(void);

/**
 * @brief 锁定复位寄存器
 */
void scu_lock_reset_config(void);

/**
 * @brief 设置外设时钟开关
 * @param device_base ，需要设置的外设基址
 * @param gate ，DISABLE ：关闭 ，ENABLE ：打开
 * @return PARA_ERROR: 参数错误 ，RETURN_OK：配置完成
 */
int32_t scu_set_device_gate(uint32_t device_base, int32_t gate);

/**
 * @brief 配置外设复位
 * @note  配合scu_set_device_reset_Release 使用，先reset,然后release，外设复位完成
 * @param device_base， 设备基地址
 * @return PARA_ERROR: 参数错误 ，RETURN_OK：配置完成
 */
int32_t scu_set_device_reset(uint32_t device_base);

/**
 * @brief 配置外设复位释放
 * @note  配合scu_set_device_reset  使用，先reset,然后release，外设复位完成
 * @param device_base， 设备基地址
 * @return PARA_ERROR: 参数错误 ，RETURN_OK：配置完成
 */
int32_t scu_set_device_reset_release(uint32_t device_base);

int32_t scu_set_ext_wakeup_int(Ext_Num num, FunctionalState cmd);
int32_t scu_clear_ext_int_state(Ext_Num num);
void scu_set_ext_filter_config(Ext_Num num,FunctionalState cmd,uint32_t param);
int8_t scu_para_en_disable(uint32_t device_base);
int8_t scu_para_en_enable(uint32_t device_base);
int32_t scu_set_div_parameter(uint32_t device_base,uint32_t div_num);
void scu_spiflash_no_boot_set(void);
void scu_run_in_flash(void);
void scu_run_not_in_flash(void);
int32_t scu_get_system_reset_state(void);
int32_t scu_set_system_clk_gate(Sys_Clk_Gate_t base,FunctionalState gate);
void scu_set_wwdg_halt();
void scu_clean_wwdg_halt();
void scu_set_dma_mode(DmaInt_Sel_t channel);
void scu_sel_dtrflash_clk(Dtr_Clk_Sel_t clk);
void scu_wait_pll_lock_state();

//IIS相关系统接口
void scu_iis_src_config(IIS_Src_Config_t *config,IIS_Mclk_Source_t src);
void scu_iis_mclk_Config(IIS_Mclk_Config_t *config);
void scu_iis_pad_mclk_config(IIS_Mclk_Source_t src, IIS_Clk_Mode_t mode);
void scu_iis_codec_mclk_config(Codec_Channel_t channel, IIS_Mclk_Source_t src);
void scu_iis_pdm_mclk_config(IIS_Mclk_Source_t src);
void scu_iis_clk_config(IISNumx device, IIS_Clk_Source_t clk_source);
void scu_iis_pad_clk_config(IIS_Clk_Source_t clk_source, IIS_Clk_Mode_t mode);
void scu_iis_codec_dac_data_config(Codec_Dac_Data_Sel_t src);
void scu_iis_pad_data_config(Pad_IIS_Data_Sel_t src);
void iis_clk_config(IIS_Clk_ConfigTypedef* config);

void scu_rc_trim_en(bool en);
void scu_rc_trim_state_clear(void);
void scu_nmi_irq_cfg(Nmi_Irq_t irq);

uint8_t scu_get_rc_trim_state(void);
uint8_t scu_get_rc_trim_c_value(void);
uint8_t scu_get_rc_trim_f_value(void);

void dsu_init(unsigned int addr);

#ifdef __cplusplus
}
#endif 

#endif /*CI130X_SCU_H*/

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/