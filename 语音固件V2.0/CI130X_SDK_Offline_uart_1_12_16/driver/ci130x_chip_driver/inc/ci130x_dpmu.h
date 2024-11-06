#ifndef CI130X_DPMU_H
#define CI130X_DPMU_H

#include "ci130x_system.h"
#include "ci130x_scu.h"

typedef enum
{
    DPMU_IO_PULL_DISABLE = 0,
    DPMU_IO_PULL_UP     = 1,
    DPMU_IO_PULL_DOWN   = 2,
}Dpmu_Io_Pull_t;

typedef enum
{
    DPMU_IO_DIRECTION_INPUT = 0,
    DPMU_IO_DIRECTION_OUTPUT = 1,
}Dpmu_Io_Direction_t;

typedef enum
{
    DPMU_LOWPOWER_NO_MODE = 0,
    DPMU_LOWPOWER_SLEEP_MODE = 1,
    DPMU_LOWPOWER_DEEP_SLEEP_MODE = 2,
	DPMU_LOWPOWER_BOTH_MODE = 3,
}Dpmu_Lowpower_Mode_t;

/**
 * @brief 系统退出低功耗进入系统复位流程时是否复位
 *
 */
typedef enum
{
    DPMU_WAKEUP_GPIO             = 0, //aon gpio模块
    DPMU_WAKEUP_IWDG             = 1, //iwdg模块
    DPMU_WAKEUP_EFUSE            = 2, //efuse ctrl模块
    DPMU_WAKEUP_TIMER01_GPWM01   = 3, //timer0/1_gpwm0/1模块
    DPMU_WAKEUP_IO_REG           = 4, //DPMU里IO相关控制寄存器
    DPMU_WAKEUP_PMU_RC_REG       = 5, //DPMU里PMU及RC相关控制寄存器
}Dpmu_Wakeup_Reset_Cfg_t;

/**
 * @brief 晶振功能
 *
 */
typedef enum
{
    DPMU_XTAL_EN      = 10,    //晶振功能开始使能
    DPMU_XTAL_BYP     = 9,     //晶振BYPASS
    DPMU_XTAL_RF_EN   = 8,     //晶振反馈使能
}Dpmu_Xtal_Mode_t;

/**
 * @brief IO触发器选择
 *
 */
typedef enum
{
    DPMU_IO_SCHMITT_TRIGGER_DISABLE = 0,  //普通触发器
    DPMU_IO_SCHMITT_TRIGGER_ENABLE = 1,   //施密特触发器
}Dpmu_Io_Schmitt_Trigger_t;

/**
 * @brief IO电压转换率模式
 *
 */
typedef enum
{
    DPMU_IO_SLEW_RATE_SLOW = 0,    //转换速率慢
    DPMU_IO_SLEW_RATE_FAST = 1,    //转换速率快
}Dpmu_Io_Slew_Rate_t;

/**
 * @brief IO驱动强度（数字越大，驱动能力越强）
 *
 */
typedef enum
{
    DPMU_IO_DRIVER_STRENGTH_0 = 0,
    DPMU_IO_DRIVER_STRENGTH_1 = 1,
    DPMU_IO_DRIVER_STRENGTH_2 = 2,
    DPMU_IO_DRIVER_STRENGTH_3 = 3,
}Dpmu_Io_Driver_Strength_t;

/**
 * @brief SRC时钟源选择
 * 
 */
typedef enum
{
    /*系统默认选择，PAD，以及efuse选择*/
    DPMU_SRC_USE_SYSTEM_DEFAULT = 0,
    /*SRC时钟来源于内部RC*/
    DPMU_SRC_USE_INNER_RC = 1,
    /*SRC时钟来源于外部OSC*/
    DPMU_SRC_USE_OUTSIDE_OSC = 3,
}Dpmu_Src_Source_Sel_t;


/**
 * @brief 低电压检测档位设置
 * 
 */
typedef enum
{
    DPMU_VDT_LV_2_4V = 0,
    DPMU_VDT_LV_2_5V,
    DPMU_VDT_LV_2_6V,
    DPMU_VDT_LV_2_7V,
    DPMU_VDT_LV_2_8V,
    DPMU_VDT_LV_2_9V,
    DPMU_VDT_LV_3_0V,
    DPMU_VDT_LV_3_1V,
}Dpmu_Vdt_Lv_t;


/**
 * @brief PMU配置update的更新项
 * 
 */
typedef enum
{
    DPMU_UPDATE_EN_NUM_LDO1 = 0,
    DPMU_UPDATE_EN_NUM_LDO2,
    DPMU_UPDATE_EN_NUM_LDO3,
    DPMU_UPDATE_EN_NUM_VDT,
    DPMU_UPDATE_EN_NUM_TRIM,
}Dpmu_Update_En_Num_t;


/**
 * @brief RC的频率段选择
 * 
 */
typedef enum
{
    DPMU_RC_FREQ_12d288M = 0,
    DPMU_RC_FREQ_2M,
    DPMU_RC_FREQ_4M,
    DPMU_RC_FREQ_8M,
    DPMU_RC_FREQ_16M,
    DPMU_RC_FREQ_24M,
    DPMU_RC_FREQ_32M,
    DPMU_RC_FREQ_64M,
}Dpmu_Rc_Freq_Sel_t;

/**
 * @brief 解锁dpmu配置寄存器
 */
void dpmu_unlock_cfg_config(void);

/**
 * @brief 锁定dpmu配置寄存器
 */
void dpmu_lock_cfg_config(void);

/**
 * @brief 配置管脚复用对应功能
 * @param pin， 管脚名称
 * @param io_function， 第 X 功能选择
 */
void dpmu_set_io_reuse(PinPad_Name pin,IOResue_FUNCTION io_function);

/**
 * @brief 配置管脚数字模拟功能
 * @param pin 管脚名
 * @param adio_mode 数字/模拟，功能选择
 */
void dpmu_set_adio_reuse(PinPad_Name pin,ADIOResue_MODE adio_mode);

/**
 * @brief 配置管脚开漏功能（例如IIC需要引脚配置成此功能）
 * @param pin， 管脚名
 * @param cmd，ENABLE 使能开漏功能，DISABLE，不使能开漏功能
 */
void dpmu_set_io_open_drain(PinPad_Name pin,FunctionalState cmd);

/**
 * @brief 配置管脚上下拉功能
 * @param pin，管脚名
 * @param pull，关闭上下拉、开上拉，开下拉
 */
void dpmu_set_io_pull(PinPad_Name pin,Dpmu_Io_Pull_t pull);

/**
 * @brief 配置管脚方向
 * @param pin，管脚名
 * @param dir，输入、输出
 */
void dpmu_set_io_direction(PinPad_Name pin,Dpmu_Io_Direction_t dir);

/**
 * @brief 配置管脚电压转换率模式
 * @param pin， 管脚名
 * @param slew_rate，电压转换率slow、fast
 */
void dpmu_set_io_slew_rate(PinPad_Name pin,Dpmu_Io_Slew_Rate_t slew_rate);

/**
 * @brief 配置管脚触发器模式
 * @param pin， 管脚名
 * @param schmitt_trigger，触发器模式normal、schmitt trigger
 */
void dpmu_set_io_schmitt_trigger(PinPad_Name pin,Dpmu_Io_Schmitt_Trigger_t schmitt_trigger);

/**
 * @brief 配置管脚驱动强度
 * @param pin， 管脚名
 * @param driver_strength，驱动强度级别选择
 */
void dpmu_set_io_driver_strength(PinPad_Name pin,Dpmu_Io_Driver_Strength_t driver_strength);

/**
 * @brief 配置晶振脚PA0、PA1功能选择（晶振/GPIO）
 * 
 * @param en: ENABLE（GPIO功能），DISABLE（晶振功能）
 */
void dpmu_osc_pad_for_gpio(FunctionalState en);

void dpmu_set_low_power_mode(Dpmu_Lowpower_Mode_t mode);
void dpmu_set_wakeup_int(int32_t wake_int_num,FunctionalState flag);
void dpmu_pll_12d_config(uint32_t clk);
void dpmu_iwdg_reset_none_config(void);
void dpmu_iwdg_reset_system_config(void);
void dpmu_iwdg_reset_bus_config(void);
void dpmu_twdg_reset_none_config(void);
void dpmu_twdg_reset_sysytem_config(void);
void dpmu_twdg_reset_bus_config(void);
void dpmu_software_reset_none_config(void);
void dpmu_software_reset_system_config(void);
void dpmu_software_reset_bus_config(void);
void dpmu_core_reset_none_config();
void dpmu_core_reset_system_config();
void dpmu_core_reset_bus_config();
void dpmu_set_ext_filter_config(Ext_Num num,FunctionalState flag,uint32_t param);
void dpmu_set_iwdg_halt();
void dpmu_clean_iwdg_halt();
void dpmu_set_src_source(Dpmu_Src_Source_Sel_t sel);

void dpmu_pmu_trim_en(bool en);
void dpmu_set_pmu_trim_value(uint8_t val);
uint8_t dpmu_get_pmu_trim_value(void);
void dpmu_pmu_div_resistance_en(bool en);
void dpmu_vdt_lv_set(Dpmu_Vdt_Lv_t lv);
void dpmu_vdt_en(bool en);
void dpmu_ldo2_en(bool en);
void dpmu_ldo3_en(bool en);
void dpmu_ldo1_lv_set(uint8_t lv);
void dpmu_ldo2_lv_set(uint8_t lv);
void dpmu_ldo3_lv_set(uint8_t lv);
void dpmu_config_update_en(Dpmu_Update_En_Num_t num);

void dpmu_rc_freq_sel(Dpmu_Rc_Freq_Sel_t sel);
void dpmu_set_rc_trim_c_value(uint8_t val);
void dpmu_set_rc_trim_f_value(uint8_t val);
void dpmu_set_rc_en(bool en);
void dpmu_set_rc_update_cfg(void);
uint32_t dpmu_get_pll_frequency();

void dpmu_set_vdt_mask(bool en);
void dpmu_osc_pad_cfg_fma(uint8_t num);
void dpmu_para_en_disable(uint32_t device_base);
void dpmu_para_en_enable(uint32_t device_base);
uint32_t dpmu_get_wakeup_state(void);
void dpmu_wakeup_reset_cfg(Dpmu_Wakeup_Reset_Cfg_t model, FunctionalState flag);
void dpmu_use_rc(void);

#endif /*CI130X_DPMU_H*/
