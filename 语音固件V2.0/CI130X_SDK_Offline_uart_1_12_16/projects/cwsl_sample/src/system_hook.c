#include <stdlib.h>

#include "command_info.h"
#include "voice_module_uart_protocol.h"
#include "i2c_protocol_module.h"
#include "ci130x_core_misc.h"
#include "baudrate_calibrate.h"
#include "ci_log.h"
#include "system_hook.h"
//#include "ci130x_dpmu.h"



/**
 * @brief 系统启动事件钩子，系统启动时会调用此函数
 * 
 */
__WEAK void sys_power_on_hook(void)
{
    #if MSG_COM_USE_UART_EN
    // #if UART_BAUDRATE_CALIBRATE
    // baudrate_calibrate_stop();
    // #endif
    #if (UART_PROTOCOL_VER == 1)
    #elif (UART_PROTOCOL_VER == 2)
    vmup_send_notify(VMUP_MSG_DATA_NOTIFY_POWERON);
    #elif (UART_PROTOCOL_VER == 255)
    deal_welcome_for_usr();
    #endif
    #endif
}

/**
 * @brief 系统唤醒事件钩子，系统进入唤醒状态时会调用此函数
 * 
 */
__WEAK void sys_weakup_hook(void)
{
    #if MSG_COM_USE_UART_EN
    #if UART_BAUDRATE_CALIBRATE
    baudrate_calibrate_stop();
    #endif
    #if (UART_PROTOCOL_VER == 1)
    #elif (UART_PROTOCOL_VER == 2)
    vmup_send_notify(VMUP_MSG_DATA_NOTIFY_WAKEUPENTER);
    #endif
    #endif
}


/**
 * @brief 系统退出唤醒事件钩子，系统退出唤醒状态时会调用此函数
 * 
 */
__WEAK void sys_sleep_hook(void)
{
    #if MSG_COM_USE_UART_EN
    #if UART_BAUDRATE_CALIBRATE
    baudrate_calibrate_stop();
    #endif
    #if (UART_PROTOCOL_VER == 1)
    uart_send_exit_wakeup();
    #elif (UART_PROTOCOL_VER == 2)
    vmup_send_notify(VMUP_MSG_DATA_NOTIFY_WAKEUPEXIT);
    #elif (UART_PROTOCOL_VER == 255)
    deal_exit_wakeup_for_usr();
    #endif
    #endif
    
    #if MSG_USE_I2C_EN
    i2c_sleep_deal();
    #endif
}

/**
 * @brief 语音识别事件钩子，语音模块输出识别结果时会调用此函数
 * 
 */
__WEAK void sys_asr_result_hook(cmd_handle_t cmd_handle, uint8_t asr_score)
{
    #if MSG_COM_USE_UART_EN
    #if UART_BAUDRATE_CALIBRATE
    baudrate_calibrate_stop();
    #endif
    #if (UART_PROTOCOL_VER == 1)
    uart_send_AsrResult(cmd_info_get_command_id(cmd_handle),asr_score);
    #elif (UART_PROTOCOL_VER == 2)
    vmup_send_asr_result_cmd(cmd_handle, asr_score);
    #elif (UART_PROTOCOL_VER == 255)
    deal_asr_msg_by_cmd_id_for_usr(cmd_handle);
    #endif
    #endif
    
    #if MSG_USE_I2C_EN
    i2c_product_send_msg(cmd_handle);
    #endif
}


