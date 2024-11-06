
/**
 * @file ci_debug_config.h
 * @brief 用于debug的宏开关
 * @version 1.0
 * @date 2019-02-21
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _CI_DEBUG_CONFIG_H_
#define _CI_DEBUG_CONFIG_H_

#include <stdint.h>

/**
 * @addtogroup assist
 * @{
 */

/**
 * @brief assist使能配置
 * 
 */
#define CONFIG_DEBUG_EN                 1

/**
 * @brief systemview使能配置
 * 
 */
#define CONFIG_SYSTEMVIEW_EN            0
#define CONFIG_SYSVIEW_UART_PORT        (UART0)

/**
 * @brief CLI使能配置
 * 
 */
#define CONFIG_CLI_EN                   0

/** @} */

/**
 * @brief CLI串口选择配置
 * 
 */
#define CONFIG_CLI_UART_PORT            (CONFIG_CI_LOG_UART)


extern void vRegisterCLICommands(void);
extern void vUARTCommandConsoleStart( uint16_t usStackSize, unsigned long uxPriority );
extern void vSYSVIEWUARTInit(void);

#endif  /* _CI_DEBUG_CONFIG_H_ */
