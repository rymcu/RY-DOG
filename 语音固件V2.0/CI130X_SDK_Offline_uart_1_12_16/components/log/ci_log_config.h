/**
 * @file ci_log_config.h
 * @brief sdk log 输出配置宏
 * @version 0.1
 * @date 2019-01-09
 *
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 *
 */
#ifndef _CI_LOG_CONFIG_H_
#define _CI_LOG_CONFIG_H_

#include "ci130x_system.h"
#include "sdk_default_config.h"

/* log打印等级 参考Andriod Logcat */
#define CI_LOG_VERBOSE             (1 << 6)
#define CI_LOG_DEBUG               (1 << 5)
#define CI_LOG_INFO                (1 << 4)
#define CI_LOG_WARN                (1 << 3)
#define CI_LOG_ERROR               (1 << 2)
#define CI_LOG_ASSERT              (1 << 1)
#define CI_LOG_NONE                (1 << 0)

/* 打印模块开关 */
#if (CONFIG_CI_LOG_UART != 0)
#define CONFIG_CI_LOG_EN            1
#define CONFIG_SMT_PRINTF_EN        1
#else
#define CONFIG_CI_LOG_EN            0
#define CONFIG_SMT_PRINTF_EN        0
#endif

/*only print long log will save time, because uart fifo so big, 64byte*/
#define UART_LOG_UASED_INT_MODE     0
#define UART_LOG_BUFF_SIZE          512
/* 日志打印使用的串口 */
#define CONFIG_CI_LOG_UART_PORT     ((UART_TypeDef*)CONFIG_CI_LOG_UART)

/* 打印等级 */
#define LOG_ASSERT                  CI_LOG_ERROR
#define LOG_SYS_INFO                CI_LOG_DEBUG

#define LOG_PLAY_CARD               CI_LOG_INFO
#define LOG_SPI                     CI_LOG_INFO
#define LOG_TWDT                    CI_LOG_DEBUG
#define LOG_IWDT                    CI_LOG_DEBUG
#define LOG_SDIO                    CI_LOG_INFO
#define LOG_IIS                     CI_LOG_INFO
#define LOG_CACHE                   CI_LOG_INFO
#define LOG_PM                      CI_LOG_INFO
#define LOG_IIC_DRIVER              CI_LOG_INFO
#define LOG_QSPI_DRIVER             CI_LOG_INFO
#define LOG_GPIO_DRIVER             CI_LOG_INFO
#define LOG_NORMAL_SPI_DRIVER       CI_LOG_DEBUG
#define LOG_NORMAL_SPI_SD           CI_LOG_DEBUG

#define LOG_FLASH_CTL               CI_LOG_INFO
#define LOG_WIFI_EVENT              CI_LOG_DEBUG
#define LOG_DUERAPP                 CI_LOG_INFO
#define LOG_AUDIO_PLAY              CI_LOG_INFO
#define LOG_AUDIO_GET_DATA          CI_LOG_INFO
#define LOG_USER                    CI_LOG_DEBUG
#define LOG_SYS_MONITOR             CI_LOG_INFO
#define LOG_COM_UART                CI_LOG_DEBUG
#define LOG_NVDATA                  CI_LOG_INFO
#define LOG_CMD_INFO                CI_LOG_INFO
#define LOG_FAULT_INT               CI_LOG_VERBOSE
#define LOG_ASR_PCM_BUF             CI_LOG_ERROR
#define LOG_AUDIO_IN                CI_LOG_VERBOSE
#define LOG_ASR_DECODER             CI_LOG_INFO
#define LOG_MIDEA                   CI_LOG_DEBUG
#define LOG_IR                      CI_LOG_INFO
#define LOG_MEDIA                   CI_LOG_DEBUG
#define LOG_OTA                     CI_LOG_INFO
#define LOG_VOICE_UPLOAD            CI_LOG_DEBUG
#define LOG_VOICE_CAPTURE           CI_LOG_DEBUG
#define LOG_SSP_MODULE              CI_LOG_DEBUG
#define LOG_CWSL					CI_LOG_DEBUG
#endif
