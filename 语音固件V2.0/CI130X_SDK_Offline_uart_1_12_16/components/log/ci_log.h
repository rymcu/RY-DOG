/**
 * @file ci_log.h
 * @brief sdklog输出组件
 * 这个文件用于定义sdk日志打印组件，log打印等级 参考Andriod Logcat 提供了多个打印函数，
 * 开发者可以自行定义模块名称，统一模块打印等级宏
 * 
 * @version 1.0
 * @date 2019-01-09
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _CI_LOG_H_
#define _CI_LOG_H_

#include "ci_log_config.h"
#include "ci_assert.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* 
 * 本模块使用大量多次##重载符，MISR规则19.12中被视为警告，因为该重载符在标准多次使用时
 * 在标准c中宏展开顺序是UB行为，目前测试在IAR8.3和GCC8.2.X中展开处理结果正确，使用c-stat
 * 或pclint请注意。
 */


extern int _printf(char* format,...);
extern void uart_log_int_handler(void);
extern void uart_log_int_mode_init(void);

#if USE_STD_PRINTF
#define LOG_DEBUG(fmt,...) printf(fmt,##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt,...) _printf(fmt,##__VA_ARGS__)
#endif

#if CONFIG_CI_LOG_EN
/*need open SecureCRT ansi color*/
#define CI_LOG(debug, comlevel, message, args...)                                          \
    do {                                                                                   \
        if((debug)<=(comlevel)) {                                                          \
            if((debug) & (CI_LOG_ASSERT))LOG_DEBUG("\033[35m"message"\033[0m", ## args);   \
            if((debug) & (CI_LOG_ERROR))LOG_DEBUG("\033[31m"message"\033[0m", ## args);    \
            if((debug) & (CI_LOG_WARN))LOG_DEBUG("\033[33m"message"\033[0m", ## args);     \
            if((debug) & (CI_LOG_INFO))LOG_DEBUG(message, ## args);                        \
            if((debug) & (CI_LOG_DEBUG))LOG_DEBUG("\033[36m"message"\033[0m", ## args);    \
            if((debug) & (CI_LOG_VERBOSE))LOG_DEBUG("\033[32m"message"\033[0m", ## args);  \
        }                                                                                  \
    } while(0)
#else
#define CI_LOG(debug, message, args...)   do{}while(0)           
#endif


#if CONFIG_SMT_PRINTF_EN
    #if USE_STD_PRINTF
    #define _mprintf(fmt,...) printf(fmt,##__VA_ARGS__)
    #else
	#define _mprintf(fmt,...) _printf(fmt,##__VA_ARGS__)
    #endif
#else
	#define _mprintf(fmt,...) do{}while(0)
#endif

/*--------------------------------------------------------------*/
/* Additional user defined functions                            */
/*--------------------------------------------------------------*/
/**
 * @ingroup log
 * @{
 */
/** @brief 日志打印--详细 */
#define ci_logverbose(comlevel, message, args...) CI_LOG(CI_LOG_VERBOSE, comlevel, message, ## args)
/** @brief 日志打印--调试 */
#define ci_logdebug(comlevel, message, args...)   CI_LOG(CI_LOG_DEBUG, comlevel, message, ## args)
/** @brief 日志打印--信息 */
#define ci_loginfo(comlevel, message, args...)    CI_LOG(CI_LOG_INFO, comlevel, message, ## args)
/** @brief 日志打印--警告 */
#define ci_logwarn(comlevel, message, args...)    CI_LOG(CI_LOG_WARN, comlevel, message, ## args)
/** @brief 日志打印--错误 */
#define ci_logerr(comlevel, message, args...)     CI_LOG(CI_LOG_ERROR, comlevel, message, ## args)
/** @brief 日志打印--断言 */
#define ci_logassert(comlevel, message, args...)  CI_LOG(CI_LOG_ASSERT, comlevel, message, ## args)
/** @} */       

#define mprintf(fmt, args...)                     _mprintf(fmt, ## args)

// 日志模块初始化
void ci_log_init(void);

#ifdef __cplusplus
}
#endif

#endif
