/**
 * @file ci130x_cache.h
 * @brief  cache驱动文件
 * @version 0.1
 * @date 2019-04-03
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#ifndef _CI130X_CACHE_H_
#define _CI130X_CACHE_H_

#include "ci130x_system.h"

/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_cache ci130x_cache
 * @brief CI130X芯片CACHE驱动
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define ICACHE_TCM_S (0x1ffa8000)/*!< ICACHE TCM起始地址 */
#define ICACHE_TCM_E (0x1ffaffff)/*!< ICACHE TCM终止地址 */
#define ICACHE_TCM_A (0x1fbb0000)/*!< ICACHE TCM真实地址*/

/**
 * @brief CACHE控制器定义
 */
typedef enum
{
    ICACHE = HAL_ICACHE_BASE,/*!< ICACHE */
    SCACHE = HAL_SCACHE_BASE,/*!< SCACHE */
}cache_base_t;

void cache_enable_auto(cache_base_t base,unsigned int start,unsigned int end);
void cache_disable_auto(cache_base_t base);
void get_hit_miss(cache_base_t base,unsigned int* hit,unsigned int* miss);
void cache_alias_mode(cache_base_t base,unsigned int start,unsigned int end,
                    unsigned int alias);
void i_cache_tcm_enable(unsigned int flash_user_offset);
void s_cache_tcm_enable(void);
void show_cache(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
