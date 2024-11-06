/**
 * @file ci130x_core_misc.h
 * @brief 内核相关其他驱动
 * @version 1.0
 * @date 2019-11-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CI130X_CORE_MISC_H_
#define __CI130X_CORE_MISC_H_

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "ci130x_core_eclic.h"


void disable_mcycle_minstret(void);
void enable_mcycle_minstret(void);
uint64_t get_instret_value(void);
uint64_t get_cycle_value(void);
int32_t check_curr_trap(void);

#define __SEV           core_send_event
#define __WFI           core_wfi
#define __WFE           core_wfe
#define __NOP           core_nop
#define __no_init       __attribute__((section(".no_init")))
#define __interrupt     __attribute__((interrupt))
#define __WEAK          __attribute__((weak))

/**
 * @brief WFI
 *
 */
static inline void core_wfi(void)
{
    __asm volatile("wfi");
}


/**
 * @brief WFE
 *
 */
static inline void core_wfe(void)
{
    extern void core_wfe_asm(void);
    core_wfe_asm();
}


/**
 * @brief send event(SEV)
 *
 */
static inline void core_send_event(void)
{
    set_csr(0x812, 0x1);
}


/**
 * @brief NOP
 *
 */
static inline void core_nop(void)
{
    __asm volatile("nop");
}


#endif
