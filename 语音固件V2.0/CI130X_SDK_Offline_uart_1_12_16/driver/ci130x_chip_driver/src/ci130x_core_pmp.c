/**
 * @file ci130x_core_pmp.c
 * @brief 内核PMP驱动
 * @version 1.0
 * @date 2019-11-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ci130x_core_pmp.h"

#include "ci130x_core_eclic.h"

// Configure PMP to make all the address space accesable and executable
void pmp_open_all_space(void)
{
    // Config entry0 addr to all 1s to make the range cover all space
    asm volatile ("li x6, 0xffffffff":::"x6");
    asm volatile ("csrw pmpaddr0, x6":::);
    // Config entry0 cfg to make it NAPOT address mode, and R/W/X okay
    asm volatile ("li x6, 0x7f":::"x6");
    asm volatile ("csrw pmpcfg0, x6":::);
}


void switch_m2u_mode(void)
{
    clear_csr (0x300,0x00001800);
    //printf("\nIn the m2u function, the mstatus is 0x%x\n", read_csr(mstatus));
    //printf("\nIn the m2u function, the mepc is 0x%x\n", read_csr(mepc));
    asm volatile ("la x6, 1f    ":::"x6");
    asm volatile ("csrw mepc, x6":::);
    asm volatile ("mret":::);
    asm volatile ("1:":::);
} 
