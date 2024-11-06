/**
 * @file ci130x_startup.S
 * @brief ci130x启动文件
 * @version 1.0.0
 * @date 2019-11-21
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include "RISCV/riscv_encoding.h"

	.section .init

	.globl _start
	.type _start,@function
	.weak _start
_start:
	csrc CSR_MSTATUS, MSTATUS_MIE

	/* Jump to logical address first to ensure correct operation of RAM region  */
	/* 这个操作很重要，如果不进行启动地址转换可能导致gp赋值错误引起相对寻址出错 */
    la		a0,	_start             //a0 = _start
    li		a1,	0x1ff51             //a1 = 0x1ff51
	slli	a1,	a1, 12             //a1 = 0x1ff51000 raw起始地址
    bleu	a1, a0, _virtual_start //if( a1 <= a0 ) JUMP _virtual_start
    la		a0,	_virtual_start     //a0 = _virtual_start
    add		a0, a0, a1             //a0 = a0+a1
	jr      a0                     //JUMP a0

	.globl _virtual_start
	.type _virtual_start,@function
_virtual_start:
    /* Set the the NMI base to share with mtvec by setting CSR_MMISC_CTL */
    li t0, (0x1 << 9);
    csrs CSR_MMISC_CTL, t0

	/* Intial the mtvt*/
    la t0, vector_base
    csrw CSR_MTVT, t0

	/* Intial the mtvt2 and enable it*/
    la t0, irq_entry
    csrw CSR_MTVT2, t0
    csrs CSR_MTVT2, 0x1

    /* Intial the CSR MTVEC for the Trap ane NMI base addr*/
    la t0, trap_entry
    csrw CSR_MTVEC, t0

	/* OS启动前配置中断栈为FHEAP的end地址 */
	la t0, __befor_os_int_sp
	csrw CSR_MSCRATCH, t0

#ifdef __riscv_flen
	/* Enable FPU */
	li t0, (0x1 << 13)   //配置fs状态为Inital
	csrs mstatus, t0 
	csrw fcsr, x0        //复位浮点状态位
#endif

	/* Enable x */
	li t0, (0x1 << 15)   //配置  状态为Inital
	csrs mstatus, t0

    .option push
    .option norelax
	la gp, __global_pointer$
    .option pop
	la sp, _sp

	/* Bob: Load code section from flash to ILM */
	la a0, _ilm_lma
	la a1, _ilm
    beq a0, a1, 2f  /*If the ILM phy-address same as the logic-address, then quit*/
	la a2, _eilm
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:

	/* Load data section */
	la a0, _data_lma
	la a1, _data
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:

	/* Clear bss section */
	la a0, __bss_start
	la a1, __bss_end
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:
	/*enable mcycle_minstret*/
    csrci CSR_MCOUNTINHIBIT, 0x5
	/* Call global constructors */
	la a0, __libc_fini_array
	call atexit
	call __libc_init_array

	/* argc = argv = 0 */
	li a0, 0
	li a1, 0
	call main
	tail exit

1:
	j 1b

	.global disable_mcycle_minstret
disable_mcycle_minstret:
    csrsi CSR_MCOUNTINHIBIT, 0x5
	ret

	.global enable_mcycle_minstret
enable_mcycle_minstret:
    csrci CSR_MCOUNTINHIBIT, 0x5
	ret

	.global core_wfe_asm
core_wfe_asm:
    csrc CSR_MSTATUS, MSTATUS_MIE
    csrs CSR_WFE, 0x1
    wfi
    csrc CSR_WFE, 0x1
    csrs CSR_MSTATUS, MSTATUS_MIE
	ret

/*
 *********************************************************************************
 *   CI Loader Flag
 *********************************************************************************
 */
	.section .load_addr,"a",%progbits
	.type load_addr, %object
	.size load_addr, .-load_addr
load_addr:
	.byte  'L'
	.byte  'O'
	.byte  'A'
	.byte  'D'
	.byte  'V'
	.byte  'E'
	.byte  'R'
	.byte  0x2
	.word  LOAD_TYPE
	.word  START_ADDR
	.word  vector_base
	.word  ITCM_EN
	.word  STCM_EN
	.word  ICACHE_EN
	.word  ICACHE_START_ADDR
	.word  ICACHE_END_ADDR
	.word  ICACHE_ALIAS_ADDR
	.word  SCACHE_EN
	.word  SCACHE_START_ADDR
	.word  SCACHE_END_ADDR
	.word  SCACHE_ALIAS_ADDR
