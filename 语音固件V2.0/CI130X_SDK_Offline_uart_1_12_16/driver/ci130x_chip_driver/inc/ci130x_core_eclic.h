/**
 * @file ci130x_core_eclic.h
 * @brief 内核中断控制器驱动
 * @version 1.0
 * @date 2019-11-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CI130X_CORE_ECLIC_H_
#define __CI130X_CORE_ECLIC_H_


#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
    asm volatile ("csrw " #reg ", %0" :: "i"(val)); \
  else \
    asm volatile ("csrw " #reg ", %0" :: "r"(val)); })

#define swap_csr(reg, val) ({ unsigned long __tmp; \
  if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
    asm volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "i"(val)); \
  else \
    asm volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "r"(val)); \
  __tmp; })

#define set_csr(reg, bit) ({ unsigned long __tmp; \
  if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
    asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit)); \
  else \
    asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit)); \
  __tmp; })

#define clear_csr(reg, bit) ({ unsigned long __tmp; \
  if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
    asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit)); \
  else \
    asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit)); \
  __tmp; })


#ifdef __ASSEMBLER__
#define _AC(X,Y)        X
#define _AT(T,X)        X
#else
#define _AC(X,Y)        (X##Y)
#define _AT(T,X)        ((T)(X))
#endif /* !__ASSEMBLER__*/

#define _BITUL(x)       (_AC(1,UL) << (x))
#define _BITULL(x)      (_AC(1,ULL) << (x))

#define SOC_ECLIC_CTRL_ADDR      _AC(0xEC000000,UL)
#define SOC_ECLIC_NUM_INTERRUPTS 32


// Need to know the following info from the soc.h, so include soc.h here
//   SOC_ECLIC_CTRL_ADDR      : what is the base address of ECLIC in this SoC
//   SOC_ECLIC_NUM_INTERRUPTS : how much of irq configured in total for the ECLIC in this SoC

#define CLICINTCTLBITS  3

//ECLIC memory map
//   Offset
//  0x0000       1B          RW        cliccfg
#define ECLIC_CFG_OFFSET               0x0
//  0x0004       4B          R         clicinfo   
#define ECLIC_INFO_OFFSET              0x4
//  0x000B       1B          RW        mintthresh 
#define ECLIC_MTH_OFFSET               0xB
//
//  0x1000+4*i   1B/input    RW        clicintip[i]
#define ECLIC_INT_IP_OFFSET            _AC(0x1000,UL)
//  0x1001+4*i   1B/input    RW        clicintie[i]
#define ECLIC_INT_IE_OFFSET            _AC(0x1001,UL)
//  0x1002+4*i   1B/input    RW        clicintattr[i]
#define ECLIC_INT_ATTR_OFFSET          _AC(0x1002,UL)

#define ECLIC_INT_ATTR_SHV              0x01
#define ECLIC_INT_ATTR_TRIG_LEVEL       0x00
#define ECLIC_INT_ATTR_TRIG_EDGE        0x02
#define ECLIC_INT_ATTR_TRIG_POS         0x00
#define ECLIC_INT_ATTR_TRIG_NEG         0x04

//  0x1003+4*i   1B/input    RW        clicintctl[i]
#define ECLIC_INT_CTRL_OFFSET          _AC(0x1003,UL)
//
//  ...
//
#define ECLIC_ADDR_BASE           SOC_ECLIC_CTRL_ADDR

#define ECLIC_NUM_INTERRUPTS      (SOC_ECLIC_NUM_INTERRUPTS + 19)

#define ECLIC_CFG_NLBITS_MASK     _AC(0x1E,UL)
#define ECLIC_CFG_NLBITS_LSB      (1u)

#define MTIME_HANDLER             eclic_mtip_handler
#define MSIP_HANDLER              eclic_msip_handler 


/* constants definitions */
#define ECLIC_PRIGROUP_LEVEL0_PRIO3        0    /*!< 0 bits for level 4 bits for priority */
#define ECLIC_PRIGROUP_LEVEL1_PRIO2        1    /*!< 1 bits for level 3 bits for priority */
#define ECLIC_PRIGROUP_LEVEL2_PRIO1        2    /*!< 2 bits for level 2 bits for priority */
#define ECLIC_PRIGROUP_LEVEL3_PRIO0        3    /*!< 3 bits for level 1 bits for priority */

#ifndef __ASSEMBLER__

#include <stdint.h>

/* function declarations */
void eclic_init(uint32_t num_irq);
void eclic_mode_enable(void);
void eclic_set_mth(uint8_t mth);
uint8_t eclic_get_mth(void);
void eclic_global_interrupt_enable(void);
void eclic_global_interrupt_disable(void);
void eclic_priority_group_set(uint32_t prigroup);
void eclic_irq_enable(uint32_t source);
void eclic_irq_set_priority(uint32_t source, uint8_t lvl_abs, uint8_t priority);
void eclic_irq_disable(uint32_t source);
void eclic_set_vmode(uint32_t source);
void eclic_set_nonvmode(uint32_t source);
void __eclic_irq_set_vector(uint32_t source, int32_t addr);
uint8_t eclic_get_pending(uint32_t source);
void eclic_set_pending(uint32_t source);
void eclic_clear_pending(uint32_t source);
void eclic_set_level_trig(uint32_t source);
void eclic_set_posedge_trig(uint32_t source);
void eclic_set_negedge_trig(uint32_t source);

#endif

#endif
