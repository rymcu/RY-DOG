/**
 * @file ci130x_core_timer.h
 * @brief 内核计数器定时器驱动
 * @version 1.0
 * @date 2019-11-21
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CI130X_CORE_TIMER_H_
#define __CI130X_CORE_TIMER_H_

#ifdef __ASSEMBLER__
#define _AC(X,Y)        X
#define _AT(T,X)        X
#else
#define _AC(X,Y)        (X##Y)
#define _AT(T,X)        ((T)(X))
#endif /* !__ASSEMBLER__*/

#define _BITUL(x)       (_AC(1,UL) << (x))
#define _BITULL(x)      (_AC(1,ULL) << (x))

#define SOC_TIMER_CTRL_ADDR       _AC(0xE2000000,UL)

#define TIMER_MSIP                0xFFC
#define TIMER_MTIMECTL            0xFF8
#define TIMER_MSFTRST             0xFF0
#define TIMER_MSIP_size           0x4
#define TIMER_MTIMECMP            0x8
#define TIMER_MTIMECMP_size       0x8
#define TIMER_MTIME               0x0
#define TIMER_MTIME_size          0x8

#define TIMER_CTRL_ADDR           SOC_TIMER_CTRL_ADDR
#define TIMER_REG(offset)         _REG32(TIMER_CTRL_ADDR, offset)

#ifndef __ASSEMBLER__
#define SOC_TIMER_FREQ            get_systick_clk()
#define TIMER_FREQ                SOC_TIMER_FREQ
uint32_t get_cpu_freq(void);
uint64_t get_timer_value(void);
void clear_timer_value(void);
void set_msip_int(void);
void clear_msip_int(void);
void core_system_reset(void);
void set_timer_stop(void);
void clear_timer_stop(void);
void set_timer_cmpclrem(void);
void clear_timer_cmpclrem(void);
void set_timer_clksrc(void);
void clear_timer_clksrc(void);
#endif

#endif
