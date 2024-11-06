/**
 * @file ci130x_core_eclic.c
 * @brief 内核中断控制器驱动
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
#include "ci130x_core_eclic.h"


static void eclic_enable_interrupt(uint32_t source)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IE_OFFSET + source * 4) = 1;
}


static void eclic_disable_interrupt(uint32_t source)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IE_OFFSET + source * 4) = 0;
}


static void eclic_set_intctrl(uint32_t source, uint8_t intctrl)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_CTRL_OFFSET + source * 4) = intctrl;
}


static uint8_t eclic_get_intctrl(uint32_t source)
{
    return *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_CTRL_OFFSET + source * 4);
}


static void eclic_set_intattr(uint32_t source, uint8_t intattr)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_ATTR_OFFSET + source * 4) = intattr;
}


static uint8_t eclic_get_intattr(uint32_t source)
{
    return *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_ATTR_OFFSET + source * 4);
}


static void eclic_set_cliccfg(uint8_t cliccfg)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_CFG_OFFSET) = cliccfg;
}


static uint8_t eclic_get_cliccfg(void)
{
    return *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_CFG_OFFSET);
}


static void eclic_set_nlbits(uint8_t nlbits)
{
    //shift nlbits to correct position
    uint8_t nlbits_shifted = nlbits << ECLIC_CFG_NLBITS_LSB;

    //read the current cliccfg
    uint8_t old_cliccfg = eclic_get_cliccfg();
    uint8_t new_cliccfg = (old_cliccfg & (~ECLIC_CFG_NLBITS_MASK)) | (ECLIC_CFG_NLBITS_MASK & nlbits_shifted);

    eclic_set_cliccfg(new_cliccfg);
}


//get nlbits
static uint8_t eclic_get_nlbits(void)
{
    //extract nlbits
    uint8_t nlbits = eclic_get_cliccfg();
    nlbits = (nlbits & ECLIC_CFG_NLBITS_MASK) >> ECLIC_CFG_NLBITS_LSB;
    return nlbits;
}


//sets an interrupt level based encoding of nlbits and CLICINTCTLBITS
void eclic_set_irq_lvl(uint32_t source, uint8_t lvl)
{
    //extract nlbits
    uint8_t nlbits = eclic_get_nlbits();
    if (nlbits > CLICINTCTLBITS)
    {
        nlbits = CLICINTCTLBITS;
    }

    //shift lvl right to mask off unused bits
    lvl = lvl >> (8 - nlbits);
    //shift lvl into correct bit position
    lvl = lvl << (8 - nlbits);

    //write to clicintctrl
    uint8_t current_intctrl = eclic_get_intctrl(source);
    //shift intctrl left to mask off unused bits
    current_intctrl = current_intctrl << nlbits;
    //shift intctrl into correct bit position
    current_intctrl = current_intctrl >> nlbits;

    eclic_set_intctrl(source, (current_intctrl | lvl));
}


//gets an interrupt level based encoding of nlbits
uint8_t eclic_get_irq_lvl(uint32_t source)
{
    //extract nlbits
    uint8_t nlbits = eclic_get_nlbits();
    if (nlbits > CLICINTCTLBITS)
    {
        nlbits = CLICINTCTLBITS;
    }

    uint8_t intctrl = eclic_get_intctrl(source);

    //shift intctrl
    intctrl = intctrl >> (8 - nlbits);
    //shift intctrl
    uint8_t lvl = intctrl << (8 - nlbits);

    return lvl;
}


static void eclic_set_irq_lvl_abs(uint32_t source, uint8_t lvl_abs)
{
    //extract nlbits
    uint8_t nlbits = eclic_get_nlbits();
    if (nlbits > CLICINTCTLBITS)
    {
        nlbits = CLICINTCTLBITS;
    }

    //shift lvl_abs into correct bit position
    uint8_t lvl = lvl_abs << (8 - nlbits);

    //write to clicintctrl
    uint8_t current_intctrl = eclic_get_intctrl(source);
    //shift intctrl left to mask off unused bits
    current_intctrl = current_intctrl << nlbits;
    //shift intctrl into correct bit position
    current_intctrl = current_intctrl >> nlbits;

    eclic_set_intctrl(source, (current_intctrl | lvl));
}


uint8_t eclic_get_irq_lvl_abs(uint32_t source)
{
    //extract nlbits
    uint8_t nlbits = eclic_get_nlbits();
    if (nlbits > CLICINTCTLBITS)
    {
        nlbits = CLICINTCTLBITS;
    }

    uint8_t intctrl = eclic_get_intctrl(source);

    //shift intctrl
    intctrl = intctrl >> (8 - nlbits);
    //shift intctrl
    uint8_t lvl_abs = intctrl;

    return lvl_abs;
}


//sets an interrupt priority based encoding of nlbits and ECLICINTCTLBITS
static uint8_t eclic_set_irq_priority(uint32_t source, uint8_t priority)
{
	//extract nlbits
	uint8_t nlbits = eclic_get_nlbits();
	if (nlbits >= CLICINTCTLBITS) {
		nlbits = CLICINTCTLBITS;
		return 0;
	}

	//shift priority into correct bit position
	priority = priority << (8 - CLICINTCTLBITS);

	//write to eclicintctrl
	uint8_t current_intctrl = eclic_get_intctrl(source);
	//shift intctrl right to mask off unused bits
	current_intctrl = current_intctrl >> (8-nlbits);
	//shift intctrl into correct bit position
	current_intctrl = current_intctrl << (8-nlbits);

	eclic_set_intctrl(source, (current_intctrl | priority));

	return priority;
}


/**
 * @brief 初始化ECLIC中断控制器
 * 
 * @param num_irq 中断数
 */
void eclic_init(uint32_t num_irq)
{
    typedef volatile uint32_t vuint32_t;

    //clear cfg register
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_CFG_OFFSET) = 0;

    //clear minthresh register
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_MTH_OFFSET) = 0;

    //clear all IP/IE/ATTR/CTRL bits for all interrupt sources
    vuint32_t *ptr;

    vuint32_t *base = (vuint32_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IP_OFFSET);
    vuint32_t *upper = (vuint32_t *)(base + num_irq * 4);

    for (ptr = base; ptr < upper; ptr = ptr + 1)
    {
        *ptr = 0;
    }
}


/**
 * @brief 使能ECLIC中断模式
 * 
 */
void eclic_mode_enable(void)
{
    uint32_t mtvec_value = read_csr(mtvec);
    mtvec_value = mtvec_value & 0xFFFFFFC0;
    mtvec_value = mtvec_value | 0x00000003;
    write_csr(mtvec, mtvec_value);
}


/**
 * @brief 设置中断优先级阈值(类似ARM CORTEX-M 的base_pri)
 * 
 * @param mth 优先级阈值
 */
void eclic_set_mth(uint8_t mth)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_MTH_OFFSET) = mth;
}


/**
 * @brief 获取当前中断优先级阈值(类似ARM CORTEX-M 的base_pri)
 * 
 * @return uint8_t 优先级阈值
 */
uint8_t eclic_get_mth(void)
{
    return *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_MTH_OFFSET);
}


/**
 * @brief 系统全局中断使能
 * @note 涉及到CSR寄存器体系相关，谨慎使用
 */
void eclic_global_interrupt_enable(void)
{
    /* set machine interrupt enable bit */
    set_csr(0x300, 0x00000008);
}


/**
 * @brief 系统全局中断关闭
 * @note 涉及到CSR寄存器体系相关，谨慎使用
 */
void eclic_global_interrupt_disable(void)
{
    /* clear machine interrupt enable bit */
    clear_csr(0x300, 0x00000008);
}


/**
 * @brief 设置优先级组
 * 
 * @param prigroup specify the priority group
 *   @arg ECLIC_PRIGROUP_LEVEL0_PRIO3
 *   @arg ECLIC_PRIGROUP_LEVEL1_PRIO2
 *   @arg ECLIC_PRIGROUP_LEVEL2_PRIO1
 *   @arg ECLIC_PRIGROUP_LEVEL3_PRIO0
 */
void eclic_priority_group_set(uint32_t prigroup)
{
    eclic_set_nlbits(prigroup);
}


/**
 * @brief 使能中断
 * 
 * @param source 中断号
 */
void eclic_irq_enable(uint32_t source)
{
    eclic_enable_interrupt(source);
}


/**
 * @brief 设置中断优先级
 * 
 * @param source 中断号
 * @param lvl_abs 抢占优先级
 * @param priority 响应优先级
 */
void eclic_irq_set_priority(uint32_t source, uint8_t lvl_abs, uint8_t priority)
{
    eclic_set_irq_lvl_abs(source, lvl_abs);
    eclic_set_irq_priority(source, priority);
}


/**
 * @brief 关闭中断
 * 
 * @param source 中断号
 */
void eclic_irq_disable(uint32_t source)
{
    eclic_disable_interrupt(source);
}


/**
 * @brief 设置中断为向量模式
 * 
 * @param source 中断号
 */
void eclic_set_vmode(uint32_t source)
{
    //read the current attr
    uint8_t old_intattr = eclic_get_intattr(source);
    // Keep other bits unchanged and only set the LSB bit
    uint8_t new_intattr = (old_intattr | ECLIC_INT_ATTR_SHV);

    eclic_set_intattr(source, new_intattr);
}


/**
 * @brief 设置中断为非向量模式
 * 
 * @param source 中断号
 */
void eclic_set_nonvmode(uint32_t source)
{
    //read the current attr
    uint8_t old_intattr = eclic_get_intattr(source);
    // Keep other bits unchanged and only clear the LSB bit
    uint8_t new_intattr = (old_intattr & (~ECLIC_INT_ATTR_SHV));

    eclic_set_intattr(source, new_intattr);
}


/**
 * @brief 设置中断服务函数地址
 * @note 注意只能在中断向量表位于ram的系统内使用该函数
 * 
 * @param source 中断号
 * @param addr 中断服务函数地址
 */
void __eclic_irq_set_vector(uint32_t source, int32_t addr)
{
    uint32_t vector_addr = read_csr(0x307);//mtvt在有些编译器上不能识别
    *(volatile int32_t *)(vector_addr+source*4) = addr;
}

int32_t __eclic_irq_get_vector(uint32_t source)
{
    uint32_t vector_addr = read_csr(0x307);
    return *(volatile int32_t *)(vector_addr+source*4);
}

/**
 * @brief 获取中断挂起
 *
 * @param source 中断号
 * @return 1:中断挂起 0:中断未被挂起
 */
uint8_t eclic_get_pending(uint32_t source)
{
	return (0x1&(*(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IP_OFFSET + source * 4)));
}


/**
 * @brief 设置中断挂起
 *
 * @param source 中断号
 */
void eclic_set_pending(uint32_t source)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IP_OFFSET + source * 4) = 1;
}


/**
 * @brief 清除中断挂起
 * 
 * @param source 中断号
 */
void eclic_clear_pending(uint32_t source)
{
    *(volatile uint8_t *)(ECLIC_ADDR_BASE + ECLIC_INT_IP_OFFSET + source * 4) = 0;
}


/**
 * @brief 设置中断电平触发
 * 
 * @param source 中断号
 */
void eclic_set_level_trig(uint32_t source)
{
    //read the current attr
    uint8_t old_intattr = eclic_get_intattr(source);
    // Keep other bits unchanged and only clear the bit 1
    uint8_t new_intattr = (old_intattr & (~0x2));

    eclic_set_intattr(source, new_intattr);
}


/**
 * @brief 设置中断上升沿触发
 * 
 * @param source 中断号
 */
void eclic_set_posedge_trig(uint32_t source)
{
    //read the current attr
    uint8_t old_intattr = eclic_get_intattr(source);
    // Keep other bits unchanged and only set the bit 1
    uint8_t new_intattr = (old_intattr | 0x2);
    // Keep other bits unchanged and only clear the bit 2
    new_intattr = (new_intattr & (~0x4));

    eclic_set_intattr(source, new_intattr);
}


/**
 * @brief 设置中断下降沿触发
 * 
 * @param source 中断号
 */
void eclic_set_negedge_trig(uint32_t source)
{
    //read the current attr
    uint8_t old_intattr = eclic_get_intattr(source);
    // Keep other bits unchanged and only set the bit 1
    uint8_t new_intattr = (old_intattr | 0x2);
    // Keep other bits unchanged and only set the bit 2
    new_intattr = (new_intattr | 0x4);

    eclic_set_intattr(source, new_intattr);
}
