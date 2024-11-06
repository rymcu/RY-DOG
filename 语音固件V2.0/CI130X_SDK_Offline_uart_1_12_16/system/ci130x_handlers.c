/**
 * @file ci130x_handlers.c
 * @brief 
 * @version 1.0.0
 * @date 2019-11-21
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "ci130x_core_eclic.h"
#include "ci130x_core_misc.h"
#include "ci_assert.h"
#include "ci_log.h"

typedef struct
{
	uint32_t zer0, ra, sp, gp, tp, t0, t1, t2, fp, s1, a0, a1, a2, a3, a4, a5, a6, a7, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, t3, t4, t5, t6, mstatus, mepc, msubm, mcause;
}exception_registers_t;

__WEAK void	_exit(int __status)
{
    CI_ASSERT(0,"application exit\n");
    for(;;);
}

__WEAK uintptr_t handle_nmi(void)
{
    ci_loginfo(LOG_FAULT_INT, "nmi\n");
    _exit(1);
    return 0;
}

#define NEW_WRITE   1

#if NEW_WRITE   
char ra_string[] = "ra";
char sp_string[] = "sp";
char gp_string[] = "gp";
char tp_string[] = "tp";
char t0_string[] = "t0";
char t1_string[] = "t1";
char t2_string[] = "t2";
char fp_string[] = "fp";
char s1_string[] = "s1";
char a0_string[] = "a0";
char a1_string[] = "a1";
char a2_string[] = "a2";
char a3_string[] = "a3";
char a4_string[] = "a4";
char a5_string[] = "a5";
char a6_string[] = "a6";
char a7_string[] = "a7";
char s2_string[] = "s2";
char s3_string[] = "s3";
char s4_string[] = "s4";
char s5_string[] = "s5";
char s6_string[] = "s6";
char s7_string[] = "s7";
char s8_string[] = "s8";
char s9_string[] = "s9";
char s10_string[] = "s10";
char s11_string[] = "s11";
char t3_string[] = "t3";
char t4_string[] = "t4";
char t5_string[] = "t5";
char t6_string[] = "t6";
char mstatus_string[] = "mstatus";
char mepc_string[] = "mepc";
char msubm_string[] = "msubm";
char mcause_string[] = "mcause";
#endif


extern char _data;
uint32_t get_caller(uint32_t sp, uint32_t * p_caller_pc, uint32_t * p_caller_sp)
{
    *p_caller_pc = NULL;
    *p_caller_sp = NULL;
    uint32_t search_offset = 16;

    for (int i = 0; i < 256;i++)
    {
        uint32_t caller_pc = *(uint32_t*)(sp + search_offset - 4);           //取得假定的函数返回地址
        if (    (caller_pc > 0x1f000000 && caller_pc < 0x1f020000)
            ||  (caller_pc > 0x1ff51000 && caller_pc < &_data))              //返回地址是否为有效地址
        {
            uint32_t caller_ins = *(uint32_t*)(caller_pc - 4);               //取得假定的调用指令
            int finded_ins_bytes = 0;                                        //记录回退搜索到的跳转指令的长度
            if ((caller_ins & 0x03) == 0x03)
            {
                // 32bit instruction
                uint32_t tag = caller_ins & 0x7F;
                if (   tag == 0x6F                          // jal指令
                    || tag == 0x67)                         // jalr   
                {
                    finded_ins_bytes = 4;
                }
            }
            
            if (!finded_ins_bytes)
            {
                // 16bit instrction
                uint16_t caller_ins_16 = *(uint16_t*)(caller_pc - 2);
                uint16_t tag = caller_ins_16 & 0xE003;
                if (   tag == 0x2001                        // c.jal
                    || tag == 0x2001)                       // c.jalr   
                {
                    finded_ins_bytes = 2;
                }
            }
            if (finded_ins_bytes)
            {
                *p_caller_sp = sp + search_offset;
                *p_caller_pc = caller_pc - finded_ins_bytes;
                return 1;
            }
        }
        search_offset += 16;
    }
    return 0;
}

void stack_trace(void * e_sp, void *cur_task_sp)
{
    uint32_t cur_sp = e_sp;
    uint32_t caller_pc, caller_sp;
    while(get_caller(cur_sp, &caller_pc, &caller_sp))
    {
        ci_loginfo(LOG_FAULT_INT, "stack trace:0x%08x\n", caller_pc);
        cur_sp = caller_sp;
    }
    ci_loginfo(LOG_FAULT_INT, "stack trace:end\n", caller_pc);
}

uintptr_t handle_trap(uintptr_t mcause, uintptr_t sp)
{
    if ((mcause&0xFFF) == 0xFFF)
    {
        handle_nmi();
    }
    else
    {
        exception_registers_t *regs = (exception_registers_t *)sp;
        #if NEW_WRITE
        const char* except_reg_str_array[] = {ra_string,sp_string,gp_string,tp_string,t0_string,t1_string,t2_string,fp_string,
                                        s1_string,a0_string,a1_string,a2_string,a3_string,a4_string,a5_string,a6_string,a7_string,
                                        s2_string,s3_string,s4_string,s5_string,s6_string,s7_string,s8_string,s9_string,
                                        s10_string,s11_string,t3_string,t4_string,t5_string,t6_string,mstatus_string,
                                        mepc_string,msubm_string,mcause_string,};
        uint32_t* data = (uint32_t*)((uint32_t)&regs->ra);
        for(int i=0;i<35;i++)
        {
            ci_loginfo(LOG_FAULT_INT, "%s\t = 0x%x\n",except_reg_str_array[i],data[i]);
        }
        ci_loginfo(LOG_FAULT_INT, "mdcause\t = 0x%x\n",read_csr(0x7c9));//mdcause
        ci_loginfo(LOG_FAULT_INT, "mtval\t = 0x%x\n",read_csr(0x343));
        // stack_trace(regs->sp, pxCurrentTCB->pxTopOfStack);
        stack_trace(regs->sp + sizeof(exception_registers_t), NULL);

        #else
        ci_loginfo(LOG_FAULT_INT,"application trap\n");
        ci_loginfo(LOG_FAULT_INT, "----------------------------------\n");
        ci_loginfo(LOG_FAULT_INT, "ra\t = 0x%x\n",regs->ra);
        ci_loginfo(LOG_FAULT_INT, "sp\t = 0x%x\n",regs->sp);
        ci_loginfo(LOG_FAULT_INT, "t0\t = 0x%x\n",regs->t0);
        ci_loginfo(LOG_FAULT_INT, "t1\t = 0x%x\n",regs->t1);
        ci_loginfo(LOG_FAULT_INT, "t2\t = 0x%x\n",regs->t2);
        ci_loginfo(LOG_FAULT_INT, "fp(s0)\t = 0x%x\n",regs->fp);
        ci_loginfo(LOG_FAULT_INT, "s1\t = 0x%x\n",regs->s1);
        ci_loginfo(LOG_FAULT_INT, "a0\t = 0x%x\n",regs->a0);
        ci_loginfo(LOG_FAULT_INT, "a1\t = 0x%x\n",regs->a1);
        ci_loginfo(LOG_FAULT_INT, "a2\t = 0x%x\n",regs->a2);
        ci_loginfo(LOG_FAULT_INT, "a3\t = 0x%x\n",regs->a3);
        ci_loginfo(LOG_FAULT_INT, "a4\t = 0x%x\n",regs->a4);
        ci_loginfo(LOG_FAULT_INT, "a5\t = 0x%x\n",regs->a5);
        ci_loginfo(LOG_FAULT_INT, "a6\t = 0x%x\n",regs->a6);
        ci_loginfo(LOG_FAULT_INT, "a7\t = 0x%x\n",regs->a7);
        ci_loginfo(LOG_FAULT_INT, "s2\t = 0x%x\n",regs->s2);
        ci_loginfo(LOG_FAULT_INT, "s3\t = 0x%x\n",regs->s3);
        ci_loginfo(LOG_FAULT_INT, "s4\t = 0x%x\n",regs->s4);
        ci_loginfo(LOG_FAULT_INT, "s5\t = 0x%x\n",regs->s5);
        ci_loginfo(LOG_FAULT_INT, "s6\t = 0x%x\n",regs->s6);
        ci_loginfo(LOG_FAULT_INT, "s7\t = 0x%x\n",regs->s7);
        ci_loginfo(LOG_FAULT_INT, "s8\t = 0x%x\n",regs->s8);
        ci_loginfo(LOG_FAULT_INT, "s9\t = 0x%x\n",regs->s9);
        ci_loginfo(LOG_FAULT_INT, "s10\t = 0x%x\n",regs->s10);
        ci_loginfo(LOG_FAULT_INT, "s11\t = 0x%x\n",regs->s11);
        ci_loginfo(LOG_FAULT_INT, "t3\t = 0x%x\n",regs->t3);
        ci_loginfo(LOG_FAULT_INT, "t4\t = 0x%x\n",regs->t4);
        ci_loginfo(LOG_FAULT_INT, "t5\t = 0x%x\n",regs->t5);
        ci_loginfo(LOG_FAULT_INT, "t6\t = 0x%x\n",regs->t6);
        ci_loginfo(LOG_FAULT_INT, "mstatus\t = 0x%x\n",regs->mstatus);
        ci_loginfo(LOG_FAULT_INT, "msubm\t = 0x%x\n",regs->msubm);
        ci_loginfo(LOG_FAULT_INT, "mepc\t = 0x%x\n",regs->mepc);
        ci_loginfo(LOG_FAULT_INT, "mcause\t = 0x%x\n",regs->mcause);
        ci_loginfo(LOG_FAULT_INT, "mdcause\t = 0x%x\n",read_csr(0x7c9));//mdcause
        ci_loginfo(LOG_FAULT_INT, "mtval\t = 0x%x\n",read_csr(0x343));
        ci_loginfo(LOG_FAULT_INT, "----------------------------------\n");
        #endif
		_exit(mcause);
    }
    return 0;
}

