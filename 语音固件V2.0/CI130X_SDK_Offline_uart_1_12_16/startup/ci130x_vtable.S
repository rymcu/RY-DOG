/**
 * @file ci130x_vtable.S
 * @brief ci130x中断向量表
 * @version 1.0.0
 * @date 2019-11-21
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

	.section .vtable

    .weak  eclic_msip_handler
    .weak  eclic_mtip_handler

    .globl vector_base
vector_base:
    j _start                   //0
    .align 2
    .word  0                   //1
    .word  0                   //2
    .word  eclic_msip_handler  //3
    .word  0                   //4
    .word  0                   //5
    .word  0                   //6
    .word  eclic_mtip_handler  //7
    .word  0                   //8
    .word  0                   //9
    .word  0                   //10
    .word  0                   //11
    .word  0                   //12
    .word  0                   //13
    .word  0                   //14
    .word  0                   //15
    .word  0                   //16
    .word  0                   //17
    .word  0                   //18
    .word  TWDG_IRQHandler        //19
    .word  SCU_IRQHandler         //20
    .word  NPU_IRQHandler          //21
    .word  ADC_IRQHandler          //22
    .word  DMA_IRQHandler          //23
    .word  TIMER0_IRQHandler      //24
    .word  TIMER1_IRQHandler      //25
    .word  TIMER2_IRQHandler      //26
    .word  TIMER3_IRQHandler      //27
    .word  IIC0_IRQHandler        //28
    .word  PA_IRQHandler       //29
    .word  PB_IRQHandler       //30
    .word  UART0_IRQHandler       //31
    .word  UART1_IRQHandler       //32
    .word  UART2_IRQHandler       //33
    .word  IIS0_IRQHandler        //34
    .word  IIS1_IRQHandler        //35
    .word  IIS2_IRQHandler        //36
    .word  IIS_DMA_IRQHandler     //37
    .word  ALC_IRQHandler         //38
    .word  PDM_IRQHandler         //39
    .word  DTR_IRQHandler         //40
    .word  V11_OK_IRQHandler      //41
    .word  VDT_IRQHandler         //42
    .word  EXT0_IRQHandler        //43
    .word  EXT1_IRQHandler        //44
    .word  IWDG_IRQHandler                //45
    .word  AON_TIM_INT0_IRQHandler        //46
    .word  AON_TIM_INT1_IRQHandler        //47
    .word  AON_EFUSE_IRQHandler           //48
    .word  AON_PC_IRQHandler           //49
    .word  MAIL_BOX_IRQHandler //50
    .fill  13, 4, 0x0          //填充向量表到下一个对齐的地址以存放load_flag
