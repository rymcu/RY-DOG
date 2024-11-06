/**
 * @file ci130x_it.h
 * @brief 中断服务函数
 * @version 1.0
 * @date 2018-05-21
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef __CI130X_IT_H
#define __CI130X_IT_H


#ifdef __cplusplus
 extern "C" {
#endif 


void MAIL_BOX_IRQHandler(void);
void IWDG_IRQHandler(void);
void TWDG_IRQHandler(void);
void SCU_IRQHandler(void);
void NPU_IRQHandler(void);
void ADC_IRQHandler(void);
void DMA_IRQHandler(void);
void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void TIMER3_IRQHandler(void);
void IIC0_IRQHandler(void);
void PA_IRQHandler(void);
void PB_IRQHandler(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void IIS0_IRQHandler(void);                    
void IIS1_IRQHandler(void);
void IIS2_IRQHandler(void);
void IIS_DMA_IRQHandler(void);
void ALC_IRQHandler(void);
void PDM_IRQHandler(void);
void DTR_IRQHandler(void);
void V11_OK_IRQHandler(void);
void VDT_IRQHandler(void);
void EXT0_IRQHandler(void);
void EXT1_IRQHandler(void);
void AON_TIM_INT0_IRQHandler(void);
void AON_TIM_INT1_IRQHandler(void);
void AON_EFUSE_IRQHandler(void);
void AON_PC_IRQHandler(void);

#ifdef __cplusplus
}
#endif 


#endif /* __CI130X_IT_H */
