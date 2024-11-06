/**
 * @file ci130x_adc.c
 * @brief CI130X芯片ADC模块的驱动程序
 * @version 0.1
 * @date 2019-05-17
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */



#include "ci130x_adc.h"

/*ADC寄存器定义*/
typedef struct
{
    volatile unsigned int ADCCTRL;//0x00
    volatile unsigned int ADCINTMASK;
    volatile unsigned int ADCINTFLG;
    volatile unsigned int ADCINTCLR;
    volatile unsigned int ADCSOFTSOC;//0x10
    volatile unsigned int ADCSOCCTRL;
    volatile unsigned int ADCRESULT[8];
    volatile unsigned int CHxPERIOD[4];
    volatile unsigned int CH0MINVALUE;
    volatile unsigned int CH0MAXVALUE;
    volatile unsigned int CH1MINVALUE;
    volatile unsigned int CH1MAXVALUE;
    volatile unsigned int CH2MINVALUE;
    volatile unsigned int CH2MAXVALUE;
    volatile unsigned int CH3MINVALUE;
    volatile unsigned int CH3MAXVALUE;
    volatile unsigned int ADCSTAT;
    volatile unsigned int ADCCTRLa;//0x6c
    volatile unsigned int ADCRSTN;
    volatile unsigned int ADCOFFSET;
    volatile unsigned int ADCLOADEN;
    volatile unsigned int ADCPWRRDY;
    volatile unsigned int CH4PERIOD;
    volatile unsigned int CH5PERIOD;
    volatile unsigned int CH4MINVALUE;
    volatile unsigned int CH4MAXVALUE;
    volatile unsigned int CH5MINVALUE;
    volatile unsigned int CH5MAXVALUE;
}ADC_TypeDef;


volatile uint8_t adc_cha0_int_flag = 0;
volatile uint8_t adc_cha1_int_flag = 0;
volatile uint8_t adc_cha2_int_flag = 0;
volatile uint8_t adc_cha3_int_flag = 0;
volatile uint8_t adc_cha4_int_flag = 0;
volatile uint8_t adc_cha5_int_flag = 0;

void adc_clear_flag(void)
{
	adc_cha0_int_flag = 0;
	adc_cha1_int_flag = 0;
	adc_cha2_int_flag = 0;
	adc_cha3_int_flag = 0;
	adc_cha4_int_flag = 0;
	adc_cha5_int_flag = 0;
}


void adc_wait_int(adc_channelx_t cha)
{
    switch(cha)
    {
        case ADC_CHANNEL_0:
            while(!adc_cha0_int_flag);
            adc_cha0_int_flag = 0;
            break;
        case ADC_CHANNEL_1:
            while(!adc_cha1_int_flag);
            adc_cha1_int_flag = 0;
            break;
        case ADC_CHANNEL_2:
            while(!adc_cha2_int_flag);
            adc_cha2_int_flag = 0;
            break;
        case ADC_CHANNEL_3:
            while(!adc_cha3_int_flag);
            adc_cha3_int_flag = 0;
            break;
        case ADC_CHANNEL_4:
			while(!adc_cha4_int_flag);
			adc_cha4_int_flag = 0;
			break;
		case ADC_CHANNEL_5:
			while(!adc_cha5_int_flag);
			adc_cha5_int_flag = 0;
			break;
        default:
            break;
    }
}
void ADC_irqhandle(void){
	    uint32_t status = ADC->ADCINTFLG;
	    if(status & (0x1 << ADC_CHANNEL_0))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_0);
	        adc_cha0_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_0);
	    }
	    if(status & (0x1 << ADC_CHANNEL_1))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_1);
	        adc_cha1_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_1);
	    }
	    if(status & (0x1 << ADC_CHANNEL_2))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_2);
	        adc_cha2_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_2);
	    }
	    if(status & (0x1 << ADC_CHANNEL_3))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_3);
	        adc_cha3_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_3);
	    }
	    if(status & (0x1 << ADC_CHANNEL_4))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_4);
	        adc_cha4_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_4);
	    }
	    if(status & (0x1 << ADC_CHANNEL_5))
	    {
	    	ADC->ADCINTCLR |= (0x1 << ADC_CHANNEL_5);
	        adc_cha5_int_flag = 1;
            adc_int_clear(ADC_CHANNEL_5);
	    }
}
/**
 * @brief ADC电路电源使能
 *
 * @param cmd 打开或者关闭
 */
void adc_power_ctrl(FunctionalState cmd)
{
    if(cmd != ENABLE)
    {
        ADC->ADCCTRL &= ~(1<<0);
	}
    else
    {
		ADC->ADCCTRL |= (1<<0);
	}
}

/**
 * @brief ADC连续转换使能
 *
 * @param cmd 打开或关闭
 */
void adc_continuons_convert(FunctionalState cmd)
{
    if(cmd != ENABLE)
    {
        ADC->ADCCTRL &= ~(1 << 1);
	}
    else
    {
		ADC->ADCCTRL |= (1 << 1);
	}
}


/**
 * @brief ADC校准使能
 *
 * @param cmd 打开或关闭
 */
void adc_calibrate(FunctionalState cmd)
{
    if(cmd == ENABLE)
    {
		ADC->ADCCTRL |= (1 << 3);
	}
    else
    {
		ADC->ADCCTRL &= ~(1 << 3);
	}
}


/**
 * @brief ADC通道周期监测使能(只支持ADC_CHANNEL_0-ADC_CHANNEL_3)
 *
 * @param channel adc通道选择
 * @param cmd 使能或关闭
 */
void adc_period_monitor(adc_channelx_t channel,FunctionalState cmd)
{
    if(cmd != DISABLE)
    {
        if(ADC_CHANNEL_4 > channel)
        {
            ADC->ADCCTRL |= (1 << (channel + 4));
        }
        else
        {
            ADC->ADCCTRLa |= (1 << channel);
        }
    }
    else
    {
        if(ADC_CHANNEL_4 > channel)
        {
            ADC->ADCCTRL &= ~(1 << (channel + 4));
        }
        else
        {
            ADC->ADCCTRLa &= ~(1 << channel);
        }
    }
    ADC->ADCLOADEN |= (0x1 << 0);
}


/**
 * @brief ADC周期监测使能(只能工作于单次采样模式下）
 *
 * @param cmd 打开或关闭
 */
void adc_period_enable(FunctionalState cmd)
{
    if(cmd != DISABLE)
    {
        ADC->ADCCTRL |= (1 << 8);
	}
    else
    {
		ADC->ADCCTRL &= ~(1 << 8);
	}
}


/**
 * @brief ADC中断产生条件选择
 *
 * @param condi ADC_Int_Condition_Sample_Abnormal：采样值异常时产生中断
 *            ADC_INT_CONDITION_SAMPLE_END：每次采样结束都产生中断
 */
void adc_int_sel(adc_int_mode_t condi)
{
    if(ADC_INT_MODE_VALUE_NOT_MEET == condi)
    {
        ADC->ADCCTRL |= (1 << 12);
	}
    else
    {
		ADC->ADCCTRL &= ~(1 << 12);
	}
}


/**
 * @brief ADC通道采样结果低于阀值下限中断使能（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 * @param cmd 使能或关闭
 */
void adc_channel_min_value_int(adc_channelx_t channel,FunctionalState cmd)
{
    if(cmd != DISABLE)
    {
        if(ADC_CHANNEL_4 > channel)
        {
            ADC->ADCCTRL |= (1 << (channel + 16));
        }
        else
        {
            ADC->ADCCTRLa |= (1 << (channel + 2));
        }
	}
    else
    {
        if(ADC_CHANNEL_4 > channel)
        {
		    ADC->ADCCTRL &= ~(1 << (channel + 16));
        }
        else
        {
            ADC->ADCCTRLa &= ~(1 << (channel + 2));
        }
	}
    ADC->ADCLOADEN |= (0x1 << 0);
}


/**
 * @brief ADC通道采样结果超过阀值上限中断使能（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 * @param cmd 使能或关闭
 */
void adc_channel_max_value_int(adc_channelx_t channel,FunctionalState cmd)
{
    if(cmd != DISABLE)
    {
        if(ADC_CHANNEL_4 > channel)
        {
            ADC->ADCCTRL |= (1 << (channel + 20));
        }
        else
        {
            ADC->ADCCTRLa |= (1 << (channel + 4));
        }
    }
    else
    {
        if(ADC_CHANNEL_4 > channel)
        {
            ADC->ADCCTRL &= ~(1 << (channel + 20));
        }
        else
        {
            ADC->ADCCTRLa |= (1 << (channel + 4));
        }
    }
    ADC->ADCLOADEN |= (0x1 << 0);
}


/**
 * @brief ADC中断屏蔽设置
 *
 * @param cmd ENABLE：屏蔽ADC中断（不会产生ADC中断）
 *            DISABLE：不屏蔽ADC中断（会产生ADC中断）
 */
void adc_mask_int(FunctionalState cmd)
{
    if(cmd != DISABLE)
    {
        ADC->ADCINTMASK |= (1 << 0);
	}
    else
    {
		ADC->ADCINTMASK &= ~(1 << 0);
	}
}


/**
 * @brief 读取某个通道的ADC中断标志（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 * @return uint32_t 0：此通道没有ADC中断
 *                非0：此通道有ADC中断
 */
uint32_t adc_int_flag(adc_channelx_t channel)
{
	return ADC->ADCINTFLG & (1 << channel);
}



/**
 * @brief ADC中断标志清除（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 */
void adc_int_clear(adc_channelx_t channel)
{
	ADC->ADCINTCLR |= (1 << channel);
}


/**
 * @brief ADC软件触发（软件强制开始转换）
 *
 * @param cmd 使能或者不使能
 */
void adc_soc_soft_ctrl(FunctionalState cmd)
{
    if(ENABLE == cmd)
    {
        ADC->ADCSOFTSOC |= 1;
    }
	else
    {
        ADC->ADCSOFTSOC &= ~(1 << 0);
    }
}


/**
 * @brief :ADC 转换通道和采样保持时间配置（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 * @param holdtime 采样保持时间选择
 */
void adc_convert_config(adc_channelx_t channel,adc_clkcyclex_t holdtime)
{
	ADC->ADCSOCCTRL = (channel << 5)|(holdtime << 12);
}


/**
 * @brief 获取ADC某个通道的转换结果（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 *
 * @param channel 通道选择
 * @return unsigned int 某个通道的ADC转换结果（12位）
 */
uint32_t adc_get_result(adc_channelx_t channel)
{
	return ADC->ADCRESULT[channel];
}


/**
 * @brief ADC通道采样周期配置（配置之后采样周期为(period + 1)*512）
 * （只支持ADC_CHANNEL_0-ADC_CHANNEL_5）
 * @param channel 通道选择
 * @param period 周期配置
 */
void adc_chax_period(adc_channelx_t channel,uint16_t period)
{
    if(ADC_CHANNEL_4 > channel)
    {
	    //ADC->CHxPERIOD[channel] = period;
        ADC->CHxPERIOD[channel] = ((period*20) - 1) & 0xFFFF;
        
    }
    else
    {
        if(ADC_CHANNEL_4 == channel)
        {
            //ADC->CH4PERIOD = period;
            ADC->CH4PERIOD = ((period*20) - 1) & 0xFFFF;
            //ADC->CH4PERIOD = ((period*30) - 1) & 0xFFFF;
        }
        else if(ADC_CHANNEL_5 == channel)
        {
            //ADC->CH5PERIOD = period;
            ADC->CH5PERIOD = ((period*20) - 1) & 0xFFFF;
            //ADC->CH5PERIOD = ((period*30) - 1) & 0xFFFF;
        }
    }
    ADC->ADCLOADEN |= (0x1 << 0);
}


/**
 * @brief ADC 通道下限阀值配置（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 * （只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 * @param channel 通道选择
 * @param val 下限值
 */
void adc_channel_min_value(adc_channelx_t channel,uint16_t val)
{
    if(ADC_CHANNEL_0 == channel)
    {
        ADC->CH0MINVALUE = val & 0x0FFF;
    }
    else if(ADC_CHANNEL_1 == channel)
    {
        ADC->CH1MINVALUE = val & 0x0FFF;
    }
    else if(ADC_CHANNEL_2 == channel)
    {
        ADC->CH2MINVALUE = val & 0x0FFF;
    }
    else if(ADC_CHANNEL_3 == channel)
    {
        ADC->CH3MINVALUE = val & 0x0FFF;
	}
    else if(ADC_CHANNEL_4 == channel)
    {
        ADC->CH4MINVALUE = val & 0x0FFF;
	}
    else if(ADC_CHANNEL_5 == channel)
    {
        ADC->CH5MINVALUE = val & 0x0FFF;
	}
    else
    {
        //CI_ASSERT(0,"adc channel err!\n");
	}
    ADC->ADCLOADEN |= (0x1 << 0);
}


/**
 * @brief ADC 通道上限阀值配置（只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 * （只支持ADC_CHANNEL_0-ADC_CHANNEL_3）
 * @param channel 通道选择
 * @param val 上限值
 */
void adc_channel_max_value(adc_channelx_t channel,uint16_t val)
{
    if(ADC_CHANNEL_0 == channel)
    {
        ADC->CH0MAXVALUE = val & 0xFFFF;
    }
    else if(ADC_CHANNEL_1 == channel)
    {
        ADC->CH1MAXVALUE = val & 0xFFFF;
    }
    else if(ADC_CHANNEL_2 == channel)
    {
        ADC->CH2MAXVALUE = val & 0xFFFF;
    }
    else if(ADC_CHANNEL_3 == channel)
    {
        ADC->CH3MAXVALUE = val & 0xFFFF;
	}
    else if(ADC_CHANNEL_4 == channel)
    {
        ADC->CH4MAXVALUE = val & 0xFFFF;
	}
    else if(ADC_CHANNEL_5 == channel)
    {
        ADC->CH5MAXVALUE = val & 0xFFFF;
	}
    else
    {
        //CI_ASSERT(0,"adc channel err!\n");
	}
}

/**
 * @brief 获取ADC转换的电压值
 *
 * @param cha ADC通道选择
 * @param vol_val 存储转换之后的电压值的指针
 * @return int8_t RETURN_OK
 */
int8_t adc_get_vol_value(adc_channelx_t cha,float* vol_val)
{
    uint32_t val = adc_get_result(cha);
    *vol_val = (float)val*3.3f/4096.0f;
    return RETURN_OK;
}

void adc_poweron(void)
{
    ADC->ADCCTRLa |= (0x1 << 3);
    ADC->ADCRSTN |= (0x7 << 0);//高于400ns
    ADC->ADCLOADEN |= (0x1 << 0);
    adc_power_ctrl(ENABLE);
    while(!(ADC->ADCPWRRDY & 0x1));
}

void adc_powerdown(void)
{
    adc_power_ctrl(DISABLE);
    adc_clear_flag();
}

void adc_reset(void)
{
	//进入复位
    ADC->ADCCTRLa &= ~(0x1 << 3);
    ADC->ADCLOADEN |= (0x1 << 0);
    while(ADC->ADCPWRRDY & 0x1);
    //释放复位
    ADC->ADCCTRLa |= (0x1 << 3);
	ADC->ADCLOADEN |= (0x1 << 0);
	while(!(ADC->ADCPWRRDY & 0x1));
}

void adc_signal_mode(adc_channelx_t cha)
{
    adc_convert_config(cha,ADC_CLKCYCLE_2);
    adc_continuons_convert(DISABLE);
    adc_int_sel(ADC_INT_MODE_TRANS_END);
    //读取异常值
    adc_soc_soft_ctrl(ENABLE);
    adc_wait_int(cha);
}

void adc_series_mode(adc_channelx_t cha)
{
    adc_convert_config(cha,ADC_CLKCYCLE_2);
    adc_channel_min_value(cha,0);
    adc_channel_min_value_int(cha,ENABLE);
    adc_channel_max_value(cha,4095);
    adc_channel_max_value_int(cha,ENABLE);
    adc_continuons_convert(ENABLE);
    adc_int_sel(ADC_INT_MODE_VALUE_NOT_MEET);
}

void adc_cycle_mode(adc_channelx_t cha,uint16_t cycle)
{
    // adc_calibrate(ENABLE); //校准使能
	adc_convert_config(cha,ADC_CLKCYCLE_2);
	adc_channel_min_value(cha,0);
    // adc_channel_min_value(cha,1241)
	adc_channel_min_value_int(cha,ENABLE);
	adc_channel_max_value(cha,4095);
    // adc_channel_min_value(cha,3723)
	adc_channel_max_value_int(cha,ENABLE);
	adc_chax_period(cha,cycle);
	adc_continuons_convert(DISABLE);
	adc_period_monitor(cha,ENABLE);
	adc_period_enable(ENABLE);
	//adc_int_sel(ADC_INT_MODE_TRANS_END);
    adc_int_sel(ADC_INT_MODE_VALUE_NOT_MEET);
}

void adc_caculate_mode(void)
{
	ADC->ADCOFFSET = 0;
	ADC->ADCCTRLa |= (0x1 << 2);
	ADC->ADCCTRLa |= (0x1 << 0);
	ADC->ADCLOADEN |= (0x1 << 0);
	adc_signal_mode(ADC_CHANNEL_0);
	adc_soc_soft_ctrl(ENABLE);
	adc_wait_int(ADC_CHANNEL_0);
	uint32_t temp = adc_get_result(ADC_CHANNEL_0);
	if(temp <= 0x800)
	{
		ADC->ADCOFFSET = 0x800 - temp;
	}
	else
	{
        ADC->ADCOFFSET = ((0x1 << 5) | (~(temp - 0x800) + 1));
	}
	ADC->ADCCTRLa &= ~(0x1 << 2);
	ADC->ADCCTRLa &= ~(0x1 << 0);
    ADC->ADCLOADEN |= (0x1 << 0);
}



void soft_trige_start(int channel_num)
{
  unsigned int tmp;
  /*选择转换通道*/
  tmp = ADC->ADCSOCCTRL;
  tmp &= ~(0x7 << 5);
  tmp |= (channel_num << 5);
  ADC->ADCSOCCTRL = tmp;
  /*ADC强制开始转化*/
  adc_soc_soft_ctrl(ENABLE);

}
void continu_init(void)
{
  int i;
  AIN3 = 0x37;
  ADC->ADCINTCLR = 0x10f;
  /*不开启ADC中断*/
  ADC->ADCINTMASK = 0x1; //屏蔽中断
  /*连续转换模式，SOC连续为高*/
  ADC->ADCCTRL |= ((0x1 << 0) | (0x1 << 1)); //SOC持续为高，连续转换
  i = 500;
  while(i--);
}
void continu_start(void)
{
  unsigned int tmp;
  /*选择转换通道，并启动ADC*/
  tmp = ADC->ADCSOCCTRL;
  tmp &= ~(0x7 << 5);
  tmp |= (3 << 5);
  ADC->ADCSOCCTRL = tmp;//ADC AIN3

  adc_soc_soft_ctrl(ENABLE);
  //ADC->ADCSOCFRC = 1;

}

void period_init(void)
{
  int i;
  AIN0 = 0x1b;
  AIN1 = 0x91;
  AIN2 = 0xA0;
  AIN3 = 0xB0;

  ADC->ADCCTRL &= ~(0x1 << 1);//单次转换模式

  ADC->ADCINTCLR = 0x10F;
  //采样周期配置寄存器
  ADC->CHxPERIOD[0] = 0;
  ADC->CHxPERIOD[1] = 1;
  ADC->CHxPERIOD[2] = 2;
  ADC->CHxPERIOD[3] = 3;

  ADC->ADCINTCLR = 0x10F;
  ADC->ADCINTMASK = 0;
  ADC->ADCCTRL |= ((0x1 << 0) | (0x1F << 4));//打开ADC，周期监测使能
  i = 500;
  while(i--);
}

void period_other_init(void)
{
  int i;

  AIN0 = 0x1a;
  AIN1 = 0x93;
  AIN2 = 0x46;
  AIN3 = 0x28;

  ADC->ADCCTRL &= ~(0x1f << 4);
  while((ADC->ADCSTAT&(0x1 << 0))==1);
  //ADC->ADCINTCLR = 0x10F;
  ADC->CHxPERIOD[0] = 2;
  ADC->CHxPERIOD[1] = 2;
  ADC->CHxPERIOD[2] = 2;
  ADC->CHxPERIOD[3] = 2;

  ADC->CH0MINVALUE = 0x1a;
  ADC->CH0MAXVALUE = 0x1a + 5;

  ADC->CH1MINVALUE = 0x93;
  ADC->CH1MAXVALUE = 0x93 + 5;

  ADC->CH2MINVALUE = 0x46;
  ADC->CH2MAXVALUE = 0x46 + 5;

  ADC->CH3MINVALUE = 0x28;
  ADC->CH3MAXVALUE = 0x28 + 5;



  ADC->ADCINTCLR = 0x10F;
  ADC->ADCINTMASK = 0;
  //ADC周期检测模式；采样值异常时产生中断；采样结果阈值上下限中断
  ADC->ADCCTRL |= ((0x1 << 0) | (0x1F << 4) | (0x1 << 12) | (0xff << 16));
  i = 500;
  while(i--);
}

/***************** (C) COPYRIGHT Chipintelli Technology Co., Ltd. *****END OF FILE****/
