/**
 * @file es8388.c
 * @brief 为ES8388封装的驱动程序
 * @version 0.1
 * @date 2019-03-28
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#include "sdk_default_config.h"
#include "ci130x_iic.h"
//#include "ci130x_iis.h"
#include "ci130x_iisdma.h"
#include "es8388.h"
#include "sdk_default_config.h"
#include <stdint.h>
#include <string.h>
#include "ci_log.h"
#include "ci130x_system.h"
#include "platform_config.h"
#include "sdk_default_config.h"


/*ES8388寄存器定义*/
#define ES8388_CONTROL1         (0x00)
#define ES8388_CONTROL2         (0x01)
#define ES8388_CHIPPOWER        (0x02)
#define ES8388_ADCPOWER         (0x03)
#define ES8388_DACPOWER         (0x04)
#define ES8388_CHIPLOPOW1       (0x05)
#define ES8388_CHIPLOPOW2       (0x06)
#define ES8388_ANAVOLMANAG      (0x07)
#define ES8388_MASTERMODE       (0x08)

#define ES8388_ADCCONTROL1      (0x09)
#define ES8388_ADCCONTROL2      (0x0a)
#define ES8388_ADCCONTROL3      (0x0b)
#define ES8388_ADCCONTROL4      (0x0c)
#define ES8388_ADCCONTROL5      (0x0d)
#define ES8388_ADCCONTROL6      (0x0e)
#define ES8388_ADCCONTROL7      (0x0f)
#define ES8388_ADCCONTROL8      (0x10)
#define ES8388_ADCCONTROL9      (0x11)
#define ES8388_ADCCONTROL10     (0x12)
#define ES8388_ADCCONTROL11     (0x13)
#define ES8388_ADCCONTROL12     (0x14)
#define ES8388_ADCCONTROL13     (0x15)
#define ES8388_ADCCONTROL14     (0x16)

#define ES8388_DACCONTROL1      (0x17)
#define ES8388_DACCONTROL2      (0x18)
#define ES8388_DACCONTROL3      (0x19)
#define ES8388_DACCONTROL4      (0x1a)
#define ES8388_DACCONTROL5      (0x1b)
#define ES8388_DACCONTROL6      (0x1c)
#define ES8388_DACCONTROL7      (0x1d)
#define ES8388_DACCONTROL8      (0x1e)
#define ES8388_DACCONTROL9      (0x1f)
#define ES8388_DACCONTROL10     (0x20)
#define ES8388_DACCONTROL11     (0x21)
#define ES8388_DACCONTROL12     (0x22)
#define ES8388_DACCONTROL13     (0x23)
#define ES8388_DACCONTROL14     (0x24)
#define ES8388_DACCONTROL15     (0x25)
#define ES8388_DACCONTROL16     (0x26)
#define ES8388_DACCONTROL17     (0x27)
#define ES8388_DACCONTROL18     (0x28)
#define ES8388_DACCONTROL19     (0x29)
#define ES8388_DACCONTROL20     (0x2a)
#define ES8388_DACCONTROL21     (0x2b)
#define ES8388_DACCONTROL22     (0x2c)
#define ES8388_DACCONTROL23     (0x2d)
#define ES8388_DACCONTROL24     (0x2e)
#define ES8388_DACCONTROL25     (0x2f)
#define ES8388_DACCONTROL26     (0x30)
#define ES8388_DACCONTROL27     (0x31)
#define ES8388_DACCONTROL28     (0x32)
#define ES8388_DACCONTROL29     (0x33)
#define ES8388_DACCONTROL30     (0x34)

#define ES8388_LADC_VOL         ES8388_ADCCONTROL8
#define ES8388_RADC_VOL         ES8388_ADCCONTROL9

#define ES8388_LDAC_VOL         ES8388_DACCONTROL4
#define ES8388_RDAC_VOL         ES8388_DACCONTROL5

#define ES8388_LOUT1_VOL        ES8388_DACCONTROL24
#define ES8388_ROUT1_VOL        ES8388_DACCONTROL25
#define ES8388_LOUT2_VOL        ES8388_DACCONTROL26
#define ES8388_ROUT2_VOL        ES8388_DACCONTROL27

#define ES8388_ADC_MUTE         ES8388_ADCCONTROL7
#define ES8388_DAC_MUTE         ES8388_DACCONTROL3

#define ES8388_IFACE            ES8388_MASTERMODE

#define ES8388_ADC_IFACE        ES8388_ADCCONTROL4

#define ES8388_DAC_IFACE        ES8388_DACCONTROL1
#define ES8388_DAC_SRATE        ES8388_DACCONTROL2
/*ES8388寄存器结束*/

// static void _delay_10us_240M(unsigned int cnt)
// {
//     unsigned int j=0;
//     unsigned int i,sum = 0;
//     for(i = 0; i < cnt; i++)
//     {
//         for(j = 0; j < 0x3f; j++)
//         {
//             sum += j % 10;
//         }
//     }
// }


/**
 * @brief es8388使用的I2C初始化
 * 
 * @param num_8388 8388的编号
 */
void es8388_i2c_init(es8388_num_sel_t num_8388)
{
    i2c_io_init();
    i2c_master_init(IIC0,100,SHORT_TIME_OUT);

}



/**
 * @brief 为8388写入一个字节数据
 * 
 * @param num_8388 8388编号
 * @param regaddr 8388寄存器地址
 * @param val 需要写入的值
 */
static void I2C_WriteEs8388Data(es8388_num_sel_t num_8388,unsigned int regaddr,char val)
{
    char buf[2] = {0};
    buf[0] = regaddr;
    buf[1] = val;

    i2c_master_only_send(0x10,buf,2);   //slave的IIC设备地址：0x10
    //_delay_10us_240M(5000);
}    


/**
 * @brief 读取8388某个寄存器的值
 * 
 * @param num_8388 8388编号
 * @param regaddr 8388寄存器地址
 * @return int 读出寄存器的值
 */
static int I2C_ReadEs8388Data(es8388_num_sel_t num_8388,unsigned int regaddr)
{
    char buf[2] = {0};
    buf[0] = regaddr;

    i2c_master_send_recv(0x10,buf,1,1);    //slave的IIC设备地址：0x10
    //_delay_10us_240M(5);
    return (unsigned char)buf[0];   
}


/**
 * @brief 设置8388 复位
 * 
 * @param num_8388 8388编号
 * @return int32_t 正确
 */
int32_t ES8388_reset(es8388_num_sel_t num_8388)
{
    volatile uint32_t reg_data = 0;
    reg_data |= (1<<8);
    I2C_WriteEs8388Data(num_8388,ES8388_CONTROL1,reg_data);
    return RETURN_OK;
}



/**
 * @brief 设置8388 解除复位
 * 
 * @param num_8388 8388编号
 * @return int32_t 正确
 */
int32_t ES8388_reset_release(es8388_num_sel_t num_8388)
{
    volatile uint32_t reg_data = 0;
    reg_data &= ~(1<<8);
    I2C_WriteEs8388Data(num_8388,ES8388_CONTROL1,reg_data);
    return RETURN_OK;
}


//ES8388_DAC_MICIN_AEC使用到
// #define RESET_RELEASE    (0x0<<7)
// #define RESET            (0x1<<7)

// #define LRCM_0       (0x0<<6)
// #define LRCM_1       (0x1<<6)

//此配置只有SameFs为1才有效
#define ADCMCLK_AS_MASTER_CLOCK  (0x0<<5)
#define DACMCLK_AS_MASTER_CLOCK  (0x1<<5)

#define ADC_FS_DIF_FROM_DAC_FS   (0x0<<4)
#define ADC_FS_SAME_AS_DAC_FS    (0x1<<4)

// #define INTERNAL_POWER_UP_DOWN_SEQUENCE_DISABLE  (0x0<<3)
// #define INTERNAL_POWER_UP_DOWN_SEQUENCE_ENABLE   (0x1<<3)

#define DISABLE_REFERENCE        (0x0<<2)
#define ENABLE_REFERENCE         (0x1<<2)

#define VMID_DISABLE     (0x0<<0)
#define DIVIDER_50K      (0x1<<0)
#define DIVIDER_500K     (0x2<<0)
#define DIVIDER_5K       (0x3<<0)


/**
 * @brief 设置8388 chipcontrol的配置
 * 
 * @param num_8388 8388编号
 * @param dacmclk DAC MCLK选择
 * @param samefs 过采样设置
 * @param ref 是否使能ref
 * @param divider 电阻选择
 * @return int32_t 正确
 */
int32_t ES8388_chip_control_config(es8388_num_sel_t num_8388,uint32_t dacmclk,uint32_t samefs,
                                        uint32_t ref,uint32_t divider)
{
    volatile uint32_t reg_data = 0;
    reg_data |= dacmclk | samefs | ref | divider;
    I2C_WriteEs8388Data(num_8388,ES8388_CONTROL1,reg_data);
    return RETURN_OK;
}



/**
 * @brief 设置8388 mastermode寄存器的值，使用基本固定，全部都是0x00
 * 
 * @param num_8388 8388编号
 * @param mode 8388的主从模式选择
 * @return int32_t 正确
 */
int32_t ES8388_mastermode_config(es8388_num_sel_t num_8388,es8388_mode_sel_t mode)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_MASTERMODE);
    reg_data &= ~(1 << 7);
    reg_data |= (mode << 7);//(mode << 7)|(1 << 5);
    I2C_WriteEs8388Data(num_8388,ES8388_MASTERMODE,reg_data);
    return RETURN_OK;
}


/**
 * @brief 设置8388 MIC增益的设置
 * 
 * @param num_8388 8388编号
 * @param l_amp 左通道MIC的增益
 * @param r_amp 右通道MIC的增益
 * @return int32_t 正确
 */
int32_t ES8388_ADC_mic_amplify_set(es8388_num_sel_t num_8388,es8388_mic_amp_t l_amp,es8388_mic_amp_t r_amp)
{
    volatile uint32_t reg_data = 0;
    reg_data = (l_amp << 4) | r_amp;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL1,reg_data);
    return RETURN_OK;
}


/*Register10 ADC Control2*/
//用法基本固定
//LINSEL
#define LINPUT1              (0x0<<6)
#define LINPUT2              (0x1<<6)
#define DIFFERENTIAL_L_CHA   (0x3<<6)

//RINSEL
#define RINPUT1              (0x0<<4)
#define RINPUT2              (0x1<<4)
#define DIFFERENTIAL_R_CHA   (0x3<<4)

//DSSEL
#define USE_ONE_DSL_REG11_BIT7               (0x0<<3)
#define USE_DSL_REG11_BIT7_DSR_REG10_BIT2    (0x1<<3)

//DSR
#define DSR_LINPUT1_RINPUT1     (0x0<<2)
#define DSR_LINPUT2_RINPUT2     (0x1<<2)


/*Register11 ADC Control3*/
//用法基本固定
//DSL
#define DSL_LINPUT1_RINPUT1     (0x0<<7)
#define DSL_LINPUT2_RINPUT2     (0x1<<7)

//MONOMIX
#define MONOMIX_STEREO               (0x0<<3)
#define ANALOG_MONO_MIX_TO_LEFT      (0x1<<3)
#define ANALOG_MONO_MIX_TO_RIGHT     (0x2<<3)

//TRI
#define ASDOUT_IS_ADC_NORMAL_OUTPUT      (0x0<<2)
#define ASDOUT_TRI_STATED                (0x1<<2)
/**
 * @brief 8388的输入线路选择
 * 
 * @param num_8388 8388编号
 * @param InChaType ES8388_Input_Channal_TypeDef类型结构体指针
 * @return int32_t 正确
 */
static int32_t ES8388_input_channal_config(es8388_num_sel_t num_8388,es8388_input_cha_t* InChaType)
{
    volatile uint32_t reg_data = 0;
    reg_data |= InChaType->l_input_single_sel | InChaType->r_input_single_sel
            | InChaType->ds_sel | InChaType->dsr;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL2,reg_data);

    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL3);
    
    reg_data |= (1<<1);
    
    reg_data &= ~(0x3f<<2);
    reg_data |= InChaType->dsl | InChaType->monomix | InChaType->tri;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL3,reg_data);
    return RETURN_OK;
}


/*Register12 ADC Control4*/
//数据通路选择
#define ADC_LDATA_LADC_RDATA_RADC (0x0<<6)
#define ADC_LDATA_LADC_RDATA_LADC (0x1<<6)
#define ADC_LDATA_RADC_RDATA_RADC (0x2<<6)
#define ADC_LDATA_RADC_RDATA_LADC (0x3<<6)

//ADC通道数据反转
//DSP/PCM格式时，选择数据最高位出现的时机
#define ADC_DATA_NORMAL_POLARITY      (0<<5)
#define ADC_DATA_INVERTED_POLARITY    (1<<5)
#define ADC_DATA_2ND_BCLK    (0<<5)
#define ADC_DATA_1ST_BCLK    (1<<5)

/**
 * @brief ADC的数据格式设置
 * 
 * @param num_8388 8388编号
 * @param data_length 数据宽度
 * @param data_format 数据格式：左右对齐；标准I2S
 * @return int32_t 正确
 */
int32_t ES8388_ADC_data_format_set(es8388_num_sel_t num_8388,es8388_adc_data_len_t data_length,es8388_adc_data_format_t data_format)
{
    volatile uint8_t reg_data = 0;
    uint8_t data_channal_set = ADC_LDATA_LADC_RDATA_RADC;
    uint8_t data_lrp = ADC_DATA_NORMAL_POLARITY;
    reg_data |= data_channal_set | data_lrp | data_length | data_format;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL4,reg_data);
    return RETURN_OK;
}



/*Register13 ADC Control5*/
//ADC采样模式
#define FS_SINGLE_SPEED  (0x0<<5)
#define FS_DOUBLE_SPEED  (0x1<<5)
//ADC过采样率
#define FS_RATIO_128   (0x0<<0)
#define FS_RATIO_192   (0x1<<0)
#define FS_RATIO_256   (0x2<<0)
#define FS_RATIO_384   (0x3<<0)
#define FS_RATIO_512   (0x4<<0)
#define FS_RATIO_576   (0x5<<0)
#define FS_RATIO_768   (0x6<<0)
#define FS_RATIO_1024  (0x7<<0)
#define FS_RATIO_1152  (0x8<<0)
#define FS_RATIO_1408  (0x9<<0)
#define FS_RATIO_1536  (0xa<<0)
#define FS_RATIO_2112  (0xb<<0)
#define FS_RATIO_2304  (0xc<<0)

#define FS_RATIO_125   (0x10<<0)
#define FS_RATIO_136   (0x11<<0)
#define FS_RATIO_250   (0x12<<0)
#define FS_RATIO_272   (0x13<<0)
#define FS_RATIO_375   (0x14<<0)
#define FS_RATIO_500   (0x15<<0)
#define FS_RATIO_544   (0x16<<0)
#define FS_RATIO_750   (0x17<<0)
#define FS_RATIO_1000  (0x18<<0)
#define FS_RATIO_1088  (0x1a<<0)
#define FS_RATIO_1496  (0x1b<<0)
#define FS_RATIO_1500  (0x1c<<0)
/**
 * @brief ADC过采样率设置
 * 
 * @param num_8388 8388编号
 * @param fs_mode ADC MCLK模式
 * @param fs_ratio 过采样率
 * @return int32_t 正确
 */
int32_t ES8388_ADC_oversample_config(es8388_num_sel_t num_8388,uint32_t fs_mode,uint32_t fs_ratio)
{
    volatile uint32_t reg_data = 0;
    reg_data |= fs_mode | fs_ratio;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL5,reg_data);
    return RETURN_OK;
}



/**
 * @brief 设置8388 ADC 数据极性反转
 * 
 * @param num_8388 8388编号
 * @param gate_l 左声道数据极性反转开关
 * @param gate_r 右声道数据极性反转开关
 * @return int32_t 正确
 */
int32_t ES8388_ADC_invl_config(es8388_num_sel_t num_8388,es8388_adc_inv_sel_t gate_l,es8388_adc_inv_sel_t gate_r)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL6);
    reg_data &= ~(3<<6);
    reg_data |= (gate_l<<7) | (gate_r<<6);
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL6,reg_data);
    return RETURN_OK;
}



//高通滤波器开关
#define ADC_HPFL_ENABLE    (1<<5)
#define ADC_HPFL_DISABLE   (0<<5)
#define ADC_HPFR_ENABLE    (1<<4)
#define ADC_HPFR_DISABLE   (0<<4)
/**
 * @brief 设置8388 ADC 高通滤波器的开关（8388的高通滤波器的截止频率大概为6.7Hz）
 * 
 * @param num_8388 8388编号
 * @param gate_l 左通道HPF开关
 * @param gate_r 右通道HPF开关
 * @return int32_t 正确
 */
int32_t ES8388_ADC_HPF_config(es8388_num_sel_t num_8388,uint32_t gate_l,uint32_t gate_r)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL6);
    reg_data &= ~(3<<4);
    reg_data |= gate_l | gate_r;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL6,reg_data);
    return RETURN_OK;
}



/*Register15 ADC Control7*/
//0.5dB per x LRCK digital volume control ramp rate
#define ADC_RAMPRATE_4LRCK     (0<<6)
#define ADC_RAMPRATE_8LRCK     (1<<6)
#define ADC_RAMPRATE_16LRCK    (2<<6)
#define ADC_RAMPRATE_32LRCK    (3<<6)

#define ADC_SOFTRAMP_ENABLE    (1<<5)
#define ADC_SOFTRAMP_DISABLE   (0<<5)
/**
 * @brief 设置8388 ADC 的softramp设置
 * 
 * @param num_8388 8388编号
 * @param gate softramp开关
 * @param x_lrck 
 * @return int32_t 正确
 */
int32_t ES8388_ADC_soft_ramp_config(es8388_num_sel_t num_8388,uint32_t gate,uint32_t x_lrck)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL7);
    reg_data &= ~(7<<5);
    reg_data |= gate | x_lrck;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL7,reg_data);
    return RETURN_OK;
}


//ADC Mute
#define ADC_MUTE_ENABLE   (1<<2)
#define ADC_MUTE_DISABLE  (0<<2)
/**
 * @brief 设置8388 ADC mute的开关
 * 
 * @param num_8388 8388编号
 * @param gate ADC 静音开关
 * @return int32_t 正确
 */
int32_t ES8388_ADC_mute_config(es8388_num_sel_t num_8388,uint32_t gate)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL7);
    reg_data &= ~(1<<2);
    reg_data |= gate;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL7,reg_data);
    return RETURN_OK;
}



/**
 * @brief 配置8388的ADCVOL的值
 * 
 * @param num_8388 8388编号
 * @param adc_vol_l 左声道数字增益，从0dB到-96dB，步进为0.5dB
 * @param adc_vol_r 右声道数字增益，从0dB到-96dB，步进为0.5dB
 * @return int32_t 正确
 */
int32_t ES8388_ADC_vol_config(es8388_num_sel_t num_8388,float adc_vol_l,float adc_vol_r)
{
    volatile uint32_t reg_data = 0;
    if(adc_vol_l < -96.0f)
    {
        adc_vol_l = -96.0f;
    }
    else if(adc_vol_l > 0.0f)
    {
        adc_vol_l = 0.0f;
    }
    
    if(adc_vol_r < -96.0f)
    {
        adc_vol_r = -96.0f;
    }
    else if(adc_vol_r > 0.0f)
    {
        adc_vol_r = 0.0f;
    }
    
    else
    {
        reg_data = (int)(-adc_vol_l * 0.5f);
        I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL8,reg_data);
        reg_data = (int)(-adc_vol_r * 0.5f);
        I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL9,reg_data);
    }
    return RETURN_OK;
}



//ALC holdtime(ms)
#define ALC_HOLDTIME_0MS     (0<<0)
#define ALC_HOLDTIME_2_67MS  (1<<0)
#define ALC_HOLDTIME_5_33MS  (2<<0)
#define ALC_HOLDTIME_10_5MS  (3<<0)
#define ALC_HOLDTIME_21MS    (4<<0)
#define ALC_HOLDTIME_42MS    (5<<0)
#define ALC_HOLDTIME_85MS    (6<<0)
#define ALC_HOLDTIME_170MS   (7<<0)
#define ALC_HOLDTIME_340MS   (8<<0)
#define ALC_HOLDTIME_680MS   (9<<0)
#define ALC_HOLDTIME_01360MS (10<<0)


/*Register20 ADC Control12*/
//下面这些time都是常用的normal mode的时间，
//limiter mode的时间为此时间的1/4.516
//ALC decay time(ms)
#define ALC_DECAYTIME_0_41MS   (0<<4) 
#define ALC_DECAYTIME_0_82MS   (1<<4) 
#define ALC_DECAYTIME_1_64MS   (2<<4) 
#define ALC_DECAYTIME_3_28MS   (3<<4) 
#define ALC_DECAYTIME_6_56MS   (4<<4) 
#define ALC_DECAYTIME_13_12MS  (5<<4) 
#define ALC_DECAYTIME_26_24MS  (6<<4) 
#define ALC_DECAYTIME_52_5MS   (7<<4) 
#define ALC_DECAYTIME_105MS    (8<<4) 
#define ALC_DECAYTIME_210MS    (9<<4) 
#define ALC_DECAYTIME_420MS    (10<<4) 

//下面这些time都是常用的normal mode的时间，
//limiter mode的时间为此时间的1/4.57
//ALC attack time(ms)
#define ALC_ATTACKTIME_0_104MS    (0<<0)
#define ALC_ATTACKTIME_0_208MS    (1<<0) 
#define ALC_ATTACKTIME_0_416MS    (2<<0) 
#define ALC_ATTACKTIME_0_832MS    (3<<0) 
#define ALC_ATTACKTIME_1_664MS    (4<<0) 
#define ALC_ATTACKTIME_3_328MS    (5<<0) 
#define ALC_ATTACKTIME_6_65MS     (6<<0) 
#define ALC_ATTACKTIME_13_3MS     (7<<0) 
#define ALC_ATTACKTIME_26_6MS     (8<<0) 
#define ALC_ATTACKTIME_53_2MS     (9<<0) 
#define ALC_ATTACKTIME_106MS      (10<<0)


/*Register21 ADC Control13*/
//ALCMODE
#define ALC_NORMAL_MODE    (0<<7)
#define ALC_LIMITER_MODE   (1<<7)

//zero cross
#define ALC_ZERO_CROSS_DISABLE  (0<<6)
#define ALC_ZERO_CROSS_ENABLE   (1<<6)
//zero cross timeout
#define ALC_ZERO_CROSS_TIMEOUT_DISABLE  (0<<5)
#define ALC_ZERO_CROSS_TIMEOUT_ENABLE   (1<<5)

/**
 * @brief 配置8388的ALC
 * 
 * @param num_8388 8388编号
 * @param ALC_str ES8388_ALC_Config_TypeDef类型结构体指针
 * @param ALC_gate 左声道、右声道或者双声道的ALC开关
 * @return int32_t 正确
 */
int32_t ES8388_ALC_config(es8388_num_sel_t num_8388,es8388_alc_config_t* ALC_str,es8388_alc_cha_gate_t ALC_gate)
{
    volatile uint32_t reg_data = 0;
    uint32_t samples = 0;
    uint32_t ALC_hold_time = ALC_HOLDTIME_0MS;
    uint32_t ALC_decay_time = ALC_DECAYTIME_0_41MS;
    uint32_t ALC_attack_time = ALC_ATTACKTIME_3_328MS;
    uint32_t ALC_mode = ALC_NORMAL_MODE;
    uint32_t ALC_zerocross_gate = ALC_ZERO_CROSS_DISABLE;
    uint32_t ALC_zerocross_timeout_gate = ALC_ZERO_CROSS_TIMEOUT_DISABLE;
    uint32_t ALC_peak_detector_windows_size = 96; 

    reg_data = (uint32_t)ALC_gate | (uint32_t)ALC_str->alc_maxgain | (uint32_t)ALC_str->alc_mingain;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL10,reg_data);
    reg_data = 0;
    reg_data = (uint32_t)ALC_str->alc_level | (uint32_t)ALC_hold_time;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL11,reg_data);
    reg_data = 0;
    reg_data = (uint32_t)ALC_decay_time | (uint32_t)ALC_attack_time;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL12,reg_data);
    reg_data = 0;
    samples = ALC_peak_detector_windows_size;

    if(samples < 96)
    {
        samples = 96;
    }
    else if(samples > 496)
    {
        samples = 496;
    }
    
    reg_data |= (samples/16) & 0x1f;
    reg_data |= ALC_mode | ALC_zerocross_gate | ALC_zerocross_timeout_gate;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL13,reg_data);
    return RETURN_OK;
}


void ES8388_alc_gate(es8388_num_sel_t num_8388,es8388_alc_cha_gate_t ALC_gate)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL10);
    reg_data &= ~(3<<6);
    reg_data |= ALC_gate;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL10,reg_data);
}


void ES8388_alc_max_gain_set(es8388_num_sel_t num_8388,es8388_alc_maxgain_t max_gain)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_ADCCONTROL10);
    reg_data &= ~(7<<3);
    reg_data |= max_gain;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL10,reg_data);
}


//NGG noise gate type
#define PGA_GAIN_HELD_CONSTANT   (0x0<<1)
#define MUTE_ADC_OUTPUT          (0x1<<1)

//NGAT noise gate function enable
#define NOISE_GATE_DISABLE   (0x0<<0)
#define NOISE_GATE_ENABLE    (0x1<<0)
/**
 * @brief 设置8388 ALC noisegate设置
 * 
 * @param num_8388 8388编号
 * @param threshold 噪声门限阈值（-76.5dBFS -- -30dBFS）
 * @param type 噪声门限模式
 * @param gate 噪声门限开关
 * @return int32_t 正确
 */
static int32_t ES8388_noise_gate_config(es8388_num_sel_t num_8388,float threshold,uint32_t type,uint32_t gate)
{
    volatile uint32_t reg_data = 0;
    volatile uint32_t threshold_int = 0;
    if(threshold < -76.5)
    {
        threshold = -76.5;
    }
    else if(threshold > -30)
    {
        threshold = -30;
    }
    
    threshold_int = (uint32_t)((threshold + 76.5)/1.5);
    reg_data = (threshold_int << 3) | type | gate;
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL14,reg_data);
    return RETURN_OK;
}


/*Register23 DAC Control1*/
#define DAC_LRSWAP_DISABLE   (0<<7)
#define DAC_LRSWAP_ENABLE    (1<<7)

#define DAC_DATA_NORMAL_POLARITY      (0<<6)
#define DAC_DATA_INVERTED_POLARITY    (1<<6)
#define DAC_DATA_2ND_BCLK    (0<<6)
#define DAC_DATA_1ST_BCLK    (1<<6)

/**
 * @brief 设置8388 DAC数据格式设置
 * 
 * @param num_8388 8388编号
 * @param data_length 数据的宽度
 * @param data_format 数据的格式
 * @return int32_t 正确
 */
int32_t ES8388_DAC_data_format_set(es8388_num_sel_t num_8388,es8388_dac_data_len_t data_length,es8388_dac_data_format_t data_format)
{
    volatile uint32_t reg_data = 0;
    uint8_t data_swap = DAC_LRSWAP_DISABLE;
    uint8_t data_lrp = DAC_DATA_NORMAL_POLARITY;
    reg_data |= data_swap | data_lrp | data_length | data_format;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL1,reg_data);
    return RETURN_OK;
}



/**
 * @brief DAC 过采样设置
 * 
 * @param num_8388 8388编号
 * @param fs_mode 过采样模式
 * @param fs_ratio 过采样率设置
 * @return int32_t 正确
 */
int32_t ES8388_DAC_oversample_config(es8388_num_sel_t num_8388,uint32_t fs_mode,uint32_t fs_ratio)
{
    volatile uint32_t reg_data = 0;
    reg_data = fs_mode | fs_ratio;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL2,reg_data);
    return RETURN_OK;
}


/**
 * @brief 配置8388的DACVOL的值
 * 
 * @param num_8388 8388编号
 * @param DAC_vol_l 左通道VOL值，0dB到-96dB，步进为0.5dB
 * @param DAC_vol_r 右通道VOL值，0dB到-96dB，步进为0.5dB
 * @return int32_t 正确
 */
int32_t ES8388_DAC_vol_config(es8388_num_sel_t num_8388,float DAC_vol_l,float DAC_vol_r)
{
    volatile uint32_t reg_data = 0;
    
    if(DAC_vol_l < -96)
    {
        DAC_vol_l = -96;
    }
    else if(DAC_vol_l > 0)
    {
        DAC_vol_l = 0;
    }
    
    if(DAC_vol_r < -96)
    {
        DAC_vol_r = -96;
    }
    else if(DAC_vol_r > 0)
    {
        DAC_vol_r = 0;
    }

    else
    {
        reg_data = (int)(-DAC_vol_l * 0.5);
        I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL4,reg_data);
        reg_data = (int)(-DAC_vol_r * 0.5);
        I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL5,reg_data);
    }
    return RETURN_OK;
}


/*Register28 DAC Contro6*/
//去加重模式
#define DAC_DEEMPHASISMODE_DISABLE   (0<<6)
#define DAC_DEEMPHASISMODE_32K       (1<<6)
#define DAC_DEEMPHASISMODE_44_1K     (2<<6)
#define DAC_DEEMPHASISMODE_48K       (3<<6)
/**
 * @brief DAC DEEMPHASISMODE的设置(去加重模式设置)
 * 
 * @param num_8388 8388编号
 * @param mode 模式设置
 * @return int32_t 正确
 */
int32_t ES8388_DAC_deemphasismode_config(es8388_num_sel_t num_8388,uint32_t mode)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL6);
    reg_data &= ~(3<<6);
    reg_data |= mode;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL6,reg_data);
    return RETURN_OK;
}



//相位是否移动180
#define DAC_L_NO_PHASE_INVERSION     (0<<5)
#define DAC_L_180_PHASE_INVERSION    (1<<5)

#define DAC_R_NO_PHASE_INVERSION     (0<<4)
#define DAC_R_180_PHASE_INVERSION    (1<<4)
/**
 * @brief DAC 左右通道相位180度翻转开关
 * 
 * @param num_8388 8388编号
 * @param phase_l 左声道数据是否翻转
 * @param phase_r 右声道数据是否翻转
 * @return int32_t 正确
 */
int32_t ES8388_DAC_phase_inversion_config(es8388_num_sel_t num_8388,uint32_t phase_l,uint32_t phase_r)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL6);
    reg_data &= ~(3<<4);
    reg_data |= phase_l | phase_r;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL6,reg_data);
    return RETURN_OK;
}



/*Register29 DAC Control7*/
//是否使DAC的输出全部为0
#define DAC_ZERO_OUTPUT_L_DISABLE    (0<<7)
#define DAC_ZERO_OUTPUT_L_ENABLE     (1<<7)

#define DAC_ZERO_OUTPUT_R_DISABLE    (0<<6)
#define DAC_ZERO_OUTPUT_R_ENABLE     (1<<6)
/**
 * @brief DAC 左右通道输出归零选择
 * 
 * @param num_8388 8388编号
 * @param zero_l 左声道数据归零选择
 * @param zero_r 右声道数据归零选择
 * @return int32_t 正确
 */
int32_t ES8388_DAC_output_zero_config(es8388_num_sel_t num_8388,uint32_t zero_l,uint32_t zero_r)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL7);
    reg_data &= ~(3<<6);
    reg_data |= zero_l | zero_r;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL7,reg_data);
    return RETURN_OK;
}


#define DAC_NO_MERGE     (0<<5)
#define DAC_L_R_MERGE    (1<<5)
/**
 * @brief 左右通道是否merge（如果merge，左右通道的和的一半通过左右通道输出）
 * 
 * @param num_8388 8388编号
 * @param merge_en 开关
 * @return int32_t 正确
 */
int32_t ES8388_DAC_mono_config(es8388_num_sel_t num_8388,uint32_t merge_en)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL7);
    reg_data &= ~(1<<5);
    reg_data |= merge_en;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL7,reg_data);
    return RETURN_OK;
}


//SE strength（SE有三种可能的意思：单端传输、音效、音能，这里最有可能是音能）
#define DAC_SE_STRENGTH_0        (0<<2)
#define DAC_SE_STRENGTH_1        (1<<2)
#define DAC_SE_STRENGTH_2        (2<<2)
#define DAC_SE_STRENGTH_3        (3<<2)
#define DAC_SE_STRENGTH_4        (4<<2)
#define DAC_SE_STRENGTH_5        (5<<2)
#define DAC_SE_STRENGTH_6        (6<<2)
#define DAC_SE_STRENGTH_7        (7<<2)

//VPP_Scale DAC输出的幅值大小
#define DAC_VPP_3_5V     (0<<0)
#define DAC_VPP_4V       (1<<0)
#define DAC_VPP_3V       (2<<0)
#define DAC_VPP_2_5V     (3<<0)
/**
 * @brief 设置8388 DAC 音能和Vpp幅值上限设置
 * 
 * @param num_8388 8388编号
 * @param se 8个SE_STRENGTH的宏定义
 * @param vpp 基准电压设置
 * @return int32_t 正确
 */
int32_t ES8388_DAC_SE_add_vpp_config(es8388_num_sel_t num_8388,uint32_t se,uint32_t vpp)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL7);
    reg_data &= ~(0x1f<<0);
    reg_data |= se | vpp;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL7,reg_data);
    return RETURN_OK;
}


//INPUT select for output mix
#define LMIXSEL_LIN1     (0<<3)
#define LMIXSEL_LIN2     (1<<3)
#define LMIXSEL_LEFT_ADC (3<<3)

#define RMIXSEL_RIN1         (0<<0)
#define RMIXSEL_RIN2         (1<<0)
#define RMIXSEL_RIGHT_ADC    (3<<0)
/**
 * @brief 左右通道mix选择
 * 
 * @param num_8388 8388编号
 * @param l_sel 左通道数据选择
 * @param r_sel 右通道数据选择
 * @return int32_t 正确
 */
int32_t ES8388_mixsel(es8388_num_sel_t num_8388,uint32_t l_sel,uint32_t r_sel)
{
    volatile uint32_t reg_data = 0;
    reg_data = l_sel | r_sel;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL16,reg_data);
    return RETURN_OK;
}


/*Register39 DAC Control17*/
//left mixer信号选择
#define L_DAC_2_L_MIXER_DISABLE     (0<<7)
#define L_DAC_2_L_MIXER_ENABLE      (1<<7)

#define LIN_2_L_MIXER_DISABLE       (0<<6)
#define LIN_2_L_MIXER_ENABLE        (1<<6)

#define LIN_2_L_MIXER_GAIN_6dB      (0<<3)
#define LIN_2_L_MIXER_GAIN_3dB      (1<<3)
#define LIN_2_L_MIXER_GAIN_0dB      (2<<3)
#define LIN_2_L_MIXER_GAIN__3dB     (3<<3)
#define LIN_2_L_MIXER_GAIN__6dB     (4<<3)
#define LIN_2_L_MIXER_GAIN__9dB     (5<<3)
#define LIN_2_L_MIXER_GAIN__12dB    (6<<3)
#define LIN_2_L_MIXER_GAIN__15dB    (7<<3)
/**
 * @brief 左通道mixer配置
 * 
 * @param num_8388 8388编号
 * @param ld2lo left DAC to left mixer开关
 * @param li2lo LIN signal to left mixer
 * @param li2lovol 增益大小
 * @return int32_t 正确
 */
int32_t ES8388_left_mixer_config(es8388_num_sel_t num_8388,uint32_t ld2lo,uint32_t li2lo,uint32_t li2lovol)
{
    volatile uint32_t reg_data = 0;
    reg_data = ld2lo | li2lo | li2lovol;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL17,reg_data);
    return RETURN_OK;
}


/*Register42 DAC Control20*/
//right mixer信号选择
#define R_DAC_2_R_MIXER_DISABLE     (0<<7)
#define R_DAC_2_R_MIXER_ENABLE      (1<<7)

#define RIN_2_R_MIXER_DISABLE       (0<<6)
#define RIN_2_R_MIXER_ENABLE        (1<<6)

#define RIN_2_R_MIXER_GAIN_6dB      (0<<3)
#define RIN_2_R_MIXER_GAIN_3dB      (1<<3)
#define RIN_2_R_MIXER_GAIN_0dB      (2<<3)
#define RIN_2_R_MIXER_GAIN__3dB     (3<<3)
#define RIN_2_R_MIXER_GAIN__6dB     (4<<3)
#define RIN_2_R_MIXER_GAIN__9dB     (5<<3)
#define RIN_2_R_MIXER_GAIN__12dB    (6<<3)
#define RIN_2_R_MIXER_GAIN__15dB    (7<<3)
/**
 * @brief 4通道mixer配置
 * 
 * @param num_8388 8388编号
 * @param rd2ro reft DAC to reft mixer开关
 * @param ri2ro rin signal to reft mixer
 * @param ri2rovol 增益大小
 * @return int32_t 正确
 */
int32_t ES8388_right_mixer_config(es8388_num_sel_t num_8388,uint32_t rd2ro,uint32_t ri2ro,uint32_t ri2rovol)
{
    uint32_t reg_data = 0;
    reg_data = rd2ro | ri2ro | ri2rovol;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL20,reg_data);
    return RETURN_OK;
}


/*Register43 DAC Control21*/
#define DAC_ADC_LRC_SEPARATE     (0<<7)
#define DAC_ADC_LRC_SAME         (1<<7)

//当master mode时，且DAC ADC 的LRC一样时
#define USE_DAC_LRCK     (0<<6)
#define USE_ADC_LRCK     (1<<6)

#define OFFSET_DISABLE   (0<<5)
#define OFFSET_ENABLE    (1<<5)

#define MCLK_INPUT_FROM_PAD_ENABLE       (0<<4)
#define MCLK_INPUT_FROM_PAD_DISABLE      (1<<4)

#define ADC_DLL_NORMOL       (0<<3)
#define ADC_DLL_POWER_DOWN   (1<<3)

#define DAC_DLL_NORMAL       (0<<2)
#define DAC_DLL_POWER_DOWN   (1<<2)
/**
 * @brief DAC的clk方面的一些配置
 * 
 * @param num_8388 8388编号
 * @param slrck lrck选择使用相同的还是不同的
 * @param lrck_sel 选择使用ADC或者DAC的LRCK
 * @param offset offset的开关
 * @param mclk MCLK的开关
 * @param adc_dll adc_dll的开关
 * @param dac_dll dac_dll的开关
 * @return int32_t 正确
 */
int32_t ES8388_dac_clk_config(es8388_num_sel_t num_8388,uint32_t slrck,uint32_t lrck_sel,
                                uint32_t offset,uint32_t mclk,uint32_t adc_dll,uint32_t dac_dll)
{
    volatile uint32_t reg_data = 0;
    reg_data = slrck | lrck_sel | offset | mclk | adc_dll | dac_dll;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL21,reg_data);
    return RETURN_OK;
}
/**
 * @brief offset大小的设置
 * 
 * @param num_8388 8388编号
 * @param offset 电压偏移值
 * @return int32_t 正确
 */
int32_t ES8388_dac_offset_config(es8388_num_sel_t num_8388,uint32_t offset)
{
    volatile uint32_t reg_data = 0;
    reg_data = offset & 0xff;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL22,reg_data);
    return RETURN_OK;
}


//1.5k VREF to analog output resistance OR 40k VREF to analog output resistance
#define VROI_1_5K    (0<<4)
#define VROI_40K     (1<<4)
/**
 * @brief VREF to analog output resistance电阻大小选择
 * 
 * @param num_8388 8388编号
 * @param vroi 电阻大小
 * @return int32_t 正确
 */
int32_t ES8388_vref_resistance_config(es8388_num_sel_t num_8388,uint32_t vroi)
{
    volatile uint32_t reg_data = 0;
    reg_data |= vroi;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL23,reg_data);
    return RETURN_OK;
}


/**
 * @brief DAC四个通道的声音大小的设置
 * 
 * @param num_8388 8388编号
 * @param l1vol 设置声音大小的范围为（-30dB - 3dB），需为整数
 * @param r1vol 设置声音大小的范围为（-30dB - 3dB），需为整数
 * @param l2vol 设置声音大小的范围为（-30dB - 3dB），需为整数
 * @param r2vol 设置声音大小的范围为（-30dB - 3dB），需为整数
 * @return int32_t 正确
 */
int32_t ES8388_dac_vol_set(es8388_num_sel_t num_8388,int32_t l1vol,int32_t r1vol,int32_t l2vol,int32_t r2vol)
{
    volatile uint32_t reg_data = 0;
    
    if(l1vol > 3)
    {
        l1vol = 3;
    }
    else if(l1vol < -30)
    {
        l1vol = -30;
    }
    
    if(r1vol > 3)
    {
        r1vol = 3;
    }
    else if(r1vol < -30)
    {
        r1vol = -30;
    }

    if(l2vol > 3)
    {
        l2vol = 3;
    }
    else if(l2vol < -30)
    {
        l2vol = -30;
    }
    
    if(r2vol > 3)
    {
        r2vol = 3;
    }
    else if(r2vol < -30)
    {
        r2vol = -30;
    }
    
    
    reg_data = l1vol + 30;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL24,reg_data);

    reg_data = 0;
    reg_data = r1vol + 30;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL25,reg_data);

    reg_data = 0;
    reg_data = l2vol + 30;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL26,reg_data);

    reg_data = 0;
    reg_data = r2vol + 30;
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL27,reg_data);
    return RETURN_OK;
}


/**
 * @brief 根据8388的编号初始化8388
 * 
 * @param num_8388 8388编号
 * @return int32_t 正确
 */
int32_t ES8388_play_init(es8388_num_sel_t num_8388)
{
    es8388_alc_config_t alc_config_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&alc_config_str,0,sizeof(es8388_alc_config_t));
    es8388_input_cha_t input_cha_str = {0};
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&input_cha_str,0,sizeof(es8388_input_cha_t));
    uint32_t regnum = 0;
    unsigned char readbuf[2];
    unsigned char cnt_reinit=0;
    mprintf("\n enter 8388 init \n");
ES8388_REINIT:
    es8388_i2c_init(num_8388);

    I2C_WriteEs8388Data(num_8388,0x0,0X86);
    _delay_10us_240M(10);
    I2C_WriteEs8388Data(num_8388,0x0,0X06);
    _delay_10us_240M(10);

    
    I2C_WriteEs8388Data(num_8388,ES8388_DACPOWER,0xc0);//power down DAC
    ES8388_mastermode_config(num_8388,ES8388_MODE_SLAVE);
    
    I2C_WriteEs8388Data(num_8388,0x19,0x26);//mute DAC
    
    I2C_WriteEs8388Data(num_8388,0x2E,0X0);//LOUT1VOL
    I2C_WriteEs8388Data(num_8388,0x2F,0x0);//ROUT1VOL
    I2C_WriteEs8388Data(num_8388,0x30,0x0);//LOUT2VOL
    I2C_WriteEs8388Data(num_8388,0x31,0x0);//ROUT2VOL
    
    I2C_WriteEs8388Data(num_8388,ES8388_CHIPPOWER,0xF3);

    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL21,0x80);//bit7-1:DACLRC & ADCLRC same ,bit6 must be 0
    I2C_WriteEs8388Data(num_8388,ES8388_CONTROL1,0x16);

    I2C_WriteEs8388Data(num_8388,ES8388_CONTROL2,0x72);
    
    I2C_WriteEs8388Data(num_8388,ES8388_ANAVOLMANAG,0x7C);

    ES8388_ADC_mic_amplify_set(num_8388,ES8388_MICAMP_33dB,ES8388_MICAMP_33dB);
    
#if 0//左通道的MIC复制为两个声道的数据
    input_cha_str.l_input_single_sel = DIFFERENTIAL_L_CHA;
    input_cha_str.r_input_single_sel = DIFFERENTIAL_R_CHA;
    input_cha_str.ds_sel = USE_ONE_DSL_REG11_BIT7;
    input_cha_str.dsr = DSR_LINPUT1_RINPUT1;
    input_cha_str.dsl = DSL_LINPUT1_RINPUT1;
    input_cha_str.monomix = MONOMIX_STEREO;
    input_cha_str.tri = ASDOUT_IS_ADC_NORMAL_OUTPUT;
#endif
    
#if 1//双通道MIC的数据独立出来
    input_cha_str.l_input_single_sel = DIFFERENTIAL_L_CHA;
    input_cha_str.r_input_single_sel = DIFFERENTIAL_R_CHA;
    input_cha_str.ds_sel = USE_DSL_REG11_BIT7_DSR_REG10_BIT2;//USE_DSL_REG11_BIT7_DSR_REG10_BIT2;//USE_ONE_DSL_REG11_BIT7;
    input_cha_str.dsr = DSR_LINPUT1_RINPUT1;
    input_cha_str.dsl = DSL_LINPUT2_RINPUT2;
    input_cha_str.monomix = MONOMIX_STEREO;
    input_cha_str.tri = ASDOUT_IS_ADC_NORMAL_OUTPUT;
#endif
    ES8388_input_channal_config(num_8388,&input_cha_str);
    
    ES8388_ADC_data_format_set(num_8388,ES8388_ADC_DATA_LEN_16BIT,ES8388_ADC_DATA_FORMAT_I2S);//ES8388_ADC_DATA_FORMAT_I2S);
    
    I2C_WriteEs8388Data(num_8388,ES8388_ADCPOWER,0x00);//打开ADC电源
    
    I2C_WriteEs8388Data(num_8388,ES8388_ADCCONTROL5,0x03);//ADC 过采样率/* 设置过采样率384 FS*/
    
    ES8388_ALC_config(num_8388,&alc_config_str,ES8388_ALC_STEREO);

    alc_config_str.alc_maxgain = ES8388_ALC_MAXGAIN_17_5dB;
    alc_config_str.alc_mingain = ES8388_ALC_MINGAIN_0dB;
    alc_config_str.alc_level = ES8388_ALC_LEVEL__3dB;
    ES8388_ALC_config(num_8388,&alc_config_str,ES8388_ALC_STEREO);
	
    ES8388_noise_gate_config(num_8388,-61.5f,MUTE_ADC_OUTPUT,NOISE_GATE_DISABLE);
    //ES8388_ADC_HPF_config(num_8388,DISABLE,DISABLE);
    #if 0
    ES8388_DAC_data_format_set(num_8388,ES8388_DAC_DATA_LEN_16BIT,ES8388_DAC_DATA_FORMAT_I2S);

    I2C_WriteEs8388Data(num_8388,0x18,0x02);//DAC 过采样率
//Default Mute on
//    I2C_WriteEs8388Data(num_8388,0x19,0x24);// 0X22 bit2 auto mute close---register 涓婇潰娌℃爣娉?
//Default mute off    
    //I2C_WriteEs8388Data(num_8388,0x19,0x22);//bit2-1:mute;//no mute DAC
	I2C_WriteEs8388Data(num_8388,0x19,0x26);//mute DAC

    ES8388_DAC_vol_config(num_8388,0,0);

    I2C_WriteEs8388Data(num_8388,0x26,0x00);//L & R MIX SEL
    I2C_WriteEs8388Data(num_8388,0x27,0xb8);//L MIX
    I2C_WriteEs8388Data(num_8388,0x2A,0xb8);//R MIX
    #endif

    ES8388_ADC_invl_config(num_8388,ES8388_ADC_INV_DISABLE,ES8388_ADC_INV_ENABLE);
    ES8388_dac_vol_set(num_8388,-30,-30,-30,-30);
    I2C_WriteEs8388Data(num_8388,ES8388_LADC_VOL,0x00);//reg16，LADC，数字增益
    I2C_WriteEs8388Data(num_8388,ES8388_RADC_VOL,0x00);//reg17，RADC，数字增益

    //I2C_WriteEs8388Data(num_8388,ES8388_DACPOWER,0x3C);//reg04，ADC和DAC的power，打开DAc的power
	//I2C_WriteEs8388Data(num_8388,ES8388_DACPOWER,0xfC);//reg04，ADC和DAC的power，关闭DAc的power
    I2C_WriteEs8388Data(num_8388,ES8388_CHIPPOWER,0x00);//reg2，
    
//-----check 8388 init success or not    
    readbuf[0]=I2C_ReadEs8388Data(num_8388,0);
    readbuf[1]=I2C_ReadEs8388Data(num_8388,1);
    if((readbuf[0]==0xff)&& (readbuf[1]==0xff) && cnt_reinit < 10)
    {
        mprintf("\r\n 8388 init err retry%d times....\r\n\r\n",cnt_reinit++);
        _delay_10us_240M(50000);
        goto ES8388_REINIT;
    }
    mprintf("8388 reg0x17=%d\r\n",I2C_ReadEs8388Data(num_8388,0x17));
    if(cnt_reinit == 10)
    {
        mprintf("\r\n 8388 init err ,then while(1)....\r\n\r\n");
        while(1)
        {
            _delay_10us_240M(5000);
        }
    }
    
    for(regnum=0;regnum<53;regnum++)
    {
        //mprintf("%d's data is 0x%x\n",regnum,I2C_ReadEs8388Data(num_8388,regnum));
    }
    return 0;
}    


/**
 * @brief 各个通道DAC电源的开关
 * 
 * @param num_8388 8388编号
 * @param cha DAc通道选择
 * @param cmd 开关
 */
void ES8388_DAC_power(es8388_num_sel_t num_8388,es8388_cha_sel_t cha,FunctionalState cmd)
{
    volatile uint32_t reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACPOWER);
    reg_data &= ~(3 << 6);
    if(DISABLE == cmd)
    {
        if(ES8388_CHA_STEREO == cha)
        {
            reg_data |= (3 << 6);
        }
		else
		{
			reg_data |= (1 << (6+cha));
		}
        
    }
    I2C_WriteEs8388Data(num_8388,ES8388_DACPOWER,reg_data);
}


/**
 * @brief DAC mute开关
 * 
 * @param num_8388 8388编号
 * @param cmd 开关
 */
void ES8388_DAC_mute(es8388_num_sel_t num_8388,FunctionalState cmd)
{
    volatile uint32_t reg_data = I2C_ReadEs8388Data(num_8388,ES8388_DACCONTROL3);
    reg_data &= ~(1 << 2);
    reg_data |= (cmd << 2);
    I2C_WriteEs8388Data(num_8388,ES8388_DACCONTROL3,reg_data);
}



/**
 * @brief 在AEC的情况下8388的配置（参照一代工程）
 * 
 * @param num_8388 8388编号
 */
void ES8388_aec_setting(es8388_num_sel_t num_8388)
{
    es8388_input_cha_t input_cha_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&input_cha_str,0,sizeof(es8388_input_cha_t));

    es8388_alc_config_t alc_config_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&alc_config_str,0,sizeof(es8388_alc_config_t));
    
    ES8388_ADC_mic_amplify_set(num_8388,ES8388_MICAMP_0dB,ES8388_MICAMP_21dB);

    input_cha_str.l_input_single_sel = DIFFERENTIAL_L_CHA;
    input_cha_str.r_input_single_sel = DIFFERENTIAL_R_CHA;
    input_cha_str.ds_sel = USE_DSL_REG11_BIT7_DSR_REG10_BIT2;
    input_cha_str.dsr = DSR_LINPUT1_RINPUT1;
    input_cha_str.dsl = DSL_LINPUT2_RINPUT2;
    input_cha_str.monomix = MONOMIX_STEREO;
    input_cha_str.tri = ASDOUT_IS_ADC_NORMAL_OUTPUT;
    ES8388_input_channal_config(num_8388,&input_cha_str);//ADC input setting，两个通道的输入是独立的

    alc_config_str.alc_maxgain = ES8388_ALC_MAXGAIN_5_5dB;
    alc_config_str.alc_mingain = ES8388_ALC_MINGAIN__12dB;
    alc_config_str.alc_level = ES8388_ALC_LEVEL__16_5dB;
    ES8388_ALC_config(num_8388,&alc_config_str,ES8388_ALC_R_ONLY);//ALC只开启一个通道的

    ES8388_noise_gate_config(num_8388,-60,PGA_GAIN_HELD_CONSTANT,NOISE_GATE_ENABLE);//噪声门限还是打开了的
}






