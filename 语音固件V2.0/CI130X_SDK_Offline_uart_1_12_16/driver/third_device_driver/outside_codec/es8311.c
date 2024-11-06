
/**
 * @file es8311.c
 * @brief 为ES8311封装的驱动程序
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
#include "es8311.h"
#include "sdk_default_config.h"
#include <stdint.h>
#include <string.h>
#include "ci_log.h"
#include "ci130x_system.h"
#include "platform_config.h"


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


static int8_t sg_8311_i2c_init = 0;
#define MAX_DAC_REAL_GAIN (0.0f)//最大音量限制，防止喇叭破音，可以自己根据项目调整，最大可为30dB
#define MIN_DAC_REAL_GAIN (-50.0f)//最小音量限制，音量太小，已无感知，最小可为-95.5dB


/**
 * @brief es8311使用的I2C初始化
 * 
 * @param num_8311 8311的编号
 */
void es8311_i2c_init(void)
{
    i2c_io_init(AIN0_PAD,AIN1_PAD);
    i2c_master_init(IIC0,100,SHORT_TIME_OUT);
    sg_8311_i2c_init = 1;
}


/**
 * @brief 为8311写入一个字节数据
 * 
 * @param num_8311 8311编号
 * @param regaddr 8311寄存器地址
 * @param val 需要写入的值
 */
static void I2C_WriteEs8311Data(unsigned int regaddr,char val)
{
    if(0 == sg_8311_i2c_init)
    {
        ci_logerr(0,"iic is not inited\n");
    }
    char buf[2] = {0};
    buf[0] = regaddr;
    buf[1] = val;

    _delay_10us_240M(500);
    i2c_master_only_send(0x18,buf,2);      //slave的IIC设备地址：0x18
}    


/**
 * @brief 读取8311某个寄存器的值
 * 
 * @param num_8311 8311编号
 * @param regaddr 8311寄存器地址
 * @return int 读出寄存器的值
 */
int I2C_ReadEs8311Data(unsigned int regaddr)
{
    if(0 == sg_8311_i2c_init)
    {
        ci_logerr(0,"iic is not inited\n");
    }

    char buf[2] = {0};
    buf[0] = regaddr;

    i2c_master_send_recv(0x18,buf,1,1);     //slave的IIC设备地址：0x18
    _delay_10us_240M(500);

    return (unsigned char)buf[0];   
}


void es8311_pga_gain(es8311_pga_gain_t gain)
{
    volatile uint32_t reg_data = I2C_ReadEs8311Data(0x14);
    reg_data &= ~(0xf);
    reg_data |= gain;
    I2C_WriteEs8311Data(0x14,reg_data);
}


void es8311_adc_gain_scale_up(es8311_adc_gain_scale_up_t scale_up)
{
    volatile uint32_t reg_data = I2C_ReadEs8311Data(0x16);
    reg_data &= ~(0x7);
    reg_data |= scale_up;
    I2C_WriteEs8311Data(0x16,reg_data);
}



typedef enum
{
    ES8311_ALC_WINSIZE_2 = 0,
    ES8311_ALC_WINSIZE_4,
    ES8311_ALC_WINSIZE_8,
    ES8311_ALC_WINSIZE_16,
    ES8311_ALC_WINSIZE_32,
    ES8311_ALC_WINSIZE_64,
    ES8311_ALC_WINSIZE_128,
    ES8311_ALC_WINSIZE_256,
    ES8311_ALC_WINSIZE_512,
    ES8311_ALC_WINSIZE_1024,
    ES8311_ALC_WINSIZE_2048,
    ES8311_ALC_WINSIZE_4096,
    ES8311_ALC_WINSIZE_8192,
    ES8311_ALC_WINSIZE_16284,
    ES8311_ALC_WINSIZE_32768,
    ES8311_ALC_WINSIZE_65536,
}es8311_alc_winsize_t;



/**
 * @brief 配置8311的ALC
 * 
 * @param num_8311 8311编号 
 * @param ALC_str ES8311_ALC_Config_TypeDef类型结构体指针
 * @param ALC_gate 左声道、右声道或者双声道的ALC开关
 * @return int32_t 正确
 */
int32_t ES8311_ALC_config(es8311_alc_config_t* ALC_str,es8311_alc_gate_t ALC_gate)
{
    volatile uint32_t reg_data = 0;
    uint32_t samples = 0;
    uint32_t ALC_peak_detector_windows_size = ES8311_ALC_WINSIZE_16; 

    float max_gain_f = ALC_str->max_gain;
    if(max_gain_f > 32.0f)
    {
        max_gain_f = 32.0f;
    }
    else if(max_gain_f < -95.5f)
    {
        max_gain_f = -95.5f;
    }
    uint32_t reg_max_gain = (int)((max_gain_f + 95.5f) * 2.0f);
    I2C_WriteEs8311Data(0x17,reg_max_gain);
    reg_data = (uint32_t)ALC_gate | (uint32_t)ALC_peak_detector_windows_size;
    I2C_WriteEs8311Data(0x18,reg_data);
    reg_data = 0;
    reg_data = (uint32_t)(ALC_str->alc_max_level << 4) | (uint32_t)ALC_str->alc_min_level;
    I2C_WriteEs8311Data(0x19,reg_data);
    I2C_WriteEs8311Data(0x1a,0x03);
    //这个设置相当于二阶高通滤波器，60Hz截止频率
    I2C_WriteEs8311Data(0x1b,0xe7);
    I2C_WriteEs8311Data(0x1c,0x67);

    //这个设置相当于关闭二阶高通滤波器
    // I2C_WriteEs8311Data(0x1b,0xe7);
    // I2C_WriteEs8311Data(0x1c,0x47);


    return RETURN_OK;
}


int8_t es8311_alc_gate_en(es8311_alc_gate_t ALC_gate)
{
    volatile uint32_t reg_data = I2C_ReadEs8311Data(0x18);
    reg_data &= ~(1<<7);
    reg_data |= ALC_gate;
    I2C_WriteEs8311Data(0x18,reg_data);
    
    return RETURN_OK;
}


int8_t es8311_auto_mute_set(es8311_auto_mute_config_t* str)
{
    volatile uint32_t reg_data = 0;
    reg_data = I2C_ReadEs8311Data(0x18);
    reg_data &= ~(1<<6);
    reg_data |= str->auto_mute_en << 6;
    I2C_WriteEs8311Data(0x18,reg_data);

    reg_data = (str->ws << 4) | str->ng;
    I2C_WriteEs8311Data(0x1a,reg_data);

    reg_data = I2C_ReadEs8311Data(0x1b);
    reg_data &= ~(7<<5);
    reg_data |= (str->vol << 5);
    
    ci_loginfo(LOG_8311_DEBUG,"read reg0x%02x data is:%02x\n",0x18,I2C_ReadEs8311Data(0x18));
    ci_loginfo(LOG_8311_DEBUG,"read reg0x%02x data is:%02x\n",0x1a,I2C_ReadEs8311Data(0x1a));
    ci_loginfo(LOG_8311_DEBUG,"read reg0x%02x data is:%02x\n",0x1b,I2C_ReadEs8311Data(0x1b));
    return RETURN_OK;
}


int8_t es8311_dac_vol_set(int8_t vol)
{
    if(vol > 100)
    {
        vol = 100;
    }
    
    if(vol < 0)
    {
        vol = 0;
    }
    float max_gain = MAX_DAC_REAL_GAIN;
    float min_gain = MIN_DAC_REAL_GAIN;
    float step = (max_gain - min_gain) / 100.0f;
    float real_vol = min_gain + (float)vol*step;

    real_vol = (real_vol + 95.5f)*2.0f;
    uint8_t reg_data = (uint8_t)real_vol;
    I2C_WriteEs8311Data(0x32,reg_data);
    
    uint8_t reg_addr = 0x32;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    return RETURN_OK;
}


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if 0
void es8311_init(void)
{
    es8311_i2c_init();

    #if 1//只有ADC的流程
    uint8_t reg_addr = 0;
    reg_addr = 0x45;
    I2C_WriteEs8311Data(0x45,0);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    reg_addr = 0x1;
    I2C_WriteEs8311Data(0x1,0x30);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));


    I2C_WriteEs8311Data(0x2,0x10);
    reg_addr = 0x2;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x2,0x00);
    reg_addr = 0x2;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x3,0x10);
    reg_addr = 0x3;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x16,0x24);
    es8311_adc_gain_scale_up(ES8311_ADC_GAIN_SCALE_UP_24dB);
    reg_addr = 0x16;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x4,0x10);
    reg_addr = 0x4;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x5,0x00);
    reg_addr = 0x5;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xB,0x0);
    reg_addr = 0xB;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xC,0x0);
    reg_addr = 0xC;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x10,0x3);
    reg_addr = 0x10;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x11,0x7B);
    reg_addr = 0x11;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x0,0x81);
    reg_addr = 0x0;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1,0x3A);
    reg_addr = 0x1;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x14,0x1A);//ADC PGA增益最大30dB
    // es8311_pga_gain(ES8311_PGA_GAIN_30dB);
    reg_addr = 0x14;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x12,0x0);
    reg_addr = 0x12;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x13,0x0);
    reg_addr = 0x13;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x9,0x0);//设置为24bit不影响
    reg_addr = 0x9;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xA,0x0);
    reg_addr = 0xA;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xD,0x0A);
    reg_addr = 0xD;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xE,0xA);
    reg_addr = 0xE;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xF,0xF9);
    reg_addr = 0xF;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x15,0x00);//soft ramp先设置为disable，再把ALC关闭，加快启动时间，然后再打开soft ramp和ALC
                                    //soft ramp在ALC动作的时候可以保证录音的失真很小
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    es8311_alc_config_t alc_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&alc_str,0,sizeof(es8311_alc_config_t));
    alc_str.max_gain = 15.0f;
    alc_str.alc_max_level = ES8311_ALC_LEVEL__6dB;
    alc_str.alc_min_level = ES8311_ALC_LEVEL__6_6dB;
    ES8311_ALC_config(&alc_str,ES8311_ALC_OFF);

    //I2C_WriteEs8311Data(0x18,0x83);
    reg_addr = 0x18;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    //I2C_WriteEs8311Data(0x19,0xFE);
    reg_addr = 0x19;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1a,0x03);
    reg_addr = 0x1a;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x1b,0xE7);
    reg_addr = 0x1b;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x1C,0x67);
    reg_addr = 0x1c;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x37,0x8);
    reg_addr = 0x37;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x44,0x08);
    reg_addr = 0x44;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x17,0xDD);//15dB
    reg_addr = 0x17;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    I2C_WriteEs8311Data(0x15,0x80);
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    es8311_alc_gate_en(ES8311_ALC_ON);
    reg_addr = 0x18;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    #else
    uint8_t reg_addr = 0;
    reg_addr = 0x45;
    I2C_WriteEs8311Data(reg_addr,0);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    reg_addr = 0x1;
    I2C_WriteEs8311Data(reg_addr,0x30);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));


    I2C_WriteEs8311Data(0x2,0x10);
    reg_addr = 0x2;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x2,0x00);
    reg_addr = 0x2;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x3,0x10);
    reg_addr = 0x3;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x16,0x24);
    reg_addr = 0x16;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x4,0x10);
    reg_addr = 0x4;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x5,0x00);
    reg_addr = 0x5;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xB,0x0);
    reg_addr = 0xB;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xC,0x0);
    reg_addr = 0xC;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x10,0x3);
    reg_addr = 0x10;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x11,0x7B);
    reg_addr = 0x11;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x0,0x81);
    reg_addr = 0x0;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1,0x3A);
    reg_addr = 0x1;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x14,0x1A);
    reg_addr = 0x14;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x12,0x0);
    reg_addr = 0x12;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x13,0x0);
    reg_addr = 0x13;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x9,0x0);
    reg_addr = 0x9;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xA,0x0);
    reg_addr = 0xA;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xD,0x0A);
    reg_addr = 0xD;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xE,0xA);
    reg_addr = 0xE;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xF,0xF9);
    reg_addr = 0xF;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x15,0x00);
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x18,0x83);
    reg_addr = 0x18;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x19,0xFE);
    reg_addr = 0x19;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1a,0x03);
    reg_addr = 0x1a;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1b,0xE7);
    reg_addr = 0x1b;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1C,0x67);
    reg_addr = 0x1c;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x37,0x8);
    reg_addr = 0x37;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x44,0x08);
    reg_addr = 0x44;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x17,0xDD);
    reg_addr = 0x17;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    es8311_alc_config_t alc_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&alc_str,0,sizeof(es8311_alc_config_t));
    alc_str.max_gain = 22.0f;
    alc_str.alc_max_level = ES8311_ALC_LEVEL__6dB;
    alc_str.alc_min_level = ES8311_ALC_LEVEL__7_2dB;
    ES8311_ALC_config(&alc_str,ES8311_ALC_OFF);
    
    I2C_WriteEs8311Data(0x15,0xA0);
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    ES8311_ALC_config(&alc_str,ES8311_ALC_ON);
    #endif
}
#endif


static int8_t es8211_init_flag = 0;
void es8311_init(void)
{
    if(es8211_init_flag > 0)
    {
        return;
    }
    uint8_t reg_addr = 0;
    
    reg_addr = 0x0;
    I2C_WriteEs8311Data(0x0,0x1f);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    reg_addr = 0x45;
    I2C_WriteEs8311Data(0x45,0);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    reg_addr = 0x1;
    I2C_WriteEs8311Data(0x1,0x30);
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    //低功耗流程，去掉这一步
    //I2C_WriteEs8311Data(0x2,0x10);
    //reg_addr = 0x2;
    //ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    I2C_WriteEs8311Data(0x2,0x00);
    reg_addr = 0x2;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x3,0x10);
    reg_addr = 0x3;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x16,0x24);
    es8311_adc_gain_scale_up(ES8311_ADC_GAIN_SCALE_UP_24dB);
    reg_addr = 0x16;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x4,0x10);
    reg_addr = 0x4;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x5,0x00);
    reg_addr = 0x5;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xB,0x0);
    reg_addr = 0xB;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xC,0x0);
    reg_addr = 0xC;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

//    I2C_WriteEs8311Data(0x10,0x3);
    I2C_WriteEs8311Data(0x10,0x1f);//低功耗从0x1f调到0xc，功耗反而增加
//    I2C_WriteEs8311Data(0x10,0x1f);
    reg_addr = 0x10;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

//    I2C_WriteEs8311Data(0x11,0x7B);
    I2C_WriteEs8311Data(0x11,0x7b);//低功耗从7f变到7b
    reg_addr = 0x11;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

//    I2C_WriteEs8311Data(0x0,0x81);
    I2C_WriteEs8311Data(0x0,0x80);
    reg_addr = 0x0;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    //I2C_WriteEs8311Data(0x0,0x00);
    ///reg_addr = 0x0;
    //ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    
    I2C_WriteEs8311Data(0xd,0x1);
    reg_addr = 0xd;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    
//    I2C_WriteEs8311Data(0x1,0x3A);
    I2C_WriteEs8311Data(0x1,0x3f);
    reg_addr = 0x1;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x14,0x1A);//ADC PGA增益最大30dB
    // es8311_pga_gain(ES8311_PGA_GAIN_30dB);
    reg_addr = 0x14;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x12,0x00);
    reg_addr = 0x12;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x13,0x0);//低功耗调整为0
//    I2C_WriteEs8311Data(0x13,0x10);
    reg_addr = 0x13;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x9,0x0);//设置为24bit不影响
    reg_addr = 0x9;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xA,0x0);
    reg_addr = 0xA;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

//    I2C_WriteEs8311Data(0xD,0x0A);
//    reg_addr = 0xD;
//    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xE,0xA);//低功耗调整为0xa
//    I2C_WriteEs8311Data(0xE,0x2);
    reg_addr = 0xE;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0xF,0xFF);//低功耗调整为ff
//    I2C_WriteEs8311Data(0xF,0x44);
    reg_addr = 0xF;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x15,0x00);//soft ramp先设置为disable，再把ALC关闭，加快启动时间，然后再打开soft ramp和ALC
                                    //soft ramp在ALC动作的时候可以保证录音的失真很小
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    es8311_alc_config_t alc_str;
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(&alc_str,0,sizeof(es8311_alc_config_t));
    alc_str.max_gain = 15.0f;
    alc_str.alc_max_level = ES8311_ALC_LEVEL__6dB;
    alc_str.alc_min_level = ES8311_ALC_LEVEL__6_6dB;
    ES8311_ALC_config(&alc_str,ES8311_ALC_OFF);

    //I2C_WriteEs8311Data(0x18,0x83);
    reg_addr = 0x18;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    //I2C_WriteEs8311Data(0x19,0xFE);
    reg_addr = 0x19;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    I2C_WriteEs8311Data(0x1a,0x03);
    reg_addr = 0x1a;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x1b,0xE7);
    reg_addr = 0x1b;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x1C,0x67);
    reg_addr = 0x1c;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

//    I2C_WriteEs8311Data(0x37,0x8);
    I2C_WriteEs8311Data(0x37,0x08);
    reg_addr = 0x37;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr)); 

    I2C_WriteEs8311Data(0x44,0x08);
    reg_addr = 0x44;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));

    // I2C_WriteEs8311Data(0x17,0xDD);//15dB
    reg_addr = 0x17;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    I2C_WriteEs8311Data(0x15,0x80);
    reg_addr = 0x15;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    es8311_alc_gate_en(ES8311_ALC_ON);
    reg_addr = 0x18;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    es8211_init_flag++;
    // I2C_WriteEs8311Data(0x32,0xBC);
    //es8311_dac_vol_set(0);
//    reg_addr = 0x32;
//    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
}


void es8311_dac_off(void)
{
    volatile uint32_t reg_data = I2C_ReadEs8311Data(0x1);
    reg_data &= ~(1<<0 | 1<<2);
    I2C_WriteEs8311Data(0x1,reg_data);
    uint8_t reg_addr = 0x1;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    reg_data = I2C_ReadEs8311Data(0xd);
    reg_data |= (1<<3);
    I2C_WriteEs8311Data(0xd,reg_data);
    reg_addr = 0xd;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    reg_data = I2C_ReadEs8311Data(0x12);
    reg_data |= (1<<1);
    reg_data &= ~(1<<0);
    I2C_WriteEs8311Data(0x12,reg_data);
    reg_addr = 0x12;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    ci_loginfo(LOG_8311_DEBUG,"dac off\n");
}


void es8311_dac_on(void)
{
    volatile uint32_t reg_data = I2C_ReadEs8311Data(0x1);
    reg_data |= (1<<0 | 1<<2);
    I2C_WriteEs8311Data(0x1,reg_data);
    uint8_t reg_addr = 0x1;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    reg_data = I2C_ReadEs8311Data(0xd);
    reg_data &= ~(1<<3);
    I2C_WriteEs8311Data(0xd,reg_data);
    reg_addr = 0xd;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    reg_data = I2C_ReadEs8311Data(0x12);
    reg_data &= ~(1<<1);
    reg_data |= (1<<0);
    I2C_WriteEs8311Data(0x12,reg_data);
    reg_addr = 0x12;
    ci_loginfo(LOG_8311_DEBUG,"%02x%02x\n",reg_addr,I2C_ReadEs8311Data(reg_addr));
    
    ci_loginfo(LOG_8311_DEBUG,"dac on\n");
}


