#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "ci130x_iic.h"          /*包含I2C寄存器、基地址相关定义*/
#include "ci130x_system.h"       /*包含I2C相关接口定义*/
#include "platform_config.h"     /*包含引脚选择接口定义i2c_io_init*/
#include "string.h"              /*包含strcpy、memcpy、strncpy接口定义*/
#include "ci_log.h"
#include "sdk_default_config.h"

//#include "ci130x_iis.h"
// #include "ci130x_audio_pre_rslt_out.h"
#define ES7243_I2C_ADDR         (0x13)    /*SLAVE设备IIC总线地址*/

/*寄存器地址宏定义*/
#define ES7243_REG00            (0x00)
#define ES7243_REG01            (0x01)
#define ES7243_REG02            (0x02)
#define ES7243_REG03            (0x03)
#define ES7243_REG04            (0x04)
#define ES7243_REG05            (0x05)
#define ES7243_REG06            (0x06)
#define ES7243_REG07            (0x07)
#define ES7243_REG08            (0x08)
#define ES7243_REG09            (0x09)
#define ES7243_REG10            (0x0A)
#define ES7243_REG11            (0x0B)
#define ES7243_REG12            (0x0C)
#define ES7243_REG13            (0x0D)
#define ES7243_REG14            (0x0E)


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
 * @brief 为7243写入一个字节数据
 * 
 * @param regaddr 7243寄存器地址
 * @param val 需要写入的值
 */
static void I2C_WriteEs7243Data( unsigned int regaddr,char val)
{
    char buf[2] = {0};
    buf[0] = regaddr;
    buf[1] = val;

    i2c_master_only_send(ES7243_I2C_ADDR,buf,2);
//    _delay_10us_240M(5000);
}   

/**
 * @brief 读取7243某个寄存器的值
 * 
 * @param regaddr 7243寄存器地址
 * @return int 读出寄存器的值
 */
static int I2C_ReadEs7243Data(unsigned int regaddr)
{
    char buf[2] = {0};
    buf[0] = regaddr;

    i2c_master_send_recv(ES7243_I2C_ADDR,buf,1,1);     
    //_delay_10us_240M(5);
    return (unsigned char)buf[0];   
}


// /**
//  * @brief 设置7243 复位
//  * 
//  * @return int32_t 正确
//  */
// int32_t ES7243_reset(void)
// {
//     volatile uint32_t reg_data = 0;
//     reg_data |= (1<<8);
//     I2C_WriteEs7243Data(ES7243_CONTROL1,reg_data);
//     return RETURN_OK;
// }



// /**
//  * @brief 设置7243 解除复位
//  * 
//  * @return int32_t 正确
//  */
// int32_t ES7243_reset_release(void)
// {
//     volatile uint32_t reg_data = 0;
//     reg_data &= ~(1<<8);
//     I2C_WriteEs7243Data(num_7243,ES7243_CONTROL1,reg_data);
//     return RETURN_OK;
// }
#define PAG_GAIN_1DB        (0x10)
#define PAG_GAIN_3_5DB      (0x12)
#define PAG_GAIN_18DB       (0x20)
#define PAG_GAIN_20_5DB     (0x22)
#define PAG_GAIN_22_5DB     (0x04)
#define PAG_GAIN_24_5DB     (0x40)
#define PAG_GAIN_25DB       (0x06)
#define PAG_GAIN_27DB       (0x42)
#define DIFFERENTIAL_INPUT  (0x01)

void es7243_pga_gain(uint8_t gain)
{
    volatile uint8_t reg_data = 0;
    reg_data = gain|0x01;
    I2C_WriteEs7243Data(ES7243_REG08,reg_data);

}
void es7243_i2c_init(void)
{
    /*硬件引脚初始化，引脚选用请看plaform_config.c文件i2c_io_init接口注释部分**/
    i2c_io_init(UART2_RX_PAD,UART2_TX_PAD);
    /*master模式初始化：100 Kbit/s，timeout 0X5FFFFF*/
    i2c_master_init(IIC1,100,0X5FFFFF);
}


static int reg_data = 0;
void es7243_test(void *p_arg)
{
    es7243_i2c_init();

    I2C_WriteEs7243Data(ES7243_REG00,0x01);  //slave mode,software mode,single speed mode,mclk div = 1
    I2C_WriteEs7243Data(ES7243_REG06,0x00);  //Reset ADC
    I2C_WriteEs7243Data(ES7243_REG05,0x1B);  //Mute ADC
    // I2C_WriteEs7243Data
    I2C_WriteEs7243Data(ES7243_REG01,0x0C);  //I2S -16BIT
    es7243_pga_gain(PAG_GAIN_22_5DB);
//    uint8_t data = I2C_ReadEs7243Data(ES7243_REG08);
//    mprintf("data = %d\n",data);
    // I2C_WriteEs7243Data(ES7243_REG06,0x00);  //UnRESET ADC
    I2C_WriteEs7243Data(ES7243_REG05,0x013); //UnMute ADC


    while(1)
    {
        _delay_10us_240M(5000);
    }
}

void es7243_aec_hw_init_back(void)
{
    es7243_i2c_init();

    I2C_WriteEs7243Data(ES7243_REG00,0x01);  //slave mode,software mode,single speed mode,mclk div = 1
    I2C_WriteEs7243Data(ES7243_REG06,0x00);  //Reset ADC
    I2C_WriteEs7243Data(ES7243_REG05,0x1B);  //Mute ADC
    // I2C_WriteEs7243Data
    //I2C_WriteEs7243Data(ES7243_REG01,0x0C);  //I2S -16BIT
    /*反转极性*/
    I2C_WriteEs7243Data(ES7243_REG01,0x4c);
    es7243_pga_gain(PAG_GAIN_1DB);
    uint8_t data = I2C_ReadEs7243Data(ES7243_REG01);
    mprintf("data = %d\n",data);
    // I2C_WriteEs7243Data(ES7243_REG06,0x00);  //UnRESET ADC
    I2C_WriteEs7243Data(ES7243_REG05,0x013); //UnMute ADC
}

void es7243_aec_hw_init(void)
{
    es7243_i2c_init();
    I2C_WriteEs7243Data(0x00, 0x01 ); //software mode
    I2C_WriteEs7243Data(0x06, 0x18 ); //reset all digital logic,reset adc logic
    mprintf("0x06= %x \n",I2C_ReadEs7243Data(0x06));
    I2C_WriteEs7243Data(0x07, 0x20 ); //adcvref generate power down
    mprintf("0x07= %x \n",I2C_ReadEs7243Data(0x07));
    I2C_WriteEs7243Data(0x09, 0x80 );//power down whole analog
    mprintf("0x09= %x \n",I2C_ReadEs7243Data(0x09));
    I2C_WriteEs7243Data(0x01, 0x2c);//16bit,i2s
    //I2C_WriteEs7243Data(0x01, 0x2d);//16bit,lj
    mprintf("0x01= %x \n",I2C_ReadEs7243Data(0x01));
    I2C_WriteEs7243Data(0x02, 0x10 ); //LRCK DIVIDER
    mprintf("0x02= %x \n",I2C_ReadEs7243Data(0x02));
    #if USE_I2S_INTERFACE_SCK_LRCK_32
    I2C_WriteEs7243Data(0x03, 0x08 ); //BCLK DIVIDER 8:SCK/LRCK = 32
    #else
    I2C_WriteEs7243Data(0x03, 0x04 ); //BCLK DIVIDER 8:SCK/LRCK = 32
    #endif
    mprintf("0x03= %x \n",I2C_ReadEs7243Data(0x03));
    I2C_WriteEs7243Data(0x04, 0x02 ); //CLOCK DIVIDER FOR ADC
    mprintf("0x04= %x \n",I2C_ReadEs7243Data(0x04));
    //I2C_WriteEs7243Data(0x05, 0x1a );  // 0x11
    I2C_WriteEs7243Data(0x05, 0x13 );
    mprintf("0x05= %x \n",I2C_ReadEs7243Data(0x05));
    I2C_WriteEs7243Data(0x09, 0x3F ); 
    mprintf("0x09= %x \n",I2C_ReadEs7243Data(0x09));
    I2C_WriteEs7243Data(0x07, 0x80 );
    mprintf("0x07= %x \n",I2C_ReadEs7243Data(0x07));
    //I2C_WriteEs7243Data(0x08, 0x11 );
    //I2C_WriteEs7243Data(0x08, 0x13 );//0x11:1db 0x13:3.5db 
    es7243_pga_gain( PAG_GAIN_3_5DB );
    mprintf("0x08= %x \n",I2C_ReadEs7243Data(0x08));
    I2C_WriteEs7243Data(0x06, 0x00 );   
    mprintf("0x06= %x \n",I2C_ReadEs7243Data(0x06));
    I2C_WriteEs7243Data(0x05, 0x01 ); 
    mprintf("0x05= %x \n",I2C_ReadEs7243Data(0x05));
    mprintf("0x00= %x \n",I2C_ReadEs7243Data(0x00));
}

