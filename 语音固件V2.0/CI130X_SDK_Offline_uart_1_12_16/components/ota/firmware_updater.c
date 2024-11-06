/**
  ******************************************************************************
  * @文件    main.c
  * @版本    V1.0.0
  * @日期    2018-5-22
  * @概要    这个文件是chipintelli公司的CI130X芯片程序的主函数.
  ******************************************************************************
  * @注意
  *
  * 版权归chipintelli公司所有，未经允许不得使用或修改
  *
  ******************************************************************************
  */

#include <stdint.h>
#include <string.h>
#include "ci130x_scu.h"
#include "ci130x_dma.h"
#include "ci_log.h"
#include "ci130x_uart.h"
#include "ci130x_spiflash.h"
#include "ci130x_timer.h"
#include "ci130x_system.h"
#include "ci130x_core_eclic.h"
#include "sdk_default_config.h"
#include "stdlib.h"
#include "FreeRTOS.h"

#include "flash_update.h"
#include "platform_config.h"
#include "ci130x_spiflash.h"


/**************************************************************************
                    type define
****************************************************************************/
 
typedef enum
{
    UPDATE_REQTIMEOUT = 0,
    UPDATE_UPDATING,
    UPDATE_UPDATECOMPLETE,
}Update_State_t;


/**************************************************************************
                    extern
****************************************************************************/
extern FileConfig_Struct FileConfigInfo;

/**************************************************************************
                    global
****************************************************************************/
static uint32_t Update_state;

void updater_uart_handler(void)
{
    if(UART_MaskIntState(OTA_UPDATE_PORT,UART_RXInt))
    {
        while(0 == (OTA_UPDATE_PORT->UARTFlag & (0x1 << 6)))
        {
            receive_func(UART_RXDATA(OTA_UPDATE_PORT));
        }
    }
    UART_IntClear(OTA_UPDATE_PORT,UART_AllInt);
}


void platform_init(uint32_t baudrate)
{
    /* 升级串口 */
    UARTPollingConfig(OTA_UPDATE_PORT,(UART_BaudRate)baudrate);/*update port*/
    if (OTA_UPDATE_PORT == UART0)
    {
    	__eclic_irq_set_vector(UART0_IRQn,(int)updater_uart_handler);
    }
    else if (OTA_UPDATE_PORT == UART1)
    {
    	__eclic_irq_set_vector(UART1_IRQn,(int)updater_uart_handler);
    }
    UARTInterruptConfig(OTA_UPDATE_PORT,(UART_BaudRate)baudrate);

    /* QFLASH0 初始化*/
    ////scu_set_device_reset(HAL_QSPI0_BASE);
    ////scu_set_device_reset_release(HAL_QSPI0_BASE);
    flash_init(QSPI0);
    spic_protect(QSPI0,DISABLE);
}



/**
  * @功能:主函数，进入应用程序的入口
  * @注意:无
  * @参数:无
  * @返回值:无
  */
int firmware_update_main(uint32_t baudrate)
{
    mprintf("Enter update mode\n");
    //vTaskDelay(pdMS_TO_TICKS(100));
        
    
    /* 平台相关初始化 */
    platform_init(baudrate);
    flash_update_buf_init();
    while(1)
    {
        if(RETURN_OK == have_a_new_message())// TODO: need modify
        {
            Resolution_func(); 
        }
        else
        {
            switch(Update_state)
            {
                case UPDATE_REQTIMEOUT:                    
                    break;

                case UPDATE_UPDATING:                  
                    if(RETURN_OK == get_update_state())
                        Update_state = UPDATE_UPDATECOMPLETE;
                    break;

                case UPDATE_UPDATECOMPLETE:
                    {
                        //uint32_t UserCodeStartAddr = 0;
                        //uint32_t UserCodeSize = 0;
                        
                        //优先从最近一次更新成功的User code分区加载
                        //if (FileConfigInfo.UserCode1CompltStatus == USER_CODE_AREA_STA_OK)
                        //{
                        //    UserCodeStartAddr = FileConfigInfo.UserCode1StartAddr;
                        //    UserCodeSize = FileConfigInfo.UserCode1Size;
                        //}
                        //else if (FileConfigInfo.UserCode2CompltStatus == USER_CODE_AREA_STA_OK)
                        //{
                        //    UserCodeStartAddr = FileConfigInfo.UserCode2StartAddr;
                        //    UserCodeSize = FileConfigInfo.UserCode2Size;
                        //}
                    }
                    break;
                    
                default:
                    break;

            }
        }  
    }
}


