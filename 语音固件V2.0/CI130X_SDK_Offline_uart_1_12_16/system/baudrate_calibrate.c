
#include "sdk_default_config.h"
#include "ci_log.h"
#include "ci130x_uart.h"
#include "ci_log_config.h"
#include "ci130x_gpio.h"
#include "ci130x_dpmu.h"
#include "ci130x_timer.h"
#include "FreeRTOS.h"
#include "system_msg_deal.h"
#include "baudrate_calibrate.h"

#if UART_BAUDRATE_CALIBRATE


// static const uint8_t sync_req[] = {'B', 'C'};

static const float factors[] = {
    0.90, 0.92, 0.940, 0.960, 0.980, 1.0, 1.02, 1.04, 1.06, 1.08, 1.10 };
// static const float factors[] = {1.0, 1.015, 0.955, 0.97, 0.985, 1.0, 1.015, 1.03, 1.045};


// #define SYNC_REQ_LENGTH     (sizeof(sync_req) / sizeof(sync_req[0]))
#define FACTOR_NUM      (sizeof(factors)/sizeof(factors[0]))

#ifndef BAUD_CALIBRATE_MAX_WAIT_TIME
#define BAUD_CALIBRATE_MAX_WAIT_TIME        60
#endif

typedef enum{
    BC_IDLE,
    BC_START,
    BC_STOP,
    BC_SEND_REQ,
    BC_WAIT_ACK,
    BC_CHANGE_BAUDRATE,
    BC_CALC,
}status_t;

typedef struct {
    volatile status_t status;
    send_sync_req_func_t send_sync_req_func;
    int32_t factor_index;
    UART_TypeDef *UARTx; 
    uint32_t std_baudrate;
    uint32_t idle_count;
    uint32_t sync_interval;
    uint8_t test_flag[FACTOR_NUM];
    volatile uint8_t ack_flag;
    uint8_t old_baudrate_index;
    volatile uint8_t frist_sync;
    volatile uint8_t force_stop;
    volatile int8_t up_index;
    volatile int8_t down_index;
}bc_info_t;

volatile static bc_info_t bc_info;
static void task_baudrate_calibrate(void *p_arg);


void baudrate_calibrate_init(UART_TypeDef *UARTx, uint32_t baudrate, send_sync_req_func_t send_sync_req_func)
{
    bc_info.status = BC_IDLE;
    bc_info.std_baudrate = baudrate;
    bc_info.UARTx = UARTx;    
    bc_info.idle_count = 0;
    bc_info.old_baudrate_index = sizeof(factors)/sizeof(factors[0])/2;
    bc_info.send_sync_req_func = send_sync_req_func;
    bc_info.frist_sync = 1;    
    bc_info.force_stop = 0;
    bc_info.ack_flag = 0;
    bc_info.sync_interval = BAUDRATE_SYNC_PERIOD;
    xTaskCreate(task_baudrate_calibrate, "baudrate_sync", 196, NULL, 4, NULL);
}
void baudrate_calibrate_start(void)
{
    if (bc_info.status == BC_IDLE)
    {
        bc_info.status = BC_START;
    }
}

void baudrate_calibrate_stop(void)
{
    bc_info.force_stop = 1;
    vTaskDelay(pdMS_TO_TICKS(2));
    while(bc_info.status != BC_IDLE)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void baudrate_calibrate_wait_finish(void)
{
    while(bc_info.status != BC_IDLE)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// 语音协议模块收到同步ACK的时候，通过调用此接口告知已收到ACK。
void baudrate_calibrate_set_ack()
{
    bc_info.ack_flag = 1;
}


void task_baudrate_calibrate(void *p_arg)
{
    int wait_count = 0;
    vTaskDelay(pdMS_TO_TICKS(1));
    bc_info.status = BC_START;
    while(1)
    {
        switch (bc_info.status)
        {
        case BC_START:
            if (bc_info.status != BC_STOP)
            {
                bc_info.factor_index = -1;
                bc_info.status = BC_SEND_REQ;
                bc_info.up_index = bc_info.old_baudrate_index;
                bc_info.down_index = bc_info.old_baudrate_index;
                memset(bc_info.test_flag, 0, FACTOR_NUM*sizeof(bc_info.test_flag[0]));
                mprintf("send @ index:%d\n", bc_info.old_baudrate_index);
            }
            break;
        case BC_STOP:
            if (bc_info.force_stop)
            {
                bc_info.force_stop = 0;
                UARTInterruptConfig(bc_info.UARTx, bc_info.std_baudrate*factors[bc_info.old_baudrate_index]);
                UART_IntMaskConfig(bc_info.UARTx, UART_RXTimeoutInt, DISABLE);
                UART_IntMaskConfig(bc_info.UARTx, UART_RXInt, ENABLE);
                if (BAUDRATE_SYNC_PERIOD > 4000)
                {
                    bc_info.idle_count = (BAUDRATE_SYNC_PERIOD-1000)/10;
                }
            }
            else
            {
                bc_info.idle_count = 0;
            }
            bc_info.status = BC_IDLE;
            break;
        case BC_SEND_REQ:
            bc_info.ack_flag = 0;
            if (bc_info.send_sync_req_func)
            {
                bc_info.send_sync_req_func();
            }

            //发送成功，转入等待ACK的状态
            if (bc_info.status != BC_STOP)
            {
                bc_info.status = BC_WAIT_ACK;
                wait_count = 0;                
            }            
            break;
        case BC_WAIT_ACK:
            if (bc_info.ack_flag)
            {
                // 收到回应
                if (bc_info.factor_index == -1)
                {
                    // 当前波特率正确，不用调整。
                    bc_info.sync_interval = BAUDRATE_SYNC_PERIOD;
                    bc_info.status = BC_STOP;
                    if (bc_info.frist_sync)
                    {
                        bc_info.frist_sync = 0;
                        sys_power_on_hook();
                    }
                    mprintf("keep baudrate %d\n", (uint32_t)(bc_info.std_baudrate*factors[bc_info.old_baudrate_index]));
                }
                else
                {
                    bc_info.test_flag[bc_info.factor_index] = 1;
                    {
                        bc_info.status = BC_CALC;
                    }
                }
            }
            else
            {
                if (wait_count < BAUD_CALIBRATE_MAX_WAIT_TIME/6)
                {
                    vTaskDelay(pdMS_TO_TICKS(6));
                    wait_count++;
                }
                else
                {
                    bc_info.status = BC_CALC;
                }
            }
            break;
        case BC_CHANGE_BAUDRATE:
            if ((bc_info.up_index - bc_info.old_baudrate_index) > (bc_info.old_baudrate_index - bc_info.down_index))
            {
                if(bc_info.down_index > 0)
                {
                    bc_info.down_index--;
                    bc_info.factor_index = bc_info.down_index;
                }
                else if (bc_info.up_index < (FACTOR_NUM - 1))
                {
                    bc_info.up_index++;
                    bc_info.factor_index = bc_info.up_index;
                }
                else
                {
                    bc_info.status = BC_CALC;
                    break;
                }
            }
            else
            {
                if (bc_info.up_index < (FACTOR_NUM - 1))
                {
                    bc_info.up_index++;
                    bc_info.factor_index = bc_info.up_index;
                }
                else if (bc_info.down_index > 0)
                {
                    bc_info.down_index--;
                    bc_info.factor_index = bc_info.down_index;
                }
                else
                {
                    bc_info.status = BC_CALC;
                    break;
                }
            }
            UARTInterruptConfig(bc_info.UARTx, bc_info.std_baudrate*factors[bc_info.factor_index]);
			UART_IntMaskConfig(bc_info.UARTx, UART_RXInt, ENABLE);
            bc_info.status = BC_SEND_REQ;
            mprintf("send @ index:%d\n", bc_info.factor_index);
            break;
        case BC_CALC:
        {
            int low = 0;
            int high = FACTOR_NUM - 1;
            while(low < high)
            {
                if (bc_info.test_flag[low] == 0)
                {
                    low++;
                }
                else
                {
                    if(bc_info.test_flag[high] == 0)
                    {
                        high--;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if (high == low)
            {
                if (bc_info.down_index == 0 && bc_info.up_index == (FACTOR_NUM-1))   
                {
                    if (bc_info.test_flag[low] == 1)
                    {
                        bc_info.factor_index = low;
                    }
                    else
                    {
                        bc_info.factor_index = -1;
                    } 
                }
                else
                {
                    bc_info.status = BC_CHANGE_BAUDRATE;
                    break;
                }
            }
            else
            {
                if (high > bc_info.old_baudrate_index)
                {
                    bc_info.factor_index = high;
                }
                else
                {
                    bc_info.factor_index = low;
                }
            }
            
            if (bc_info.factor_index >= 0)
            {
                UARTInterruptConfig(bc_info.UARTx, bc_info.std_baudrate*factors[bc_info.factor_index]);
                bc_info.old_baudrate_index = bc_info.factor_index;
                bc_info.sync_interval = BAUDRATE_SYNC_PERIOD;
                mprintf("change baudrate to %d\n", (uint32_t)(bc_info.std_baudrate*factors[bc_info.factor_index]));
            }
            else
            {
                UARTInterruptConfig(bc_info.UARTx, bc_info.std_baudrate*factors[bc_info.old_baudrate_index]);
                mprintf("keep baudrate %d\n", (uint32_t)(bc_info.std_baudrate*factors[bc_info.old_baudrate_index]));
                bc_info.sync_interval = BAUDRATE_FAST_SYNC_PERIOD;
            }
            UART_IntMaskConfig(bc_info.UARTx, UART_RXTimeoutInt, DISABLE);
            UART_IntMaskConfig(bc_info.UARTx, UART_RXInt, ENABLE);
            if (bc_info.frist_sync)
            {
                bc_info.frist_sync = 0;
                sys_power_on_hook();
            }
            bc_info.status = BC_STOP;
            break;
        }
        case BC_IDLE:
        default:
            vTaskDelay(pdMS_TO_TICKS(10));
            if (++bc_info.idle_count > bc_info.sync_interval/10)
            {
                bc_info.idle_count = 0;
                if (get_wakeup_state() == SYS_STATE_UNWAKEUP)
                {
                    bc_info.status = BC_START;
                }
            }
            break;
        }

        if (bc_info.force_stop && (bc_info.status != BC_IDLE))
        {
            bc_info.status = BC_STOP;
        }
        else
        {
            bc_info.force_stop = 0;
        }
    }
}

#endif


