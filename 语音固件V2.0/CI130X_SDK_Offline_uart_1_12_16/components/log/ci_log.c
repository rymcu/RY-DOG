/**
 * @file ci_log.c
 * @brief sdk log
 * @version 0.1
 * @date 2019-01-09
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdarg.h>
#include <stdio.h>
#include "ci_log.h"

#include "ci130x_system.h"
#include "ci130x_uart.h"
#include "ci130x_lowpower.h"
#include "ci130x_core_eclic.h"
#include <stdbool.h>
#include "romlib_runtime.h"


// 日志模块初始化
void ci_log_init(void)
{
    #if CONFIG_CI_LOG_EN
    #if UART_LOG_UASED_INT_MODE
    UARTInterruptConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);
    #else
    UARTPollingConfig((UART_TypeDef*)CONFIG_CI_LOG_UART_PORT, UART_BaudRate921600);
    #endif  //UART_LOG_UASED_INT_MODE
    #endif  //CONFIG_CI_LOG_EN
}


#if UART_LOG_UASED_INT_MODE
typedef struct
{
    char *base_addr;
    uint32_t total_size;

    uint32_t wp;
    uint32_t rp;
    uint32_t data_cnt;
}uart_log_ring_buffer_t;

static uart_log_ring_buffer_t uart_log_buffer;

static char log_buff[UART_LOG_BUFF_SIZE];

static void uart_log_ring_buff_lock_init(void)
{

}

static inline void uart_log_ring_buff_lock(void)
{
    #if ((HAL_UART0_BASE) == (CONFIG_CI_LOG_UART))
    eclic_irq_disable(UART0_IRQn);
    #endif
    #if ((HAL_UART1_BASE) == (CONFIG_CI_LOG_UART))
    eclic_irq_disable(UART1_IRQn);
    #endif
}


static inline void uart_log_ring_buff_unlock(void)
{
    #if ((HAL_UART0_BASE) == (CONFIG_CI_LOG_UART))
    eclic_irq_enable(UART0_IRQn);
    #endif
    #if ((HAL_UART1_BASE) == (CONFIG_CI_LOG_UART))
    eclic_irq_enable(UART1_IRQn);
    #endif
}


void uart_log_ring_buffer_clear(uart_log_ring_buffer_t *ring_buf)
{
    ring_buf->data_cnt = 0;
    ring_buf->wp = 0;
    ring_buf->rp = 0;
}

void uart_log_ring_buffer_init(uart_log_ring_buffer_t *ring_buf,char *base_addr,uint32_t total_size)
{
    ring_buf->base_addr = base_addr;
    ring_buf->total_size = total_size;
    uart_log_ring_buffer_clear(ring_buf);
    uart_log_ring_buff_lock_init();
}


static inline bool uart_log_ring_is_full(uart_log_ring_buffer_t *ring_buf)
{
    if(ring_buf->data_cnt >= ring_buf->total_size)
    {
        return true;
    }
    else
    {
        return false;
    }
}


static inline bool uart_log_ring_is_empty(uart_log_ring_buffer_t *ring_buf)
{
    if(ring_buf->data_cnt <= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}


int32_t uart_log_ring_buffer_write(uart_log_ring_buffer_t *ring_buf,uint8_t ch)
{
    int32_t ret;
    uart_log_ring_buff_lock();

    if(uart_log_ring_is_full(ring_buf))
    {
        ret = RETURN_ERR;
    }
    else
    {
        ring_buf->base_addr[ring_buf->wp] = ch;
        ring_buf->wp = (ring_buf->wp+1)%ring_buf->total_size;
        ring_buf->data_cnt++;
        ret = RETURN_OK;
    }

    uart_log_ring_buff_unlock();
    return ret;
}


int32_t uart_log_ring_buffer_read(uart_log_ring_buffer_t *ring_buf,char *ch)
{
    int32_t ret;
    uart_log_ring_buff_lock();

    if(uart_log_ring_is_empty(ring_buf))
    {
        ret = RETURN_ERR;
    }
    else
    {
        *ch = ring_buf->base_addr[ring_buf->rp];
        ring_buf->rp = (ring_buf->rp+1)%ring_buf->total_size;
        ring_buf->data_cnt--;
        ret = RETURN_OK;
    }
    
    uart_log_ring_buff_unlock();
    return ret;
}


int _putchar(int ch)
{
    if('\n'==ch)
    {
        uart_log_ring_buffer_write(&uart_log_buffer,'\r');
        #if RTL8189_LOG
        UART_IntMaskConfig(CONFIG_CI_LOG_UART_PORT,UART_TXInt,DISABLE); //用于8189WiFi库
        #endif
    }
    uart_log_ring_buffer_write(&uart_log_buffer,ch);
    return ch;
}

int _puts(char* str)
{
    int counter=0;
    if(!str)
    {
        return 0;
    }
    while(*str && (counter < UART_LOG_BUFF_SIZE))
    {
        _putchar(*str++);
        counter++;
    }
    UART_IntMaskConfig(CONFIG_CI_LOG_UART_PORT,UART_TXInt,DISABLE);
    return counter;
}

void uart_log_int_handler(void)
{
    char ch;
    UART_TypeDef* UART = CONFIG_CI_LOG_UART_PORT;

    /*all int will deal, so no if(UART->UARTMIS & (1UL << UART_TXInt))*/
    {
        do
        {
            if(RETURN_OK != uart_log_ring_buffer_read(&uart_log_buffer,&ch))
            {
                UART_IntMaskConfig(CONFIG_CI_LOG_UART_PORT,UART_TXInt,ENABLE);
                break;
            }
            UART->UARTWrDR = (unsigned int)ch;
        }while(0 == (UART->UARTFlag & (0x1 << 7)));/*bit7 is FIFO FULL*/
        
        UART->UARTICR = (1UL << UART_TXInt);
    }
}


void uart_log_int_mode_init(void)
{
    uart_log_ring_buffer_init(&uart_log_buffer,log_buff,sizeof(log_buff));
    UART_IntMaskConfig(CONFIG_CI_LOG_UART_PORT,UART_TXInt,ENABLE);
    UART_IntMaskConfig(CONFIG_CI_LOG_UART_PORT,UART_RXInt,ENABLE);
}

#else

#if (USE_STD_PRINTF == 0)
void _putchar(char ch)
{
    if('\n'==ch)
    {
        #if CONFIG_CI_LOG_UART
        UartPollingSenddata(CONFIG_CI_LOG_UART_PORT,'\r');
        #endif
    }
    #if CONFIG_CI_LOG_UART
    UartPollingSenddata(CONFIG_CI_LOG_UART_PORT,ch);
    #endif
    return ;
}

/**
  * @功能:字符串打印
  * @参数: 1.str字符串的首地址
  * @返回值:打印的字符个数
  */
int _puts(char* str)
{
    int counter=0;
    while(*str && (counter < UART_LOG_BUFF_SIZE))
    {
        _putchar(*str++);
        counter++;
    }
    return counter;
}
#endif
#endif

#if (USE_STD_PRINTF == 0)
static char printk_string[UART_LOG_BUFF_SIZE] = {0};
int _printf(char* format,...)
{
    
    va_list args;
    int plen;
    va_start(args, format);
    plen = MASK_ROM_LIB_FUNC->newlibcfunc.vsnprintf_p(printk_string, sizeof(printk_string)/sizeof(char) - 1, format, args);
    _puts(printk_string);
    va_end(args);
    return plen;
}
#endif