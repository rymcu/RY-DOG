#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#include "ci130x_scu.h"
#include "ci130x_uart.h"
#include "ci130x_system.h"
#include "ci130x_core_eclic.h"
#include "ci_debug_config.h"

#define _SERVER_HELLO_SIZE (4)
#define _TARGET_HELLO_SIZE (4)

static struct
{
    U8 NumBytesHelloRcvd;
    U8 NumBytesHelloSent;
    int ChannelID;
} _SVInfo;

static const U8 _abHelloMsg[_TARGET_HELLO_SIZE] = {'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000), (SEGGER_SYSVIEW_VERSION / 1000) % 10}; // "Hello" message expected by SysView: [ 'S', 'V', <PROTOCOL_MAJOR>, <PROTOCOL_MINOR> ]


/**
 * @brief This function starts and initializes a SystemView session, if necessary.
 * 
 */
static void _StartSysView(void)
{
    int r;

    r = SEGGER_SYSVIEW_IsStarted();
    if (r == 0)
    {
        SEGGER_SYSVIEW_Start();
    }
}


/**
 * @brief This function is called when the UART receives data.
 * 
 * @param Data 
 */
static void _cbOnRx(U8 Data)
{
    if (_SVInfo.NumBytesHelloRcvd < _SERVER_HELLO_SIZE)
    { // Not all bytes of <Hello> message received by SysView yet?
        _SVInfo.NumBytesHelloRcvd++;
        /* 目前版本V3.10，增加这个判断才能正确启动  modify by QQM */
        if(_SVInfo.NumBytesHelloRcvd == _SERVER_HELLO_SIZE-1)
        {
        	_StartSysView();
        }
        goto Done;
    }
    _StartSysView();
    SEGGER_RTT_WriteDownBuffer(_SVInfo.ChannelID, &Data, 1); // Write data into corresponding RTT buffer for application to read and handle accordingly
Done:
    return;
}


/**
 * @brief This function is called when the UART should transmit data.
 * 
 * @param pChar 
 * @return int 
 */
static int _cbOnTx(U8 *pChar)
{
    int r;

    if (_SVInfo.NumBytesHelloSent < _TARGET_HELLO_SIZE)
    { // Not all bytes of <Hello> message sent to SysView yet?
        *pChar = _abHelloMsg[_SVInfo.NumBytesHelloSent];
        _SVInfo.NumBytesHelloSent++;
        r = 1;
        goto Done;
    }
    r = SEGGER_RTT_ReadUpBufferNoLock(_SVInfo.ChannelID, pChar, 1);
    if (r < 0)
    { // Failed to read from up buffer?
        r = 0;
    }
Done:
    return r;
}


void vSYSVIEWUARTEnableTXEInterrupt(U32 NumBytes)
{
	UART_IntMaskConfig( CONFIG_SYSVIEW_UART_PORT, UART_TXInt, DISABLE );
}


/**
 * @brief sysview uart handle
 * 
 */
void vSYSVIEWUARTInterruptHandler(void)
{
    char cChar;

    /*接收*/
    if (CONFIG_SYSVIEW_UART_PORT->UARTMIS & (1UL << UART_RXInt))
    {
        cChar = (char)(CONFIG_SYSVIEW_UART_PORT->UARTRdDR & 0xff);
        _cbOnRx(cChar);
    }

    /*发送*/
    if (CONFIG_SYSVIEW_UART_PORT->UARTMIS & (1UL << UART_TXInt))
    {
        if (0 == _cbOnTx(&cChar))
        {
            UART_IntMaskConfig(CONFIG_SYSVIEW_UART_PORT, UART_TXInt, ENABLE);
        }
        else
        {
            CONFIG_SYSVIEW_UART_PORT->UARTWrDR = (unsigned int)cChar;
        }
    }
    CONFIG_SYSVIEW_UART_PORT->UARTICR = 0xFFF;
}


/**
 * @brief sysview uart init
 * 
 */
void vSYSVIEWUARTInit(void)
{
    _SVInfo.ChannelID = SEGGER_SYSVIEW_GetChannelID(); // Store system view channel ID for later communication

    /* 初始化串口 */
    if (UART0 == CONFIG_SYSVIEW_UART_PORT)
    {
        eclic_irq_set_priority(UART0_IRQn, 1, 0);
        __eclic_irq_set_vector(UART0_IRQn, (int32_t)vSYSVIEWUARTInterruptHandler);
    }
    else if (UART1 == CONFIG_SYSVIEW_UART_PORT)
    {
        eclic_irq_set_priority(UART1_IRQn, 1, 0);
        __eclic_irq_set_vector(UART1_IRQn, (int32_t)vSYSVIEWUARTInterruptHandler);
    }

    UARTInterruptConfig(CONFIG_SYSVIEW_UART_PORT, UART_BaudRate460800);
    // UARTInterruptConfig(CONFIG_SYSVIEW_UART_PORT, UART_BaudRate921600);
}
