/*
 * FreeRTOS Kernel V10.1.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "ci130x_scu.h"
#include "ci130x_uart.h"
#include "ci130x_system.h"
#include "ci130x_core_eclic.h"

/* Demo application includes. */
#include "ci_debug_config.h" 
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
// static QueueHandle_t xCharsForTx;


/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
xComPortHandle xReturn;

	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	// xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	if( ( xRxedChars != serINVALID_QUEUE )/* && ( xCharsForTx != serINVALID_QUEUE )*/ )
	{
        /* 初始化串口 */
		if (UART0 == CONFIG_CLI_UART_PORT)
		{
    		__eclic_irq_set_vector(UART0_IRQn,(int32_t)vUARTInterruptHandler);
		}    
		else if (UART1 == CONFIG_CLI_UART_PORT)
		{
    		__eclic_irq_set_vector(UART1_IRQn,(int32_t)vUARTInterruptHandler);
		}
		else if (UART2 == CONFIG_CLI_UART_PORT)
		{
    		__eclic_irq_set_vector(UART2_IRQn,(int32_t)vUARTInterruptHandler);
		}
        UARTInterruptConfig(CONFIG_CLI_UART_PORT,UART_BaudRate921600);
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	( void ) usStringLength;
	( void ) pxPort;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports CONFIG_CLI_UART_PORT. */
	( void ) pxPort;

	/* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;
	while( *pxNext )
	{
		xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
		pxNext++;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
    signed portBASE_TYPE xReturn = pdPASS;

	// if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
	// {
	// 	xReturn = pdPASS;
	// 	UART_IntMaskConfig( CONFIG_CLI_UART_PORT, UART_TXInt, DISABLE );
	// }
	// else
	// {
	// 	xReturn = pdFAIL;
	// }

    UartPollingSenddata(CONFIG_CLI_UART_PORT, cOutChar);
	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/



void vUARTInterruptHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	/*发送*/
    // if(((UART_TypeDef*)CONFIG_CLI_UART_PORT)->UARTMIS & (1UL << UART_TXInt))
	// {
	// 	/* The interrupt was caused by the THR becoming empty.  Are there any
	// 	more characters to transmit? */
	// 	if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
	// 	{
	// 		/* A character was retrieved from the queue so can be sent to the
	// 		THR now. */
	// 		((UART_TypeDef*)CONFIG_CLI_UART_PORT)->UARTWrDR = (unsigned int)cChar;
	// 	}
	// 	else
	// 	{
	// 		UART_IntMaskConfig( CONFIG_CLI_UART_PORT, UART_TXInt, ENABLE );		
	// 	}
	//}

	/*接收*/
	if(((UART_TypeDef*)CONFIG_CLI_UART_PORT)->UARTMIS & (1UL << UART_RXInt))
	{
		//here FIFO DATA must be read out 
		cChar = (char)(((UART_TypeDef*)CONFIG_CLI_UART_PORT)->UARTRdDR & 0xff);
		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken ); 
	}
	
	((UART_TypeDef*)CONFIG_CLI_UART_PORT)->UARTICR = 0xFFF;
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
