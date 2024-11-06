/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * NOTE:  This file uses a third party USB CDC driver.
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Example includes. */
#include "FreeRTOS_CLI.h"

/* Demo application includes. */
#include "serial.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE		50

/* Dimentions a buffer to be used by the UART driver, if the UART driver uses a
buffer at all. */
#define cmdQUEUE_LENGTH			1024

/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )
#define cmdDEL_CHAR         "\b \b"

/* The maximum time to wait for the mutex that guards the UART to become
available. */
#define cmdMAX_MUTEX_WAIT		       pdMS_TO_TICKS( 300 )

#ifndef configCLI_BAUD_RATE
    #define configCLI_BAUD_RATE        921600
#endif

#ifndef configMAX_HISTORY_NUM
    #define  configMAX_HISTORY_NUM     5
#endif

#ifndef configMAX_COMPLETION_NUM
    #define configMAX_COMPLETION_NUM   5
#endif

#ifndef configERR_KEY_SEND_RING
    #define configERR_KEY_SEND_RING    0
#endif

#ifndef configSUPPOST_XTERM
    #define configSUPPOST_XTERM        0
#endif

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask( void *pvParameters );
void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );

/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
const char * const pcWelcomeMessage = 
"Chipintelli command console.";
static const char * const pcEndOfOutputMessage = "\r\n\033[32mCI130X@FREERTOS>\033[0m";
static const char * const pcNewLine = "\r\n";

/* Used to guard access to the UART in case messages are sent to the UART from
more than one task. */
static SemaphoreHandle_t xTxMutex = NULL;

/* The handle to the UART port, which is not used by all ports. */
static xComPortHandle xPort = 0;

/* 历史命令表 */
static char cHistoryInputString[configMAX_HISTORY_NUM][ cmdMAX_INPUT_SIZE ] = {0};

/* 匹配补全缓冲区 */
char cCompletionString[cmdMAX_INPUT_SIZE*configMAX_COMPLETION_NUM];

/*-----------------------------------------------------------*/

static void vCursorRight(int num)
{
    /* 光标左移 */
    char cmdCursorRight[6];
    sprintf(cmdCursorRight,"\033[%dD",num);
    vSerialPutString( xPort, (signed char *)cmdCursorRight, ( unsigned short ) strlen( cmdCursorRight ) );	
}
/*-----------------------------------------------------------*/

static void vCursorLeft(int num)
{
    /* 光标右移 */
    char cmdCursorLeft[6];
    sprintf(cmdCursorLeft,"\033[%dC",num);
    vSerialPutString( xPort, (signed char *)cmdCursorLeft, ( unsigned short ) strlen( cmdCursorLeft ) );	
}
/*-----------------------------------------------------------*/

void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority )
{
    /* Create the semaphore used to access the UART Tx. */
    xTxMutex = xSemaphoreCreateMutex();
    configASSERT( xTxMutex );

    vRegisterSampleCLICommands();

    /* Create that task that handles the console itself. */
    xTaskCreate( 	prvUARTCommandConsoleTask,	/* The task that implements the command console. */
                    "CLI",						/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
                    usStackSize,				/* The size of the stack allocated to the task. */
                    NULL,						/* The parameter is not used, so NULL is passed. */
                    uxPriority,					/* The priority allocated to the task. */
                    NULL );						/* A handle is not required, so just pass NULL. */
}
/*-----------------------------------------------------------*/
static void prvUARTCommandConsoleTask( void *pvParameters )
{
signed char cRxedChar;
uint8_t ucInputIndex = 0;
char *pcOutputString;
static char cInputString[ cmdMAX_INPUT_SIZE ];
static char cInputStringBack[ cmdMAX_INPUT_SIZE ];
static uint8_t ucInputStringBackIndex = 0;
static uint8_t ucHistoryInputStringIndex = 0;
static uint8_t ucCtlInputStringIndex = 1;
BaseType_t xReturned;
int retcmp;
uint8_t ucFindCompletionNum = 0;
uint8_t ucCompletionlen;
uint8_t ucTempIndex;

    ( void ) pvParameters;

    /* Obtain the address of the output buffer.  Note there is no mutual
    exclusion on this buffer as it is assumed only one command console interface
    will be used at any one time. */
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    /* Initialise the UART. */
    xPort = xSerialPortInitMinimal( configCLI_BAUD_RATE, cmdQUEUE_LENGTH );

    /* Send the welcome message. */
    vSerialPutString( xPort, ( signed char * ) pcNewLine, ( unsigned short ) strlen( pcNewLine ) );
    vSerialPutString( xPort, ( signed char * ) pcWelcomeMessage, ( unsigned short ) strlen( pcWelcomeMessage ) );
    vSerialPutString( xPort, ( signed char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );

    for( ;; )
    {
        /* Wait for the next character.  The while loop is used in case
        INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
        be a genuine block time rather than an infinite block time. */
        while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );

        /* Ensure exclusive access to the UART Tx. */
        if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
        {
            /* Was it the end of the line? */
            if( cRxedChar == '\n' || cRxedChar == '\r' )
            {
                /* Echo the character back. */
                xSerialPutChar( xPort, cRxedChar, portMAX_DELAY );

                /* Just to space the output from the input. */
                vSerialPutString( xPort, ( signed char * ) pcNewLine, ( unsigned short ) strlen( pcNewLine ) );

                for(int i = ucInputStringBackIndex-1; i >= 0; i--)
                {
                    cInputString[ucInputIndex] = cInputStringBack[i];
                    ucInputIndex++;
                }
                cInputString[ucInputIndex] = '\0';
                ucInputStringBackIndex = 0;

                if( ucInputIndex != 0 )
                {
                    /* Pass the received command to the command interpreter.  The
                    command interpreter is called repeatedly until it returns
                    pdFALSE	(indicating there is no more output) as it might
                    generate more than one string. */
                    do
                    {
                        /* Get the next output string from the command interpreter. */
                        xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

                        /* Write the generated string to the UART. */
                        vSerialPutString( xPort, ( signed char * ) pcOutputString, ( unsigned short ) strlen( pcOutputString ) );

                    } while( xReturned != pdFALSE );

                    /* All the strings generated by the input command have been
                    sent.  Clear the input string ready to receive the next command.
                    Remember the command that was just processed first in case it is
                    to be processed again. */
                    if(ucHistoryInputStringIndex >= 1)
                    {
                        retcmp = strcmp( cInputString, cHistoryInputString[ucHistoryInputStringIndex-1] );
                    }
                    else
                    {
                        retcmp = strcmp( cInputString, cHistoryInputString[configMAX_HISTORY_NUM-1] );
                    }
                    if(0 != retcmp)
                    {
                        strcpy( cHistoryInputString[ucHistoryInputStringIndex], cInputString );
                        ucHistoryInputStringIndex++;
                        if(ucHistoryInputStringIndex >= configMAX_HISTORY_NUM)
                        {
                            ucHistoryInputStringIndex = 0;
                        }
                    }
                    vSerialPutString( xPort, ( signed char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );
                }
                else
                {
                    vSerialPutString( xPort, ( signed char * ) pcEndOfOutputMessage+2, ( unsigned short ) strlen( pcEndOfOutputMessage+2 ) );
                }
            
                ucInputIndex = 0;
                ucCtlInputStringIndex = 1;
                memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
            }
            else
            {
                if( cRxedChar == '\r' )
                {
                    /* Echo the character back. */
                    xSerialPutChar( xPort, cRxedChar, portMAX_DELAY );
                }
                else if( ( cRxedChar == '\b' ) || ( cRxedChar == cmdASCII_DEL ) )
                {   
                    /* BackSpace */
                    if( ucInputIndex > 0 )
                    {
                        /* 清除光标后字符 */
                        vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	

                        /* 删除一个字符 */
                        ucInputIndex--;
                        cInputString[ ucInputIndex ] = '\0';
                        vSerialPutString( xPort, (signed char *)cmdDEL_CHAR, ( unsigned short ) strlen( cmdDEL_CHAR ) );						
                        
                        /* 打印光标后的字符 */
                        for(int i = ucInputStringBackIndex-1; i >= 0; i--)
                        {
                            xSerialPutChar( xPort, cInputStringBack[i],portMAX_DELAY);
                        }
                        if(ucInputStringBackIndex > 0)
                        {
                            vCursorRight(ucInputStringBackIndex);
                        }
                    }
                    else
                    {
                        #if configERR_KEY_SEND_RING
                        vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                        #endif
                    }
                }
                else if( cRxedChar == '\t' )
                {
                    /* Tab */
                    /* 丢弃光标后字符，并清除显示 */
                    ucInputStringBackIndex = 0;
                    vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	

                    /* 匹配命令 */
                    ucFindCompletionNum = configMAX_COMPLETION_NUM;
                    FreeRTOS_CLICompletionCommand(cInputString,ucInputIndex,cCompletionString,&ucFindCompletionNum,&ucCompletionlen);
                    if(ucFindCompletionNum == 0)
                    {
                        /* 未匹配到任何命令 */
                        #if configERR_KEY_SEND_RING
                        vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                        #endif
                    }
                    else if(ucFindCompletionNum == 1)
                    {
                        /* 仅匹配到一条命令 */
                        /* 清除显示 */
                        if(ucInputIndex > 0)
                        {
                            vCursorRight(ucInputIndex);
                            vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	
                        }

                        /* 补全命令并打印 */
                        strcpy( cInputString, cCompletionString );
                        vSerialPutString(xPort, (signed char *)cInputString, (unsigned short) strlen(cInputString));
                        ucInputIndex = strlen(cInputString);
                    }
                    else
                    {
                        /* 匹配多条命令 */
                        /* 新行打印匹配命令 */
                        vSerialPutString(xPort,(signed char *)pcNewLine, (unsigned short) strlen(pcNewLine));
                        vSerialPutString(xPort,(signed char *)cCompletionString, (unsigned short) strlen(cCompletionString));
                        if(ucFindCompletionNum >= configMAX_COMPLETION_NUM)
                        {
							vSerialPutString( xPort, "\r\n无法显示更多匹配项，请增加configMAX_COMPLETION_NUM大小", ( unsigned short ) strlen( "\r\n无法显示更多匹配项，请增加configMAX_COMPLETION_NUM大小" ) );
                        }

                        /* 重新打印当前输入信息 */
                        vSerialPutString( xPort, ( signed char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );
                        if(ucInputIndex != 0)
                        {
                            cCompletionString[ucCompletionlen] = '\0';

                            /* 补全命令并打印 */
                            strcpy( cInputString, cCompletionString );
                            vSerialPutString(xPort, (signed char *)cInputString, (unsigned short) strlen(cInputString));
                            ucInputIndex = strlen(cInputString);
                        }
                    }
                    
                }
                else if( cRxedChar == 0x1b )
                {
                    while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
                    if( cRxedChar == 0x5b )
                    {
                        while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
                        switch(cRxedChar)
                        {
                            /* Top */
                            case 'A':
                            {
                                /* 清除显示 */
                                if(ucInputIndex > 0)
                                {
                                    vCursorRight(ucInputIndex);
                                    vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	
                                }

                                /* 打印历史命令并更新到输入区 */
                                if(ucHistoryInputStringIndex >= ucCtlInputStringIndex)
                                {
                                    strcpy( cInputString, cHistoryInputString[ucHistoryInputStringIndex-ucCtlInputStringIndex] );
                                }
                                else
                                {
                                    strcpy( cInputString,  cHistoryInputString[configMAX_HISTORY_NUM-ucCtlInputStringIndex+ucHistoryInputStringIndex] );
                                }
                                vSerialPutString(xPort, (signed char *)cInputString, (unsigned short) strlen(cInputString));
                                ucInputIndex = strlen(cInputString);
                                ucCtlInputStringIndex++;
                                if(ucCtlInputStringIndex > configMAX_HISTORY_NUM)
                                {
                                    ucCtlInputStringIndex = 1;
                                }
                                ucInputStringBackIndex = 0;
                                break;
                            }
                            /* Down */
                            case 'B':
                            {
                                /* 清除显示 */
                                if(ucInputIndex > 0)
                                {
                                    vCursorRight(ucInputIndex);
                                    vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	
                                }

                                /* 打印历史命令并更新到输入区 */
                                if(ucHistoryInputStringIndex >= ucCtlInputStringIndex)
                                {
                                    strcpy( cInputString, cHistoryInputString[ucHistoryInputStringIndex-ucCtlInputStringIndex] );
                                }
                                else
                                {
                                    strcpy( cInputString,  cHistoryInputString[configMAX_HISTORY_NUM-ucCtlInputStringIndex+ucHistoryInputStringIndex] );
                                }
                                vSerialPutString(xPort, (signed char *)cInputString, (unsigned short) strlen(cInputString));
                                ucInputIndex = strlen(cInputString);
                                ucCtlInputStringIndex--;
                                if(ucCtlInputStringIndex < 1)
                                {
                                    ucCtlInputStringIndex = configMAX_HISTORY_NUM;
                                }
                                ucInputStringBackIndex = 0;
                                break;
                            }
                            /* Right */
                            case 'C':
                            {
                                if(ucInputStringBackIndex > 0)
                                {
                                    xSerialPutChar( xPort, cInputStringBack[ucInputStringBackIndex-1], portMAX_DELAY );
                                    cInputString[ ucInputIndex] = cInputStringBack[ucInputStringBackIndex-1];
                                    ucInputStringBackIndex--;
                                    ucInputIndex++;
                                }
                                else
                                {
                                    #if configERR_KEY_SEND_RING
                                    vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                    #endif
                                }
                                break;
                            }
                            /* Left */ 
                            case 'D':
                            {
                                if(ucInputIndex > 0)
                                {
                                    xSerialPutChar( xPort, '\b', portMAX_DELAY );
                                    cInputStringBack[ucInputStringBackIndex] = cInputString[ ucInputIndex-1];
                                    ucInputStringBackIndex++;
                                    cInputString[ ucInputIndex ] = '\0';
                                    ucInputIndex--;
                                }
                                else
                                {
                                    #if configERR_KEY_SEND_RING
                                    vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                    #endif
                                }
                                break;
                            }
                            /* Delete */
                            case '3':
                            {
                                while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
                                if( ucInputStringBackIndex > 0 )
                                {
                                    /* 清除光标后字符 */
                                    vSerialPutString( xPort, "\033[K", ( unsigned short ) strlen( "\033[K" ) );	

                                    /* 删除一个字符 */
                                    cInputStringBack[ ucInputStringBackIndex ] = '\0';
                                    ucInputStringBackIndex--;
                                    
                                    /* 打印光标后的字符 */
                                    for(int i = ucInputStringBackIndex-1; i >= 0; i--)
                                    {
                                        xSerialPutChar( xPort, cInputStringBack[i],portMAX_DELAY);
                                    }
                                    if(ucInputStringBackIndex > 0)
                                    {
                                        vCursorRight(ucInputStringBackIndex);
                                    }
                                }
                                else
                                {
                                    #if configERR_KEY_SEND_RING
                                    vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                    #endif
                                }
                                break;
                            }
                            /* Home */
                            case '1':
                            {
                                while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
                            }
                            #if configSUPPOST_XTERM
                            case 'H':
                            #endif
                            {
                                if(ucInputIndex > 0)
                                {
                                    vCursorRight(ucInputIndex);
                                    while(ucInputIndex > 0)
                                    {
                                        cInputStringBack[ucInputStringBackIndex] = cInputString[ ucInputIndex-1];
                                        ucInputStringBackIndex++;
                                        cInputString[ ucInputIndex ] = '\0';
                                        ucInputIndex--;
                                    }
                                }
                                else
                                {
                                    #if configERR_KEY_SEND_RING
                                    vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                    #endif
                                }
                                break;
                            }
                            /* End */
                            case '4':
                            {
                                while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
                            }
                            #if configSUPPOST_XTERM
                            case 'F':
                            #endif
                            {
                                if(ucInputStringBackIndex > 0)
                                {
                                    ucTempIndex = ucInputIndex;
                                    while(ucInputStringBackIndex > 0)
                                    {
                                        cInputString[ ucInputIndex] = cInputStringBack[ucInputStringBackIndex-1];
                                        ucInputStringBackIndex--;
                                        ucInputIndex++;
                                    }
                                    vSerialPutString( xPort, (signed char *)cInputString+ucTempIndex, ( unsigned short ) strlen( cInputString+ucTempIndex ) );	
                                }
                                else
                                {
                                    #if configERR_KEY_SEND_RING
                                    vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                    #endif
                                }
                                break;
                            }
                            default:
                            {
                                #if configERR_KEY_SEND_RING
                                vSerialPutString( xPort, "\a", ( unsigned short ) strlen( "\a" ) );	
                                #endif
                                break;
                            }
                        }
                    }
                }
                else
                {
                    /* Echo the character back. */
                    xSerialPutChar( xPort, cRxedChar, portMAX_DELAY );
                    
                    for(int i = ucInputStringBackIndex-1; i >= 0; i--)
                    {
                        xSerialPutChar( xPort, cInputStringBack[i],portMAX_DELAY);
                    }
                    if(ucInputStringBackIndex > 0)
                    {
                        vCursorRight(ucInputStringBackIndex);
                    }
                    /* A character was entered.  Add it to the string entered so
                    far.  When a \n is entered the complete	string will be
                    passed to the command interpreter. */
                    if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
                    {
                        if( ucInputIndex < cmdMAX_INPUT_SIZE )
                        {
                            cInputString[ ucInputIndex ] = cRxedChar;
                            ucInputIndex++;
                        }
                    }
                }
            }

            /* Must ensure to give the mutex back. */
            xSemaphoreGive( xTxMutex );
        }
    }
}
/*-----------------------------------------------------------*/

void vOutputString( const char * const pcMessage )
{
    if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
    {
        vSerialPutString( xPort, ( signed char * ) pcMessage, ( unsigned short ) strlen( pcMessage ) );
        xSemaphoreGive( xTxMutex );
    }
}
/*-----------------------------------------------------------*/
