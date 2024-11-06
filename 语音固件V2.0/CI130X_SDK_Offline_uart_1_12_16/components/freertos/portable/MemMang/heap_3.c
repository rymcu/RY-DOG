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
 * Implementation of pvPortMalloc() and vPortFree() that relies on the
 * compilers own malloc() and free() implementations.
 *
 * This file can only be used if the linker is configured to to generate
 * a heap memory area.
 *
 * See heap_1.c, heap_2.c and heap_4.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */

#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/*-----------------------------------------------------------*/
#define CHECK_MALLOC_IS_OVER_WRITE	0

#if CHECK_MALLOC_IS_OVER_WRITE
#define MAX_MALLOC_TIME	1000

#define CHECK_NUM	8
#define DATA_TAG (0xa5a5a5a5 + 1)
static volatile uint32_t malloc_addr_size[MAX_MALLOC_TIME][3];
static volatile uint32_t malloc_time = 0;
#endif


void *pvPortMalloc( size_t xWantedSize )
{
void *pvReturn;

	vTaskSuspendAll();
	#if CHECK_MALLOC_IS_OVER_WRITE
	if(xWantedSize % 4)
	{
		xWantedSize = (xWantedSize / 4 + 1) * 4;
	}
	xWantedSize += CHECK_NUM * 2 * sizeof(uint32_t);
	#endif
	{
		pvReturn = malloc( xWantedSize );
		traceMALLOC( pvReturn, xWantedSize );

		#if CHECK_MALLOC_IS_OVER_WRITE
		if(malloc_time == 19)
		{
			mprintf("pvReturn = %x\n",pvReturn);
		}
		malloc_addr_size[malloc_time][0] = pvReturn;
		malloc_addr_size[malloc_time][1] = xWantedSize;
		
		malloc_addr_size[malloc_time][2] = xWantedSize + pvReturn - CHECK_NUM*sizeof(uint32_t);
		malloc_time++;
		if(malloc_time >= MAX_MALLOC_TIME)
		{
			mprintf("malloc too many\n");
			while(1);
		}
		uint32_t* data = (uint32_t*)pvReturn;
		for(int k=0;k<CHECK_NUM;k++)
		{
			data[k] = DATA_TAG;
		}
		data = (uint32_t*)(xWantedSize + pvReturn - CHECK_NUM*sizeof(uint32_t));
		for(int k=0;k<CHECK_NUM;k++)
		{
			data[k] = DATA_TAG;
		}

		pvReturn += CHECK_NUM*sizeof(uint32_t);
		#endif
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
	}
	#endif

	return pvReturn;
}
/*-----------------------------------------------------------*/

#if CHECK_MALLOC_IS_OVER_WRITE
void malloc_check_host(void);

static uint32_t check_num = 0;
#endif

void vPortFree( void *pv )
{
	if( pv )
	{
		vTaskSuspendAll();
		#if CHECK_MALLOC_IS_OVER_WRITE
		malloc_check_host();
		pv -= CHECK_NUM*sizeof(uint32_t);
		check_num = 0;
		for(int i=0;i<MAX_MALLOC_TIME;i++)
		{
			if(((pv) == malloc_addr_size[check_num][0]))
			{
				malloc_addr_size[check_num][0] = 0;
				malloc_addr_size[check_num][1] = 0;
				malloc_addr_size[check_num][2] = 0;
				break;
			}
			check_num++;
		}	
		#endif

		{
			free( pv );
			traceFREE( pv, 0 );
		}
		( void ) xTaskResumeAll();
	}
}

#if CHECK_MALLOC_IS_OVER_WRITE
void malloc_check_host(void)
{
	vTaskSuspendAll();

	for(int i=0;i<malloc_time;i++)
	{
		if(malloc_addr_size[i][0] != 0)
		{
			uint32_t* data = (uint32_t*)malloc_addr_size[i][0];
			uint32_t* data1 = (uint32_t*)malloc_addr_size[i][2];
			for(int k=0;k<CHECK_NUM;k++)
			{
				if((data[k] != DATA_TAG) || (data1[k] != DATA_TAG))
				{
					mprintf("data over write %x i= %d\n",malloc_addr_size[i][0],i);
					while(1);
				}
			}
			
		}
	}

	xTaskResumeAll();
}
#endif

