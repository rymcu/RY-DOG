#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include "ci130x_uart.h"
#include "sdk_default_config.h"



void _delay_10us_240M(uint32_t cnt)
{
    volatile uint32_t i,j = 0;
    for(i = 0; i < cnt; i++)
    {
        for(j = 0; j < 160; j++)
        {
            __asm volatile("nop");
        }
    }
}

