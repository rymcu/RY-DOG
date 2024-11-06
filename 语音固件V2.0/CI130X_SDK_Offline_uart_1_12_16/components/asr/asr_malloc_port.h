/**
 * @file asr_decoder_port.h
 * @brief 
 * @version 0.1
 * @date 2019-06-19
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#include "stdint.h"

#ifndef _ASR_DECODER_PORT_H_
#define _ASR_DECODER_PORT_H_


#ifdef __cplusplus
extern "C"
{
#endif    

unsigned int get_pruntab_nocache_addr(uint16_t size);
void free_asrwindows_nocache_addr(unsigned int addr);

extern void *decoder_port_malloc(int size);
extern void decoder_port_free(void *pp);
extern void* asr_hardware_malloc(int size);
extern void asr_hardware_free(void* p);
extern void free_dnn_outbuf_addr(void*p);
extern unsigned int malloc_dnn_outbuf_addr(int buf_size,int * buf_total_size);
void * malloc_insram(int size);
void* malloc_insram_bnpu(int size);
void free_insram_bnpu(void* p);
void free_insram(void* p);
void free_inscache_psram(void* p);
void * malloc_inscache_psram(int size);
int check_bufaddr_is_insram(unsigned int addr ,int size);

#ifdef __cplusplus
}
#endif

#endif
