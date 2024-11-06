/**
 * @file parse_m4a_atom_containers_port.c
 * @brief 解析m4a格式头信息数据读入接口
 * @version 1.0
 * @date 2019-07-18
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "romlib_runtime.h"

#include "parse_m4a_atom_containers_port.h"


mem_file_t *mf_open(uint8_t* mem_addr, uint32_t total_size, uint8_t *attr) 
{
    mem_file_t *handle;
    handle = (mem_file_t *)pvPortMalloc(sizeof(mem_file_t));
    handle->base_addr = mem_addr;
    handle->total_size = total_size;
    handle->current_offset = 0;
    return handle;
}


/*now must be contiguous memory, so no address, just total size set*/
int mf_resize(mem_file_t *handle, uint32_t total_size, uint8_t *attr) 
{
    handle->total_size = total_size;
        
    return 0;
}


int mf_close(mem_file_t *handle) 
{
    vPortFree(handle);
    
    return 0;
}


int mf_tell(mem_file_t *handle)
{
    return handle->current_offset;
}

int mf_size(mem_file_t *handle)
{
    return handle->total_size;
}

int mf_read(void *buffer, size_t size, size_t count, mem_file_t *handle)
{
    uint32_t end = size*count + handle->current_offset;
    uint32_t cpy_size;

    if(end > handle->total_size)
    {
        cpy_size = handle->total_size - handle->current_offset;
    }
    else
    {
        cpy_size = size*count;
    }
    
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(buffer,handle->base_addr+handle->current_offset,cpy_size);
    handle->current_offset += cpy_size;
    return cpy_size;
}


int mf_seek(mem_file_t *handle, long offset, int fromwhere)
{
    if(SEEK_SET == fromwhere)
    {
        handle->current_offset =  offset;    
    }
    else if(SEEK_CUR == fromwhere)
    {
        handle->current_offset += offset;
    }
    else if(SEEK_END == fromwhere)
    {
        //??
        //handle->current_offset = handle->base_addr + handle->total_size + offset;    
    }
    
    if(handle->current_offset > handle->total_size)
    {
        handle->current_offset = handle->total_size;
    }
    return 0;
}
