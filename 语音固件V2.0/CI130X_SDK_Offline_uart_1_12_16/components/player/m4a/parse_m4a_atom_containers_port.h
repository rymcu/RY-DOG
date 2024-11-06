/**
 * @file parse_m4a_atom_containers_port.h
 * @brief 解析m4a格式头信息数据读入接口
 * @version 1.0
 * @date 2019-07-18
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _PARSE_M4A_ATOM_CONTAINERS_PORT_H_
#define _PARSE_M4A_ATOM_CONTAINERS_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct 
{
    uint8_t* base_addr;
    uint32_t total_size;
    uint32_t current_offset;
}mem_file_t;

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2


mem_file_t *mf_open(uint8_t* mem_addr, uint32_t total_size, uint8_t *attr);

/*now must be contiguous memory, so no address, just total size set*/
int mf_resize(mem_file_t *handle, uint32_t total_size, uint8_t *attr);

int mf_close(mem_file_t *handle); 

int mf_tell(mem_file_t *handle);

int mf_size(mem_file_t *handle);

int mf_read(void *buffer, size_t size, size_t count, mem_file_t *handle);

int mf_seek(mem_file_t *handle, long offset, int fromwhere);

#ifdef __cplusplus
}
#endif


#endif

