#ifndef CI_ALG_MALLOC_H
#define CI_ALG_MALLOC_H

#include <stdio.h>
#include <stdint.h>

#define USE_STANDARD_MALLOC 1

/*
 * Readme first:
 */

#ifdef __cplusplus
extern "C"
{
#endif

    /**
 * @brief same as malloc().
 * 
 * @param size_in_byte the buffer length in byte.

 * @return The pointer of buffer, NULL if error, else ok.
 */
    void *ci_algbuf_malloc(size_t size_in_byte);
    /**
 * @brief same as calloc().
 * @param size_in_byte the buffer length in byte.
 * @param size_of_byte the buffer length.
 * @return The pointer of buffer, NULL if error, else ok.
 */
    void *ci_algbuf_calloc(size_t size_in_byte, size_t size_of_byte);

    /**
 * @brief same as free().
 * @param buf the pointer of buffer ( from the return value of ci_algbuf_malloc() or ci_algbuf_calloc() ).
 * @return The pointer of buffer, NULL if error, else ok.
 */
    void ci_algbuf_free(void *buf);
    /**
 * @brief 获取malloc分配内存大小.
  * @return malloc分配内存大小.
 */
    uint32_t get_alg_malloc_size(void);
    /**
 * @brief 获取calloc分配内存大小.
  * @return calloc分配内存大小.
 */
    uint32_t get_alg_calloc_size(void);

#ifdef __cplusplus
}
#endif

#endif /* CI_DATA_IO_H */
