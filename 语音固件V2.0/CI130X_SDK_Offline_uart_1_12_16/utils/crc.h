#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

/**
 * @brief 计算给定数据的8位CRC校验值。
 * 
 * @param pre_crc 前一次计算的CRC值，如果是第一批数据，此值为0。
 * @param data    无符号char指针，指向需要计算CRC校验值的数据。
 * @param length  需要计算CRC校验值的数据的长度。
 * @return unsigned char  计算出来的CRC校验值。
 */
extern uint8_t crc8(uint8_t pre_crc, const uint8_t * data, uint32_t length);

/**
 * @brief 计算给定数据的16位CRC校验值, 数据格式为CCITT标准(初始值0x0000，低位在前，高位在后，结果与0x0000异或)。
 * 
 * @param pre_crc 前一次计算的CRC值，如果是第一批数据，此值为0。
 * @param data    无符号char指针，指向需要计算CRC校验值的数据。
 * @param length  需要计算CRC校验值的数据的长度。
 * @return unsigned short 计算出来的CRC校验值。
 */
extern uint16_t crc16_ccitt(uint16_t pre_crc, const uint8_t * data, uint32_t length);

/**
 * @brief 计算给定数据的32位CRC校验值。
 * 
 * @param pre_crc 前一次计算的CRC值，如果是第一批数据，此值为0。
 * @param data    无符号char指针，指向需要计算CRC校验值的数据。
 * @param length  需要计算CRC校验值的数据的长度。
 * @return unsigned long 计算出来的CRC校验值。
 */
extern uint32_t crc32(uint32_t pre_crc, const uint8_t * data, uint32_t length);


#endif
