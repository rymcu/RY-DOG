/**
 * @file ci130x_spiflash.h
 * @brief  SPIFLASH驱动文件
 *
 * @version 0.1
 * @date 2019-03-28
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#ifndef _CI130X_SPIFLASH_H_
#define _CI130X_SPIFLASH_H_

#include "ci130x_system.h"

/**
 * @ingroup ci130x_chip_driver
 * @defgroup ci130x_spiflash ci130x_spiflash
 * @brief CI130X芯片SPIFLASH驱动
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SPI0FIFO_BASE
#define SPI0FIFO_BASE 0x60000000/*!< SPI0FIFO 基地址定义 */
#endif

#define FLASH_MSF (0)/*!< 当Flash容量大于16M byte时，应该定义为 1 */

/**
 * @brief SPI控制器定义
 */
typedef enum
{
    QSPI0 = HAL_DTRFLASH_BASE,/*!< SPI0控制器 */
}spic_base_t;

/**
 * @brief FLASH命令定义
 */
typedef enum
{
    SPIC_CMD_CODE_WRITE_ENABLE                  = 0x06,      /*!< Write Enable */
    SPIC_CMD_CODE_READMANUFACTURERIDDEVICEID    = 0x90,     /*!< Read Manufacturer / Device ID */
    SPIC_CMD_CODE_READJEDECID                   = 0x9F,     /*!< Read JEDECID ID */
    SPIC_CMD_CODE_READSTATUSREG1                = 0x05,     /*!< Read Status Register1 */
    SPIC_CMD_CODE_READSTATUSREG2                = 0x35,     /*!< Read Status Register2 */
    SPIC_CMD_CODE_READSTATUSREG3                = 0x15,     /*!< Read Status Register3 */
    SPIC_CMD_CODE_SECTORERASE4K                 = 0x20,     /*!< 4K Erase */
    SPIC_CMD_CODE_BLOCKERASE32K                 = 0x52,     /*!< 32K Erase */
    SPIC_CMD_CODE_BLOCKERASE64K                 = 0xd8,     /*!< 64K Erase */
    SPIC_CMD_CODE_CHIPERASE                     = 0xc7,     /*!< Chip Erase ZBIT:Quad模式不支持 */
    SPIC_CMD_CODE_PAGEPROGRAM                   = 0x02,     /*!< Page Program */
    SPIC_CMD_CODE_QUADINPUTPAGEPROGRAM          = 0x32,     /*!< Quad Input Page Program  ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_WRSTATUSREG                   = 0x01,     /*!< Write Status Register1 */
    SPIC_CMD_CODE_WRSTATUSREG2                  = 0x31,     /*!< Write Status Register2 */
    SPIC_CMD_CODE_WRSTATUSREG3                  = 0x11,     /*!< Write Status Register3 */
    SPIC_CMD_CODE_READDATA                      = 0x03,     /*!< Read Data ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_FASTREAD                      = 0x0b,     /*!< Fast Read */
    SPIC_CMD_CODE_FASTREADDUALOUTPUT            = 0x3b,     /*!< Fast Read Dual Output ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_FASTREADQUADOUTPUT            = 0x6b,     /*!< Fast Read Quad Output ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_FASTREADQUADIO                = 0xeb,     /*!< Fast Read Quad IO */
    SPIC_CMD_CODE_POWERDOWN                     = 0xb9,     /*!< PowerDown */
    SPIC_CMD_CODE_RELEASEPOWERDOWN              = 0xab,     /*!< Release PowerDown */
    SPIC_CMD_CODE_ENABLERESET                   = 0x66,     /*!< Enable Reset */
    SPIC_CMD_CODE_RESET                         = 0x99,     /*!< Reset Device */
    SPIC_CMD_CODE_READ_UNIQUE_ID                = 0x4b,     /*!< Read Uniaue ID Number ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_ERASE_SECURITY_REG            = 0x44,     /*!< Erase Security Registers ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_WRITE_SECURITY_REG            = 0x42,     /*!< Program Security Registers ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_READ_SECURITY_REG             = 0x48,     /*!< Read Security Registers ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_WRITE_EXTENDED_ADDR_REG       = 0xC5,     /*!< write extended Registers ZBIT:Quad模式不支持*/
    SPIC_CMD_CODE_READ_EXTENDED_ADDR_REG        = 0xC8,     /*!< read extended Registers ZBIT:Quad模式不支持*/
	SPIC_CMD_CODE_ENABLE_QUAD                   = 0x38,     /*!<  */
	SPIC_CMD_CODE_DISABLE_QUAD                  = 0xFF,     /*!<  */
}spic_cmd_code_t;

/**
 * @brief FLASH安全寄存器定义
 */
typedef enum
{
    SPIC_SECURITY_REG1 = 0,/*!< 安全寄存器0 */
    SPIC_SECURITY_REG2 = 1,/*!< 安全寄存器1 */
    SPIC_SECURITY_REG3 = 2,/*!< 安全寄存器2 */
}spic_security_reg_t;

int32_t flash_init(spic_base_t spic);
int32_t spic_read_unique_id(spic_base_t spic,uint8_t* unique);
int32_t spic_read_jedec_id(spic_base_t spic,uint8_t* jedec);
int32_t spic_erase_security_reg(spic_base_t spic,spic_security_reg_t reg);
int32_t spic_write_security_reg(spic_base_t spic,spic_security_reg_t reg,
                              uint32_t buf,uint32_t addr,uint32_t size);
int32_t spic_read_security_reg(spic_base_t spic,spic_security_reg_t reg,
                              uint32_t buf,uint32_t addr,uint32_t size);
int32_t spic_security_reg_lock(spic_base_t spic,spic_security_reg_t reg);
int32_t flash_erase(spic_base_t spic,uint32_t addr,uint32_t size);
int32_t flash_write(spic_base_t spic,uint32_t addr,uint32_t buf,uint32_t size);
int32_t flash_read(spic_base_t spic,uint32_t buf,uint32_t addr,uint32_t size);
int32_t dnn_mode_config(spic_base_t spic,uint32_t start_addr,uint32_t size);
int32_t flash_dnn_mode(spic_base_t spic,FunctionalState cmd);
uint32_t flash_check_mode(spic_base_t spic);
int32_t spic_quad_mode(spic_base_t spic);
int32_t spic_erase(spic_base_t spic,spic_cmd_code_t code,uint32_t addr);
int32_t spic_protect(spic_base_t spic,FunctionalState cmd);
int32_t flash_is_dnn_mode(spic_base_t spic);
int32_t spic_reset(spic_base_t spic);
int32_t spic_xipconfig(spic_base_t spic);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
