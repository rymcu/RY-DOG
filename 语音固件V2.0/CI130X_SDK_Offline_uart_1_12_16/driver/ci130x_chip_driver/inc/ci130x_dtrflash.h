#ifndef __SPIC_H_
#define __SPIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define SPIC_DMA_MODEFI              0   //待芯片回来，提高主频后，再测试哪一种方式效率更高，且代码更少

//FLASH的时序窗口调整
#define RX_NAGE_SAMPLE               0
#define RX_NAGE_EN                   1
#define DELAY_LINE                   0x1A   //1 ~ 32S

typedef enum/*!< 功能:flash器件时钟与core_clk时钟频率关系 */
{
    FLASH_CLK_DIV_2 = 0,/*!< 功能:2分频 */
    FLASH_CLK_DIV_4 = 1,/*!< 功能:4分频 */
    FLASH_CLK_DIV_6 = 2,/*!< 功能:6分频 */
    FLASH_CLK_DIV_8 = 3,/*!< 功能:8分频 */
}flash_clk_div_t;

typedef enum/*!< 功能:发送命令、地址、数据的模式选择 */
{
    MD_SEL_LINE_1           =  0,/*!< 功能:单线模式 */
    MD_SEL_LINE_4           =  1,/*!< 功能:四线模式 */
    MD_SEL_LINE_8           =  2,/*!< 功能:八线模式 */
    MD_SEL_RESERVED_0       =  3,/*!< 功能:保留 */
    MD_SEL_LINE_1_MERGE     =  4,/*!< 功能:单线MERGE模式 */
    MD_SEL_LINE_4_MERGE     =  5,/*!< 功能:四线MERGE模式 */
    MD_SEL_RESERVED_1       =  6,/*!< 功能:保留 */
    MD_SEL_RESERVED_2       =  7,/*!< 功能:保留 */
    MD_SEL_DTR_LINE_1       =  8,/*!< 功能:DTR单线模式 */
    MD_SEL_DTR_LINE_4       =  9,/*!< 功能:DTR四线模式 */
    MD_SEL_DTR_LINE_8       = 10,/*!< 功能:DTR八线模式 */
    MD_SEL_RESERVED_3       = 11,/*!< 功能:保留 */
    MD_SEL_DTR_LINE_1_MERGE = 12,/*!< 功能:DTR单线MERGE模式 */
    MD_SEL_DTR_LINE_4_MERGE = 13,/*!< 功能:DTR四线MERGE模式 */
    MD_SEL_RESERVED_4       = 14,/*!< 功能:保留 */
}md_sel_t;

typedef struct
{
    flash_clk_div_t flash_clk_div;/*!< 功能:flash时钟分频 */
    uint32_t flash_sel;/*!< 功能:flash选择：0：flash0；1：flash1 */
    uint32_t addr_size;/*!< 功能:地址宽度(单位Byte) */
}spic_init_t,*spic_init_p;

typedef struct
{
    md_sel_t cmd_md;/*!< 功能:命令模式 */
    md_sel_t data_md;/*!< 功能:数据模式 */
    uint32_t cmd0;/*!< 功能:命令0 */
    uint32_t cmd1;/*!< 功能:命令1 */
    uint32_t addr;/*!< 功能:操作地址 */
    uint32_t addr_en;/*!< 功能:地址是否有效 */
    uint32_t dummy;/*!< 功能:Dummy周期 */
    uint32_t dummy_en;/*!< 功能:Dummy是否有效 */
}spic_base_config_t,*spic_base_config_p;

int32_t spic_cmd(uint32_t spic_base,spic_base_config_p spic_base_config);

int32_t spic_read_by_cpu(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* read_data,uint32_t read_len);

int32_t spic_write_by_cpu(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* write_data,uint32_t write_len);

int32_t spic_read_by_dma(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* read_data,uint32_t read_len);

int32_t spic_write_by_dma(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* write_data,uint32_t write_len);

int32_t spic_read_xip(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* read_data,uint32_t read_len);

int32_t spic_init(uint32_t spic_base,spic_init_p init);

int32_t spic_clk_phase_set(uint32_t spic_base,uint32_t tx_shift,uint32_t tx_nege_en,uint32_t rx_shift,uint32_t rx_nege_en);

void spic_hardware_reset(uint32_t spic_base,uint8_t enable);

int32_t spic_xip_config(uint32_t spic_base,spic_base_config_p spic_base_config);

void spic_prefetch_en(uint32_t spic_base,bool en);

void spic_change_clk(uint32_t spic_base,flash_clk_div_t clk);

#ifdef __cplusplus
}
#endif

#endif
