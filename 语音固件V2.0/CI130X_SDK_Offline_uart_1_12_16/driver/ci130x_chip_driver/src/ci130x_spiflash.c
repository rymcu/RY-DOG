/**
 * @file ci130x_spiflash.c
 * @brief  SPIFLASH驱动文件
 *
 * @version 0.1
 * @date 2019-03-27
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include <string.h>
#include "ci130x_spiflash.h"
#include "ci130x_dma.h"
#include "ci130x_scu.h"
#include "ci130x_dpmu.h"
#include "ci130x_core_eclic.h"
#include "ci130x_dtrflash.h"
#include "platform_config.h"
#include "romlib_runtime.h"
#include "ci_log.h"

#define SPIC_CLK                    (get_apb_clk()) //60000000
#define SPIC_MS                     (SPIC_CLK / 1000)
#define OTHER_TIMR                  (20UL)    /* ms */
#define ERASE_64K                   (64 * 1024)
#define ERASE_4K                    (4 * 1024)
#define FLASH_PAGE_SIZE             (256)

#if FLASH_MSF
#define FLASH_SISZE_16M             (16 * 1024 * 1024)
#endif

#define SPIFLASH_CPU                (0)    /*!< 数据FIFO：0，DMA模式，1，CPU模式 */
#define SPIFLASH_CMD_QUAD           (0)    /*!< 命令模式：0，单线，1，四线 */
#define SPIFLASH_DATA_QUAD          (1)    /*!< 数据模式：0，单线，1，四线 */

#define SPIFLASH_NORMAL_READ        (1)    /*!< 0：不使用普通读，1：使用普通读 */

#define SPIFLASH_NORMAL_WRITE_READ_CLK  (FLASH_CLK_DIV_4)     /*!240M主频，普通模式读写只能4分频 */
#define SPIFLASH_XIP_READ_CLK           (FLASH_CLK_DIV_2)     /*!240M主频，XIP可以2分频 */

uint8_t spi_flash_drv0_drv1 = 1;     /*!< FLASH驱动能力，0：25%，1：50%，2:75%，3:100% */

uint8_t spic_cmd_quad_flag = 0;
uint8_t spic_data_quad_flag = 0;

/**
 * @brief SPI保护类型定义
 */
typedef enum{
    SPIC_SOFTWAREPROTECTION        =0,     /*!< 软件保护 */
    SPIC_HARDWAREPROTECTION        =1,     /*!< 保护由硬件决定 WP为低则保护 */
    SPIC_POWERSUPPLYLOCK_DOWN      =2,     /*!< 必须产生上电序列才能写 */
    SPIC_ONETIMEPROGRAM            =3,     /*!< 一次性编程保护，flash被永久性保护 */
    SPIC_RESV                      =-1,    /*!< 保留 */
}spic_status_protect_t;

static int32_t spic_send_cmd(spic_base_t spic, spic_cmd_code_t cmd)
{
    spic_base_config_t spic_base_config = {0};
	spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.cmd0 = cmd;
    if(spic_cmd(spic,&spic_base_config))
    {
        RETURN_ERR;
    }
    return RETURN_OK;
}

static int32_t spic_read_cmd_value(spic_base_t spic, spic_cmd_code_t cmd, uint8_t* value, uint8_t value_len)
{
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = MD_SEL_LINE_1;
    spic_base_config.data_md = MD_SEL_LINE_1;
    spic_base_config.cmd0 = cmd;

    if(SPIC_CMD_CODE_READ_UNIQUE_ID == cmd)
    {
        spic_base_config.cmd_md = MD_SEL_LINE_1;
        spic_base_config.data_md = MD_SEL_LINE_1;
        #if 1
        spic_base_config.addr = 0x00;
        spic_base_config.addr_en = 1;
        spic_base_config.dummy = 8;
        #else
        spic_base_config.dummy = 32;
        #endif
        spic_base_config.dummy_en = 1;
    }
    else if(SPIC_CMD_CODE_READJEDECID == cmd)
    {
        spic_base_config.cmd_md = spic_cmd_quad_flag;
        spic_base_config.data_md = spic_cmd_quad_flag;
    }
    else if(SPIC_CMD_CODE_RELEASEPOWERDOWN == cmd)
    {
        spic_base_config.dummy = 24;
        spic_base_config.dummy_en = 1;
    }
    else if(SPIC_CMD_CODE_READSTATUSREG1 == cmd)
    {
        spic_base_config.cmd_md = spic_cmd_quad_flag;
        spic_base_config.data_md = spic_cmd_quad_flag;
    }
    #if FLASH_MSF
    else if(SPIC_CMD_CODE_READ_EXTENDED_ADDR_REG == cmd)
    {
        spic_base_config.cmd_md = MD_SEL_LINE_1;
        spic_base_config.data_md = MD_SEL_LINE_1;
    }
    #endif

    if(spic_read_by_cpu(spic,&spic_base_config,value,value_len))
    {
        return RETURN_ERR;
    }

    return RETURN_OK;
}

#if FLASH_MSF
/**
 * @brief 读扩展地址寄存器
 *
 * @param spic spiflash控制器组
 * @param cmd 命令
 * @param reg 寄存器值(flash地址的（24 ~ 31）bit,用来访问16M以上的FLASH)
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_read_extend_addr_register(spic_base_t spic,uint8_t* reg)
{
    if(spic_read_cmd_value(spic, SPIC_CMD_CODE_READ_EXTENDED_ADDR_REG, reg, 1))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

/**
 * @brief 写扩展地址寄存器
 *
 * @param spic spiflash控制器组
 * @param reg 寄存器值(flash地址的（24 ~ 31）bit,用来访问16M以上的FLASH)
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_write_extend_addr_register(spic_base_t spic,uint8_t reg)
{
    ifspic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = MD_SEL_LINE_1;
    spic_base_config.cmd0 = SPIC_CMD_CODE_WRITE_EXTENDED_ADDR_REG;
    if(spic_write_by_reg(spic,&spic_base_config,reg,1))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif

static int32_t spic_quad_enable(spic_base_t spic,uint8_t cmd_md, uint8_t data_md)
{
    spic_cmd_code_t cmd = SPIC_CMD_CODE_DISABLE_QUAD;
    if(cmd_md)
    {
    	spic_cmd_quad_flag = MD_SEL_LINE_1;     //用单线模式发命令去切换成4线模式
    	cmd = SPIC_CMD_CODE_ENABLE_QUAD;
    }
    else
    {
    	spic_cmd_quad_flag = MD_SEL_LINE_4;     //用4线模式发命令去切换成单线模式
        cmd = SPIC_CMD_CODE_DISABLE_QUAD;
    }
    if(spic_send_cmd(spic, cmd))
    {
        return RETURN_ERR;
    }
    spic_cmd_quad_flag = cmd_md;                //配置后续命令模式
    spic_data_quad_flag = data_md;              //配置后续数据模式
    return RETURN_OK;
}

/**
 * @brief 读取状态寄存器
 *
 * @param spic spiflash控制器组
 * @param reg 状态寄存器
 * @param status 读取到的状态值
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_read_status_register(spic_base_t spic,spic_cmd_code_t reg,uint8_t *status)
{
    if(spic_read_cmd_value(spic, reg, status, 1))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

/**
 * @brief 检查BUSY状态
 *
 * @param spic spiflash控制器组
 * @param timeout 超时时间
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_check_busy(spic_base_t spic,int32_t timeout)
{
    uint8_t status;
    do
    {
        if(RETURN_ERR == spic_read_status_register(spic,
                                                   SPIC_CMD_CODE_READSTATUSREG1,&status))
        {
            return RETURN_ERR;
        }
    } while ((status & (0x1 << 0)) && timeout--);
    if(0 >= timeout)
    {
        return RETURN_ERR;
    }
    else
    {
        return RETURN_OK;
    }
}

/**
 * @brief 写状态寄存器
 *
 * @param spic spiflash控制器组
 * @param reg1 状态寄存器1的值
 * @param reg2 状态寄存器2的值
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_write_status_register(spic_base_t spic,char reg1,char reg2,char reg3)
{
    uint8_t temp[3] = {0};
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    temp[0] = reg1;
    temp[1] = reg2;
    temp[2] = reg3;
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.cmd0 = SPIC_CMD_CODE_WRSTATUSREG;
    if(spic_write_by_cpu(spic,&spic_base_config,temp,2))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config.cmd0 = SPIC_CMD_CODE_WRSTATUSREG2;
    if(spic_write_by_cpu(spic,&spic_base_config,&temp[1],1))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config.cmd0 = SPIC_CMD_CODE_WRSTATUSREG3;
    if(spic_write_by_cpu(spic,&spic_base_config,&temp[2],1))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

/**
 * @brief 读取Unique ID
 *
 * @param spic spiflash控制器组
 * @param unique 华邦：64bit，GD：128bit
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_read_unique_id(spic_base_t spic,uint8_t* unique)
{
    //当配置成4线命令、4线数据时，读Unique ID必须切成单线模式
    spic_quad_enable(QSPI0,MD_SEL_LINE_1,MD_SEL_LINE_1);
    uint8_t temp[20] = {0};
    if(spic_read_cmd_value(spic, SPIC_CMD_CODE_READ_UNIQUE_ID, temp, 20))
    {
        return RETURN_ERR;
    }
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)unique,(void*)temp,16);
    spic_quad_enable(QSPI0, SPIFLASH_CMD_QUAD, SPIFLASH_DATA_QUAD);
    return RETURN_OK;
}

/**
 * @brief 读取jedec ID
 *
 * @param spic spiflash控制器组
 * @param jedec
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_read_jedec_id(spic_base_t spic,uint8_t* jedec)
{
    uint8_t temp[4] = {0};
    if(spic_read_cmd_value(spic, SPIC_CMD_CODE_READJEDECID, temp, 4))
    {
        return RETURN_ERR;
    }
    jedec[0] = temp[3];
    jedec[1] = temp[2];
    jedec[2] = temp[1];
    return RETURN_OK;
}

/**
 * @brief 设置FLASH四线模式
 *
 * @param spic spiflash控制器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_quad_mode(spic_base_t spic)
{
    uint8_t status;
    uint8_t reg1,reg2,reg3;
    if(RETURN_ERR == spic_read_status_register(spic,SPIC_CMD_CODE_READSTATUSREG1,&status))
    {
        return RETURN_ERR;
    }
    reg1 = status;
    if(RETURN_ERR == spic_read_status_register(spic,SPIC_CMD_CODE_READSTATUSREG2,&status))
    {
        return RETURN_ERR;
    }
    reg2 = status;
    if(RETURN_ERR == spic_read_status_register(spic,SPIC_CMD_CODE_READSTATUSREG3,&status))
    {
        return RETURN_ERR;
    }
    reg3 = status;
    if(0 == (reg2 & (0x1 << 1)))
    {
        reg2 |= (0x1 << 1);
        if(RETURN_ERR == spic_write_status_register(spic,reg1,reg2,reg3))
        {
            return RETURN_ERR;
        }
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

/**
 * @brief powerdown
 *
 * @param spic spiflash控制器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_powerdown(spic_base_t spic)
{
    volatile int32_t timeout;
    if(spic_send_cmd(spic, SPIC_CMD_CODE_POWERDOWN))
    {
        return RETURN_ERR;
    }
    timeout = 0x5FFF;    //延时等待配置成功
    while(timeout--);
    return RETURN_OK;
}

/**
 * @brief releasepowerdown
 *
 * @param spic spiflash控制器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t spic_releasepowerdown(spic_base_t spic)
{
    uint8_t ID = 0;
    volatile int32_t timeout;
    if(spic_read_cmd_value(spic, SPIC_CMD_CODE_RELEASEPOWERDOWN, &ID, 1))
    {
        return RETURN_ERR;
    }
    timeout = 0x5FFF;
    while(timeout--);
    return RETURN_OK;
}

/**
 * @brief reset flash
 *
 * @param spic spiflash控制器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_reset(spic_base_t spic)
{
    volatile int32_t timeout;
    if(spic_send_cmd(spic, SPIC_CMD_CODE_ENABLERESET))
    {
        return RETURN_ERR;
    }
    if(spic_send_cmd(spic, SPIC_CMD_CODE_RESET))
    {
        return RETURN_ERR;
    }
    timeout = 0x3D70;
    while(timeout--);
    spic_cmd_quad_flag = 0;
    if(spic_send_cmd(spic, SPIC_CMD_CODE_ENABLERESET))
    {
        return RETURN_ERR;
    }
    if(spic_send_cmd(spic, SPIC_CMD_CODE_RESET))
    {
        return RETURN_ERR;
    }
    timeout = 0x3D70;
    while(timeout--);
    return RETURN_OK;
}

/**
 * @brief SPIflash保护设置
 *
 * @param spic spiflash控制器
 * @param cmd 保护开关
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_protect(spic_base_t spic,FunctionalState cmd)
{
    uint8_t status_reg;
    int32_t reg1 = 0,reg2 = 0,reg3 = 0,srp1 = 0,srp0 = 0,ret = 0;
    volatile spic_status_protect_t status = SPIC_RESV;
    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG1,&status_reg))
    {
        return RETURN_ERR;
    }
    reg1 = status_reg;

    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG2,&status_reg))
    {
        return RETURN_ERR;
    }
    reg2 = status_reg;

    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG3,&status_reg))
    {
        return RETURN_ERR;
    }
    reg3 = status_reg;

    srp0 = (reg1 &(1 << 7))?1:0;
    srp1 = (reg2 &(1 << 0))?1:0;
    status =(spic_status_protect_t)((srp1 << 1) | (srp0 << 0));

    switch (status)
    {
        case SPIC_SOFTWAREPROTECTION:
            ret=0;/*可以正常写，必须发写使能后*/
            break;
        case SPIC_HARDWAREPROTECTION:
            ret=1;/*硬件保护控制，如果/WP 引脚为低电平，不能写*/
            break;
        case SPIC_POWERSUPPLYLOCK_DOWN:
            ret=2;/*直到下一个 power-down power-up时序后，才能被写*/
            if(RETURN_ERR == spic_powerdown(spic))
            {
                return RETURN_ERR;
            }
            if(RETURN_ERR == spic_releasepowerdown(spic))
            {
                return RETURN_ERR;
            }
            break;
        case SPIC_ONETIMEPROGRAM:
            ret=3;/*一次性编程保护，不能再写*/
            return ret;
        default:
            ret=0XFF;
            break;
    }
    if(cmd != ENABLE)
    {
          reg1 &= ~(0x7 << 2);//block protec disable
          reg1 &= ~(0x1 << 5);//Top/Bottom Block protect disbale
          reg1 &= ~(0x1 << 6);//Sector /SEC
          reg2 &= ~(0x1 << 6);//CMP =0
          reg2 &= ~(0x7 << 3);//BP2,BP1,BP0
    }
    else
    {
          reg1 &= ~(0xF << 2);
          reg1 |= (0xB << 2);//保护前256K
          reg2 &= ~(0x1 << 6);//CMP =0
    }
    reg3 &= ~(0x3 << 5);
    reg3 |= (spi_flash_drv0_drv1 << 5);

    if(RETURN_ERR == spic_write_status_register(spic,reg1,reg2,reg3))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    return ret;
}

/**
 * @brief 擦除FLASH安全寄存器
 *
 * @param spic spiflash控制器
 * @param reg 擦除寄存器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_erase_security_reg(spic_base_t spic,spic_security_reg_t reg)
{
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = MD_SEL_LINE_1;
    spic_base_config.data_md = MD_SEL_LINE_1;
    spic_base_config.cmd0 = SPIC_CMD_CODE_ERASE_SECURITY_REG;
    switch (reg)
    {
        case SPIC_SECURITY_REG1:
            spic_base_config.addr = 0x1000;
            break;
        case SPIC_SECURITY_REG2:
            spic_base_config.addr = 0x2000;
            break;
        case SPIC_SECURITY_REG3:
            spic_base_config.addr = 0x3000;
            break;
        default:
            break;
    }
    spic_base_config.addr_en = 1;
    if(spic_cmd(spic,&spic_base_config))
    {
        return RETURN_ERR;
    }
    return spic_check_busy(spic, OTHER_TIMR* SPIC_MS);
}

/**
 * @brief 写FLASH的安全寄存器
 *
 * @param spic spiflash控制器
 * @param reg 擦除寄存器
 * @param buf mem地址
 * @param addr FLASH寄存器地址：华邦(0 - 256),GD(0 - 1024)
 * @param size 写FLASH的字节数：华邦(0 - 256),GD(0 - 1024)
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_write_security_reg(spic_base_t spic,spic_security_reg_t reg,
                              uint32_t buf,uint32_t addr,uint32_t size)
{
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = MD_SEL_LINE_1;
    spic_base_config.data_md = MD_SEL_LINE_1;
    spic_base_config.cmd0 = SPIC_CMD_CODE_WRITE_SECURITY_REG;
    switch (reg)
    {
        case SPIC_SECURITY_REG1:
            spic_base_config.addr = 0x1000 | addr;
            break;
        case SPIC_SECURITY_REG2:
            spic_base_config.addr = 0x2000 | addr;
            break;
        case SPIC_SECURITY_REG3:
            spic_base_config.addr = 0x3000 | addr;
            break;
        default:
            break;
    }
    spic_base_config.addr_en = 1;
    if(spic_write_by_cpu(spic,&spic_base_config,(uint8_t *)buf,size))
    {
        return RETURN_ERR;
    }
    return spic_check_busy(spic, OTHER_TIMR * SPIC_MS);
}

/**
 * @brief 读FLASH的安全寄存器
 *
 * @param spic spiflash控制器
 * @param reg 擦除寄存器
 * @param buf mem地址
 * @param addr FLASH寄存器地址：华邦(0 - 256),GD(0 - 1024)
 * @param size 写FLASH的字节数：华邦(0 - 256),GD(0 - 1024)
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_read_security_reg(spic_base_t spic,spic_security_reg_t reg,
                              uint32_t buf,uint32_t addr,uint32_t size)
{
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = MD_SEL_LINE_1;
    spic_base_config.data_md = MD_SEL_LINE_1;
    spic_base_config.cmd0 = SPIC_CMD_CODE_READ_SECURITY_REG;
    switch (reg)
    {
        case SPIC_SECURITY_REG1:
            spic_base_config.addr = 0x1000 | addr;
            break;
        case SPIC_SECURITY_REG2:
            spic_base_config.addr = 0x2000 | addr;
            break;
        case SPIC_SECURITY_REG3:
            spic_base_config.addr = 0x3000 | addr;
            break;
        default:
            break;
    }
    spic_base_config.addr_en = 1;
    spic_base_config.dummy = 8;
    spic_base_config.dummy_en = 1;
    if(spic_read_by_cpu(spic,&spic_base_config,(uint8_t *)buf,size))
    {
        return RETURN_ERR;
    }
    return spic_check_busy(spic,OTHER_TIMR * SPIC_MS);
}

/**
 * @brief FLASH的安全寄存器上锁,慎用:上锁之后将导致该安全寄存器不可再此编程
 *
 * @param spic spiflash控制器
 * @param reg 寄存器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_security_reg_lock(spic_base_t spic,spic_security_reg_t reg)
{
    uint8_t status;
    uint32_t lock = 0;
    switch (reg)
    {
        case SPIC_SECURITY_REG1:
            lock = 0x1 << 3;
            break;
        case SPIC_SECURITY_REG2:
            lock = 0x1 << 4;
            break;
        case SPIC_SECURITY_REG3:
            lock = 0x1 << 5;
            break;
        default:
            break;
    }
    int32_t reg1,reg2,reg3;
    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG1,&status))
    {
        return RETURN_ERR;
    }
    reg1 = status;
    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG2,&status))
    {
        return RETURN_ERR;
    }
    reg2 = status;
    if(RETURN_ERR == spic_read_status_register(spic,
                                               SPIC_CMD_CODE_READSTATUSREG3,&status))
    {
        return RETURN_ERR;
    }
    reg3 = status;
    reg2 |= lock;
    if(RETURN_ERR == spic_write_status_register(spic,reg1,reg2,reg3))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

/**
 * @brief 擦除FLASH
 *
 * @param spic spiflash控制器
 * @param code 擦除命令
 * @param addr 擦除地址
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_erase(spic_base_t spic,spic_cmd_code_t code,uint32_t addr)
{
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.cmd0 = code;
    spic_base_config.addr = addr;
    spic_base_config.addr_en = 1;
    if(spic_cmd(spic,&spic_base_config))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

#if SPIC_DMA_MODEFI
void spic_dma_init(DMAC_FLOWCTRL flowctrl)
{
    dma_config_temp config;
    config.flowctrl = flowctrl;
    config.busrtsize = BURSTSIZE1;
    config.transferwidth = TRANSFERWIDTH_32b;
    extern DMACChannelx dmachanel;
    spic_dma_config_temp(dmachanel,&config);
}
#endif //#if SPIC_DMA_MODEFI

/**
 * @brief 写FLASH的某一页
 *
 * @param spic spiflash控制器
 * @param buf mem地址
 * @param addr FLASH地址
 * @param size 写FLASH的字节数
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_quad_write_page(spic_base_t spic,uint32_t buf,uint32_t addr,
                                    uint32_t size)
{
    if(spic_send_cmd(spic, SPIC_CMD_CODE_WRITE_ENABLE))
    {
        return RETURN_ERR;
    }
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.data_md = spic_data_quad_flag;   
    if((spic_cmd_quad_flag == 0) && spic_data_quad_flag)
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_QUADINPUTPAGEPROGRAM;      //单线命令、4线写
    }
    else
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_PAGEPROGRAM;
    }
    spic_base_config.addr = addr;
    spic_base_config.addr_en = 1;
#if SPIFLASH_CPU
    if(spic_write_by_cpu(spic,&spic_base_config,(uint8_t*)buf,size))
    {
        return RETURN_ERR;
    }
#else
	if(spic_write_by_dma(spic,&spic_base_config,(uint8_t *)buf,size))
	{
		return RETURN_ERR;
	}
#endif
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
}

#if SPIFLASH_NORMAL_READ
/**
 * @brief 读FLASH的某一块
 *
 * @param spic spiflash控制器
 * @param buf mem地址
 * @param addr FLASH地址
 * @param size 读FLASH的字节数
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t spic_quad_read_page(spic_base_t spic,uint32_t buf,uint32_t addr,
                                    uint32_t size)
{
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.data_md = spic_data_quad_flag;   
    if((spic_cmd_quad_flag == 0) && spic_data_quad_flag)
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_FASTREADQUADOUTPUT;  //单线命令时，数据需用这个命令
    }
    else
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_FASTREAD;
    }
    spic_base_config.addr = addr;
    spic_base_config.addr_en = 1;
    if(MD_SEL_LINE_1 == spic_base_config.cmd_md)
    {
    	spic_base_config.dummy = 8;
    }
    else if(MD_SEL_LINE_4 == spic_base_config.cmd_md)
    {
    	spic_base_config.dummy = 2;
    }
    spic_base_config.dummy_en = 1;
#if SPIFLASH_CPU
    if(spic_read_by_cpu(spic,&spic_base_config,(uint8_t*)buf,size))
    {
        return RETURN_ERR;
    }
#else
    if(spic_read_by_dma(spic,&spic_base_config,(uint8_t *)buf,size))
	{
		return RETURN_ERR;
	}
#endif
    if(RETURN_ERR == spic_check_busy(spic,OTHER_TIMR * SPIC_MS))
    {
	   return RETURN_ERR;
    }
    return RETURN_OK;
}
#endif

int32_t spic_xipconfig(spic_base_t spic)
{
    spic_change_clk(spic,SPIFLASH_XIP_READ_CLK);
    spic_base_config_t spic_base_config = {0};
    spic_base_config.cmd_md = spic_cmd_quad_flag;
    spic_base_config.data_md = spic_data_quad_flag;   
    if((spic_cmd_quad_flag == 0) && spic_data_quad_flag)
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_FASTREADQUADOUTPUT;  //单线命令时，数据需用这个命令
    }
    else
    {
        spic_base_config.cmd0 = SPIC_CMD_CODE_FASTREAD;
    }
    spic_base_config.addr_en = 1;
    if(MD_SEL_LINE_1 == spic_base_config.cmd_md)
    {
    	spic_base_config.dummy = 8;
    }
    else if(MD_SEL_LINE_4 == spic_base_config.cmd_md)
    {
    	spic_base_config.dummy = 2;
    }
    spic_base_config.dummy_en = 1;
    spic_xip_config(spic,&spic_base_config);

    return RETURN_OK;
}
/**
 * @brief FLASH初始化
 *
 * @param spic spiflash控制器
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t flash_init(spic_base_t spic)
{
    // scu_set_device_reset(HAL_GDMA_BASE);           //DMA硬件复位
    // scu_set_device_reset_release(HAL_GDMA_BASE);    //DMA复位完成
    scu_set_device_gate(HAL_GDMA_BASE,ENABLE);      //打开DMA时钟
    eclic_irq_enable(DMA_IRQn);              //DMA中断使能
    extern DMACChannelx dmachanel;
    clear_dma_translate_flag(dmachanel);      //DMA传输完成标志清除，DMA的通道0为DTR FLASH所用

    scu_set_device_reset((uint32_t)spic);         //DTRFLASH控制器硬件复位
    scu_set_device_reset_release((uint32_t)spic);  //DTRFLASH控制器复位完成

    scu_set_div_parameter(HAL_DTRFLASH_BASE,0);
    scu_set_div_parameter(HAL_DTRFLASH_RAM_BASE,0);

    scu_set_device_gate((uint32_t)spic,ENABLE);    //打开DTRFLASH控制器时钟
    //scu_spiflash_no_boot_set();                 //预留
    #if !SPIFLASH_CPU
    #if SPIC_DMA_MODEFI
    spic_dma_stady_init(dmachanel);
    #endif
    #endif
    spic_init_t init;
    init.flash_clk_div = SPIFLASH_NORMAL_WRITE_READ_CLK;
    init.flash_sel = 0;
    init.addr_size = 3;
    spic_init(spic,&init);
    // spic_clk_phase_set(spic,0,0,0,0);
    spic_clk_phase_set(spic,0,0,0,RX_NAGE_EN); //puya的快速FLASH需要调整该配置
    //1、掉电
    //先用单线写一次掉电、复位
    if(RETURN_ERR == spic_powerdown(spic))       
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_releasepowerdown(spic))
    {
        return RETURN_ERR;
    }
    //再用4线写一次掉电、复位
    spic_cmd_quad_flag = 1;              
    if(RETURN_ERR == spic_powerdown(spic))       
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_releasepowerdown(spic))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == spic_reset(spic))
    {
        return RETURN_ERR;
    }         
    if(RETURN_ERR == spic_protect(spic,ENABLE))
    {
        return RETURN_ERR;
    }
    init.flash_clk_div = SPIFLASH_NORMAL_WRITE_READ_CLK;
    init.flash_sel = 0;
    init.addr_size = 3;
    spic_init(spic,&init);
    // spic_clk_phase_set(spic,0,0,0,0);
    spic_clk_phase_set(spic,0,0,0,RX_NAGE_EN); //puya的快速FLASH需要调整该配置
    if(RETURN_ERR == spic_quad_mode(spic))
	{
		return RETURN_ERR;
	}
	spic_quad_enable(QSPI0, SPIFLASH_CMD_QUAD, SPIFLASH_DATA_QUAD);
    return RETURN_OK;
}

/**
 * @brief FLASH擦除
 *
 * @param spic spiflash控制器
 * @param addr 地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t flash_policy_erase(spic_base_t spic,uint32_t addr,uint32_t size)
{
    /* other erase */
    uint32_t current_addr = addr;
    uint32_t current_size = size;
    do
    {
        if((0 == (current_addr % ERASE_64K)) && (size >= ERASE_64K))
        {
            current_size = ERASE_64K;
            if(RETURN_ERR == spic_erase(spic,SPIC_CMD_CODE_BLOCKERASE64K,
                                        current_addr))
            {
                return RETURN_ERR;
            }
        }
        else if((0 == (current_addr % ERASE_4K)) && (size >= ERASE_4K))
        {
            current_size = ERASE_4K;
            if(RETURN_ERR == spic_erase(spic,SPIC_CMD_CODE_SECTORERASE4K,
                                        current_addr))
            {
                return RETURN_ERR;
            }
        }
        else
        {
            return RETURN_ERR;
        }
        size -= current_size;
        current_addr += current_size;
    }
    while(size > 0);
    return RETURN_OK;
}

/**
 * @brief spiflash 写规则
 *
 * @param spic spiflash控制器
 * @param buf mem地址
 * @param addr flash地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t flash_write_rule(spic_base_t spic,uint32_t buf,uint32_t addr,
                                uint32_t size)
{
#if 1 
    uint32_t write_64word_count = size / 256;
    uint32_t write_1word_count = (size % 256) / 4;
    #if !SPIFLASH_CPU
    #if SPIC_DMA_MODEFI
    spic_dma_init(M2P_DMA);
    #endif
    #endif
    for(int i = 0;i < write_64word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,256))
        {
            return RETURN_ERR;
        }
        buf += 256;
        addr += 256;
    }
    for(int i = 0;i < write_1word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,4))
        {
            return RETURN_ERR;
        }
        buf += 4;
        addr += 4;
    }
#else
    uint32_t write_64word = 64 * 4,write_64word_count = 0;
    uint32_t write_32word = 32 * 4,write_32word_count = 0;
    uint32_t write_16word = 16 * 4,write_16word_count = 0;
    uint32_t write_8word  = 8 * 4,write_8word_count = 0;
    uint32_t write_4word  = 4 * 4,write_4word_count = 0;
    uint32_t write_1word  = 4,write_1word_count = 0;
    write_64word_count = size / write_64word;
    write_32word_count = (size % write_64word) / write_32word;
    write_16word_count = (size % write_32word) / write_16word;
    write_8word_count = (size % write_16word) / write_8word;
    write_4word_count = (size % write_8word) / write_4word;
    write_1word_count = (size % write_4word) / write_1word;
    #if !SPIFLASH_CPU
    #if SPIC_DMA_MODEFI
    spic_dma_init(M2P_DMA);
    #endif
    #endif
    for(int i = 0;i < write_64word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_64word))
        {
            return RETURN_ERR;
        }
        buf += write_64word;
        addr += write_64word;
    }
    for(int i = 0;i < write_32word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_32word))
        {
            return RETURN_ERR;
        }
        buf += write_32word;
        addr += write_32word;
    }
    for(int i = 0;i < write_16word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_16word))
        {
            return RETURN_ERR;
        }
        buf += write_16word;
        addr += write_16word;
    }
    for(int i = 0;i < write_8word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_8word))
        {
            return RETURN_ERR;
        }
        buf += write_8word;
        addr += write_8word;
    }
    for(int i = 0;i < write_4word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_4word))
        {
            return RETURN_ERR;
        }
        buf += write_4word;
        addr += write_4word;
    }
    for(int i = 0;i < write_1word_count;i++)
    {
        if(RETURN_ERR == spic_quad_write_page(spic,buf,addr,write_1word))
        {
            return RETURN_ERR;
        }
        buf += write_1word;
        addr += write_1word;
    }
    return RETURN_OK;
#endif
}

/**
 * @brief 写FLASH
 *
 * @param spic spiflash控制器
 * @param addr FLASH地址
 * @param buf mem地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t flash_quad_write(spic_base_t spic,uint32_t addr,uint32_t buf, uint32_t size)
{
    uint8_t temp[256] = {0};
    uint32_t size_word = 0;
    uint32_t page_size_head = 0,page_size_tail = 0,page_count = 0;
    page_size_head = addr % FLASH_PAGE_SIZE;
    if(0 != page_size_head)
    {
        page_size_head = FLASH_PAGE_SIZE - page_size_head;
    }
    if(page_size_head > size)
    {
        page_size_head = size;
    }
    page_size_tail = ((size - page_size_head) % FLASH_PAGE_SIZE);
    page_count = ((size - page_size_head) / FLASH_PAGE_SIZE);
    if(0 != page_size_head)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)temp,(void*)buf,page_size_head);
        if(page_size_head % 4)
        {
            size_word = ((page_size_head + 4) / 4) * 4;
        }
        else
        {
            size_word = page_size_head;
        }
        for(int i = 0;i < (size_word - page_size_head);i++)
        {
            temp[page_size_head + i] = 0xFF;
        }
        if(RETURN_ERR == flash_write_rule(spic,(uint32_t)temp,addr,size_word))
        {
            //ci_logerr(LOG_QSPI_DRIVER,"qspi write error\n");
            return RETURN_ERR;
        }
        buf += page_size_head;
        addr += page_size_head;
    }
    for(int i = 0;i < page_count;i++)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)temp,(void*)buf,FLASH_PAGE_SIZE);
        if(RETURN_ERR == flash_write_rule(spic,(uint32_t)temp,addr,FLASH_PAGE_SIZE))
        {
            //ci_logerr(LOG_QSPI_DRIVER,"qspi write error\n");
            return RETURN_ERR;
        }
        buf += FLASH_PAGE_SIZE;
        addr += FLASH_PAGE_SIZE;
    }
    if(0 != page_size_tail)
    {
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)temp,(void*)buf,page_size_tail);
        if(page_size_tail % 4)
        {
            size_word = ((page_size_tail + 4) / 4) * 4;
        }
        else
        {
            size_word = page_size_tail;
        }
        for(int i = 0;i < (size_word - page_size_tail);i++)
        {
            temp[page_size_tail + i] = 0xFF;
        }
        if(RETURN_ERR == flash_write_rule(spic,(uint32_t)temp,addr,size_word))
        {
            //ci_logerr(LOG_QSPI_DRIVER,"qspi write error\n");
            return RETURN_ERR;
        }
    }
    return RETURN_OK;
}

#if SPIFLASH_NORMAL_READ
/**
 * @brief 读FLASH
 *
 * @param spic spiflash控制器
 * @param buf mem地址
 * @param addr FLASH地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
static int32_t flash_quad_read(spic_base_t spic,uint32_t buf,uint32_t addr,
                        uint32_t size)
{
    uint8_t temp[256] = {0};
    uint32_t size_word = 0;
    uint32_t block_count = 0;
    uint32_t block_size_tail = 0;

    uint32_t size_start = addr % 4;   
    uint32_t addr_start = addr & 0xFFFFFC;

    #if !SPIFLASH_CPU
    #if SPIC_DMA_MODEFI
    spic_dma_init(P2M_DMA);
    #endif
    #endif

    if(0 != size_start)
    {
        if(RETURN_ERR == spic_quad_read_page(spic,(uint32_t)temp,addr_start,4))  //最少按4字节读取
        {
            return RETURN_ERR;
        }
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)buf,(void*)temp+size_start,4-size_start);
        buf += (4-size_start);
        addr += (4-size_start);
        size -= (4-size_start);
    }

    block_count = size / FLASH_PAGE_SIZE;
    block_size_tail = size % FLASH_PAGE_SIZE;

    for(int i = 0;i < block_count;i++)
    {
        if(RETURN_ERR == spic_quad_read_page(spic,(uint32_t)temp,addr,FLASH_PAGE_SIZE))
        {
            //ci_logerr(LOG_QSPI_DRIVER,"qspi read error\n");
            return RETURN_ERR;
        }
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)buf,(void*)temp,FLASH_PAGE_SIZE);
        buf += FLASH_PAGE_SIZE;
        addr += FLASH_PAGE_SIZE;
    }
    if(0 != block_size_tail)
    {
        if(block_size_tail % 4)
        {
            size_word = (block_size_tail + 3) & 0xFFFFFFFC;
        }
        else
        {
            size_word = block_size_tail;
        }
        if(RETURN_ERR == spic_quad_read_page(spic,(uint32_t)temp,addr,size_word))
        {
            //ci_logerr(LOG_QSPI_DRIVER,"qspi read error\n");
            return RETURN_ERR;
        }
        MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void*)buf,(void*)temp,block_size_tail);
    }
    return RETURN_OK;
}
#endif

/**
 * @brief FLASH擦除,兼容32M以上Flash
 *
 * @param spic spiflash控制器
 * @param addr 地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t flash_erase(spic_base_t spic,uint32_t addr,uint32_t size)
{
#if FLASH_MSF
    uint32_t extend_addr,flash_addr;
    extend_addr = addr >> 24;
    flash_addr = 0xFFFFFF & addr;
    if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
    {
        return RETURN_ERR;
    }
    if(RETURN_ERR == flash_policy_erase(spic,flash_addr,size))
    {
        return RETURN_ERR;
    }
    return RETURN_OK;
#else
    return flash_policy_erase(spic,addr,size);
#endif
}

/**
 * @brief 写FLASH,兼容32M以上Flash
 *
 * @param spic spiflash控制器
 * @param addr FLASH地址
 * @param buf mem地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t flash_write(spic_base_t spic,uint32_t addr,uint32_t buf,uint32_t size)
{
    spic_change_clk(spic, SPIFLASH_NORMAL_WRITE_READ_CLK);
#if FLASH_MSF
    uint32_t extend_addr,mem_addr,flash_addr,write_size,residue_size,block_count;
    residue_size = size;
    mem_addr = buf;
    extend_addr = addr >> 24;
    flash_addr = 0xFFFFFF & addr;
    write_size = FLASH_SISZE_16M - flash_addr;
    if(write_size < residue_size)
    {
        /* 有跨界情况 */
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_write(spic,flash_addr,mem_addr,write_size))
        {
            return RETURN_ERR;
        }
        flash_addr += write_size;
        mem_addr += write_size;
        residue_size -= write_size;
        extend_addr += 1;
    }
    else
    {
        /* 无跨界情况 */
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_write(spic,flash_addr,mem_addr,residue_size))
        {
            return RETURN_ERR;
        }
        return RETURN_OK;
    }
    /* 计算剩余多少块 */
    block_count = residue_size / FLASH_SISZE_16M;
    for(int i = 0;i < block_count;i++)
    {
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_write(spic,flash_addr,mem_addr,FLASH_SISZE_16M))
        {
            return RETURN_ERR;
        }
        flash_addr += FLASH_SISZE_16M;
        mem_addr += FLASH_SISZE_16M;
        residue_size -= FLASH_SISZE_16M;
        extend_addr += 1;
    }
    /* 是否有剩余不满一块的数据 */
    if(residue_size)
    {
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_write(spic,flash_addr,mem_addr,residue_size))
        {
            return RETURN_ERR;
        }
    }
    return RETURN_OK;
#else
    return flash_quad_write(spic,addr,buf,size);
#endif
}

#if SPIFLASH_NORMAL_READ
/**
 * @brief 读FLASH,兼容32M以上Flash
 *
 * @param spic spiflash控制器
 * @param buf mem地址
 * @param addr FLASH地址
 * @param size 大小
 *
 * @retval RETURN_OK
 * @retval RETURN_ERR
 */
int32_t flash_read(spic_base_t spic,uint32_t buf,uint32_t addr,uint32_t size)
{
    spic_change_clk(spic, SPIFLASH_NORMAL_WRITE_READ_CLK);
#if FLASH_MSF
    uint32_t extend_addr,mem_addr,flash_addr,write_size,residue_size,block_count;
    residue_size = size;
    mem_addr = buf;
    extend_addr = addr >> 24;
    flash_addr = 0xFFFFFF & addr;
    write_size = FLASH_SISZE_16M - flash_addr;
    if(write_size < residue_size)
    {
        /* 有跨界情况 */
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_read(spic,mem_addr,flash_addr,write_size))
        {
            return RETURN_ERR;
        }
        flash_addr += write_size;
        mem_addr += write_size;
        residue_size -= write_size;
        extend_addr += 1;
    }
    else
    {
        /* 无跨界情况 */
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_read(spic,mem_addr,flash_addr,residue_size))
        {
            return RETURN_ERR;
        }
        return RETURN_OK;
    }
    /* 计算剩余多少块 */
    block_count = residue_size / FLASH_SISZE_16M;
    for(int i = 0;i < block_count;i++)
    {
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_read(spic,mem_addr,flash_addr,FLASH_SISZE_16M))
        {
            return RETURN_ERR;
        }
        flash_addr += FLASH_SISZE_16M;
        mem_addr += FLASH_SISZE_16M;
        residue_size -= FLASH_SISZE_16M;
        extend_addr += 1;
    }
    /* 是否有剩余不满一块的数据 */
    if(residue_size)
    {
        if(RETURN_ERR == spic_write_extend_addr_register(spic,extend_addr))
        {
            return RETURN_ERR;
        }
        if(RETURN_ERR == flash_quad_read(spic,mem_addr,flash_addr,residue_size))
        {
            return RETURN_ERR;
        }
    }
    return RETURN_OK;
#else
    return flash_quad_read(spic,buf,addr,size);
#endif
}
#endif

