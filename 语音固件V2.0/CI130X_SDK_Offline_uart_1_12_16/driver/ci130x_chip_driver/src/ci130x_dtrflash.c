#include "ci130x_dtrflash.h"
#include "ci130x_system.h"
#include "ci130x_dma.h"
#include "romlib_runtime.h"

#if (0)
extern void dma_channel0_lock(void);
extern void dma_channel0_unlock(void *xHigherPriorityTaskWoken);
#define DMAC_CHANNEL0_LOCK()          dma_channel0_lock();
#define DMAC_CHANNEL0_UNLOCK()        dma_channel0_unlock(NULL);
#else
#define DMAC_CHANNEL0_LOCK()          do{}while(0);
#define DMAC_CHANNEL0_UNLOCK()        do{}while(0);
#endif

#define SPIC_TIMEOUT_ARBT             (5000)
#define SPIC_TIMEOUT_MAIN_CTRL        (5000)
#define SPIC_TIMEOUT_DMA              (5000)

#define SPIC_FIFO_ADDR                (0x50000000)    //cpu在FLASH正常模式下访问该地址、dma在XIP模式下访问该地址
#define SPIC_DMA_FIFO_ADDR            (0x60000000)    //dma在FLASH正常模式下，访问该地址

#define SPIC_CS_HIGH_CYCLE            (1)
#define SPIC_CS_LOW_CYCLE             (1)

#define SPIC_WFIFO_AMFULL_LEVEL       (60)
#define SPIC_RFIFO_AMEMPTY_LEVEL      (60)
#define SPIC_RFIFO_AMFULL_LEVEL       (4)

#define STATE_REG_GM_DONE_INT         (0x1 << 3)

static uint8_t spi_dma_buf[512];/* DMA访问 */

static uint32_t global_flash_sel = 0;
static uint32_t global_addr_size = 0;
DMACChannelx dmachanel = DMACChannel0;

typedef enum/*!< 功能:flash fifo时钟与sram时钟频率关系 */
{
    RAM_CLK_MD_0 = 0,/*!< 功能:同频 */
    RAM_CLK_MD_2 = 1,/*!< 功能:2分频 */
    RAM_CLK_MD_4 = 2,/*!< 功能:4分频 */
}ram_clk_md_t;

typedef volatile struct spic_register
{
    uint32_t globe_cfg;/*!< 属性:RW 偏移:0x00 位宽:32 功能:全局配置寄存器0 */
    uint32_t globe1_cfg;/*!< 属性:RW 偏移:0x04 位宽:32 功能:全局配置寄存器1 */
    uint32_t rx_clk_cfg;/*!< 属性:RW 偏移:0x08 位宽:32 功能:接收时钟配置寄存器 */
    uint32_t addr_mask_cfg;/*!< 属性:RW 偏移:0x0C 位宽:32 功能:地址MASK寄存器 *//*!< 功能:addr_mask[x]:1 = nomask，0 = mask;例:器件大小16MB，器件有效地址24bit,addr_mask = 16'h00FF;器件大小32MB，器件有效地址25bit,addr_mask = 16'h01FF; */
    union/*!< 属性:RW 偏移:0x10 位宽:32 功能:通用模式配置寄存器0 */
    {
        struct
        {
            uint32_t gm_en:             1;/*!< 功能:General mode(通用模式)总使能:0 = 禁止，1 = 使能,写1后，使能生效后该bit位模块自清 */
            uint32_t gm_cmd_en:         1;/*!< 功能:发送命令使能：0 = 禁止，1 = 使能 */
            uint32_t gm_cmd_md_sel:     4;/*!< 功能:发送命令的模式选择 */
            uint32_t gm_addr_en:        1;/*!< 功能:发送地址使能：0 = 禁止，1 = 使能 */
            uint32_t gm_addr_md_sel:    4;/*!< 功能:发送地址的模式选择 */
            uint32_t gm_crb_en:         1;/*!< 功能:Continous Read bit发送使能：0 = 禁止，1 = 使能 */
            uint32_t gm_dummy_en:       1;/*!< 功能:Dummy等待使能：0 = 禁止，1 = 使能 */
            uint32_t gm_read_en:        1;/*!< 功能:读数据使能:0 = 禁止，1 = 使能 */
            uint32_t gm_write_en:       1;/*!< 功能:写数据使能:0 = 禁止，1 = 使能 */
            uint32_t gm_data_md_sel:    4;/*!< 功能:读写数据的模式选择 */
            uint32_t gm_crm_en:         1;/*!< 功能:Continous Read 模式使能:0 = 禁止，1 = 使能 */
            uint32_t gm_flash_sel:      1;/*!< 功能:双flash选择:0 = flash0（单线和4线模式时选择flash0），1 = flash1（单线和4线模式时选择flash1) */
            uint32_t gm_dma_en:         1;/*!< 功能:DMA模式使能：0 = 禁止，1 = 使能 */
            uint32_t gm_dma_wreq_md:    1;/*!< 功能:DMA写请求发送方式:0:wfifo empty时发送写请求，1:wfifo amempty时发送写请求 */
            uint32_t gm_data_store_md:  1;/*!< 功能:读写数据存放方式:0:fifo，1:rw_data_reg(不支持dma模式） */
            uint32_t gm_dummy_cycle:    5;/*!< 功能:DUMMY等待flash的周期:x个周期 */
            uint32_t reserved:          3;/*!< 功能:保留 */
        };
        uint32_t val;
    }gm_cfg;
    uint32_t gm1_cfg;/*!< 属性:RW 偏移:0x14 位宽:32 功能:通用模式配置寄存器1 */
    uint32_t gm_addr_cfg;/*!< 属性:RW 偏移:0x18 位宽:32 功能:通用模式读写地址 */
    uint32_t gm_data_szie_cfg;/*!< 属性:RW 偏移:0x1C 位宽:32 功能:通用模式数据大小配置寄存器 */
    union/*!< 属性:RW 偏移:0x20 位宽:32 功能:读操作模式配置寄存器0 */
    {
        struct
        {
            uint32_t rd_en:             1;/*!< 功能:read mode(读模式)总使能:0 = 禁止，1 = 使能 */
            uint32_t rd_cmd_en:         1;/*!< 功能:发送命令使能:0 = 禁止，1 = 使能 */
            uint32_t rd_cmd_md_sel:     4;/*!< 功能:发送命令的模式选择 */
            uint32_t rd_addr_md_sel:    4;/*!< 功能:发送地址的模式选择 */
            uint32_t rd_crb_en:         1;/*!< 功能:Continous Read bit发送使能:0 = 禁止，1 = 使能 */
            uint32_t rd_dummy_en:       1;/*!< 功能:Dummy等待使能:0 = 禁止，1 = 使能 */
            uint32_t rd_data_md_sel:    4;/*!< 功能:读数据的模式选择 */
            uint32_t rd_flash_sel:      1;/*!< 功能:双flash选择:0 = flash0（单线和4线模式时选择flash0）,1 = flash1（单线和4线模式时选择flash1) */
            uint32_t rd_dummy_cycle:    5;/*!< 功能:DUMMY等待flash的周期:x个周期 */
            uint32_t prefetch_en:       1;/*!< 功能:prefetch read mode(预取读模式)总使能:0 = 禁止，1 = 使能 */
            uint32_t reserved:          9;/*!< 功能:保留 */
        };
        uint32_t val;
    }rd_cfg;
    uint32_t rd1_cfg;/*!< 属性:RW 偏移:0x24 位宽:32 功能:读操作模式配置寄存器1 */
    uint32_t wr_cfg;/*!< 属性:RW 偏移:0x28 位宽:32 功能:写操作模式配置寄存器0(PSRAM用) */
    uint32_t wr1_cfg;/*!< 属性:RW 偏移:0x2C 位宽:32 功能:写操作模式配置寄存器1(PSRAM用) */
    uint32_t fifo_level_cfg;/*!< 属性:RW 偏移:0x30 位宽:32 功能:FIFO水线配置寄存器 */
    uint32_t rw_data0_cfg;/*!< 属性:RW 偏移:0x34 位宽:32 功能:读写数据寄存器0 */
    uint32_t rw_data1_cfg;/*!< 属性:RW 偏移:0x38 位宽:32 功能:读写数据寄存器1 */
    uint32_t int_ctrl_cfg;/*!< 属性:RW 偏移:0x3C 位宽:32 功能:中断控制寄存器 */
    uint32_t state_reg;/*!< 属性:RO 偏移:0x40 位宽:32 功能:状态寄存器 */
}spic_register_t,*spic_register_p;

typedef struct
{
    /*======gm&xip=======*/
    md_sel_t cmd_md_sel;
    md_sel_t addr_md_sel;
    md_sel_t data_md_sel;
    uint32_t cmd_code0;
    uint32_t cmd_code1;
    uint32_t cmd_size;
    uint32_t dummy;
    uint32_t dummy_en;
    /*========gm=========*/
    uint32_t gm_addr;
    uint32_t gm_data_size;
    uint32_t gm_cmd_en;
    uint32_t gm_addr_en;
    uint32_t gm_write_en;
    uint32_t gm_read_en;
    uint32_t gm_dma_en;
    uint32_t gm_data_store_md;
}spic_config_t,*spic_config_p;

void spic_change_clk(uint32_t spic_base,flash_clk_div_t clk)
{
    spic_register_p spic = (spic_register_p)spic_base;
    spic->globe_cfg &= ~(0x3 << 0);
    spic->globe_cfg |= (clk << 0);
}

/**
 * @brief 等待spic空闲
 *
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_wait_idle(uint32_t spic_base)
{
    spic_register_p spic = (spic_register_p)spic_base;
    uint32_t timeout_arbt = SPIC_TIMEOUT_ARBT;
    uint32_t timeout_main_ctrl = SPIC_TIMEOUT_MAIN_CTRL;
    //while((spic->state_reg & (0x1 << 7))  && timeout_arbt--);
    while((spic->state_reg & (0x1 << 8)) && timeout_main_ctrl--);
    if((0 == timeout_arbt) || (0 == timeout_main_ctrl))
    {
        return -1;
    }
    return 0;
}

/**
 * @brief spic是否能够进行模式切换
 *
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_ready_change_mode(uint32_t spic_base)
{
    spic_register_p spic = (spic_register_p)spic_base;
    if(spic_wait_idle(spic_base))
    {
        return -1;
    }
    spic->gm_cfg.gm_en = 0;
    spic->rd_cfg.rd_en = 0;
    //spic->wr_cfg.wr_en = 0;
    if(spic_wait_idle(spic_base))
    {
        return -1;
    }
    return 0;
}

/**
 * @brief 通用模式配置
 *
 * @param config 配置结构体指针
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_general_mode_config(uint32_t spic_base,spic_config_p config)
{
    spic_register_p spic = (spic_register_p)spic_base;
    if(spic_ready_change_mode(spic_base))
    {
        return -1;
    }
    spic->gm_cfg.gm_flash_sel = global_flash_sel;
    spic->gm_cfg.gm_cmd_md_sel = config->cmd_md_sel;
    spic->gm_cfg.gm_addr_md_sel = config->addr_md_sel;
    spic->gm_cfg.gm_data_md_sel = config->data_md_sel;
    uint32_t temp = 0;
    temp |= (config->cmd_code0) << 0;
    temp |= (config->cmd_code1) << 8;
    if(global_addr_size)
    {
    	temp |= (global_addr_size - 1) << 24;
    }
    if(config->cmd_size)
    {
    	temp |= (config->cmd_size - 1) << 26;
    }
    spic->gm1_cfg = temp;
    /*===================gm=========================*/
    spic->gm_addr_cfg = config->gm_addr;
    spic->gm_data_szie_cfg = config->gm_data_size;
    spic->gm_cfg.gm_cmd_en = config->gm_cmd_en;
    spic->gm_cfg.gm_addr_en = config->gm_addr_en;
    spic->gm_cfg.gm_write_en = config->gm_write_en;
    spic->gm_cfg.gm_read_en = config->gm_read_en;
    spic->gm_cfg.gm_dma_en = config->gm_dma_en;
    spic->gm_cfg.gm_data_store_md = config->gm_data_store_md;
    spic->gm_cfg.gm_dummy_cycle = config->dummy - 1;
    spic->gm_cfg.gm_dummy_en = config->dummy_en;
    //总使能最后才能打开
    spic->gm_cfg.gm_en = 1;
    return 0;
}

/**
 * @brief XIP模式读数据
 *
 * @param config 配置结构体指针
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_xip_read_mode_config(uint32_t spic_base,spic_config_p config)
{
    spic_register_p spic = (spic_register_p)spic_base;
    if(spic_ready_change_mode(spic_base))
    {
        return -1;
    }
    spic->rd_cfg.rd_cmd_en = 1;
    spic->rd_cfg.rd_flash_sel = global_flash_sel;
    spic->rd_cfg.rd_cmd_md_sel = config->cmd_md_sel;
    spic->rd_cfg.rd_addr_md_sel = config->addr_md_sel;
    spic->rd_cfg.rd_data_md_sel = config->data_md_sel;
    spic->rd_cfg.rd_dummy_cycle = config->dummy - 1;
    spic->rd_cfg.rd_dummy_en = config->dummy_en;
    spic->rd_cfg.prefetch_en = 1;
    uint32_t temp = 0;
    temp |= (config->cmd_code0) << 0;
    temp |= (config->cmd_code1) << 8;
    if(global_addr_size)
    {
    	temp |= (global_addr_size - 1) << 24;
    }
    if(config->cmd_size)
    {
    	temp |= (config->cmd_size - 1) << 26;
    }
    spic->rd1_cfg = temp;
    //总使能最后才能打开
    spic->rd_cfg.rd_en = 1;
    return 0;
}

#if 0
/**
 * @brief XIP模式写数据(只针对接PSRAM)
 *
 * @param config 配置结构体指针
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_xip_write_mode_config(uint32_t spic_base,spic_config_p config)
{
    spic_register_p spic = (spic_register_p)spic_base;
    if(spic_ready_change_mode(spic_base))
    {
        return -1;
    }
    spic->wr_cfg.wr_flash_sel = global_flash_sel;
    spic->wr_cfg.wr_cmd_md_sel = config->cmd_md_sel;
    spic->wr_cfg.wr_addr_md_sel = config->addr_md_sel;
    spic->wr_cfg.wr_data_md_sel = config->data_md_sel;
    spic->wr_cfg.wr_dummy_cycle = config->dummy - 1;
    spic->wr_cfg.wr_dummy_en = config->dummy_en;
    uint32_t temp = 0;
    temp |= (config->cmd_code0) << 0;
    temp |= (config->cmd_code1) << 8;
    if(global_addr_size)
    {
    	temp |= (global_addr_size - 1) << 24;
    }
    if(config->cmd_size)
    {
    	temp |= (config->cmd_size - 1) << 26;
    }
    spic->wr1_cfg.val = temp;
    //总使能最后才能打开
    spic->wr_cfg.wr_en = 1;
    return 0;
}
#endif

/**
 * @brief 配置结构体简化
 *
 * @param config 完整的配置结构体指针
 * @param spic_base_config 简化版配置结构体指针
 * @return int32_t 0:成功;1:失败;
 */
static int32_t spic_config_init(spic_config_p config,spic_base_config_p spic_base_config)
{
    if((0x00 == spic_base_config->cmd0) || ((0x00 == spic_base_config->cmd0) && (0x00 == spic_base_config->cmd1)))
    {
        return -1;
    }
    //命令配置
    config->cmd_md_sel = spic_base_config->cmd_md;
    config->addr_md_sel = spic_base_config->cmd_md;
    config->data_md_sel = spic_base_config->data_md;
    config->cmd_code0 = spic_base_config->cmd0;
    config->cmd_code1 = spic_base_config->cmd1;
    config->cmd_size = 1;
    if(0x00 != spic_base_config->cmd1)
    {
        config->cmd_size = 2;
    }
    config->gm_addr = spic_base_config->addr;
    config->gm_addr_en = spic_base_config->addr_en;
    config->dummy = spic_base_config->dummy;
    config->dummy_en = spic_base_config->dummy_en;
    config->gm_cmd_en = 1;
    return 0;
}

/**
 * @brief spic向flash发送命令
 *
 * @param spic_base_config 配置结构体指针
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_cmd(uint32_t spic_base,spic_base_config_p spic_base_config)
{
    spic_register_p spic = (spic_register_p)spic_base;
    spic_config_t config = {0};
    if(spic_config_init(&config,spic_base_config))
    {
        return -1;
    }
    if(spic_general_mode_config(spic_base,&config))
    {
        return -2;
    }
    return 0;
}

/**
 * @brief spic以CPU的方式向flash读数据
 *
 * @param spic_base_config 配置结构体指针
 * @param read_data 数据指针
 * @param read_len 数据长度
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_read_by_cpu(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* read_data,uint32_t read_len)
{
	uint32_t data_reg[2] = {0};
	uint32_t temp = 0;
    spic_register_p spic = (spic_register_p)spic_base;
    spic_config_t config = {0};
    if(spic_config_init(&config,spic_base_config))
    {
        return -2;
    }
    if(read_len > 8)
    {
        config.gm_data_size = (read_len/4)*4;
    }
    else
	{
    	config.gm_data_store_md = 1;
    	config.gm_data_size = ((read_len/4)+1)*4;
	}
    config.gm_read_en = 1;
    spic->state_reg |= STATE_REG_GM_DONE_INT;
    spic->gm_cfg.gm_dma_wreq_md = 1;
    if(spic_general_mode_config(spic_base,&config))
    {
        return -3;
    }
    if(read_len > 8)
    {
    	MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(read_data,(void *)SPIC_FIFO_ADDR,(read_len/4)*4);
    	while(!(spic->state_reg & STATE_REG_GM_DONE_INT));
    	spic->state_reg |= STATE_REG_GM_DONE_INT;
    }
    else
    {
    	while(!(spic->state_reg & STATE_REG_GM_DONE_INT));
    	spic->state_reg |= STATE_REG_GM_DONE_INT;
    	data_reg[0] =  spic->rw_data0_cfg;
		data_reg[1] = spic->rw_data1_cfg;
		spic->rw_data0_cfg = 0xFFFFFFFF;
		spic->rw_data1_cfg = 0xFFFFFFFF;
		temp = data_reg[0];
		data_reg[0] = ((temp&0xFF)<< 24)|((temp&0xFF00)<<8)|((temp&0xFF0000)>>8)|((temp&0xFF000000)>>24);
		temp = data_reg[1];
		data_reg[1] = ((temp&0xFF)<< 24)|((temp&0xFF00)<<8)|((temp&0xFF0000)>>8)|((temp&0xFF000000)>>24);
		MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(read_data,data_reg,read_len);
    }
    if(spic_wait_idle(spic_base))
    {
        return -4;
    }
    return 0;
}

/**
 * @brief spic以CPU的方式向flash写数据
 *
 * @param spic_base_config 配置结构体指针
 * @param write_data 数据指针
 * @param write_len 数据长度
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_write_by_cpu(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* write_data,uint32_t write_len)
{
	uint32_t data_reg[2] = {0};
	uint32_t temp = 0;
    spic_register_p spic = (spic_register_p)spic_base;
    spic_config_t config = {0};
    spic->gm_cfg.gm_dma_wreq_md = 1;
    if(spic_config_init(&config,spic_base_config))
    {
        return -2;
    }
    if(write_len > 8)
    {
        config.gm_data_size = (write_len/4)*4;
    }
    else
	{
    	config.gm_data_size = write_len;
    	config.gm_data_store_md = 1;
    	MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(data_reg,write_data,write_len);
		temp = data_reg[0];
		data_reg[0] = ((temp&0xFF)<< 24)|((temp&0xFF00)<<8)|((temp&0xFF0000)>>8)|((temp&0xFF000000)>>24);
		temp = data_reg[1];
		data_reg[1] = ((temp&0xFF)<< 24)|((temp&0xFF00)<<8)|((temp&0xFF0000)>>8)|((temp&0xFF000000)>>24);
    	spic->rw_data0_cfg = data_reg[0];
    	spic->rw_data1_cfg = data_reg[1];
	}
    config.gm_write_en = 1;
    if(spic_general_mode_config(spic_base,&config))
    {
        return -3;
    }
    if(write_len > 8)
    {
    	MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((void *)SPIC_FIFO_ADDR,write_data,(write_len/4)*4);
    }
    while(!(spic->state_reg & STATE_REG_GM_DONE_INT));
    spic->state_reg |= STATE_REG_GM_DONE_INT;
    if(spic_wait_idle(spic_base))
    {
        return -4;
    }
    return 0;
}

/**
 * @brief spic以DMA的方式向flash读数据
 *
 * @param spic_base_config 配置结构体指针
 * @param read_data 数据指针
 * @param read_len 数据长度
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_read_by_dma(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* read_data,uint32_t read_len)
{
    spic_register_p spic = (spic_register_p)spic_base;
    spic_config_t config = {0};
    if(spic_config_init(&config,spic_base_config))
    {
        return -2;
    }
    config.gm_data_size = (read_len/4)*4;
    config.gm_read_en = 1;
    config.gm_dma_en = 1;
    //配置DMA
    #if SPIC_DMA_MODEFI
    DMAC_CHANNEL0_LOCK();
    spic_dma_transfer_config(SPIC_DMA_FIFO_ADDR, (uint32_t)spi_dma_buf, read_len / 4);
    #else
    dma_config_t dam_config = {0};
    dam_config.flowctrl = P2M_DMA;
    dam_config.busrtsize = BURSTSIZE1;
    dam_config.transferwidth = TRANSFERWIDTH_32b;
    dam_config.srcaddr = SPIC_DMA_FIFO_ADDR;
    dam_config.destaddr = (uint32_t)spi_dma_buf;
    dam_config.transfersize = read_len / 4;
    DMAC_CHANNEL0_LOCK();
    spic_dma_config(dmachanel,&dam_config);
    #endif //#if SPIC_DMA_MODEFI
    spic->state_reg |= STATE_REG_GM_DONE_INT;
    spic->gm_cfg.gm_dma_wreq_md = 0;
    if(spic_general_mode_config(spic_base,&config))
    {
        return -3;
    }
    while(!(spic->state_reg & STATE_REG_GM_DONE_INT));
    spic->state_reg |= STATE_REG_GM_DONE_INT;
    if(spic_wait_idle(spic_base))
    {
        return -4;
    }
    if(RETURN_ERR == wait_dma_translate_flag(dmachanel,SPIC_TIMEOUT_DMA))
    {
        DMAC_CHANNEL0_UNLOCK();
        return -5;
    }
    DMAC_CHANNEL0_UNLOCK();
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(read_data,spi_dma_buf,(read_len/4)*4);
    return 0;
}

/**
 * @brief spic以DMA的方式向flash写数据
 *
 * @param spic_base_config 配置结构体指针
 * @param write_data 数据指针
 * @param write_len 数据长度
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_write_by_dma(uint32_t spic_base,spic_base_config_p spic_base_config,uint8_t* write_data,uint32_t write_len)
{
    spic_register_p spic = (spic_register_p)spic_base;
    spic_config_t config = {0};
    if(spic_config_init(&config,spic_base_config))
    {
        return -2;
    }
    config.gm_data_size = (write_len/4)*4;
    config.gm_write_en = 1;
    config.gm_dma_en = 1;
    //配置DMA
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(spi_dma_buf,write_data,write_len);
    #if SPIC_DMA_MODEFI
    DMAC_CHANNEL0_LOCK();
    spic_dma_transfer_config((uint32_t)spi_dma_buf, SPIC_DMA_FIFO_ADDR, write_len / 4);
    #else
    dma_config_t dam_config = {0};
    dam_config.flowctrl = M2P_DMA;
    dam_config.busrtsize = BURSTSIZE1;
    dam_config.transferwidth = TRANSFERWIDTH_32b;
    dam_config.srcaddr = (uint32_t)spi_dma_buf;
    dam_config.destaddr = SPIC_DMA_FIFO_ADDR;
    dam_config.transfersize = write_len / 4;
    DMAC_CHANNEL0_LOCK();
    spic_dma_config(dmachanel,&dam_config);
    #endif //#if SPIC_DMA_MODEFI
    spic->gm_cfg.gm_dma_wreq_md = 0;
    if(spic_general_mode_config(spic_base,&config))
    {
        return -3;
    }
    while(!(spic->state_reg & STATE_REG_GM_DONE_INT));
    spic->state_reg |= STATE_REG_GM_DONE_INT;
    if(spic_wait_idle(spic_base))
    {
        return -4;
    }
    if(RETURN_ERR == wait_dma_translate_flag(dmachanel,SPIC_TIMEOUT_DMA))
    {
        DMAC_CHANNEL0_UNLOCK();
        return -5;
    }
    DMAC_CHANNEL0_UNLOCK();
    return 0;
}

int32_t spic_xip_config(uint32_t spic_base,spic_base_config_p spic_base_config)
{
	spic_register_p spic = (spic_register_p)spic_base;
	spic_config_t config = {0};
	if(spic_config_init(&config,spic_base_config))
	{
		return -2;
	}
	if(spic_xip_read_mode_config(spic_base,&config))
	{
		return -3;
	}
	return 0;
}

/**
 * @brief spic控制器初始化
 *
 * @param init 初始化结构体
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_init(uint32_t spic_base,spic_init_p init)
{
    spic_register_p spic = (spic_register_p)spic_base;
    //配置globe_cfg寄存器
    spic->globe_cfg = ((init->flash_clk_div << 0) | (RAM_CLK_MD_0 << 2));
    //配置globe_cfg1寄存器
    spic->globe1_cfg = ((0x1 << 1) | (SPIC_CS_HIGH_CYCLE << 11) | (SPIC_CS_LOW_CYCLE << 14));
    //配置rx_clk_cfg寄存器
    spic->rx_clk_cfg = ((DELAY_LINE << 4) | (RX_NAGE_SAMPLE));  //改成窗口中间的时序：26
    //配置addr_mask_cfg寄存器
    spic->addr_mask_cfg = 0xfc0;     //8M配0xf80，4M配0xfc0，该配置会影响xip模式的使用
    //配置int_ctrl_cfg寄存器
    spic->int_ctrl_cfg = 0;
    //配置fifo_level_cfg寄存器
    spic->fifo_level_cfg = ((SPIC_WFIFO_AMFULL_LEVEL << 0) | (SPIC_RFIFO_AMEMPTY_LEVEL << 7) | (SPIC_RFIFO_AMFULL_LEVEL << 13));
    //保存配置信息
    global_flash_sel = init->flash_sel;
    global_addr_size = init->addr_size;
    return 0;
}

/**
 * @brief flash时钟相位调制
 *
 * @param tx_shift 偏移X个core_clk，注意X必须小于core_clk与flash_clk的频率倍数（dtr模式下小于倍数的一半）
 * @param tx_nege_en 配置为1时在shift的基础上再向后平移半个core_clk周期
 * @param rx_shift 偏移X个core_clk，注意X必须小于core_clk与flash_clk的频率倍数（dtr模式下小于倍数的一半）
 * @param rx_nege_en 配置为1时在shift的基础上再向后平移半个core_clk周期
 * @return int32_t 0:成功;1:失败;
 */
int32_t spic_clk_phase_set(uint32_t spic_base,uint32_t tx_shift,uint32_t tx_nege_en,uint32_t rx_shift,uint32_t rx_nege_en)
{
    spic_register_p spic = (spic_register_p)spic_base;
    uint32_t max_shift = 0;
    md_sel_t mode = spic->gm_cfg.gm_cmd_md_sel;
    flash_clk_div_t clk_div = (spic->globe_cfg & 0x3);
    switch(clk_div)
    {
        case FLASH_CLK_DIV_2:
        max_shift = 2;
        break;
        case FLASH_CLK_DIV_4:
        max_shift = 4;
        break;
        case FLASH_CLK_DIV_6:
        max_shift = 6;
        break;
        case FLASH_CLK_DIV_8:
        max_shift = 8;
        break;
    }
    if(mode > MD_SEL_RESERVED_2)
    {
        max_shift /= 2;
    }
    if((tx_shift > max_shift) || (rx_shift > max_shift))
    {
        return -1;
    }
    if(mode > MD_SEL_RESERVED_2)
    {
        spic->globe_cfg &= ~(0xF << 8);
        spic->globe_cfg |= (tx_shift << 8) | (tx_nege_en << 11);
    }
    else
    {
        spic->globe_cfg &= ~(0xF << 4);
        spic->globe_cfg |= (tx_shift << 4) | (tx_nege_en << 7);
    }
    spic->rx_clk_cfg &= ~0xF;
    spic->rx_clk_cfg |= (rx_shift << 0) | (rx_nege_en << 3);
    return 0;
}

#if 0
/**
 * @brief flash硬件复位，需要接FLASH器件的reset脚
 *
 * @param enable 0：不复位；1：复位；
 */
void spic_hardware_reset(uint32_t spic_base,uint8_t enable)
{
    spic_register_p spic = (spic_register_p)spic_base;
    if(enable)
    {
        spic->globe1_cfg.flash_hw_reset = 1;
    }
    else
    {
        spic->globe1_cfg.flash_hw_reset = 0;
    }
}
#endif

/**
 * @brief flash预取开关
 *
 * @param en enable：开启；disable：关闭；
 */
void spic_prefetch_en(uint32_t spic_base,bool en)
{
    spic_register_p spic = (spic_register_p)spic_base;
    spic->rd_cfg.prefetch_en = en;
}