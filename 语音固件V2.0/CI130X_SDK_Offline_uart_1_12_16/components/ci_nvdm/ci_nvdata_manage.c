/**
 * @file ci_nvdata_manage.c
 * @brief 
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ci_log.h"

#include "ci_nvdata_manage.h"
#include "ci_nvdata_port.h"

#include "ci130x_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sdk_default_config.h"
#include "romlib_runtime.h"

#if USE_VP
#include "voiceprint_port.h"
#endif

#define MEMORY_COPY_USED (1)

/* !!! feb: flash erase block, this block size maybe not same as spiflash 'block erased cmd' size, 
            just represent can erased size. default used spiflash erase sector cmd 4KByte, see 
            CINV_FLASH_ERASE_BLOCK_SIZE in ci_nvdata_managed.h !!! */

#if(CINV_FLASH_ERASE_BLOCK_SIZE > 65536)
#error "CINV_FLASH_ERASE_BLOCK_SIZE config error,this code used uint16_t as feb offset"
#endif
    
/*define buffer size of internal used buffer, bigger can reduce flash access*/
#define CINV_IO_BUFFER_SIZE    (256)

#if(CINV_IO_BUFFER_SIZE > CINV_FLASH_ERASE_BLOCK_SIZE)
#error "CINV_IO_BUFFER_SIZE config error"
#endif

/*here used this for traverse_feb block reduce flash read*/
#define CINV_DATA_ITEM_MAX_LEN (CINV_IO_BUFFER_SIZE - sizeof(cinv_data_item_header_t))

static uint8_t cinv_rd_io_buf[CINV_IO_BUFFER_SIZE];
/*because our chip code and data all in ram, so used wr buf for code flow simply*/
static uint8_t cinv_wr_io_buf[CINV_IO_BUFFER_SIZE];

typedef struct cinv_hotid_node_st
{
    uint32_t hot_id;
    uint16_t id_blk;    /*feb number of current hotid*/
    uint16_t id_off;    /*offset of current hotid*/
    struct cinv_hotid_node_st * next;
}cinv_hotid_node_t;


typedef struct
{
    cinv_hotid_node_t * head;
}cinv_hotid_mang_t;

static cinv_hotid_mang_t cinv_hotid_m = {NULL};

typedef struct
{
    uint16_t *p_feb_off; /*<! offset write into the erase block of the first available erased space*/
    uint16_t *p_feb_lost;/*<! count of the bytes lost for erase block, such as deleted item*/
    uint16_t feb_rev; /*<! block offset reserved for reclaim, now only used one*/
}cinv_mang_rec_t;

static cinv_mang_rec_t cinv_mang_r;

/*used 4byte for our flash controller align*/
typedef enum 
{
    CINV_FEB_STATUS_ERASED = 0xFFFFFFFF,     /*<! also as revserved*/
    CINV_FEB_STATUS_EMPTY = 0xFFFFFEFE,      /*<! the block has valid feb header, no valid data */
    CINV_FEB_STATUS_ACTIVED = 0xFFFFF8F8,    /*<! the block has valid feb header, actived maybe have valid data */
    CINV_FEB_STATUS_TRANSFERING = 0xFFFFF0F0,/*<! the block has valid feb header, old feb starting relaim*/
    #if 0/* todo now no used in ver 0.1*/
    //CINV_FEB_STATUS_TRANSFERED = 0xFFFFE0E0,
    //CINV_FEB_STATUS_RECLAIMING = 0xFFFFC0C0,
    #endif
}cinv_feb_status_t;

//#define CINV_FEB_HEADER_MAGIC_VALID (0xABCDEF01) move to ci_nvdata_manage.h
#define CINV_FEB_HEADER_MAGIC_ERASED (0xFFFFFFFF)
#define CINV_FEB_HEADER_OFFSET (0)
#define CINV_FEB_HEADER_SIZE (sizeof(cinv_feb_head_t))

typedef struct
{
    uint32_t magic;
    cinv_feb_status_t status;
    uint32_t is_rev; /* todo now no used in ver 0.1*/
}cinv_feb_head_t;

typedef enum 
{
    CINV_DATA_ITEM_STATUS_EMPTY = 0xFFFFFFFF,   /*<! no data item exist after it */
    CINV_DATA_ITEM_STATUS_WRITING = 0xFEFEFEFE, /*<! data item is being written to new place*/
    CINV_DATA_ITEM_STATUS_VALID = 0xFCFCFCFC,   /*<! data item has been written to this place successfully*/
    CINV_DATA_ITEM_STATUS_TRANS = 0xF8F8F8F8,   /*<! mark this data item old, and wait new write, and delete this*/
    CINV_DATA_ITEM_STATUS_DELETE = 0xF0F0F0F0,  /*<! data item has been discarded because of new copy is ready*/
    CINV_DATA_ITEM_STATUS_ERROR = 0x00000000,   /*<!   */
}cinv_data_item_status_t;

typedef struct 
{
    cinv_data_item_status_t status;  /*<! status of data item */
    uint32_t id;
    uint32_t len;
    uint32_t chk_sum;
    uint8_t buf[0];
}cinv_data_item_header_t;

#define CINV_DATA_ITEM_START_OFF (CINV_FEB_HEADER_OFFSET+CINV_FEB_HEADER_SIZE)
#define CINV_DATA_ITEM_STATUS_OFF (0)
/*calculate flash address from feb number and offset in feb*/
#define CINV_FEB_NUM_TO_ADDR(blk_num,offset) (g_flash_base_addr + CINV_FLASH_ERASE_BLOCK_SIZE*(blk_num) + (offset))

#define CINV_FEB_NO_USED       (0xffffu)

#define CINV_ITEM_NO_USED       (0xffffu)

/*now align to 4byte*/
#define CINV_ITEM_LEN_ACTUAL_USED( len )  (((len)+3)/4*4)

/*just for test*/
#define POWEROFF_TEST(pos) do{}while(0)  /*no test and save code size*/
//#define POWEROFF_TEST(pos) poweroff_random_test(pos)

#define PFT_POS_RECLAIM_BEFORE_ERASE1       0
#define PFT_POS_BEFORE_WRITING_NEW          1
#define PFT_POS_BEFORE_WRITING_START        2
#define PFT_POS_BEFORE_WRITING_DONE         3
#define PFT_POS_AFTER_MARK_TARNS            4
#define PFT_POS_RECLAIM_BEFORE_ERASE2       5
#define PFT_POS_DELETE_ITEM                 6
#define PFT_POS_SET_TRANSFERING             7

static uint32_t g_flash_base_addr; /*start addr of nvdata in flash */
static uint16_t g_feb_total_count; /*flash total used size is CINV_FLASH_ERASE_BLOCK_SIZE*g_feb_total_count*/


#if USE_VP
static uint32_t vp_flash_addr = 0;    //VP模板在flash地址
#endif

#if USE_VP
uint32_t get_vp_flash_addr(void)
{
    return vp_flash_addr;
}
#endif


static void poweroff_random_test(uint32_t pos)
{
    if(PFT_POS_SET_TRANSFERING == pos)
    {
        while(1);/*reset or while 1 for manual reset*/
    }
}


/**
 * @brief check current buffer data is erased
 * 
 * @param buf : data buffer pointer
 * @param len : data buffer length
 * @return true : erased
 * @return false : wrong, have no erased data
 */
static bool check_erased(const void *buf, int32_t len)
{
    int32_t i;

    for (i = 0; i < len; i++) 
    {
        if (((const uint8_t *)buf)[i] != 0xff) 
        {
            return false;
        }
    }

    return true;
}


/**
 * @brief erased flash block, no check, 
 * 
 * @param blk_num : feb number
 */
static void erase_feb(uint16_t blk_num)
{
    uint32_t addr = CINV_FEB_NUM_TO_ADDR(blk_num,0);
    
    cinv_port_flash_erase(addr);

    cinv_mang_r.p_feb_off[blk_num] = CINV_DATA_ITEM_START_OFF;
    cinv_mang_r.p_feb_lost[blk_num] = 0;
}


/**
 * @brief first read,if already is erased, do nothing, else erase it
 * 
 * @param blk_num : feb number
 */
/* !!! caution !!! used cinv_rd_io_buf*/
static void check_and_erase_feb(uint16_t blk_num)
{
    uint32_t addr;
    uint32_t read_len = 0;
    uint32_t remain_size = CINV_FLASH_ERASE_BLOCK_SIZE;
    addr = CINV_FEB_NUM_TO_ADDR(blk_num, 0);
    do{
        if(remain_size > CINV_IO_BUFFER_SIZE)
        {
            read_len = CINV_IO_BUFFER_SIZE;
        }
        else
        {
            read_len = remain_size;
        }
        cinv_port_flash_read(addr, cinv_rd_io_buf, read_len);
        if(!check_erased(cinv_rd_io_buf, read_len))
        {
            erase_feb(blk_num);
            break;
        }
        addr += read_len;
        remain_size -= read_len;
    }while(remain_size > 0);
}


static cinv_hotid_node_t * find_hotid_node(uint32_t id)
{
    cinv_hotid_node_t * hotid_node = cinv_hotid_m.head;
    while(hotid_node)
    {
        if (hotid_node->hot_id == id)
        {
            return hotid_node;
        }
        hotid_node = hotid_node->next;
    }
    return NULL;
}


/*here for fast access, maybe config hotid size same as item count*/
static void hotid_init(void)
{
    cinv_hotid_m.head = NULL;
}


/**
 * @brief register hotid.
 * 
 * @param id    Nvdata item id
 *
 * @retval CINV_OPER_SUCCESS   Id already existed, no action taken.
 * @retval CINV_OPER_FAILED    Failure to find or create id.
 */
cinv_item_ret_t cinv_register_hotid(uint32_t hot_id)
{
    cinv_hotid_node_t * new_node = (cinv_hotid_node_t *)pvPortMalloc(sizeof(cinv_hotid_node_t));
    if (new_node)
    {
        new_node->hot_id = hot_id;
        new_node->id_blk = CINV_FEB_NO_USED;
        new_node->id_off = CINV_ITEM_NO_USED;
        new_node->next = cinv_hotid_m.head;
        cinv_hotid_m.head = new_node;
        return CINV_OPER_SUCCESS;
    }
    return CINV_OPER_FAILED;
}

/**
 * @brief if nvdata item id in hot id array, update item address
 * 
 * @param blk_num : feb number of item address
 * @param off : offset in feb of item address 
 * @param id : nvdata item id
 */
static void hotid_update(uint16_t blk_num, uint16_t off, uint32_t id)
{
    cinv_hotid_node_t * hotid_node = find_hotid_node(id);

    if(hotid_node)
    {
        hotid_node->id_blk = blk_num;
        hotid_node->id_off = off;
    }
}


/**
 * @brief find hot id address
 * 
 * @param id ：nvdata item id
 * @param blk_num ：return value feb number
 * @return uint32_t CINV_ITEM_NO_USED : no find, other: offset of feb
 */
static uint32_t find_hotid(uint32_t id, uint16_t *blk_num)
{
    cinv_hotid_node_t * hotid_node = find_hotid_node(id);
    uint16_t offset = CINV_ITEM_NO_USED;

    *blk_num = CINV_ITEM_NO_USED;

    if(hotid_node)
    {
        offset = hotid_node->id_off;/*must be init CINV_ITEM_NO_USED in cinv_init()*/
        *blk_num = hotid_node->id_blk; 
    }

    return offset;
}


/**
 * @brief calculate the data checksum
 * 
 * @param buf : data buffer pointer
 * @param len : buffer lens
 * @return uint32_t : checksum value
 */
static uint32_t calc_chk_sum(uint8_t *buf, uint16_t len)
{
    uint32_t chk_sum = 0;

    for(int i=0; i<len; i++)
    {
        chk_sum += buf[i];
    }

    return chk_sum;
}


/**
 * @brief write nvdata item header status to flash, and verify
 * 
 * @param blk_num : feb number of item address
 * @param offset : offset in feb of item address
 * @param status : data item status
 * @return true : write ok
 * @return false : write error
 */
/*flash CS pin to high, will real program, so status write independently, status can program fast*/
static bool write_item_status(uint16_t blk_num, uint16_t offset, cinv_data_item_status_t status)
{
    uint32_t addr = CINV_FEB_NUM_TO_ADDR(blk_num,offset);
    cinv_data_item_header_t *di_hdr = (cinv_data_item_header_t *)cinv_wr_io_buf;
    uint32_t write_size = sizeof(di_hdr->status);

    addr += CINV_DATA_ITEM_STATUS_OFF;
    di_hdr->status = status;
    cinv_port_flash_write(addr, cinv_wr_io_buf, write_size);
    
    /*verify*/
    cinv_port_flash_read(addr, cinv_rd_io_buf, sizeof(cinv_data_item_header_t));/*read large for detele used*/
    if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.memcmp_p(cinv_wr_io_buf, cinv_rd_io_buf, write_size))
    {
        if(CINV_DATA_ITEM_STATUS_DELETE == status)
        {
            di_hdr = (cinv_data_item_header_t *)cinv_rd_io_buf;
            cinv_mang_r.p_feb_lost[blk_num] += CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len) + sizeof(cinv_data_item_header_t);
        }
        return true;
    }
    cinv_assert(0);
    return false;
}


/**
 * @brief write feb header status to flash, and verify
 * 
 * @param blk_num : feb number
 * @param status : feb status
 * @return true : write ok
 * @return false : write error
 */
static bool write_feb_status(uint16_t blk_num, cinv_feb_status_t status)
{
    uint32_t addr = CINV_FEB_NUM_TO_ADDR(blk_num,CINV_FEB_HEADER_OFFSET);
    cinv_feb_head_t *feb_hdr_wr = (cinv_feb_head_t *)cinv_wr_io_buf;
    uint32_t write_size = sizeof(feb_hdr_wr->magic)+sizeof(feb_hdr_wr->status);
    
    feb_hdr_wr->magic = CINV_FEB_HEADER_MAGIC_VALID;
    feb_hdr_wr->status = status;
    cinv_port_flash_write(addr, cinv_wr_io_buf, write_size);

    /*verify*/
    cinv_port_flash_read(addr, cinv_rd_io_buf, write_size);
    if(0 == MASK_ROM_LIB_FUNC->newlibcfunc.memcmp_p(cinv_wr_io_buf,cinv_rd_io_buf,write_size))
    {
        return true;
    }
    cinv_assert(0);
    return false;
}


/**
 * @brief   writes an item to nvdata region
 * 
 * @param   blk_num : feb number
 * @param   id : nvdata item id
 * @param   len : byte count of data to write
 * @param   buf : data buffer
 * @return  true : write ok
 * @return  false : have some error
 */
static bool write_item(uint16_t blk_num, uint32_t id, uint16_t len, void *buf)
{
    uint16_t offset = cinv_mang_r.p_feb_off[blk_num];
    uint32_t item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(len);
    cinv_data_item_header_t *di_hdr;
    uint32_t addr;

// TODO:  need retry ??? in new feb
/*now this code no support retry,just lost block last space*/
/*write_retry:*/

    /*starting write, flash cs pin to high, will real program, so status write independently*/
    write_item_status(blk_num, offset, CINV_DATA_ITEM_STATUS_WRITING);

    POWEROFF_TEST(PFT_POS_BEFORE_WRITING_START);

    /* buffer reused, must be careful*/
    MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(cinv_wr_io_buf, 0x0, item_len);
    di_hdr = (cinv_data_item_header_t *)cinv_wr_io_buf;
    di_hdr->status = CINV_DATA_ITEM_STATUS_WRITING;
    di_hdr->id = id;
    di_hdr->len = len;
    di_hdr->chk_sum = calc_chk_sum((uint8_t *)buf, len);
    MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p(di_hdr->buf, buf, len);

    /*wirte header and data*/
    addr = CINV_FEB_NUM_TO_ADDR(blk_num,offset);
    cinv_port_flash_write(addr, cinv_wr_io_buf, item_len);

    /*read for verify*/
    cinv_port_flash_read(addr, cinv_rd_io_buf, item_len);
    
    if(0 != MASK_ROM_LIB_FUNC->newlibcfunc.memcmp_p(cinv_wr_io_buf,cinv_rd_io_buf,item_len))
    {
        /*error, mark this block last don't used*/
        cinv_mang_r.p_feb_off[blk_num] = CINV_FLASH_ERASE_BLOCK_SIZE;
        cinv_mang_r.p_feb_lost[blk_num] += CINV_FLASH_ERASE_BLOCK_SIZE - cinv_mang_r.p_feb_off[blk_num];
        write_item_status(blk_num, offset, CINV_DATA_ITEM_STATUS_ERROR);
        cinv_assert(0);
        return false;
    }
    
    /*write done*/
    write_item_status(blk_num, offset, CINV_DATA_ITEM_STATUS_VALID);

    POWEROFF_TEST(PFT_POS_BEFORE_WRITING_DONE);

    /*updata some info*/    
    hotid_update(blk_num, offset, di_hdr->id);
    cinv_mang_r.p_feb_off[blk_num] += item_len;

    return true;
}


#if 0/*test code, else flow will save code size*/
/**
 * @brief   reclaim a feb, valid item of this feb copy to reserved feb, then erased it and set this to reserved 
 * 
 * @param blk_num : feb number
 * @param old_id  : item id don't copy, this id will write a new , so old don't copy
 * @return true  : relaim ok
 * @return false 
 */
static bool reclaim_block(uint32_t blk_num, uint32_t old_id)
{
    uint32_t offset;
    uint32_t addr;
    cinv_data_item_header_t *di_hdr;
    uint32_t item_len;

    uint32_t write_off = CINV_DATA_ITEM_START_OFF;
    uint32_t write_addr;

    offset = CINV_DATA_ITEM_START_OFF;
    do
    {
        addr = CINV_FEB_NUM_TO_ADDR(blk_num,offset);
        cinv_port_flash_read(addr,cinv_rd_io_buf,sizeof(cinv_rd_io_buf));
        di_hdr = (cinv_data_item_header_t *)cinv_rd_io_buf;

        item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len);
        
        if((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status) ||
            (CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)||
            (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status))
        {
            if(((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status)
                || (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status)) &&    
                (di_hdr->id != old_id) &&
                (di_hdr->chk_sum == calc_chk_sum(di_hdr->buf, di_hdr->len)))
            {
                di_hdr->status = CINV_DATA_ITEM_STATUS_VALID;/*because can write, so force trans to valid*/
                write_addr =  CINV_FEB_NUM_TO_ADDR(cinv_mang_r.feb_rev, write_off);
                cinv_port_flash_write(write_addr, (uint8_t *)di_hdr, item_len); // TODO: no verify
                hotid_update(cinv_mang_r.feb_rev, write_off, di_hdr->id);
                write_off += item_len;
                cinv_mang_r.p_feb_off[cinv_mang_r.feb_rev] += item_len;
            }
        }
        else if((di_hdr->status == CINV_DATA_ITEM_STATUS_EMPTY) ||
            (di_hdr->status == CINV_DATA_ITEM_STATUS_WRITING))
        {
            break;/*also end*/
        }
        else
        {
            POWEROFF_TEST(offset);
            POWEROFF_TEST(di_hdr->status);
            while(1);
            break;/*mabey error or last data*/ // TODO:  deal error?
        }
        offset += item_len;
    } while (offset < (CINV_FLASH_ERASE_BLOCK_SIZE - sizeof(cinv_data_item_header_t)));

    if(CINV_ITEM_NO_USED != old_id)
    {
        /*deal erase feb after new item write done in function init_item()*/
    }
    else
    {
        /*mark new ok*/
        write_feb_status(cinv_mang_r.feb_rev,CINV_FEB_STATUS_ACTIVED);
        POWEROFF_TEST(PFT_POS_RECLAIM_BEFORE_ERASE1);
        erase_feb(blk_num);
        cinv_mang_r.feb_rev = blk_num; 
    }

    return true;
}


static int32_t init_page_id(uint32_t blk_num, uint32_t id, bool transid_valid)
{
    uint32_t offset;
    uint32_t addr;
    cinv_data_item_header_t *di_hdr;
    uint32_t lost;
    uint32_t item_len;

    offset = CINV_DATA_ITEM_START_OFF;
    lost = 0;

    do
    {
        // TODO: now read size too large,used sizeof(cinv_rd_io_buf)
        addr = CINV_FEB_NUM_TO_ADDR(blk_num,offset);
        cinv_port_flash_read(addr,cinv_rd_io_buf,sizeof(cinv_rd_io_buf));
        di_hdr = (cinv_data_item_header_t *)cinv_rd_io_buf;

        if(CINV_DATA_ITEM_STATUS_EMPTY == di_hdr->status)
        {
            break;
        }

        /*get the actual size in bytes which is the ceiling(di_hdr.len)*/
        item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED( di_hdr->len );

        /*a bad 'len' write has blown away the rest of the page*/
        if (item_len > (CINV_FLASH_ERASE_BLOCK_SIZE - offset))
        {
            lost += (CINV_FLASH_ERASE_BLOCK_SIZE - offset);
            offset = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }

        if((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status) ||
            (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status))
        {
            if(id == di_hdr->id)
            {
                if((transid_valid) ||
                    (CINV_DATA_ITEM_STATUS_VALID == di_hdr->status))
                {
                    return offset;
                }
            }
        }
        else if(CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)
        {
            lost += item_len;
        }
        else if(CINV_DATA_ITEM_STATUS_WRITING == di_hdr->status)
        {
            /*don't modify status, just lost this feb last space*/
            /*so reclaim_block this status deal as empty*/
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - offset;
            offset = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }
        else
        {
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - offset;
            offset =  CINV_FLASH_ERASE_BLOCK_SIZE;
            while(1);
            break;/*error*/
        }
        
        offset += item_len;

    } while (offset < (CINV_FLASH_ERASE_BLOCK_SIZE - sizeof(cinv_data_item_header_t)));

    // TODO: this function no need update cinv_mang_r

    cinv_mang_r.p_feb_off[blk_num] = offset;
    cinv_mang_r.p_feb_lost[blk_num] = lost;

    return CINV_ITEM_NO_USED;
}



static uint16_t trans_blk = CINV_FEB_NO_USED;
static uint16_t trans_off = CINV_ITEM_NO_USED;
static uint32_t trans_id;

static int32_t init_page(uint32_t blk_num)
{
    uint32_t offset;
    uint32_t addr;
    cinv_data_item_header_t *di_hdr;
    uint32_t lost;
    uint32_t item_len;

    // TODO:check feb header

    offset = CINV_DATA_ITEM_START_OFF;
    lost = 0;
    do
    {
        // TODO: now read size too large,used sizeof(cinv_rd_io_buf)
        addr = CINV_FEB_NUM_TO_ADDR(blk_num,offset);
        cinv_port_flash_read(addr,cinv_rd_io_buf,sizeof(cinv_rd_io_buf));
        di_hdr = (cinv_data_item_header_t *)cinv_rd_io_buf;

        if(CINV_DATA_ITEM_STATUS_EMPTY == di_hdr->status)
        {
            break;
        }

        /*get the actual size in bytes which is the ceiling(di_hdr.len)*/
        item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED( di_hdr->len );

        /*a bad 'len' write has blown away the rest of the page*/
        if (item_len > (CINV_FLASH_ERASE_BLOCK_SIZE - offset))
        {
            lost += (CINV_FLASH_ERASE_BLOCK_SIZE - offset);
            offset = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }

        if ((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status) || 
            (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status))
        {
            if (di_hdr->chk_sum == calc_chk_sum(di_hdr->buf, di_hdr->len))
            {
                if(CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status)
                {
                    trans_blk = blk_num;
                    trans_off = offset;
                    trans_id = di_hdr->id;
                }
                // TODO: don't call in here?
                /*because this code must be call cinv_item_init before used read/write, 
                so this hotid_update also can call in cinv_item_init*/
                /*call here in cinv_init,must be used last,confirm all feb ok*/
                hotid_update(blk_num,offset,di_hdr->id);
            }
            else
            {
                /*just delete bad checksum item*/
                write_item_status(blk_num, offset, CINV_DATA_ITEM_STATUS_DELETE);  // TODO: don't know how do it                
                lost += item_len;
            }
        }
        else if(CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)
        {
            lost += item_len;
        }
        else if(CINV_DATA_ITEM_STATUS_WRITING == di_hdr->status)
        {
            /*don't modify status, just lost this feb last space*/
            /*so reclaim_block this status deal as empty*/
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - offset;
            offset = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }
        else
        {
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - offset;
            offset = CINV_FLASH_ERASE_BLOCK_SIZE;
            while(1);
            break;/*error*/
        }
        
        offset += item_len;
    } while (offset < (CINV_FLASH_ERASE_BLOCK_SIZE - sizeof(cinv_data_item_header_t)));

    cinv_mang_r.p_feb_off[blk_num] = offset;
    cinv_mang_r.p_feb_lost[blk_num] = lost;

    return CINV_ITEM_NO_USED;
}
#else

typedef enum
{
    ITEM_DEAL_SUCESS,/**/
    ITEM_DEAL_ERROR,/**/
    ITEM_DEAL_FIND_ID_DONE,/**/
}item_deal_status_t;


item_deal_status_t reclaim_block_deal_flow(cinv_data_item_header_t *di_hdr, uint16_t *write_off, uint32_t old_id)
{
    uint32_t write_addr;
    uint32_t item_len;

    item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len);

    if(CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)
    {
        return ITEM_DEAL_SUCESS;
    }
    else if((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status) ||
        (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status))
    {
        if((di_hdr->id != old_id) &&
            (di_hdr->chk_sum == calc_chk_sum(di_hdr->buf, di_hdr->len)))/*why no deal p_feb_lost? in erase_feb*/
        {
            di_hdr->status = CINV_DATA_ITEM_STATUS_VALID;/*because can write, so force trans to valid*/
            write_addr = CINV_FEB_NUM_TO_ADDR(cinv_mang_r.feb_rev, *write_off);
            cinv_port_flash_write(write_addr, (uint8_t *)di_hdr, item_len);   // TODO: no verify, if add verify, add other feb_rev is more better
            hotid_update(cinv_mang_r.feb_rev, *write_off, di_hdr->id);
            *write_off += item_len;
            cinv_mang_r.p_feb_off[cinv_mang_r.feb_rev] = *write_off;
        }

        return ITEM_DEAL_SUCESS;
    }
    else  /*CINV_DATA_ITEM_STATUS_WRITING as error*/
    {
        return ITEM_DEAL_ERROR;/*mabey error or last data*/
    }
}


static uint16_t trans_blk = CINV_FEB_NO_USED;
static uint16_t trans_off = CINV_ITEM_NO_USED;
static uint32_t trans_id;

item_deal_status_t init_page_deal_flow(cinv_data_item_header_t *di_hdr, uint32_t blk_num, uint16_t off, uint16_t *lost)
{
    uint32_t item_len;

    item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len);

    if ((CINV_DATA_ITEM_STATUS_VALID == di_hdr->status) || 
            (CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status))
    {
        if (di_hdr->chk_sum == calc_chk_sum(di_hdr->buf, di_hdr->len))
        {
            if(CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status)
            {
                trans_blk = blk_num;
                trans_off = off;
                trans_id = di_hdr->id;
            }

            /*call here in cinv_init,must be used last,confirm all feb ok*/
            hotid_update(blk_num, off, di_hdr->id);
        }
        else
        {
            /*just delete bad checksum item*/
            write_item_status(blk_num, off, CINV_DATA_ITEM_STATUS_DELETE);
            *lost += item_len;
        }
        return ITEM_DEAL_SUCESS;
    }
    else if(CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)
    {
        *lost += item_len;  /* just deal*/
        return ITEM_DEAL_SUCESS;
    }
    else if(CINV_DATA_ITEM_STATUS_WRITING == di_hdr->status)
    {
        /*don't modify status, just lost this feb last space*/
        /*so reclaim_block this status deal as error*/
        return ITEM_DEAL_ERROR;
    }
    else
    {
        return ITEM_DEAL_ERROR; /*error*/
    }
}


item_deal_status_t init_page_id_deal_flow(cinv_data_item_header_t *di_hdr, uint32_t id, bool transid_valid, uint16_t *lost)
{
    uint32_t item_len;

    item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len);

    if(CINV_DATA_ITEM_STATUS_VALID == di_hdr->status)
    {
        if(id == di_hdr->id)
        {
            return ITEM_DEAL_FIND_ID_DONE;//read_off + buf_offset;  /*find*/
        }
    }
    else if(CINV_DATA_ITEM_STATUS_TRANS == di_hdr->status)
    {
        if((id == di_hdr->id)&&(transid_valid))
        {
            return ITEM_DEAL_FIND_ID_DONE;//read_off + buf_offset;  /*find*/
        }
    }
    else if(CINV_DATA_ITEM_STATUS_DELETE == di_hdr->status)
    {
        *lost += item_len;  /* just deal*/
    }
    else /*include (CINV_DATA_ITEM_STATUS_WRITING == di_hdr->status)*/
    {
        return ITEM_DEAL_ERROR; /*error*/
    }
    
    return ITEM_DEAL_SUCESS;
}


typedef enum
{
    ITEM_BUF_DEAL_SUCESS,/*if feb not traverse done, need continue deal*/
    ITEM_BUF_DEAL_ERROR,/*item have some error,lost and offset need modify*/
    ITEM_BUF_DEAL_END,/*feb traverse done, like empty*/
}item_buf_deal_status_t;

static int32_t traverse_feb(uint32_t func, uint32_t blk_num, uint32_t id, bool transid_valid)
{
    uint32_t read_off;/*flash read used*/
    uint32_t read_len;
    uint32_t read_addr;
    
    uint16_t remain_size;/*buffer data dealed*/
    uint16_t dealed_size;
    item_buf_deal_status_t buf_deal_sta;

    uint16_t write_off = CINV_DATA_ITEM_START_OFF;
    
    uint16_t lost;

    read_off = CINV_DATA_ITEM_START_OFF;
    read_len = CINV_IO_BUFFER_SIZE;
    lost = 0;
    do{
        read_addr = CINV_FEB_NUM_TO_ADDR(blk_num, read_off);
        MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(cinv_rd_io_buf, 0xff, CINV_IO_BUFFER_SIZE);
        cinv_port_flash_read(read_addr, cinv_rd_io_buf, read_len);
        cinv_assert((0 == (read_addr%4))&&(0 == (read_len%4))&&(0 != read_len));

        {/*also can write a xxx() function*/
            /*deal buffer data, for reduce flash read*/
            cinv_data_item_header_t *di_hdr;
            uint32_t item_len;
            item_deal_status_t ret;

            remain_size = read_len;
            dealed_size = 0;
            
            while(1)
            {
                di_hdr = (cinv_data_item_header_t *)(cinv_rd_io_buf + dealed_size);
                item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(di_hdr->len);
                
                if(CINV_DATA_ITEM_STATUS_ERROR == di_hdr->status)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_ERROR;
                    break;/*end : data status error*/
                }

                if (item_len > CINV_IO_BUFFER_SIZE)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_ERROR;
                    break;
                }

                /*---generic deal---*/
                if(remain_size < item_len)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_SUCESS;
                    break;/*no enough data, need continue deal*/
                }

                if(CINV_DATA_ITEM_STATUS_EMPTY == di_hdr->status)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_END;
                    break;/*end : but no error*/
                }

                /*a bad 'len' write has blown away the rest of the page*/
                if (item_len > (CINV_FLASH_ERASE_BLOCK_SIZE - (read_off + dealed_size)))
                {
                    buf_deal_sta = ITEM_BUF_DEAL_ERROR;
                    break; /*end : have error*/
                }

                /*---different deal---*/
                if(1 == func)
                {
                    ret = init_page_id_deal_flow(di_hdr, id, transid_valid, &lost);
                }
                else if(2 == func)
                {
                    ret = init_page_deal_flow(di_hdr, blk_num, read_off+dealed_size, &lost);
                }
                else if(3 == func)
                {
                    ret = reclaim_block_deal_flow(di_hdr, &write_off, id);
                }
                
                if(ITEM_DEAL_SUCESS == ret)
                {
                    remain_size -= item_len;
                    dealed_size += item_len;
                    if(remain_size > sizeof(cinv_data_item_header_t))
                    {
                        continue;/* have enough data, continue deal*/
                    }
                    else
                    {
                        buf_deal_sta = ITEM_BUF_DEAL_SUCESS;
                        break;/*no enough data, need continue deal*/
                    }
                }
                else if(ITEM_DEAL_ERROR == ret)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_ERROR;
                    break;
                }
                else if(ITEM_DEAL_FIND_ID_DONE == ret)
                {
                    buf_deal_sta = ITEM_BUF_DEAL_END;
                    return read_off + dealed_size;
                }
            }

        }

        read_off += dealed_size;/*real read_off*/
        if(ITEM_BUF_DEAL_SUCESS == buf_deal_sta)/*if feb not traverse done, need continue deal*/
        {
            read_len = CINV_IO_BUFFER_SIZE;/*read to cinv_rd_io_buf full*/ 
            if((read_off+read_len) > CINV_FLASH_ERASE_BLOCK_SIZE)
            {
                read_len = CINV_FLASH_ERASE_BLOCK_SIZE - read_off;            
            }
            cinv_assert(read_off <= CINV_FLASH_ERASE_BLOCK_SIZE);
            continue;
        }
        else if(ITEM_BUF_DEAL_ERROR == buf_deal_sta)/*item have some error,lost and offset need modify*/
        {
            /* -- PFT_POS_BEFORE_WRITING_START -- */
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - read_off;
            read_off = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }
        else if(ITEM_BUF_DEAL_END == buf_deal_sta)/*feb traverse done, like empty*/
        {
            break;
        }
        else
        {
            cinv_assert(0);
            lost += CINV_FLASH_ERASE_BLOCK_SIZE - read_off;
            read_off = CINV_FLASH_ERASE_BLOCK_SIZE;
            break;
        }
        
    }while(read_off < (CINV_FLASH_ERASE_BLOCK_SIZE - sizeof(cinv_data_item_header_t)));

    if(2 == func)/*init_page*/
    {
        cinv_mang_r.p_feb_off[blk_num] = read_off;
        cinv_mang_r.p_feb_lost[blk_num] = lost;
    }
    
    return CINV_ITEM_NO_USED;
}


static int32_t init_page_id(uint32_t blk_num, uint32_t id, bool transid_valid)
{
    return traverse_feb(1, blk_num, id, transid_valid);
}


static int32_t init_page(uint32_t blk_num)
{
    return traverse_feb(2, blk_num, 0xffffffff, false);
}


/**
 * @brief   reclaim a feb, valid item of this feb copy to reserved feb, then erased it and set this to reserved 
 * 
 * @param blk_num : feb number
 * @param old_id  : item id don't copy, this id will write a new , so old don't copy
 * @return true  : relaim ok
 * @return false 
 */
static bool reclaim_block(uint32_t blk_num, uint32_t old_id)
{
    /*mark old peb status is transfer*/
    POWEROFF_TEST(PFT_POS_SET_TRANSFERING);

    write_feb_status(blk_num,CINV_FEB_STATUS_TRANSFERING);

    traverse_feb(3, blk_num, old_id, false);

    if(CINV_ITEM_NO_USED != old_id)
    {
        /*deal erase feb after new item write done in function init_item()*/
    }
    else
    {
        /*mark new ok, call from cinv_init*/
        write_feb_status(cinv_mang_r.feb_rev,CINV_FEB_STATUS_ACTIVED);
        POWEROFF_TEST(PFT_POS_RECLAIM_BEFORE_ERASE1);
        erase_feb(blk_num);
        cinv_mang_r.feb_rev = blk_num; 
    }

    return true;
}
#endif


/**
 * @brief 
 * 
 * @param id 
 * @param len 
 * @param buf 
 * @return int32_t 
 */
static int32_t init_item(uint32_t id, uint16_t len, void *buf)
{
    int32_t ret = CINV_FEB_NO_USED;
    int32_t cnt = g_feb_total_count;
    uint32_t block = cinv_mang_r.feb_rev+1; /*first used feb after feb cinv_mang_r.feb_rev priority.*/
    uint32_t item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(len);

    // TODO: other method used,such as used empty feb first??
    do{
        block %= g_feb_total_count;

        if (block != cinv_mang_r.feb_rev)
        {
            if (item_len <= (CINV_FLASH_ERASE_BLOCK_SIZE - cinv_mang_r.p_feb_off[block] + cinv_mang_r.p_feb_lost[block]))
            {
                break;
            }
        }

        block++;
    }while (--cnt);

    if (cnt)
    {
        /* if empty space too small, first reclaim, then write*/
        if ( item_len > (CINV_FLASH_ERASE_BLOCK_SIZE - cinv_mang_r.p_feb_off[block]) )
        {
            if (reclaim_block(block, id))
            {
                POWEROFF_TEST(PFT_POS_BEFORE_WRITING_NEW);
                
                if (write_item(cinv_mang_r.feb_rev, id, len, buf))
                {
                    ret = cinv_mang_r.feb_rev;
                }
                /*mark new ok*/
                write_feb_status(cinv_mang_r.feb_rev,CINV_FEB_STATUS_ACTIVED);
                POWEROFF_TEST(PFT_POS_RECLAIM_BEFORE_ERASE2);
                erase_feb(block);
                cinv_mang_r.feb_rev = block; 
            }
        }
        else
        {
            if (write_item( block, id, len, buf ))
            {
                ret = block;
            }
        }
    }

    return ret;
}


/**
 * @brief    find a nvdata item, and return address
 * 
 * @param id : nvdata item id
 * @param ret_blk : return value feb number of address
 * @param transid_valid : if CINV_DATA_ITEM_STATUS_TRANS item used as valid item
 * @return uint16_t : offset in feb of address or CINV_FEB_NO_USED if not find
 */
static uint16_t find_item(uint32_t id, uint16_t *ret_blk, bool transid_valid)
{
    uint16_t off;
    uint16_t block;

    for (block = 0; block < g_feb_total_count; block++)
    {
        if(block == cinv_mang_r.feb_rev)/*reduce flash access*/
        {
            continue;
        }
        off = init_page_id(block, id, transid_valid);
        if(off != CINV_ITEM_NO_USED)
        {
            *ret_blk = block;
            return off;
        }
    }

    *ret_blk = CINV_FEB_NO_USED;
    return CINV_ITEM_NO_USED;
}


static uint16_t find_id(uint32_t id, uint16_t *blk)
{
    uint16_t offset;
    offset = find_hotid(id, blk);
    if(CINV_ITEM_NO_USED == offset)
    {
        offset = find_item(id, blk, true);
    }

    return offset;
}


/**
 * @brief initialize the nvdata management information and nvdata region 
 * 
 * @param flash_addr   nvdata region start address from flash
 * @param size   nvdata region size
 */
void cinv_init(uint32_t flash_addr, uint32_t size)
{
    uint32_t addr;
    cinv_feb_head_t *feb_hdr;
    uint32_t block;
    uint32_t old_block = CINV_FEB_NO_USED;

    if((0 != (flash_addr % CINV_FLASH_ERASE_BLOCK_SIZE)) || (0 != (size % CINV_FLASH_ERASE_BLOCK_SIZE)))
    {
        CI_ASSERT(0,"cinv_init para err\n");
    }

    /* 如果使能了VP，则从size里划出用了存VP模板的空间 */
    #if USE_VP
    if(size>(get_vp_template_size()*VP_TEMPLATE_NUM))
    {
        size -= get_vp_template_size()*VP_TEMPLATE_NUM;
        vp_flash_addr = flash_addr+size;
    }
    else
    {
        CI_ASSERT(0,"nvdata size too small\n");
    }
    #endif

    g_flash_base_addr = flash_addr;
    cinv_port_mutex_creat();
    cinv_port_mutex_take();
    cinv_port_flash_used_request();
    ci_logdebug(LOG_NVDATA,"\nnvdata init\n\n");
    cinv_port_flash_protect(F_DISABLE);

    cinv_mang_r.feb_rev = CINV_FEB_NO_USED;

    uint16_t block_num = (size + CINV_FLASH_ERASE_BLOCK_SIZE - 1)/CINV_FLASH_ERASE_BLOCK_SIZE;
    g_feb_total_count = block_num;

    cinv_mang_r.p_feb_off = pvPortMalloc(sizeof(uint16_t)*block_num);
    cinv_mang_r.p_feb_lost = pvPortMalloc(sizeof(uint16_t)*block_num);
    if (NULL == cinv_mang_r.p_feb_off || NULL == cinv_mang_r.p_feb_lost)
    {
        ci_logerr(LOG_NVDATA, "NVData malloc error\n");
    }

    /*erase feb if first time used, other find reserved, find transfering(relaim process) block*/
    for(block = 0; block < block_num; block++)
    {  
        //addr = CINV_FEB_NUM_TO_ADDR(block,CINV_FEB_HEADER_OFFSET);
        addr = flash_addr + CINV_FLASH_ERASE_BLOCK_SIZE*block;
        cinv_port_flash_read(addr, cinv_rd_io_buf, CINV_FEB_HEADER_SIZE);
        feb_hdr = (cinv_feb_head_t*)cinv_rd_io_buf;

        /*if feb header not valid, force erase*/
        if((CINV_FEB_HEADER_MAGIC_VALID != feb_hdr->magic) && 
            (CINV_FEB_HEADER_MAGIC_ERASED != feb_hdr->magic))
        {
            cinv_port_flash_erase(addr);
            feb_hdr->status = CINV_FEB_STATUS_ERASED;
        }

        /*find a reseved feb*/
        if(CINV_FEB_STATUS_ERASED == feb_hdr->status)
        {
            if(CINV_FEB_NO_USED == cinv_mang_r.feb_rev)
            {
                /* !!! caution !!! check_and_erase_feb reused cinv_rd_io_buf*/
                check_and_erase_feb(block);/*make sure feb is erased block*/
                cinv_mang_r.feb_rev = block;                
            }
            else
            {
                write_feb_status(block,CINV_FEB_STATUS_EMPTY);
            }
        }
        /*find a transfering feb was in progress*/
        else if(CINV_FEB_STATUS_TRANSFERING == feb_hdr->status)
        {
            old_block = block;/*can't have more than one, in cinv_init, first deal it*/
        }
        else if((CINV_FEB_STATUS_ACTIVED == feb_hdr->status) || 
            (CINV_FEB_STATUS_EMPTY == feb_hdr->status))
        {

        }
        else
        {
            cinv_port_flash_erase(addr);      
            write_feb_status(block,CINV_FEB_STATUS_EMPTY);
            cinv_log_error("\n\nunknow feb status\n\n");
        }
        //mprintf("feb_hdr->magic=0x%x, feb_hdr->status = 0x%x\n",feb_hdr->magic,feb_hdr->status);
    }

    /*if a page reclaim was interrupted before the old page was erased*/
    if (old_block != CINV_FEB_NO_USED)
    {
        /*interrupted reclaim before the target block was put in use,so do it again*/
        if (cinv_mang_r.feb_rev != CINV_FEB_NO_USED)
        {
            /* -- PFT_POS_SET_TRANSFERING -- */
            /* -- PFT_POS_BEFORE_WRITING_NEW -- */
            /* -- PFT_POS_RECLAIM_BEFORE_ERASE1 -- first:PFT_POS_BEFORE_WRITING_NEW*/
            /*already confirm cinv_mang_r.feb_rev is erased, so just retarnsfer*/
            (void)reclaim_block(old_block, CINV_ITEM_NO_USED);
        }
        /*interrupted reclaim after the target block was put in use,
          but before the old page was erased; so erase it now and create a new reserve page.*/
        else
        {
            /* -- PFT_POS_RECLAIM_BEFORE_ERASE2 -- */
            erase_feb(old_block);
            cinv_mang_r.feb_rev = old_block;
        }
    }
 
    /* else if there is no reserve page, interrupted by a erase on reset,so need find all page.
       and find transfer item, only deal transfer item as duplicate item.*/ 
    trans_blk = CINV_FEB_NO_USED;
    trans_off = CINV_ITEM_NO_USED;
    hotid_init();
    for (block = 0; block < g_feb_total_count; block++)
    {
        init_page(block);
    }

    if(CINV_ITEM_NO_USED != trans_off)
    {
        uint16_t src_off,src_blk;

        /*find again*/
        src_off = find_item(trans_id, &src_blk, false);
        if(CINV_ITEM_NO_USED != src_off)/*find done, have old one, delete it*/
        {
            /* -- PFT_POS_BEFORE_WRITING_DONE -- */
            /*delete old*/
            write_item_status(trans_blk, trans_off, CINV_DATA_ITEM_STATUS_DELETE);            
        }
        else/*no find, do nothing, old used as valid*/
        {
            /* -- PFT_POS_AFTER_MARK_TARNS -- */
        }
    }

    // TODO: deal duplicate items. now no deal this, because if hotid count not same as item count, find duplicate will recursive all find_item
    ci_logdebug(LOG_NVDATA,"cinv feb no used\n");
    if (CINV_FEB_NO_USED == cinv_mang_r.feb_rev)/*such as eraseing transfer blk*/
    {
        uint32_t most_lost = 0;

        for (block = 0; block < g_feb_total_count; block++)
        {
            if(cinv_mang_r.p_feb_lost[block] > cinv_mang_r.p_feb_lost[most_lost])
            {
                most_lost = block;
            }
        }

        cinv_mang_r.feb_rev = most_lost;
        erase_feb(cinv_mang_r.feb_rev);  /*the last page erase had been interrupted by a power-cycle.*/
    }

    /*update hotid, because our hotid count can define same as item count, so fast access item function*/    
    hotid_init();
    for (block = 0; block < g_feb_total_count; block++)
    {
        init_page(block);
    }

    cinv_port_flash_used_release();
    cinv_port_flash_protect(F_ENABLE);
    cinv_port_mutex_give();
}


/**
 * @brief If the nvdata item does not already exist, it is created and
 *          initialized with the data passed to the function, if any.
 *          This function must be called before calling cinv_item_read() or
 *          cinv_item_write().
 * 
 * @param id    Nvdata item id
 * @param len   Nvdata item length
 * @param buf   Nvdata item data pointer
 *
 * @retval CINV_ITEM_UNINIT    Id did not exist and was created successfully.  
 * @retval CINV_OPER_SUCCESS   Id already existed, no action taken.
 * @retval CINV_OPER_FAILED    Failure to find or create id.
 * @retval CINV_ITEM_LEN_ERR   Item length error
 */
cinv_item_ret_t cinv_item_init(uint32_t id, uint16_t len, void *buf)
{
    uint16_t offset;
    cinv_item_ret_t ret;
    uint16_t blk;

    if (!cinv_port_power_check())
    {
        return CINV_OPER_FAILED;
    }

    if((len > CINV_DATA_ITEM_MAX_LEN)||(0 == len))
    {
        return CINV_ITEM_LEN_ERR;
    }

    cinv_port_mutex_take();
    cinv_port_flash_used_request();

    offset = find_id(id, &blk);

    if(offset != CINV_ITEM_NO_USED)
    {
        ret = CINV_OPER_SUCCESS;
    }
    else 
    {
        cinv_port_flash_protect(F_DISABLE);
        if(init_item(id, len, buf) != CINV_FEB_NO_USED)
        {
            ret = CINV_ITEM_UNINIT;
        }
        else
        {
            ret = CINV_OPER_FAILED;
        }
        cinv_port_flash_protect(F_ENABLE);
    }
    
    cinv_port_flash_used_release();
    cinv_port_mutex_give();

    return ret;
}


/**
 * @brief Write a data item to nvdata. 
 * 
 * @param id    Nvdata item id
 * @param len   Nvdata item length
 * @param buf   Nvdata item data pointer
 *
 * @retval CINV_ITEM_UNINIT    Id did not exist.  
 * @retval CINV_OPER_SUCCESS   Write ok.
 * @retval CINV_OPER_FAILED    Failure to wirte.
 * @retval CINV_ITEM_LEN_ERR   Item length error
 */
cinv_item_ret_t cinv_item_write(uint32_t id, uint16_t len, void *buf)
{
    uint16_t src_off;
    uint16_t src_blk;
    uint16_t dst_blk;

    cinv_item_ret_t ret = CINV_OPER_SUCCESS;

    if (!cinv_port_power_check())
    {
        return CINV_OPER_FAILED;
    }
    
    if((len > CINV_DATA_ITEM_MAX_LEN)||(0 == len))
    {
        return CINV_ITEM_LEN_ERR;
    }
    
    cinv_port_mutex_take();
    cinv_port_flash_used_request();

    src_off = find_id(id, &src_blk);
    
    if (CINV_ITEM_NO_USED == src_off)
    {
        ret = CINV_ITEM_UNINIT;
    }
    else
    {
        cinv_port_flash_protect(F_DISABLE);
        
        write_item_status(src_blk, src_off, CINV_DATA_ITEM_STATUS_TRANS);
        
        POWEROFF_TEST(PFT_POS_AFTER_MARK_TARNS);
        
        dst_blk = init_item(id, len, buf);

        if (CINV_FEB_NO_USED == dst_blk)
        {
            ret = CINV_OPER_FAILED;
        }
        else if(src_blk != cinv_mang_r.feb_rev)
        {
            write_item_status(src_blk, src_off, CINV_DATA_ITEM_STATUS_DELETE);
        }

        cinv_port_flash_protect(F_ENABLE);
    }

    cinv_port_flash_used_release();
    cinv_port_mutex_give();
    
    return ret;
}


/**
 * @brief Read data from nvdata. This function can be used to read an entire item or short.
 *         Read data is copied into *buf. If input length longer than item real length, read
 *         item real length data to *buf.
 * 
 * @param id     Nvdata item id
 * @param len    Read length of this nvdata item
 * @param buf    Nvdata item data pointer
 * @param real_len   Nvdata item real length to read. 
 *
 * @retval CINV_ITEM_UNINIT    Id did not exist.  
 * @retval CINV_OPER_SUCCESS   Read ok.
 * @retval CINV_OPER_FAILED    Failure to read.
 */
cinv_item_ret_t cinv_item_read(uint32_t id, uint16_t len, void *buf, uint16_t *real_len)
{
    uint16_t offset;
    uint16_t blk;
    cinv_item_ret_t ret = CINV_OPER_SUCCESS;
    uint32_t addr;
    uint16_t item_len = sizeof(cinv_data_item_header_t) + CINV_ITEM_LEN_ACTUAL_USED(len);
    cinv_data_item_header_t *di_hdr;
    uint32_t chk_sum;

    item_len = item_len > CINV_IO_BUFFER_SIZE ? CINV_IO_BUFFER_SIZE:item_len;

    cinv_port_mutex_take();
    cinv_port_flash_used_request();

    offset = find_id(id, &blk);

    if(CINV_ITEM_NO_USED == offset)
    {
        ret = CINV_ITEM_UNINIT;
    }
    else
    {
        addr = CINV_FEB_NUM_TO_ADDR(blk, offset);
        cinv_port_flash_read(addr, cinv_rd_io_buf, item_len);
        di_hdr = (cinv_data_item_header_t *)cinv_rd_io_buf;
        chk_sum = calc_chk_sum(di_hdr->buf, di_hdr->len);
        if(chk_sum != di_hdr->chk_sum)
        {
            ret = CINV_OPER_FAILED;
        }
        else
        {
            if(len <= di_hdr->len)
            {
                *real_len = len;
            }
            else
            {
                *real_len = di_hdr->len;
            }
            MASK_ROM_LIB_FUNC->newlibcfunc.memcpy_p((uint8_t *)buf, di_hdr->buf, *real_len);
        }
    }

    cinv_port_flash_used_release();
    cinv_port_mutex_give();
    return ret;
}


/**
 * @brief Delete data item from nvdata.
 * 
 * @param id    Nvdata item id
 *
 * @retval CINV_ITEM_UNINIT    Id did not exist and was created successfully.  
 * @retval CINV_OPER_SUCCESS   Delete ok.
 * @retval CINV_OPER_FAILED    Failure to delete.
 * @retval CINV_ITEM_LEN_ERR   Item length error
 */
cinv_item_ret_t cinv_item_delete(uint32_t id)
{
    uint16_t offset;
    uint16_t blk;
    cinv_item_ret_t ret = CINV_OPER_SUCCESS;

    cinv_port_mutex_take();
    cinv_port_flash_used_request();

    offset = find_id(id, &blk);
    if (offset == CINV_ITEM_NO_USED)
    {
        /*NV item does not exist*/
        ret = CINV_ITEM_UNINIT;
    }
    else
    {
        hotid_update(CINV_FEB_NO_USED, CINV_ITEM_NO_USED, id);

        POWEROFF_TEST(PFT_POS_DELETE_ITEM);

        cinv_port_flash_protect(F_DISABLE);
        write_item_status(blk, offset, CINV_DATA_ITEM_STATUS_DELETE);
        cinv_port_flash_protect(F_ENABLE);

        /*verify that item has been removed*/
        offset = find_id(id, &blk);
        if ( offset != CINV_ITEM_NO_USED )
        {
            ret = CINV_OPER_FAILED;
        }
    }

    cinv_port_flash_used_release();
    cinv_port_mutex_give();
    return ret;
}


