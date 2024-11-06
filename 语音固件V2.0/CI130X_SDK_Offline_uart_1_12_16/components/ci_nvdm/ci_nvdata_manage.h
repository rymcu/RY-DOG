/**
 * @file ci_nvdata_manage.h
 * @brief 
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

    
#ifndef _CI_NVDATA_MANGE_H_
#define _CI_NVDATA_MANGE_H_


#ifdef __cplusplus
extern "C"{
#endif


/**************************************************************************
                    user modify defined
****************************************************************************/

/*ID don't use 0xFF000000 to 0xFFFFFFFF*/
#define NVDATA_ID_VOLUME                0x50000001 /*'VOLU'*/
#define NVDATA_ID_CWSL_START             0x50001000 /*CWSL*/

#define NVDATA_ID_VP_MOULD_INFO         0xA0000001 /*VP mould info*/

#define NVDATA_ID_USER_START            0x60000001 /*Start ID for user define. The ID user defined mast greater than this.*/
/*Add user ID here */
#define NVDATA_ID_USER_WAKEUP_NET		0x60000004

#define NVDATA_ID_USER_END              0xF0000000 /*End ID for user define. The ID user defined mast smaller than this.*/

//#define CINV_FLASH_BASE_ADDR        (0x7F0000)      //xiang.x:use the last 64k flash when test

#define CINV_FLASH_ERASE_BLOCK_SIZE (4096)/*4KB,32KB,64KB,4KB suggest,64KB max,so some used uint16_t for save code size*/

//#define CINV_FEB_TOTAL_COUNT        (16)/*flash total used size is CINV_FLASH_ERASE_BLOCK_SIZE*CINV_FEB_TOTAL_COUNT*/

/*last byte also can for version, if want used all new nvdata, can modify this, will casue erase all feb*/
#define CINV_FEB_HEADER_MAGIC_VALID (0xABCDEF03)

/**************************************************************************
                    
****************************************************************************/
/**
 * @addtogroup nv_data
 * @{
 */

/**
 * @brief nvdata function return value
 * 
 */
typedef enum
{
    CINV_ITEM_UNINIT,/*!< Id did not exist and was created successfully  */
    CINV_OPER_FAILED,/*!< Id already existed, no action taken  */
    CINV_OPER_SUCCESS,/*!< Failure to find or create id  */
    CINV_ITEM_LEN_ERR,/*!< Item length error */
}cinv_item_ret_t;

void cinv_init(uint32_t flash_addr, uint32_t size);
cinv_item_ret_t cinv_item_init(uint32_t id, uint16_t len, void *buf);
cinv_item_ret_t cinv_item_read(uint32_t id, uint16_t len, void *buf, uint16_t *real_len);
cinv_item_ret_t cinv_item_write(uint32_t id, uint16_t len, void *buf);
cinv_item_ret_t cinv_item_delete(uint32_t id);
cinv_item_ret_t cinv_register_hotid(uint32_t hot_id);

/** @} */  

void nvdata_test(void *p_arg);

#ifdef __cplusplus
}
#endif

#endif

