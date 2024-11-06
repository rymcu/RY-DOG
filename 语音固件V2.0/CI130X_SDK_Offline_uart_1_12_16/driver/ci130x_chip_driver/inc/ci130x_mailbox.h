/**
  ******************************************************************************
  * @file  ci130x_mailbox.h 
  * @author  
  * @version V1.0.0
  * @date  2018.10.31
  * @brief 
  ******************************************************************************
  **/ 
#include <stdint.h>

#ifndef _CI130X_MAILBOX_H_
#define _CI130X_MAILBOX_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mailbox_irq_cmd_cb_t)(uint32_t data0, uint32_t data1);

typedef enum {
    MAILBOX_POWERUP_CMD = 0,
    MAILBOX_RPMSG_CMD = 1,
    MAILBOX_NUCLEAR_CMD = 2,

    MAILBOX_UNKNOWN_CMD = 0xFFFFFFFF,
}mailbox_cmd_t;

/**************************************************************************
                    function
****************************************************************************/
void mailbox_preinit(void);
uint32_t mailbox_init(mailbox_irq_cmd_cb_t callback_func);
void mailbox_deinit(void);
uint32_t host_mail_send_msg(uint32_t data0, uint32_t data1, mailbox_cmd_t cmd);
uint32_t host_mail_rev_msg(uint32_t *data0, uint32_t *data1, mailbox_cmd_t *cmd);
void mailboxboot_sync(void);
uint32_t mailbox_send_msg(uint32_t data0, uint32_t data1, mailbox_cmd_t cmd);


#ifdef __cplusplus
}
#endif

#endif

