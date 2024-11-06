#ifndef __FLASH_MANAGE_OUTSIDE_PORT_H_
#define __FLASH_MANAGE_OUTSIDE_PORT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


int32_t post_write_flash(char *buf, uint32_t addr, uint32_t size);
int32_t post_read_flash(char *buf, uint32_t addr, uint32_t size);
int32_t post_erase_flash(uint32_t addr, uint32_t size);
void flash_manage_nuclear_com_outside_port_init(void);
int32_t post_spic_read_unique_id(uint8_t* unique_id);
void is_flash_power_off(bool* state);

#ifdef __cplusplus
}
#endif

#endif