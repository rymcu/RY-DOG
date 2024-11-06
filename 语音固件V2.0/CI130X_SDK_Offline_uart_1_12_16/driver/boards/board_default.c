#include "ci130x_core_misc.h"
#include "ci130x_dpmu.h"
#include "board.h"
#include "ci_log.h"

__WEAK void pad_config_for_iis(void)
{
     mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_power_amplifier(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void power_amplifier_on(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void power_amplifier_off(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_i2c(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_color_light(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void night_light_disable(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void night_light_enable(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_night_light(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void blink_light_on_off(uint8_t on_off)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_blink_light(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void vad_light_on(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void vad_light_off(void)
{ 
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void pad_config_for_vad_light(void)
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void audio_in_codec_registe()
{
    mprintf("no function: %s\n", __FUNCTION__);
}

__WEAK void board_clk_source_set(void)
{
    dpmu_unlock_cfg_config();
    #if USE_EXTERNAL_CRYSTAL_OSC
    dpmu_set_src_source(DPMU_SRC_USE_OUTSIDE_OSC);
    #else
    dpmu_set_src_source(DPMU_SRC_USE_INNER_RC);
    #endif
    dpmu_lock_cfg_config();
}
