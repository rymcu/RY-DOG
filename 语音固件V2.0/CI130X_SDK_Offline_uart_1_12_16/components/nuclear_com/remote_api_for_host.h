
#ifndef __REMOTE_API_FOR_HOST_H__
#define __REMOTE_API_FOR_HOST_H__

#if CORE_ID == 0


#include "ci130x_nuclear_com.h"
#include "alg_preprocess.h"

extern void RC_HOST_CALLER(set_ssp_registe(audio_capture_t* audio_capture, ci_ssp_st* ci_ssp, int module_num));
extern char RC_HOST_CALLER(set_agc_gain_enable(char agc_gain_enable));
extern void RC_HOST_CALLER(set_freqvad_start_para_gain(short vad_start_gain_t));
extern void RC_HOST_CALLER(vad_rollback_frm_cfg(short vad_back_frm));

#endif
#endif

