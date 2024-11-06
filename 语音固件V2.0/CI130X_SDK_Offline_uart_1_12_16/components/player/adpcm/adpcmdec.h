/**
 * @file adpcmdec.h
 * @brief adpcm解码器
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _ADPCMDEC_H_
#define _ADPCMDEC_H_


void de_codeadpcm_mono_to_stereo(char* src,unsigned int size,short* dst);
void de_codeadpcm_mono_to_mono(char* src,unsigned int size,short* dst);

#endif /* _ADPCMDEC_H_ */
