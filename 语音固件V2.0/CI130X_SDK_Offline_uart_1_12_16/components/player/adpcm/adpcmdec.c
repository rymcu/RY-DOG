/**
 * @file adpcmdec.c
 * @brief adpcm解码器
 * @version 1.0
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#include "adpcmdec.h"
#include "adpcm.h"
#include "ci_log.h"
#include <string.h>
#include "sdk_default_config.h"


/**
 * @brief adpcm解码算法
 * 
 * @param src 
 * @param size 
 * @param dst 
 */
void de_codeadpcm_mono_to_stereo(char *src, unsigned int size, short *dst)
{
    unsigned int realsize = size;
    unsigned char tmp = 0, *psrc = (unsigned char *)src;
    short dsttmp = 0;
    if (!src || !dst || !size)
    {
        return;
    }
    for (unsigned int i = 0; i < realsize; i++)
    {
        tmp = *psrc++;
        dsttmp = ADPCM_Decode(tmp & 0xF);
        *dst++ = dsttmp;
        *dst++ = dsttmp;
        dsttmp = ADPCM_Decode((tmp >> 4) & 0xF);
        *dst++ = dsttmp;
        *dst++ = dsttmp;
    }
}


/**
 * @brief adpcm解码算法
 * 
 * @param src 
 * @param size 
 * @param dst 
 */
void de_codeadpcm_mono_to_mono(char *src, unsigned int size, short *dst)
{
    unsigned int realsize = size;
    unsigned char tmp = 0, *psrc = (unsigned char *)src;
    short dsttmp = 0;
    if (!src || !dst || !size)
    {
        return;
    }
    for (unsigned int i = 0; i < realsize; i++)
    {
        tmp = *psrc++;
        #if AUDIO_PLAY_USE_QSPI_FLASH_LIST
        if(tmp == 's')
        {
          if(MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p(psrc,"plc",3) == 0)
            {
                ADPCM_Decode_clear();
                tmp = 0;
                *(psrc) = 0;
                *(psrc+1) = 0;
                *(psrc+2) = 0;
            }
        }
        #endif
        dsttmp = ADPCM_Decode(tmp & 0xF);
        *dst++ = dsttmp;
        dsttmp = ADPCM_Decode((tmp >> 4) & 0xF);
        *dst++ = dsttmp;
    }
}

