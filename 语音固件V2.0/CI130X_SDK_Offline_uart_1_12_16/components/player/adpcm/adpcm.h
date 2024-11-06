#ifndef __ADPCM_H
#define __ADPCM_H

#include <stdint.h>

uint8_t ADPCM_Encode(int32_t sample);
int16_t ADPCM_Decode(uint8_t code);
int32_t ADPCM_Decode_clear(void);
void set_adpcm_decode_index(int16_t decode_index);
void set_adpcm_decode_predsample(int32_t decode_predsample);

#endif /* __ADPCM_H*/
