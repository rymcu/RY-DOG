/**
 * @file ci_voice_head.h
 * @brief wave head 结构
 * @version 0.9.0
 * @date 2019-03-28
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef CI_VOICE_HEAD_H_
#define CI_VOICE_HEAD_H_

#include <stdint.h>

#pragma pack(1)
typedef struct
{
    char RIFF[4];            //"RIFF"
    uint32_t file_size;      //文件大小
    char WAVE[4];            //"WAVE"
    char fmt[4];             //"fmt "
    uint32_t fmt_chunk_size; //fmt chunk大小
    uint16_t wav_formatag;   //编码格式: 0x0011---IMA ADPCM  0x0001---PCM
    uint16_t nChannls;       //声道数，单声道为1，双声道为2
    uint32_t nSamplesPerSec; //采样频率
    uint32_t nAvgBytesperSec;//码率 注：byte/s，即比特率/8
    uint16_t nBlockAlign;    //块对齐
    uint16_t wBitsPerSample; //WAVE文件的采样大小
} WAVEFORMATEX;

#pragma pack(1)
typedef struct{
    char fact[4];           //"fact"
    uint32_t chunksize;
    uint32_t datafactsize;  //数据转换为PCM格式后的大小。
}factchunk;

#pragma pack(1)
typedef struct 
{
    char data[4];           //"data"
    uint32_t chunksize;
}data_chunk;

#pragma pack(1)
typedef struct
{
    WAVEFORMATEX wfmt;  //36Byte
    data_chunk dchunk;  //8Byte
}WAVEFORMAT_PCM_44;

#pragma pack(1)
typedef struct
{
    WAVEFORMATEX wfmt;  //36Byte
    uint16_t sbSize;
    data_chunk dchunk;  //8Byte
}WAVEFORMAT_PCM_46;

#pragma pack(1)
typedef struct
{
    WAVEFORMATEX wfmt;  //36Byte
    uint16_t sbSize;
    uint16_t nSamplesPerBlock;
    data_chunk dchunk;  //8Byte
}WAVEFORMAT_PCM_48;

#pragma pack(1)
typedef struct
{
    WAVEFORMATEX wfmt;  //36Byte
    uint16_t sbSize;
    uint16_t nSamplesPerBlock;
    factchunk fchunk;   //12Byte
    data_chunk dchunk;  //8Byte
}WAVEFORMAT_IMAADPCM;

#pragma pack(1)
typedef struct
{
    char ID3[3];               //"ID3"
    char ver;                  //3
    char revision;             //0
    char flag;                 //0
    uint32_t total_frame_size; //标签帧大小
    char frame_ID[4];          //"PRIV"
    uint32_t frame_size;       //PRIV大小
    uint16_t frame_flag;       //0
    char CI[2];                //"CI"
    uint32_t file_size;        //文件大小
    uint32_t pcm_size;         //PCM大小
}WAVEFORMAT_MP3;


#pragma pack(1)
typedef struct
{
    char head;                 //0x00
    char size[3];              //0x0 0x0 0x22
    char info[34];             
}streaminfo_block;

#pragma pack(1)
typedef struct
{
    char head;                 //0x84
    char size[3];              //0x0 0x0 0x11
    char ci_flag_str[9];       //"filesize="
    char ci_size_str[8];       
}vorbis_comment_block;

#pragma pack(1)
typedef struct
{
    char FLAC[4];                         //"fLaC"
    streaminfo_block streaminfo;          
    vorbis_comment_block vorbis_comment; 
}WAVEFORMAT_FLAC;

#pragma pack(1)
typedef union
{
    WAVEFORMAT_PCM_44 wave_pcm_44;
    WAVEFORMAT_PCM_46 wave_pcm_46;
    WAVEFORMAT_PCM_48 wave_pcm_48;
    WAVEFORMAT_IMAADPCM wave_adpcm;
    WAVEFORMAT_MP3 wave_ci_mp3;
    WAVEFORMAT_FLAC wave_ci_flac;
}ci_voice_head_t;

#endif /* CI_VOICE_HEAD_H_ */
