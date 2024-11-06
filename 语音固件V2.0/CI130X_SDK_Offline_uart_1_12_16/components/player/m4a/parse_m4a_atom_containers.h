/**
 * @file parse_m4a_atom_containers.h
 * @brief 解析m4a格式头信息以获取aac解码需要的信息
 * @version 1.0
 * @date 2019-07-18
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _PARSE_M4A_ATOM_CONTAINERS_H_
#define _PARSE_M4A_ATOM_CONTAINERS_H_

#include <stdint.h>
#include "parse_m4a_atom_containers_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct sound_format
{
	char audioFmtString[4];
	uint8_t reserved[6];
	uint16_t dataRefIndex;
	uint16_t version;
	uint16_t revision;
	uint32_t vendor;
	uint16_t numChannel;
	uint16_t sampleSize;
	uint16_t complesionID;
	uint16_t packetSize;
	uint16_t sampleRateInteger;
	uint16_t sampleRateReminder;
} sound_format;

typedef struct sound_flag
{
	uint32_t process, complete;
} sound_flag;

typedef struct media_sound
{
	sound_flag flag;
	sound_format format;
	uint32_t ctime;
	uint32_t mtime;
	uint32_t timeScale;
	uint32_t duration;
} media_sound;

typedef struct esds_format
{
	char esdsString[4];
//	uint8_t reserved[22];
	uint32_t maxBitrate, avgBitrate;
} esds_format;

typedef struct media_info_typedef
{
	media_sound sound;
	sound_format format;
	esds_format bitrate;
} media_info_typedef;

typedef struct 
{
	int numEntry;
	int mdatOffset;
}aac_stco_Typedef;

typedef struct 
{
	uint8_t *nameTag;
	uint8_t *artistTag;
	uint8_t *albumTag;
	media_info_typedef *media_info;
	aac_stco_Typedef *aac_stco_struct;
}m4a_file_info_t;

enum AtomEnum {
	FTYP, /* - */ WIDE, /* - */ MDAT, /* - */ MOOV, /* + */ MVHD, /* - */ TRAK, /* + */
	TKHD, /* - */ TAPT, /* + */ CLEF, /* - */ PROF, /* - */ ENOF, /* - */ EDTS, /* + */
	ELST, /* - */ MDIA, /* + */ MDHD, /* - */ HDLR, /* - */ MINF, /* + */ VMHD, /* - */
	SMHD, /* - */ DINF, /* + */ DREF, /* - */ STBL, /* + */ STSD, /* - */ STTS, /* - */
	STSC, /* - */ STSZ, /* - */ STCO, /* - */ UDTA, /* + */ FREE, /* - */ SKIP, /* - */
	META, /* + */ LOAD, /* - */ IODS, /* - */ ILST, /* + */ KEYS, /* - */ DATA, /* - */
	TRKN, /* + */ DISK, /* + */ CPIL, /* + */ PGAP, /* + */ TMPO, /* + */ GNRE, /* + */
	COVR, /* - */ CNAM, /* - */ CART, /* - */ CALB, /* - */ CGEN, /* - */ CDAY, /* - */
	CTOO, /* - */ CWRT, /* - */ NONE, /* - */
};



int32_t alloc_m4a_info(m4a_file_info_t* m4a_file_info);
int32_t free_m4a_info(m4a_file_info_t* m4a_file_info);
int collectMediaData(mem_file_t *fp,m4a_file_info_t* m4a_file_info, uint32_t parentAtomSize, uint32_t child);

#ifdef __cplusplus
}
#endif

#endif /* _PARSE_M4A_ATOM_CONTAINERS_H_ */
