/**
 * @file parse_m4a_atom_containers.c
 * @brief 解析m4a格式头信息以获取aac解码需要的信息
 * @version 1.0
 * @date 2019-07-18
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "parse_m4a_atom_containers_port.h"
#include "parse_m4a_atom_containers.h"
#include "romlib_runtime.h"


#if defined(__linux__) /* linux */
#include <endian.h>
#define RETURN_OK   (0)
#define RETURN_ERR  (-1)
#define mprintf  printf
#define USE_M4A_ATOM_PARSE_VERBOSE      1
#define BE16TOH(x)   (uint16_t)be16toh(*((uint16_t *)(x)))
#define BE32TOH(x)   (uint32_t)be32toh(*((uint32_t *)(x)))
#elif (defined(__ICCARM__) || defined(__CC_ARM) || (defined(__GNUC__) && (defined(ARM) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)))) /* arm */
#include "ci_log.h"
#include "ci130x_system.h"
#include "core_cm4.h"
#define USE_M4A_ATOM_PARSE_VERBOSE      0
#define BE16TOH(x)   (uint16_t)__REV16(*(uint16_t *)(x))
#define BE32TOH(x)   (uint32_t)__REV(*(uint32_t *)(x))
#else
#include "ci_log.h"
#include "ci130x_system.h"
#define USE_M4A_ATOM_PARSE_VERBOSE      0
#define BE16TOH(x)   (uint16_t)be16toh(*((uint16_t *)(x)))
#define BE32TOH(x)   (uint32_t)be32toh(*((uint32_t *)(x)))

static uint16_t be16toh(uint16_t value)
{
	uint16_t result;

	result = ((value & 0xff00) >> 8 )
		 | ((value & 0x00ff) << 8);
	return result;
}

static uint32_t be32toh(uint32_t value)
{
	uint32_t result;

	result =  ((value & 0xff000000) >> 24)
          | ((value & 0x00ff0000) >> 8 )
          | ((value & 0x0000ff00) << 8 )
          | ((value & 0x000000ff) << 24);
	return result;
}
#endif


static uint8_t atomHasChild[] = 
{
	0, 0, 0, 1, 0, 1,
	0, 1, 0, 0, 0, 1,
	0, 1, 0 ,0, 1, 0,
	0, 1, 0, 1, 0, 0,
	0, 0, 0, 1, 0, 0,
	1, 0, 0, 1, 0, 0,
	1, 1, 1, 1, 1, 1,
	0,
	
    0, 0,
	0, 0,
	0, 0,
	0,

	0
};
#define ATOM_ITEMS (sizeof(atomHasChild) / sizeof(atomHasChild[0]))
#define CMARK 0xA9
#define TAG_MAX_CNT 100

/* mdhd 采样频率和采样系数 */
/* stsd 音频信息,对于mp42保存在子atom "mp4a" 中,记录了音频信息采样通道和采样位数 */
/* stsz 音频数据每一帧的偏移地址，音频数据总帧数 */
/* stco 记录了音频数据的起始偏移地址 */
static const uint8_t atomTypeString[ATOM_ITEMS][5] =
{
	"ftyp", /* - */ "wide", /* - */ "mdat", /* - */ "moov", /* + */ "mvhd", /* - */ "trak", /* + */
	"tkhd", /* - */ "tapt", /* + */ "clef", /* - */ "prof", /* - */ "enof", /* - */ "edts", /* + */
	"elst", /* - */ "mdia", /* + */ "mdhd", /* - */ "hdlr", /* - */ "minf", /* + */ "vmhd", /* - */
	"smhd", /* - */ "dinf", /* + */ "dref", /* - */ "stbl", /* + */ "stsd", /* - */ "stts", /* - */
	"stsc", /* - */ "stsz", /* - */ "stco", /* - */ "udta", /* + */ "free", /* - */ "skip", /* - */
	"meta", /* + */ "load", /* - */ "iods", /* - */ "ilst", /* + */ "keys", /* - */ "data", /* - */
	"trkn", /* + */ "disk", /* + */ "cpil", /* + */ "pgap", /* + */ "tmpo", /* + */ "gnre", /* + */
	"covr", /* - */

	{CMARK, 'n', 'a', 'm', '\0'}, /* - */ {CMARK, 'A', 'R', 'T', '\0'}, /* - */
	{CMARK, 'a', 'l', 'b', '\0'}, /* - */ {CMARK, 'g', 'e', 'n', '\0'}, /* - */
	{CMARK, 'd', 'a', 'y', '\0'}, /* - */ {CMARK, 't', 'o', 'o', '\0'}, /* - */
	{CMARK, 'w', 'r', 't', '\0'}, /* - */

	"----", /* - */
};

static uint8_t atombuf[512];



/**
 * @brief 转换字节序返回atom size
 * 
 * @param atom buff地址
 * @return uint32_t atom值
 */
static inline uint32_t getAtomSize(void* atom)
{
	return BE32TOH(atom);
}


/**
 * @brief 解析m4a容器
 * 
 * @param fp 数据读取描述符
 * @param m4a_file_info m4a容器解码器
 * @param parentAtomSize 文件大小
 * @param child atom节点
 * @retval RETURN_OK 播放启动成功
 * @retval RETURN_ERR 播放启动失败
 */
int collectMediaData(mem_file_t *fp,m4a_file_info_t* m4a_file_info, uint32_t parentAtomSize, uint32_t child)
{
	int totalAtomSize = 0, contentLength = 0, atomSize, index, is;
    int f_offset = 0;
    uint32_t etds_size = 0;
    
    (void)etds_size;
    (void)contentLength;
	MASK_ROM_LIB_FUNC->newlibcfunc.memset_p(atombuf, '\0', sizeof(atombuf));
   
    
	do{
		/* 读取atom大小和类型 共8byte */
		mf_read(atombuf, 1, 8, fp);
		/* 前4byte转换字节序及为该atom大小 */
		atomSize = getAtomSize(atombuf);

		/* 后4byte为atom类型,比较获取index序号 */
		for(index = 0;index < ATOM_ITEMS;index++)
		{
			if(!MASK_ROM_LIB_FUNC->newlibcfunc.strncmp_p((char*)&atombuf[4], (char*)&atomTypeString[index][0], 4))
			{
				for(is = child;is > 0;is--) 
				{
					mprintf("-");
				}
				mprintf("%s %d\n", (char*)&atomTypeString[index][0], atomSize);
				break;
			}
		}

		/* 跳过未知的atom类型 */
		if(index >= ATOM_ITEMS)
		{
			mprintf("unrecognized atom:%s %d\n", &(atombuf[4]), atomSize);
			goto NEXT;
		}

		/* 保存当前文件指针 */
		f_offset = mf_tell(fp);

		/* 解析atuo类型 */
		switch(index)
		{
			#if USE_M4A_ATOM_PARSE_VERBOSE
			case COVR: 
				/* Cover image */
				mf_seek(fp, 16, SEEK_CUR);
				break;
			case CNAM:
				/* Title 标签值  */
				/* 保存在 用户数据(udta) 元数据的表项里 */
				/* 参考文档:https://developer.apple.com/library/archive/documentation/QuickTime/QTFF/QTFFChap2/qtff2.html#//apple_ref/doc/uid/TP40000939-CH204-SW1 */
				mf_read(atombuf, 1, 16, fp);
				contentLength = getAtomSize(atombuf) - 16;
				contentLength = contentLength < TAG_MAX_CNT ? contentLength : TAG_MAX_CNT;
				mf_read(m4a_file_info->nameTag, 1, contentLength, fp);
				m4a_file_info->nameTag[contentLength] = '\0';
				m4a_file_info->nameTag[contentLength + 1] = '\0';
				break;
			case CART:
				/* Artist 标签值 */
				/* 保存在 用户数据(udta) 元数据的表项里 */
				/* 未找到参考文档 */
				mf_read(atombuf, 1, 16, fp);
				contentLength = getAtomSize(atombuf) - 16;
				contentLength = contentLength < TAG_MAX_CNT ? contentLength : TAG_MAX_CNT;
				mf_read(m4a_file_info->artistTag, 1, contentLength, fp);
				m4a_file_info->artistTag[contentLength] = '\0';
				m4a_file_info->artistTag[contentLength + 1] = '\0';
				break;
			case CALB:
				/* Album 标签值 */
				/* 保存在 用户数据(udta) 元数据的表项里 */
				/* 未找到参考文档 */
				mf_read(atombuf, 1, 16, fp);
				contentLength = getAtomSize(atombuf) - 16;
				contentLength = contentLength < TAG_MAX_CNT ? contentLength : TAG_MAX_CNT;
				mf_read(m4a_file_info->albumTag, 1, contentLength, fp);
				m4a_file_info->albumTag[contentLength] = '\0';
				m4a_file_info->albumTag[contentLength + 1] = '\0';
				break;
			#endif
			case META:
				/* 元数据结构 */
				/* 元数据结构是一种容器,一般是自定义数据(例如:udta)中用这个组织,未找到参考文档
					[0:3]       -------------- version/flags
						跳过这4个字节解析元数据容器内容
				 */
				f_offset += 4;
				totalAtomSize -= 4;
				atomSize -= 4;
				break;
			case MDHD:
				/* Media Atoms */
				/* 参考文档:https://developer.apple.com/library/archive/documentation/QuickTime/QTFF/QTFFChap2/qtff2.html#//apple_ref/doc/uid/TP40000939-CH204-BBCGDJID */
				/* Figure 2-21 中
					[0:3]       -------------- version/flags
					[4:7]       -------------- Creation time      :    创建时间
					[8:11]      -------------- Modification time  :    修改时间
					[12:15]     -------------- time scale         :    采样率
					[16:19]     -------------- duration           :    持续时间单位为采样率
					[20:21]     -------------- language
					[22:23]     -------------- pre_defined
				 */
				#if USE_M4A_ATOM_PARSE_VERBOSE
				mf_seek(fp, 4, SEEK_CUR); // skip ver/flag
				mf_read(atombuf, 1, 4, fp); // Creation time
				m4a_file_info->media_info->sound.ctime = getAtomSize(atombuf);
				mf_read(atombuf, 1, 4, fp); // Modification time
				m4a_file_info->media_info->sound.mtime = getAtomSize(atombuf);
				#else
				mf_seek(fp, 12, SEEK_CUR); // skip ver/flag  creationtime modificationtime
				#endif

				mf_read(atombuf, 1, 4, fp); // time scale
				m4a_file_info->media_info->sound.timeScale = getAtomSize(atombuf);

				#if USE_M4A_ATOM_PARSE_VERBOSE
				mf_read(atombuf, 1, 4, fp); // duration
				m4a_file_info->media_info->sound.duration = getAtomSize(atombuf);
				#endif
				break;
			case STSD:
				/* Sample Description Atoms */
				/* 参考文档:https://developer.apple.com/library/archive/documentation/QuickTime/QTFF/QTFFChap2/qtff2.html#//apple_ref/doc/uid/TP40000939-CH204-BBCGDJID */
				/* Figure 2-34 中
					[0:3]       -------------- version/flags
					[4:7]       -------------- Number of entries
					[8: ]       -------------- Data format,即Media Data Atom Types的Atom

					存在Media Data Atom Types的Atom(列如:mp4a,avc1,mp4v,dvc,等等)
					参考文档:https://developer.apple.com/library/archive/documentation/QuickTime/QTFF/QTFFChap3/qtff3.html#//apple_ref/doc/uid/TP40000939-CH205-SW1
					
					对于mp42版本文件中为 mp4a ,保存了sound_format相关信息
					资料表明在文档<ISO / IEC FDIS 14496-15>中,但是我没找到这个文档中相关描述?
					[8:11]      -------------- atom size
					[12:15]     -------------- atom type "mp4a"
					[16:21]     -------------- Reserved (6 bytes)
					[22:23]     -------------- Data reference index
					[24:25]     -------------- Version
					[26:27]     -------------- Revision level
					[28:31]     -------------- Vendor
					[32:33]     -------------- Number of channels        :    采样率
					[34:35]     -------------- Sample size (bits)        :    持续时间单位为采样率
					[36:37]     -------------- Compression ID
					[38:39]     -------------- Packet size
					[40:41]     -------------- rate integer part  (Sample rate (16.16))
					[42:43]     -------------- rate reminder part (Sample rate (16.16))

					紧跟着是etds atom保存了码率信息,未找到参考文档
					[44:47]     -------------- atom size
					[48:51]     -------------- atom type "etds"
					[52:55]     -------------- version/flags
					[56]        -------------- 
					[57]        -------------- 
					[58:59]     -------------- ESID
					[60]        -------------- 
					[61]        -------------- 
					[62]        -------------- 
					[63]        -------------- 
					[64]        -------------- 
				 */
				mf_seek(fp, 8, SEEK_CUR); // skip Reserved(6bytes)/Data Reference Index

				mf_read(atombuf, 1, 4, fp); // media data type atom size
				mf_read(&m4a_file_info->media_info->format, 1, sizeof(sound_format), fp);	
				mprintf("Media Data Atom Types is %s\n",m4a_file_info->media_info->format.audioFmtString);

				m4a_file_info->media_info->format.numChannel = BE16TOH(&m4a_file_info->media_info->format.numChannel);
				#if USE_M4A_ATOM_PARSE_VERBOSE
				m4a_file_info->media_info->format.version = BE16TOH(&m4a_file_info->media_info->format.version);
				m4a_file_info->media_info->format.revision = BE16TOH(&m4a_file_info->media_info->format.revision);
				m4a_file_info->media_info->format.vendor = BE32TOH(&m4a_file_info->media_info->format.vendor);
				m4a_file_info->media_info->format.sampleSize = BE16TOH(&m4a_file_info->media_info->format.sampleSize);
				m4a_file_info->media_info->format.complesionID = BE16TOH(&m4a_file_info->media_info->format.complesionID);
				m4a_file_info->media_info->format.packetSize = BE16TOH(&m4a_file_info->media_info->format.packetSize);
				m4a_file_info->media_info->format.sampleRateInteger = BE16TOH(&m4a_file_info->media_info->format.sampleRateInteger);
				m4a_file_info->media_info->format.sampleRateReminder = BE16TOH(&m4a_file_info->media_info->format.sampleRateReminder);
				
				mf_read(atombuf, 1, 4, fp); // etds atom size
				etds_size = BE32TOH(&atombuf[0]);
				if(etds_size - 4 - 1 > 511)
				{
					etds_size = 511+4+1;
				}
				mf_read(atombuf, 1, 512, fp);

				m4a_file_info->media_info->bitrate.maxBitrate = 0;
				m4a_file_info->media_info->bitrate.avgBitrate = 0;

				for(int i = 0;i < etds_size - 4 - 1;i++){
					if(atombuf[i] == 0x40 && atombuf[i + 1] == 0x15){
						m4a_file_info->media_info->bitrate.maxBitrate = BE32TOH(&atombuf[i + 5]);
						m4a_file_info->media_info->bitrate.avgBitrate = BE32TOH(&atombuf[i + 9]);
						break;
					}
				}
				#endif
				break;
			case STCO:
				/* Chunk Offset Atoms */
				/* 参考文档:https://developer.apple.com/library/archive/documentation/QuickTime/QTFF/QTFFChap2/qtff2.html#//apple_ref/doc/uid/TP40000939-CH204-BBCGDJID */
				/*Figure 2-50 中
					[0:3]       -------------- version/flags
					[4:7]       -------------- Number of entries : 块偏移表中的条目数
					[8:11]      -------------- Chunk offset table:由偏移值数组组成的块偏移表(这个表项第一个即为音频数据起始的偏移地址
				 */
				mf_seek(fp, 4, SEEK_CUR); // skip flag ver
				mf_read(atombuf, 1, 4, fp); // numEntry
				#if USE_M4A_ATOM_PARSE_VERBOSE
				m4a_file_info->aac_stco_struct->numEntry = getAtomSize(atombuf);
				#endif
				mf_read(atombuf, 1, 4, fp); // aac_stco_size
				m4a_file_info->aac_stco_struct->mdatOffset = getAtomSize(atombuf);
				break;
			default:
				break;
		}

		/* 恢复文件指针位置 */
		mf_seek(fp,f_offset,SEEK_SET);

		/* 如果该atom存在子atom */
		if(atomHasChild[index])
		{	
			/* 递归查找子atom,查找范围为当前atom区域,大小为atomSize-8 */
			if(collectMediaData(fp, m4a_file_info, atomSize - 8, child + 1) != 0) // Re entrant
			{ 
				return -1;
			}
			/* 查找完毕再次恢复文件指针位置 */
			mf_seek(fp,f_offset,SEEK_SET);
		}

NEXT:
		/* 当前atom查找完毕,跳过当前atom */
		mf_seek(fp, atomSize - 8, SEEK_CUR);
		totalAtomSize += atomSize;
		//mprintf("parentAtomSize:%d totalAtomSize:%d\n", parentAtomSize, totalAtomSize);
	}while(parentAtomSize > (totalAtomSize + 8));

	return 0;
}


/**
 * @brief 申请m4a容器解码器
 * 
 * @param m4a_file_info m4a容器解码器
 * @retval RETURN_OK 申请成功
 * @retval RETURN_ERR 申请失败
 */
int32_t alloc_m4a_info(m4a_file_info_t* m4a_file_info)
{
	#if USE_M4A_ATOM_PARSE_VERBOSE
	m4a_file_info->nameTag = (uint8_t *)malloc(TAG_MAX_CNT*sizeof(uint8_t));
	m4a_file_info->artistTag = (uint8_t *)malloc(TAG_MAX_CNT*sizeof(uint8_t));
	m4a_file_info->albumTag = (uint8_t *)malloc(TAG_MAX_CNT*sizeof(uint8_t));

	if(!(m4a_file_info->nameTag && m4a_file_info->artistTag && m4a_file_info->albumTag))
	{
		return RETURN_ERR;
	}
	#endif

	m4a_file_info->media_info = (media_info_typedef *)malloc(sizeof(media_info_typedef));
	m4a_file_info->aac_stco_struct = (aac_stco_Typedef *)malloc(sizeof(aac_stco_Typedef));
	
	if(m4a_file_info->media_info && m4a_file_info->aac_stco_struct)
	{
		return RETURN_OK;
	}
	else
	{
		return RETURN_ERR;
	}
	
}


/**
 * @brief 销毁m4a容器解码器
 * 
 * @param m4a_file_info m4a容器解码器
 * @retval RETURN_OK 销毁成功
 * @retval RETURN_ERR 销毁失败
 */
int32_t free_m4a_info(m4a_file_info_t* m4a_file_info)
{
	#if USE_M4A_ATOM_PARSE_VERBOSE
	free(m4a_file_info->nameTag);
	free(m4a_file_info->artistTag);
	free(m4a_file_info->albumTag);
	#endif
	free(m4a_file_info->media_info);
	free(m4a_file_info->aac_stco_struct);
	return RETURN_OK;
}

#if 0
int main(int argc,char** argv)
{
	m4a_file_info_t m4a_file_info;
	FILE *fd_m4a = NULL;
	uint8_t *m4a_file = NULL;
    int32_t filesize = 0;
    uint8_t atombuf[8];
	
	if(argc == 2)
	{
		printf("开始解码%s\n",argv[1]);
		/* 打开待解码文件 */
    	fd_m4a = fopen(argv[1], "r");
	}
	else
	{
		printf("arg err!\n");
		return -1;
	}

	/* 获取文件大小 */
    fseek(fd_m4a, 0L, SEEK_END);  
    filesize = ftell(fd_m4a);  
    fseek(fd_m4a, 0, SEEK_SET);
	if(filesize >= 1024*1024)
	{
		printf("文件大小:%.2fMB \n\n",(float)filesize/(1024.0*1024.0));
	}
	else if(filesize >= 1024)
	{
		printf("文件大小:%.2fKB \n\n",(float)filesize/(1024.0));
	}
	else
	{
		printf("文件大小:%dB \n\n",filesize);
	}

	/* 读取文件到m4a_file缓冲器 */
	m4a_file = (uint8_t *)malloc(filesize*sizeof(uint8_t));
	fread(m4a_file,sizeof(uint8_t),filesize,fd_m4a);
	fclose(fd_m4a);


	/* 创建atom解析器 */
	mem_file_t *mf_file = mf_open(m4a_file,filesize,NULL);
	alloc_m4a_info(&m4a_file_info);
	collectMediaData(mf_file,&m4a_file_info, filesize,0);
    
	/* 打印获取的音频详细信息 */
	printf("\n\n解码M4A文件头完成，音频信息如下:\n");
	printf("-------------------------------------------\n");
	printf("nameTag:                         %s\n", m4a_file_info.nameTag);
	printf("artistTag:                       %s\n", m4a_file_info.artistTag);
	printf("albumTag:                        %s\n", m4a_file_info.albumTag);
	printf("-------------------------------------------\n");
	
	printf("media_info.format.version:       %d\n", m4a_file_info.media_info->format.version);
	printf("media_info.format.revision:      %d\n", m4a_file_info.media_info->format.revision);
	printf("media_info.format.vendor:        %d\n", m4a_file_info.media_info->format.vendor);
	printf("media_info.format.complesionID:  %d\n", m4a_file_info.media_info->format.complesionID);
	printf("media_info.format.packetSize:    %d\n", m4a_file_info.media_info->format.packetSize);
	printf("media_info->bitrate.avgBitrate:  %d\n", m4a_file_info.media_info->bitrate.avgBitrate);
	printf("media_info->bitrate.maxBitrate:  %d\n", m4a_file_info.media_info->bitrate.maxBitrate);
	printf("-------------------------------------------\n");
	printf("totalSec:                        %d\n", (int)((float)m4a_file_info.media_info->sound.duration / (float)m4a_file_info.media_info->sound.timeScale + 0.5));
	printf("sampleSize:                      %d\n", m4a_file_info.media_info->format.sampleSize);
	printf("sampleRateInteger:               %d\n", m4a_file_info.media_info->format.sampleRateInteger);
	printf("sampleRateReminder:              %d\n", m4a_file_info.media_info->format.sampleRateReminder);
	printf("numChannel:                      %d\n", m4a_file_info.media_info->format.numChannel);
	printf("duration:                        %d\n", m4a_file_info.media_info->sound.duration);
	printf("timeScale:                       %d\n", m4a_file_info.media_info->sound.timeScale);
	printf("Creation time:                   %d\n", m4a_file_info.media_info->sound.ctime);
	printf("Modification time:               %d\n", m4a_file_info.media_info->sound.mtime);
	printf("-------------------------------------------\n");
	printf("mdatOffset:                      0x%x\n",m4a_file_info.aac_stco_struct->mdatOffset);
	printf("-------------------------------------------\n\n\n");
	

	printf("调用helix解码器AACSetRawBlockParams,配置参数,即可开始解码,示例如下:\n");
	printf("---------------------------------------------------------------\n");
	printf("readptr = buffbase + mdatOffset;        /*mdatOffset: 0x%x*/\n",m4a_file_info.aac_stco_struct->mdatOffset);
	printf("aacFrameInfo.sampRateCore = timeScale;  /*timeScale : %d*/\n", m4a_file_info.media_info->sound.timeScale);
	printf("aacFrameInfo.nChans = numChannel;       /*numChannel: %d*/\n", m4a_file_info.media_info->format.numChannel);
	printf("aacFrameInfo.profile = 0;\n");
	printf("AACSetRawBlockParams(hAACDecoder, 0, &aacFrameInfo);\n");
	printf("---------------------------------------------------------------\n");
	printf("\n\n");
		
	mf_close(mf_file);
	free(m4a_file);

    return 0;
}
#endif
