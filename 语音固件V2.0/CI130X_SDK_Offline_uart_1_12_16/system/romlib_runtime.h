#ifndef ROMLIB_RUNTIME_H
#define ROMLIB_RUNTIME_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

typedef void *HMP3Decoder;

typedef void *MP3SaveLastInfo;

typedef struct _MP3FrameInfo {
	int bitrate;
	int nChans;
	int samprate;
	int bitsPerSample;
	int outputSamps;
	int layer;
	int version;
} MP3FrameInfo;

typedef enum {
	MPEG1 =  0,
	MPEG2 =  1,
	MPEG25 = 2
} MPEGVersion;

enum {
	ERR_MP3_NONE =                  0,
	ERR_MP3_INDATA_UNDERFLOW =     -1,
	ERR_MP3_MAINDATA_UNDERFLOW =   -2,
	ERR_MP3_FREE_BITRATE_SYNC =    -3,
	ERR_MP3_OUT_OF_MEMORY =	       -4,
	ERR_MP3_NULL_POINTER =         -5,
	ERR_MP3_INVALID_FRAMEHEADER =  -6,
	ERR_MP3_INVALID_SIDEINFO =     -7,
	ERR_MP3_INVALID_SCALEFACT =    -8,
	ERR_MP3_INVALID_HUFFCODES =    -9,
	ERR_MP3_INVALID_DEQUANTIZE =   -10,
	ERR_MP3_INVALID_IMDCT =        -11,
	ERR_MP3_INVALID_SUBBAND =      -12,

	ERR_UNKNOWN =                  -9999
};

typedef struct
{
    uint16_t fftLen;                   /**< length of the FFT. */
	const float *pTwiddle;             /**< points to the Twiddle factor table. */
	const float *pTwiddle2;   
	const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
    uint16_t bitRevLength;             /**< bit reversal table length. */
} riscv_cfft_instance_f32;
  
typedef struct
{
    riscv_cfft_instance_f32 Sint;      /**< Internal CFFT structure. */
    uint16_t fftLenRFFT;             /**< length of the real sequence */
    const float * pTwiddleRFFT;        /**< Twiddle factors real stage  */
	const float * pTwiddleRFFT2; 
} riscv_rfft_fast_instance_f32 ;

typedef struct romruntime_func_s {
	char ci_lib_romruntime[8];
	uint32_t verison;
	uint32_t reserved;
	int (*init_lib_romruntime_p)(void);
	struct mp3func_s {
		void        (*MP3Lib_Set_Func_p)(void* f1, void* f2,void* f3,void* f4,void* f5);
		HMP3Decoder (*MP3InitDecoder_p)(void);
		int         (*MP3ClearLastDecodeMainData_p)(HMP3Decoder hMP3Decoder);
		int         (*MP3SaveLastDecodeMainData_p)(HMP3Decoder hMP3Decoder,MP3SaveLastInfo saveInfo);
		int         (*MP3LoadLastDecodeMainData_p)(HMP3Decoder hMP3Decoder,MP3SaveLastInfo saveInfo);
		int         (*MP3FindSyncWord_p)(unsigned char *buf, int nBytes);
		int         (*MP3Decode_p)(HMP3Decoder hMP3Decoder, unsigned char **inbuf, int *bytesLeft, short *outbuf, int useSize);
		void        (*MP3GetLastFrameInfo_p)(HMP3Decoder hMP3Decoder, MP3FrameInfo *mp3FrameInfo);
		void        (*MP3FreeDecoder_p)(HMP3Decoder hMP3Decoder);
	}mp3func;
	struct fftfunc_s {
		void (*riscv_rfft_fast_f32_p)(riscv_rfft_fast_instance_f32 * S,	float * p,float * pOut,uint8_t ifftFlag);
		int (*riscv_rfft_fast_init_f32_p)( riscv_rfft_fast_instance_f32 * S,unsigned int fft_len);}fftfunc;
	struct newlibcfunc_s {
		void 	    (*NEWLib_Set_Func_p)(void* f1,void* f2,void* f3);
		int 	    (*memcmp_p)(const void *, const void *, size_t);
		void *      (*memcpy_p)(void *__restrict, const void *__restrict, size_t);
		void *	    (*memmove_p)(void *, const void *, size_t);
		void *	    (*memset_p)(void *, int, size_t);
		char *      (*strcat_p)(char *__restrict, const char *__restrict);
		int	        (*strcmp_p)(const char *, const char *);
		char *      (*strcpy_p)(char *__restrict, const char *__restrict);
		size_t	    (*strlen_p)(const char *);
		char *      (*strncat_p)(char *__restrict, const char *__restrict, size_t);
		int	        (*strncmp_p)(const char *, const char *, size_t);
		char *      (*strncpy_p)(char *__restrict, const char *__restrict, size_t);
		char *      (*strstr_p)(const char *, const char *);
		char *      (*stpcpy_p)(char *__restrict, const char *__restrict);
		char *      (*stpncpy_p)(char *__restrict, const char *__restrict, size_t);
		size_t	    (*strnlen_p)(const char *, size_t);
		unsigned long (*strtoul_p)(const char *__restrict __n, char **__restrict __end_PTR, int __base);
		long	    (*atol_p)(const char *__nptr);
		void	    (*qsort_p)(void *__base, size_t __nmemb, size_t __size, __compar_fn_t _compar);
		void *	    (*malloc_p)(size_t __size);
		void	    (*free_p)(void *);
		int	        (*vsnprintf_p)(char *__restrict, size_t, const char *__restrict, __VALIST);
		int	        (*sprintf_p) (char *__restrict, const char *__restrict, ...);
		int			(*sscanf_p)(const char *__restrict, const char *__restrict, ...);
		int	  (*abs_p)(int);
		int   (*isnan_p)(double);
		int   (*isinf_p)(double);
		float (*cosf_p)(float);
		float (*sinf_p)(float);
		float (*tanf_p)(float);
		float (*expf_p)(float);
		float (*sqrtf_p)(float);
		float (*fabsf_p)(float);
		float (*logf_p)(float);
		float (*log10f_p)(float);
	}newlibcfunc;
} romruntime_func_tab_t;

#define MASK_ROM_BOOT_CODE_ADDR      (0x1F000000)    
#define MASK_ROM_CORE_MAGIC_NUM_ADDR (0x1F000000+0x1C)
#define MASK_ROM_WINDOW_ADDR         (0x1F000000+79*1024)                              // sin窗函数
#define MASK_ROM_FFT_BIT_ADDR        (0x1F000000+8*1024)                               // 512fft的表1
#define MASK_ROM_FFT_COEF_R_ADDR     (MASK_ROM_FFT_BIT_ADDR + sizeof(short)*440)       // 512fft的表2
#define MASK_ROM_FFT_COEF_ADDR       (MASK_ROM_FFT_COEF_R_ADDR + sizeof(float)*512)    // 512fft的表3
#define MASK_ROM_MEL_SCALE_ADDR      (MASK_ROM_FFT_COEF_ADDR + sizeof(float)*512)      // mel需要的表1
#define MASK_ROM_MEL_OFFSET_ADDR     (MASK_ROM_MEL_SCALE_ADDR + sizeof(float)*512)     // mel需要的表2
#define MASK_ROM_ASR_WINDOW_ADDR     (MASK_ROM_MEL_OFFSET_ADDR + sizeof(short)*(60))   // fe 需要的窗函数
#define MASK_ROM_PCM_TABLE_ADDR      (MASK_ROM_ASR_WINDOW_ADDR + sizeof(short)*(400))  // ADPCM需要用的表
#define MASK_ROM_LIB_RUNTIME_ADDR    (0x1F000000+16*1024)

#define MASK_ROM_LIB_FUNC            ((romruntime_func_tab_t* )MASK_ROM_LIB_RUNTIME_ADDR)



#endif /*ROMLIB_RUNTIME_H*/
