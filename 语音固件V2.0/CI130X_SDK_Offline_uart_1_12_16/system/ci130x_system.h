/**
 * @file ci130x_system.h
 * @brief chip级定义
 * @version 1.0
 * @date 2021-06-30
 * 
 * @copyright Copyright (c) 2021 Chipintelli Technology Co., Ltd.
 * 
 */
#ifndef _CI130X_SYSTEM_H_
#define _CI130X_SYSTEM_H_

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*******function return defines******/
#define INT32_T_MAX (0x7fffffff)
#define INT32_T_MIN (0x80000000)

enum _retval
{
    RETURN_OK = 0,       RET_SUCCESS = RETURN_OK,
    PARA_ERROR = -1,     RET_INVALIDARGMENT = PARA_ERROR,
    RETURN_ERR = -2,     RET_FAIL = RETURN_ERR,
                         RET_MOMEM = -3,
                         RET_READONLY = -4,
                         RET_OUTOFRANGE = -5,
                         RET_TIMEOUT = -6,
                         RET_NOTRANSFEINPROGRESS = -7,

                         RET_UNKNOW = INT32_T_MIN,
};


typedef enum IRQn
{
/******  RISC-V N307 Processor Exceptions Numbers *******************************/
    MSIP_IRQn                 = 3,      /*!<            */
    MTIP_IRQ                  = 7,      /*!<            */
/****** smt specific Interrupt Numbers ****************************************/
    TWDG_IRQn                 = 19 + 0,      /*!<            */
    SCU_IRQn                  = 19 + 1,      /*!<            */
    NPU_IRQn                  = 19 + 2,      /*!<            */
    ADC_IRQn                  = 19 + 3,      /*!<            */
    DMA_IRQn                  = 19 + 4,      /*!<            */
    TIMER0_IRQn               = 19 + 5,      /*!<            */
    TIMER1_IRQn               = 19 + 6,      /*!<            */
    TIMER2_IRQn               = 19 + 7,      /*!<            */
    TIMER3_IRQn               = 19 + 8,      /*!<            */
    IIC0_IRQn                 = 19 + 9,      /*!<            */
    PA_IRQn                   = 19 + 10,     /*!<            */
    PB_IRQn                   = 19 + 11,     /*!<            */
    UART0_IRQn                = 19 + 12,     /*!<            */
    UART1_IRQn                = 19 + 13,     /*!<            */
    UART2_IRQn                = 19 + 14,     /*!<            */
    IIS0_IRQn                 = 19 + 15,     /*!<            */
    IIS1_IRQn                 = 19 + 16,     /*!<            */
    IIS2_IRQn                 = 19 + 17,     /*!<            */
    IIS_DMA_IRQn              = 19 + 18,     /*!<            */
    ALC_IRQn                  = 19 + 19,     /*!<            */
    PDM_IRQn                  = 19 + 20,     /*!<            */
    DTR_IRQn                  = 19 + 21,     /*!<            */
    V11_OK_IRQn               = 19 + 22,     /*!<            */
    VDT_IRQn                  = 19 + 23,     /*!<            */
    EXT0_IRQn                 = 19 + 24,     /*!<            */
    EXT1_IRQn                 = 19 + 25,     /*!<            */
    IWDG_IRQn                 = 19 + 26,     /*!<            */
    AON_TIM_INT0_IRQn         = 19 + 27,     /*!<            */
    AON_TIM_INT1_IRQn         = 19 + 28,     /*!<            */
    AON_EFUSE_IRQn            = 19 + 29,     /*!<            */
    AON_PC_IRQn               = 19 + 30,     /*!<            */
    MAILBOX_IRQn              = 19 + 31,     /*!<            */
/********************************* END ****************************************/
} IRQn_Type;

#define IRQn_MAX_NUMBER (51)


typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    EXT0 = 0,
    EXT1 = 1,
}Ext_Num;


#ifndef NULL
#define NULL 0
#endif


/*SCU寄存器定义*/

typedef struct
{
    volatile unsigned int SYS_CTRL_CFG;         /*!< offest:0x00 功能: */
    volatile uint32_t REV_SYR_CFG_0[(0x0C-0x00)/4-0x1];       
    volatile unsigned int EXT_INT_CFG;          /*!< offest:0x0C 功能: */
    volatile uint32_t REV_SYR_CFG_1[(0x50-0x0C)/4-0x1];       
    volatile unsigned int SYSCFG_LOCK_CFG;      /*!< offest:0x50 功能: */
    volatile unsigned int RSTCFG_LOCK_CFG;      /*!< offest:0x54 功能: */
    volatile unsigned int CKCFG_LOCK_CFG;       /*!< offest:0x58 功能: */
    volatile uint32_t REV_SYR_CFG_2[(0x80-0x58)/4-0x1];       
    volatile unsigned int CLK_DIV_PARAM0_CFG;   /*!< offest:0x80 功能: */
    volatile unsigned int CLK_DIV_PARAM1_CFG;   /*!< offest:0x84 功能: */
    volatile unsigned int CLK_DIV_PARAM2_CFG;   /*!< offest:0x88 功能: */
    volatile uint32_t REV_SYR_CFG_3[(0xB0-0x88)/4-0x1];       
    volatile unsigned int CLK_DIV_PARAM_EN_CFG; /*!< offest:0xB0 功能: */
    volatile uint32_t REV_SYR_CFG_4[(0xC0-0xB0)/4-0x1];       
    volatile unsigned int SRC0_MCLK_CFG;        /*!< offest:0xC0 功能: */
    volatile unsigned int SRC1_MCLK_CFG;        /*!< offest:0xC4 功能: */
    volatile unsigned int SRC2_MCLK_CFG;        /*!< offest:0xC8 功能: */
    volatile uint32_t REV_SYR_CFG_5[(0xD0-0xC8)/4-0x1];       
    volatile unsigned int MCLK0_CFG;            /*!< offest:0xD0 功能: */
    volatile unsigned int MCLK1_CFG;            /*!< offest:0xD4 功能: */
    volatile unsigned int MCLK2_CFG;            /*!< offest:0xD8 功能: */
    volatile uint32_t REV_SYR_CFG_6[(0xE0-0xD8)/4-0x1];       
    volatile unsigned int IIS0_CLK_SEL_CFG;     /*!< offest:0xE0 功能: */
    volatile unsigned int IIS1_CLK_SEL_CFG;     /*!< offest:0xE4 功能: */
    volatile unsigned int IIS2_CLK_SEL_CFG;     /*!< offest:0xE8 功能: */
    volatile uint32_t REV_SYR_CFG_7[(0xF0-0xE8)/4-0x1];       
    volatile unsigned int PAD_CLK_SEL_CFG;      /*!< offest:0xF0 功能: */
    volatile unsigned int CODEC_CLK_SEL_CFG;    /*!< offest:0xF4 功能: */
    volatile unsigned int PDM_CLK_SEL_CFG;      /*!< offest:0xF8 功能: */
    volatile uint32_t REV_SYR_CFG_8[(0x11C-0xF8)/4-0x1];       
    volatile unsigned int SYS_CLKGATE_CFG0;     /*!< offest:0x11C 功能: */
    volatile unsigned int SYS_CLKGATE_CFG1;     /*!< offest:0x120 功能: */
    volatile unsigned int AHB_CLKGATE_CFG;      /*!< offest:0x124 功能: */
    volatile unsigned int APB0_CLKGATE_CFG;     /*!< offest:0x128 功能: */
    volatile unsigned int APB1_CLKGATE_CFG;     /*!< offest:0x12C 功能: */
    volatile uint32_t REV_SYR_CFG_9[(0x178-0x12C)/4-0x1];       
    volatile unsigned int SCU_STATE_REG;        /*!< offest:0x178 功能: */
    volatile uint32_t REV_SYR_CFG_10[(0x190-0x178)/4-0x1];       
    volatile unsigned int AHB_RESET_CFG;        /*!< offest:0x190 功能: */
    volatile unsigned int APB0_RERST_CFG;       /*!< offest:0x194 功能: */
    volatile unsigned int APB1_RERST_CFG;       /*!< offest:0x198 功能: */
    volatile uint32_t REV_SYR_CFG_11[(0x1DC-0x198)/4-0x1];
    volatile unsigned int WAKEUP_MASK_CFG0;     /*!< offest:0x1DC 功能: */         
    volatile unsigned int WAKEUP_MASK_CFG1;     /*!< offest:0x1E0 功能: */
    volatile unsigned int EXT0_FILTER_CFG;      /*!< offest:0x1E4 功能: */
    volatile unsigned int EXT1_FILTER_CFG;      /*!< offest:0x1E8 功能: */
    volatile uint32_t REV_SYR_CFG_12[(0x1F4-0x1E8)/4-0x1];       
    volatile unsigned int INT_STATE_REG0;       /*!< offest:0x1F4 功能: */
    volatile unsigned int INT_STATE_REG1;       /*!< offest:0x1F8 功能: */
    volatile uint32_t REV_SYR_CFG_13[(0x210-0x1F8)/4-0x1];       
    volatile unsigned int RC_TRIM_CFG;          /*!< offest:0x210 功能: */
    volatile unsigned int RC_TRIM_STATE;        /*!< offest:0x214 功能: */
    volatile uint32_t REV_SYR_CFG_14[(0x240-0x214)/4-0x1];       
    volatile unsigned int MEM0_EMA_CFG;         /*!< offest:0x240 功能: */
    volatile unsigned int MEM1_EMA_CFG;         /*!< offest:0x244 功能: */
    volatile uint32_t REV_SYR_CFG_15[(0x264-0x244)/4-0x1];       
    volatile unsigned int IIS_DATA_SEL_CFG;     /*!< offest:0x264 功能: */
    volatile uint32_t REV_SYR_CFG_16[(0x290-0x264)/4-0x1];       
    volatile unsigned int PAD_STATE;            /*!< offest:0x290 功能: */
    volatile uint32_t REV_SYR_CFG_17[(0x2C0-0x290)/4-0x1];
    volatile unsigned int BOOT_ADDR_CFG;        /*!< offest:0x2C0 功能: */
    volatile uint32_t REV_SYR_CFG_18[(0x2D0-0x2C0)/4-0x1];
    volatile unsigned int DMAINT_SEL_CFG;       /*!< offest:0x2D0 功能: */
}SCU_TypeDef;


/**
 * @brief DPMU寄存器结构体
 */
typedef struct
{
    volatile unsigned int CFG_LOCK_CFG;         /*!< offest:0x00 功能: */
    volatile uint32_t REV_SYR_CFG_0[(0x10-0x00)/4-0x1];       
    volatile unsigned int SYS_RESET_CFG;        /*!< offest:0x10 功能: */
    volatile unsigned int SYS_SOFTRST_CFG;      /*!< offest:0x14 功能: */
    volatile uint32_t REV_SYR_CFG_1[(0x20-0x14)/4-0x1];       
    volatile unsigned int SYS_CLK_SEL_CFG;      /*!< offest:0x20 功能: */
    volatile uint32_t REV_SYR_CFG_2[(0x30-0x20)/4-0x1];       
    volatile unsigned int PLL_CFG;              /*!< offest:0x30 功能: */
    volatile unsigned int AON_CLK_PARAM_CFG;    /*!< offest:0x34 功能: */
    volatile uint32_t REV_SYR_CFG_3[(0x40-0x34)/4-0x1];       
    volatile unsigned int AON_CLK_PARAM_EN_CFG; /*!< offest:0x40 功能: */
    volatile uint32_t REV_SYR_CFG_4[(0x50-0x40)/4-0x1];       
    volatile unsigned int AON_CLKGATE_CFG;      /*!< offest:0x50 功能: */
    volatile uint32_t REV_SYR_CFG_5[(0x70-0x50)/4-0x1];       
    volatile unsigned int AON_RESET_CFG;        /*!< offest:0x70 功能: */
    volatile uint32_t REV_SYR_CFG_6[(0xC0-0x70)/4-0x1];       
    volatile unsigned int PMU_CFG;              /*!< offest:0xC0 功能: */
    volatile uint32_t REV_SYR_CFG_7[(0xC8-0xC0)/4-0x1];       
    volatile unsigned int PMU_UPDATE_EN;        /*!< offest:0xC8 功能: */
    volatile uint32_t REV_SYR_CFG_8[(0xD0-0xC8)/4-0x1];       
    volatile unsigned int LOW_POWER_CFG;        /*!< offest:0xD0 功能: */
    volatile unsigned int PMU_PWROFF_CFG;       /*!< offest:0xD4 功能: */
    volatile unsigned int PMU_PWRON_CFG;        /*!< offest:0xD8 功能: */
    volatile uint32_t REV_SYR_CFG_9[(0xE0-0xD8)/4-0x1];       
    volatile unsigned int WAKEUP_RESET_CFG;     /*!< offest:0xE0 功能: */
    volatile unsigned int WAKEUP_MASK_CFG;      /*!< offest:0xE4 功能: */
    volatile unsigned int WAKEUP_EXT_FILTER_CFG;/*!< offest:0xE8 功能: */
    volatile unsigned int WAKEUP_CFG;           /*!< offest:0xEC 功能: */
    volatile uint32_t REV_SYR_CFG_10[(0x100-0xEC)/4-0x1];       
    volatile unsigned int RC_CFG;               /*!< offest:0x100 功能: */
    volatile unsigned int RC_UPDATE_CFG;        /*!< offest:0x104 功能: */
    volatile unsigned int OSC_PAD_CFG;          /*!< offest:0x108 功能: */
    volatile uint32_t REV_SYR_CFG_11[(0x140-0x108)/4-0x1];
    volatile unsigned int IOREUSE_CFG0;         /*!< offest:0x140 功能: */
    volatile unsigned int IOREUSE_CFG1;         /*!< offest:0x144 功能: */
    volatile uint32_t REV_SYR_CFG_12[(0x14c-0x144)/4-0x1];
    volatile unsigned int OD_CFG0;              /*!< offest:0x14c 功能: */
    volatile unsigned int PD_CFG0;              /*!< offest:0x150 功能: */
    volatile uint32_t REV_SYR_CFG_13[(0x158-0x150)/4-0x1];       
    volatile unsigned int PU_CFG0;              /*!< offest:0x158 功能: */
    volatile uint32_t REV_SYR_CFG_14[(0x160-0x158)/4-0x1];       
    volatile unsigned int DS_CFG0;              /*!< offest:0x160 功能: */
    volatile unsigned int DS_CFG1;              /*!< offest:0x164 功能: */
    volatile uint32_t REV_SYR_CFG_15[(0x170-0x164)/4-0x1];       
    volatile unsigned int SL_CFG0;              /*!< offest:0x170 功能: */
    volatile uint32_t REV_SYR_CFG_16[(0x178-0x170)/4-0x1];       
    volatile unsigned int ST_CFG0;              /*!< offest:0x178 功能: */
    volatile uint32_t REV_SYR_CFG_17[(0x180-0x178)/4-0x1];       
    volatile unsigned int IE_CFG0;              /*!< offest:0x180 功能: */
    volatile uint32_t REV_SYR_CFG_18[(0x190-0x180)/4-0x1];       
    volatile unsigned int AD_CFG0;              /*!< offest:0x190 功能: */
    volatile unsigned int OD_CFG1;              /*!< offest:0x194 功能: */
    volatile unsigned int PD_CFG1;              /*!< offest:0x198 功能: */
    volatile unsigned int PU_CFG1;              /*!< offest:0x19c 功能: */
    volatile unsigned int DS_CFG2;              /*!< offest:0x1A0 功能: */
    volatile unsigned int SL_CFG1;              /*!< offest:0x1A4 功能: */
    volatile unsigned int ST_CFG1;              /*!< offest:0x1A8 功能: */
    volatile unsigned int IE_CFG1;              /*!< offest:0x1AC 功能: */
    volatile uint32_t REV_SYR_CFG_19[(0x1C0-0x1AC)/4-0x1];       
    volatile unsigned int RST_STATE_REG;        /*!< offest:0x1C0 功能: */
    volatile uint32_t REV_SYR_CFG_20[(0x1D0-0x1C0)/4-0x1];       
    volatile unsigned int PWR_WAKEUP_STATE_REG; /*!< offest:0x1D0 功能: */
    volatile uint32_t REV_SYR_CFG_21[(0x1E0-0x1D0)/4-0x1];
    volatile unsigned int CHIP_STATE_REG_ADDR;  /*!< offest:0x1E0 功能: */
    volatile unsigned int CHIP_INT_MASK_CFG_ADDR; /*!< offest:0x1E4 功能: */
    volatile unsigned int PAD_FILTER_CFG_ADDR;  /*!< offest:0x1E8 功能: */
}DPMU_TypeDef;


/*UART寄存器定义*/
typedef struct
{
    volatile unsigned int UARTRdDR;         //0x0
    volatile unsigned int UARTWrDR;         //0x04
    volatile unsigned int UARTRxErrStat;    //0x08
    volatile unsigned int UARTFlag;         //0x0c
    volatile unsigned int UARTIBrd;         //0x10
    volatile unsigned int UARTFBrd;         //0x14
    volatile unsigned int UARTLCR;          //0x18
    volatile unsigned int UARTCR;           //0x1c
    volatile unsigned int UARTFIFOlevel;    //0x20
    volatile unsigned int UARTMaskInt;      //0x24
    volatile unsigned int UARTRIS;          //0x28
    volatile unsigned int UARTMIS;          //0x2c
    volatile unsigned int UARTICR;          //0x30
    volatile unsigned int UARTDMACR;        //0x34
    volatile unsigned int UARTTimeOut;      //0x38
    volatile unsigned int REMCR;            //0x3c
    volatile unsigned int REMTXDATA;        //0x40
    volatile unsigned int REMRXDATA;        //0x44
    volatile unsigned int REMINTCLR;        //0x48
    volatile unsigned int REMINTSTAE;       //0x4c
    volatile unsigned int BYTE_HW_MODE;     //0x50
}UART_TypeDef;


typedef struct
{
	volatile unsigned int DMACCxSrcAddr;
	volatile unsigned int DMACCxDestAddr;
	volatile unsigned int DMACCxLLI;
	volatile unsigned int DMACCxControl;
	volatile unsigned int DMACCxConfiguration;
	unsigned int reserved[3];
}DMACChanx_TypeDef;


typedef struct
{
	volatile unsigned int DMACIntStatus;
	volatile unsigned int DMACIntTCStatus;
	volatile unsigned int DMACIntTCClear;
	volatile unsigned int DMACIntErrorStatus;
	volatile unsigned int DMACIntErrClr;
	volatile unsigned int DMACRawIntTCStatus;
	volatile unsigned int DMACRawIntErrorStatus;
	volatile unsigned int DMACEnbldChns;
	volatile unsigned int DMACSoftBReq;
	volatile unsigned int DMACSoftSReq;
	volatile unsigned int DMACSoftLBReq;
	volatile unsigned int DMACSoftLSReq;
	volatile unsigned int DMACConfiguration;
	volatile unsigned int DMACSync;
	unsigned int reserved1[50];
	DMACChanx_TypeDef DMACChannel[8];
	unsigned int reserved2[195];
	volatile unsigned int DMACITCR;
	volatile unsigned int DMACITOP[3];
	unsigned int reserved3[693];
	volatile unsigned int DMACPeriphID[4];
	volatile unsigned int DMACPCellID[4];
}DMA_TypeDef;


typedef struct
{
	unsigned int SrcAddr;
	unsigned int DestAddr;
	unsigned int NextLLI;
	unsigned int Control;
}DMAC_LLI;


typedef struct
{
    volatile unsigned int IISxDMARADDR;         //4     //1C    //34
    volatile unsigned int IISxDMARNUM;          //8     //20    //38
    volatile unsigned int IISxDMATADDR0;        //C     //24    //3c
    volatile unsigned int IISxDMATNUM0;         //10    //28    //40
    volatile unsigned int IISxDMATADDR1;        //14    //2C    //44
    volatile unsigned int IISxDMATNUM1;         //18    //30    //48
}IISDMAChanx_TypeDef;


typedef struct
{
    volatile unsigned int IISDMACTRL;           //0
    IISDMAChanx_TypeDef   IISxDMA[3];
    volatile unsigned int IISDMAPTR;            //4c
    volatile unsigned int IISDMASTATE;          //50
    volatile unsigned int IISDMACLR;            //54
    volatile unsigned int IISDMAIISCLR;         //58
    volatile unsigned int IISDMARADDR[3];       //5C,60,64
    volatile unsigned int RX_VAD_CTRL;          //68
	volatile unsigned int RX_LAST_ADDR;         //6C
	volatile unsigned int IIS_END_NUM_EN;       //70
	volatile unsigned int IIS_END_NUM;          //74
	volatile unsigned int DMA_REQ_CLR_STATE;    //78
	volatile unsigned int DMATADDR[3];          //7C,80,84
}IISDMA_TypeDef;


/*CODEC寄存器*/
//以2代的CODEC为基础，修改一些

typedef struct 
{
    volatile unsigned int reg40;
    volatile unsigned int reg41;
    volatile unsigned int reg42;
    volatile unsigned int reg43;
    volatile unsigned int reg44;
    volatile unsigned int reg45;
    volatile unsigned int reg46;
    volatile unsigned int reg47;
    volatile unsigned int reg48;
    volatile unsigned int reg49;
    unsigned int resver7[2];
    volatile unsigned int reg4c;
    unsigned int resver8[3];
}CODEC_ALC_TypeDef;

typedef struct 
{
    volatile unsigned int reg0;
    unsigned int resver1;
    volatile unsigned int reg2;
    volatile unsigned int reg3;
    volatile unsigned int reg4;
    volatile unsigned int reg5;
    volatile unsigned int reg6;
    volatile unsigned int reg7;
    volatile unsigned int adc_dig_gain_reg[2];
    volatile unsigned int rega;
    unsigned int resver4[22];
    volatile unsigned int reg21;
    volatile unsigned int reg22;
    volatile unsigned int reg23;
    volatile unsigned int reg24;
    volatile unsigned int reg25;
    volatile unsigned int reg26;
    volatile unsigned int pga_gain_reg[2];
    volatile unsigned int reg29;
    volatile unsigned int reg2a;
    volatile unsigned int reg2b;
    volatile unsigned int reg2c;
    volatile unsigned int reg2d;
    volatile unsigned int reg2e;
    volatile unsigned int reg2f;
    volatile unsigned int reg30;
    unsigned int resver6[15];

    CODEC_ALC_TypeDef alc_reg[2];
}CODEC_TypeDef;
/*CODEC寄存器end*/


/*PDM寄存器*/
typedef struct 
{
    volatile unsigned int reg0;
    unsigned int resver1;
    volatile unsigned int reg2;
    volatile unsigned int reg3;
    volatile unsigned int reg4;
    volatile unsigned int reg5;
    unsigned int resver2;
    volatile unsigned int reg7;
    volatile unsigned int pdm_dig_gain[2];
    volatile unsigned int rega;
    unsigned int resver4[22];
    volatile unsigned int reg21;
    volatile unsigned int reg22;
    volatile unsigned int reg23;
    volatile unsigned int reg24;
    volatile unsigned int reg25;
    volatile unsigned int reg26;
    volatile unsigned int reg27;
    volatile unsigned int reg28;
    volatile unsigned int reg29;
    volatile unsigned int reg2a;
    volatile unsigned int reg2b;
    volatile unsigned int reg2c;
    volatile unsigned int reg2d;
    volatile unsigned int reg2e;
    volatile unsigned int reg2f;
    volatile unsigned int reg30;
    unsigned int resver6[15];
    CODEC_ALC_TypeDef alc_reg[2];
}PDM_TypeDef;
/*PDM寄存器end*/


typedef struct
{
    volatile uint32_t ALC_INT_STATUS;   /*0x00*/
    volatile uint32_t ALC_INT_EN;       /*0x04*/
    volatile uint32_t ALC_INT_CLR;      /*0x08*/
    volatile uint32_t GLB_CTRL;         /*0x0c*/
    volatile uint32_t LEFT_CTRL;        /*0x10*/
    volatile uint32_t L_STEP_TMR_ATK;    /*0x14*/
    volatile uint32_t L_STEP_TMR_DCY;    /*0x18*/
    volatile uint32_t L_CFG_ADDR;       /*0x1c*/
    volatile uint32_t L_MIN_MAX_DB;     /*0x20*/
    volatile uint32_t L_DB_STATUS;      /*0x24*/
    volatile uint32_t L_ATK_DCY_STATUS; /*0x28*/
    uint32_t reserve;
    volatile uint32_t RIGHT_CTRL;        /*0x30*/
    volatile uint32_t R_STEP_TMR_ATK;    /*0x34*/
    volatile uint32_t R_STEP_TMR_DCY;    /*0x38*/
    volatile uint32_t R_CFG_ADDR;       /*0x3c*/
    volatile uint32_t R_MIN_MAX_DB;     /*0x40*/
    volatile uint32_t R_DB_STATUS;      /*0x44*/
    volatile uint32_t R_ATK_DCY_STATUS; /*0x48*/

    volatile uint32_t PAUSE_STATUS;     /*0x4c*/
}ALC_TypeDef;

#define HAL_NPU_BASE    (0xECC644)

/* APB peripherals*/
#define IPCORE_BASE             (0xa6a6a6a6)
#define APB_BASE                (0x5a5a5a5a)
#define SYSTICK_BASE            (0x77777777)
#define PLL_BASE                (0x34343434)
#define HAL_DTRFLASH_RAM_BASE   (0x45454545) 


#define SCU_BASE_INDEX		0
#define HAL_SCU_BASE        (0x40000000)
#define HAL_GDMA_BASE       (0x40001000)
#define HAL_ADC_BASE        (0x40002000)
#define HAL_IISDMA0_BASE    (0x40003000)
#define HAL_DTRFLASH_BASE   (0x40004000)
#define HAL_ALC_BASE        (0x40005000)


#define HAL_TWDG_BASE       (0x40010000) //WWDG
#define HAL_IIC0_BASE       (0x40011000)
#define HAL_PDM_BASE        (0x40012000)
#define HAL_ALC_PDM_BASE    (0x40012200)
#define HAL_CODEC_BASE      (0x40013000)
#define HAL_PWM0_BASE       (0x40014000)
#define HAL_PWM1_BASE       (0x40015000)
#define HAL_PWM2_BASE       (0x40016000)
#define HAL_PWM3_BASE       (0x40017000)
#define HAL_TIMER0_BASE     (0x40018000)
#define HAL_TIMER1_BASE     (0x40019000)
#define HAL_TIMER2_BASE     (0x4001a000)
#define HAL_TIMER3_BASE     (0x4001b000)
//以下4个为非真实的地址
#define CODEC_AD_GATE       (0x4001c000)
#define CODEC_DA_GATE       (0x4001d000)
#define WWDG_CPU0_HALT_GATE (0x4001e000)
#define WWDG_CPU1_HALT_GATE (0x4001f000)


#define HAL_PA_BASE         (0x40020000)
#define HAL_PB_BASE         (0x40021000)
#define HAL_UART0_BASE      (0x40022000)
#define HAL_UART1_BASE      (0x40023000)
#define HAL_UART2_BASE      (0x40024000)
#define HAL_IIS0_BASE       (0x40025000)//(IO)
#define HAL_IIS1_BASE       (0x40026000)//(CODEC)
#define HAL_IIS2_BASE       (0x40027000)//(PDM)
#define HAL_PD_BASE         (0x40028000)
//以下2个为非真实的地址
#define IIS1_RX_GATE        (0x40029000)
#define IIS1_TX_GATE        (0x4002A000)


#define HAL_DPMU_BASE       (0x40030000)
#define HAL_PC_BASE         (0x40031000)
#define HAL_IWDG_BASE       (0x40032000)
#define HAL_EFUSE_BASE      (0x40033000)
#define HAL_PWM4_BASE       (0x40034000)//复用TIMER0
#define HAL_PWM5_BASE       (0x40035000)//复用TIMER1
//以下2个为非真实的地址
#define IWDG_CPU0_HALT_GATE (0x40036000)
#define IWDG_CPU1_HALT_GATE (0x40037000)
#define PLL_OUT_GATE        (0x40038000)


#define HAL_MAILBOX0_BASE   (0x40006000)
#define HAL_MAILBOX1_BASE   (0x30010000)


#define SPI0FIFO_BASE       (0x60000000)
#define UART0FIFO_BASE      (0x61000000)
#define UART1FIFO_BASE      (0x62000000)
#define UART2FIFO_BASE      (0x63000000)


#define SCU                 ((SCU_TypeDef*)HAL_SCU_BASE)
#define DPMU                ((DPMU_TypeDef*)HAL_DPMU_BASE)
#define ADC                 ((ADC_TypeDef*)HAL_ADC_BASE)
#define UART0               ((UART_TypeDef*)HAL_UART0_BASE)
#define UART1               ((UART_TypeDef*)HAL_UART1_BASE)
#define UART2               ((UART_TypeDef*)HAL_UART2_BASE)
#define	 TWDG	            ((TWDG_TypeDef *)HAL_TWDG_BASE)
#define PDM                 ((PDM_TypeDef*)HAL_PDM_BASE)
#define CODEC               ((CODEC_TypeDef*)HAL_CODEC_BASE)
#define ALC                 ((ALC_TypeDef*)HAL_ALC_BASE)
#define ALC_PDM             ((ALC_TypeDef*)HAL_ALC_PDM_BASE)
#define DMAC                ((DMA_TypeDef*)HAL_GDMA_BASE)
#define IISDMA0             ((IISDMA_TypeDef*)HAL_IISDMA0_BASE)


/**
  * @}
  */


/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

void _delay_10us_240M(uint32_t cnt);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

/************************ (C) COPYRIGHT chipintelli *****END OF FILE****/











