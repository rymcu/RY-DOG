/**
 * @file ci130x_iic.c
 * @brief CI130X芯片IIC模块的驱动程序
 * @version 0.1
 * @date 2021-10-28
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */
#include <stdint.h>
#include "ci130x_iic.h"
#include "ci_log.h"
#include "ci130x_scu.h"
#include "platform_config.h"

#define IIC_DIRVER_DBG           1  

#define WAKE_UP                  0xacced
#define SLEEP                    0x0

#define RXDUFLIE                 (0x1 << 1)
#define TXDEPTIE                 (0x1 << 2)
#define SSTOPIE                  (0x1 << 4)
#define SLVADIE                  (0x1 << 6)

#define IIC_TRANSMIT             0x1
#define IIC_RECEIVE              0x0
#define IIC_TRANS_OVER           0x1
#define IIC_TRANS_ON             0x0


#define CR_STOP                  (0x1 << 3)
#define CR_START                 (0x1 << 4)
#define CR_IRQ_CLEAR             (0x7f)
#define SR_BUSY                  (0x1 << 15)
#define SR_ABITOR_LOST           (0x1 << 5)
#define SR_TRANS_ERR             (0x1 << 3)
#define SR_RXACK                 (0x1 << 14)
#define SR_IRQ_FLAG              (0x1 << 12)
#define CR_NACK                  (0x1 << 2)
#define CR_TB                    (0x1 << 0)
#define CR_WRITE                  0
#define CR_READ                   0
/**
 * @brief IIC寄存器结构体
 */
typedef struct
{
    volatile unsigned int  SCLDIV;             /*!< offest:0x00 功能:SCL分频参数寄存 */
    volatile unsigned int  SRHLD;              /*!< offest:0x04 功能:Start条件hold time */
    volatile unsigned int  DTHLD;              /*!< offest:0x08 功能:SDA Data time */
    volatile unsigned int  GLBCTRL;            /*!< offest:0x0c 功能:全局控制寄存器 */
    volatile unsigned int  CMD;                /*!< offest:0x10 功能:命令寄存器 */
    volatile unsigned int  INTEN;              /*!< offest:0x14 功能:中断使能控制寄存器 */
    volatile unsigned int  INTCLR;             /*!< offest:0x18 功能:中断清除寄存器 */
    volatile unsigned int  SLVADR;             /*!< offest:0x1c 功能:Slave地址寄存器 */
    volatile unsigned int  TXDR;               /*!< offest:0x20 功能:发送数据寄存器 */
    volatile unsigned int  RXDR;               /*!< offest:0x24 功能:接收数据寄存器 */
    volatile unsigned int  TIMEOUT;            /*!< offest:0x28 功能:超时寄存器 */
    volatile unsigned int  STATUS;             /*!< offest:0x2c 功能:状态寄存器 */
    volatile unsigned int  BUSMON;             /*!< offest:0x30 功能:总线信号检测寄存器 */
    volatile unsigned int  INTSTAT;            /*!< offest:0x34 功能: */
}iic_register_t;

typedef struct
{
    volatile uint32_t is_master;      /* 0为SLAVE 1为MASTER */
    volatile uint32_t is_slave_recv;      /* -1为未被寻址到 0为send 1为recv */
    volatile int32_t  timeout;
    volatile int32_t  wait;
    volatile uint32_t errors;
    volatile uint32_t over;
    master_send_cb_t master_send_cb;
    master_recv_cb_t master_recv_cb;
    slave_send_cb_t slave_send_cb;
    slave_recv_cb_t slave_recv_cb;
} iic_status_t;

static iic_status_t iic0_status = {0};

static void delay_time(int32_t time)
{
    volatile int32_t loop;
    while(time--)
    {
        loop = 100;
        while(loop--);
    }
}

/**
 * @brief 阻塞查询数据接收/发送
 * 
 * @param base iic号
 * @return int32_t 是否等到flag或超时
 */
static int32_t iic_wait(iic_base_t base)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }
    

    int32_t ret = 0;

    while(((iic_p->STATUS) & SR_IRQ_FLAG) == 0)
    {
        //400K时，这里必须加一行代码
        delay_time(1);
    }

    iic_s->errors = iic_p->STATUS;
    if(iic_s->over == IIC_TRANS_OVER)
    {
        iic_s->errors &= ~SR_RXACK;
        iic_s->over = IIC_TRANS_ON;
    }  
    
    iic_s->errors &= (~SR_IRQ_FLAG);
    return ret;
}

/**
 * @brief 复位IIC总线
 * 
 * @param base iic号
 */
static void iic_reset(iic_base_t base)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_p->CMD = CR_STOP | CR_TB;     //产生stop条件，并命令有效
}


/**
 * @brief IIC总线发送start位
 * 
 * @param base iic号
 */
static void iic_start(iic_base_t base)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    
    #if IIC_DIRVER_DBG
    ci_loginfo(LOG_IIC_DRIVER,"=== START\n");
    #endif

    iic_p->INTCLR = (0x1 << 4);
    iic_p->CMD = CR_START | CR_TB;

    iic_wait(base);    
}

/**
 * @brief iic总线发送stop位
 * 
 * @param base iic号
 */
static void iic_stop(iic_base_t base)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }
    
    #if IIC_DIRVER_DBG
    ci_loginfo(LOG_IIC_DRIVER,"=== STOP\n");
    #endif

    iic_s->over = IIC_TRANS_OVER;
    iic_p->CMD = CR_STOP | CR_TB;
    iic_p->INTCLR = CR_IRQ_CLEAR;

    iic_wait(base);
}

/**
 * @brief iic发送寻址
 * 
 * @param base iic号
 * @param addr 7位地址
 * @param flags 读写标志
 */
static void iic_address(iic_base_t base, uint16_t addr, uint16_t flags)
{
    iic_register_t* iic_p = (iic_register_t*)base;

    uint8_t dst_addr = (uint8_t) ( (0x7f & addr) << 1 );
    if (flags & IIC_M_READ )
    {
        dst_addr |= 1;
    }

    iic_p->TXDR = dst_addr;
}

/**
 * @brief 数据阻塞发送
 * 
 * @param base iic号
 * @param lastbyte 是否是end数据
 * @param receive 读写方向
 * @param midbyte 中间数据
 */
static void iic_transfer_transmit(iic_base_t base, int32_t lastbyte, int32_t receive, int32_t midbyte)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    if(lastbyte)
    {
        iic_s->over = IIC_TRANS_OVER;
        if(receive == IIC_RECEIVE)
        {
            //iic_p->CMD = CR_READ | CR_IRQ_CLEAR | CR_NACK | CR_STOP ;
            iic_p->CMD = CR_NACK | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        else
        {
            //iic_p->CMD = CR_WRITE | CR_IRQ_CLEAR  | CR_NACK | CR_STOP ;
            iic_p->CMD = CR_WRITE | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        #if IIC_DIRVER_DBG
        ci_loginfo(LOG_IIC_DRIVER,"=== LASTBYTE\n");
        #endif
    }
    else if(midbyte)
    {
        if(receive == IIC_RECEIVE)
        {
            //iic_p->CMD = CR_READ | CR_IRQ_CLEAR  ;
            iic_p->CMD = CR_READ | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        else
        {
            //iic_p->CMD = CR_WRITE | CR_IRQ_CLEAR | CR_NACK ;
            iic_p->CMD = CR_WRITE | CR_NACK | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        #if IIC_DIRVER_DBG
        ci_loginfo(LOG_IIC_DRIVER,"=== MIDBYTE\n");
        #endif
    }

    iic_wait(base);
}

static uint8_t iic_transfer_receive(iic_base_t base, int32_t lastbyte, int32_t receive, int32_t midbyte)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    if(lastbyte)
    {
        iic_s->over = IIC_TRANS_OVER;
        if(receive == IIC_RECEIVE)
        {
            //iic_p->CMD = CR_READ | CR_IRQ_CLEAR | CR_NACK | CR_STOP ;
            iic_p->CMD = CR_NACK | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        else
        {
            //iic_p->CMD = CR_WRITE | CR_IRQ_CLEAR  | CR_NACK | CR_STOP ;
            iic_p->CMD = CR_WRITE | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        #if IIC_DIRVER_DBG
        ci_loginfo(LOG_IIC_DRIVER,"=== LASTBYTE\n");
        #endif
    }
    else if(midbyte)
    {
        if(receive == IIC_RECEIVE)
        {
            //iic_p->CMD = CR_READ | CR_IRQ_CLEAR  ;
            iic_p->CMD = CR_READ | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        else
        {
            //iic_p->CMD = CR_WRITE | CR_IRQ_CLEAR | CR_NACK ;
            iic_p->CMD = CR_WRITE | CR_NACK | CR_TB;
            iic_p->INTCLR = CR_IRQ_CLEAR;
        }
        #if IIC_DIRVER_DBG
        ci_loginfo(LOG_IIC_DRIVER,"=== MIDBYTE\n");
        #endif
    }
    
    iic_wait(base);
  
  	uint8_t ch = iic_p->RXDR;

    return ch;
}

/**
 * @brief iic阻塞发送数据
 * 
 * @param base iic号
 * @param buf 数据buff
 * @param count 数据格式
 * @param last end标志
 * @param last_ack_flag 最后一笔数据ack标志
 * @return int32_t 成功发送个数
 */
static int32_t iic_sendbytes(iic_base_t base, const int8_t *buf, int32_t count, int32_t last, uint8_t *last_ack_flag)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    if(last_ack_flag != NULL)
    {
        *last_ack_flag = 0;
    }
    
    int32_t wrcount = 0;
    for (wrcount=0; wrcount<count; )
    {
        iic_p->TXDR = buf[wrcount];
        if ((wrcount==(count-1)) && last)
        {
            iic_transfer_transmit( base, last, IIC_TRANSMIT, 0);
            iic_stop(base);
        }
        else
        {
            iic_transfer_transmit( base, 0, IIC_TRANSMIT, 1);
        }
      
        ++wrcount;
        if(iic_s->errors & (SR_ABITOR_LOST|SR_TRANS_ERR|SR_RXACK))
        {
            if(iic_s->errors & SR_RXACK)
            {   
                #if IIC_DIRVER_DBG
                ci_loginfo(LOG_IIC_DRIVER,"=== NOT ACK received after SLA+R\n");
                #endif
                if(last_ack_flag != NULL)
                {
                    *last_ack_flag = 1; //NACK
                }
            }
            iic_stop(base);
            break;
        }
    }
    
    return (wrcount);
}

/**
 * @brief iic读取数据
 * 
 * @param base iic号
 * @param buf 数据buff
 * @param count 数据个数
 * @param last end标志
 * @return uint32_t 读取到的数据个数
 */
static uint32_t iic_readbytes(iic_base_t base, int8_t *buf, int32_t count, int32_t last)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    int32_t i;

    /* increment number of bytes to read by one -- read dummy byte */
    for (i = 0; i <= count; i++)
    {
        if (i!=0)
        {
            /* set ACK to NAK for last received byte ICR[ACKNAK] = 1
            only if not a repeated start */

	        if ((i == count) && last) 
	        {
	            buf[i-1] = iic_transfer_receive( base, last, IIC_RECEIVE, 0);
	        }
	        else
	        {
	            buf[i-1] = iic_transfer_receive( base,0, IIC_RECEIVE, 1);
	        }
            #if IIC_DIRVER_DBG
            ci_loginfo(LOG_IIC_DRIVER,"rev---%d\n",buf[i-1]);
            #endif
        
            if(iic_s->errors & (SR_ABITOR_LOST|SR_TRANS_ERR|SR_RXACK))
            {            
                break;
            }
        }
        else
        {
            volatile int8_t temp = iic_p->RXDR; /*! dummy read */
        }
    }
    iic_stop(base);
    return (i - 1);
}

/**
 * @brief iic以查询方式初始化
 * 
 * @param base iic号
 * @param speed 时钟速度
 * @param slaveaddr 配置为0则为master模式，否则为slave模式，并使用这个只作为addr
 * @param timeout 超时时间
 */
void iic_polling_init(iic_base_t base, uint32_t speed, uint32_t slaveaddr, IIC_TimeOut timeout)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    IRQn_Type irq = IIC0_IRQn;

    if(IIC0 == base)
    {
        iic_s = &iic0_status;
        irq = IIC0_IRQn;
    }

    eclic_irq_disable(irq);
    scu_set_device_gate((uint32_t)base,ENABLE);
    scu_set_device_reset((uint32_t)base);
    scu_set_device_reset_release((uint32_t)base);

    iic_s->errors = 0;
    iic_s->over = 0;
    
    iic_s->timeout = timeout;
    iic_s->wait = SLEEP;
    iic_p->SLVADR = slaveaddr;
    iic_p->TIMEOUT = timeout;

    uint32_t clock = get_apb_clk();
    uint32_t scldiv_l = (clock / (speed *1000)) / 2 -15;
    uint32_t scldiv_h = (scldiv_l << 16);
    uint32_t scldiv = (scldiv_l | scldiv_h);

    iic_p->INTCLR = 0xffffffff;
    if(slaveaddr == 0)
    {
        iic_p->GLBCTRL = 0x00040080;
        iic_p->GLBCTRL |= 0x3;
        iic_s->is_master = 1;
    }
    else
    {
        iic_p->GLBCTRL = 0x00040084;
        iic_p->GLBCTRL |= 0x2;
        iic_s->is_master = 0;
    }
    iic_s->is_slave_recv = -1;
    iic_s->slave_recv_cb = NULL;
    iic_s->master_send_cb = NULL;
    iic_s->master_recv_cb = NULL;
    iic_s->slave_send_cb = NULL;
    iic_p->SCLDIV = scldiv;
    iic_p->SRHLD = scldiv;
    
    uint32_t tmp = scldiv_l / 2;
    tmp = (tmp << 16) | (scldiv_l / 2);
    if(tmp)
    {
        iic_p->DTHLD = tmp;
    }
    else
    {
        iic_p->DTHLD = 0x00040004;
    }
    
    iic_p->INTCLR = 0xff;
    iic_p->INTEN = (RXDUFLIE |RXDUFLIE | TXDEPTIE | SSTOPIE | SLVADIE | (0x1<<0));
    //iic_p->CMD = CR_STOP | CR_TB;
}

/**
 * @brief iic以中断方式初始化
 * 
 * @param base iic号
 * @param speed 时钟速度
 * @param slaveaddr 配置为0则为master模式，否则为slave模式，并使用这个只作为addr
 * @param timeout 超时时间
 */
void iic_interrupt_init(iic_base_t base, uint32_t speed, uint32_t slaveaddr, IIC_TimeOut timeout)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    IRQn_Type irq = IIC0_IRQn;

    if(IIC0 == base)
    {
        iic_s = &iic0_status;
        irq = IIC0_IRQn;
    }

    scu_set_device_gate((uint32_t)base,ENABLE);
    scu_set_device_reset((uint32_t)base);
    scu_set_device_reset_release((uint32_t)base);
   

    iic_s->errors = 0;
    iic_s->over = 0;
    
    iic_s->timeout = timeout;
    iic_s->wait = SLEEP;
    iic_p->SLVADR = slaveaddr;
    iic_p->TIMEOUT = timeout;

    uint32_t clock = get_apb_clk();
    uint32_t scldiv_l = (clock / (speed *1000)) / 2 -15;
    uint32_t scldiv_h = (scldiv_l << 16);
    uint32_t scldiv = (scldiv_l | scldiv_h);

    iic_p->INTCLR = 0xffffffff;
    if(slaveaddr == 0)
    {
        iic_p->GLBCTRL = 0x00040080;
        iic_p->GLBCTRL |= 0x3;
        iic_s->is_master = 1;
    }
    else
    {
        iic_p->GLBCTRL = 0x00040084;
        iic_p->GLBCTRL |= 0x2;
        iic_s->is_master = 0;
    }
    iic_s->is_slave_recv = -1;
    iic_s->slave_recv_cb = NULL;
    iic_s->master_send_cb = NULL;
    iic_s->master_recv_cb = NULL;
    iic_s->slave_send_cb = NULL;

    iic_p->SCLDIV = scldiv;
    iic_p->SRHLD = scldiv;
    
    uint32_t tmp = scldiv_l / 2;
    tmp = (tmp << 16) | (scldiv_l / 2);
    if(tmp)
    {
        iic_p->DTHLD = tmp;
    }
    else
    {
        iic_p->DTHLD = 0x00040004;
    }
    
    iic_p->INTCLR = 0x7f;
    iic_p->INTEN = (RXDUFLIE | TXDEPTIE | SSTOPIE | SLVADIE | (0x1<<0));
    eclic_clear_pending(irq);
    eclic_irq_enable(irq);
}

/**
 * @brief iic master 查询方式发送数据
 * 
 * @param base iic号
 * @param addr 目标slave地址
 * @param buf 待发送的数据buff
 * @param count 需要发送的数据个数
 * @param last_ack_flag 用来返回数据发送完毕后最好一次收到ack or nack
 * @return int32_t 成功发送的个数（这里最后一次收到nack会被记录）
 */
int32_t iic_master_polling_send(iic_base_t base, uint16_t addr, const char *buf, int32_t count, uint8_t *last_ack_flag)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    int32_t timeout = iic_s->timeout;

    while ((iic_p->STATUS& SR_BUSY) && timeout--)
    {
        delay_time(10);
    }
    if (iic_p->STATUS & SR_BUSY)
    {
        return -2;
    }

    iic_address(base,addr,IIC_M_WRITE);
    iic_start(base);
    if(iic_s->errors & SR_ABITOR_LOST)
    {
        iic_stop(base);
        return -2;   
    }
    if(iic_s->errors & SR_TRANS_ERR)
    {
        iic_stop(base);
        return -2;  
    }

    if(iic_s->errors & SR_RXACK)
    {
        iic_stop(base);
        return -2;  
    }

    int32_t retval = iic_sendbytes(base, (int8_t *)buf, count, 1, last_ack_flag);
    return retval;  
}

/**
 * @brief iic master 模式中断发送注册并启动
 * 
 * @param base iic号
 * @param addr 目标slave地址
 * @param master_send_cb 发送回调函数，用户需要实现该函数，待开始发送时中断里会调用该还是获取发送数据以及通知发送状态
 * @return int32_t 0:启动发送成功 其他:启动发送异常
 */
int32_t iic_master_interrupt_send(iic_base_t base, uint16_t addr, master_send_cb_t master_send_cb)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    iic_s->master_send_cb = master_send_cb;

    int32_t timeout = iic_s->timeout;

    while ((iic_p->STATUS& SR_BUSY) && timeout--)
    {
        delay_time(10);
    }
    if (iic_p->STATUS & SR_BUSY)
    {
        return -2;
    }

    iic_address(base,addr,IIC_M_WRITE);

    iic_p->INTCLR = (0x1 << 4);
    iic_p->CMD = CR_START | CR_TB;
    
    return 0;
}

/**
 * @brief iic master 查询接收
 * 
 * @param base iic号
 * @param addr 目标slave地址
 * @param buf 数据存储buff
 * @param count 打算接收的数据个数
 * @return int32_t 成功收到的数据个数
 */
int32_t iic_master_polling_recv(iic_base_t base, uint16_t addr, char *buf, int32_t count)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    int32_t timeout = iic_s->timeout;

    while ((iic_p->STATUS& SR_BUSY) && timeout--)
    {
        delay_time(10);
    }
    if (iic_p->STATUS & SR_BUSY)
    {
        return -2;
    }

    iic_address(base,addr,IIC_M_READ);
    iic_start(base);
    if(iic_s->errors & SR_ABITOR_LOST)
    {
        iic_stop(base);
        return -2;   
    }
    if(iic_s->errors & SR_TRANS_ERR)
    {
        iic_stop(base);
        return -2;  
    }

    if(iic_s->errors & SR_RXACK)
    {
        iic_stop(base);
        return -2;  
    }

    int32_t retval = iic_readbytes(base, (int8_t *)buf, count, 1);
    return retval;  
}

/**
 * @brief iic master 中断方式接收注册并启动
 * 
 * @param base iic号
 * @param addr 目标slave地址
 * @param master_recv_cb 接收数据的回调函数，需要用户实现在该函数会将每个接收到的字节下发并获取是否继续发送的信息
 * @return int32_t 0:启动接收成功 其他:启动接收异常 
 */
int32_t iic_master_interrupt_recv(iic_base_t base, uint16_t addr, master_recv_cb_t master_recv_cb)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    iic_s->master_recv_cb = master_recv_cb;

    int32_t timeout = iic_s->timeout;

    while ((iic_p->STATUS& SR_BUSY) && timeout--)
    {
        delay_time(10);
    }
    if (iic_p->STATUS & SR_BUSY)
    {
        return -2;
    }

    iic_address(base,addr,IIC_M_READ);

    iic_p->INTCLR = (0x1 << 4);
    iic_p->CMD = CR_START | CR_TB;
    
    return 0;
}

/**
 * @brief iic slave 查询方式发送
 * 
 * @param base iic号
 * @param buf 待发送的数据buff
 * @param count 待发送的数据数
 * @param last_ack_flag 最后一次发送收到的ack
 * @return int32_t 成功发送的个数（这里最后一次收到nack会被记录）
 */
int32_t iic_slave_polling_send(iic_base_t base, const char *buf, int32_t count, uint8_t *last_ack_flag)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    
    if(last_ack_flag != NULL)
    {
        *last_ack_flag = 0;
    }

    int32_t timeout = iic_s->timeout;

    while((((iic_p->STATUS) & (1<<6)) == 0)&& timeout--)  //等待总线被master寻找找到
    {
        delay_time(200);
    }
    if (((iic_p->STATUS) & (1<<6)) == 0)
    {
        return -2;
    }

    iic_p->INTCLR |= (1<<6);
    if((iic_p->STATUS) & (1<<2)){
        iic_p->INTCLR |= (1<<2);
    }
    iic_p->TXDR = buf[0];
    iic_p->CMD = CR_WRITE | CR_TB;

    int send_num = 0;
    for (;;) 
    {
        while(((iic_p->STATUS) & ((1<<2)|(1<<4))) == 0) 
        {
            //delay_time(200);
        }

        uint32_t status = iic_p->STATUS;

        if(status & (1<<2))//发送到1字节数据
        {
            iic_p->INTCLR |= (1<<2);
            send_num++;
            if(status & (1<<14))
            {
                iic_p->CMD = CR_WRITE | CR_STOP | CR_TB;
                if(last_ack_flag != NULL)
                {
                    *last_ack_flag = 1; //NACK
                }
                break;
            }
            else
            {
                if(send_num <= count)
                {
                    iic_p->TXDR = buf[send_num];
                    iic_p->CMD = CR_WRITE | CR_TB;
                }
                else
                {
                    //收光所有数据了
                    iic_p->CMD = CR_WRITE | CR_TB | CR_NACK;
                    break;
                }
            }
        }
        
        if(status & (1<<4)) //收到停止信号
        {
            iic_p->INTCLR |= (1<<4);
            iic_p->CMD = CR_STOP | CR_TB;
            return send_num;
        }
    }
    
    while(((iic_p->STATUS) & (1<<4)) == 0) 
    {
        //delay_time(200);
    }
    iic_p->INTCLR |= (1<<4);
    iic_p->CMD = CR_TB;
    return send_num;
}

/**
 * @brief iic slave 中断方式发送注册（被动启动）
 * 
 * @param base iic号
 * @param slave_send_cb 中断发送回调函数，需要用户编写，在该函数会请求待发送的数据并通知发送中的ack以及中断状态
 * @return int32_t 0:注册成功 其他:注册异常 
 */
int32_t iic_slave_interrupt_send(iic_base_t base, slave_send_cb_t slave_send_cb)
{
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    
    iic_s->slave_send_cb = slave_send_cb;
    return 0;
}

/**
 * @brief iic slave 模式查询方式接收
 * 
 * @param base iic号
 * @param buf 待接收数据buff
 * @param count 打算接收的数据个数
 * @return int32_t 成功接收的数据个数
 */
int32_t iic_slave_polling_recv(iic_base_t base, char *buf, int32_t count)
{
    bool skip = false;
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    int32_t timeout = iic_s->timeout;
    while((((iic_p->STATUS) & (1<<6)) == 0)&& timeout--)  //等待总线被master寻找找到
    {
        delay_time(200);
    }
    if (((iic_p->STATUS) & (1<<6)) == 0)
    {
        return -2;
    }

    iic_p->INTCLR |= (1<<6);
    iic_p->CMD = CR_TB;

    int recv_num = 0;
    for (;;) 
    {
        while(((iic_p->STATUS) & ((1<<1)|(1<<4))) == 0) 
        {
            //delay_time(200);
        }

        uint32_t status = iic_p->STATUS;
        if(status & (1<<1))//接收到1字节数据
        {
            iic_p->INTCLR |= (1<<1);

         
            if(skip)
            {
                 char dummy = iic_p->RXDR;
            }
            else
            {
                buf[recv_num] = iic_p->RXDR;
                #if IIC_DIRVER_DBG
                ci_loginfo(LOG_IIC_DRIVER,"recv---%d\n",buf[recv_num]);
                #endif
                recv_num++;
            }
            if(recv_num == count)
            {
                iic_p->CMD = CR_READ | CR_TB | CR_NACK;
                skip = true;
            }
            else
            {
                iic_p->CMD = CR_READ | CR_TB;
            }
        }
        
        if(status & (1<<4)) //收到停止信号
        {
            iic_p->INTCLR |= (1<<4);
            iic_p->CMD = CR_STOP | CR_TB;
            return recv_num;
        }
    }
}

/**
 * @brief iic slave 中断方式接收注册（被动启动）
 * 
 * @param base iic号
 * @param slave_recv_cb 数据接收回调函数，需要用户实现，该函数会将接收的数据传入并获取是否继续接收的状态
 * @return int32_t 0:注册成功 其他:注册异常 
 */
int32_t iic_slave_interrupt_recv(iic_base_t base, slave_recv_cb_t slave_recv_cb)
{
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    iic_s->slave_recv_cb = slave_recv_cb;
    
    return 0;
}

int32_t iic_master_multi_transmission(iic_base_t base, uint16_t addr, multi_transmission_msg *msg, int msg_count)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }


    if(msg_count > 0)
    {
        int32_t timeout = iic_s->timeout;

        while ((iic_p->STATUS& SR_BUSY) && timeout--)
        {
            delay_time(10);
        }
        if (iic_p->STATUS & SR_BUSY)
        {
            return -2;
        }

        for(int i = 0; i < msg_count; i++)
        {
            char* buf = msg[i].buf;
            int count = msg[i].size;
            iic_multi_transmission_type type = msg[i].flag;
            int last = 0;
            if(i == msg_count-1)
            {
                last = 1;
            }
 
            iic_address(base,addr,type);
            iic_start(base);
            if((iic_s->errors & SR_ABITOR_LOST)||(iic_s->errors & SR_TRANS_ERR)||(iic_s->errors & SR_RXACK))
            {
                iic_stop(base);
                return i;   
            }
            if(type == IIC_M_WRITE)
            {
                msg[i].write_size = iic_sendbytes(base, (int8_t *)buf, count, last, NULL);
                if(msg[i].write_size != count)
                {
                    return i;
                }
            }
            else
            {
                msg[i].read_size = iic_readbytes(base, (int8_t *)buf, count, last);
                if(msg[i].read_size != count)
                {
                    return i;
                }
            }
        }
    }
    
    return msg_count;
}

/**
 * @brief iic中断统一处理函数
 * 
 * @param base iic号
 */
void IIC_IRQHandler(iic_base_t base)
{
    iic_register_t* iic_p = (iic_register_t*)base;
    iic_status_t *iic_s = &iic0_status;
    if(IIC0 == base)
    {
        iic_s = &iic0_status;
    }

    uint32_t intstat = iic_p->INTSTAT;
    uint32_t status = iic_p->STATUS;
    #if IIC_DIRVER_DBG
    ci_loginfo(LOG_IIC_DRIVER,"intstat 0x%x status 0x%x\n",intstat,status);
    #endif

    /* 处理异常 */
    if(intstat & (1<<0))  //Timeout
    {
        iic_p->INTCLR |= (1<<0);
        iic_p->CMD = CR_STOP | CR_TB;
    }
    if(intstat & (1<<3))  //总线错误
    {
        iic_p->INTCLR |= (1<<3);
        iic_p->CMD = CR_STOP | CR_TB;
    }
    if(intstat & (1<<5))  //总线仲裁丢失
    {
        iic_p->INTCLR |= (1<<5);
        iic_p->CMD = CR_STOP | CR_TB;
    }
    /*if((status & 0x1ff) == 0)  
    {
        iic_p->INTCLR |= 0x1ff;
    }*/

    if(iic_s->is_master == 0)
    {
        if(intstat & (1<<6))  //总线被master寻找找到
        {
            if((status & (1<<13)) == 0)
            {
                iic_s->is_slave_recv = 1;
            }
            else
            {
                iic_s->is_slave_recv = 0;
            }
        }
        
        if(iic_s->is_slave_recv == 1)
        {
            if(intstat & (1<<6))  //总线被master寻找找到
            {
                iic_p->INTCLR |= (1<<6);
                
                if(intstat & (1<<2))
                {
                    iic_p->INTCLR |= (1<<2);
                }
                iic_p->CMD = CR_TB;
            }
            else if(intstat & (1<<1)) //接收到1字节数据
            {
                iic_p->INTCLR |= (1<<1);

                bool will_continue = false;
                if(iic_s->slave_recv_cb != NULL)
                {
                    will_continue = iic_s->slave_recv_cb(iic_p->RXDR,false);
                }
                if(!will_continue) 
                {
                    iic_p->CMD = CR_READ | CR_TB | CR_NACK;
                }
                else
                {
                    iic_p->CMD = CR_READ | CR_TB;
                }
            }
            else if(intstat & (1<<4)) //收到停止信号
            {
                iic_p->INTCLR |= (1<<4);
                if(iic_s->slave_recv_cb != NULL)
                {
                    iic_s->slave_recv_cb(0,true);
                }
                iic_p->CMD = CR_STOP | CR_TB;
                iic_s->is_slave_recv = -1;
            }
        }
        else if(iic_s->is_slave_recv == 0)
        {
            if(intstat & (1<<6))  //总线被master寻找找到
            {
                iic_p->INTCLR |= (1<<6);
                if(intstat & (1<<2))
                {
                    iic_p->INTCLR |= (1<<2);
                }
                char data = 0;
                if(iic_s->slave_send_cb != NULL)
                {
                    if(iic_s->slave_send_cb(&data, IIC_SENDSTATE_OK, IIC_ACKTYPE_ERR))
                    {
                        iic_p->TXDR = data;
                        iic_p->CMD = CR_WRITE | CR_TB;
                    }
                    else
                    {
                        iic_p->CMD = CR_WRITE | CR_TB | CR_NACK;
                    }
                }
            }
            else if(intstat & (1<<2)) //发送1字节数据
            {
                iic_p->INTCLR |= (1<<2);
                char data = 0;
                if(iic_s->slave_send_cb != NULL)
                {
                    if(iic_s->slave_send_cb(&data, IIC_SENDSTATE_OK, (IIC_AckType)((status & (1<<14))>>14)))
                    {
                        iic_p->TXDR = data;
                        iic_p->CMD = CR_WRITE | CR_TB;
                    }
                    else
                    {
                        iic_p->CMD = CR_WRITE | CR_TB | CR_NACK;
                    }
                }
                else
                {
                    iic_p->CMD = CR_WRITE | CR_TB | CR_NACK;
                }
            }
            else if(intstat & (1<<4)) //收到停止信号
            {
                iic_p->INTCLR |= (1<<4);
                iic_p->CMD = CR_STOP | CR_TB;
                if(iic_s->slave_send_cb != NULL)
                {
                    iic_s->slave_send_cb(NULL, IIC_SENDSTATE_STOP, IIC_ACKTYPE_ERR);
                }
                iic_s->is_slave_recv = -1;
            }
        }
        else
        {
            iic_p->INTCLR = 0x7f;
        }
    }
    else
    {
        if(iic_s->master_send_cb != NULL)
        {
            if(intstat & (1<<2))  //成功发送完1byte数据
            {
                iic_p->INTCLR |= (1<<2);
                if(status & (1<<14))  // 收到NACK
                {
                    iic_p->CMD = CR_STOP | CR_TB;
                    if(iic_s->master_send_cb != NULL)
                    {
                        iic_s->master_send_cb(NULL, IIC_SENDSTATE_NOTIFY, (IIC_AckType)((status & (1<<14))>>14));
                    }
                }
                else
                {
                    char txdata = 0;
                    bool vaild_data = false;
                    if(iic_s->master_send_cb != NULL)
                    {
                        vaild_data = iic_s->master_send_cb(&txdata, IIC_SENDSTATE_OK, (IIC_AckType)((status & (1<<14))>>14));
                    }
                    if(vaild_data)
                    {
                        iic_p->TXDR = txdata;
                        iic_p->CMD = CR_WRITE | CR_TB;
                    }
                    else
                    {
                        iic_p->CMD = CR_STOP | CR_TB;
                    }
                }
            }
            else if(intstat & (1<<4)) //收到停止信号
            {
                iic_p->INTCLR |= (1<<4);
                iic_p->CMD = CR_STOP | CR_TB;
                if(iic_s->master_send_cb != NULL)
                {
                    iic_s->master_send_cb(NULL, IIC_SENDSTATE_STOP, IIC_ACKTYPE_ERR);
                }
            }
        }
        else if (iic_s->master_recv_cb != NULL)
        {
            if(intstat & (1<<2))  //发送到1字节地址 
            {
                iic_p->INTCLR = (1 << 2);
                iic_p->CMD = CR_READ | CR_TB;
            }
            else if(intstat & (1<<1))  //接收到1字节数据
            {
                iic_p->INTCLR = (1 << 1);
                if(status & (1<<14))
                {
                    iic_p->CMD = CR_STOP | CR_READ | CR_TB;
                }
                else
                {
                    bool will_continue = false;
                    if(iic_s->master_recv_cb != NULL)
                    {
                        will_continue = iic_s->master_recv_cb(iic_p->RXDR,false);
                    }
                    if(!will_continue) 
                    {
                        iic_p->CMD = CR_READ | CR_TB | CR_NACK;
                    }
                    else
                    {
                        iic_p->CMD = CR_READ | CR_TB;
                    }
                }
            }
            else if(intstat & (1<<4)) //收到停止信号
            {
                iic_p->INTCLR |= (1<<4);
                iic_p->CMD = CR_STOP | CR_TB;
                if(iic_s->master_recv_cb != NULL)
                {
                    iic_s->master_recv_cb(iic_p->RXDR,true);
                }
            }
        }
        else
        {
            iic_p->INTCLR = 0x7f;
        }
    }
}


void IIC0_IRQHandler(void){
    IIC_IRQHandler(IIC0);
}


/***************************************** 兼容旧驱动接口 **************************************************/
/**
 * @brief I2C 发送数据
 *
 * @param client : 对应的I2C传输数据的SLAVE设备属性
 * @param buf : 发送数据缓存指针
 * @param count : 发送数据字节个数
 *
 * @return int32_t : count 成功发送数据字节个数，-1 超时，-2 错误
 */
int32_t i2c_master_only_send(char slave_ic_address, const char *buf, int32_t count)
{
    int32_t ret = -1;
    iic_base_t base = IIC0;

    multi_transmission_msg msg = {
        .buf = (char *)buf,
        .size = count,
        .flag = IIC_M_WRITE,
    };
    ret = iic_master_multi_transmission(base,slave_ic_address,&msg,1);
    return ret;
}


/**
 * @brief I2C 发送接收数据
 *
 * @param client : 对应的I2C传输数据的SLAVE设备属性
 * @param buf : 发送/接收数据缓存指针
 * @param send_len : 发送数据字节个数
 * @param rev_len : 接收数据字节个数
 *
 * @return int32_t : rev_len 接收数据个数，-2 错误
 */
int32_t i2c_master_send_recv(char slave_ic_address, char *buf, int32_t send_len, int32_t rev_len)
{
    int32_t ret = -1;
    iic_base_t base = IIC0;

    multi_transmission_msg msg[2] = {
        {
            .buf = buf,
            .size = send_len,
            .flag = IIC_M_WRITE,
        },
        {
            .buf = buf,
            .size = rev_len,
            .flag = IIC_M_READ,
        }
    };
    ret = iic_master_multi_transmission(base,slave_ic_address,msg,2);
    return ret;
}


/**
 * @brief I2C 接收数据
 *
 * @param client : 对应的I2C传输数据的SLAVE设备属性
 * @param buf : 发送/接收数据缓存指针
 * @param send_len : 发送数据字节个数
 * @param rev_len : 接收数据字节个数
 *
 * @return int32_t : rev_len 接收数据个数，-2 错误
 */
int32_t i2c_master_only_recv(char slave_ic_address, char *buf, int32_t rev_len)
{
    int32_t ret = -1;
    iic_base_t base = IIC0;

    multi_transmission_msg msg = {
        .buf = buf,
        .size = rev_len,
        .flag = IIC_M_READ,
    };
    ret = iic_master_multi_transmission(base,slave_ic_address,&msg,1);
    return ret;
}
