/**
 * @file ci_task_monitor.c
 * @brief  系统监控组件
 *
 * 负责监视控制加入监控队列里的每一个任务，以防有任务出现异常情况，
 * 避免因异常未及时处理而造成的损失。
 *
 * @version 0.1
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */

#include "ci_task_monitor.h"

/*监控任务的个数,最大为23*/
#define MONITOR_TAST_MAX_COUNT (23)/* 范围：0 - 23 根据实际情况合理设置*/
/*监控周期(单位：MS)*/
#define MONITOR_PERIOD (1000)

typedef struct
{
    uint8_t state;
    uint8_t id;
    uint8_t flag;
    uint8_t priority;
    uint32_t time;
    uint32_t last_time;
    TaskHandle_t handle;
}monitor_t;

static monitor_t join_monitor_list[MONITOR_TAST_MAX_COUNT];
static uint8_t join_monitor_count = 0;
static EventGroupHandle_t eventgroup = NULL;

typedef void (*func)(void);
func iwdg_callback = NULL;

/**
 * @brief 监控事件组创建函数
 *
 * @param call_back 系统异常复位前的回调函数
 */
void monitor_creat(void (*call_back)(void))
{
    /*创建事件标志组*/
    eventgroup = xEventGroupCreate();
    if(NULL == eventgroup)
    {
        /*事件标志组创建失败*/
        ci_logdebug(LOG_SYS_MONITOR,"xEventGroupCreate Failed\n");
    }
    ci_logdebug(LOG_SYS_MONITOR,"xEventGroupCreate Success\n");
    iwdg_callback = call_back;
}

/**
 * @brief 任务加入监控队列
 *
 * @param id 分配给任务的监控ID
 * @param time_ms 每次上报运行状态的最大时间间隔
 * @param handle 任务句柄（用来调节任务优先级）
 */
void join_monitor(uint8_t* id,uint32_t time_ms, TaskHandle_t handle)
{
    if(join_monitor_count > MONITOR_TAST_MAX_COUNT)
    {
        ci_logdebug(LOG_SYS_MONITOR,"已经达到监控任务的最大个数.\n");
        CI_ASSERT(0,"已经达到监控任务的最大个数.\n");
    }
    for(int i = 0;i < join_monitor_count;i++)
    {
        if(0 == join_monitor_list[i].state)
        {
            *id = join_monitor_list[i].id;
            join_monitor_list[i].state = 1;
            join_monitor_list[i].flag = 0;
            join_monitor_list[i].time = time_ms;
            join_monitor_list[i].handle = handle;
            return;
        }
    }
    /*给加入监控的任务编号，对应事件组的某个bit*/
    *id = join_monitor_count;
    /* 保存任务属性 */
    join_monitor_list[join_monitor_count].state = 1;
    join_monitor_list[join_monitor_count].flag = 0;
    join_monitor_list[join_monitor_count].id = join_monitor_count;
    join_monitor_list[join_monitor_count].time = time_ms;
    join_monitor_list[join_monitor_count].handle = handle;
    /*准备下个加入监控任务的编号*/
    join_monitor_count++;
    ci_logdebug(LOG_SYS_MONITOR,"已经监控 %d 个任务.\n",join_monitor_count);
}

/**
 * @brief 任务退出监控队列
 *
 * @param id 分配给任务的监控ID
 */
void exit_monitor(uint8_t id)
{
    join_monitor_list[id].state = 0;
}

/**
 * @brief 被监控任务状态上报函数
 *
 * @param id 任务的监控ID
 */
void task_alive(uint8_t id)
{
    xEventGroupSetBits(eventgroup, (0x1 << id));
}

/**
 * @brief 监控任务函数
 *
 * @param pvparameters 任务参数
 */
void task_monitor(void *pvparameters)
{
    /* 计算事件组的所有有效位 */
    uint32_t bit_all;
    EventBits_t bit_rev;
    /* 打开IWDG的时钟 */
    //scu_set_device_gate(HAL_IWDG_BASE,ENABLE);
    /* 打开IWDG的复位配置 */
    //Scu_Iwdg_RstSys_Config();
    /* 配置IWDG并启动 */
    iwdg_init_t init;
    init.irq = iwdg_irqen_enable;
    init.res = iwdg_resen_enable;
    init.count = (2 * MONITOR_PERIOD) * ((get_ipcore_clk()/0x10)/1000);/* IWDG时钟从IPCORE经过16分频得到：164000000 / 16;这里配置两秒*/
    iwdg_init(IWDG,init);
    iwdg_open(IWDG);
    ci_logdebug(LOG_SYS_MONITOR,"IWDG init ok.\n");

    for(;;)
    {
        bit_all = (((uint32_t)0x1)<<(join_monitor_count+1)) - 1;
        /* 等待所有任务发来事件标志 */
        bit_rev = xEventGroupWaitBits(eventgroup,/* 事件标志组句柄 */
                                      bit_all,/* 等待TASK_BIT_ALL被设置 */
                                      pdTRUE,/* 退出前TASK_BIT_ALL被清除*/
                                      pdTRUE,/* 等待TASK_BIT_ALL都被设置*/
                                      pdMS_TO_TICKS(MONITOR_PERIOD));/* 等待延迟时间 */
        xEventGroupClearBits(eventgroup,bit_all);
        /*监控任务正常，喂狗*/
        iwdg_feed(IWDG);
        ci_logdebug(LOG_SYS_MONITOR,"IWDG feed ok.\n");
        /* 判断任务是否正常喂狗 */
        for (int i = 0; i < join_monitor_count; i++)
        {
            if(join_monitor_list[i].state)
            {
                /* 该bit有没有被设置 */
                if((bit_rev & (0x1 << join_monitor_list[i].id)) != (0x1 << join_monitor_list[i].id))
                {
                    join_monitor_list[i].last_time += MONITOR_PERIOD;
                    if(join_monitor_list[i].last_time >= join_monitor_list[i].time)
                    {
                        ci_logdebug(LOG_SYS_MONITOR,"task flag is :%d\n",join_monitor_list[i].flag);
                        if(join_monitor_list[i].flag == 0)
                        {
                            ci_logdebug(LOG_SYS_MONITOR,"task id is :%d\n",join_monitor_list[i].id);
                            if(join_monitor_list[i].handle != NULL)
                            {
                                ci_logdebug(LOG_SYS_MONITOR,"调整任务优先级\n");
                                /*存储任务优先级*/
                                join_monitor_list[i].priority = uxTaskPriorityGet(join_monitor_list[i].handle);
                                if(configMAX_PRIORITIES >= (join_monitor_list[i].priority + 1))
                                {
                                    /*临时修改任务优先级*/
                                    vTaskPrioritySet(join_monitor_list[i].handle,join_monitor_list[i].priority + 1);
                                }
                            }

                            ci_logdebug(LOG_SYS_MONITOR,"task handle is :0x%x\n",join_monitor_list[i].handle );

                            join_monitor_list[i].last_time = 0;
                            join_monitor_list[i].flag = 1;
                        }
                        else
                        {
                            ci_loginfo(LOG_SYS_MONITOR,"task id is :%d\n",join_monitor_list[i].id);
                            if(NULL != iwdg_callback)
                            {
                                iwdg_callback();
                            }
                            ci_loginfo(LOG_SYS_MONITOR,"System Rst\n");
                            /* 任务死掉,复位系统 */
                            //Scu_SoftwareRst_System();
                            UartPollingSenddone((UART_TypeDef*)CONFIG_CI_LOG_UART);
                            dpmu_software_reset_system_config();
                            while(1);
                        }
                    }
                }
                else
                {
                    ci_logdebug(LOG_SYS_MONITOR,"%d:该bit已经被设置\n",join_monitor_list[i].id);
                    join_monitor_list[i].last_time = 0;
                    if(join_monitor_list[i].flag == 1)
                    {
                        ci_logdebug(LOG_SYS_MONITOR,"task id is :%d\n",join_monitor_list[i].id);
                        if(join_monitor_list[i].handle)
                        {
                            ci_logdebug(LOG_SYS_MONITOR,"还原任务优先级\n");
                            /*还原任务优先级*/
                            vTaskPrioritySet(join_monitor_list[i].handle, join_monitor_list[i].priority);
                        }
                        join_monitor_list[i].flag = 0;
                    }
                }
            }
        }
    }
}
