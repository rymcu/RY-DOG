/**
 * @file ci130x_gpio.c
 * @brief  GPIO驱动文件
 * @version 0.1
 * @date 2019-05-07
 *
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 *
 */
#include <stdlib.h>
#include "ci130x_gpio.h"
#include "ci_log.h"

static gpio_irq_callback_list_t g_gpio_callback_list_root[6] =
{
    {NULL,NULL},{NULL,NULL},{NULL,NULL},{NULL,NULL},{NULL,NULL},{NULL,NULL}
};


/**
 * @brief GPIO控制器寄存器定义
 */
typedef struct
{
	volatile uint32_t gpio_data[256];    /*!< 0x000-0x3FC 数据寄存器 */
	volatile uint32_t gpio_dir;          /*!< 0x400-0x404 输入输出控制寄存器 */
	volatile uint32_t gpio_is;           /*!< 0x404-0x408 中断源（边沿/电平）寄存器 */
	volatile uint32_t gpio_ibe;          /*!< 0x408-0x40C 中断源双边沿触发寄存器 */
	volatile uint32_t gpio_iev;          /*!< 0x40C-0x410 中断事件寄存器 */
	volatile uint32_t gpio_ie;           /*!< 0x410-0x414 中断屏蔽寄存器 */
	volatile  uint32_t gpio_ris;          /*!< 0x414-0x418 中断原始状态寄存器 */
	volatile  uint32_t gpio_mis;          /*!< 0x418-0x41C 中断屏蔽状态寄存器 */
	volatile uint32_t gpio_ic;           /*!< 0x41C-0x420 中断清除寄存器 */
	volatile uint32_t gpio_afsel;        /*!< 0x420-0x424 模式控制寄存器 */
}gpio_register_t;

/*--------------------以下API可同时操作一个或多个pin脚------------------------*/

/**
 * @brief 管脚配置为输出模式
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_set_output_mode(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_dir |= pins;
}

/**
 * @brief 管脚配置为输入模式
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_set_input_mode(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_dir &= ~pins;
}

/**
 * @brief 获取管脚输入输出状态
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 *
 * @return uint8_t :pin(0-7)对应status的bit(0-7) 输出对应bit为1 输入对应bit为0
 */
uint8_t gpio_get_direction_status(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t status;
    status = gpio->gpio_dir & pins;
    return status;
}

/**
 * @brief 屏蔽管脚中断
 *
 * @param base :PA PB PC 
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_irq_mask(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_ie &= ~pins;
}

/**
 * @brief 取消屏蔽管脚中断
 *
 * @param base :PA PB PC
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_irq_unmask(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_ie |= pins;
}

/**
 * @brief 设置管脚的中断触发方式
 *
 * @param base :PA PB PC
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 * @param trigger :high_level_trigger、low_level_trigger、up_edges_trigger、
                    down_edges_trigger、both_edges_trigger
 */
void gpio_irq_trigger_config(gpio_base_t base,gpio_pin_t pins,
                            gpio_trigger_t trigger)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    switch(trigger)
    {
        case high_level_trigger:
        {
            gpio->gpio_iev |= pins;
            gpio->gpio_ibe &= ~(pins);
            gpio->gpio_is |= pins;
            break;
        }
        case low_level_trigger:
        {
            gpio->gpio_iev &= ~(pins);
            gpio->gpio_ibe &= ~(pins);
            gpio->gpio_is |= pins;
            break;
        }
        case up_edges_trigger:
        {
            gpio->gpio_iev |= pins;
            gpio->gpio_ibe &= ~(pins);
            gpio->gpio_is &= ~(pins);
            break;
        }
        case down_edges_trigger:
        {
            gpio->gpio_iev &= ~(pins);
            gpio->gpio_ibe &= ~(pins);
            gpio->gpio_is &= ~(pins);
            break;
        }
        case both_edges_trigger:
        {
            gpio->gpio_ibe |= pins;
            gpio->gpio_is &= ~(pins);
            break;
        }
    }
    gpio_irq_unmask(base,pins);
}

/**
 * @brief 管脚输出高电平
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_set_output_high_level(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_data[pins] = 0xFF;
}

/**
 * @brief 管脚输出低电平
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 */
void gpio_set_output_low_level(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_data[pins] = 0x0;
}

/**
 * @brief 获取管脚输入电平
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7,pin_all;中的一个或多个
 *
 * @return uint8_t :pin(0-7)对应value的bit(0-7) 高电平对应bit为1 低电平对应bit为0
 */
uint8_t gpio_get_input_level(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t value;
    value = gpio->gpio_data[pins];
    return value;
}

/*----------------------以下API一次只操作一个pin脚----------------------------*/

/**
 * @brief 获取某一个管脚输入输出状态
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7;中的一个
 *
 * @return uint8_t :输出:1 输入:0
 */
uint8_t gpio_get_direction_status_single(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t status;
    status = gpio->gpio_dir & pins;
    return (status?1:0);
}

/**
 * @brief 获取某个管脚中断屏蔽前状态
 *
 * @param base :PA PB PC
 * @param pins :pin_0,pin_1...pin_7;中的一个
 *
 * @return uint8_t :有中断为1 无中断为0
 */
uint8_t gpio_get_irq_raw_status_single(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t status;
    status = gpio->gpio_ris & pins;
    return (status?1:0);
}

/**
 * @brief 获取某个管脚中断屏蔽后状态
 *
 * @param base :PA PB PC
 * @param pins :pin_0,pin_1...pin_7;中的一个
 *
 * @return uint8_t :没有屏蔽为1 屏蔽为0
 */
uint8_t gpio_get_irq_mask_status_single(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t status;
    status = gpio->gpio_mis & pins;
    return (status?1:0);
}

/**
 * @brief 清除某个管脚的中断
 *
 * @param base :PA PB PC
 * @param pins :pin_0,pin_1...pin_7;中的一个
 */
void gpio_clear_irq_single(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_ic |= pins;
}

/**
 * @brief 某个管脚输出电平
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7;中的一个
 * @param level :1:输出高电平   0:输出低电平
 */
void gpio_set_output_level_single(gpio_base_t base,gpio_pin_t pins,uint8_t level)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    gpio->gpio_data[pins] = (level?0xFF:0x0);
}

/**
 * @brief 获取某个管脚输入电平
 *
 * @param base :PA PB PC PD
 * @param pins :pin_0,pin_1...pin_7;中的一个
 *
 * @return uint8_t :高电平:1 低电平:0
 */
uint8_t gpio_get_input_level_single(gpio_base_t base,gpio_pin_t pins)
{
    gpio_register_t* gpio = (gpio_register_t*)base;
    uint8_t value;
    value = gpio->gpio_data[pins] & pins;
    return (value?1:0);
}


/**
 * @brief gpio中断回调函数链表注册节点
 *
 * @param gpio_irq_callback_list 链表根节点
 * @param gpio_irq_callback_node 注册节点
 */
static void gpio_callback_add_node_in_list(gpio_irq_callback_list_t* gpio_irq_callback_list, gpio_irq_callback_list_t* gpio_irq_callback_node)
{
    gpio_irq_callback_list_t *list  = gpio_irq_callback_list;
    if(list->gpio_irq_callback == NULL)
    {
        *list = *gpio_irq_callback_node;
        return;
    }
    for( ; list->next != NULL ;list = list->next);
    list->next = gpio_irq_callback_node;
}


/**
 * @brief 搜索链表调用中断回调函数
 *
 * @param gpio_irq_callback_list 链表根节点
 */
static void call_irq_callback(gpio_irq_callback_list_t* gpio_irq_callback_list)
{
    gpio_irq_callback_list_t *list  = gpio_irq_callback_list;

    do
    {
        if(list->gpio_irq_callback != NULL)
        {
            list->gpio_irq_callback();
        }
        list = list->next;
    }while(list != NULL);
}


/**
 * @brief 注册中断回掉函数
 *
 * @param base :PA,PB,PC
 * @param gpio_irq_callback_node 中断回调函数节点
 */
void registe_gpio_callback(gpio_base_t base, gpio_irq_callback_list_t *gpio_irq_callback_node)
{
    switch (base)
    {
        case PC:
        {
            gpio_callback_add_node_in_list(&g_gpio_callback_list_root[2], gpio_irq_callback_node);
            break;
        }
        case PB:
        {
            gpio_callback_add_node_in_list(&g_gpio_callback_list_root[1], gpio_irq_callback_node);
            break;
        }
        case PA:
        {
            gpio_callback_add_node_in_list(&g_gpio_callback_list_root[0], gpio_irq_callback_node);
            break;
        }
    }
}

/*------------------------以下API为中断处理函数-------------------------------*/

void GPIO_IRQ_Default_proc(gpio_base_t base, int gpio_port_index)
{
	call_irq_callback(&g_gpio_callback_list_root[gpio_port_index]);
	for (int i = 0; i < 8; i++)
	{
		if(gpio_get_irq_mask_status_single(base,(0x1<<i)))
		{
			gpio_clear_irq_single(base,(0x1<<i));
			//ci_loginfo(LOG_GPIO_DRIVER,"GPIO%d Pin%d IRQ\n", gpio_port_index, i);
		}
	}
}

/**
 * @brief PA中断处理函数
 *
 */
void PA_IRQHandler(void)
{
	GPIO_IRQ_Default_proc(PA, 0);
}

/**
 * @brief PB中断处理函数
 *
 */
void PB_IRQHandler(void)
{
	GPIO_IRQ_Default_proc(PB, 1);
}

/**
 * @brief PC中断处理函数
 *
 */
void AON_PC_IRQHandler(void)
{
	GPIO_IRQ_Default_proc(PC, 2);
}
