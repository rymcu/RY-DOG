
#ifndef __CWSL_MANAGE_H__
#define __CWSL_MANAGE_H__


#include "stdio.h"
#include "string.h"

#include "ci130x_system.h"

#define CWSL_MAX_TPL_NUM        32      // 最大模板数量


// 命令词类枚举型定义
typedef enum{
    CMD_WORD = 0,                       // 命令词
    WAKEUP_WORD,                        // 唤醒词
    ALL_WORD,                           // 所有学习的词
}cwsl_word_type_t;

// 学习结果枚举定义
typedef enum
{
    CWSL_RECORD_SUCCESSED,              // 学习成功
    CWSL_RECORD_FAILED,                 // 学习失败
    CWSL_REG_FINISHED,                  // 学习完成
    CWSL_REG_ABORT,                     // 退出学习
    CWSL_NOT_ENOUGH_FRAME,              // 帧数不够    
    CWSL_REG_INVALID_DATA,              // 数据无效
    CWSL_RECORD_FAILED_BY_DEFAULTCMD,     // 学习失败,与默认指令冲突
}cwsl_reg_result_t;

// cwsl_manage模块当前状态枚举定义
typedef enum
{
    CWSL_STA_IDLE,          // 空闲状态
    CWSL_STA_RECOGNIZATION, // 识别状态
    CWSL_STA_REG_TEMPLATE,  // 学习状态
    CWSL_STA_DEL_TEMPLATE,  // 删除状态
} cwsl_manage_status_t;


typedef enum
{
    CWSL_REG_1_TIMES = 1, // 学习 说一遍
    CWSL_REG_2_TIMES,     // 学习 说两遍
    CWSL_REG_3_TIMES,     // 学习 说三遍
} cwsl_manage_reg_time_t;

typedef struct{
    //uint8_t reg_distance;               // 学习距离门限，学习时，已录制两个模板的距离小于该值，及认为学习成功。              
    //uint8_t rec_distance;               // 识别距离门限，识别时，已录制的音频特征数据与已学习的任意模板的距离小于该值，及认为识别成功。
    //uint8_t head_match_distance;        // 匹配头的最大距离，默认值为（rec_max_distance + 25）
    uint8_t tpl_min_word_length;        // 自学习模板的最小字数，默认2个字，可设置模板最小2~5个字。
    uint8_t wait_time;                  // 等待时间, 单位为毫秒, 默认值为200。
    uint8_t sg_reg_times;               // 学习时 每个词需说几遍，默认1遍即可，可以设置1,2,3
    uint8_t wakeup_threshold;           // 学习的唤醒词的阈值门限，越小越灵敏，默认 35；
    uint8_t cmdword_threshold;           // 学习的命令词词的阈值门限，越小越灵敏，默认 35；
    uint8_t reg_2times_flow_v2;          // 学习时，说两遍逻辑，版本二流程，后续均和第一次的比较，一致学习成功，不一致，最多支持说 3 次
    uint8_t reg_vad_level; //学习时，vad的灵敏度选取，0 低灵敏度，减少噪音对学习干扰，需人声较大；1 高灵敏度，人声可小声一些
}cwsl_init_parameter_t;

void cwsl_set_vad_alc_config(int cmd);

void cwsl_init();

int cwsl_recognize_start(cwsl_word_type_t word_type);

int cwsl_recognize_stop();

int cwsl_reg_word(uint32_t cmd_id, uint16_t group_id, cwsl_word_type_t word_type);

int cwsl_exit_reg_word();

void cwsl_set_reg_restart_flag(void);

int cwsl_reg_restart();

int cwsl_reg_record_start();

int cwsl_reg_record_stop();

int cwsl_reg_check_other_asrcmd(uint32_t cmd_id,int valid);

// cwsl_manage模块复位，用于系统退出唤醒状态时调用
int cwsl_manage_reset();

// 获取cwsl模板大小上限(BYTE)
int cwsl_get_max_tpl_size();

// 获取cwsl_manage模块当前状态
cwsl_manage_status_t cwsl_get_status(void);
// 删除唤醒词
// cmd_id： 指定命令词ID, -1为通配符（删除满足其他条件的所有模板）
// group_id: 指定分组号， -1为通配符（删除满足其他条件的所有模板）
// word_type: 指定是否唤醒词，-1为通配符（删除满足其他条件的所有模板）
int cwsl_delete_word(uint32_t cmd_id, uint16_t group_id, cwsl_word_type_t word_type);

//在学习状态下 发送 删除指定模板 消息
int cwsl_delete_word_when_reg(uint32_t cmd_id, uint16_t group_id, cwsl_word_type_t word_type);

// 设置学习和识别时的距离门限
// reg_max_distance: 学习距离门限，学习时，已录制的3个模板中有任意两个模板的距离小于该值，及认为学习成功。
// rec_max_distance: 识别距离门限，识别时，已录制的音频特征数据与已学习的任意模板的距离小于该值，及认为识别成功。
void cwsl_set_distance_threshold(uint8_t reg_max_distance, uint8_t rec_max_distance);

//设置学习命令词修改id信息
void cwsl_set_wordinfo(uint32_t cmd_id,uint16_t group_id,cwsl_word_type_t wordtype);

///////////////应用层不使用一下接口////////////////////////////////
int send_nn_start_msg_to_cwsl();

int send_nn_out_msg_to_cwsl(short * data, short data_length);

int send_nn_end_msg_to_cwsl(int *data, int data_length);





#endif //__CWSL_MANAGE_H__

