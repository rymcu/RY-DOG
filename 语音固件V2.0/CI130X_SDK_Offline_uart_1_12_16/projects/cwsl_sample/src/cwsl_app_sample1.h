#ifndef __CWSL_APP_SAMPLE1_H__
#define __CWSL_APP_SAMPLE1_H__

#include "system_msg_deal.h"
#include "cwsl_manage.h"

typedef enum
{
    ///tag-insert-cwsl-remind-play-id-functioncode-pos-8
    //cwsl_remind_play_id_function
    CWSL_DELETE_ALL                  = 10019,      // 全部删除
    CWSL_DELETE_CMD                  = 10020,      // 删除命令词
    CWSL_DELETE_WAKE                 = 10021,      // 删除唤醒词
    CWSL_REGISTRATION_NEXT           = 10022,      // 学习下一个
    CWSL_REGISTRATION_CMD            = 10023,      // 学习命令词
    CWSL_REGISTRATION_WAKE           = 10024,      // 学习唤醒词
    CWSL_DELETE_FUNC                 = 10025,      // 我要删除
    CWSL_EXIT_DELETE                 = 10026,      // 退出删除
    CWSL_EXIT_REGISTRATION           = 10027,      // 退出学习
    CWSL_REGISTER_AGAIN              = 10028,      // 重新学习
    CWSL_SPEAK_AGAIN                 = 20019,      // 再说一次
    CWSL_REG_FAILED                  = 20020,      // 学习失败
    CWSL_DATA_ENTERY_FAILED           = 20021,      // 学习失败版本二
    CWSL_REGISTRATION_ALL           = 20022,      // 学习完成
    CWSL_REGISTRATION_SUCCESSFUL     = 20023,      // 学习成功
    CWSL_REG_FAILED_DEFAULT_CMD_CONFLICT           = 20024,      // 指令冲突
    CWSL_TOO_SHORT                   = 20025,      // 语音长度不够
    CWSL_TEMPLATE_FULL               = 20026,      // 超过上限
    CWSL_DELETE_SUCCESSFUL           = 20027,      // 删除成功


}cicwsl_func_index;


////cwsl process ASR message///////////////////////////////////////////////
/**
 * @brief 命令词自学习消息处理函数
 * 
 * @param asr_msg ASR识别结果消息
 * @param cmd_handle 命令词handle
 * @param cmd_id 命令词ID
 * @retval 1 数据有效,消息已处理
 * @retval 0 数据无效,消息未处理
 */
uint32_t cwsl_app_process_asr_msg(sys_msg_asr_data_t *asr_msg, cmd_handle_t *cmd_handle, uint16_t cmd_id);

// cwsl_manage模块复位，用于系统退出唤醒状态时调用
int cwsl_app_reset();

#endif  // __CWSL_APP_SAMPLE1_H__
