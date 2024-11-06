#include <stdbool.h>
#include "ci_log.h"
#include "cwsl_app_sample1.h"
#include "cwsl_manage.h"
#include "cwsl_template_manager.h"
#include "prompt_player.h"
#include "status_share.h"

extern void set_state_enter_wakeup(uint32_t exit_wakup_ms);
extern void cwsl_set_reserved_num(int num);


#define CWSL_WAKEUP_NUMBER 2                    // 可学习的唤醒词数量
#define WAKE_UP_ID 1
#define CWSL_CMD_NUMBER ((sizeof(reg_cmd_list) / sizeof(reg_cmd_list[0])))     // 可学习的命令词数量

#define CWSL_REG_TIMES (1)
#define CWSL_TPL_MINWORD (2)
#define CWSL_WAKEUP_THRESHOLD (37)
#define CWSL_CMD_THRESHOLD (35)
#define MAX_LEARN_REPEAT_NUMBER     ( CWSL_REG_TIMES + 2 )     // 学习时，重复的总次数, 建议范围 CWSL_REG_TIMES + 1、CWSL_REG_TIMES + 2、CWSL_REG_TIMES + 3
												
												
//FOR_REG_2TIMES_FLOW_V2 配置 1时, CWSL_REG_TIMES 必须是 2或3																		
#define FOR_REG_2TIMES_FLOW_V2      (0)             // 学习时，说两遍/三遍逻辑，版本二流程，后续均和第一次的比较，一致学习成功，不一致，最多支持说 3 次

#define CWSL_REG_VAD_LEVEL (0) // 学习过程，灵敏度选项配置： 0 低灵敏度，可减少噪声对学习的干扰，需学习过程大声说话；1 高灵敏度，但也可以导致干扰噪声干扰学习

typedef struct cwsl_reg_asr_struct
{
    int reg_cmd_id;         // 命令词ID
    int reg_play_id;        // 学习提示播报音ID
}cwsl_reg_asr_struct_t;

const cwsl_reg_asr_struct_t reg_cmd_list[]=
{   //命令词ID     //学习提示播报音ID
    ///tag-insert-cwsl-cmd-learn-by-id-functioncode-pos-7
    //cwsl_cmd_learn_by_idfunction
    {4,             20028},
    {5,             20029},
    {6,             20030},
    {7,             20031},
    {8,             20032},
    {9,             20033},
    {10,             20034},
    {11,             20035},
    {12,             20036},
    {13,             20037},

    
};

typedef enum
{
    CWSL_APP_REC, // 识别模式
    CWSL_APP_REG, // 学习模式
    CWSL_APP_DEL, // 删除模式
} cwsl_app_mode_t;

typedef struct
{
    int word_id;                    // 正在学习的命令词ID
    cwsl_word_type_t word_type;     // 正在学习的命令词类型
    cwsl_app_mode_t app_mode;       // 当前工作模式
    uint8_t continus_flag;          // 是否连续学习，用于简化连续学习命令词时的提示音,0:非连续学习; 1:连续学习
} cwsl_app_t;

cwsl_app_t cwsl_app;

static int get_next_reg_cmd_word_index();

int cwsl_app_reset();


///////////重新学习中的删除上一次学习词条的逻辑 ////////////
static uint16_t sg_prev_group_id = 0;
static uint32_t sg_prev_cmd_id = 0;
static cwsl_word_type_t sg_prev_wordtype ;

static uint16_t sg_prev_group_id_tmp = 0;
static uint32_t sg_prev_cmd_id_tmp = 0;
static cwsl_word_type_t sg_prev_wordtype_tmp ;
static int sg_prev_appword_id = 0;

void cwsl_set_prev_appwordid(int appword_id)
{
    sg_prev_appword_id = appword_id;
}

void cwsl_save_prev_info(uint32_t cmd_id,uint16_t group_id,cwsl_word_type_t wordtype)
{
    sg_prev_group_id_tmp = group_id;
    sg_prev_cmd_id_tmp = cmd_id;
    sg_prev_wordtype_tmp = wordtype;
}

void cwsl_update_prev_info(void)
{
    sg_prev_group_id = sg_prev_group_id_tmp;
    sg_prev_cmd_id = sg_prev_cmd_id_tmp;
    sg_prev_wordtype = sg_prev_wordtype_tmp;
}

void cwsl_clear_prev_info(void)
{
    sg_prev_group_id = (uint16_t)-1;
    sg_prev_cmd_id = (uint32_t)-1;
    sg_prev_wordtype = CMD_WORD;

    sg_prev_group_id_tmp = (uint16_t)-1;
    sg_prev_cmd_id_tmp = (uint32_t)-1;
    sg_prev_wordtype_tmp = CMD_WORD;
    sg_prev_appword_id = (int)-1;
}

void cwsl_get_prev_info(uint32_t* cmd_id,uint16_t* group_id,int *appword_id,cwsl_word_type_t *wordtype )
{
    *group_id = sg_prev_group_id;
    *cmd_id = sg_prev_cmd_id ;
    *appword_id = sg_prev_appword_id;
    *wordtype = sg_prev_wordtype;
}

//重新学习前 删除上一次模板
void cwsl_app_delete_prev_word(void)
{
    if(1 == CWSL_REG_TIMES)
    {// 满足前提，先删除 上一次的 模板
        uint32_t cmd_id;
        uint16_t group_id;
        cwsl_word_type_t wordtype;
        int app_wordid ;
        //获取上一次模板的信息
        cwsl_get_prev_info(&cmd_id,&group_id,&app_wordid,&wordtype);
        
        if( (cmd_id != (uint32_t)-1) && (group_id != (uint16_t)-1) )
        {//删除模板
            // 在学习状态下 发送 删除指定模板 消息
            cwsl_delete_word_when_reg(cmd_id, group_id,wordtype);
            //更新 重新 学习 信息和 播报提示信息
            cwsl_set_wordinfo(cmd_id,group_id,wordtype);

            if(app_wordid != (int)-1)
            {
                cwsl_app.word_id = app_wordid;
            }
        }
    }
}


////cwsl 事件响应函数////////////////////////////////////////////////

// cwsl 模块初始化事件响应
// 必须返回可学习的模板数量 函数内部不能阻塞，延时
int on_cwsl_init(cwsl_init_parameter_t *cwsl_init_parameter)
{    
    cwsl_set_reserved_num(CICWSL_TOTAL_TEMPLATE);
    cwsl_app.app_mode = CWSL_APP_REC;
    cwsl_app.word_id = -1;
    CI_ASSERT(CICWSL_TOTAL_TEMPLATE >= (CWSL_WAKEUP_NUMBER + CWSL_CMD_NUMBER), "not enough template space\n");
    cwsl_init_parameter->wait_time = 0;                                 // 使用默认值(20ms)
    cwsl_init_parameter->sg_reg_times = CWSL_REG_TIMES ;                
    cwsl_init_parameter->tpl_min_word_length = CWSL_TPL_MINWORD ;       
    cwsl_init_parameter->wakeup_threshold = CWSL_WAKEUP_THRESHOLD ;     
    cwsl_init_parameter->cmdword_threshold = CWSL_CMD_THRESHOLD ;       
    cwsl_init_parameter->reg_2times_flow_v2 = FOR_REG_2TIMES_FLOW_V2 ;

	if(FOR_REG_2TIMES_FLOW_V2)
	{
		if(CWSL_REG_TIMES < 2)
		{
			cwsl_init_parameter->sg_reg_times = 2;
			mprintf("CWSL_REG_TIMES %d must be 2 or 3\n",CWSL_REG_TIMES);
		}
		else if(CWSL_REG_TIMES > 3)
		{
			cwsl_init_parameter->sg_reg_times = 3;
			mprintf("CWSL_REG_TIMES %d must be 2 or 3\n",CWSL_REG_TIMES);
		}
	}
    cwsl_init_parameter->reg_vad_level = CWSL_REG_VAD_LEVEL ;
    return CWSL_WAKEUP_NUMBER + CWSL_CMD_NUMBER;
}

// 播放回调处理
static void cwsl_play_done_callback_default(cmd_handle_t cmd_handle)
{
}

// 播放回调处理, 开始录制模板
static void cwsl_play_done_callback_with_start_record(cmd_handle_t cmd_handle)
{
    cwsl_reg_record_start(); // 播放完 "开始学习" 提示音后，开始录制模板
}

// 学习开始事件响应 函数内部不能阻塞，延时
int on_cwsl_reg_start(uint32_t cmd_id, uint16_t group_id, cwsl_word_type_t word_type)
{    
    set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
    ci_logdebug(LOG_CWSL, "==on_cwsl_reg_start\n");
    cwsl_reg_record_stop(); // 在提示音播放期间，关闭模板录制功能

    cwsl_save_prev_info(cmd_id,group_id,word_type);// 记录 学习的命令词 cmd_id 、 group_id;

    if (word_type == WAKEUP_WORD)
    {
        // 播放提示音 "开始学习唤醒词"
        prompt_play_by_cmd_id(CWSL_REGISTRATION_WAKE, -1, cwsl_play_done_callback_with_start_record, false);
    }
    else
    {
        if (!cwsl_app.continus_flag) // 如果是连续学习，就不播报“开始学习”
        {
            prompt_play_by_cmd_id(CWSL_REGISTRATION_CMD, -1, cwsl_play_done_callback_default, false);
        }

        prompt_play_by_cmd_id(reg_cmd_list[cwsl_app.word_id].reg_play_id, -1, cwsl_play_done_callback_with_start_record, false);
    }
    return 0;
}

// 学习停止事件响应 函数内部不能阻塞，延时
int on_cwsl_reg_abort()
{
    ci_logdebug(LOG_CWSL, "==on_cwsl_reg_abort\n");
    prompt_play_by_cmd_id(CWSL_EXIT_REGISTRATION, -1, cwsl_play_done_callback_default, true);
    cwsl_app.app_mode = CWSL_APP_REC;
    return 0;
}

// 录制开始事件响应 函数内部不能阻塞，延时
int on_cwsl_record_start()
{
    ci_logdebug(LOG_CWSL, "==on_cwsl_record_start\n");
    return 0;
}

// 录制结束事件响应 函数内部不能阻塞，延时 
int on_cwsl_record_end(int times, cwsl_reg_result_t result)
{
    ci_logdebug(LOG_CWSL, "==on_cwsl_record_end %d,%d\n", times, result);

    TaskHandle_t sys_task_handle = xTaskGetHandle("UserTaskManageP");
    vTaskSuspend(sys_task_handle);
    if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
    {
        set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
        sys_ignore_exit_msg_in_queue();
        vTaskResume(sys_task_handle);
        if (cwsl_app.app_mode == CWSL_APP_REG)
        {
            cwsl_update_prev_info(); //本次记录学习的 cmd_id 、 group_id 已处理完（学习完成或失败);

            if (CWSL_RECORD_SUCCESSED == result)
            {
                if (MAX_LEARN_REPEAT_NUMBER > times)
                {
                    // if (times == 1)
                    {
                        prompt_play_by_cmd_id(CWSL_SPEAK_AGAIN, -1, cwsl_play_done_callback_with_start_record, true);
                    }
                    // else
                    // {
                    //     prompt_play_by_cmd_id(CWSL_DATA_ENTERY_SUCCESSFUL, -1, cwsl_play_done_callback_with_start_record, true);
                    // }
                }
                else
                {
                    // 学习次数超过上限，自动退出
                    prompt_play_by_cmd_id(CWSL_REG_FAILED, -1, cwsl_play_done_callback_default, true);
                    // 转回识别模式
                    cwsl_app_reset();
                }
            }
            else if ((CWSL_RECORD_FAILED == result ) || ( CWSL_RECORD_FAILED_BY_DEFAULTCMD == result ))
            {
                cicwsl_func_index  cmd_id_for_play = CWSL_DATA_ENTERY_FAILED;
                if(CWSL_RECORD_FAILED_BY_DEFAULTCMD == result)
                {
                    cmd_id_for_play = CWSL_REG_FAILED_DEFAULT_CMD_CONFLICT;
                }
                if((2 == CWSL_REG_TIMES)||(3 == CWSL_REG_TIMES))
                {// 
                    #if FOR_REG_2TIMES_FLOW_V2
                    if (MAX_LEARN_REPEAT_NUMBER > times)
                    {   // 播报 "学习失败，请再说一次" 或者 "与默认指令冲突，请换种说法"
                        //新流程 2遍情况下  和第一次不一致，学习失败
                        prompt_play_by_cmd_id(cmd_id_for_play, -1, cwsl_play_done_callback_with_start_record, true);
                    }
                    else
                    {
                        // 学习次数超过上限，自动退出
                        prompt_play_by_cmd_id(CWSL_REG_FAILED, -1, cwsl_play_done_callback_default, true);
                        // 转回识别模式
                        cwsl_app_reset();
                    }
                    #else
                    if(2 == CWSL_REG_TIMES)
                    {
                        //2遍情况下，学习失败，提示重新学习该词
                        if(CWSL_RECORD_FAILED_BY_DEFAULTCMD == result)
                        {
                            prompt_play_by_cmd_id(CWSL_REG_FAILED_DEFAULT_CMD_CONFLICT, 1, cwsl_play_done_callback_default, false);
                        }
                        else
                        {
                            //播报 "学习失败，请重新学习 XX 指令"
                            prompt_play_by_cmd_id(CWSL_REG_FAILED, 1, cwsl_play_done_callback_with_start_record, false);
                        }
                        //重新学习
                        extern void cwsl_app_reg_word_restart();
                        cwsl_app_reg_word_restart();
                    }
                    else
                    {//not this func
                    }
                    #endif 
                }
                else
                {
                    if (MAX_LEARN_REPEAT_NUMBER > times)
                    {   // 播报 "学习失败，请再说一次" 或者 "与默认指令冲突，请换种说法"
                        prompt_play_by_cmd_id(cmd_id_for_play, -1, cwsl_play_done_callback_with_start_record, true);
                    }
                    else
                    {
                        // 学习次数超过上限，自动退出
                        prompt_play_by_cmd_id(CWSL_REG_FAILED, -1, cwsl_play_done_callback_default, true);
                        // 转回识别模式
                        cwsl_app_reset();
                    }
                }
            }
            else if (CWSL_REG_FINISHED == result)
            {
                prompt_play_by_cmd_id(CWSL_REGISTRATION_SUCCESSFUL, -1, cwsl_play_done_callback_default, true);
                
                int next_cmd_index = get_next_reg_cmd_word_index(cwsl_app.word_id);

                if (CMD_WORD == cwsl_app.word_type && CWSL_CMD_NUMBER > cwsl_tm_get_reg_tpl_number(CMD_WORD) && (cwsl_app.word_id < (CWSL_CMD_NUMBER - 1)) && (next_cmd_index != -1))
                {
                    //为了配合学习下一个的逻辑，加了判断条件(cwsl_app.word_id < (CWSL_CMD_NUMBER - 1))且(next_cmd_index != -1)
                    cwsl_set_prev_appwordid(cwsl_app.word_id);
                    cwsl_app.word_id = next_cmd_index;
                    cwsl_app.continus_flag = 1;   
                
                    // 指定是连续学习，用于简化播报提示音
                    cwsl_reg_word(reg_cmd_list[next_cmd_index].reg_cmd_id, 0, CMD_WORD);
                }
                else
                {
                    // 转回识别模式
                    cwsl_app_reset();
                }
            }
            else if (CWSL_NOT_ENOUGH_FRAME == result)
            {
                if (MAX_LEARN_REPEAT_NUMBER > times)
                {
                    prompt_play_by_cmd_id(CWSL_TOO_SHORT, -1, cwsl_play_done_callback_with_start_record, true);
                }
                else
                {
                    // 学习次数超过上限，自动退出
                    prompt_play_by_cmd_id(CWSL_REG_FAILED, -1, cwsl_play_done_callback_default, true);
                    // 转回识别模式
                    cwsl_app_reset();
                }
            }
            else if (CWSL_REG_INVALID_DATA == result)
            {
                cwsl_reg_record_start(); 
            }
        }
    }
    else
    {
        vTaskResume(sys_task_handle);
        // 系统已进入唤醒词监听状态
        // 转回识别模式
        cwsl_app_reset();
        #if (USE_LOWPOWER_DOWN_FREQUENCY == 0)
        if (CWSL_REG_FINISHED == result)
        {
            prompt_play_by_cmd_id(CWSL_REGISTRATION_SUCCESSFUL, -1, cwsl_play_done_callback_default, false);
        }
        #endif
    }
    return 0;
}

// 删除模板成功事件响应 函数内部不能阻塞，延时
int on_cwsl_delete_successed()
{
    prompt_play_by_cmd_id(CWSL_DELETE_SUCCESSFUL, -1, cwsl_play_done_callback_default, true);
    cwsl_app.app_mode = CWSL_APP_REC; // 删除成功后，转回识别模式
    return 0;
}

// 识别成功事件响应 函数内部不能阻塞，延时
int on_cwsl_rgz_successed(uint16_t cmd_id, uint32_t distance)
{
    cmd_handle_t cmd_handle = cmd_info_find_command_by_id(cmd_id);
    sys_msg_t send_msg;
    send_msg.msg_type = SYS_MSG_TYPE_ASR;
    send_msg.msg_data.asr_data.asr_status = MSG_CWSL_STATUS_GOOD_RESULT;
    send_msg.msg_data.asr_data.asr_cmd_handle = cmd_handle;
    send_msg_to_sys_task(&send_msg, NULL);
    ci_logdebug(LOG_CWSL, "cwsl result: %d, %d\n", cmd_id, distance);
    return 0;
}

// 查找下一个需要学习的命令词索引，用于实现“从上次中断处开始学习”.
static int get_next_reg_cmd_word_index(int word_id)
{
    int ret = -1;

    // 查找学习的命令词
    uint8_t cmd_word_tm_index[CWSL_CMD_NUMBER];
    int cmd_tpl_count = cwsl_tm_get_words_index(cmd_word_tm_index, CWSL_CMD_NUMBER, -1, CMD_WORD);
    
    //为了配合学习下一个的逻辑，加了条件int i = word_id
    for (int i = word_id; i < CWSL_CMD_NUMBER; i++)
    {
        int find_flag = 0;
        for (int j = 0; j < cmd_tpl_count; j++)
        {
            if (cwsl_tm_get_tpl_cmd_id_by_index(cmd_word_tm_index[j]) == reg_cmd_list[i].reg_cmd_id)
            {
                find_flag = 1;
                break;
            }
        }
        if (!find_flag)
        {
            ret = i;
            return ret;
        }
    }
    return ret;
}



////cwsl API///////////////////////////////////////////////

// 学习唤醒词
void cwsl_app_reg_word(cwsl_word_type_t word_type)
{
    if (cwsl_app.app_mode == CWSL_APP_REC)
    {
        // 清除 前一次学习的信息
        cwsl_clear_prev_info();
        if (WAKEUP_WORD == word_type)
        {
            if ( ( CWSL_WAKEUP_NUMBER > cwsl_tm_get_reg_tpl_number(WAKEUP_WORD) ) && (cwsl_tm_get_left_tpl_number() > 0))
            {
                cwsl_app.word_type = WAKEUP_WORD;
                cwsl_reg_word(WAKE_UP_ID, 0, WAKEUP_WORD);
                cwsl_app.app_mode = CWSL_APP_REG;
            }
            else
            {
                // 唤醒词已经学习满了
                prompt_play_by_cmd_id(CWSL_TEMPLATE_FULL, -1, cwsl_play_done_callback_default, true);
            }
        }
        else if (CMD_WORD == word_type)
        {
            if ((CWSL_CMD_NUMBER > cwsl_tm_get_reg_tpl_number(CMD_WORD)) && (cwsl_tm_get_left_tpl_number() > 0) )
            {
                int next_cmd_index = get_next_reg_cmd_word_index(0);
                cwsl_app.word_id = next_cmd_index;
                cwsl_app.word_type = CMD_WORD;
                cwsl_reg_word(reg_cmd_list[next_cmd_index].reg_cmd_id, 0, CMD_WORD);
                cwsl_app.app_mode = CWSL_APP_REG;
                cwsl_app.continus_flag = 0;
            }
            else
            {
                // 命令词已经学习满了
                prompt_play_by_cmd_id(CWSL_TEMPLATE_FULL, -1, cwsl_play_done_callback_default, true);
            }
        }
    }
}

void cwsl_app_reg_next_callback()
{
    cwsl_app.word_type = CMD_WORD;
    cwsl_reg_word(reg_cmd_list[cwsl_app.word_id].reg_cmd_id, 0, CMD_WORD);
    cwsl_app.app_mode = CWSL_APP_REG;
    cwsl_app.continus_flag = 1;
}

void cwsl_app_reg_next(void)
{
    if (cwsl_app.app_mode == CWSL_APP_REG && cwsl_app.word_type == CMD_WORD)
    {
        cwsl_exit_reg_word();
        int next_cmd_index = (cwsl_app.word_id < (CWSL_CMD_NUMBER - 1)) ? get_next_reg_cmd_word_index(cwsl_app.word_id + 1) : -1;
        if(-1 == next_cmd_index)
        {   
            //未找到下一个，所以播报 “学习完成”
            prompt_play_by_cmd_id(CWSL_REGISTRATION_ALL, -1, cwsl_play_done_callback_default, true);
            // 转回识别模式
			cwsl_app_reset();
        }
        else
        {
            cwsl_app.word_id = next_cmd_index;
            prompt_play_by_cmd_id(CWSL_REGISTRATION_NEXT, -1, cwsl_app_reg_next_callback, true);
        }
    }
}

// 重新学习
void cwsl_app_reg_word_restart()
{
    if (cwsl_app.app_mode == CWSL_APP_REG)
    {
        cwsl_set_reg_restart_flag();
        cwsl_app_delete_prev_word();
        cwsl_reg_restart();
    }
}

// 播放回调处理, 退出学习
static void cwsl_play_done_callback_with_exit_reg(cmd_handle_t cmd_handle)
{
    // cwsl_exit_reg_word();
}

// 退出学习
void cwsl_app_exit_reg()
{
    if (cwsl_app.app_mode == CWSL_APP_REG)
    {
        // cwsl_reg_record_stop();
        cwsl_exit_reg_word();
        cwsl_app.app_mode = CWSL_APP_REC;
        prompt_play_by_cmd_id(CWSL_EXIT_REGISTRATION, -1, cwsl_play_done_callback_with_exit_reg, true);
    }
}

// 进入删除模式
void cwsl_app_enter_delete_mode()
{
    if (cwsl_app.app_mode == CWSL_APP_REC)
    {
        cwsl_app.app_mode = CWSL_APP_DEL;
        prompt_play_by_cmd_id(CWSL_DELETE_FUNC, -1, cwsl_play_done_callback_default, true);
    }
}

// 退出删除模式
void cwsl_app_exit_delete_mode()
{
    if (cwsl_app.app_mode == CWSL_APP_DEL)
    {
        cwsl_app.app_mode = CWSL_APP_REC;
        prompt_play_by_cmd_id(CWSL_EXIT_DELETE, -1, cwsl_play_done_callback_default, true);
    }
}

// 删除指定类型模板
// cmd_id: 指定要删除的命令词ID, 传入-1为通配符，忽略此项
// group_id: 指定要删除的命令词分组号, 传入-1为通配符，忽略此项
// word_type: 指定要删除的命令词类型，传入-1为通配符，忽略此项
void cwsl_app_delete_word(uint32_t cmd_id, uint16_t group_id, cwsl_word_type_t word_type)
{
    if (cwsl_app.app_mode == CWSL_APP_DEL)
    {
        cwsl_delete_word(cmd_id, group_id, word_type);
    }
}

// cwsl_manage模块复位，用于系统退出唤醒状态时调用
int cwsl_app_reset()
{
    cwsl_app.app_mode = CWSL_APP_REC;
    cwsl_manage_reset();
    return 0;
}



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

uint32_t cwsl_app_process_asr_msg(sys_msg_asr_data_t *asr_msg, cmd_handle_t *cmd_handle, uint16_t cmd_id)
{
    uint32_t ret = 0;
   
    switch(cmd_id)
    {
    case CWSL_REGISTRATION_WAKE://删除和学习不能连续调用
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            #if USE_AEC_MODULE
            ciss_set(CI_SS_AEC_MUTE_STATE,CI_SS_AEC_MUTE_ON);
            #endif
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_reg_word(WAKEUP_WORD);
        }
        ret = 2;
        break;
    case CWSL_REGISTRATION_CMD://删除和学习不能连续调用
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            #if USE_AEC_MODULE
            ciss_set(CI_SS_AEC_MUTE_STATE,CI_SS_AEC_MUTE_ON);
            #endif
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_reg_word(CMD_WORD);
        }
        ret = 2;
        break;
    case CWSL_REGISTRATION_NEXT:
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_reg_next();
        }
        ret = 2;
        break;
    case CWSL_REGISTER_AGAIN:
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_app_reg_word_restart();
        }
        ret = 2;
        break;
    case CWSL_EXIT_REGISTRATION:
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            #if USE_AEC_MODULE
            ciss_set(CI_SS_AEC_MUTE_STATE,CI_SS_AEC_MUTE_OFF);
            #endif
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_app_exit_reg();
        }
        ret = 2;
        break;
    case CWSL_DELETE_FUNC:
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_enter_delete_mode();
        }
        ret = 2;
        break;
    case CWSL_EXIT_DELETE:
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_exit_delete_mode();
        }
        ret = 2;
        break;
    case CWSL_DELETE_WAKE://删除和学习不能连续调用
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_delete_word((uint32_t)-1, (uint16_t)-1, WAKEUP_WORD);
        }
        ret = 2;
        break;
    case CWSL_DELETE_CMD://删除和学习不能连续调用
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_delete_word((uint32_t)-1, (uint16_t)-1, CMD_WORD);
        }
        ret = 2;
        break;
    case CWSL_DELETE_ALL://删除和学习不能连续调用
        if (ciss_get(CI_SS_START_SLEEP_PROCESS) == 0)
        {
            set_state_enter_wakeup(EXIT_WAKEUP_TIME); // 更新退出唤醒时间
            sys_ignore_exit_msg_in_queue();
            cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
            cwsl_app_delete_word((uint32_t)-1, (uint16_t)-1, ALL_WORD);
        }
        ret = 2;
        break;
    default:
        cwsl_reg_check_other_asrcmd(cmd_id,1);//记录默认命令词，检测是否与学习的词条
        if (cwsl_app.app_mode == CWSL_APP_REG || cwsl_app.app_mode == CWSL_APP_DEL)
        {
            send_nn_end_msg_to_cwsl(NULL, 0);
            ret = 2;
        }
        break;
    }
   
    return ret;
}





