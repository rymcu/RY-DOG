/**
 * @file prompt_player.c
 * @brief 
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#include <stdio.h>
#include "command_file_reader.h"
#include "command_info.h"
#include "audio_play_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ci_log.h"
#include "prompt_player.h"
#include "codec_manager.h"
#include "status_share.h"
#include "ci_flash_data_info.h"
#include "asr_api.h"
#include "timers.h"


typedef 
struct voice_play_info_st
{
    cmd_handle_t cmd_handle;
    uint16_t start_index;
    uint16_t end_index;
    int select_index;                           //!选择播报索引, >=0:指定的选择索引号;-1:交由系统选择;-2:指定为voice_id播报
    play_done_callback_t play_done_callback;    //!播放结束的回调函数;
}voice_play_info_t;

typedef 
struct prompt_player_st
{
    FunctionalState enabled_flag;               //!提示音播报是否使能
    QueueHandle_t play_queue;                   //!播放队列
    SemaphoreHandle_t semaphore;    
    bool mute_voice_in_flag;                    //!语音输入mute标志
    uint8_t combination_number;
    uint8_t combination_index;
    uint8_t pause_asr_count;
    #if USE_AEC_MODULE
    TimerHandle_t timer_handle;
    #endif
    
    uint32_t combination_list[MAX_COMBINATION_COUNT];
}prompt_player_t;


static prompt_player_t prompt_player = 
{
    ENABLE,     //enabled_flag
    0,          //play_queue
    NULL,       //semaphore
    false,      //mute_voice_in_flag
    0,          //combination_number
    0,          //combination_index    
    0,          //pause_asr_count
    #if USE_AEC_MODULE
    0,          //timer_handle
    #endif
};

void resume_asr(uint8_t);

/**
 * @brief Get the mute voice in state object, DENOISE will used this function
 *
 * @return true mute voice in, data is 0
 * @return false voice in is normal
 */
bool get_mute_voice_in_state( void )
{
    return prompt_player.mute_voice_in_flag;
}

static void combination_callback(int32_t play_cb_state);

static void pop_from_play_queue()
{
    voice_play_info_t voice_play_info;
    BaseType_t rst = xQueueReceive(prompt_player.play_queue, &voice_play_info, 0);
    if (pdTRUE == rst)
    {
        #if ONE_SHOT_ENABLE
        resume_asr(!cmd_info_is_wakeup_word(voice_play_info.cmd_handle));
        #elif USE_AEC_MODULE
        resume_asr(ciss_get(CI_SS_AEC_MUTE_STATE));
        #else
        resume_asr(1);
        #endif
        if (voice_play_info.play_done_callback)
        {
            voice_play_info.play_done_callback(voice_play_info.cmd_handle);
        }
    }
}

static void clean_play_queue()
{
    voice_play_info_t voice_play_info;
    while (pdTRUE == xQueueReceive(prompt_player.play_queue, &voice_play_info, 0))
    {
        #if ONE_SHOT_ENABLE
        resume_asr(!cmd_info_is_wakeup_word(voice_play_info.cmd_handle));
        #elif USE_AEC_MODULE
        resume_asr(ciss_get(CI_SS_AEC_MUTE_STATE));
        #else
        resume_asr(1);
        #endif
        if (voice_play_info.play_done_callback)
        {
            voice_play_info.play_done_callback(voice_play_info.cmd_handle);
        }
    }
    prompt_player.combination_number = 0;
    prompt_player.combination_index = 0;
}


static int prompt_play_inner(voice_play_info_t *p_voice_play_info, bool from_callback)
{
    uint16_t voice_id_buffer[MAX_COMBINATION_COUNT];
    int32_t combination_number;
    if (p_voice_play_info->select_index == -2)  // 是否为voice ID播报
    {
        combination_number = 1;
        voice_id_buffer[0] = (uint16_t)(p_voice_play_info->cmd_handle);
    }
    else
    {
        combination_number = cmd_info_get_voice_index(
                                p_voice_play_info->start_index,
                                p_voice_play_info->end_index, 
                                p_voice_play_info->select_index, 
                                voice_id_buffer, 
                                MAX_COMBINATION_COUNT);
    }
    
    if (combination_number <= 0)
    {
        pop_from_play_queue();
        return 1;
    }
    else
    {
        if (combination_number <= MAX_COMBINATION_COUNT)
        {
            get_voice_addr_by_id(voice_id_buffer, prompt_player.combination_list, combination_number);
            prompt_player.combination_number = combination_number;
            prompt_player.combination_index = 0;

            /*audio PA on*/
            #if (PLAYER_CONTROL_PA)
            //audio_play_hw_start(ENABLE);
            audio_play_hw_pa_da_ctl(ENABLE,true);
            vTaskDelay(pdMS_TO_TICKS(100));
            #else
            audio_play_hw_pa_da_ctl(ENABLE,false);
            #endif
            pause_audio_play_prompt(prompt_player.combination_list[prompt_player.combination_index++], 1, combination_callback);
        }
        else
        {
            ci_logerr(CI_LOG_ERROR,"too many combination voice\n");
            clean_play_queue();
        }
    }
    return 0;
}

void pause_asr(uint8_t voice_in_mute)
{
    //xSemaphoreTake(muteMutex, portMAX_DELAY);
    if (prompt_player.pause_asr_count == 0)
    {
        #if USE_AEC_MODULE
        if(voice_in_mute) //oneshot时，命令词不做AEC，唤醒词做AEC
        {
            //#if (PAUSE_VOICE_IN_WITH_PLAYING || ONE_SHOT_ENABLE)
            //if (ciss_get(CI_SS_CWSL_IN_REG))
            {
                prompt_player.mute_voice_in_flag = true;
                cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,ENABLE);//TODO 要修改
                ciss_set(CI_SS_MIC_VOICE_STATUE,CI_SS_MIC_VOICE_MUTE);
                // asrtop_asr_system_pause();
            }
            //#endif
        }
        #else
        #if PAUSE_VOICE_IN_WITH_PLAYING
        prompt_player.mute_voice_in_flag = true;
        cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,ENABLE);//TODO 要修改
        ciss_set(CI_SS_MIC_VOICE_STATUE,CI_SS_MIC_VOICE_MUTE);
        asrtop_asr_system_pause();
        #endif
        #endif

        #if USE_AEC_MODULE
        ciss_set(CI_SS_INTERCEPT_ASR_OUT, 1);
        #endif
    }
    prompt_player.pause_asr_count++;
    //xSemaphoreGive(muteMutex);
}

#if USE_AEC_MODULE
void intercept_timer_callback(TimerHandle_t xTimer)
{
    ciss_set(CI_SS_INTERCEPT_ASR_OUT, 0);
    xTimerStop(prompt_player.timer_handle, 0);
} 
#endif

/**
 * @brief resume voice in, so can recover ASR, call this after play done,
 *
 */
void  resume_asr(uint8_t voice_in_mute)
{
    //xSemaphoreTake(muteMutex, portMAX_DELAY);    
    if (prompt_player.pause_asr_count > 0)
    {
        prompt_player.pause_asr_count--;
        if (prompt_player.pause_asr_count == 0)
        {
            #if USE_AEC_MODULE
            if(voice_in_mute) //oneshot时，命令词不做AEC，唤醒词做AEC
            {
                //#if (PAUSE_VOICE_IN_WITH_PLAYING || ONE_SHOT_ENABLE)
                // if (ciss_get(CI_SS_CWSL_IN_REG))
                {
                    cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,DISABLE);//TODO 要修改
                    ciss_set(CI_SS_MIC_VOICE_STATUE,CI_SS_MIC_VOICE_NORMAL);
                    // asrtop_asr_system_continue();
                    prompt_player.mute_voice_in_flag = false;
                }
                //#endif
            }
            #else
            #if PAUSE_VOICE_IN_WITH_PLAYING
            cm_set_codec_mute(HOST_MIC_RECORD_CODEC_ID,CODEC_INPUT,1,DISABLE);//TODO 要修改
            ciss_set(CI_SS_MIC_VOICE_STATUE,CI_SS_MIC_VOICE_NORMAL);
            asrtop_asr_system_continue();
            prompt_player.mute_voice_in_flag = false;
            #endif
            #endif

            #if USE_AEC_MODULE
            // ciss_set(CI_SS_INTERCEPT_ASR_OUT, 1);
            if (prompt_player.timer_handle == NULL)
            {
                prompt_player.timer_handle = xTimerCreate("intercept_timer", pdMS_TO_TICKS(600),pdFALSE, (void *)0, intercept_timer_callback);
            }
            xTimerStart(prompt_player.timer_handle, 0);
            #endif
        }
    }
    //xSemaphoreGive(muteMutex);
}

static void combination_callback(int32_t play_cb_state)
{
    voice_play_info_t voice_play_info;
    
    if (prompt_player.semaphore)
    {
        xSemaphoreTake(prompt_player.semaphore, portMAX_DELAY);
    }

    if (AUDIO_PLAY_CB_STATE_PLAY_THRESHOLD == play_cb_state &&
        (prompt_player.combination_index >= prompt_player.combination_number))
    {
        #if OPEN_ASR_IN_ADVANCE
        resume_asr();
        #endif
        if (prompt_player.semaphore)
        {
            xSemaphoreGive(prompt_player.semaphore);
        }
        return;
    }

    if ((prompt_player.combination_index >= prompt_player.combination_number) 
            || (AUDIO_PLAY_CB_STATE_PAUSE_BEFORE_THRESHOLD == play_cb_state)
            || (AUDIO_PLAY_CB_STATE_PAUSE_AFTER_THRESHOLD == play_cb_state)
            || (play_cb_state < 0))
    {
        //调用上一个播报音的结束回调
        BaseType_t rst = xQueueReceive(prompt_player.play_queue, &voice_play_info, 0);
        if (pdTRUE == rst)
        {
            // if ((AUDIO_PLAY_CB_STATE_PAUSE_BEFORE_THRESHOLD == play_cb_state) || (play_cb_state < 0))
            // {
            //     #if OPEN_ASR_IN_ADVANCE
            //     resume_asr();
            //     #endif
            // }
            #if !OPEN_ASR_IN_ADVANCE
            #if ONE_SHOT_ENABLE
            resume_asr(!cmd_info_is_wakeup_word(voice_play_info.cmd_handle));
            #elif USE_AEC_MODULE
            resume_asr(ciss_get(CI_SS_AEC_MUTE_STATE)); 
            #else
            resume_asr(1);
            #endif
            #endif
            if (voice_play_info.play_done_callback)     
            {                
                voice_play_info.play_done_callback(voice_play_info.cmd_handle);
            }
        }
        
        //播放下一个顺序播报音
        rst = xQueuePeek(prompt_player.play_queue, &voice_play_info, 0);
        if (pdTRUE == rst && (play_cb_state == AUDIO_PLAY_CB_STATE_DONE))
        {
            if (prompt_play_inner(&voice_play_info, true) != 0)
            {
                clean_play_queue();
            }
        }        
        else
        {
            clean_play_queue();

            #if (PLAYER_CONTROL_PA)
            /*audio PA on*/
            //audio_play_hw_stop(ENABLE);
            //gpio_set_output_low_level(AUDIO_PLAY_CARD_DEFAULT_GPIO_BASE,AUDIO_PLAY_CARD_DEFAULT_GPIO_PIN);
            audio_play_hw_pa_da_ctl(DISABLE,true);
            #else
            audio_play_hw_pa_da_ctl(DISABLE,false);
            #endif
        }
    }
    else if (prompt_player.combination_index < prompt_player.combination_number && play_cb_state == AUDIO_PLAY_CB_STATE_DONE)
    {
        //播放下一个组合播报音
        pause_audio_play_prompt(prompt_player.combination_list[prompt_player.combination_index++], 1, combination_callback);
    }

    if (prompt_player.semaphore)
    {
        xSemaphoreGive(prompt_player.semaphore);
    }
}


static uint32_t voice_play_info_add_to_queue(voice_play_info_t *p_voice_play_info)
{
    if (errQUEUE_FULL == xQueueSend(prompt_player.play_queue,p_voice_play_info, 0))
    {
        ci_logerr(LOG_CMD_INFO, "prompt queue full\n");
        return 1;
    }
    return 0;
}

void prompt_player_enable(FunctionalState state)
{
#if PROMPT_CLOSE_EN
    prompt_player.enabled_flag = state;
#endif
}


uint32_t prompt_play_by_cmd_handle(
            cmd_handle_t cmd_handle, 
            int select_index, 
            play_done_callback_t play_done_callback,
            bool preemptive)
{
    #if !AUDIO_PLAYER_ENABLE
        if(play_done_callback)
        {
            cmd_handle = (select_index == -2) ? NULL : cmd_handle;
            play_done_callback(cmd_handle);
        }
        return 0;
    #endif
        
    uint32_t ret = 1;

    if (prompt_player.semaphore == NULL)
    {
        prompt_player.semaphore = xSemaphoreCreateMutex();
    }
    if (prompt_player.semaphore)
    {
        xSemaphoreTake(prompt_player.semaphore, portMAX_DELAY);
    }

    if (!prompt_player.play_queue)
    {
        prompt_player.play_queue = xQueueCreate(5, sizeof(voice_play_info_t));
        if (!prompt_player.play_queue)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
    }

    if (!prompt_player.enabled_flag)
    {
        cmd_handle = cmd_info_find_command_by_string("<beep>");
        select_index = -1;
    }

    voice_play_info_t voice_play_info;
    voice_play_info.cmd_handle = cmd_handle;
    voice_play_info.play_done_callback = play_done_callback;
    voice_play_info.select_index = select_index;
    if (select_index == -2)
    {
        voice_play_info.start_index = 1;
        voice_play_info.end_index = 0;
    }
    else
    {
        cmd_info_get_voice_index_from_handle(cmd_handle, &(voice_play_info.start_index), &(voice_play_info.end_index));
    }

    if ((!is_valid_cmd_handle(cmd_handle) && (select_index != -2)) || !prompt_player.play_queue)
    {
        if (play_done_callback)
        {
            play_done_callback(cmd_handle);
        }
        if (prompt_player.semaphore)
        {
            xSemaphoreGive(prompt_player.semaphore);
        }
        return ret;
    }

    if (prompt_player.combination_number > 0 && preemptive)     //当前是否正在播报,需要打断
    {
        pause_play(NULL,NULL);
        int timeout = 2000;        //2秒
        if (prompt_player.semaphore)
        {
            xSemaphoreGive(prompt_player.semaphore);
        }
        while(prompt_player.combination_number > 0 && timeout > 0)
        {
            pause_play(NULL,NULL);
            vTaskDelay(1);
            timeout--;
        }
        if (prompt_player.semaphore)
        {
            xSemaphoreTake(prompt_player.semaphore, portMAX_DELAY);
        }  
    }    

    ret = voice_play_info_add_to_queue(&voice_play_info);
    if (ret != 0)
    {
        if (voice_play_info.play_done_callback)
        {
            cmd_handle = (select_index == -2) ? NULL : cmd_handle;
            voice_play_info.play_done_callback(cmd_handle);
        }
    }
    else
    {
        #if ONE_SHOT_ENABLE
        pause_asr(!cmd_info_is_wakeup_word(cmd_handle));
        #elif USE_AEC_MODULE
        pause_asr(ciss_get(CI_SS_AEC_MUTE_STATE)); 
        #else
        pause_asr(1);
        #endif
        if (preemptive || prompt_player.combination_number <= 0)
        {
            prompt_play_inner(&voice_play_info, false);
        }
        ret = 0;
    }
    
    if (prompt_player.semaphore)
    {
        xSemaphoreGive(prompt_player.semaphore);
    }
    return ret;
}

uint32_t prompt_play_by_voice_id(uint16_t voice_id, play_done_callback_t play_done_callback, bool preemptive)
{
    return prompt_play_by_cmd_handle(voice_id, -2, play_done_callback, preemptive);
}

uint32_t prompt_play_by_cmd_id(
            uint16_t cmd_id, 
            int select_index, 
            play_done_callback_t play_done_callback,
            bool preemptive)
{
    cmd_handle_t cmd_handle = cmd_info_find_command_by_id(cmd_id);
    return prompt_play_by_cmd_handle(cmd_handle, select_index, play_done_callback, preemptive);
}

uint32_t prompt_play_by_semantic_id(
            uint32_t semantic_id, 
            int select_index, 
            play_done_callback_t play_done_callback,
            bool preemptive)
{
    cmd_handle_t cmd_handle = cmd_info_find_command_by_semantic_id(semantic_id);
    return prompt_play_by_cmd_handle(cmd_handle, select_index, play_done_callback, preemptive);
}

uint32_t prompt_play_by_cmd_string(
            char* cmd_string, 
            int select_index, 
            play_done_callback_t play_done_callback,
            bool preemptive)
{
    cmd_handle_t cmd_handle = cmd_info_find_command_by_string(cmd_string);
    return prompt_play_by_cmd_handle(cmd_handle, select_index, play_done_callback, preemptive);
}


typedef struct multi_id_info_st
{
    uint32_t id;
    uint16_t select_index;
}multi_id_info_t;


uint32_t prompt_play_by_multi_cmd_id(prompt_play_info_t *p_play_info, int number, play_done_callback_t play_done_callback)
{
    uint32_t ret = 1;
    if (prompt_player.semaphore == NULL)
    {
        prompt_player.semaphore = xSemaphoreCreateMutex();
    }
    if (prompt_player.semaphore)
    {
        xSemaphoreTake(prompt_player.semaphore, portMAX_DELAY);
    }
    if (!prompt_player.play_queue)
    {
        prompt_player.play_queue = xQueueCreate(5, sizeof(voice_play_info_t));
    }

    if ((!prompt_player.play_queue) || (!prompt_player.enabled_flag) || (number > MAX_COMBINATION_COUNT))
    {
        if (play_done_callback)
        {
            play_done_callback(0);
        }
        if (prompt_player.semaphore)
        {
            xSemaphoreGive(prompt_player.semaphore);
        }
        return ret;
    }

    voice_play_info_t voice_play_info;
    if (prompt_player.combination_number > 0)     //当前是否正在播报,需要打断
    {
        if (RETURN_ERR == pause_play(NULL,NULL))
        {
            vTaskDelay(1);
            pause_play(NULL,NULL);
        }
        if (prompt_player.semaphore)
        {
            xSemaphoreGive(prompt_player.semaphore);
        }

        int timeout = 2000;        //2秒
        while(prompt_player.combination_number > 0 && timeout > 0)
        {
            timeout--;
            vTaskDelay(1);
        }
        if (prompt_player.semaphore)
        {
            xSemaphoreTake(prompt_player.semaphore, portMAX_DELAY); } }
    voice_play_info.cmd_handle = 0;
    voice_play_info.select_index = 0;
    voice_play_info.play_done_callback = play_done_callback;    
    voice_play_info.play_done_callback = play_done_callback;    
    ret = voice_play_info_add_to_queue(&voice_play_info);
    prompt_player.combination_number = 0;
    for (int i = 0;i < number;i++)
    {
    	cmd_handle_t cmd_handle = cmd_info_find_command_by_id(p_play_info[i].cmd_id);
        uint8_t select_index = p_play_info[i].select_index;
        uint16_t voice_id_buffer[MAX_COMBINATION_COUNT];
        uint16_t start_index = ((command_info_t*)cmd_handle)->voice_start_index;
        uint16_t end_index = ((command_info_t*)cmd_handle)->voice_end_index;
        uint8_t combination_number = cmd_info_get_voice_index(start_index, end_index, select_index, voice_id_buffer, MAX_COMBINATION_COUNT);
        if (prompt_player.combination_number + combination_number <= MAX_COMBINATION_COUNT)
        {
            get_voice_addr_by_id(voice_id_buffer, &prompt_player.combination_list[prompt_player.combination_number], combination_number);
            prompt_player.combination_number += combination_number;
        }
    }
    prompt_player.combination_index = 0;

    /*audio PA on*/
    #if (PLAYER_CONTROL_PA)
    //audio_play_hw_start(ENABLE);
    audio_play_hw_pa_da_ctl(ENABLE,true);
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    #if ONE_SHOT_ENABLE
    pause_asr(!cmd_info_is_wakeup_word(voice_play_info.cmd_handle));
    #elif USE_AEC_MODULE
    pause_asr(ciss_get(CI_SS_AEC_MUTE_STATE)); 
    #else
    pause_asr(1);
    #endif
    pause_audio_play_prompt(prompt_player.combination_list[prompt_player.combination_index++], 1, combination_callback);

    if (prompt_player.semaphore)
    {
        xSemaphoreGive(prompt_player.semaphore);
    }
    ret = 0;
    return ret;
}


uint32_t prompt_is_playing()
{
    if (prompt_player.combination_number > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint32_t prompt_stop_play()
{
    voice_play_info_t voice_play_info;
    stop_play(NULL,NULL);
    int timeout = 2000;
    while(prompt_player.combination_number > 0 && timeout > 0)
    {
        timeout--;
        vTaskDelay(1);
    }
    return 0;
}
