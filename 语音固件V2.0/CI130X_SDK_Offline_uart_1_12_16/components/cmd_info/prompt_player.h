/**
 * @file prompt_player.h
 * @author your name (you@domain.com)
 * @brief Used to play prompt that corresponding to command.
 * @version 0.1
 * @date 2019-10-23
 * 
 * @copyright Copyright (c) 2019  Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef __PROMPT_PLAYER_H__
#define __PROMPT_PLAYER_H__

#include "command_info.h"


/**
 * @addtogroup prompt_player
 * @{
 */


//!Maximum combined prompts number.
#define MAX_COMBINATION_COUNT           16  

/**
 * @brief Define the type of callback function that needed by play interfaces.
 * 
 */
typedef void (*play_done_callback_t)(cmd_handle_t cmd_handle);

/**
 * @brief Prompt infomation struct.Needed by function "prompt_play_by_multi_cmd_id",used to specify which prompt to play.
 * 
 */
typedef struct prompt_play_info_st
{
    uint32_t cmd_id;			    //!Command ID.
    uint16_t select_index;			//!Index of specifiled prompt in options. -1: means select by command information.
}prompt_play_info_t;

/**
 * @brief Play the prompt that corresponding to the command that specifiled by cmd_handle.
 * 
 * @param cmd_handle The handle to the specifiled command.
 * @param select_index Index of specifiled prompt in options. -1: means select by command information.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @param preemptive Whether to interrupt the current playback.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_cmd_handle(
                cmd_handle_t cmd_handle, 
                int select_index, 
                play_done_callback_t play_done_callback,
                bool preemptive);

/**
 * @brief Play the prompt that corresponding to the command that specifiled by command ID.
 * 
 * @param cmd_id command ID.
 * @param select_index Index of specifiled prompt in options. -1: means select by command information.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @param preemptive Whether to interrupt the current playback.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_cmd_id(
                uint16_t cmd_id,
                int select_index, 
                play_done_callback_t play_done_callback,
                bool preemptive);

/**
 * @brief Play the prompt that corresponding to the command that specifiled by semantic ID.
 * 
 * @param semantic_id A unique ID that cresponding to semantic of the spcifiled command.
 * @param select_index Index of specifiled prompt in options. -1: means select by command information.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @param preemptive Whether to interrupt the current playback.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_semantic_id(
                uint32_t semantic_id, 
                int select_index, 
                play_done_callback_t play_done_callback,
                bool preemptive);

/**
 * @brief Play the prompt that corresponding to the command that specifiled by string.
 * 
 * @param cmd_string Pointer to a string of the specifiled command.
 * @param select_index Index of specifiled prompt in options. -1: means select by command information.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @param preemptive Whether to interrupt the current playback.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_cmd_string(
                char* cmd_string, 
                int select_index, 
                play_done_callback_t play_done_callback,
                bool preemptive);

/**
 * @brief Play the prompt that corresponding to the command that specifiled by voice file id.
 * 
 * @param voice_id A unique ID that cresponding to the spcifiled voice file.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @param preemptive Whether to interrupt the current playback.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_voice_id(uint16_t voice_id, play_done_callback_t play_done_callback, bool preemptive);

/**
 * @brief Set the enable state of the prompt player.
 * 
 * @param state ENABLE/DISABLE
 */
extern void prompt_player_enable(FunctionalState state);

/**
 * @brief Is playing prompt.
 * 
 * @return uint32_t 0:no playback, nonzero: Is playing.
 */
extern uint32_t prompt_is_playing();

/**
 * @brief Play multiple prompt.
 * 
 * @param p_play_info Pointer to an array of prompt_play_info_t.
 * @param number The number of prompts need to play.
 * @param play_done_callback Pointer to a callback function,that invoked by prompt player when prompt play over.
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_play_by_multi_cmd_id(prompt_play_info_t *p_play_info, int number, play_done_callback_t play_done_callback);

/** @} */


/**
 * @brief Stop prompt play back.
 * 
 * @return uint32_t 0:sucessful, nonzero: failed.
 */
extern uint32_t prompt_stop_play();

#endif



