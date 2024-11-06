/**
 * @file command_info.h
 * @brief Used to access information of command word.
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */

#ifndef __COMMAND_INFO_H__
#define __COMMAND_INFO_H__


#include <stdint.h>
#include "product_semantic.h"

#ifdef __cplusplus
extern "C" {
#endif

//!Invalid 16bit ID.
#define INVALID_SHORT_ID        0xFFFF

//!Invalid 32bit ID.
#define INVALID_LONG_ID	        0xFFFFFFFF

//!Invalid handle.
#define INVALID_HANDLE          ((uint32_t)-1)

//!Used to get product ID from semantic ID.
#define get_product_id_from_semantic_id(semantic_id) ((semantic_id >> 18) & 0x00003FFF)

//!Used to get function ID from semantic ID.
#define get_function_id_from_semantic_id(semantic_id) ((semantic_id >> 6) & 0x00000FFF)


/**
 * @brief Type define of voice select type.
 * 
 */
typedef enum
{
    VOICE_SELECT_RANDOM,        /*!<random selection.           */
    VOICE_SELECT_USER,          /*!<select by user program.     */
}voice_select_type_t;

//!Define the type of command handle.
typedef void* cmd_handle_t;

#pragma pack(push)
#pragma pack(1)

typedef struct
{
    uint8_t combination_voice_number;
    uint32_t *p_voice_data_address_table;
}voice_select_item_t;

typedef struct voice_st
{
    voice_select_type_t select_type;
    uint8_t select_item_number;
    voice_select_item_t *p_select_table;
}voice_t;

typedef struct {
    uint8_t length;
    uint8_t data[0];
}cmd_sr_data_t;

#pragma pack(pop)






/**
 * @brief Initialize command information module. Must invoked after model changed.
 * 
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_info_init(uint32_t cmd_file_addr_in_flash, uint32_t voice_patition_addr, uint8_t model_group_id);

/**
 * @brief Destroy command information module,and release memory resource.
 * 
 */
extern void cmd_info_destroy();

/**
 * @brief Change current model group.
 * 
 * @param model_group_id ID of medel group in cmd file.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_info_change_cur_model_group(uint8_t model_group_id);

/**
 * @brief Find command information by command string.
 * 
 * @param cmd_string String of the specifiled command.
 * @return cmd_handle_t The handle to the specifiled command.
 */
extern cmd_handle_t cmd_info_find_command_by_string(const char * cmd_string);

/**
 * @brief Find command information by command string.
 * 
 * @param cmd_id ID of the specifiled command.
 * @return cmd_handle_t The handle to the specifiled command.
 */
extern cmd_handle_t cmd_info_find_command_by_id(uint16_t cmd_id);

/**
 * @brief Find command information by semantic ID.
 * 
 * @param semantic_id A unique ID that cresponding to semantic of the spcifiled command.
 * @return cmd_handle_t The handle to the specifiled command.
 */
extern cmd_handle_t cmd_info_find_command_by_semantic_id(uint32_t semantic_id);
/**
 * @brief Get command ID by command string.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint16_t command ID.
 */
extern uint16_t cmd_info_get_command_id(cmd_handle_t cmd_handle);

/**
 * @brief Get command string by command handle.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return char* Pointer to the string of the specifiled command. NULL if failed.
 */
extern char * cmd_info_get_command_string(cmd_handle_t cmd_handle);

/**
 * @brief Get the score of command by command ID.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint8_t The score of command.
 */
extern uint8_t cmd_info_get_cmd_score(cmd_handle_t cmd_handle);

/**
 * @brief Get the semantic ID by command ID.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t Semantic ID of command.
 */
extern uint32_t cmd_info_get_semantic_id(cmd_handle_t cmd_handle);

/**
 * @brief Get the function ID by command ID.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t function ID fo command. Used to specify some operation or response after this command be sended.
 */
extern uint16_t cmd_info_get_function_id(cmd_handle_t cmd_handle);

/**
 * @brief Checks whether the specifiled command word is special word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_special_word(cmd_handle_t cmd_handle);

/**
 * @brief Checks whether the specifiled command word is wakeup word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_wakeup_word(cmd_handle_t cmd_handle);

/**
 * @brief Checks whether the specifiled command word is combo word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_combo_word(cmd_handle_t cmd_handle);


/**
 * @brief Checks whether the specifiled command word have auto play flag.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_auto_play(cmd_handle_t cmd_handle);

/**
 * @brief Get the wait time for the specifiled special word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return int32_t Wait time.
 */
extern int32_t cmd_info_get_special_wait_count(cmd_handle_t cmd_handle);

/**
 * @brief Get the select type of multiple voice options.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return voice_select_type_t VOICE_SELECT_RANDOM:Random selection; VOICE_SELECT_USER: Select by user code.
 */
//extern voice_select_type_t get_voice_select_type(cmd_handle_t cmd_handle);

/** @} */



/**
 * @brief Get models ID.
 * 
 * @param p_dnn_id A pointer to a uint32_t variable used to receive dnn file id.
 * @param p_asr_id A pointer to a uint32_t variable used to receive asr file id.
 * @param p_voice_group_id A pointer to a uint32_t variable used to receive voice group id.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_info_get_cur_model_id(uint32_t* p_dnn_id, uint32_t* p_asr_id, uint32_t* p_voice_group_id);

/**
 * @brief Get flags of command by command ID.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint8_t Flags of the specifiled command.
 */
extern uint8_t cmd_info_get_cmd_flag(cmd_handle_t cmd_handle);

/**
 * @brief Checks whether the specifiled command word is expected word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_expected_word(cmd_handle_t cmd_handle);

/**
 * @brief Checks whether the specifiled command word is unexppected word.
 * 
 * @param cmd_handle A handle to the specifiled command.
 * @return uint32_t 1:yes, 0:no
 */
extern uint32_t cmd_info_is_unexpected_word(cmd_handle_t cmd_handle);

extern uint32_t prompt_is_playing();

//extern uint8_t get_voice_option_number(cmd_handle_t cmd_handle);

extern uint32_t get_voice_addr_by_index(uint16_t voice_index);

extern voice_t * cmd_info_get_cmd_voice(uint16_t cmd_id);

extern int32_t cmd_info_get_voice_index(uint16_t start_index, uint16_t end_index, uint8_t select_index, uint16_t* id_buffer, int buffer_length);

extern uint32_t is_valid_cmd_handle(cmd_handle_t cmd_handle);

extern uint32_t cmd_info_get_voice_index_from_handle(cmd_handle_t cmd_handle, uint16_t* start_index, uint16_t* end_index);

#ifdef __cplusplus
}
#endif
#endif




