#ifndef __COMMAND_FILE_READER_V3_H__
#define __COMMAND_FILE_READER_V3_H__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#define CMD_FLAG_WAKEUP_WORD        0x01		//!Wakeup word flag.
#define CMD_FLAG_COMBO_WORD         0x02		//!Combo word flag.
#define CMD_FLAG_EXPECTED_WORD      0x04		//!Expected word flag.
#define CMD_FLAG_UNEXPECTED_WORD    0x08		//!Unexpected word flag.
#define CMD_FLAG_SPECIAL_WORD       0x10		//!Special word flag.
#define CMD_FLAG_AUTO_PLAY          0x20        //!Auto play flag.


#pragma pack(push)
#pragma pack(1)

/**
 * @brief Command information
 * 
 */
typedef struct command_info_st
{
	uint8_t flags;					//!Use to save flags that indicate this command is a special word,or is a combo word,or is a expected word,and so on.
	uint8_t score;					//!100 times the true float value of score.
	uint16_t voice_start_index;		//!Index of this command's voice in voice table.
	uint16_t voice_end_index;		//!Index of this command's voice in voice table.
	uint16_t command_id;			//!Identity of command.
	uint32_t semantic_id;			//!Semantically related unique value.
    uint16_t function_id;
}command_info_t;

/**
 * @brief time wait after special word.
 * 
 */
typedef struct Special_wait_count_st
{
	uint16_t index_of_cmd_table;	//!Identity of command.
	uint8_t wait_count;				//!Time wait after special word.
}special_wait_count_t;

/**
 * @brief Index of command string, used to get command id by command string.
 * 
 */
typedef struct command_string_index_st
{
	uint32_t command_string_offset;	//!Index of command string in table of command string.
	uint16_t index_of_cmd_table;	//!Identity of command.
}command_string_index_t;

/**
 * @brief Voice information
 * 
 */
typedef struct voice_info_st
{
	uint16_t voice_group;			//!voice group information.
	uint16_t voice_id;				//!Voice ID.
	//uint32_t voice_data_addr;		//!Address of voice data in flash.
}voice_info_t;

#pragma pack(pop)


/**
 * @brief Initialize command file reader.
 * 
 * @param start_addr_in_flash Start address of command file in flash.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_reader_init(uint32_t start_addr_in_flash);


/**
 * @brief Get models number.
 * 
 * @return uint32_t Number of model in cmd file.
 */
extern uint32_t cmd_file_get_model_number();

/**
 * @brief Change current model group.
 * 
 * @param model_group_id ID of medel group in cmd file.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_change_model(uint32_t model_index);


/**
 * @brief Get models ID.
 * 
 * @param p_dnn_id A pointer to a uint32_t variable used to receive dnn file id.
 * @param p_asr_id A pointer to a uint32_t variable used to receive asr file id.
 * @param p_voice_group_id A pointer to a uint32_t variable used to receive voice group id.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_get_cur_model_id(uint32_t *p_dnn_id, uint32_t *p_asr_id);

/**
 * @brief Get number of commands in current model.
 * 
 * @return uint32_t number of commands.
 */
extern uint32_t cmd_file_get_command_number();

/**
 * @brief Reads commands table to the specifiled buffer.
 * 
 * @param table_buffer 	The buffer use to save commands table, provided by caller. 
 * 						Buffer size is larger or equal to commands number times sizeof(command_info_t).
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_read_command_table(void* table_buffer);

/**
 * @brief Get number of special words.
 * 
 * @return uint32_t number of special words.
 */
extern uint32_t cmd_file_get_special_word_number();

/**
 * @brief Reads special words table to the specifiled buffer.
 * 
 * @param table_buffer 	The buffer use to save special words table, provided by caller.
 * 						Buffer size is larger or equal to special words number times sizeof(special_wait_count_t).
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_read_special_word_table(void* table_buffer);

/**
 * @brief Read string index table to the specifiled buffer.
 * 
 * @param table_buffer	The buffer use to save string index table, provided by caller.
 * 						Buffer size is larger or eaqual to commands number times sizeof(command_string_index_t).
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_read_string_index_table(void* table_buffer);

/**
 * @brief Get size of string table.
 * 
 * @return uint32_t Size of string table.
 */
extern uint32_t cmd_file_get_string_table_size();

/**
 * @brief Read string table to the specifiled buffer.
 * 
 * @param table_buffer The buffer use to save string table, provided by caller.
 * 						Buffer size is larger or eaqual to string table size.
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_read_string_table(void* table_buffer);

/**
 * @brief Get number of voice.
 * 
 * @return uint32_t Number of voice.
 */
extern uint32_t cmd_file_get_voice_number();

/**
 * @brief Read voice table to the specifiled buffer.
 * 
 * @param table_buffer 	The buffer used to save voice table, provided by caller.
 * 						Buffer size is larger or equal to voice number times sizeof(voice_info_t).
 * @return uint32_t 0:sucessful, others:error number.
 */
extern uint32_t cmd_file_read_voice_table(void* table_buffer);

/**
 * @brief Get id of model group, that prun of asr is the max.
 * 
 * @return id of model group.
 */
extern uint32_t cmd_file_get_max_model_group_id();

extern uint32_t cmd_file_change_cur_model_group(uint8_t model_group_id);

extern int32_t cmd_file_get_send_data_item_len(void);
extern int32_t cmd_file_get_recieve_data_item_len(void);
extern int32_t cmd_file_read_send_data_table(uint8_t* p_send_data_table, uint8_t* p_item_length);
extern int32_t cmd_file_read_recieve_data_table(uint8_t* p_recieve_data_table, uint8_t* p_item_length);

#ifdef __cplusplus
}
#endif
#endif


