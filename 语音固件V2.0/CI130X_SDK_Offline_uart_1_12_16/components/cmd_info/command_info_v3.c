/**
 * @file command_info.c
 * @brief 
 * @version 0.1
 * @date 2019-04-30
 * 
 * @copyright Copyright (c) 2019 Chipintelli Technology Co., Ltd.
 * 
 */


#include "stdio.h"
#include "command_info.h"
#include "dichotomy_find.h"
#include "ci_flash_data_info.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"
// #include "asr_api.h"
#include "ci_log.h"
#include "romlib_runtime.h"

#define MAX_COMBINATION_COUNT           16  

#define cmd_info_malloc(x)              pvPortMalloc(x)
#define cmd_info_free(x)                vPortFree(x)


typedef struct cmd_info_variable_st
{
    uint32_t command_number;
    uint32_t special_word_number;
    //uint32_t voice_number;
    command_info_t* p_cmd_table;
    special_wait_count_t* p_special_word_table;
    command_string_index_t* p_cmd_str_index_table;
    char* p_string_table;
    uint8_t* send_data_table;           // 发送数据表
    uint8_t* recieve_data_table;        // 接收数据表
    //voice_info_t* p_voice_info_table;
    uint32_t voice_patition_addr;
    SemaphoreHandle_t semaphore;
    uint8_t send_data_item_len;         // 发送数据表每个表项占用空间
    uint8_t recieve_data_item_len;      // 接收数据表每个表项占用空间
    uint8_t check_number;               // 用于命令词有效性检查
    uint8_t is_changing_model;          // 是否正在切换模型
} cmd_info_variable_t;


cmd_info_variable_t cmd_info_variable = {0};

static uint32_t cmd_info_change_cur_model_group_inner(uint8_t model_group_id);

uint32_t is_valid_cmd_handle(cmd_handle_t cmd_handle)
{
    return ((cmd_handle != INVALID_HANDLE) && ((((command_info_t*)cmd_handle)->flags & 0xC0) == cmd_info_variable.check_number));
}

uint32_t cmd_info_init(uint32_t cmd_file_addr_in_flash, uint32_t voice_patition_addr, uint8_t model_group_id)
{
    uint32_t ret = cmd_file_reader_init(cmd_file_addr_in_flash);
    if (0 == ret)
    {
        cmd_info_variable.voice_patition_addr = voice_patition_addr;
        #if USE_SEPARATE_WAKEUP_EN        
        ret = cmd_info_change_cur_model_group_inner(cmd_file_get_max_model_group_id());
        #else
        ret = cmd_info_change_cur_model_group_inner(DEFAULT_MODEL_GROUP_ID);
        #endif
    }
    cmd_info_variable.semaphore = xSemaphoreCreateMutex();
    //srand(*((volatile uint32_t*)0xE000E018));   //初始化随机种子。导致复位，将其屏蔽
    return ret;
}


static uint32_t cmd_info_change_cur_model_group_inner(uint8_t model_group_id)
{
    uint32_t ret = 1;
    if (0 == cmd_file_change_cur_model_group(model_group_id))
    {
        if (cmd_info_variable.p_cmd_table)
        {
            cmd_info_destroy();
        }

        cmd_info_variable.command_number = cmd_file_get_command_number();
        cmd_info_variable.p_cmd_table = (command_info_t*)cmd_info_malloc(sizeof(command_info_t) * cmd_info_variable.command_number);
        if (!cmd_info_variable.p_cmd_table)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
        cmd_file_read_command_table(cmd_info_variable.p_cmd_table);

        cmd_info_variable.check_number += 0x40;
        for (int i = 0;i < cmd_info_variable.command_number;i++)
        {
            cmd_info_variable.p_cmd_table[i].flags |= cmd_info_variable.check_number;
        }

        cmd_info_variable.p_cmd_str_index_table = (command_string_index_t*)cmd_info_malloc(sizeof(command_string_index_t) * cmd_info_variable.command_number);
        if (!cmd_info_variable.p_cmd_str_index_table)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
        cmd_file_read_string_index_table(cmd_info_variable.p_cmd_str_index_table);

        cmd_info_variable.special_word_number = cmd_file_get_special_word_number();
        if (cmd_info_variable.special_word_number)
        {
            cmd_info_variable.p_special_word_table = (special_wait_count_t*)cmd_info_malloc(sizeof(special_wait_count_t) * cmd_info_variable.special_word_number);
            if (!cmd_info_variable.p_special_word_table)
            {
                ci_logerr(LOG_CMD_INFO, "not enough memory\n");
            }
            cmd_file_read_special_word_table(cmd_info_variable.p_special_word_table);
        }

        int size = cmd_file_get_string_table_size();
        cmd_info_variable.p_string_table = (char*)cmd_info_malloc(size);
        if (!cmd_info_variable.p_string_table)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
        cmd_file_read_string_table(cmd_info_variable.p_string_table);
        size = cmd_file_get_send_data_item_len()*cmd_info_variable.command_number;
        cmd_info_variable.send_data_table = cmd_info_malloc(size);
        if (!cmd_info_variable.send_data_table)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
        cmd_file_read_send_data_table(cmd_info_variable.send_data_table, &cmd_info_variable.send_data_item_len);
        size = cmd_file_get_recieve_data_item_len()* cmd_info_variable.command_number;
        cmd_info_variable.recieve_data_table = cmd_info_malloc(size);
        if (!cmd_info_variable.recieve_data_table)
        {
            ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        }
        cmd_file_read_recieve_data_table(cmd_info_variable.recieve_data_table, &cmd_info_variable.recieve_data_item_len);
    }
    return ret;
}


uint32_t cmd_info_change_cur_model_group(uint8_t model_group_id)
{
    static unsigned char last_mode_group_id = 0;
    uint32_t ret = 1;
    if (last_mode_group_id != model_group_id)
    {
        cmd_info_variable.is_changing_model = 1;
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        last_mode_group_id = model_group_id;

        extern int set_change_model_state(void);
        set_change_model_state();

        cmd_info_change_cur_model_group_inner(model_group_id);
        ci_loginfo(LOG_CMD_INFO, "change asr mode %d\n",model_group_id);

        uint32_t lg_model_addr;
        uint32_t lg_model_size;
        uint32_t ac_model_addr;
        uint32_t ac_model_size;
        get_current_model_addr(&ac_model_addr, &ac_model_size, &lg_model_addr, &lg_model_size);

        asrtop_asr_system_create_model(0x50000000+lg_model_addr, lg_model_size, 0x50000000+ac_model_addr, ac_model_size, 0);
        cmd_info_variable.is_changing_model = 0;
        xSemaphoreGive(cmd_info_variable.semaphore);
        ret = 0;
    }
    else
    {
        ret = 0;
    }
    return ret;
}


uint32_t cmd_info_get_cur_model_id(uint32_t* p_dnn_id, uint32_t* p_asr_id, uint32_t* p_voice_group_id)
{
    return cmd_file_get_cur_model_id(p_dnn_id, p_asr_id);
}

void cmd_info_destroy()
{
    if (cmd_info_variable.p_cmd_table)
    {
        cmd_info_free(cmd_info_variable.p_cmd_table);
        cmd_info_variable.p_cmd_table = NULL;
    }

    if (cmd_info_variable.p_special_word_table)
    {
        cmd_info_free(cmd_info_variable.p_special_word_table);
        cmd_info_variable.p_special_word_table = NULL;
    }

    if (cmd_info_variable.p_cmd_str_index_table)
    {
        cmd_info_free(cmd_info_variable.p_cmd_str_index_table);
        cmd_info_variable.p_cmd_str_index_table = NULL;
    }

    if (cmd_info_variable.p_string_table)
    {
        cmd_info_free(cmd_info_variable.p_string_table);
        cmd_info_variable.p_string_table = NULL;
    }

    if (cmd_info_variable.send_data_table)
    {
        cmd_info_free(cmd_info_variable.send_data_table);
        cmd_info_variable.send_data_table = NULL;
    }

    if (cmd_info_variable.recieve_data_table)
    {
        cmd_info_free(cmd_info_variable.recieve_data_table);
        cmd_info_variable.recieve_data_table = NULL;
    }
}


static int cmd_string_find_callback(void* pValue, int index, void* CallbackPara)
{
    cmd_info_variable_t * p_cmd_info_variable = (cmd_info_variable_t *)CallbackPara;
    char *p_str = p_cmd_info_variable->p_string_table + p_cmd_info_variable->p_cmd_str_index_table[index].command_string_offset;
    int rst = MASK_ROM_LIB_FUNC->newlibcfunc.strcmp_p(pValue, p_str);
    if (rst > 0)
        return 1;
    else if (rst < 0)
        return -1;
    else
        return 0;
}


static int cmd_id_find_callback(void* pValue, int index, void* CallbackPara)
{
    cmd_info_variable_t* p_cmd_info_variable = (cmd_info_variable_t*)CallbackPara;
    uint16_t t_id = p_cmd_info_variable->p_cmd_table[index].command_id;
    int rst = (int)pValue - (int)t_id;
    if (rst > 0)
        return 1;
    else if (rst < 0)
        return -1;
    else
        return 0;
}


cmd_handle_t cmd_info_find_command_by_id(uint16_t cmd_id)
{
    cmd_handle_t ret = INVALID_HANDLE;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        int rst = dichotomy_find((void*)(int)cmd_id, 0, cmd_info_variable.command_number - 1, cmd_id_find_callback, &cmd_info_variable);
        if (rst >= 0)
        {
            ret = (cmd_handle_t) & (cmd_info_variable.p_cmd_table[rst]);
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}


cmd_handle_t cmd_info_find_command_by_string(const char* cmd_string)
{
    cmd_handle_t ret = INVALID_HANDLE;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        int rst = dichotomy_find((void*)cmd_string, 0, cmd_info_variable.command_number - 1, cmd_string_find_callback, &cmd_info_variable);
        if (rst >= 0)
        {
            uint16_t cmd_index = cmd_info_variable.p_cmd_str_index_table[rst].index_of_cmd_table;
            ret = (cmd_handle_t)&cmd_info_variable.p_cmd_table[cmd_index];
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

cmd_handle_t cmd_info_find_command_by_semantic_id(uint32_t semantic_id)
{
    cmd_handle_t ret = INVALID_HANDLE;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        for (int i = 0;i < cmd_info_variable.command_number;i++)
        {
            if (cmd_info_variable.p_cmd_table[i].semantic_id == semantic_id)
            {
                ret = (cmd_handle_t)&cmd_info_variable.p_cmd_table[i];
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

cmd_handle_t cmd_info_find_command_by_recieve_data(uint8_t * data, uint8_t data_length)
{
    cmd_handle_t ret = INVALID_HANDLE;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        for (int i = 0;i < cmd_info_variable.command_number;i++)
        {
            cmd_sr_data_t* p_sr_data = (cmd_sr_data_t*)(cmd_info_variable.recieve_data_table + cmd_info_variable.recieve_data_item_len*i);
            if (p_sr_data->length == data_length)
            {
                if (!memcmp(data, p_sr_data->data, data_length))
                {
                    ret = (cmd_handle_t)&cmd_info_variable.p_cmd_table[i];
                    break;
                }
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint16_t cmd_info_get_command_id(cmd_handle_t cmd_handle)
{
    uint16_t ret = INVALID_SHORT_ID;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            ret = ((command_info_t*)cmd_handle)->command_id;
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

static uint16_t get_command_index(cmd_handle_t cmd_handle)
{
    command_info_t* p = (command_info_t*)cmd_handle;
    return p - cmd_info_variable.p_cmd_table;
}

char* cmd_info_get_command_string(cmd_handle_t cmd_handle)
{
    char *ret = NULL;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            uint16_t cmd_index = get_command_index(cmd_handle);
            for (int i = 0; i < cmd_info_variable.command_number; i++)
            {
                if (cmd_info_variable.p_cmd_str_index_table[i].index_of_cmd_table == cmd_index)
                {
                    ret = cmd_info_variable.p_string_table + cmd_info_variable.p_cmd_str_index_table[i].command_string_offset;
                    break;
                }
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint8_t cmd_info_get_cmd_score(cmd_handle_t cmd_handle)
{
    uint8_t ret = 0xFF;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            ret = ((command_info_t*)cmd_handle)->score;
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint32_t cmd_info_get_semantic_id(cmd_handle_t cmd_handle)
{
    uint32_t ret = INVALID_LONG_ID;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            ret = ((command_info_t*)cmd_handle)->semantic_id;
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint8_t cmd_info_get_cmd_flag(cmd_handle_t cmd_handle)
{
    uint8_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            ret = ((command_info_t*)cmd_handle)->flags;
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}


uint32_t cmd_info_is_special_word(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_SPECIAL_WORD)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint32_t cmd_info_is_auto_play(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_AUTO_PLAY)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;

}

uint32_t cmd_info_is_wakeup_word(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_WAKEUP_WORD)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint32_t cmd_info_is_combo_word(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_COMBO_WORD)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint32_t cmd_info_is_expected_word(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_EXPECTED_WORD)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

uint32_t cmd_info_is_unexpected_word(cmd_handle_t cmd_handle)
{
    uint32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            if (((command_info_t*)cmd_handle)->flags & CMD_FLAG_UNEXPECTED_WORD)
            {
                ret = 1;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}


static int wait_count_find_callback(void* pValue, int index, void* CallbackPara)
{
    cmd_info_variable_t* p_cmd_info_variable = (cmd_info_variable_t*)CallbackPara;
    uint16_t t_id = p_cmd_info_variable->p_special_word_table[index].index_of_cmd_table;
    int rst = (int)pValue - (int)t_id;
    if (rst > 0)
        return 1;
    else if (rst < 0)
        return -1;
    else
        return 0;
}

int32_t cmd_info_get_special_wait_count(cmd_handle_t cmd_handle)
{
    int32_t ret = 0;
    if (!cmd_info_variable.is_changing_model)
    {
        xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
        if (is_valid_cmd_handle(cmd_handle))
        {
            uint16_t cmd_index = get_command_index(cmd_handle);
            int rst = dichotomy_find((void*)cmd_index, 0, (int)(cmd_info_variable.special_word_number - 1), wait_count_find_callback, (void*)&cmd_info_variable);
            if (rst >= 0)
            {
                ret = cmd_info_variable.p_special_word_table[rst].wait_count;
            }
        }
        xSemaphoreGive(cmd_info_variable.semaphore);
    }
    return ret;
}

static uint8_t get_combination_voice_number(voice_info_t* p_voice_info_table, uint16_t start_index, uint16_t end_index)
{
    int number = 1;
    uint16_t group = p_voice_info_table[start_index].voice_group & 0x7FFF;
    for (uint16_t i = start_index+1; i <= end_index; i++)
    {
        if ((p_voice_info_table[i].voice_group & 0x7FFF) == group && \
            !(p_voice_info_table[i].voice_group & 0x8000))
        {
            number++;
        }
        else
        {
            break;
        }
    }
    return number;
}

int32_t cmd_info_get_voice_index(uint16_t start_index, uint16_t end_index, uint8_t select_index, uint16_t *id_buffer, int buffer_length)
{
    xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
    uint32_t voice_number = cmd_file_get_voice_number();
    voice_info_t* p_voice_info_table = (voice_info_t*)cmd_info_malloc(sizeof(voice_info_t) * voice_number);
    if (!p_voice_info_table)
    {
        xSemaphoreGive(cmd_info_variable.semaphore);
        ci_logerr(LOG_CMD_INFO, "not enough memory\n");
        return 0;
    }
    cmd_file_read_voice_table(p_voice_info_table);

    uint16_t group = p_voice_info_table[start_index].voice_group;
    voice_select_type_t select_type = (group & 0x4000) ? VOICE_SELECT_RANDOM : VOICE_SELECT_USER;
    uint8_t option_number = (group >> 7) & 0x007F;

    if (option_number <= 0)
    {
        cmd_info_free(p_voice_info_table);
        xSemaphoreGive(cmd_info_variable.semaphore);
        return 0;
    }

    if (select_index >= 0 && select_index < option_number)
    {
        select_index = select_index;
    }
    else if (select_type == VOICE_SELECT_RANDOM)
    {
        select_index = rand() % option_number;
    }
    else
    {
        select_index = 0;
    }

    uint8_t combination_number;

    // 找到选择的起始项
    for (int i = 0; i < select_index; i++)
    {
        combination_number = get_combination_voice_number(p_voice_info_table, start_index, end_index);
        start_index += combination_number;
    }

    combination_number = get_combination_voice_number(p_voice_info_table, start_index, end_index);
    if (combination_number > 0)
    {
        if (combination_number <= MAX_COMBINATION_COUNT)
        {
            combination_number = combination_number > buffer_length ? buffer_length : combination_number;
            for (int i = 0; i < combination_number; i++)
            {
                id_buffer[i] = p_voice_info_table[start_index + i].voice_id;
            }
            cmd_info_free(p_voice_info_table);
            xSemaphoreGive(cmd_info_variable.semaphore);
            return combination_number;
        }
        else
        {
            mprintf("too many combination voice\n");
        }
    }
    cmd_info_free(p_voice_info_table);
    xSemaphoreGive(cmd_info_variable.semaphore);
    return 0;
}

uint32_t cmd_info_get_voice_index_from_handle(cmd_handle_t cmd_handle, uint16_t* start_index, uint16_t* end_index)
{
    uint32_t ret = 0;
    xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
    if (is_valid_cmd_handle(cmd_handle))
    {
        *start_index = ((command_info_t*)cmd_handle)->voice_start_index;
        *end_index = ((command_info_t*)cmd_handle)->voice_end_index;
    }
    xSemaphoreGive(cmd_info_variable.semaphore);
    return ret;
}


uint16_t cmd_info_get_function_id(cmd_handle_t cmd_handle)
{
    uint16_t ret;
    xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
    if (is_valid_cmd_handle(cmd_handle))
    {
        ret = ((command_info_t*)cmd_handle)->function_id;
    }
    xSemaphoreGive(cmd_info_variable.semaphore);
    return ret;
}


cmd_sr_data_t * cmd_info_get_send_data(cmd_handle_t cmd_handle)
{
    cmd_sr_data_t *ret = NULL;
    xSemaphoreTake(cmd_info_variable.semaphore, portMAX_DELAY);
    if (is_valid_cmd_handle(cmd_handle))
    {
        uint16_t cmd_index = get_command_index(cmd_handle);
        ret = (cmd_sr_data_t*)(cmd_info_variable.send_data_table + cmd_info_variable.send_data_item_len*cmd_index);
    }
    xSemaphoreGive(cmd_info_variable.semaphore);
    return ret;
}



