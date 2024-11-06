
#include <ry_ble.h>
#include <ry_code.h>
QueueHandle_t BLE_DATA_Queue;
uint8_t my_BLE_data_buffer[200];
uint16_t heart_rate_handle_table[HRS_IDX_NB];

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
        // 15_hugh.广播数据设置完成事件
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGE(RYMCU_TAG, "15_hugh.config advertising data complete");
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            // 开始广播，完成后触发开始广播完成事件-->17_hugh
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGE(RYMCU_TAG, "  15.1_hugh.config advertising data complete and start advertising");
        }

        break;
        // 16_hugh.扫描响应数据设置完成事件
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            // 开始广播开始广播，完成后触发开始广播完成事件-->17_hugh
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGE(RYMCU_TAG, "16_hugh.config scan response data complete and start advertising");
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
        // 17_hugh.处理开始广播事件，完成广播流程
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            ESP_LOGE(RYMCU_TAG, "17_hugh.advertising start successfully and game over!");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_conn_update_params_t conn_params = {0};
    switch (event)
    {
    // 11_hugh.再次处理app id注册事件ESP_GATTS_REG_EVT
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGE(RYMCU_TAG, "11_hugh.app id register done event again");
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME); // 12.1 设置设备名称
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        ESP_LOGE(RYMCU_TAG, "  11.1_hugh.esp_ble_gap_set_device_name");
#ifdef CONFIG_SET_RAW_ADV_DATA
        // 12_hugh.设置广播数据,设置完成触发ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT事件，-->14_hugh
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        ESP_LOGE(RYMCU_TAG, "12_hugh.config advertising data");
        adv_config_done |= ADV_CONFIG_FLAG;
        // 13_hugh.配置扫描响应数据，设置完成触发ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT事件--->15_hugh
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        ESP_LOGE(RYMCU_TAG, "13_hugh.config scan response data");
#else
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT: // 来自外部读事件--------------------------------------------------------------------------------------------
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        // add by hugh--start
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        ESP_LOGE(RYMCU_TAG, "char_uuid= %4x,conn_id=%x", (int)param->read.handle, (int)param->read.conn_id);
        if (param->read.handle == heart_rate_handle_table[IDX_CHAR_VAL_A]) // 来自A
        {
            uint8_t temp[] = "rsp read form A";
            rsp.attr_value.len = sizeof(temp);
            for (size_t i = 0; i < sizeof(temp); i++)
            {
                rsp.attr_value.value[i] = temp[i];
            }
        }
        else if (param->read.handle == heart_rate_handle_table[IDX_CHAR_VAL_B]) // 来自B
        {
            uint8_t temp[] = "rsp read form B";
            rsp.attr_value.len = sizeof(temp);
            for (size_t i = 0; i < sizeof(temp); i++)
            {
                rsp.attr_value.value[i] = temp[i];
            }
        }
        else if (param->read.handle == heart_rate_handle_table[IDX_CHAR_VAL_C]) // 来自C
        {
            uint8_t temp[] = "rsp read form C";
            rsp.attr_value.len = sizeof(temp);
            for (size_t i = 0; i < sizeof(temp); i++)
            {
                rsp.attr_value.value[i] = temp[i];
            }
        }
        else
        {
            uint8_t temp[] = "rsp read form other";
            rsp.attr_value.len = sizeof(temp);
            for (size_t i = 0; i < sizeof(temp); i++)
            {
                rsp.attr_value.value[i] = temp[i];
            }
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        // addd by hugh-end
        break;
    case ESP_GATTS_WRITE_EVT:      // 来自外部的写数据，此处处理写过来的数据-----------------------------------------------------------------------
        if (!param->write.is_prep) // 数据长度小于GATTS_DEMO_CHAR_VAL_LEN_MAX，进入处理
        {
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.

            if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i % 0xff;
                    }
                    // the size of notify_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    // the size of indicate_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(indicate_data), indicate_data, true);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }
            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            // 区分来自谁的数据（当数据长度小于LEN_MAX）--------------------------------------------------------------------------------------
            if (param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_A]) // 来自A
            {
                ESP_LOGI(RYMCU_TAG, "Write form A,len = %d, value:", param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                if (param->write.len == CODE_LENGTH)//长度相等则发送消息
                    xQueueSend(BLE_DATA_Queue, param->write.value, 0);
            }
            else if (param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_B]) // 来自B
            {
                ESP_LOGI(RYMCU_TAG, "Write form B,len = %d, value:", param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            }
            else if (param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_C]) // 来自C
            {
                ESP_LOGI(RYMCU_TAG, "Write form C,len = %d, value:", param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            }
            else
            {
                ESP_LOGI(RYMCU_TAG, "Write form other,len = %d, value:", param->write.len);
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            }
        }
        else // 数据大于GATTS_DEMO_CHAR_VAL_LEN_MAX，进入处理
        {
            /* handle prepare write */
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param); // 把所有数据放入prepare_buf中，再进入ESP_GATTS_EXEC_WRITE_EVT进一步处理
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        // esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, HRS_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
            esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    // 10_hugh.处理app id注册事件-----------------------------------------------------------------------------------------------------------------

    // 10.1初始化接口gatts_if
    if (event == ESP_GATTS_REG_EVT)
    {
        ESP_LOGE(RYMCU_TAG, "10_hugh.app id register done");
        if (param->reg.status == ESP_GATT_OK)
        {
            ESP_LOGE(RYMCU_TAG, "  10.1_hugh.get gatts interface");
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    { // 10.2 再进入循环，进入对应app profile的回调函数
        ESP_LOGE(RYMCU_TAG, "  10.2_hugh.set each app callback gatts_cb");
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
void set_adv_name(void)
{
    // 获取ESP32 UID并转换为字符串------------------------------------------------------------------------------------------------------------------
    char chip_id_str[25];
    uint8_t chip_id[6] = {0};
    esp_efuse_mac_get_default(chip_id); // esp_mac.h
    sprintf(chip_id_str, "%02X%02X%02X%02X%02X%02X", chip_id[0], chip_id[1], chip_id[2], chip_id[3], chip_id[4], chip_id[5]);
    ESP_LOGE(RYMCU_TAG, "chip_id =  %s", chip_id_str);
    // 设置广播数据，并只在广播数据中添加蓝牙名称------------------------------------------------------------------------------------------------------
    // 蓝牙名称由中文+ UID最后四个字符组成，其他参数在广播响应数据中------------------------------------------------------------------------------------
    // 广播数据总共31个字节，去掉前2个字节表示长度及类型，去掉后4个字节的UID，名字最大为25字节，每个汉字占3个字节，汉字最多为8个。---------------------------
    uint8_t length = sizeof(my_name);
    raw_adv_data[0] = length + 1 + 4; // 发送数据总长度：类型+中文名+UID
    raw_adv_data[1] = 0x09;           // 类型为local Name
    for (uint8_t i = 0; i < length; i++)
    {
        raw_adv_data[i + 2] = my_name[i];
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        raw_adv_data[length + 1 + i] = chip_id_str[8 + i];
    }
    // raw_adv_data
    ESP_LOGE(RYMCU_TAG, "length= %d", length);
    ESP_LOGE(RYMCU_TAG, "%s", my_name);
}

void initBLE(void)
{

    //-1_hugh.创建蓝牙数据处理队列
    BLE_DATA_Queue = xQueueCreate(10, sizeof(my_BLE_data_buffer)); // 10条消息，每条消息最多200字节
    if (!BLE_DATA_Queue)
        ESP_LOGE(RYMCU_TAG, "-1_hugh.failed to create BLE_DATA_Queue.");
    else
        ESP_LOGE(RYMCU_TAG, "-1_hugh.Created BLE_DATA_Queue.");
    //-2_hugh.创建演示任务
    // xTaskCreate(task1, "task1", 4096, NULL, 10, &myTask1);
    // xTaskCreate(task2, "task2", 4096, NULL, 10, &myTask2);
    // 0_hugh.设置广播名称
    set_adv_name();
    esp_err_t ret;
    // 1_hugh.flash初始化------------------------------------------------------------------------------------------------------------------------
    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGE(RYMCU_TAG, "1_hugh.nvs_flash_init");
    // 2_hugh.释放经典蓝牙内存--------------------------------------------------------------------------------------------------------------------
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_LOGE(RYMCU_TAG, "2_hugh.esp_bt_controller_mem_release");
    // 3_hugh.初始化蓝牙控制器（默认设置）---------------------------------------------------------------------------------------------------------
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGE(RYMCU_TAG, "3_hugh.esp_bt_controller_init");
    // 4_hugh.使能蓝牙控制器---------------------------------------------------------------------------------------------------------------------
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGE(RYMCU_TAG, "4_hugh.esp_bt_controller_enable");
    // 5_hugh.初始化蓝牙bluedroid----------------------------------------------------------------------------------------------------------------
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGE(RYMCU_TAG, "5_hugh.esp_bluedroid_init");
    // 6_hugh.使能蓝牙bluedroid------------------------------------------------------------------------------------------------------------------
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGE(RYMCU_TAG, "6_hugh.esp_bluedroid_enable");
    // 7_hugh.注册GATT回调-----------------------------------------------------------------------------------------------------------------------
    // gatt回调：处理数据读写通知等回调
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ESP_LOGE(RYMCU_TAG, "7_hugh.esp_ble_gatts_register_callback");
    // 8_hugh.注册gap回调------------------------------------------------------------------------------------------------------------------------
    // gap回调：处理扫描，广播，连接等事件
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ESP_LOGE(RYMCU_TAG, "8_hugh.esp_ble_gap_register_callback");
    // 9_hugh注册APP ID------------------------------------------------------------------------------------------------------------------------
    // 注册完成后，发送ESP_GATTS_REG_EVT事件，进入gatt回调gatts_event_handler进行处理-->9_hugh
    ESP_LOGE(RYMCU_TAG, "9_hugh.esp_ble_gatts_app_register");
    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    // 14_hugh.设置最大数据包--------------------------------------------------------------------------------------------------------------------
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500); // 设置玩成触发ESP_GATTS_MTU_EVT事件-->14.1_hugh
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    ESP_LOGE(RYMCU_TAG, "14_hugh.esp_ble_gatt_set_local_mtu");
}
