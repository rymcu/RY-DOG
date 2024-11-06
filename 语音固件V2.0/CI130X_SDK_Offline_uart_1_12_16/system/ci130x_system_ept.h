#ifndef CI130X_SYSTEM_EPT_H
#define CI130X_SYSTEM_EPT_H



/**
 * @brief 通信模块划分的枚举类型
 *          一般情况下，只有需要相应mailbox中断服务的模块，才使用一个类型
 * 
 */
typedef enum
{
    decoder_inner_serve_ept_num = 0,//
    // vad_fe_inner_serve_ept_num,
    // dnn_inner_serve_ept_num,
    asr_top_inner_serve_ept_num,
    asr_top_outside_serve_ept_num,
    flash_manage_serve_inner_ept_num,
    flash_manage_serve_outside_ept_num,
    flash_control_serve_inner_ept_num,
    codec_manage_serve_inner_ept_num,
    audio_in_serve_ept_num,
    other_func_inner_serve_ept_num,
    serve_ept_end,
}mailbox_serve_ept_t;


/**
 * @brief 这个枚举类型是每个请求另外一个核处理的函数的编号，同一个模块内部的函数编号唯一即可
 * 
 */
typedef enum
{
    /*decoder模块定义的函数编号*/
    decoder_rpmsg_port_num_start = 0,

    open_asr_detail_result_cal_ept_num, //1

    get_asr_sys_verinfo_cal_ept_num,    //2

    send_startup_msg_to_decoder_cal_ept_num,    //3

    wait_decoder_createmodel_done_cal_ept_num,  //4

    send_create_model_msg_to_decoder_cal_ept_num,   //5

    set_model_parameter_info_cfg_lg_model_info_cal_ept_num, //6

    set_model_parameter_info_cfg_ac_info_cal_ept_num,   //7

    set_model_parameter_info_startup_index_cal_ept_num, //8

    model_parameter_info_clear_cal_ept_num, //9

    dynmic_confidence_config_cal_ept_num,   //10

    dynmic_confidence_en_cfg_cal_ept_num,   //11

    decoder_split_init_cal_ept_num, //12

    get_decoded_slice_nums_cal_ept_num, //13

    wait_decoder_task_vadfe_clear_done_cal_ept_num, //14

    send_vadfe_cleardone_msg_to_decoder_cal_ept_num,    //15

    send_dnn_cleardone_msg_to_decoder_cal_ept_num,  //16

    send_dnn_mtxdone_msg_to_decoder_cal_ept_num,    //17

    get_decode_baseaddr_curframes_cal_ept_num,  //18

    nn_get_decoding_state_cal_ept_num,  //19

    get_decoder_manage_is_decodeing_state_addr_cal_ept_num, //20

    get_g_prun_count_max_addr_cal_ept_num,  //21

    get_decode_cur_asrresult_isgood_ept_num,    //22
    

    decoder_rpmsg_port_num_end,     //23

    /*vad_fe模块定义的函数编号*/
    vad_fe_rpmsg_port_num_start,    //24

    cmvn_update_mean_var_config_cal_ept_num,    //25

    cmvn_update_weight_config_cal_ept_num,          //26

    send_vadcfg_vadend_frames_msg_cal_ept_num,      //27
    jd_asr_rlt_good_ept_num,                        //28

    vad_fe_rpmsg_port_num_end,                      //29


    /*dnn模块定义的函数编号*/
    dnn_rpmsg_port_num_start,                       //30

    get_dnn_version_ept_num,                        //31

    set_g_asrtop_cursegment_decoderinbuf_nums_cal_ept_num,  //32

    get_g_asrtop_cursegment_decoderinbuf_nums_cal_ept_num,  //33

    get_g_prun_count_cal_ept_num,                   //34

    get_g_prun_tab_cal_ept_num,                     //35

    get_dnn_asr_skpnums_cal_ept_num,                //36

    set_g_prun_count_cal_ept_num,                   //37

    set_g_dnn_output2_enable_cal_ept_num,           //38

    get_g_dnn_asr_acmodel_inflash_flag_cal_ept_num, //39

    get_g_prun_count_addr_cal_ept_num,              //40

    set_g_dnn_asr_acmodel_addr_cal_ept_num,         //41

    set_g_dnn_asr_acmodel_inflash_flag_cal_ept_num, //42

    dnn_output2_able_state_cal_ept_num,             //43

    get_dnn_fe_ready_nums_cal_ept_num,              //44

    check_fe_nums_isready_cal_ept_num,              //45

    send_dnn_run_msg_cal_ept_num,                   //46

    send_clear_msg_to_dnn_cal_ept_num,              //47

    wait_dnn_task_clear_done_cal_ept_num,           //48

    get_dnn_outbuf_nums_cal_ept_num,                //49

    get_dnn_each_buf_slice_count_cal_ept_num,       //50

    get_dnn_mtx_slices_cal_ept_num,                 //51

    check_decoder_is_overwritten_cal_ept_num,       //52

    send_lastdnn_run_msg_cal_ept_num,               //53

    dnn_split_init_cal_ept_num,                     //54

    send_suspend_msg_to_dnn_cal_ept_num,            //55

    wait_dnn_suspend_done_cal_ept_num,              //56

    send_config_msg_to_dnn_cal_ept_num,             //57

    wait_dnn_config_done_cal_ept_num,               //58

    send_resume_msg_to_dnn_cal_ept_num,             //59

    send_startup_msg_to_dnn_cal_ept_num,            //60

    get_g_asrtop_dnnoutbuf_start_addr_cal_ept_num,  //61

    get_g_asrtop_dnnoutbuf_size_cal_ept_num,        //62

    set_nn_to_dtw_init_info_ept_num,                //63

    dnn_rpmsg_port_num_end,                         //64

    /*asr_top模块定义的函数编号*/
    asr_top_rpmsg_port_num_start,                   //65

    set_g_asrtop_asrsys_clear_flag_cal_ept_num,     //66

    send_asrsys_clear_msg_cal_ept_num,              //67

    get_g_asrtop_asrsys_clear_flag_cal_ept_num,     //68

    dy_adj_cmpt_callback_cal_ept_num,               //69

    asrtop_asr_system_continue_ept_num,             //70
    asrtop_asr_system_continue_done_isr_ept_num,    //71

    asrtop_asr_system_pause_ept_num,                //72
    asrtop_asr_system_pause_done_isr_ept_num,       //73

    asrtop_asr_system_create_model_ept_num,         //74
    asrtop_asr_system_create_model_done_isr_ept_num,//75

    asr_top_rpmsg_port_num_end,//偶数               //76


    /*flash_manage模块定义的函数编号*/
    flash_manage_nuclear_com_num_start,             //77

    post_read_flash_ept_num,                        //78
    post_read_flash_op_done_ept_num,                //79

    post_erase_flash_ept_num,                       //80
    post_erase_flash_op_done_ept_num,               //81

    post_write_flash_ept_num,                       //82
    post_write_flash_op_done_ept_num,               //83

    post_spic_read_unique_id_ept_num,               //84
    post_spic_read_unique_id_op_done_ept_num,       //85

    is_flash_power_off_ept_num,                     //86

    flash_manage_nuclear_com_num_end,               //87


    /*flash_control模块定义的函数编号*/
    flash_control_nuclear_com_num_start,            //88

    get_current_model_addr_ept_num,                 //89

    is_ci_flash_data_info_inited_ept_num,           //90


    flash_control_nuclear_com_num_end,              //91


    /*codec_manage模块定义的函数编号*/
    codec_manage_nuclear_com_num_start,             //92

    cm_write_codec_ept_num,                         //93
    cm_get_pcm_buffer_ept_num,                      //94
    cm_release_pcm_buffer_ept_num,                  //95
    cm_config_pcm_buffer_ept_num,                   //96
    cm_config_codec_ept_num,                        //97
    cm_start_codec_ept_num,                         //98
    cm_set_codec_mute_ept_num,                      //99
    cm_stop_codec_ept_num,                          //100
    cm_get_codec_empty_buffer_number_ept_num,       //101
    cm_get_codec_busy_buffer_number_ept_num,        //102
    audio_pre_rslt_write_data_ept_num,              //103
    cm_set_codec_alc_ept_num,                       //104
    cm_set_codec_adc_gain_ept_num,                  //105

    cinn_malloc_in_host_sram_ept_num,               //106
    cinn_free_in_host_sram_ept_num,                 //107
    

    codec_manage_nuclear_com_num_end,               //108

    audio_in_nuclear_com_num_start,                 //109
    send_voice_info_to_audio_in_manage_ept_num,     //110
    set_ssp_registe_ept_num,                        //111
    audio_in_nuclear_com_num_end,                   //112


    other_fun_nuclear_com_num_start,                //113
    ciss_init_ept_num,                              //114
    other_fun_nuclear_com_num_end,                  //115
    
    get_g_dnn_stop_flag_cal_ept_num,                //116
}asr_rpmsg_ept_num_t;

#endif /* CI130X_SYSTEM_EPT_H */



