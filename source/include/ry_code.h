/*
 * BLE
 */
#pragma once

#ifndef __RY_CODE
#define __RY_CODE
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CODE_LENGTH 3

    /* Attributes State Machine */
    enum code_CMD
    {
        CODE_START,

        CRAZY_DOG,
        SHAKE_HAND,
        SIT_DOWN,
        STAND_UP,
        GET_DOWN,
        BIG_SMART,
        MOVE_FORWARD,
        MOVE_BACK,
        TURN_LEFT,
        TURN_LIGHT,
        DOG_QUIT,
        PATH_FINDING,
        SPEED_ADD,
        SPEED_MORE,
        SPEED_FAST,
        SPEED_SUB,
        SPEED_LESS,
        SPEED_SLOW,
        CODE_END
    };

    typedef struct
    {
        uint8_t code_data[CODE_LENGTH];
    } code_data_t;
    static const code_data_t my_code_db[CODE_END] =
        {
            [CODE_START] = {0x55, 0x00, 0xAA},     // 暂时不用
            
            [CRAZY_DOG] = {0x55, 0x01, 0xAA},     // 呼叫潦草小狗
            [SHAKE_HAND] = {0x55, 0x02, 0xAA},   // 握手
            [SIT_DOWN] = {0x55, 0x03, 0xAA},     // 坐下
            [STAND_UP] = {0x55, 0x04, 0xAA},     // 站起来
            [GET_DOWN] = {0x55, 0x05, 0xAA},     // 趴下
            [BIG_SMART] = {0x55, 0x06, 0xAA},     // 上电播报
            [MOVE_FORWARD] = {0x55, 0x07, 0xAA}, // 前进
            [MOVE_BACK] = {0x55, 0x08, 0xAA},    // 后退
            [TURN_LEFT] = {0x55, 0x09, 0xAA},    // 左拐
            [TURN_LIGHT] = {0x55, 0x0A, 0xAA},   // 右拐
            [DOG_QUIT] = {0x55, 0x0B, 0xAA},     // 退出唤醒
            [PATH_FINDING] = {0x55, 0x0C, 0xAA}, // 探路
            [SPEED_ADD] = {0x55, 0x0D, 0xAA},//加速
            [SPEED_MORE] = {0x55, 0x0E, 0xAA},//快点
            [SPEED_FAST] = {0x55, 0x0F, 0xAA},//再快点
            [SPEED_SUB] = {0x55, 0x10, 0xAA},//减速
            [SPEED_LESS] = {0x55, 0x11, 0xAA},//慢点
            [SPEED_SLOW] = {0x55, 0x12, 0xAA},//再慢点

    };

#ifdef __cplusplus
}
#endif

#endif /*__RY_BLE*/