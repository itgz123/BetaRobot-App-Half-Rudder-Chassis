#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

/*============================================
 *            底盘-云台 CAN 通信数据结构
 *   （CAN1 / idseq / PROTO_CUSTOM：payload 即业务结构体，
 *   线上帧 = [0xA5][seq][payload N][CRC8][0x5A]，N = sizeof(结构体)）
 *============================================*/
#pragma pack(push, 1)
/* 云台→底盘（13B）与底盘→云台（12B）各一结构体；两端 app.h 的定义须字节对齐
 * （COMM_DEF 内 _Static_assert 校验 sizeof == 约定线长）。 */
typedef struct
{
    uint8_t enabled; // 0：失能，1：使能
    // 设定速度
    float vx;
    float vy;
    float w;
} gimbal2chassis_data_t;
typedef struct
{
    // 反馈速度
    float vx;
    float vy;
    float w;
} chassis2gimbal_data_t;
#pragma pack(pop)

#endif // !__APP_H
