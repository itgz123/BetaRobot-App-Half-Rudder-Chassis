#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2chassis_queue_handle;
extern QueueHandle_t chassis2cmd_queue_handle;

/*============================================
 *              任务间通信
 *============================================*/
typedef struct
{
    uint8_t temp_unused;
} cmd2chassis_data_t;
typedef struct
{
    uint8_t temp_unused;
} chassis2cmd_data_t;

/*============================================
 *            底盘-云台 CAN 通信数据结构
 *   （CAN1 / idseq / PROTO_CUSTOM：payload 即业务结构体，
 *   线上帧 = [0xA5][seq][payload N][CRC8][0x5A]，N = sizeof(结构体)）
 *============================================*/
#pragma pack(push, 1)
/* 收发暂用同一结构体，先放 1B 占位；两端 app.h 的定义须字节对齐（COMM_DEF 内
 * _Static_assert 校验 sizeof == 约定线长）。 */
typedef struct
{
    float placeholder;
} gimbal_chassis_data_t;
typedef struct
{
    float placeholder;
    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
} chassis_gimbal_data_t;
#pragma pack(pop)

#endif // !__APP_H
