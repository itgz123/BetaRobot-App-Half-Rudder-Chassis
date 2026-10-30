#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

/*============================================
 *              内置音乐请求
 *============================================*/
/**
 * @brief 请求播放 / 停止内置音乐（See You Again）
 * @param play 非 0 = 从头播放；0 = 停止音乐（正在播的报警不受影响）
 * @note 只置一个请求位，真正的 BuzzerPlay/BuzzerStopMusic 在 daemon 任务里执行：
 *       drv_buzzer_simple 不加锁，所有蜂鸣器调用必须在同一任务上下文（daemon 任务）。
 */
void AppMusicRequest(uint8_t play);

/*============================================
 *            底盘-云台 CAN 通信数据结构
 *   （CAN1 / idseq / PROTO_EXT：payload 即业务结构体，先过扩展汉明码再算 CRC，
 *   线上帧 = [0xA5][seq][汉明编码区 E][CRC8][0x5A]，N = sizeof(结构体)）
 *============================================*/
#pragma pack(push, 1)
/* 云台→底盘（14B）与底盘→云台（12B）各一结构体；两端 app.h 的定义须字节对齐
 * （COMM_DEF 内 _Static_assert 校验 sizeof == 约定线长）。 */
typedef enum : uint8_t
{
    robot_mode_stop = 0,
    robot_mode_normal = 1,
    robot_mode_gyro = 2,
    robot_mode_hole = 3,
} robot_mode; // 状态机 参考`app/half_rudder_gimbal/README.md`
// 方向定义，vx>0:前;xy>0:左;w>0:俯视逆时针
typedef struct
{
    robot_mode enabled;
    // 设定速度
    float vx;
    float vy;
    float w;
    uint8_t power_boost_enable; // 1 = 在基础功率上限上加一段额外功率（云台侧遥控 ch5）
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
