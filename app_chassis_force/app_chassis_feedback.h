/**
 * @file app_chassis_feedback.h
 * @brief 底盘反馈处理：IMU → 底盘系、杆臂补偿、轮速反馈换算、轮速正解
 *
 * 方案切换见 app_chassis_force_def.h 的 CHASSIS_FEEDBACK_SCHEME：
 *   0 = 独立：IMU 给 yaw rate，轮速正解给 vx/vy（当前实现）
 *   1 = 融合：ESKF 融合 IMU + 轮速（预留）
 *
 * 模块只做数据加工，不持有驱动实例；输入/输出都是 plain struct。
 */

#ifndef __APP_CHASSIS_FEEDBACK_H
#define __APP_CHASSIS_FEEDBACK_H

#include "app_chassis_force_def.h"
#include "lib_chassis.h"
#include "lib_math.h"

/* IMU 原始样本（BMI088 驱动输出，仍是 IMU 系） */
typedef struct
{
    vector3_t gyro; // IMU 系角速度 (rad/s)
    vector3_t acc;  // IMU 系比力 (m/s²)
    float yaw_rate; // world 竖直轴角速度 (rad/s)
    float dt;       // 本拍新样本间隔 (s)，0 = 本拍没有新样本
    uint8_t valid;  // 1 = 本拍姿态可用
} ChassisFeedbackImuInput_s;

/* 驱动轮原始反馈（协议方向，未乘 CHASSIS_WHEEL_x_DIR） */
typedef struct
{
    float speed;        // 电机轴速度 (rad/s)
    float torque;       // 电机轴力矩 (N·m)
    float current;      // 电流 (A)
    int8_t temperature; // 温度 (℃)
    uint8_t error;      // 错误码
} ChassisWheelRaw_s;

typedef struct
{
    ChassisWheelRaw_s wheel[APP_WHEEL_NUM];
} ChassisFeedbackWheelInput_s;

/* 反馈处理结果 */
typedef struct
{
    AppChassisImu_s imu;                     // 底盘系 IMU 观测
    AppWheelFeedback_s wheel[APP_WHEEL_NUM]; // 已换算到 app 正方向
    LibChassisState_s state;                 // 实际舵角/轮速（舵轮侧）
    vector3_t twist;                         // vx/vy 轮速正解，w = IMU 或融合值
    uint8_t valid;                           // 1 = 本拍 IMU 可用（融合方案预留）
} ChassisFeedback_s;

typedef struct
{
    const matrix4_t *imu_to_chassis;
    LibChassisInstance_s *chassis;
    float wheel_dir[APP_WHEEL_NUM]; // 协议方向 → app 正方向
    float steer_gear_ratio_inv;     // 电机侧 rad → 舵轮侧 rad
    float drive_reduction;          // 电机侧 rad/s → 舵轮侧 rad/s
} ChassisFeedbackCfg_s;

void ChassisFeedbackInit(const ChassisFeedbackCfg_s *cfg);
void ChassisFeedbackReset(void);

/* 每拍依次调用：IMU → 轮向反馈 → 正解（正解用 SetRef 刷新后的舵向位置） */
void ChassisFeedbackUpdateImu(const ChassisFeedbackImuInput_s *in, ChassisFeedback_s *out);
void ChassisFeedbackUpdateWheels(const ChassisFeedbackWheelInput_s *in, ChassisFeedback_s *out);
void ChassisFeedbackUpdateKinematics(const float steer_motor_pos[LIB_CHASSIS_HALF_RUDDER_NUM], ChassisFeedback_s *out);

#endif // !__APP_CHASSIS_FEEDBACK_H
