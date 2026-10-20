/**
 * @file app_chassis_eskf.h
 * @brief 底盘反馈融合：IMU + 轮速正解 的 6 态 ESKF（CHASSIS_FEEDBACK_SCHEME=1）
 *
 * 状态：车体系速度 (vx,vy)、加速度计零偏 (bax,bay)、世界系航向角 theta、
 *       yaw rate 零偏 bg。
 * 预测：IMU 线性加速度（已按姿态去重力）+ 角速度；
 * 量测：轮速正解 (vx,vy,w) 与 IMU 世界 yaw rate 的一致性；
 *       ZUPT/ZARU：静止时速度=0、yaw rate=0。
 *
 * 模块没有时间/驱动依赖：dt、时间戳、有效性全部由调用方传入。
 */

#ifndef __APP_CHASSIS_ESKF_H
#define __APP_CHASSIS_ESKF_H

#include <stdint.h>

#include "app_chassis_force_def.h"
#include "lib_math.h"

typedef struct
{
    float task_dt_s;             // 本拍任务周期 (s)
    vector3_t acc_lin;           // 车体系线性加速度 (m/s²)，已按姿态扣重力
    float gyro_z;                // 车体系 z 轴角速度 (rad/s)，运输定理项用
    float yaw_rate;              // 世界系 yaw 角速度 (rad/s)，IMU 输出
    uint8_t imu_valid;           // 1 = IMU 数据新鲜可用
    uint8_t wheel_valid;         // 1 = 两轮帧新鲜且正解成功
    uint64_t wheel_timestamp_us; // 轮速帧时间戳（取两轮较新者），0 = 无
    vector3_t wheel_twist;       // 轮速正解 (vx,vy,w)
    float wheel_speed_abs_max;   // 轮缘速度绝对值最大者 (m/s)，静止判据用
} ChassisEskfInput_s;

typedef struct
{
    vector3_t velocity;     // 车体系 vx/vy (m/s)
    float yaw;              // 世界系航向角 (rad)，相对上电时刻，无绝对参考
    float gyro_bias;        // yaw rate 残余零偏 (rad/s)
    uint8_t velocity_valid; // 1 = 近期接受过轮速/ZUPT 量测
    uint8_t zupt;           // 1 = ZUPT/ZARU 生效
    uint8_t slip;           // 1 = 本帧轮速被拒或 R 被放大
} ChassisEskfOutput_s;

void ChassisEskfInit(void);
void ChassisEskfReset(void);
void ChassisEskfUpdate(const ChassisEskfInput_s *in, ChassisEskfOutput_s *out);

#endif // !__APP_CHASSIS_ESKF_H
