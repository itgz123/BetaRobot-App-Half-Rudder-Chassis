/**
 * @file app_chassis_output.h
 * @brief 输出：速度逆解定舵角 + 广义力投影到两个轮向
 *
 * 输入：指令速度、实测舵角/轮速/轮向力矩、广义力 (fx, fy, mw)
 * 输出：目标舵角、沿目标轮向的带符号轮缘力矩、余弦降额系数与 ±180° 分支状态
 *
 * ±180° 分支选择由 CHASSIS_OUTPUT_SCHEME 切换：
 *   0 = 直接最短路径（当前默认）；
 *   1 = 4 组合代价函数（角度误差 + 力矩误差 + 两舵反向惩罚，带迟滞）。
 *
 * 模块只做几何/分配计算，不持有电机实例；舵向位置环与轮向力矩下发仍由 app 完成。
 */

#ifndef __APP_CHASSIS_OUTPUT_H
#define __APP_CHASSIS_OUTPUT_H

#include "app_chassis_force_def.h"
#include "lib_chassis.h"
#include "lib_math.h"

typedef struct
{
    vector3_t twist_cmd;     // 指令速度 (vx [m/s], vy [m/s], w [rad/s])
    LibChassisState_s state; // 实测舵角/轮速（舵轮侧）
    vector3_t wrench;        // 期望广义力 (fx [N], fy [N], mw [N·m])
    float wheel_torque_fb[LIB_CHASSIS_HALF_RUDDER_NUM]; // 实测轮缘力矩 (N·m，app 正方向)
} ChassisOutputInput_s;

typedef struct
{
    ChassisCostSel_s sel;      // 目标舵角/轮向力矩/分支状态
    LibChassisTorque_s torque; // 分配结果（含侧向力，当前仅用于调试观察）
} ChassisOutput_s;

typedef struct
{
    LibChassisInstance_s *chassis; // 半舵运动学内核（正解/逆解/力分配共用）
} ChassisOutputCfg_s;

void ChassisOutputInit(const ChassisOutputCfg_s *cfg);

/* 非 RUN 时清空输出（目标/分支/力矩全 0），下次使能从正向分支重新决策 */
void ChassisOutputReset(ChassisOutput_s *out);

void ChassisOutputUpdate(const ChassisOutputInput_s *in, ChassisOutput_s *out);

#endif // !__APP_CHASSIS_OUTPUT_H
