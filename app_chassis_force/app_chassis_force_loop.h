/**
 * @file app_chassis_force_loop.h
 * @brief 闭环：速度误差 → 底盘广义力 (fx, fy, mw)
 *
 * 方案切换见 app_chassis_force_def.h 的 CHASSIS_CLOSED_LOOP_SCHEME：
 *   0 = PID 反馈 + 可配置前馈（当前实现；前馈系数默认全 0）
 *   1 = 反馈 + RLS/功率约束的整体规划（预留）
 *
 * 模块只做控制量计算，不持有电机/通信实例；输入/输出都是 plain struct。
 * 前馈系数通过 cfg 指针引用 app 的 g_chassis_force_cfg，运行期可继续用调试器改。
 */

#ifndef __APP_CHASSIS_FORCE_LOOP_H
#define __APP_CHASSIS_FORCE_LOOP_H

#include "app_chassis_force_def.h"
#include "lib_math.h"

/* 每拍输入：指令速度、实测速度、步长 */
typedef struct
{
    vector3_t cmd;      // 指令速度 (vx [m/s], vy [m/s], w [rad/s])
    vector3_t twist_fb; // 实测速度（同一坐标系/符号约定）
    float dt;           // 步长 (s)
} ChassisForceLoopInput_s;

typedef struct
{
    const ChassisForceCfg_s *force_cfg; // 前馈系数（模块只读）
} ChassisForceLoopCfg_s;

void ChassisForceLoopInit(const ChassisForceLoopCfg_s *cfg);
void ChassisForceLoopReset(void);

/**
 * @brief 每拍更新闭环
 * @param in      指令/反馈/步长
 * @param enabled 1 = RUN，正常计算；0 = 非运行态，清 PID 状态并输出 0
 * @return W = (fx [N], fy [N], mw [N·m])
 * @note 无论 enabled 与否都会记录本拍指令，供下一拍算惯性前馈（与拆分前一致）。
 */
vector3_t ChassisForceLoopUpdate(const ChassisForceLoopInput_s *in, uint8_t enabled);

#endif // !__APP_CHASSIS_FORCE_LOOP_H
