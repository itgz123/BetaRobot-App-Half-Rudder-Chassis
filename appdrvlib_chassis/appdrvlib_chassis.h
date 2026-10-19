/**
 * @file appdrvlib_chassis.h
 * @brief 半舵底盘**分层力分配**（app 层，不改 lib_chassis 内核）
 *
 * 背景：lib_chassis 的 4 行最小范数把 mz 分给舵轮侧向力，app 再把侧向力
 * 代数地换算成舵角目标。舵角是慢执行器，目标角每拍可以跳几万 °/s，
 * 位置环长期饱和，yaw 环形成极限环；纯横移换向时两轮还可能选到不同的
 * ±180° 等价分支，产生 ±100° 的 toe 和偏航冲击。
 *
 * 本模块把分配拆成两个时间尺度：
 *   - 快层：`(fx, fy)` 直接按平移力方向分配给两个驱动轮；yaw 的**快分量**
 *     用左右驱动差模承担（权限 ∝ |cosψ|）。
 *   - 慢层：yaw 的**慢分量**（低通后的 mz）用舵向反称 toe trim 承担
 *     （权限 ∝ |sinψ|），toe 限幅 + 限速。
 *   - 分支选择：两轮**共用一个 ±180° 分支**（带迟滞），避免一轮正转、
 *     一轮反转造成的大 toe。
 *
 * 坐标/符号与 lib_chassis 一致：x+前、y+左、w+逆时针；
 * 舵角为舵轮侧连续角 (rad)；wheel_torque 为沿目标舵向的带符号轮缘力矩 (N·m)。
 */

#ifndef __APPDRVLIB_CHASSIS_H
#define __APPDRVLIB_CHASSIS_H

#include <stdint.h>

#include "lib_math.h"
#include "lib_chassis_half_rudder.h"

/*============================================
 *              可调参数
 *============================================*/
#ifndef APPDRVLIB_CHASSIS_FORCE_MIN
#define APPDRVLIB_CHASSIS_FORCE_MIN (1.0f) // N，平移力低于此值进入纯偏航/保持模式
#endif
#ifndef APPDRVLIB_CHASSIS_MZ_MIN
#define APPDRVLIB_CHASSIS_MZ_MIN (0.05f) // N·m，偏航力矩低于此值视为静止
#endif
#ifndef APPDRVLIB_CHASSIS_TRIM_LPF_RC
#define APPDRVLIB_CHASSIS_TRIM_LPF_RC (0.20f) // s，mz 慢分量一阶低通
#endif
#ifndef APPDRVLIB_CHASSIS_TRIM_MAX_DEG
#define APPDRVLIB_CHASSIS_TRIM_MAX_DEG (30.0f) // deg，单轮 toe trim 限幅
#endif
#ifndef APPDRVLIB_CHASSIS_TRIM_SLEW_DEG_S
#define APPDRVLIB_CHASSIS_TRIM_SLEW_DEG_S (60.0f) // deg/s，trim 变化率上限
#endif
#ifndef APPDRVLIB_CHASSIS_TRIM_SIN_MIN
#define APPDRVLIB_CHASSIS_TRIM_SIN_MIN (0.30f) // |sinψ| 低于此值不用 toe trim
#endif
#ifndef APPDRVLIB_CHASSIS_FAST_COS_MIN
#define APPDRVLIB_CHASSIS_FAST_COS_MIN (0.30f) // |cosψ| 低于此值驱动差模偏航权限为 0
#endif
#ifndef APPDRVLIB_CHASSIS_FORCE_MAX
#define APPDRVLIB_CHASSIS_FORCE_MAX (100.0f) // N，单轮驱动力限幅（≈5 N·m 轮缘）
#endif
#ifndef APPDRVLIB_CHASSIS_BRANCH_MARGIN_DEG
#define APPDRVLIB_CHASSIS_BRANCH_MARGIN_DEG (10.0f) // deg，±180° 分支切换迟滞
#endif

#define APPDRVLIB_CHASSIS_PI (3.14159265358979323846f)
#define APPDRVLIB_CHASSIS_DEG2RAD(d) ((d) * (APPDRVLIB_CHASSIS_PI / 180.0f))

#define APPDRVLIB_CHASSIS_TRIM_MAX (APPDRVLIB_CHASSIS_DEG2RAD(APPDRVLIB_CHASSIS_TRIM_MAX_DEG))
#define APPDRVLIB_CHASSIS_TRIM_SLEW (APPDRVLIB_CHASSIS_DEG2RAD(APPDRVLIB_CHASSIS_TRIM_SLEW_DEG_S))
#define APPDRVLIB_CHASSIS_BRANCH_MARGIN (APPDRVLIB_CHASSIS_DEG2RAD(APPDRVLIB_CHASSIS_BRANCH_MARGIN_DEG))

/*============================================
 *              类型
 *============================================*/
typedef struct
{
    float pos_y;        // 舵轮回转中心 |y| (m)，半舵几何下左右对称
    float wheel_radius; // 驱动轮半径 (m)
} AppDrvlibChassisCfg_s;

typedef struct
{
    float mz_slow;    // 低通后的偏航力矩 (N·m)，慢层输入
    float trim_angle; // 当前 toe trim (rad，舵轮侧)
    uint8_t branch;   // 共用的 ±180° 分支：0 = ψ，1 = ψ+π
    uint8_t valid;    // 1 = 已有历史状态
} AppDrvlibChassisState_s;

typedef struct
{
    float angle_target[LIB_CHASSIS_HALF_RUDDER_NUM]; // 目标舵角 (rad，舵轮侧连续角)
    float wheel_torque[LIB_CHASSIS_HALF_RUDDER_NUM]; // 沿目标舵向的带符号轮缘力矩 (N·m)
    float wheel_align[LIB_CHASSIS_HALF_RUDDER_NUM];  // cos(目标−实际)，给驱动轮出力投影
    float mz_fast;                                   // 本拍交给驱动差模的偏航力矩 (N·m)
    float mz_slow;                                   // 本拍交给 toe trim 的偏航力矩 (N·m)
    float trim_angle;                                // 本拍 toe trim (rad)
    uint8_t reverse;                                 // 本拍共用分支：0/1
    uint8_t valid;                                   // 1 = 输出有效
} AppDrvlibChassisOut_s;

/*============================================
 *              接口
 *============================================*/
/** @brief 清空历史（离开 RUN / 重新使能时调用） */
void AppDrvlibChassisReset(AppDrvlibChassisState_s *st);

/**
 * @brief 分层力分配
 * @param st          分配器状态（历史）
 * @param cfg         几何配置
 * @param steer_angle 当前舵角（舵轮侧连续角，rad），长度 2
 * @param fx          车体系 x 方向广义力 (N)
 * @param fy          车体系 y 方向广义力 (N)
 * @param mz          车体系偏航力矩 (N·m)
 * @param dt_s        控制周期 (s)
 * @param out         分配结果
 * @retval 0  成功
 * @retval -1 空指针 / 几何非法
 * @note 只做分配，不含任何 PID；调用方负责限幅/下发。
 */
int8_t AppDrvlibChassisAllocate(AppDrvlibChassisState_s *st,
                                const AppDrvlibChassisCfg_s *cfg,
                                const float steer_angle[LIB_CHASSIS_HALF_RUDDER_NUM],
                                float fx,
                                float fy,
                                float mz,
                                float dt_s,
                                AppDrvlibChassisOut_s *out);

#endif // !__APPDRVLIB_CHASSIS_H
