/**
 * @file app_chassis_power.h
 * @brief 驱动轮功率预测：RLS 在线辨识 k1/k2，仅输出 P_fb / P_cmd，不限幅
 *
 * 模型（只有两个 LK MF7015 驱动轮，舵向按 2027 规则不计入功率）：
 *   P = τ_L·ω_L + τ_R·ω_R + k1·(|ω_L| + |ω_R|) + k2·(τ_L² + τ_R²) + k3
 *
 * 回归量（固定尺度归一化，两个 θ 共用同一参数区间）：
 *   φ0 = (|ω_L| + |ω_R|) / 100
 *   φ1 = (τ_L² + τ_R²)   / 1
 *   y  = (P_meas − P_mech − k3) / 10
 *   θ0 = k1·10, θ1 = k2·0.1
 *
 * ⚠ 本模块只读控制链数据、只写自己的输出：
 *   不修改 wheel_torque / setref / 分配 / PID / 超电命令，这一版不做功率限幅。
 *
 * 单位约定：τ 为电机轴 N·m（iq×Kt），ω 为电机轴 rad/s，均为 app 正方向；
 *           P 单位为 W，正=从超电取电，负=回馈。
 */

#ifndef __APP_CHASSIS_POWER_H
#define __APP_CHASSIS_POWER_H

#include <stdint.h>

#include "app_chassis_force_def.h" // APP_WHEEL_NUM
#include "lib_rls.h"

/* 门控掩码：bit = 1 表示该条件通过；全部通过才执行一次 RLS 更新。
 * 仅在 RUN 内累计；非 RUN 时掩码基本为 0，便于日志定位"为什么没更新"。 */
#define CHASSIS_POWER_GATE_RUN (1u << 0)         // state == RUN
#define CHASSIS_POWER_GATE_WHEEL_FRESH (1u << 1) // 两驱动轮反馈新鲜
/* 注：LK 广播反馈的 `echo` 是回显命令字（0xA1/0x9C），不是错误码；
 *     轮向可用性由 timestamp + daemon 判，这里不设 error 门控。 */
#define CHASSIS_POWER_GATE_SUPERCAP_OK (1u << 2)  // 超电协议/DCDC/故障等级正常
#define CHASSIS_POWER_GATE_SUPERCAP_NEW (1u << 3) // 自上次更新后有新的超电帧
#define CHASSIS_POWER_GATE_POWER_RANGE (1u << 4)  // |P_meas| 与编码量程
#define CHASSIS_POWER_GATE_MOTION (1u << 5)       // 轮速和高于死区
#define CHASSIS_POWER_GATE_TORQUE (1u << 6)       // 实测/指令力矩未饱和
#define CHASSIS_POWER_GATE_NO_REVERSAL (1u << 7)  // 不在换向保持窗内
#define CHASSIS_POWER_GATE_FINITE (1u << 8)       // 输入全部有限
#define CHASSIS_POWER_GATE_WARMUP (1u << 9)       // RUN 暖机完成
#define CHASSIS_POWER_GATE_PERIOD (1u << 10)      // 20ms 更新周期到
#define CHASSIS_POWER_GATE_ALL ((uint16_t)0x07FFu)

typedef struct
{
    /* 静态损耗（驱动失能、静止、DCDC 使能时实测 chassis_power_w 均值） */
    float k3_w;

    /* 物理量初值；Init 内部换算成归一化 θ */
    float k1_init_nm;
    float k2_init_w_per_nm2;

    /* 机械项增益：P_mech 用 mech_gain·Στω。
     * 正常为 1.0；若确认 LK 力矩常数刻度有问题，在这里补偿（不动控制链）。
     * 实测（2026-10-22 跑车日志）Στω 系数约 0.738，待第二轮数据确认后再改。 */
    float mech_gain;

    /* lib_rls 配置（归一化 θ 空间） */
    float lambda;
    float p_init;
    float p_min;
    float p_max;
    float param_min;
    float param_max;
    float innovation_limit_w; // 物理 W；模块内除以 10 后传给 lib_rls

    /* 滤波与降频 */
    float lpf_rc_s;     // φ0/φ1/P_mech 共用的低通时间常数
    float rls_period_s; // RLS 更新周期（同时要求超电有新帧）

    /* 门控阈值 */
    float meas_min_abs_w;       // |P_meas| 下限
    float meas_min_w;           // 编码量程下限（避开饱和）
    float meas_max_w;           // 编码量程上限
    float motion_min_sum_radps; // |ω_L| + |ω_R| 下限
    float torque_gate_max_nm;   // 实测/指令力矩上限
    float cmd_torque_limit_nm;  // τ_cmd 进入 P_cmd 前的钳位

    uint64_t wheel_timeout_us;
    uint64_t supercap_timeout_us;
    uint32_t warmup_ticks;           // RUN 上升沿后的暖机拍数
    uint32_t reversal_holdoff_ticks; // 判到换向后的保持拍数

    uint8_t update_enable; // 0 = 只预测不更新（固定参数影子模式）
} ChassisPowerCfg_s;

typedef struct
{
    /* 驱动轮反馈，app 正方向、电机轴 */
    float wheel_speed_radps[APP_WHEEL_NUM];
    float wheel_torque_nm[APP_WHEEL_NUM];
    float wheel_torque_cmd_nm[APP_WHEEL_NUM]; // 本拍下发（已含 wheel_align）
    uint64_t wheel_timestamp_us[APP_WHEEL_NUM];

    /* 超电实测功率 */
    float meas_power_w;
    uint64_t meas_timestamp_us;
    uint8_t meas_protocol_valid;
    uint8_t meas_dcdc_enabled;
    uint8_t meas_fault_level; // SuperCapYq2027FaultLevel_e

    float dt_s;
    uint64_t now_us;
    uint8_t run; // state == CHASSIS_STATE_RUN
} ChassisPowerInput_s;

typedef struct
{
    float p_meas_w;
    float p_fb_raw_w; // 当前实测 τ/ω 的瞬时估计
    float p_fb_lpf_w; // 与 P_meas 同相位口径，用于对账
    float p_cmd_w;    // 本拍指令力矩 + 实测转速，预测下一拍

    float p_mech_fb_w;   // Στ_fb·ω
    float p_fric_fb_w;   // k1·Σ|ω|
    float p_copper_fb_w; // k2·Στ_fb²
    float p_static_w;    // k3

    /* 回归量诊断：供 PC 离线拟合 / |ω| 与 ω² 两种摩擦力模型对比。
     * 低通项是 RLS 实际用的；raw 项用于核对低通重建。 */
    float sum_abs_omega_lpf; // Σ|ω| 低通 (rad/s)
    float sum_tau2_fb_lpf;   // Στ² 低通 ((N·m)²)
    float sum_omega2_lpf;    // Σω² 低通 ((rad/s)²)
    float p_mech_fb_lpf;     // mech_gain·Στω 低通 (W)，y 里减掉的就是它
    float sum_abs_omega_raw; // Σ|ω| 瞬时 (rad/s)
    float sum_tau2_fb_raw;   // Στ² 瞬时 ((N·m)²)
    float sum_omega2_raw;    // Σω² 瞬时 ((rad/s)²)
    float p_mech_fb_raw;     // Στω 瞬时 (W)
    float mech_gain;         // 机械项增益（当前配置值）

    float k1_nm;
    float k2_w_per_nm2;
    float k3_w;

    float innovation_w; // lib_rls 原始新息 × 10（物理 W）
    uint16_t gate_mask;
    uint32_t update_count;

    LibRls_Status_e rls_status;
    uint8_t valid;   // RLS 已初始化，预测可用
    uint8_t frozen;  // RLS 已数值冻结
    uint8_t updated; // 本拍执行了 RLS 更新
} ChassisPowerOutput_s;

/**
 * @brief 初始化（配置 + RLS 初值）
 * @retval 0  成功
 * @retval -1 参数非法 / lib_rls 初始化失败
 */
int8_t ChassisPowerInit(const ChassisPowerCfg_s *cfg);

/**
 * @brief 恢复初值、清滤波/计时/门控状态（保留输入配置）
 */
void ChassisPowerReset(void);

/**
 * @brief 500 Hz 调用：算预测，并按门控决定是否更新 RLS
 * @note 不修改任何控制量；输出只给遥测/调试。
 */
void ChassisPowerUpdate(const ChassisPowerInput_s *in, ChassisPowerOutput_s *out);

#endif // !__APP_CHASSIS_POWER_H
