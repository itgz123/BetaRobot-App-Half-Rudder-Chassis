/**
 * @file app_chassis_power.h
 * @brief 驱动轮功率：RLS 在线辨识 k1/k2，输出 P_fb / P_cmd，并提供硬限幅
 *
 * 模型（只有两个 LK MF7015 驱动轮，舵向按 2027 规则不计入功率）：
 *   P = g·(τ_L·ω_L + τ_R·ω_R) + k1·(|ω_L| + |ω_R|) + k2·(τ_L² + τ_R²) + k3
 *
 * 回归量（固定尺度归一化，两个 θ 共用同一参数区间）：
 *   φ0 = (|ω_L| + |ω_R|) / 100
 *   φ1 = (τ_L² + τ_R²)   / 1
 *   y  = (P_meas − g·Στω − k3) / 10
 *   θ0 = k1·10, θ1 = k2·0.1
 *
 * 职责边界：
 *   - ChassisPowerUpdate() 只读控制链数据、只写预测输出（不改任何控制量）；
 *   - ChassisPowerLimitUpdate() 也**不直接写 setref**，只给出 tau_out，
 *     由 app 决定是否写回，保证限幅逻辑与调用顺序显式可见。
 *
 * 限幅口径（v1）：预测总功率超上限时，回馈轮保持指令并把负功率计入预算池，
 * 零指令轮保持 0 并扣除其固定损耗，其余正功率轮按功率占比分预算，
 * 每轮独立解 k2·τ² + g·ω·τ + (k1|ω| + k3/2 − budget) = 0。
 * 上限来源用宏切换（固定 / 规则查表预留），不做能量环、不做反馈修正。
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
     * 1.0 = 不补偿；实测三份日志 g≈0.723（LK Kt/iq 刻度偏大 ~38%），
     * 只补偿功率模型，不动 LK 驱动的 torque_constant（见 def.h 说明）。 */
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
    float cmd_torque_limit_nm;  // τ_cmd 进入 P_cmd 前的钳位；限幅求解后也用它钳位

    /* 功率限幅（v1 固定值硬限幅） */
    float limit_fixed_w;    // 固定上限（CHASSIS_POWER_LIMIT_FIXED_W）
    float limit_margin_w;   // 生效上限先减余量
    float limit_min_w;      // 生效上限下限
    float limit_tau_eps_nm; // |τ_cmd| 小于它按“零指令轮”处理

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

/*============================================
 *              功率限幅（v1 硬限幅）
 *============================================*/

typedef struct
{
    float tau_cmd_nm[APP_WHEEL_NUM];  // 本拍待下发，app 正方向，已含 wheel_align
    float omega_radps[APP_WHEEL_NUM]; // 实测轮速，app 正方向
    float limit_w;                    // >0 运行时覆盖；<=0 用 cfg.limit_fixed_w
    uint8_t enabled;                  // 0 = 直通（非 RUN 由 app 传 0）
} ChassisPowerLimitInput_s;

typedef struct
{
    float limit_eff_w; // 生效上限（已扣余量、已下限）
    float p_before_w;  // ΣP_i(τ_cmd)，含 k3
    float p_after_w;   // ΣP_i(τ_out)
    float p_regen_w;   // 回馈信用（≥0）
    float p_fixed_w;   // 零指令轮固定损耗（≥0）
    float pool_w;      // 驱动轮可分配池

    float tau_out_nm[APP_WHEEL_NUM];
    float budget_w[APP_WHEEL_NUM];
    float scale[APP_WHEEL_NUM]; // τ_out/τ_cmd（零指令轮记 1）
    float scale_min;            // 驱动轮最小值；无驱动轮时 1.0

    uint8_t active; // 1 = 本拍超限并执行了限幅
    uint8_t valid;  // 1 = 输入/参数有效
} ChassisPowerLimitOutput_s;

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

/**
 * @brief 500 Hz 调用（在驱动轮下发之前）：按预测功率裁 τ_cmd
 * @note 纯函数：不写 setref、不改任何控制量，由调用方决定是否写回 tau_out。
 * @note 必须在 ChassisPowerInit 之后调用；未初始化/输入非法/enabled=0 时直通。
 */
void ChassisPowerLimitUpdate(const ChassisPowerLimitInput_s *in, ChassisPowerLimitOutput_s *out);

#endif // !__APP_CHASSIS_POWER_H
