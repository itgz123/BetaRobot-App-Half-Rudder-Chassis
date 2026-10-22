/**
 * @file app_chassis_power.c
 * @brief 驱动轮功率预测（RLS 辨识 k1/k2）+ 硬限幅，见 app_chassis_power.h
 *
 * 预测顺序：
 *   1. 取驱动轮实测 τ/ω、本拍指令力矩、超电实测功率；
 *   2. 用同一低通滤 φ0/φ1/P_mech（与超电内部已低通的 P_meas 同相位）；
 *   3. 每 20ms 且门控全通过时做一次 RLS 更新；
 *   4. 每个 500Hz 周期都用最新 θ 输出 P_fb（实测口径）与 P_cmd（指令口径）。
 *
 * 限幅（ChassisPowerLimitUpdate）：超限时按功率占比给每轮分预算，
 * 每轮独立解二次方程；只输出 tau_out，由 app 决定写回。返回前不改任何 setref。
 */

#include "app_chassis_power.h"

#include <math.h>

#include "lib_math.h"

/* 固定归一化尺度：见头文件映射。 */
#define POWER_SPEED_SCALE 100.0f
#define POWER_TORQUE2_SCALE 1.0f
#define POWER_POWER_SCALE 10.0f

/* 换向检测死区：避免零附近符号抖动反复触发保持窗。 */
#define POWER_REVERSAL_SPEED_RADPS 1.0f
#define POWER_REVERSAL_TORQUE_NM 0.05f

/* 输出保护上限（W）：只影响遥测显示，不改 RLS 状态。 */
#define POWER_OUTPUT_LIMIT_W 1000.0f

/* 限幅参数兜底：RLS 参数下限对应的 k2；配置异常时的默认值。 */
#define POWER_LIMIT_K2_MIN 0.1f
#define POWER_LIMIT_FIXED_FALLBACK_W 70.0f
#define POWER_LIMIT_TAU_MAX_FALLBACK_NM 4.5f

_Static_assert(APP_WHEEL_NUM == 2, "power model assumes exactly 2 drive wheels");

LIB_RLS_INSTANCE_DEF(s_power_rls, 2);

static struct
{
    ChassisPowerCfg_s cfg;
    uint8_t initialized;

    /* 低通状态（物理量：rad/s、(N·m)²、(rad/s)²、W） */
    float sum_abs_omega_lpf;
    float sum_tau2_fb_lpf;
    float sum_omega2_lpf;
    float p_mech_fb_lpf;
    uint8_t lpf_seeded;

    /* 降频与超电帧去重 */
    float period_accum_s;
    uint64_t last_meas_ts_us;

    /* RUN 暖机 */
    uint32_t warmup_ticks_left;

    /* 换向保持 */
    float omega_sign[APP_WHEEL_NUM];
    float cmd_sign[APP_WHEEL_NUM];
    uint8_t omega_sign_valid[APP_WHEEL_NUM];
    uint8_t cmd_sign_valid[APP_WHEEL_NUM];
    uint32_t reversal_ticks_left;

    uint8_t was_run;
} s_power;

/*============================ 私有助手 ============================*/

static float Power_Abs(float v)
{
    return (v >= 0.0f) ? v : -v;
}

/**
 * @brief 从归一化 θ 反算物理 k1/k2（预测与限幅共用同一口径）
 */
static void Power_GetCoeffs(float *k1, float *k2)
{
    *k1 = s_power_rls.theta[0] * (POWER_POWER_SCALE / POWER_SPEED_SCALE);
    *k2 = s_power_rls.theta[1] * (POWER_POWER_SCALE / POWER_TORQUE2_SCALE);
}

static uint8_t Power_IsFinite(float v)
{
    return (uint8_t)(isfinite(v) != 0);
}

static uint8_t Power_FrameFresh(uint64_t ts_us, uint64_t now_us, uint64_t timeout_us)
{
    return (uint8_t)((ts_us != 0u) && (now_us >= ts_us) && ((now_us - ts_us) < timeout_us));
}

/**
 * @brief 输出保护：非有限清零；超出 ±1000W 钳位（只影响显示）
 */
static float Power_Sanitize(float v)
{
    if (Power_IsFinite(v) == 0u)
    {
        return 0.0f;
    }
    return Lib_Math_Clamp(v, -POWER_OUTPUT_LIMIT_W, POWER_OUTPUT_LIMIT_W);
}

/**
 * @brief 清运行时状态（保留配置与已辨识 θ）
 */
static void Power_ResetRuntime(void)
{
    s_power.sum_abs_omega_lpf = 0.0f;
    s_power.sum_tau2_fb_lpf = 0.0f;
    s_power.sum_omega2_lpf = 0.0f;
    s_power.p_mech_fb_lpf = 0.0f;
    s_power.lpf_seeded = 0u;

    s_power.period_accum_s = 0.0f;
    s_power.last_meas_ts_us = 0u;
    s_power.warmup_ticks_left = 0u;
    s_power.reversal_ticks_left = 0u;

    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        s_power.omega_sign[i] = 0.0f;
        s_power.cmd_sign[i] = 0.0f;
        s_power.omega_sign_valid[i] = 0u;
        s_power.cmd_sign_valid[i] = 0u;
    }
}

/**
 * @brief 换向检测：符号变化后进入保持窗，窗内不更新 RLS
 */
static void Power_UpdateReversal(const ChassisPowerInput_s *in)
{
    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        const float w = in->wheel_speed_radps[i];
        if (Power_Abs(w) >= POWER_REVERSAL_SPEED_RADPS)
        {
            const float sign = (w > 0.0f) ? 1.0f : -1.0f;
            if ((s_power.omega_sign_valid[i] != 0u) && (sign != s_power.omega_sign[i]))
            {
                s_power.reversal_ticks_left = s_power.cfg.reversal_holdoff_ticks;
            }
            s_power.omega_sign[i] = sign;
            s_power.omega_sign_valid[i] = 1u;
        }

        const float t = in->wheel_torque_cmd_nm[i];
        if (Power_Abs(t) >= POWER_REVERSAL_TORQUE_NM)
        {
            const float sign = (t > 0.0f) ? 1.0f : -1.0f;
            if ((s_power.cmd_sign_valid[i] != 0u) && (sign != s_power.cmd_sign[i]))
            {
                s_power.reversal_ticks_left = s_power.cfg.reversal_holdoff_ticks;
            }
            s_power.cmd_sign[i] = sign;
            s_power.cmd_sign_valid[i] = 1u;
        }
    }
}

/*============================ 外部接口 ============================*/

int8_t ChassisPowerInit(const ChassisPowerCfg_s *cfg)
{
    if (cfg == NULL)
    {
        return -1;
    }

    s_power.cfg = *cfg;
    s_power.initialized = 0u;

    /* 防御非法配置；RLS 自身参数交给 LibRlsInit 校验 */
    if ((Power_IsFinite(s_power.cfg.lpf_rc_s) == 0u) || (s_power.cfg.lpf_rc_s < 0.0f))
    {
        s_power.cfg.lpf_rc_s = 0.0f;
    }
    if ((Power_IsFinite(s_power.cfg.rls_period_s) == 0u) || (s_power.cfg.rls_period_s <= 0.0f))
    {
        s_power.cfg.rls_period_s = 0.02f;
    }
    if ((Power_IsFinite(s_power.cfg.mech_gain) == 0u) || (s_power.cfg.mech_gain <= 0.0f))
    {
        s_power.cfg.mech_gain = 1.0f;
    }
    if ((Power_IsFinite(s_power.cfg.limit_fixed_w) == 0u) || (s_power.cfg.limit_fixed_w <= 0.0f))
    {
        s_power.cfg.limit_fixed_w = POWER_LIMIT_FIXED_FALLBACK_W;
    }
    if ((Power_IsFinite(s_power.cfg.limit_margin_w) == 0u) || (s_power.cfg.limit_margin_w < 0.0f))
    {
        s_power.cfg.limit_margin_w = 0.0f;
    }
    if ((Power_IsFinite(s_power.cfg.limit_min_w) == 0u) || (s_power.cfg.limit_min_w < 0.0f))
    {
        s_power.cfg.limit_min_w = 0.0f;
    }
    if ((Power_IsFinite(s_power.cfg.limit_tau_eps_nm) == 0u) || (s_power.cfg.limit_tau_eps_nm <= 0.0f))
    {
        s_power.cfg.limit_tau_eps_nm = 0.02f;
    }
    if ((Power_IsFinite(s_power.cfg.cmd_torque_limit_nm) == 0u) || (s_power.cfg.cmd_torque_limit_nm <= 0.0f))
    {
        s_power.cfg.cmd_torque_limit_nm = POWER_LIMIT_TAU_MAX_FALLBACK_NM;
    }

    /* 物理量 → 归一化 θ：θ0 = k1·S_w/S_p，θ1 = k2·S_t/S_p */
    const float theta_init[2] = {
        s_power.cfg.k1_init_nm * (POWER_SPEED_SCALE / POWER_POWER_SCALE),
        s_power.cfg.k2_init_w_per_nm2 * (POWER_TORQUE2_SCALE / POWER_POWER_SCALE),
    };

    const LibRls_Config_s rls_cfg = {
        .lambda = s_power.cfg.lambda,
        .p_init = s_power.cfg.p_init,
        .p_min = s_power.cfg.p_min,
        .p_max = s_power.cfg.p_max,
        .param_min = s_power.cfg.param_min,
        .param_max = s_power.cfg.param_max,
        .innovation_limit = s_power.cfg.innovation_limit_w / POWER_POWER_SCALE,
        .fail_policy = LIB_RLS_FAIL_FREEZE,
        .theta_init = theta_init,
    };

    if (LibRlsInit(&s_power_rls, &rls_cfg) != LIB_RLS_OK)
    {
        return -1;
    }

    s_power.initialized = 1u;
    Power_ResetRuntime();
    s_power.was_run = 0u;
    return 0;
}

void ChassisPowerReset(void)
{
    if (s_power.initialized != 0u)
    {
        (void)LibRlsReset(&s_power_rls);
    }
    Power_ResetRuntime();
    s_power.was_run = 0u;
}

void ChassisPowerUpdate(const ChassisPowerInput_s *in, ChassisPowerOutput_s *out)
{
    if (in == NULL || out == NULL)
    {
        return;
    }

    *out = (ChassisPowerOutput_s){0};
    out->k3_w = s_power.cfg.k3_w;
    out->p_static_w = s_power.cfg.k3_w;

    if (s_power.initialized == 0u)
    {
        out->valid = 0u;
        return;
    }
    out->valid = 1u;

    const ChassisPowerCfg_s *cfg = &s_power.cfg;
    const uint8_t run = (uint8_t)(in->run != 0u);

    /* 本拍预测用的参数（更新成功后下一拍才生效） */
    float k1 = 0.0f;
    float k2 = 0.0f;
    Power_GetCoeffs(&k1, &k2);
    out->k1_nm = k1;
    out->k2_w_per_nm2 = k2;
    out->update_count = s_power_rls.update_count;
    out->rls_status = s_power_rls.status;
    out->frozen = s_power_rls.frozen;
    out->innovation_w = s_power_rls.last_innovation * POWER_POWER_SCALE;

    /* ---- 有限性：坏帧直接跳过，绝不把 NaN 喂给 RLS ---- */
    uint8_t finite = 1u;
    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        finite &= Power_IsFinite(in->wheel_speed_radps[i]);
        finite &= Power_IsFinite(in->wheel_torque_nm[i]);
        finite &= Power_IsFinite(in->wheel_torque_cmd_nm[i]);
    }
    finite &= Power_IsFinite(in->meas_power_w);
    finite &= Power_IsFinite(in->dt_s);
    if (finite == 0u)
    {
        return;
    }
    out->p_meas_w = in->meas_power_w;

    /* ---- RUN 边沿：进入时重播种滤波器并暖机；离开时清运行时状态 ---- */
    if ((run != 0u) && (s_power.was_run == 0u))
    {
        Power_ResetRuntime();
        s_power.warmup_ticks_left = cfg->warmup_ticks;
    }
    else if ((run == 0u) && (s_power.was_run != 0u))
    {
        Power_ResetRuntime();
    }
    s_power.was_run = run;

    /* ---- 原始量 ---- */
    const float w_l = in->wheel_speed_radps[APP_WHEEL_L];
    const float w_r = in->wheel_speed_radps[APP_WHEEL_R];
    const float tau_l = in->wheel_torque_nm[APP_WHEEL_L];
    const float tau_r = in->wheel_torque_nm[APP_WHEEL_R];

    const float sum_abs_omega = Power_Abs(w_l) + Power_Abs(w_r);
    const float sum_tau2_fb = tau_l * tau_l + tau_r * tau_r;
    const float sum_omega2 = w_l * w_l + w_r * w_r;
    const float p_mech_fb = tau_l * w_l + tau_r * w_r;

    /* ---- 低通：φ0/φ1/Σω²/P_mech 共用同一 RC，只在 RUN 内推进 ---- */
    if (s_power.lpf_seeded == 0u)
    {
        s_power.sum_abs_omega_lpf = sum_abs_omega;
        s_power.sum_tau2_fb_lpf = sum_tau2_fb;
        s_power.sum_omega2_lpf = sum_omega2;
        s_power.p_mech_fb_lpf = p_mech_fb * cfg->mech_gain;
        s_power.lpf_seeded = 1u;
    }
    else if ((run != 0u) && (in->dt_s > 0.0f))
    {
        const float alpha = in->dt_s / (cfg->lpf_rc_s + in->dt_s);
        s_power.sum_abs_omega_lpf += (sum_abs_omega - s_power.sum_abs_omega_lpf) * alpha;
        s_power.sum_tau2_fb_lpf += (sum_tau2_fb - s_power.sum_tau2_fb_lpf) * alpha;
        s_power.sum_omega2_lpf += (sum_omega2 - s_power.sum_omega2_lpf) * alpha;
        s_power.p_mech_fb_lpf += (p_mech_fb * cfg->mech_gain - s_power.p_mech_fb_lpf) * alpha;
    }

    /* ---- 预测 ---- */
    const float cmd_limit = (cfg->cmd_torque_limit_nm > 0.0f) ? cfg->cmd_torque_limit_nm : 0.0f;
    const float cmd_l = Lib_Math_Clamp(in->wheel_torque_cmd_nm[APP_WHEEL_L], -cmd_limit, cmd_limit);
    const float cmd_r = Lib_Math_Clamp(in->wheel_torque_cmd_nm[APP_WHEEL_R], -cmd_limit, cmd_limit);

    /* 回归量诊断（与模型计算共用同一批中间量，不额外滤波） */
    out->sum_abs_omega_raw = sum_abs_omega;
    out->sum_tau2_fb_raw = sum_tau2_fb;
    out->sum_omega2_raw = sum_omega2;
    out->p_mech_fb_raw = p_mech_fb;
    out->sum_abs_omega_lpf = s_power.sum_abs_omega_lpf;
    out->sum_tau2_fb_lpf = s_power.sum_tau2_fb_lpf;
    out->sum_omega2_lpf = s_power.sum_omega2_lpf;
    out->p_mech_fb_lpf = s_power.p_mech_fb_lpf;
    out->mech_gain = cfg->mech_gain;

    out->p_mech_fb_w = p_mech_fb * cfg->mech_gain;
    out->p_fric_fb_w = k1 * sum_abs_omega;
    out->p_copper_fb_w = k2 * sum_tau2_fb;
    out->p_fb_raw_w = Power_Sanitize(out->p_mech_fb_w + out->p_fric_fb_w + out->p_copper_fb_w + cfg->k3_w);

    out->p_fb_lpf_w = Power_Sanitize(s_power.p_mech_fb_lpf + k1 * s_power.sum_abs_omega_lpf + k2 * s_power.sum_tau2_fb_lpf + cfg->k3_w);

    out->p_cmd_w = Power_Sanitize(cfg->mech_gain * (cmd_l * w_l + cmd_r * w_r) + k1 * sum_abs_omega + k2 * (cmd_l * cmd_l + cmd_r * cmd_r) + cfg->k3_w);

    /* ---- 门控 ---- */
    uint16_t mask = 0u;
    const uint64_t now_us = in->now_us;

    if (run != 0u)
    {
        mask |= CHASSIS_POWER_GATE_RUN;
    }

    if ((Power_FrameFresh(in->wheel_timestamp_us[APP_WHEEL_L], now_us, cfg->wheel_timeout_us) != 0u) &&
        (Power_FrameFresh(in->wheel_timestamp_us[APP_WHEEL_R], now_us, cfg->wheel_timeout_us) != 0u))
    {
        mask |= CHASSIS_POWER_GATE_WHEEL_FRESH;
    }

    /* fault_level == 0 = SUPER_CAP_YQ2027_FAULT_NONE */
    if ((in->meas_protocol_valid != 0u) && (in->meas_dcdc_enabled != 0u) && (in->meas_fault_level == 0u))
    {
        mask |= CHASSIS_POWER_GATE_SUPERCAP_OK;
    }

    if ((in->meas_timestamp_us != 0u) && (in->meas_timestamp_us != s_power.last_meas_ts_us) &&
        (Power_FrameFresh(in->meas_timestamp_us, now_us, cfg->supercap_timeout_us) != 0u))
    {
        mask |= CHASSIS_POWER_GATE_SUPERCAP_NEW;
    }

    if ((in->meas_power_w >= cfg->meas_min_w) && (in->meas_power_w <= cfg->meas_max_w) && (Power_Abs(in->meas_power_w) > cfg->meas_min_abs_w))
    {
        mask |= CHASSIS_POWER_GATE_POWER_RANGE;
    }

    if (sum_abs_omega >= cfg->motion_min_sum_radps)
    {
        mask |= CHASSIS_POWER_GATE_MOTION;
    }

    if ((Power_Abs(tau_l) <= cfg->torque_gate_max_nm) && (Power_Abs(tau_r) <= cfg->torque_gate_max_nm) && (Power_Abs(cmd_l) <= cfg->torque_gate_max_nm) &&
        (Power_Abs(cmd_r) <= cfg->torque_gate_max_nm))
    {
        mask |= CHASSIS_POWER_GATE_TORQUE;
    }

    if (s_power.reversal_ticks_left == 0u)
    {
        mask |= CHASSIS_POWER_GATE_NO_REVERSAL;
    }

    mask |= CHASSIS_POWER_GATE_FINITE;

    /* 降频：RUN 内累加，饱和在一个周期，避免抖动后补发式连续更新 */
    if ((run != 0u) && (in->dt_s > 0.0f))
    {
        s_power.period_accum_s += in->dt_s;
        if (s_power.period_accum_s > cfg->rls_period_s)
        {
            s_power.period_accum_s = cfg->rls_period_s;
        }
    }

    if (s_power.warmup_ticks_left == 0u)
    {
        mask |= CHASSIS_POWER_GATE_WARMUP;
    }

    if (s_power.period_accum_s >= cfg->rls_period_s)
    {
        mask |= CHASSIS_POWER_GATE_PERIOD;
    }

    /* 换向检测放在门控之后、更新之前：本拍判到翻转就挡住本拍 */
    if (run != 0u)
    {
        Power_UpdateReversal(in);
        if (s_power.reversal_ticks_left != 0u)
        {
            mask &= (uint16_t)~CHASSIS_POWER_GATE_NO_REVERSAL;
        }
    }

    out->gate_mask = mask;

    /* ---- RLS 更新 ---- */
    if ((run != 0u) && (mask == CHASSIS_POWER_GATE_ALL) && (s_power_rls.frozen == 0u) && (cfg->update_enable != 0u))
    {
        const float phi[2] = {
            s_power.sum_abs_omega_lpf / POWER_SPEED_SCALE,
            s_power.sum_tau2_fb_lpf / POWER_TORQUE2_SCALE,
        };
        const float y = (in->meas_power_w - s_power.p_mech_fb_lpf - cfg->k3_w) / POWER_POWER_SCALE;

        const LibRls_Status_e status = LibRlsUpdate(&s_power_rls, phi, y);

        /* 无论成功与否都消费这一帧并清周期，失败按 20ms 节奏重试，不会每拍刷 */
        s_power.last_meas_ts_us = in->meas_timestamp_us;
        s_power.period_accum_s = 0.0f;

        out->updated = (uint8_t)((status == LIB_RLS_OK) ? 1u : 0u);
        out->rls_status = status;
        out->update_count = s_power_rls.update_count;
        out->frozen = s_power_rls.frozen;
        out->innovation_w = s_power_rls.last_innovation * POWER_POWER_SCALE;
    }

    /* 暖机 / 换向保持窗推进（本拍判到的保持窗也会在这里减一） */
    if (run != 0u)
    {
        if (s_power.warmup_ticks_left != 0u)
        {
            s_power.warmup_ticks_left--;
        }
        if (s_power.reversal_ticks_left != 0u)
        {
            s_power.reversal_ticks_left--;
        }
    }
}

/*============================ 功率限幅 ============================*/

/**
 * @brief 单轮二次方程求解：给定功率预算，求同号、最近、不超过指令幅值的力矩
 *
 * a  = k2
 * b  = g·ω
 * c0 = k1·|ω| + k3/N
 * k2·τ² + b·τ + (c0 − budget) = 0
 *
 * Δ<=0 取顶点（预算低于最小可达功率）；Δ>0 只在**与 τ_cmd 同号**的根里取
 * 离 τ_cmd 最近的一个，两个都不同号则输出 0（绝不为限功率而反向出力）。
 */
static float Power_SolveWheelTorque(float tau_cmd, float omega, float budget, float k1, float k2, float k3_each, float g, float tau_max)
{
    const float a = k2;
    const float b = g * omega;
    const float c0 = k1 * Power_Abs(omega) + k3_each;
    const float disc = b * b - 4.0f * a * (c0 - budget);
    float tau;

    if (disc <= 0.0f)
    {
        tau = -b / (2.0f * a);
    }
    else
    {
        const float s = sqrtf(disc);
        const float r_hi = (-b + s) / (2.0f * a);
        const float r_lo = (-b - s) / (2.0f * a);
        const uint8_t hi_ok = (uint8_t)((tau_cmd >= 0.0f) ? (r_hi >= 0.0f) : (r_hi <= 0.0f));
        const uint8_t lo_ok = (uint8_t)((tau_cmd >= 0.0f) ? (r_lo >= 0.0f) : (r_lo <= 0.0f));

        if ((hi_ok != 0u) && (lo_ok != 0u))
        {
            tau = (Power_Abs(r_hi - tau_cmd) <= Power_Abs(r_lo - tau_cmd)) ? r_hi : r_lo;
        }
        else if (hi_ok != 0u)
        {
            tau = r_hi;
        }
        else if (lo_ok != 0u)
        {
            tau = r_lo;
        }
        else
        {
            tau = 0.0f;
        }
    }

    /* 符号保护：限功率不能把驱动方向翻成反向 */
    if (((tau_cmd > 0.0f) && (tau < 0.0f)) || ((tau_cmd < 0.0f) && (tau > 0.0f)))
    {
        tau = 0.0f;
    }
    /* 不放大指令幅值（数值兜底） */
    if (Power_Abs(tau) > Power_Abs(tau_cmd))
    {
        tau = tau_cmd;
    }
    return Lib_Math_Clamp(tau, -tau_max, tau_max);
}

void ChassisPowerLimitUpdate(const ChassisPowerLimitInput_s *in, ChassisPowerLimitOutput_s *out)
{
    if ((in == NULL) || (out == NULL))
    {
        return;
    }

    *out = (ChassisPowerLimitOutput_s){0};
    out->scale_min = 1.0f;
    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        out->tau_out_nm[i] = in->tau_cmd_nm[i];
        out->scale[i] = 1.0f;
    }

    if (s_power.initialized == 0u)
    {
        return;
    }

    float k1 = 0.0f;
    float k2 = 0.0f;
    Power_GetCoeffs(&k1, &k2);
    if ((Power_IsFinite(k1) == 0u) || (Power_IsFinite(k2) == 0u) || (k2 < POWER_LIMIT_K2_MIN))
    {
        /* 模型参数坏了就直通，避免突然削力矩 */
        return;
    }
    if (k1 < 0.0f)
    {
        k1 = 0.0f;
    }

    const ChassisPowerCfg_s *cfg = &s_power.cfg;
    const float g = cfg->mech_gain;
    const float k3_each = cfg->k3_w / (float)APP_WHEEL_NUM;
    const float eps = cfg->limit_tau_eps_nm;
    const float tau_max = cfg->cmd_torque_limit_nm;

    float limit_src = ((Power_IsFinite(in->limit_w) != 0u) && (in->limit_w > 0.0f)) ? in->limit_w : cfg->limit_fixed_w;
    if ((Power_IsFinite(limit_src) == 0u) || (limit_src <= 0.0f))
    {
        return;
    }
    float limit_eff = limit_src - cfg->limit_margin_w;
    if (limit_eff < cfg->limit_min_w)
    {
        limit_eff = cfg->limit_min_w;
    }
    out->limit_eff_w = limit_eff;

    float tau_cmd[APP_WHEEL_NUM];
    float omega[APP_WHEEL_NUM];
    float p[APP_WHEEL_NUM];
    float p_before = 0.0f;

    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        if ((Power_IsFinite(in->tau_cmd_nm[i]) == 0u) || (Power_IsFinite(in->omega_radps[i]) == 0u))
        {
            /* fail-open：有限指令原样保留，非有限指令清 0 */
            for (uint8_t j = 0u; j < (uint8_t)APP_WHEEL_NUM; j++)
            {
                out->tau_out_nm[j] = (Power_IsFinite(in->tau_cmd_nm[j]) != 0u) ? in->tau_cmd_nm[j] : 0.0f;
            }
            return;
        }

        tau_cmd[i] = in->tau_cmd_nm[i];
        omega[i] = in->omega_radps[i];
        p[i] = k2 * tau_cmd[i] * tau_cmd[i] + g * omega[i] * tau_cmd[i] + k1 * Power_Abs(omega[i]) + k3_each;
        p_before += p[i];
    }
    out->p_before_w = p_before;
    out->valid = 1u;

    if ((in->enabled == 0u) || (p_before <= limit_eff))
    {
        out->p_after_w = p_before;
        return;
    }

    /* 分类：零指令轮保持 0 并扣固定损耗；回馈轮保持指令并把负功率加回池；
     * 其余正功率轮按 P_i 占比分配预算。 */
    float pool = limit_eff;
    float drive_sum = 0.0f;
    uint8_t drive[APP_WHEEL_NUM] = {0u};
    uint8_t n_drive = 0u;

    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        if (Power_Abs(tau_cmd[i]) < eps)
        {
            if (p[i] > 0.0f)
            {
                pool -= p[i];
                out->p_fixed_w += p[i];
            }
            out->tau_out_nm[i] = 0.0f;
        }
        else if (p[i] <= 0.0f)
        {
            pool -= p[i]; // p<0：回馈，增加可分配池
            out->p_regen_w += -p[i];
        }
        else
        {
            drive[i] = 1u;
            n_drive++;
            drive_sum += p[i];
        }
    }
    out->pool_w = pool;

    if ((n_drive != 0u) && (drive_sum > 0.0f))
    {
        if (pool <= 0.0f)
        {
            for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
            {
                if (drive[i] != 0u)
                {
                    out->tau_out_nm[i] = 0.0f;
                }
            }
        }
        else
        {
            /* 每轮在“保持指令符号”的可行域内的最小功率点：
             * 顶点同号且不超出指令幅值 ⇒ 顶点；否则 τ=0。
             * 预算分不到这个最小值时，解出的根在可行域外只能输出 0，
             * 反而固定多耗 k1|ω|+k3/N ⇒ 先垫底，再按“高于垫底的部分”分剩余预算。 */
            float p_min[APP_WHEEL_NUM] = {0.0f};
            float min_sum = 0.0f;
            float headroom_sum = 0.0f;

            for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
            {
                if (drive[i] == 0u)
                {
                    continue;
                }
                const float b = g * omega[i];
                const float c0 = k1 * Power_Abs(omega[i]) + k3_each;
                const float vertex = -b / (2.0f * k2);
                const uint8_t vertex_ok =
                    (uint8_t)(((tau_cmd[i] >= 0.0f) ? (vertex >= 0.0f) : (vertex <= 0.0f)) && (Power_Abs(vertex) <= Power_Abs(tau_cmd[i])));
                float pmin = (vertex_ok != 0u) ? (c0 - b * b / (4.0f * k2)) : c0;
                if (pmin > p[i])
                {
                    pmin = p[i];
                }
                p_min[i] = pmin;
                min_sum += pmin;
                headroom_sum += p[i] - pmin;
            }

            for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
            {
                if (drive[i] == 0u)
                {
                    continue;
                }
                float budget;
                if ((min_sum > pool) || (headroom_sum <= 0.0f))
                {
                    /* 预算连各轮最小值都覆盖不了：各自只到最小功率点（尽力而为） */
                    budget = p_min[i];
                }
                else
                {
                    budget = p_min[i] + (pool - min_sum) * (p[i] - p_min[i]) / headroom_sum;
                }
                out->budget_w[i] = budget;
                out->tau_out_nm[i] = Power_SolveWheelTorque(tau_cmd[i], omega[i], budget, k1, k2, k3_each, g, tau_max);
            }
        }
    }

    /* 限幅后功率与缩放诊断 */
    float p_after = 0.0f;
    for (uint8_t i = 0u; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        const float tau = out->tau_out_nm[i];
        p_after += k2 * tau * tau + g * omega[i] * tau + k1 * Power_Abs(omega[i]) + k3_each;

        if ((drive[i] != 0u) && (Power_Abs(tau_cmd[i]) >= eps))
        {
            const float s = out->tau_out_nm[i] / tau_cmd[i];
            out->scale[i] = s;
            if (s < out->scale_min)
            {
                out->scale_min = s;
            }
        }
    }
    out->p_after_w = p_after;
    out->active = 1u;
}
