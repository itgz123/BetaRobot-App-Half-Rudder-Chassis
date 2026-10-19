/**
 * @file appdrvlib_chassis.c
 * @brief 半舵底盘分层力分配实现（快层平移 + 慢层 toe trim）
 */

#include "appdrvlib_chassis.h"

#include <math.h>
#include <stddef.h>

/*============================================
 *              小工具
 *============================================*/
static float Clampf(float x, float lo, float hi)
{
    if (x > hi)
    {
        return hi;
    }
    if (x < lo)
    {
        return lo;
    }
    return x;
}

static void FillHold(const float steer_angle[LIB_CHASSIS_HALF_RUDDER_NUM],
                     uint8_t branch,
                     AppDrvlibChassisOut_s *out)
{
    for (uint8_t i = 0u; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        out->angle_target[i] = steer_angle[i];
        out->wheel_torque[i] = 0.0f;
        out->wheel_align[i] = 1.0f;
    }
    out->mz_fast = 0.0f;
    out->mz_slow = 0.0f;
    out->trim_angle = 0.0f;
    out->reverse = branch;
    out->valid = 1u;
}

/*============================================
 *              接口
 *============================================*/
void AppDrvlibChassisReset(AppDrvlibChassisState_s *st)
{
    if (st == NULL)
    {
        return;
    }
    *st = (AppDrvlibChassisState_s){0};
}

int8_t AppDrvlibChassisAllocate(AppDrvlibChassisState_s *st,
                                const AppDrvlibChassisCfg_s *cfg,
                                const float steer_angle[LIB_CHASSIS_HALF_RUDDER_NUM],
                                float fx,
                                float fy,
                                float mz,
                                float dt_s,
                                AppDrvlibChassisOut_s *out)
{
    if ((st == NULL) || (cfg == NULL) || (steer_angle == NULL) || (out == NULL))
    {
        return -1;
    }
    if ((cfg->pos_y <= 0.0f) || (cfg->wheel_radius <= 0.0f))
    {
        return -1;
    }

    /* 首次调用：历史清零，避免把上一次使能的 trim 带进来 */
    if (st->valid == 0u)
    {
        st->mz_slow = 0.0f;
        st->trim_angle = 0.0f;
        st->branch = 0u;
        st->valid = 1u;
    }

    const float F = hypotf(fx, fy); // 平移力模长 (N)
    const float mz_abs = fabsf(mz);
    const uint8_t yaw_only = ((F < APPDRVLIB_CHASSIS_FORCE_MIN) && (mz_abs > APPDRVLIB_CHASSIS_MZ_MIN)) ? 1u : 0u;
    const uint8_t idle = ((F < APPDRVLIB_CHASSIS_FORCE_MIN) && (mz_abs <= APPDRVLIB_CHASSIS_MZ_MIN)) ? 1u : 0u;

    /* 力方向单位向量（F 很小时不除，避免 0/0） */
    float cos_psi = 1.0f;
    float sin_psi = 0.0f;
    if (F > 1.0e-6f)
    {
        cos_psi = fx / F;
        sin_psi = fy / F;
    }

    /* 快/慢层权限调度：|cosψ| 越大，驱动差模做偏航越有效；
     * |sinψ| 越大，toe trim 越有效。用平滑权重分给两层，避免在阈值处跳变。
     *   w_fast = clamp((|cosψ|−cos_min)/(1−cos_min), 0, 1)
     * 纯偏航/静止时 w_fast=1（差模模式）、慢分量收回到 0。 */
    const float fast_den = 1.0f - APPDRVLIB_CHASSIS_FAST_COS_MIN;
    const float w_fast = ((yaw_only != 0u) || (idle != 0u))
                             ? 1.0f
                             : Clampf((fabsf(cos_psi) - APPDRVLIB_CHASSIS_FAST_COS_MIN) / fast_den, 0.0f, 1.0f);

    /* ---------------- mz 慢分量（一阶低通） ----------------
     * 只把 (1−w_fast)·mz 交给 trim；纯偏航/静止时目标为 0，避免下次平移带着旧 trim。 */
    const float mz_slow_target = ((yaw_only != 0u) || (idle != 0u)) ? 0.0f : ((1.0f - w_fast) * mz);
    if (dt_s > 0.0f)
    {
        const float alpha = dt_s / (APPDRVLIB_CHASSIS_TRIM_LPF_RC + dt_s);
        st->mz_slow += alpha * (mz_slow_target - st->mz_slow);
    }
    out->mz_slow = st->mz_slow;

    /* ---------------- 静止：保持舵角、不出力 ---------------- */
    if (idle != 0u)
    {
        FillHold(steer_angle, st->branch, out);
        out->mz_slow = st->mz_slow;
        /* 历史 trim 缓慢回中（不参与本拍出力） */
        const float max_step = APPDRVLIB_CHASSIS_TRIM_SLEW * ((dt_s > 0.0f) ? dt_s : 0.0f);
        if (st->trim_angle > max_step)
        {
            st->trim_angle -= max_step;
        }
        else if (st->trim_angle < -max_step)
        {
            st->trim_angle += max_step;
        }
        else
        {
            st->trim_angle = 0.0f;
        }
        out->trim_angle = st->trim_angle;
        return 0;
    }

    /* ---------------- 基础舵向 ----------------
     * 平移：ψ = atan2(fy, fx)；纯偏航：两轮朝前（0°），用差模出力。 */
    const float base = (yaw_only != 0u) ? 0.0f : atan2f(fy, fx);

    /* ---------------- 两轮共用 ±180° 分支（带迟滞） ----------------
     * 每轮独立取最短会在换向时出现"一轮正向、一轮反向"，toe 可达 100°+，
     * 产生偏航冲击。这里用两轮实际角的平均值做统一决策。 */
    const float mid = 0.5f * (steer_angle[LIB_CHASSIS_HALF_RUDDER_L] + steer_angle[LIB_CHASSIS_HALF_RUDDER_R]);
    const float d = Lib_Math_WrapAngleNegPIToPI(base - mid);
    const float abs_d = fabsf(d);
    const float half_pi = 0.5f * APPDRVLIB_CHASSIS_PI;
    if (st->branch == 0u)
    {
        if (abs_d > (half_pi + APPDRVLIB_CHASSIS_BRANCH_MARGIN))
        {
            st->branch = 1u;
        }
    }
    else
    {
        if (abs_d < (half_pi - APPDRVLIB_CHASSIS_BRANCH_MARGIN))
        {
            st->branch = 0u;
        }
    }
    const uint8_t branch = st->branch;
    const float branch_angle = (branch != 0u) ? APPDRVLIB_CHASSIS_PI : 0.0f;
    const float force_sign = (branch != 0u) ? -1.0f : 1.0f;

    /* ---------------- 慢层：toe trim ----------------
     * 小角度下 mz_toe = a·F·sinψ·tan(trim)
     * ⇒ trim = atan(mz_slow / (a·F·sinψ))。
     * 只对慢分量生效；|sinψ| 太小时 toe 没有偏航权限，交给驱动差模。 */
    float trim_target = 0.0f;
    if (yaw_only == 0u)
    {
        if (fabsf(sin_psi) > APPDRVLIB_CHASSIS_TRIM_SIN_MIN)
        {
            const float denom = cfg->pos_y * F * sin_psi;
            if (fabsf(denom) > 1.0e-4f)
            {
                trim_target = atanf(st->mz_slow / denom);
            }
        }
    }
    trim_target = Clampf(trim_target, -APPDRVLIB_CHASSIS_TRIM_MAX, APPDRVLIB_CHASSIS_TRIM_MAX);

    /* trim 限速：换向时 mz_slow 与 ψ 都会变，防止 toe 一步到位 */
    const float trim_step = APPDRVLIB_CHASSIS_TRIM_SLEW * ((dt_s > 0.0f) ? dt_s : 0.0f);
    float dtrim = trim_target - st->trim_angle;
    if (dtrim > trim_step)
    {
        dtrim = trim_step;
    }
    else if (dtrim < -trim_step)
    {
        dtrim = -trim_step;
    }
    st->trim_angle += dtrim;
    out->trim_angle = st->trim_angle;

    /* ---------------- 快层：驱动差模 + 平移 ----------------
     * 纯偏航：f_L = -mz/(2a), f_R = +mz/(2a)，两轮反向出力。
     * 有平移：共同分量承担 (fx, fy)，差模承担剩余 mz：
     *   mz_diff = 2a·Δf·cosψ·cos(trim)
     * 权限不足（|cosψ| 小）时 Δf 置 0，yaw 全部由慢层 trim 承担。 */
    float f_l = 0.0f;
    float f_r = 0.0f;
    if (yaw_only != 0u)
    {
        const float df = mz / (2.0f * cfg->pos_y);
        f_l = -df;
        f_r = df;
        out->mz_fast = mz;
    }
    else
    {
        const float mz_fast = mz - st->mz_slow;
        const float cos_eff = cos_psi * cosf(st->trim_angle);
        float df = 0.0f;
        if (w_fast > 0.0f && fabsf(cos_eff) > 1.0e-3f)
        {
            df = mz_fast / (2.0f * cfg->pos_y * cos_eff);
        }
        /* 共同分量按 1/cos(trim) 补偿 toe 造成的平移力损失 */
        const float f_common = F / (2.0f * cosf(st->trim_angle));
        f_l = f_common - df;
        f_r = f_common + df;
        out->mz_fast = mz_fast;
    }

    /* 单轮驱动力限幅（差模偏航允许一轮反推，故不能按 F 的比例限制） */
    f_l = Clampf(f_l, -APPDRVLIB_CHASSIS_FORCE_MAX, APPDRVLIB_CHASSIS_FORCE_MAX);
    f_r = Clampf(f_r, -APPDRVLIB_CHASSIS_FORCE_MAX, APPDRVLIB_CHASSIS_FORCE_MAX);

    /* ---------------- 输出 ---------------- */
    out->angle_target[LIB_CHASSIS_HALF_RUDDER_L] = base + branch_angle + st->trim_angle;
    out->angle_target[LIB_CHASSIS_HALF_RUDDER_R] = base + branch_angle - st->trim_angle;
    out->wheel_torque[LIB_CHASSIS_HALF_RUDDER_L] = force_sign * f_l * cfg->wheel_radius;
    out->wheel_torque[LIB_CHASSIS_HALF_RUDDER_R] = force_sign * f_r * cfg->wheel_radius;
    for (uint8_t i = 0u; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        const float dphi = Lib_Math_WrapAngleNegPIToPI(out->angle_target[i] - steer_angle[i]);
        out->wheel_align[i] = cosf(dphi);
    }
    out->reverse = branch;
    out->valid = 1u;
    return 0;
}
