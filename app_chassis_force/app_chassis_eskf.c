/**
 * @file app_chassis_eskf.c
 * @brief 底盘反馈融合 ESKF：6 态误差 [vx, vy, bax, bay, theta, bg]
 *
 * 名义状态放在 ctx（lib_eskf 不持有名义值），误差态进滤波器。
 * 车体系速度模型（运输定理）：
 *   v̇ = a_b − ω×v,   a_b = acc_lin − b_a
 *   θ̇ = yaw_rate − bg
 * 量测：轮速正解 (vx,vy) 与世界 yaw rate；静止时速度/角速度全零（ZUPT+ZARU）。
 */

#include "app_chassis_eskf.h"

#include <math.h>
#include <stdint.h>

#include "lib_eskf.h"

#if CHASSIS_FEEDBACK_SCHEME == 1

/*============================ 状态索引 ============================*/
enum
{
    ESKF_I_VX = 0,
    ESKF_I_VY,
    ESKF_I_BAX,
    ESKF_I_BAY,
    ESKF_I_YAW,
    ESKF_I_BG,
    ESKF_N,
};

#define ESKF_M 3 /* 速度 x/y + yaw rate */
#define ESKF_L 4 /* acc_lin x/y + gyro_z + yaw_rate */

typedef struct
{
    /* 名义状态 */
    float vx, vy;
    float bax, bay;
    float yaw;
    float bg;

    /* 当前拍的输入快照（量测回调里算 h 用） */
    float yaw_rate_now;

    /* 噪声与门控 */
    float q_scale;
    float r_vel;
    float r_velw;

    /* 时序 / 状态 */
    uint64_t last_wheel_ts;
    uint32_t zupt_ticks;
    uint8_t zupt;
    uint8_t slip;
    uint8_t ever_updated;
    uint32_t hold_ticks;
} ChassisEskfCtx_s;

static ChassisEskfCtx_s s_ctx;

ESKF_INSTANCE_DEF(s_eskf, ESKF_N, ESKF_M, ESKF_L);

/* P0 的初值源：Init 时按对角线填（EskfInit 会拷贝进实例） */
static float s_p0[ESKF_N * ESKF_N];

static void ChassisEskfBuildP0(void)
{
    for (int i = 0; i < ESKF_N * ESKF_N; i++)
    {
        s_p0[i] = 0.0f;
    }
    s_p0[ESKF_I_VX * ESKF_N + ESKF_I_VX] = CHASSIS_ESKF_P0_VEL;
    s_p0[ESKF_I_VY * ESKF_N + ESKF_I_VY] = CHASSIS_ESKF_P0_VEL;
    s_p0[ESKF_I_BAX * ESKF_N + ESKF_I_BAX] = CHASSIS_ESKF_P0_ACC_BIAS;
    s_p0[ESKF_I_BAY * ESKF_N + ESKF_I_BAY] = CHASSIS_ESKF_P0_ACC_BIAS;
    s_p0[ESKF_I_YAW * ESKF_N + ESKF_I_YAW] = CHASSIS_ESKF_P0_YAW;
    s_p0[ESKF_I_BG * ESKF_N + ESKF_I_BG] = CHASSIS_ESKF_P0_GYRO_BIAS;
}

/*============================ 回调 ============================*/

/**
 * @brief 传播名义状态并给出 F/Q
 * @note 两项速度更新都用旧 vx/vy（显式欧拉）
 */
static void ChassisEskfPropagate(void *ctx, const float *u, float dt, float *F, float *Q)
{
    ChassisEskfCtx_s *c = (ChassisEskfCtx_s *)ctx;

    const float w = u[2] - c->bg;
    const float ax = u[0] - c->bax;
    const float ay = u[1] - c->bay;
    const float vx0 = c->vx;
    const float vy0 = c->vy;

    /* 名义状态 */
    c->vx = vx0 + (ax + w * vy0) * dt;
    c->vy = vy0 + (ay - w * vx0) * dt;
    c->yaw = Lib_Math_WrapAngleNegPIToPI(c->yaw + (u[3] - c->bg) * dt);

    /* F = I + A·dt（行主序 6×6） */
    for (int i = 0; i < ESKF_N * ESKF_N; i++)
    {
        F[i] = 0.0f;
    }
    F[ESKF_I_VX * ESKF_N + ESKF_I_VX] = 1.0f;
    F[ESKF_I_VX * ESKF_N + ESKF_I_VY] = w * dt;
    F[ESKF_I_VX * ESKF_N + ESKF_I_BAX] = -dt;
    F[ESKF_I_VX * ESKF_N + ESKF_I_BG] = -vy0 * dt;

    F[ESKF_I_VY * ESKF_N + ESKF_I_VX] = -w * dt;
    F[ESKF_I_VY * ESKF_N + ESKF_I_VY] = 1.0f;
    F[ESKF_I_VY * ESKF_N + ESKF_I_BAY] = -dt;
    F[ESKF_I_VY * ESKF_N + ESKF_I_BG] = vx0 * dt;

    F[ESKF_I_BAX * ESKF_N + ESKF_I_BAX] = 1.0f;
    F[ESKF_I_BAY * ESKF_N + ESKF_I_BAY] = 1.0f;
    F[ESKF_I_YAW * ESKF_N + ESKF_I_YAW] = 1.0f;
    F[ESKF_I_YAW * ESKF_N + ESKF_I_BG] = -dt;
    F[ESKF_I_BG * ESKF_N + ESKF_I_BG] = 1.0f;

    /* Q = diag(q·dt) */
    for (int i = 0; i < ESKF_N * ESKF_N; i++)
    {
        Q[i] = 0.0f;
    }
    const float qv = CHASSIS_ESKF_Q_VEL * c->q_scale * dt;
    const float qa = CHASSIS_ESKF_Q_ACC_BIAS * dt;
    const float qy = CHASSIS_ESKF_Q_YAW * c->q_scale * dt;
    const float qg = CHASSIS_ESKF_Q_GYRO_BIAS * dt;
    Q[ESKF_I_VX * ESKF_N + ESKF_I_VX] = qv;
    Q[ESKF_I_VY * ESKF_N + ESKF_I_VY] = qv;
    Q[ESKF_I_BAX * ESKF_N + ESKF_I_BAX] = qa;
    Q[ESKF_I_BAY * ESKF_N + ESKF_I_BAY] = qa;
    Q[ESKF_I_YAW * ESKF_N + ESKF_I_YAW] = qy;
    Q[ESKF_I_BG * ESKF_N + ESKF_I_BG] = qg;
}

/**
 * @brief 量测公共部分：h = [vx, vy, yaw_rate − bg]，H 行距 n=6
 */
static void ChassisEskfFillH(float *H)
{
    for (int i = 0; i < ESKF_M * ESKF_N; i++)
    {
        H[i] = 0.0f;
    }
    H[0 * ESKF_N + ESKF_I_VX] = 1.0f;
    H[1 * ESKF_N + ESKF_I_VY] = 1.0f;
    H[2 * ESKF_N + ESKF_I_BG] = -1.0f; /* ∂(yaw_rate − bg)/∂bg */
}

/**
 * @brief 轮速正解量测：z = [vx_wheel, vy_wheel, w_wheel]
 */
static void ChassisEskfMeasureWheel(void *ctx, const float *z, int m_now, float *y, float *H, float *R)
{
    ChassisEskfCtx_s *c = (ChassisEskfCtx_s *)ctx;
    (void)m_now;

    y[0] = z[0] - c->vx;
    y[1] = z[1] - c->vy;
    y[2] = z[2] - (c->yaw_rate_now - c->bg);

    ChassisEskfFillH(H);

    R[0] = c->r_vel;
    R[1] = 0.0f;
    R[2] = 0.0f;
    R[3] = 0.0f;
    R[4] = c->r_vel;
    R[5] = 0.0f;
    R[6] = 0.0f;
    R[7] = 0.0f;
    R[8] = c->r_velw;
}

/**
 * @brief 静止量测（ZUPT + ZARU）：z = [0, 0, 0]
 */
static void ChassisEskfMeasureStill(void *ctx, const float *z, int m_now, float *y, float *H, float *R)
{
    ChassisEskfCtx_s *c = (ChassisEskfCtx_s *)ctx;
    (void)m_now;

    y[0] = z[0] - c->vx;
    y[1] = z[1] - c->vy;
    y[2] = z[2] - (c->yaw_rate_now - c->bg);

    ChassisEskfFillH(H);

    R[0] = CHASSIS_ESKF_R_ZUPT;
    R[1] = 0.0f;
    R[2] = 0.0f;
    R[3] = 0.0f;
    R[4] = CHASSIS_ESKF_R_ZUPT;
    R[5] = 0.0f;
    R[6] = 0.0f;
    R[7] = 0.0f;
    R[8] = CHASSIS_ESKF_R_ZARU;
}

/**
 * @brief δx 注入名义状态
 */
static void ChassisEskfInject(void *ctx, const float *delta)
{
    ChassisEskfCtx_s *c = (ChassisEskfCtx_s *)ctx;
    c->vx += delta[ESKF_I_VX];
    c->vy += delta[ESKF_I_VY];
    c->bax += delta[ESKF_I_BAX];
    c->bay += delta[ESKF_I_BAY];
    c->yaw = Lib_Math_WrapAngleNegPIToPI(c->yaw + delta[ESKF_I_YAW]);
    c->bg += delta[ESKF_I_BG];
}

/*============================ 内部工具 ============================*/

/**
 * @brief 速度两维新息平方和（2 自由度 NIS）
 * @note S = P_vv + R·I；det 过小（P 尚未建立）时返回 0，不拒帧
 */
static float ChassisEskfNis(const ChassisEskfCtx_s *c, float zvx, float zvy, float r)
{
    const float s00 = ESKF_P(&s_eskf, ESKF_I_VX, ESKF_I_VX) + r;
    const float s01 = ESKF_P(&s_eskf, ESKF_I_VX, ESKF_I_VY);
    const float s10 = ESKF_P(&s_eskf, ESKF_I_VY, ESKF_I_VX);
    const float s11 = ESKF_P(&s_eskf, ESKF_I_VY, ESKF_I_VY) + r;
    const float det = s00 * s11 - s01 * s10;
    if (det <= 1e-12f)
    {
        return 0.0f;
    }

    const float dx = zvx - c->vx;
    const float dy = zvy - c->vy;
    return (dx * (s11 * dx - s01 * dy) + dy * (s00 * dy - s10 * dx)) / det;
}

/**
 * @brief 静止判据：IMU/轮速都新鲜、轮缘几乎不转、角速度与线性加速度都小
 */
static uint8_t ChassisEskfIsStill(const ChassisEskfCtx_s *c, const ChassisEskfInput_s *in)
{
    if ((in->imu_valid == 0u) || (in->wheel_valid == 0u))
    {
        return 0u;
    }
    if (in->wheel_speed_abs_max > CHASSIS_ESKF_ZUPT_WHEEL_EPS)
    {
        return 0u;
    }
    if (Lib_Math_Fabs(in->yaw_rate - c->bg) > CHASSIS_ESKF_ZUPT_GYRO_EPS)
    {
        return 0u;
    }
    if (Lib_Math_Fabs(in->acc_lin.x) > CHASSIS_ESKF_ZUPT_ACC_EPS)
    {
        return 0u;
    }
    if (Lib_Math_Fabs(in->acc_lin.y) > CHASSIS_ESKF_ZUPT_ACC_EPS)
    {
        return 0u;
    }
    return 1u;
}

/*============================ 对外接口 ============================*/

void ChassisEskfInit(void)
{
    ChassisEskfBuildP0();

    s_ctx = (ChassisEskfCtx_s){0};
    s_ctx.q_scale = 1.0f;
    s_ctx.r_vel = CHASSIS_ESKF_R_WHEEL;
    s_ctx.r_velw = CHASSIS_ESKF_R_WHEEL_W;

    const Eskf_Init_Config_s cfg = {
        .n = ESKF_N,
        .m = ESKF_M,
        .l = ESKF_L,
        .opt = ESKF_OPT_JOSEPH,
        .ctx = &s_ctx,
        .propagate = ChassisEskfPropagate,
        .measure = ChassisEskfMeasureWheel,
        .inject = ChassisEskfInject,
        .reset_jac = NULL,
        .P0 = s_p0,
        .Q = NULL,
        .R = NULL,
    };
    (void)EskfInit(&s_eskf, &cfg);
}

void ChassisEskfReset(void)
{
    s_ctx.vx = 0.0f;
    s_ctx.vy = 0.0f;
    s_ctx.bax = 0.0f;
    s_ctx.bay = 0.0f;
    s_ctx.yaw = 0.0f;
    s_ctx.bg = 0.0f;
    s_ctx.yaw_rate_now = 0.0f;
    s_ctx.last_wheel_ts = 0u;
    s_ctx.zupt_ticks = 0u;
    s_ctx.zupt = 0u;
    s_ctx.slip = 0u;
    s_ctx.ever_updated = 0u;
    s_ctx.hold_ticks = 0u;
    s_ctx.q_scale = 1.0f;
    s_ctx.r_vel = CHASSIS_ESKF_R_WHEEL;
    s_ctx.r_velw = CHASSIS_ESKF_R_WHEEL_W;
    (void)EskfReset(&s_eskf);
}

void ChassisEskfUpdate(const ChassisEskfInput_s *in, ChassisEskfOutput_s *out)
{
    if ((in == NULL) || (out == NULL))
    {
        return;
    }

    ChassisEskfCtx_s *c = &s_ctx;
    c->slip = 0u;

    float dt = in->task_dt_s;
    if (dt > 0.0f)
    {
        if (dt > CHASSIS_ESKF_DT_MAX_S)
        {
            dt = CHASSIS_ESKF_DT_MAX_S;
        }

        /* 1) 静止检测：进入需连续满足；离开时重开 P，恢复运动后首帧不被 NIS 拒绝 */
        const uint8_t still = ChassisEskfIsStill(c, in);
        if (still != 0u)
        {
            if (c->zupt_ticks < UINT32_MAX)
            {
                c->zupt_ticks++;
            }
        }
        else
        {
            c->zupt_ticks = 0u;
        }

        const uint8_t zupt_now = (uint8_t)(c->zupt_ticks >= CHASSIS_ESKF_ZUPT_ENTER_TICKS);
        if ((c->zupt != 0u) && (zupt_now == 0u))
        {
            c->zupt = 0u;
            c->last_wheel_ts = 0u;
            (void)EskfReset(&s_eskf);
        }
        else if ((c->zupt == 0u) && (zupt_now != 0u))
        {
            c->zupt = 1u;
        }

        c->yaw_rate_now = in->yaw_rate;
        c->q_scale = (in->imu_valid != 0u) ? 1.0f : CHASSIS_ESKF_Q_NO_IMU_SCALE;

        /* 2) 预测：IMU 无效时零输入 + 放大 Q（轮速量测仍可用） */
        float u[ESKF_L] = {0.0f, 0.0f, 0.0f, 0.0f};
        if (in->imu_valid != 0u)
        {
            u[0] = in->acc_lin.x;
            u[1] = in->acc_lin.y;
            u[2] = in->gyro_z;
            u[3] = in->yaw_rate;
        }
        if (EskfPredict(&s_eskf, u, dt) != ESKF_OK)
        {
            /* 本拍没有干净的 δ 起点：清脏标志，下一拍重试 */
            (void)EskfInject(&s_eskf);
        }

        /* 3) 量测：每拍至多一次（Predict → Update → Inject） */
        uint8_t updated = 0u;

        if (c->zupt != 0u)
        {
            const float z[ESKF_M] = {0.0f, 0.0f, 0.0f};
            if (EskfUpdateM(&s_eskf, ESKF_M, z, ChassisEskfMeasureStill) == ESKF_OK)
            {
                (void)EskfInject(&s_eskf);
                updated = 1u;
            }
        }
        else if ((in->wheel_valid != 0u) && (in->wheel_timestamp_us != 0u) && (in->wheel_timestamp_us != c->last_wheel_ts))
        {
            /* 同一帧只喂一次：CAN 速率低于任务频率，重复喂会让 P 虚假变小 */
            c->last_wheel_ts = in->wheel_timestamp_us;

            float r_vel = CHASSIS_ESKF_R_WHEEL;
            float r_velw = CHASSIS_ESKF_R_WHEEL_W;
            float dw = 0.0f;

            /* IMU 无效时没有 yaw rate 参考，第三行退化（R 极大 ≈ 跳过） */
            if (in->imu_valid != 0u)
            {
                dw = Lib_Math_Fabs(in->wheel_twist.z - (in->yaw_rate - c->bg));
            }
            else
            {
                r_velw = 1.0e6f;
            }

            const float nis = ChassisEskfNis(c, in->wheel_twist.x, in->wheel_twist.y, r_vel);

            if ((nis > CHASSIS_ESKF_NIS_REJECT) || (dw > CHASSIS_ESKF_SLIP_W_REJECT))
            {
                /* 整帧判为打滑/野值：靠 IMU 惯性滑行 */
                c->slip = 1u;
            }
            else
            {
                if ((nis > CHASSIS_ESKF_NIS_INFLATE) || (dw > CHASSIS_ESKF_SLIP_W_DIFF))
                {
                    r_vel *= CHASSIS_ESKF_R_INFLATE;
                    r_velw *= CHASSIS_ESKF_R_INFLATE;
                    c->slip = 1u;
                }
                c->r_vel = r_vel;
                c->r_velw = r_velw;

                const float z[ESKF_M] = {in->wheel_twist.x, in->wheel_twist.y, in->wheel_twist.z};
                if (EskfUpdateM(&s_eskf, ESKF_M, z, ChassisEskfMeasureWheel) == ESKF_OK)
                {
                    (void)EskfInject(&s_eskf);
                    updated = 1u;
                }
            }
        }

        /* 4) 有效性记账 + 数值兜底 */
        if (updated != 0u)
        {
            c->ever_updated = 1u;
            c->hold_ticks = 0u;
        }
        else if (c->hold_ticks < UINT32_MAX)
        {
            c->hold_ticks++;
        }

        if ((isfinite(c->vx) == 0) || (isfinite(c->vy) == 0) || (isfinite(c->yaw) == 0) || (isfinite(c->bax) == 0) || (isfinite(c->bay) == 0) ||
            (isfinite(c->bg) == 0))
        {
            ChassisEskfReset();
        }
        else
        {
            c->vx = Lib_Math_Clamp(c->vx, -CHASSIS_ESKF_V_MAX, CHASSIS_ESKF_V_MAX);
            c->vy = Lib_Math_Clamp(c->vy, -CHASSIS_ESKF_V_MAX, CHASSIS_ESKF_V_MAX);
        }
    }

    out->velocity.x = c->vx;
    out->velocity.y = c->vy;
    out->velocity.z = 0.0f;
    out->yaw = c->yaw;
    out->gyro_bias = c->bg;
    out->velocity_valid = (uint8_t)((c->ever_updated != 0u) && (c->hold_ticks < CHASSIS_ESKF_VEL_HOLD_TICKS));
    out->zupt = c->zupt;
    out->slip = c->slip;
}

#else /* CHASSIS_FEEDBACK_SCHEME == 0：方案 0 不产生任何代码 */

typedef int ChassisEskfDummy_s; /* 避免空翻译单元（ISO C 要求至少一个声明） */

#endif /* CHASSIS_FEEDBACK_SCHEME */
