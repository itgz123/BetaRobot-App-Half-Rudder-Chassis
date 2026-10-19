#include "app_chassis_force.h"
#include "app_cfg.h"
#include "app_chassis_force_def.h" // 类型 + 全局变量（实体在 .def.c）
//
#include "lib_math.h"
#include "bsp_sys_status.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "drvs_djimotor_broadcast.h"
#include "drvs_lkmotor_broadcast.h"
#include "drvlib_bmi088_kalman.h"
#include "drvlib_motor.h"
#include "lib_chassis_half_rudder.h"
#include "lib_pid.h"
#include "appdrvlib_chassis.h"

#include "drv_comm.h"
#include "comm_media_can_idseq.h"
#include "drv_vofa.h"

#include <math.h>
#include <string.h>

/* 换向决策里用 g_wheel_fb[i] 按舵轮序号取轮向反馈：两套轮序必须一致 */
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_L == (int)APP_WHEEL_L, "wheel order mismatch: L");
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_R == (int)APP_WHEEL_R, "wheel order mismatch: R");

/* 日志实例（校准超时/故障上报；LOG_UART 未开时不发射代码） */
LOG_INSTANCE_DEF(g_chassis_log, "chassis", 20);

/*============================ 电机实例 ============================*/
// 舵向：DJI M3508（C620），id2 / id4，共用 0x200 广播帧
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(rudder_motor_group);

// 驱动轮：LK MF7015 直驱，id1 / id2，共用 0x280 转矩广播帧
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);
DRVS_LKMOTOR_BROADCAST_GROUP_DEF(wheel_motor_group);

// 舵向控制核：位置/速度两级级联（drvlib_motor 只算控制量，不碰协议）
DRVLIB_MOTOR_INSTANCE_DEF(rudder_l_ctrl);
DRVLIB_MOTOR_INSTANCE_DEF(rudder_r_ctrl);

// 半舵运动学内核：正解（轮速+舵角 → vx/vy）与后续的力分配共用
static LibChassisInstance_s s_chassis;

#if CHASSIS_LAYERED_ALLOC_ENABLE
/* 分层分配器：状态含 mz 慢分量/toe trim/共用分支；几何在 Init 里填 */
static AppDrvlibChassisState_s s_layered_alloc;
static AppDrvlibChassisCfg_s s_layered_cfg;
#endif

/*============================ 光电门 ============================*/
/* 左舵 PA0 → GPIO_PWM_1，右舵 PA2 → GPIO_PWM_2（DM_MC02 映射，见旧 app 的实机注释）。
 * 回调置 NULL，在控制任务里轮询（不占 EXTI 分发）；取 1→0 下降沿作零点。 */
GPIO_INSTANCE_DEF(rudder_l_gate_io);
GPIO_INSTANCE_DEF(rudder_r_gate_io);

/* 位置反馈是"电机侧 rad"（舵轮 1 圈 = 电机 CHASSIS_STEER_GEAR_RATIO 圈），
 * 机械偏置按舵轮角度标定，换算到电机侧要乘减速比。 */
#define STEER_WHEEL_DEG_TO_MOTOR_RAD(deg) ((deg) * (M_PI / 180.0f) * CHASSIS_STEER_GEAR_RATIO)

/*============================ 云台↔底盘通信 ============================*/
/* 实例、收发数据与类型都在 app_chassis_force_def.h/.c；
 * 这里只保留接收回调（ISR 上下文，必须短小）。 */
static void GimbalRecvOnFrame(const uint8_t *payload)
{
    gimbal2chassis_data_t frame;

    memcpy(&frame, payload, sizeof(frame));
    g_gimbal_mode_rx = frame.enabled;
    g_gimbal2chassis_data = frame;
}

/*============================ 底盘 IMU ============================*/
BMI088_KALMAN_INSTANCE_DEF(bmi088);

/*============================ 杆臂补偿参数 ============================*/
/* a_imu = a_center + α×r + ω×(ω×r)（r = IMU 相对底盘中心的位置）
 * ⇒ a_center = a_imu − α×r − ω×(ω×r)
 * α 由底盘系 gyro 差分得到；差分对噪声敏感，先经一阶低通再乘 r。
 * 安装矩阵/IMU 标定值/观测状态都在 app_chassis_force_def.c。 */
#ifndef CHASSIS_IMU_ALPHA_LPF_RC
#define CHASSIS_IMU_ALPHA_LPF_RC 0.02f // 角加速度低通时间常数 (s)，≈20ms
#endif
#ifndef CHASSIS_IMU_ALPHA_LIMIT
#define CHASSIS_IMU_ALPHA_LIMIT 100.0f // 角加速度限幅 (rad/s²)，防差分尖峰
#endif

/*============================ 云台链路在线 ============================*/
/* 看门狗实例内嵌在 media 基类里，收帧时由 comm 层统一喂狗。
 * daemon_reload 为 0 时后端会把阈值提升为 100ms；未挂 daemon 则按在线处理。 */
static uint8_t GimbalCommIsOnline(void)
{
    const CommMedia *media = (const CommMedia *)gimbal_comm.media; // 首成员即 CommMedia 基类
    return (uint8_t)((media != NULL && media->daemon != NULL) ? DaemonIsOnline(media->daemon) : 1);
}

/*============================ 力控：速度误差 → 广义力 ============================*/
/**
 * @brief 由速度指令与实测速度算出底盘广义力 (fx, fy, mw)
 * @param cmd      云台速度指令（车体系 vx 前 / vy 左 / w 逆时针）
 * @param twist_fb 实测速度：vx/vy 来自轮速正解，w 来自 IMU 底盘系 z 轴角速度
 * @param dt_s     本拍步长 (s)
 * @return W = (fx [N], fy [N], mw [N·m])
 * @note 三个通道的反馈都由 lib_pid 计算；平动为 P，yaw 为 PI（消稳态偏航漂移）。
 * @note 前馈三项（摩擦/惯性/重力）系数当前全 0：代码路径已接通，填数即生效。
 *       - 摩擦：沿**指令**速度符号施加（指令过零不施加，避免静止抖动）；
 *       - 惯性：指令加速度 = 本拍指令 − 上一拍指令 的差分，乘 mass / inertia_z；
 *       - 重力：车体系固定分量，平面行驶为 0，坡道标定后填。
 * @note 指令加速度用纯差分，未做低通；系数为 0 时无影响，正式标定时再看是否需要滤波。
 */
/* 力控反馈 PID 实例（只在 RUN 更新；离开 RUN 由 ChassisForceReset 清状态） */
static PIDInstance s_force_pid_vx;
static PIDInstance s_force_pid_vy;
static PIDInstance s_force_pid_w;
static uint8_t s_force_pid_inited;

static const PID_Init_Config_s s_force_pid_cfg_vx = {
    .kp = CHASSIS_FORCE_VX_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .deadband = CHASSIS_FORCE_XY_DEADBAND,
    .out_lpf_rc = CHASSIS_FORCE_XY_LPF_RC,
    .config_mask = PID_ENABLE_DEADBAND | PID_ENABLE_OUTPUT_FILTER,
};

static const PID_Init_Config_s s_force_pid_cfg_vy = {
    .kp = CHASSIS_FORCE_VY_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .deadband = CHASSIS_FORCE_XY_DEADBAND,
    .out_lpf_rc = CHASSIS_FORCE_XY_LPF_RC,
    .config_mask = PID_ENABLE_DEADBAND | PID_ENABLE_OUTPUT_FILTER,
};

static const PID_Init_Config_s s_force_pid_cfg_w = {
    .kp = CHASSIS_FORCE_W_KP,
    .ki = CHASSIS_FORCE_W_KI,
    .kd = 0.0f,
    .integral_limit = CHASSIS_FORCE_W_INTEGRAL_LIMIT,
    .out_lpf_rc = CHASSIS_FORCE_W_LPF_RC,
    /* 不开 DEADBAND：它会把 i_out 清零（见 def.h 的说明） */
    .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_OUTPUT_FILTER,
};

static void ChassisForceReset(void)
{
    LibPIDReset(&s_force_pid_vx);
    LibPIDReset(&s_force_pid_vy);
    LibPIDReset(&s_force_pid_w);
}

/* 首次进 RUN 时初始化（配置是常量，重复初始化只会白清状态） */
static void ChassisForcePidEnsureInit(void)
{
    if (s_force_pid_inited != 0u)
    {
        return;
    }
    LibPIDInit(&s_force_pid_vx, &s_force_pid_cfg_vx);
    LibPIDInit(&s_force_pid_vy, &s_force_pid_cfg_vy);
    LibPIDInit(&s_force_pid_w, &s_force_pid_cfg_w);
    s_force_pid_inited = 1u;
}

static vector3_t ChassisForceSolve(const gimbal2chassis_data_t *cmd, const vector3_t *twist_fb, float dt_s)
{
    const ChassisForceCfg_s *c = &g_chassis_force_cfg;

    /* 指令加速度（惯性前馈）：dt 非正时置 0，避免除零/首拍尖峰 */
    float a_vx = 0.0f;
    float a_vy = 0.0f;
    float a_w = 0.0f;
    if (dt_s > 0.0f)
    {
        a_vx = (cmd->vx - g_chassis_cmd_prev.x) / dt_s;
        a_vy = (cmd->vy - g_chassis_cmd_prev.y) / dt_s;
        a_w = (cmd->w - g_chassis_cmd_prev.z) / dt_s;
    }

    /* 前馈：摩擦（沿指令方向）+ 惯性 + 重力/坡道 */
    float ff_x = c->mass * a_vx + c->gravity_x;
    float ff_y = c->mass * a_vy + c->gravity_y;
    float ff_w = c->inertia_z * a_w;
    if (cmd->vx > 0.0f)
    {
        ff_x += c->friction_x;
    }
    else if (cmd->vx < 0.0f)
    {
        ff_x -= c->friction_x;
    }
    if (cmd->vy > 0.0f)
    {
        ff_y += c->friction_y;
    }
    else if (cmd->vy < 0.0f)
    {
        ff_y -= c->friction_y;
    }
    if (cmd->w > 0.0f)
    {
        ff_w += c->friction_w;
    }
    else if (cmd->w < 0.0f)
    {
        ff_w -= c->friction_w;
    }

    /* 反馈：三个 lib_pid 统一计算（前馈作为 feedforward 参数传入）。
     * 平动通道的死区/输出低通见 def.h：轮速正解在舵轮偏转/高转速下的噪声远大于
     * 真实平移，不抑制会产生与偏航力同量级的假力。
     * yaw 通道用 PI：纯 P 留下与偏航阻力成正比的稳态误差（实测 w 漂移）。 */
    ChassisForcePidEnsureInit();
    vector3_t wrench;
    wrench.x = LibPIDCalculate(&s_force_pid_vx, cmd->vx, twist_fb->x, ff_x, dt_s);
    wrench.y = LibPIDCalculate(&s_force_pid_vy, cmd->vy, twist_fb->y, ff_y, dt_s);
    wrench.z = LibPIDCalculate(&s_force_pid_w, cmd->w, twist_fb->z, ff_w, dt_s);
    return wrench;
}

/*============================ 舵向换向决策（代价函数） ============================*/
/**
 * @brief 把力分配结果还原成"该轮的期望轮向 + 期望轮缘力矩大小"
 * @param wheel 轮序（0=左舵，1=右舵）
 * @param st    本拍状态（steer_angle 为舵轮侧连续角）
 * @param alloc 力分配输出：力矩沿**当前**舵角 e_r，侧向力沿 e_t
 * @param phi   输出：期望轮向 (rad，车体系，任意区间)
 * @param tau   输出：所需轮缘力矩**大小** (N·m，≥0)
 * @note 当前舵角 α 下接触点合力 F = f_drive·e_r + f_lat·e_t。
 *       把舵轮转到 F 的方向后，同样的 F 只需驱动行承担（侧向力→0），故
 *           φ_F = α + atan2(f_lat, f_drive)
 *           τ_F = |F|·R = √(f_drive² + f_lat²)·R
 *       （f_drive = alloc->wheel_torque / R）。
 * @note φ_F 与 φ_F+180° 配 ±τ_F 是同一组力的两个等价解，选哪个由代价函数定。
 */
#if !CHASSIS_LAYERED_ALLOC_ENABLE
/* 旧路径专用：lib_chassis 分配结果 → 换向决策。分层路径不调用这些函数，
 * 条件编译避免 -Wunused-function；CHASSIS_LAYERED_ALLOC_ENABLE=0 时完整保留。 */
static void ChassisAllocToSteer(uint8_t wheel, const LibChassisState_s *st, const LibChassisTorque_s *alloc, float *phi, float *tau)
{
    const float f_drive = alloc->wheel_torque[wheel] / CHASSIS_WHEEL_RADIUS;
    const float f_lat = alloc->lateral_force[wheel];

    *phi = st->steer_angle[wheel] + atan2f(f_lat, f_drive);
    *tau = sqrtf(f_drive * f_drive + f_lat * f_lat) * CHASSIS_WHEEL_RADIUS;
}

#if CHASSIS_STEER_SELECT_SHORTEST
/**
 * @brief 逐轮最短路径：在 (φ_F, +τ_F) 与 (φ_F+180°, −τ_F) 两个等价解里
 *        各自选**转角更小**的那个
 * @note 逐轮独立决策，不枚举 4 组合、不算力代价、不加 c_diff、无组合迟滞。
 *       两解转角相差不超过 CHASSIS_COST_ANGLE_DEADBAND 时视为平手，保持该轮
 *       上一拍分支，避免目标在 90° 边界逐拍翻转。
 * @note 小力矩规则与代价函数路径保持一致：两轮都小走上面的整体锁定；单轮小
 *       不转舵，直接沿当前轮向输出 alloc 的带符号驱动力矩。
 * @note 本路径不产生组合代价，遥测 ch10~13 恒为 0；ch9 仍按 reverse 位组合。
 */
static void ChassisShortestSelect(const LibChassisState_s *st, const LibChassisTorque_s *alloc, const float *phi_f, const float *tau_f)
{
    ChassisCostSel_s *sel = &g_chassis_cost_sel;

    sel->locked = 0u;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if (tau_f[i] < CHASSIS_COST_TORQUE_LOCK)
        {
            /* 单轮小力矩：不转舵，沿当前轮向出力；reverse 保持上一拍（仅遥测用） */
            sel->angle_target[i] = st->steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->wheel_torque[i] = alloc->wheel_torque[i];
            sel->wheel_align[i] = 1.0f;
            continue;
        }

        const float dphi_fwd = Lib_Math_WrapAngleNegPIToPI(phi_f[i] - st->steer_angle[i]);
        const float dphi_rev = Lib_Math_WrapAngleNegPIToPI(phi_f[i] + M_PI - st->steer_angle[i]);
        const float abs_fwd = fabsf(dphi_fwd);
        const float abs_rev = fabsf(dphi_rev);

        uint8_t rev;
        if (abs_rev < (abs_fwd - CHASSIS_COST_ANGLE_DEADBAND))
        {
            rev = 1u;
        }
        else if (abs_fwd < (abs_rev - CHASSIS_COST_ANGLE_DEADBAND))
        {
            rev = 0u;
        }
        else
        {
            /* 平手：保持上一拍分支；首次决策（valid=0）取正向 */
            rev = (sel->valid != 0u) ? sel->reverse[i] : 0u;
        }

        const float phi_t = phi_f[i] + ((rev != 0u) ? M_PI : 0.0f);
        const float dphi = Lib_Math_WrapAngleNegPIToPI(phi_t - st->steer_angle[i]);

        sel->angle_target[i] = (fabsf(dphi) <= CHASSIS_COST_ANGLE_DEADBAND) ? st->steer_angle[i] : (st->steer_angle[i] + dphi);
        sel->angle_error[i] = fabsf(dphi);
        sel->wheel_torque[i] = (rev != 0u) ? -tau_f[i] : tau_f[i];
        sel->wheel_align[i] = Lib_Math_Cos(dphi);
        sel->reverse[i] = rev;
    }

    sel->combo = (uint8_t)((sel->reverse[LIB_CHASSIS_HALF_RUDDER_L] & 0x1u) | ((sel->reverse[LIB_CHASSIS_HALF_RUDDER_R] & 0x1u) << 1));
    for (uint8_t k = 0u; k < 4u; k++)
    {
        sel->cost[k] = 0.0f; // 本路径不算代价，遥测通道置 0
    }
    sel->valid = 1u;
}
#endif

/**
 * @brief 4 种换向组合的代价评估 + 迟滞/小力矩锁定/角度死区
 * @param st    本拍底盘状态（舵角反馈）
 * @param alloc 本拍力分配结果
 * @note 决策方式由 CHASSIS_STEER_SELECT_SHORTEST 编译期切换：=1 走逐轮最短路径
 *       （ChassisShortestSelect，不做组合代价）；=0 才走下面描述的代价函数。
 * @note 组合枚举（combo 位）：bit0 = 左舵反向(φ+180°, −τ)，bit1 = 右舵反向。
 *       每轮代价 = 角度误差×COST_RUDDER + 力矩误差×COST_WHEEL；
 *       两舵**转动方向**相反（一逆一顺）时再加 CHASSIS_COST_DIFF。
 * @note 迟滞：当前方案（combo）的代价每拍按当前状态重算，新方案必须便宜超过
 *       CHASSIS_COST_SWITCH_MARGIN 才切换；平手保持现方案。
 * @note 小力矩锁定：两轮期望力矩都低于 CHASSIS_COST_TORQUE_LOCK 时不重新决策，
 *       目标角保持当前位置、输出力矩 0 —— 静止时力分配方向由噪声决定，
 *       不锁会让舵向在 ±180° 两解间横跳。
 * @note 单轮小力矩冻结：只有一轮低于阈值时，该轮不转舵、沿当前轮向出力；
 *       力很小的轮子为了对齐方向而大幅转舵只会引入侧滑（见步骤 6 注释）。
 * @note 角度死区：|Δθ| 先扣掉 CHASSIS_COST_ANGLE_DEADBAND 再算代价（舵向小幅
 *       抖动不再影响决策）；同时也用它判定"这次转动属于哪个旋转方向"。
 */
static void ChassisCostSelect(const LibChassisState_s *st, const LibChassisTorque_s *alloc)
{
    ChassisCostSel_s *sel = &g_chassis_cost_sel;

    /* 1. 两轮的期望轮向/力矩大小 */
    float phi_f[LIB_CHASSIS_HALF_RUDDER_NUM];
    float tau_f[LIB_CHASSIS_HALF_RUDDER_NUM];
    uint8_t all_small = 1u;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        ChassisAllocToSteer(i, st, alloc, &phi_f[i], &tau_f[i]);
        if (tau_f[i] >= CHASSIS_COST_TORQUE_LOCK)
        {
            all_small = 0u;
        }
    }

    /* 2. 小力矩锁定：不重新决策，原地保持 */
    if (all_small != 0u)
    {
        sel->locked = 1u;
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->angle_target[i] = st->steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->wheel_torque[i] = 0.0f;
            sel->wheel_align[i] = 1.0f; // 不出力，比例只是占位
            /* 方案本身保持不变（下面的迟滞基准还要用它），只把给定锁在原地 */
            sel->reverse[i] = (uint8_t)((sel->combo >> i) & 0x1u);
        }
        return;
    }
    sel->locked = 0u;

#if CHASSIS_STEER_SELECT_SHORTEST
    ChassisShortestSelect(st, alloc, phi_f, tau_f);
#else
    /* 3. 每轮两种等价解的代价 + 舵轮转动方向（+1 逆时针 / −1 顺时针 / 0 死区内） */
    float cost[LIB_CHASSIS_HALF_RUDDER_NUM][2];
    int8_t rot[LIB_CHASSIS_HALF_RUDDER_NUM][2];
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        /* 实测力矩：电机轴 × 减速比 = 轮缘（MF7015 直驱，系数为 1） */
        const float tau_now = g_wheel_fb[i].torque * CHASSIS_DRIVE_REDUCTION;
        for (uint8_t rev = 0u; rev < 2u; rev++)
        {
            const float phi_t = phi_f[i] + ((rev != 0u) ? M_PI : 0.0f);
            const float tau_t = (rev != 0u) ? -tau_f[i] : tau_f[i];

            /* 目标角与当前角的最短差（wrap 到 (−π, π]），再扣角度死区 */
            const float dphi = Lib_Math_WrapAngleNegPIToPI(phi_t - st->steer_angle[i]);
            float theta_e = fabsf(dphi) - CHASSIS_COST_ANGLE_DEADBAND;
            if (theta_e < 0.0f)
            {
                theta_e = 0.0f;
            }
            const float tau_e = fabsf(tau_t - tau_now);

            cost[i][rev] = theta_e * CHASSIS_COST_RUDDER + tau_e * CHASSIS_COST_WHEEL;

            if (dphi > CHASSIS_COST_ANGLE_DEADBAND)
            {
                rot[i][rev] = 1;
            }
            else if (dphi < -CHASSIS_COST_ANGLE_DEADBAND)
            {
                rot[i][rev] = -1;
            }
            else
            {
                rot[i][rev] = 0;
            }
        }
    }

    /* 4. 四种组合的总代价 */
    uint8_t best = 0u;
    for (uint8_t combo = 0u; combo < 4u; combo++)
    {
        const uint8_t bl = (uint8_t)(combo & 0x1u);        // bit0 = 左舵反向
        const uint8_t br = (uint8_t)((combo >> 1) & 0x1u); // bit1 = 右舵反向

        float c = cost[LIB_CHASSIS_HALF_RUDDER_L][bl] + cost[LIB_CHASSIS_HALF_RUDDER_R][br];
        if ((rot[LIB_CHASSIS_HALF_RUDDER_L][bl] != 0) && (rot[LIB_CHASSIS_HALF_RUDDER_R][br] != 0) &&
            (rot[LIB_CHASSIS_HALF_RUDDER_L][bl] != rot[LIB_CHASSIS_HALF_RUDDER_R][br]))
        {
            c += CHASSIS_COST_DIFF; // 两舵轮转向相反
        }
        sel->cost[combo] = c;

        if (c < sel->cost[best])
        {
            best = combo;
        }
    }

    /* 5. 迟滞：当前执行方案的代价按本拍状态重算，新方案须便宜超过
     *    CHASSIS_COST_SWITCH_MARGIN 才切换；平手保持现方案。
     *    舵轮从一个等价解转到另一个要几百毫秒，期间两解在 |Δθ|≈90° 附近的角度
     *    代价几乎相等，实测力矩/目标方向噪声会让方案在 ±90° 边界来回翻
     *    （2026-10-18 日志：67 次切换、36% 间隔 <100ms；51% 的弱出力帧落在切换后
     *    300ms 内；弱出力时 |w| p95 6.15 vs 对齐时 2.68）。迟滞裕量本身就承担了
     *    这个消抖作用，另加固定 150ms 保持会在需要换向时把舵轮按在原方向。 */
    if (sel->valid == 0u)
    {
        sel->combo = best;
        sel->valid = 1u;
    }
    else if ((best != sel->combo) && ((sel->cost[sel->combo] - sel->cost[best]) > CHASSIS_COST_SWITCH_MARGIN))
    {
        sel->combo = best;
    }

    /* 6. 输出给定量：目标角取与当前角连续的表示（等价于 φ_t，模 2π） */
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        const uint8_t rev = (uint8_t)((sel->combo >> i) & 0x1u); // i=0 左舵 / i=1 右舵

        /* 单轮小力矩不转舵：该轮接触力很小时，"转到力方向"的收益可以忽略，
         * 但舵角一动 align 就掉、力矩投影跟着抖。2026-10-18 原地旋转日志：
         * 平移力 fx 把左轮偏航推力抵消到 ~0（左轮 |τ|<0.2 占 35.8%），左轮为
         * 对齐 1~2 N 的小力转了 60~120°，侧滑让偏航力矩塌陷，形成 0.9 Hz 振荡。
         * 力小时直接沿当前轮向输出分配出的驱动力矩（alloc->wheel_torque 已带符号），
         * 目标角保持不动；只有两轮都小时才走上面的整体锁定（出力 0）。 */
        if (tau_f[i] < CHASSIS_COST_TORQUE_LOCK)
        {
            sel->angle_target[i] = st->steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->wheel_torque[i] = alloc->wheel_torque[i];
            sel->wheel_align[i] = 1.0f;
            sel->reverse[i] = rev;
            continue;
        }

        const float phi_t = phi_f[i] + ((rev != 0u) ? M_PI : 0.0f);
        const float dphi = Lib_Math_WrapAngleNegPIToPI(phi_t - st->steer_angle[i]);

        /* 角度死区：已在死区内就不再动舵（目标 = 当前角），避免 1° 级的来回修正 */
        sel->angle_target[i] = (fabsf(dphi) <= CHASSIS_COST_ANGLE_DEADBAND) ? st->steer_angle[i] : (st->steer_angle[i] + dphi);
        sel->angle_error[i] = fabsf(dphi);
        sel->wheel_torque[i] = (rev != 0u) ? -tau_f[i] : tau_f[i];
        /* 出力比例：cos 是 2π 周期，直接对连续角差取余弦即可（不必先归一化）。
         * 反向解已在 wheel_torque 里带负号，两者相乘后与分支无关。 */
        sel->wheel_align[i] = Lib_Math_Cos(dphi);
        sel->reverse[i] = rev;
    }
#endif
}
#endif // !CHASSIS_LAYERED_ALLOC_ENABLE

/*============================ 舵向零点标定 ============================*/
/**
 * @brief 光电门跳变沿处重标零点：把当前读数改成"正前方 + 机械偏置"对应的位置
 * @param rudder 舵向控制核（位置后处理在它内部，M3508 增量编码器上电无绝对零点）
 * @param offset_deg 跳变沿 → 正前方 的舵轮角度偏置（°，见 robot_def.h）
 * @note position = wrap((多圈累加 + position_offset) * feedback_direction)，故要让读数等于
 *       target，position_offset 的增量须除以 feedback_direction。
 * @note data.position 是 SetRef 时刷新的缓存；标定发生在 SetRef 之后，需同步改本拍缓存，
 *       否则下一拍（已切 RUN）会拿旧偏置的位置去解算。
 * @note 下降沿由控制任务（2ms）轮询得到，采样滞后最多 2ms；扫描 24rad/s（电机侧）
 *       对应最大零点误差 0.048rad ≈ 0.34°（舵轮），对力控足够，不做补偿。
 */
static void RudderCalibrateOffset(DrvlibMotorInstance *rudder, float offset_deg)
{
    const float position_now = (float)rudder->data.position; // 触发瞬间读数（已含方向/偏置/归一化）
    const float target = STEER_WHEEL_DEG_TO_MOTOR_RAD(offset_deg);
    rudder->cfg.position_offset += (target - position_now) / (float)rudder->cfg.feedback_direction;
    rudder->data.position += (double)(target - position_now);
}

static void RudderCalibrateOffset_l(void)
{
    RudderCalibrateOffset(&rudder_l_ctrl, RUDDER_ZERO_OFFSET_L_DEG);
    g_rudder_l_inited = 1;
}

static void RudderCalibrateOffset_r(void)
{
    RudderCalibrateOffset(&rudder_r_ctrl, RUDDER_ZERO_OFFSET_R_DEG);
    g_rudder_r_inited = 1;
}

/**
 * @brief 校准判据：光电门 1→0 下降沿处重标零点并置完成标志
 * @note 必须在舵向 SetRef **之后**调用：标定读的是控制核本拍刚刷新的 data.position。
 */
static void ChassisCalibPollGate(void)
{
    if ((g_rudder_l_gate_state == 1) && (GPIORead(&rudder_l_gate_io) == 0))
    {
        RudderCalibrateOffset_l();
    }
    if ((g_rudder_r_gate_state == 1) && (GPIORead(&rudder_r_gate_io) == 0))
    {
        RudderCalibrateOffset_r();
    }
    // 记录本次电平，供下拍判边沿
    g_rudder_l_gate_state = (uint8_t)GPIORead(&rudder_l_gate_io);
    g_rudder_r_gate_state = (uint8_t)GPIORead(&rudder_r_gate_io);
}

/*============================ 顶层状态机（lib_fsm_table） ============================*/
/* 状态 = 底盘"现在能干的事"，不是云台下发的档位：
 *   CAL   上电校准：两舵轮扫描找光电门零点（此时不理会云台在线与否）
 *   FAULT 校准失败（连续超时）：失能 + 给定清零，终态（需复位重启再标）
 *   SAFE  已标定但不可动：云台掉线 或 云台下发 stop
 *   RUN   已标定且可动：力控/解算入口（正解 → 广义力 → 分配 → 换向决策 → 下发）
 * 事件只有一个"每拍更新"：状态完全由 (是否标定, 是否超时, 是否在线, 是否 stop) 决定。
 * ⚠ 校准是"上电即可做、与云台无关"的第一步：云台常没上电，故 CAL 不受 online 影响。 */

/*---------------- 守卫 ----------------*/
/* 守卫必须无副作用且幂等；四个条件互斥，表内先后只决定"故障优先、校准次之"。 */
static bool ChassisGuardFault(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((ChassisFsmCtx_s *)ctx)->cal_fault != 0;
}

static bool ChassisGuardCal(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done == 0) && (c->cal_fault == 0);
}

static bool ChassisGuardSafe(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && ((c->online == 0) || (c->stopped != 0));
}

static bool ChassisGuardRun(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && (c->online != 0) && (c->stopped == 0);
}

/*---------------- 转移表 ----------------*/
/* 源状态一律通配：四条守卫已把 (标定 × 在线 × stop) 完全划分，落点唯一；
 * 表顺序 = 优先级，故障排最前。 */
static const LibFsmTableTransition_s chassis_fsm_table[] = {
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_FAULT, ChassisGuardFault, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_CAL, ChassisGuardCal, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_SAFE, ChassisGuardSafe, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_RUN, ChassisGuardRun, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
};
LIB_FSM_TABLE_CHECK_TABLE(chassis_fsm_table);

/*---------------- 状态动作 ----------------*/
/* 进入运行态：舵向从"纯速度环扫描"切回"位置+速度环"（只改掩码，不重配 PID）。 */
static void ChassisRunEntry(void *ctx)
{
    (void)ctx;
    rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
    rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
}

/* 进入故障（终态）：上报一次。失能 + 给定清零在主循环 FAULT 分支里做。 */
static void ChassisFaultEntry(void *ctx)
{
    (void)ctx;
    BSPLOG(&g_chassis_log, LOG_LEVEL_ERROR, "cal fault: no photogate edge after %u retries x %u ticks; chassis disabled, reset MCU to retry",
           (unsigned)CHASSIS_CAL_MAX_RETRIES, (unsigned)CHASSIS_CAL_TIMEOUT_TICKS);
}

static const LibFsmTableStateActions_s chassis_fsm_actions[] = {
    [CHASSIS_STATE_CAL] = {NULL, NULL},
    [CHASSIS_STATE_FAULT] = {ChassisFaultEntry, NULL},
    [CHASSIS_STATE_SAFE] = {NULL, NULL},
    [CHASSIS_STATE_RUN] = {ChassisRunEntry, NULL},
};
LIB_FSM_TABLE_CHECK_ACTIONS(chassis_fsm_actions, CHASSIS_STATE_COUNT);

/*---------------- 电机使能 ----------------*/
/* 驱动轮是纯力矩协议（无使能开关），"失能"= 每拍 SetRef(0)+GroupSend；
 * 这里只管舵向控制核的软件使能。检查 enable 是为了不每拍重置 PID / 低通状态。 */
static void ChassisRudderDisable(void)
{
    if (rudder_l_ctrl.enable)
    {
        DrvlibMotorDisable(&rudder_l_ctrl);
    }
    if (rudder_r_ctrl.enable)
    {
        DrvlibMotorDisable(&rudder_r_ctrl);
    }
}

/**
 * @brief 每拍汇总条件，仅在条件变化时派发一次更新事件
 * @note 状态完全由四个布尔量决定；若每拍都派发，同状态也会走一次外部自转移
 *       （退出 + 进入各跑一遍），将来挂上状态动作就会每拍重跑，故只在变化时派发。
 * @note 故障为终态：进入后不再派发（需复位重启才能重标）。
 */
static void ChassisFsmUpdate(void)
{
    ChassisFsmCtx_s *c = &g_chassis_fsm_ctx;

    /* 校准计时：仅在校准态累加。单次超时只重试（重新计时 + 计数 + 上报），不直接判死——
     * 上电找不到零点最常见的失败是漏采到某一圈的光电门沿，下一圈就能补上；
     * 连续 CHASSIS_CAL_MAX_RETRIES 次才判故障（那时基本是传感器/线束/挡片真坏了）。 */
    if (LibFsmTableCurrent(&g_chassis_fsm) == CHASSIS_STATE_CAL)
    {
        c->cal_ticks++;
        if ((c->cal_done == 0) && (c->cal_ticks >= CHASSIS_CAL_TIMEOUT_TICKS))
        {
            c->cal_ticks = 0;
            c->cal_retries++;
            BSPLOG(&g_chassis_log, LOG_LEVEL_WARNING, "cal timeout #%u/%u, keep scanning", (unsigned)c->cal_retries, (unsigned)CHASSIS_CAL_MAX_RETRIES);
        }
    }

    // 采样四个条件
    c->cal_done = (uint8_t)(g_rudder_l_inited && g_rudder_r_inited);
    c->cal_fault = (uint8_t)((c->cal_done == 0) && (c->cal_retries >= CHASSIS_CAL_MAX_RETRIES));
    c->online = GimbalCommIsOnline();
    /* 档位取 g_rudder_mode_rx（接收回调写的原始副本），**不能取 g_gimbal2chassis_data.enabled**：
     * 采样发生在状态动作之后（SAFE 先把指令清零作废，再走到这里），若判据用同一份数据，
     * SAFE 每拍都会读到自己刚写下的 stop ⇒ stopped 恒 1 ⇒ 永远出不去。
     * 档位是"对端说了什么"，只能由接收回调更新，与"本拍要不要执行"必须分开存。 */
    c->stopped = (uint8_t)(g_gimbal_mode_rx == robot_mode_stop);

    // 打包成一个字节比较：任一条件变化就派发一次
    const uint8_t key = (uint8_t)(c->cal_done | (c->cal_fault << 1) | (c->online << 2) | (c->stopped << 3));
    if ((key != c->key_last) && (LibFsmTableCurrent(&g_chassis_fsm) != CHASSIS_STATE_FAULT))
    {
        LibFsmTableDispatch(&g_chassis_fsm, CHASSIS_EV_UPDATE, NULL);
    }
    c->key_last = key;
}

void AppChassisInit(void)
{
    /* ---- 半舵运动学内核：几何尺寸全部来自 robot_def.h ----
     * 内侧按"每组 1 个舵轮 2 行约束"组装，2 组共 4 行 3 未知量；
     * 正解即最小二乘里程计，后续力分配共用同一份几何。 */
    LibChassisHalfRudderGeom_s chassis_geom = {
        .l_pos_x = CHASSIS_RUDDER_L_POS_X,
        .l_pos_y = CHASSIS_RUDDER_L_POS_Y,
        .r_pos_x = CHASSIS_RUDDER_R_POS_X,
        .r_pos_y = CHASSIS_RUDDER_R_POS_Y,
        .radius = CHASSIS_WHEEL_RADIUS,
    };
    BSP_ASSERT_APP_CALL(LibChassisHalfRudderInit(&s_chassis, &chassis_geom));

#if CHASSIS_LAYERED_ALLOC_ENABLE
    s_layered_cfg.pos_y = CHASSIS_RUDDER_L_POS_Y;
    s_layered_cfg.wheel_radius = CHASSIS_WHEEL_RADIUS;
    AppDrvlibChassisReset(&s_layered_alloc);
#endif

    /* ---- 舵向：DJI 广播 ---- */
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&rudder_l_motor));
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&rudder_r_motor));

    DrvsDJIMotorBroadcastConfig_s rudder_l_cfg = {
        .group = &rudder_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 2,
        .torque_constant = 1, // 待标定
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1,
    };
    DrvsDJIMotorBroadcastConfig_s rudder_r_cfg = {
        .group = &rudder_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 4,
        .torque_constant = 1, // 待标定
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1,
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&rudder_l_motor, &rudder_l_cfg));
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&rudder_r_motor, &rudder_r_cfg));

    /* ---- 驱动轮：LK 广播（纯转矩，力控由 app 直接给） ---- */
    BSP_ASSERT_APP_CALL(DrvsLKMotorBroadcastRegister(&wheel_l_motor));
    BSP_ASSERT_APP_CALL(DrvsLKMotorBroadcastRegister(&wheel_r_motor));

    DrvsLKMotorBroadcastConfig_s wheel_l_cfg = {
        .group = &wheel_motor_group,
        .can_e = CAN_2,
        .motor_id = 1,
        .torque_constant = 0.28f, // MF7015-24V-23T
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1,
    };
    DrvsLKMotorBroadcastConfig_s wheel_r_cfg = {
        .group = &wheel_motor_group,
        .can_e = CAN_2,
        .motor_id = 2,
        .torque_constant = 0.28f,
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1,
    };
    BSP_ASSERT_APP_CALL(DrvsLKMotorBroadcastConfig(&wheel_l_motor, &wheel_l_cfg));
    BSP_ASSERT_APP_CALL(DrvsLKMotorBroadcastConfig(&wheel_r_motor, &wheel_r_cfg));

    /* ---- 舵向控制核（drvlib_motor）：左右舵同参数 ----
     * loop_type   ANGLE|SPEED 两环都建（校准期临时收窄成 SPEED，进 RUN 时恢复）
     * 位置模式    WRAP 360°：窗口 ±π×减速比，跨度 = 舵轮整圈 2π（电机 16π）
     *             换向决策的 φ/φ+180° 两个等价解都在窗口内，可连续绕整圈
     * position_span 单圈 2π（M3508 编码器跨度），供多圈累加
     * 方向        见 CHASSIS_RUDDER_x_DIR（motor/feedback 成对）
     * 低通        速度/电流 20ms（沿用旧整定）
     * Config 会自动给位置/速度环补 TRAPEZOID_INTEGRAL，WRAP 下再给位置环补误差归一化。 */
    DrvlibMotor_Config_s rudder_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED,
        .motor_direction = (DrvlibMotorDirection_e)CHASSIS_RUDDER_L_DIR,    // 输出方向
        .feedback_direction = (DrvlibMotorDirection_e)CHASSIS_RUDDER_L_DIR, // 反馈方向
        .position_mode = DRVLIB_MOTOR_POS_WRAP,
        .angle_limit_min = CHASSIS_STEER_WRAP_MIN, // 舵轮 −180°（电机侧 −8π）
        .angle_limit_max = CHASSIS_STEER_WRAP_MAX, // 舵轮 +180°（电机侧 +8π）
        .position_span = M_2PI,                    // 单圈 2π，供多圈累加
        .position_offset = 0.0f,                   // 光电门标定时由 RudderCalibrateOffset 改写
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE,
        .lpf_rc = 0.02f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .pid =
            {
                [DRVLIB_MOTOR_STAGE_SPEED] =
                    {
                        .kp = CHASSIS_RUDDER_PID_SPEED_KP,
                        .ki = CHASSIS_RUDDER_PID_SPEED_KI,
                        .kd = CHASSIS_RUDDER_PID_SPEED_KD,
                        .integral_limit = CHASSIS_RUDDER_PID_SPEED_INTEGRAL_LIMIT,
                        .coef_a = CHASSIS_RUDDER_PID_SPEED_COEF_A,
                        .coef_b = CHASSIS_RUDDER_PID_SPEED_COEF_B,
                        .d_lpf_rc = CHASSIS_RUDDER_PID_SPEED_D_LPF_RC,
                        .out_lpf_rc = CHASSIS_RUDDER_PID_SPEED_OUT_LPF_RC,
                        .deadband = CHASSIS_RUDDER_PID_SPEED_DEADBAND,
                        .error_normalize_range = CHASSIS_RUDDER_PID_SPEED_ERR_NORM_RANGE,
                        .out_max = CHASSIS_RUDDER_PID_SPEED_OUT_MAX,
                        .out_min = CHASSIS_RUDDER_PID_SPEED_OUT_MIN,
                        .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL |   // 梯形积分
                                       PID_ENABLE_INTEGRAL_LIMIT |       // 积分限幅
                                       PID_ENABLE_CHANGING_INTEGRATION | // 变速积分
                                       PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行
                                       PID_ENABLE_DERIVATIVE_FILTER,     // 微分滤波
                    },
                // 位置环：只上 P+D（无积分），误差归一化（WRAP 下由 Config 自动开）
                [DRVLIB_MOTOR_STAGE_ANGLE] =
                    {
                        .kp = CHASSIS_RUDDER_PID_ANGLE_KP,
                        .ki = CHASSIS_RUDDER_PID_ANGLE_KI,
                        .kd = CHASSIS_RUDDER_PID_ANGLE_KD,
                        .integral_limit = CHASSIS_RUDDER_PID_ANGLE_INTEGRAL_LIMIT,
                        .coef_a = CHASSIS_RUDDER_PID_ANGLE_COEF_A,
                        .coef_b = CHASSIS_RUDDER_PID_ANGLE_COEF_B,
                        .d_lpf_rc = CHASSIS_RUDDER_PID_ANGLE_D_LPF_RC,
                        .out_lpf_rc = CHASSIS_RUDDER_PID_ANGLE_OUT_LPF_RC,
                        .deadband = CHASSIS_RUDDER_PID_ANGLE_DEADBAND,
                        .error_normalize_range = CHASSIS_RUDDER_PID_ANGLE_ERR_NORM_RANGE,
                        .out_max = CHASSIS_RUDDER_PID_ANGLE_OUT_MAX,
                        .out_min = CHASSIS_RUDDER_PID_ANGLE_OUT_MIN,
                        /* 微分先行的测量值：drvlib_motor 在 WRAP 下用连续位置，
                         * 跨 ±180° 不会因整周期跳变打出 D 冲击；再加一阶滤波压编码器噪声。
                         * 用 D-on-measure 而不是 D-on-error：换向决策会让目标角阶跃
                         * ±180°，D-on-error 会对这个阶跃打一拳。 */
                        .config_mask = PID_ENABLE_ERROR_NORMALIZE |    // 启用误差归一化
                                       PID_ENABLE_OUTPUT_LIMIT |       // 输出限幅
                                       PID_ENABLE_DERIVATIVE_ON_MEAS | // 微分先行
                                       PID_ENABLE_DERIVATIVE_FILTER,   // 微分滤波
                    },
            },
        .dt_max = 0.01f,  // 单帧 dt 上限 (s)
        .vofa_enable = 0, // VOFA 通道由 app 统一写
    };
    // 左右舵方向一致（本次实测两舵反馈同号）
    BSP_ASSERT_APP_CALL(DrvlibMotorRegister(&rudder_l_ctrl));
    BSP_ASSERT_APP_CALL(DrvlibMotorRegister(&rudder_r_ctrl));
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&rudder_l_ctrl, &rudder_ctrl_cfg));
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&rudder_r_ctrl, &rudder_ctrl_cfg));

    /* ---- 光电门（控制任务轮询：回调 NULL，不占 EXTI 分发） ---- */
    rudder_l_gate_io.parent = &rudder_l_motor; // parent 仅作身份绑定，轮询不用它
    rudder_r_gate_io.parent = &rudder_r_motor;
    BSP_ASSERT_APP_CALL(GPIORegister(&rudder_l_gate_io));
    BSP_ASSERT_APP_CALL(GPIORegister(&rudder_r_gate_io));

    GPIO_Config_s gate_l_cfg = {
        .gpio_e = GPIO_PWM_1, // PA0 —— 左舵光电门
        .callback = NULL,
    };
    GPIO_Config_s gate_r_cfg = {
        .gpio_e = GPIO_PWM_2, // PA2 —— 右舵光电门
        .callback = NULL,
    };
    BSP_ASSERT_APP_CALL(GPIOConfig(&rudder_l_gate_io, &gate_l_cfg));
    BSP_ASSERT_APP_CALL(GPIOConfig(&rudder_r_gate_io, &gate_r_cfg));

    /* ---- 云台通信（CAN1 / IDSEQ / EXT）---- */
    // 一个实例双向：底盘在 tx_id 段发、rx_id 段收（idseq 后端支持收发异段）。
    CommMediaCanIdseqConfig_s gimbal_comm_media_cfg = {
        .can_e = CAN_1,
        .tx_id = 0x110, /* 底盘→云台 ID 段基址 */
        .rx_id = 0x100, /* 云台→底盘 ID 段基址 */
        .frame_type = CAN_STANDARD_DATA_FRAME,
        .mode = CAN_FRAME_FORMAT_CLASSIC, /* CAN1 上挂经典 CAN 的 RS05，不可用 FD */
        .timeout_ms = 1,
    };
    CommConfig_s gimbal_comm_cfg = {
        .media_cfg = &gimbal_comm_media_cfg,
        .on_frame = GimbalRecvOnFrame,
        .daemon_fault = DAEMON_FAULT_BUZZER_SHORT,
        .daemon_reload = 10, /* 对端每 2ms 发一帧，超 10ms 判离线 */
    };
    BSP_ASSERT_APP_CALL(CommRegister(&gimbal_comm));
    BSP_ASSERT_APP_CALL(CommConfig(&gimbal_comm, &gimbal_comm_cfg));

    /* ---- 底盘 IMU：BMI088 + 线性卡尔曼（纯观测） ---- */
    BSP_ASSERT_APP_CALL(BMI088KalmanRegister(&bmi088));
    BMI088Kalman_Config_s bmi088_cfg = {
        .imu =
            {
                .spi_e = SPI_BMI088,
                .cs_acc_e = GPIO_BMI088_CS_ACCEL,
                .cs_gyro_e = GPIO_BMI088_CS_GYRO,
                .int_acc_e = GPIO_BMI088_INT_ACCEL,
                .int_gyro_e = GPIO_BMI088_INT_GYRO,
                .daemon_reload = 20,
                .daemon_fault = DAEMON_FAULT_NONE,
                .acc_range = BMI088_ACC_RANGE_3G,
                .acc_bwp = BMI088_ACC_BWP_NORMAL,
                .acc_odr = BMI088_ACC_ODR_400,
                .gyro_range = BMI088_GYRO_RANGE_2000,
                .gyro_conf = BMI088_GYRO_CONF_1000_116,
                .work_mode = BMI088_MODE_INT,
                .spi_mode = BSP_DMA_MODE,
                .spi_timeout_ms = 10,
            },
        .calib = &g_bmi088_calib,
        .q_tilt = 1e-5f,
        .q_bias = 0.0f,
        .r_tilt = 4e-4f,
        .dt_max = 0.01f,
        .temp_lpf_alpha = 0.01f,
        /* IMU 自己发 VOFA 会占用 CH1~CH15（标定脚本布局）。当前遥测改成
         * 底盘数据（舵向/轮向，100Hz，见 AppChassisRun 末尾），通道只能给一方；
         * 需要重新标定 IMU 时把这里改回 1，并同时去掉底盘 VOFA 发送。 */
        .vofa_enable = 0,
    };
    BSP_ASSERT_APP_CALL(BMI088KalmanConfig(&bmi088, &bmi088_cfg));

    // IMU 原点 (0,0,0) 经齐次变换得到它在底盘系的位置（旋转 + 平移一起验证）
    g_imu_chassis.pos = Lib_Math_Mat4MulPoint(g_imu_to_chassis, (vector3_t){0.0f, 0.0f, 0.0f});

    // 杆臂补偿状态复位
    g_gyro_prev = (vector3_t){0.0f, 0.0f, 0.0f};
    g_alpha_lpf = (vector3_t){0.0f, 0.0f, 0.0f};
    g_gyro_prev_valid = 0;

    /* ---- 顶层状态机：上电从校准起，等两个光电门标定完再判在线/使能 ---- */
    LibFsmTableConfig_s chassis_fsm_cfg = {
        .initial_state = CHASSIS_STATE_CAL,
        .transitions = chassis_fsm_table,
        .transition_count = sizeof(chassis_fsm_table) / sizeof(chassis_fsm_table[0]),
        .state_actions = chassis_fsm_actions,
        .state_count = CHASSIS_STATE_COUNT,
        .ctx = &g_chassis_fsm_ctx,
        .trace_fn = NULL,
    };
    LibFsmTableInit(&g_chassis_fsm, &chassis_fsm_cfg);
    LibFsmTableStart(&g_chassis_fsm);
}

void AppChassisRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;

    /* IMU 纯观测：每拍更新。vofa_enable=1 时 Update 内部会顺带发一帧 VOFA，
     * 直接用 VOFA+ 录 CSV，喂 bmi088_calib.py 标定陀螺零偏。 */
    BMI088KalmanUpdate(&bmi088);
    const BMI088Kalman_Data_t imu = BMI088KalmanGetData(&bmi088);

    /* IMU 系 → 底盘系（方向量，不带平移） */
    const vector3_t gyro_imu = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    const vector3_t acc_imu = {imu.acc[0], imu.acc[1], imu.acc[2]};
    const vector3_t gyro = Lib_Math_Mat4MulDir(g_imu_to_chassis, gyro_imu);
    const vector3_t acc_chassis = Lib_Math_Mat4MulDir(g_imu_to_chassis, acc_imu);

    g_imu_chassis.gyro = gyro;
    g_imu_chassis.yaw_rate = imu.yaw_rate; // world 竖直轴，与安装方向无关
    g_imu_chassis.valid = imu.valid;

    /* 角加速度：只在有新陀螺样本时差分（imu.dt == 0 = 本拍没新样本，保持上次 α）。
     * 直接差分噪声大，先限幅再做一阶低通；RC 见 CHASSIS_IMU_ALPHA_LPF_RC。 */
    if (imu.dt > 0.0f)
    {
        if (g_gyro_prev_valid)
        {
            const float dt_a = imu.dt;
            vector3_t alpha = Lib_Math_Vec3Scale(Lib_Math_Vec3Sub(gyro, g_gyro_prev), 1.0f / dt_a);
            alpha.x = Lib_Math_Clamp(alpha.x, -CHASSIS_IMU_ALPHA_LIMIT, CHASSIS_IMU_ALPHA_LIMIT);
            alpha.y = Lib_Math_Clamp(alpha.y, -CHASSIS_IMU_ALPHA_LIMIT, CHASSIS_IMU_ALPHA_LIMIT);
            alpha.z = Lib_Math_Clamp(alpha.z, -CHASSIS_IMU_ALPHA_LIMIT, CHASSIS_IMU_ALPHA_LIMIT);

            const float k = dt_a / (CHASSIS_IMU_ALPHA_LPF_RC + dt_a);
            g_alpha_lpf.x += (alpha.x - g_alpha_lpf.x) * k;
            g_alpha_lpf.y += (alpha.y - g_alpha_lpf.y) * k;
            g_alpha_lpf.z += (alpha.z - g_alpha_lpf.z) * k;
        }
        g_gyro_prev = gyro;
        g_gyro_prev_valid = 1;
    }

    /* 杆臂补偿：a_center = a_imu − α×r − ω×(ω×r) */
    const vector3_t centripetal = Lib_Math_Vec3Cross(gyro, Lib_Math_Vec3Cross(gyro, g_imu_chassis.pos));
    const vector3_t tangential = Lib_Math_Vec3Cross(g_alpha_lpf, g_imu_chassis.pos);
    g_imu_chassis.acc = Lib_Math_Vec3Sub(Lib_Math_Vec3Sub(acc_chassis, tangential), centripetal);

    /*============================ 电机反馈 ============================*/
    /* 每拍都读、都喂给控制核：失能期间也必须持续处理反馈，否则手动转动舵向电机后
     * 多圈累加与真实位置脱节，再使能的第一拍位置会跳。
     * 方向：反馈一律先取协议原始量，舵向由 drvlib 的 feedback_direction 统一修正；
     *       驱动轮不走 drvlib，这里按 CHASSIS_WHEEL_x_DIR 换算到 app 正方向。 */
    const DrvsDJIMotorBroadcastData_s rudder_l_data = DrvsDJIMotorBroadcastGetData(&rudder_l_motor);
    const DrvsDJIMotorBroadcastData_s rudder_r_data = DrvsDJIMotorBroadcastGetData(&rudder_r_motor);
    const DrvlibMotorFeedback_s rudder_l_fb = {
        .position = rudder_l_data.position,
        .speed = rudder_l_data.speed,
        .current = rudder_l_data.current,
        .timestamp_us = rudder_l_data.timestamp_us,
    };
    const DrvlibMotorFeedback_s rudder_r_fb = {
        .position = rudder_r_data.position,
        .speed = rudder_r_data.speed,
        .current = rudder_r_data.current,
        .timestamp_us = rudder_r_data.timestamp_us,
    };

    const DrvsLKMotorBroadcastData_s wheel_l_data = DrvsLKMotorBroadcastGetData(&wheel_l_motor);
    const DrvsLKMotorBroadcastData_s wheel_r_data = DrvsLKMotorBroadcastGetData(&wheel_r_motor);
    g_wheel_fb[APP_WHEEL_L].speed = wheel_l_data.speed * (float)CHASSIS_WHEEL_L_DIR;
    g_wheel_fb[APP_WHEEL_L].torque = wheel_l_data.torque * (float)CHASSIS_WHEEL_L_DIR;
    g_wheel_fb[APP_WHEEL_L].current = wheel_l_data.current * (float)CHASSIS_WHEEL_L_DIR;
    g_wheel_fb[APP_WHEEL_L].temperature = wheel_l_data.temperature;
    g_wheel_fb[APP_WHEEL_L].error = wheel_l_data.echo;
    g_wheel_fb[APP_WHEEL_R].speed = wheel_r_data.speed * (float)CHASSIS_WHEEL_R_DIR;
    g_wheel_fb[APP_WHEEL_R].torque = wheel_r_data.torque * (float)CHASSIS_WHEEL_R_DIR;
    g_wheel_fb[APP_WHEEL_R].current = wheel_r_data.current * (float)CHASSIS_WHEEL_R_DIR;
    g_wheel_fb[APP_WHEEL_R].temperature = wheel_r_data.temperature;
    g_wheel_fb[APP_WHEEL_R].error = wheel_r_data.echo;

    /*============================ 按状态生成给定 ============================*/
    /* 顺序约定：① 状态给定（舵向在本拍喂反馈 + 算下发量）→ ② 校准判光电门（用 ① 刚刷新
     * 的 data.position）→ ③ 采样条件并（变化时）派发 → ④ 下发。
     * ⚠ 每拍每个舵向只跑一次 SetRef；状态变化从下一拍生效。 */
    const ChassisState_e state = (ChassisState_e)LibFsmTableCurrent(&g_chassis_fsm);
    const float dt_s = dt * 0.001f; // APP_TASK_DEF 的 dt 是 ms，drvlib 要秒

    float rudder_l_ref = 0.0f;
    float rudder_r_ref = 0.0f;
    g_wheel_l_setref = 0.0f;
    g_wheel_r_setref = 0.0f;

    switch (state)
    {
    case CHASSIS_STATE_CAL:
        // 扫描：舵向用纯速度环（位置环留到运行态），两舵轮以恒定速度转着找光电门
        rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_SPEED;
        rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_SPEED;
        DrvlibMotorEnable(&rudder_l_ctrl);
        DrvlibMotorEnable(&rudder_r_ctrl);
        rudder_l_ref = CHASSIS_STEER_ZERO_SCAN_SPEED;
        rudder_r_ref = CHASSIS_STEER_ZERO_SCAN_SPEED;
        break;

    case CHASSIS_STATE_RUN:
        /* 已校准 + 云台在线且使能（守卫保证）。
         * 给定取**上一拍**的换向决策（正解/力控/分配/决策在本函数后半段，晚于 SetRef）；
         * 2ms 滞后对 500Hz 控制可忽略。
         * ⚠ 首次进入 RUN 时决策还是 0 初始化（angle_target=0 是电机侧 0 rad，
         *   不是标定零点），不能直接下发；valid==0 时保持当前舵角、轮向 0 力矩，
         *   本拍后半段算出的决策从下一拍开始生效。 */
        rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
        rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
        DrvlibMotorEnable(&rudder_l_ctrl);
        DrvlibMotorEnable(&rudder_r_ctrl);

        if (g_chassis_cost_sel.valid != 0u)
        {
            /* 舵向：决策给的是舵轮侧 rad，控制核要电机侧 rad。
             * 角速度上限由位置环输出限幅（CHASSIS_RUDDER_PID_ANGLE_OUT_MAX）保证。 */
            rudder_l_ref = g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_L] * CHASSIS_STEER_GEAR_RATIO;
            rudder_r_ref = g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_R] * CHASSIS_STEER_GEAR_RATIO;

            /* 轮向：带符号轮缘力矩 × cos(目标−实际)（投影到目标轮面方向）。
             * 比例取上一拍决策算好的值（2ms 滞后可忽略）；不钳负，见 def.h 说明。 */
            g_wheel_l_setref = g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_L] * g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_L] /
                               CHASSIS_DRIVE_REDUCTION;
            g_wheel_r_setref = g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_R] * g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_R] /
                               CHASSIS_DRIVE_REDUCTION;
        }
        else
        {
            /* 决策尚未产生：舵向保持当前电机侧位置（data.position 是上一拍刷新的），轮向不出力 */
            rudder_l_ref = (float)rudder_l_ctrl.data.position;
            rudder_r_ref = (float)rudder_r_ctrl.data.position;
        }
        break;

    case CHASSIS_STATE_FAULT:
    case CHASSIS_STATE_SAFE:
    default:
        /* 故障 / 掉线 / 云台 stop：舵向失能、给定清零。
         * SAFE 把收到的指令一并作废，防止"链路刚恢复"那一拍拿掉线前的旧帧执行；
         * ⚠ 只清指令，勿动档位 g_gimbal_mode_rx（状态机的 stopped 判据要靠它）。 */
        ChassisRudderDisable();
        if (state == CHASSIS_STATE_SAFE)
        {
            g_gimbal2chassis_data.enabled = robot_mode_stop;
            g_gimbal2chassis_data.vx = 0.0f;
            g_gimbal2chassis_data.vy = 0.0f;
            g_gimbal2chassis_data.w = 0.0f;
        }
        break;
    }

    g_rudder_l_setref = rudder_l_ref;
    g_rudder_r_setref = rudder_r_ref;

    /* ⚠ 无论什么状态都调 SetRef：失能时输出恒 0，但反馈（多圈累加/偏置/低通）照常处理；
     *   这样"失能时手动转舵向电机 → 再使能"位置仍然连续。 */
    const float rudder_l_cmd = DrvlibMotorSetRef(&rudder_l_ctrl, rudder_l_ref, &rudder_l_fb, dt_s);
    const float rudder_r_cmd = DrvlibMotorSetRef(&rudder_r_ctrl, rudder_r_ref, &rudder_r_fb, dt_s);

    // 校准判据：用上面 SetRef 刚刷新的 data.position
    if (state == CHASSIS_STATE_CAL)
    {
        ChassisCalibPollGate();
    }

    // 采样四个条件 + 变化才派发；状态变化从下一拍生效
    ChassisFsmUpdate();

    /*============================ 正解 + 力控 ============================*/
    /* 轮速正解：舵角取本拍 SetRef 刚刷新的 data.position（电机侧 rad），换算到舵轮侧；
     * 轮速用 app 正方向（= 沿当前舵角推动车体），与内核驱动行的符号约定一致。 */
    LibChassisState_s chassis_state = {0};
    chassis_state.steer_angle[LIB_CHASSIS_HALF_RUDDER_L] = (float)rudder_l_ctrl.data.position * CHASSIS_STEER_GEAR_RATIO_INV;
    chassis_state.steer_angle[LIB_CHASSIS_HALF_RUDDER_R] = (float)rudder_r_ctrl.data.position * CHASSIS_STEER_GEAR_RATIO_INV;
    chassis_state.wheel_speed[LIB_CHASSIS_HALF_RUDDER_L] = g_wheel_fb[APP_WHEEL_L].speed / CHASSIS_DRIVE_REDUCTION;
    chassis_state.wheel_speed[LIB_CHASSIS_HALF_RUDDER_R] = g_wheel_fb[APP_WHEEL_R].speed / CHASSIS_DRIVE_REDUCTION;

    vector3_t twist;
    if (LibChassisForward(&s_chassis, &chassis_state, &twist) == 0)
    {
        g_chassis_twist_fb = twist;
    }
    else
    {
        /* 几何退化（JᵀJ 奇异）：不喂假反馈 */
        g_chassis_twist_fb = (vector3_t){0.0f, 0.0f, 0.0f};
    }
    /* w 反馈用 IMU 底盘系 z 轴角速度（与轮速正解解耦；IMU 不可用时置 0） */
    g_chassis_twist_fb.z = (g_imu_chassis.valid != 0u) ? g_imu_chassis.gyro.z : 0.0f;

    /* 速度指令 → 广义力 → 舵角/轮向给定。
     * 只在 RUN（已校准且云台使能）计算，其余状态输出清零。
     *   CHASSIS_LAYERED_ALLOC_ENABLE=1：appdrvlib_chassis 分层分配（不再调用
     *     LibChassisAllocateTorque）：快层按平移力方向分配驱动、差模承担快速 yaw，
     *     慢层用限幅限速的 toe trim 承担低通后的 yaw，两轮共用一个 ±180° 分支。
     *   =0：lib_chassis 4 行最小范数 + 换向决策（旧路径完整保留）。
     * 两条路径都写进 g_chassis_cost_sel，下一拍由 RUN 分支统一下发。 */
    if (state == CHASSIS_STATE_RUN)
    {
        g_chassis_wrench = ChassisForceSolve(&g_gimbal2chassis_data, &g_chassis_twist_fb, dt_s);
#if CHASSIS_LAYERED_ALLOC_ENABLE
        AppDrvlibChassisOut_s alloc_out;
        if (AppDrvlibChassisAllocate(&s_layered_alloc, &s_layered_cfg, chassis_state.steer_angle,
                                     g_chassis_wrench.x, g_chassis_wrench.y, g_chassis_wrench.z,
                                     dt_s, &alloc_out) != 0)
        {
            /* 参数/几何非法：输出清零，不保留上一拍结果 */
            g_chassis_torque = (LibChassisTorque_s){0};
            g_chassis_cost_sel = (ChassisCostSel_s){0};
        }
        else
        {
            ChassisCostSel_s *sel = &g_chassis_cost_sel;
            for (uint8_t i = 0u; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
            {
                sel->angle_target[i] = alloc_out.angle_target[i];
                sel->wheel_torque[i] = alloc_out.wheel_torque[i];
                sel->wheel_align[i] = alloc_out.wheel_align[i];
                sel->angle_error[i] = fabsf(Lib_Math_WrapAngleNegPIToPI(alloc_out.angle_target[i] - chassis_state.steer_angle[i]));
                sel->reverse[i] = alloc_out.reverse;
            }
            for (uint8_t k = 0u; k < 4u; k++)
            {
                sel->cost[k] = 0.0f; // 分层路径不算换向代价，遥测占位
            }
            sel->combo = (uint8_t)((alloc_out.reverse & 1u) | ((alloc_out.reverse & 1u) << 1));
            sel->locked = 0u;
            sel->valid = alloc_out.valid;
        }
#else
        if (LibChassisAllocateTorque(&s_chassis, &chassis_state, g_chassis_wrench, &g_chassis_torque) != 0)
        {
            /* 几何退化（JᵀJ 奇异）：输出清零，不保留上一拍结果；换向决策一并作废 */
            g_chassis_torque = (LibChassisTorque_s){0};
            g_chassis_cost_sel = (ChassisCostSel_s){0};
        }
        else
        {
            /* 换向决策（模式见 CHASSIS_STEER_SELECT_SHORTEST）：逐轮最短路径
             * 或 4 组合代价；结果见 g_chassis_cost_sel，下一拍由 RUN 分支下发。 */
            ChassisCostSelect(&chassis_state, &g_chassis_torque);
        }
#endif
    }
    else
    {
        g_chassis_wrench = (vector3_t){0.0f, 0.0f, 0.0f};
        g_chassis_torque = (LibChassisTorque_s){0};
        /* 离开运行态即作废换向历史：下次使能重新决策（不沿用急停前的方案） */
        g_chassis_cost_sel = (ChassisCostSel_s){0};
#if CHASSIS_LAYERED_ALLOC_ENABLE
        AppDrvlibChassisReset(&s_layered_alloc);
#endif
        ChassisForceReset(); // 三个速度环 PID 的积分/滤波状态都从 0 重新起
    }
    g_chassis_cmd_prev = (vector3_t){g_gimbal2chassis_data.vx, g_gimbal2chassis_data.vy, g_gimbal2chassis_data.w};

    /* 实测速度回传云台（线协议方向与车体系一致：vx 前 / vy 左 / w 逆时针） */
    g_chassis2gimbal_data.vx = g_chassis_twist_fb.x;
    g_chassis2gimbal_data.vy = g_chassis_twist_fb.y;
    g_chassis2gimbal_data.w = g_chassis_twist_fb.z;

    /*============================ 下发 ============================*/
    // 舵向：drvs 纯协议。两个 SetRef 必须都在 GroupSend 之前完成（一帧组两台）。
    DrvsDJIMotorBroadcastSetRef(&rudder_l_motor, rudder_l_cmd);
    DrvsDJIMotorBroadcastSetRef(&rudder_r_motor, rudder_r_cmd);
    DrvsDJIMotorBroadcastGroupSend(&rudder_motor_group);

    // 驱动轮：纯力矩协议，无使能开关。停机 = 每拍 SetRef(0) + GroupSend。
    // CHASSIS_WHEEL_x_DIR 是"app 正方向 → 电机原始方向"的力矩符号。
    DrvsLKMotorBroadcastSetRef(&wheel_l_motor, g_wheel_l_setref * (float)CHASSIS_WHEEL_L_DIR);
    DrvsLKMotorBroadcastSetRef(&wheel_r_motor, g_wheel_r_setref * (float)CHASSIS_WHEEL_R_DIR);
    DrvsLKMotorBroadcastGroupSend(&wheel_motor_group);

    /* 云台通信：每周期回一帧（vx/vy 来自轮速正解，w 来自 IMU，见上面回传段）。
     * 即使全 0 也必须发——云台侧靠这帧维持链路在线。 */
    CommSend(&gimbal_comm, (uint8_t *)&g_chassis2gimbal_data);

    /*============================ VOFA 遥测：100Hz ============================*/
    /* 500Hz 任务按时间累计，每满 10ms 发一帧（不依赖任务周期整除）。
     * 通道布局见 app_cfg.h；IMU 自己的 VOFA 已关（通道只够一方）。 */
    static float s_vofa_accum_ms = 0.0f;
    s_vofa_accum_ms += dt;
    if (s_vofa_accum_ms >= 10.0f)
    {
        s_vofa_accum_ms -= 10.0f;

        const float r2d = 180.0f / M_PI;
        /* 目标角内部是连续表达（可能 >180°），遥测统一折到 (−180°,180°] 与实测同系；
         * 遥控判换向时看角度差即可（wrap 差），不要直接减。 */
        VofaSetChannel(1, Lib_Math_WrapAngleNegPIToPI(g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_L]) * r2d); // 左舵目标 (deg)
        VofaSetChannel(2, chassis_state.steer_angle[LIB_CHASSIS_HALF_RUDDER_L] * r2d);                                    // 左舵实际 (deg)
        VofaSetChannel(3, Lib_Math_WrapAngleNegPIToPI(g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_R]) * r2d); // 右舵目标 (deg)
        VofaSetChannel(4, chassis_state.steer_angle[LIB_CHASSIS_HALF_RUDDER_R] * r2d);                                    // 右舵实际 (deg)
        VofaSetChannel(5, g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_L]);                                    // 左轮目标 (N·m)
        VofaSetChannel(6, g_wheel_fb[APP_WHEEL_L].torque * CHASSIS_DRIVE_REDUCTION);                                      // 左轮实测 (N·m)
        VofaSetChannel(7, g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_R]);                                    // 右轮目标 (N·m)
        VofaSetChannel(8, g_wheel_fb[APP_WHEEL_R].torque * CHASSIS_DRIVE_REDUCTION);                                      // 右轮实测 (N·m)
        VofaSetChannel(9, (float)g_chassis_cost_sel.combo);                                                               // 换向方案 0~3
        VofaSetChannel(10, g_chassis_cost_sel.cost[0]);                                                                   // 组合代价 (s)
        VofaSetChannel(11, g_chassis_cost_sel.cost[1]);
        VofaSetChannel(12, g_chassis_cost_sel.cost[2]);
        VofaSetChannel(13, g_chassis_cost_sel.cost[3]);
        VofaSetChannel(14, (float)g_chassis_cost_sel.locked);                          // 小力矩锁定
        VofaSetChannel(15, (float)state);                                              // 状态机状态
        VofaSetChannel(16, g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_L]); // 左轮出力比例 cos(目标−实际)
        VofaSetChannel(17, g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_R]); // 右轮出力比例 cos(目标−实际)
        /* CH18~28：命令 / 反馈 / 力控中间量。换向乱转的定位关键是
         * "期望力方向为什么转" —— 对比 CH18~20（指令）与 CH21~23（反馈），
         * 再看 CH24~26 是否跟着转，就能区分指令在转、反馈在转还是分配在转。 */
        VofaSetChannel(18, g_gimbal2chassis_data.vx);                                // 指令 vx (m/s)
        VofaSetChannel(19, g_gimbal2chassis_data.vy);                                // 指令 vy (m/s)
        VofaSetChannel(20, g_gimbal2chassis_data.w);                                 // 指令 w (rad/s)
        VofaSetChannel(21, g_chassis_twist_fb.x);                                    // 反馈 vx (m/s，轮速正解)
        VofaSetChannel(22, g_chassis_twist_fb.y);                                    // 反馈 vy (m/s，轮速正解)
        VofaSetChannel(23, g_chassis_twist_fb.z);                                    // 反馈 w (rad/s，IMU)
        VofaSetChannel(24, g_chassis_wrench.x);                                      // 期望力 fx (N)
        VofaSetChannel(25, g_chassis_wrench.y);                                      // 期望力 fy (N)
        VofaSetChannel(26, g_chassis_wrench.z);                                      // 期望力矩 mw (N·m)
        VofaSetChannel(27, g_wheel_fb[APP_WHEEL_L].speed / CHASSIS_DRIVE_REDUCTION); // 左驱动轮轮速 (rad/s)
        VofaSetChannel(28, g_wheel_fb[APP_WHEEL_R].speed / CHASSIS_DRIVE_REDUCTION); // 右驱动轮轮速 (rad/s)
        VofaSend();
    }
}
