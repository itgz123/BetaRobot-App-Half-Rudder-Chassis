/**
 * @file app_chassis_force_loop.c
 * @brief 闭环实现：三通道 lib_pid + 可配置前馈（当前方案）
 *
 * 处理顺序与拆分前完全一致：
 *   1. 用本拍指令与上一拍指令差分出加速度；
 *   2. 摩擦（沿指令符号）+ 惯性 + 重力/坡道 前馈；
 *   3. 三个 lib_pid 计算反馈，前馈作为 feedforward 传入；
 *   4. 记录本拍指令，供下一拍差分。
 * 非 RUN 时与原实现一样：每拍清 PID 积分/滤波状态，输出 0，但仍记录指令。
 */

#include "app_chassis_force_loop.h"

#include "lib_pid.h"

#if (CHASSIS_CLOSED_LOOP_SCHEME != 0) && (CHASSIS_CLOSED_LOOP_SCHEME != 1)
#error "invalid CHASSIS_CLOSED_LOOP_SCHEME"
#endif

#if CHASSIS_CLOSED_LOOP_SCHEME == 1
#error "CHASSIS_CLOSED_LOOP_SCHEME=1 (RLS/power planning) is reserved but not implemented"
#endif

/* 力控反馈 PID 实例（只在 RUN 更新；非 RUN 由 ChassisForceLoopReset 清状态） */
static struct
{
    ChassisForceLoopCfg_s cfg;

    PIDInstance pid_vx;
    PIDInstance pid_vy;
    PIDInstance pid_w;

    vector3_t cmd_prev; // 上一拍指令（算惯性前馈的指令加速度）
} s_loop;

static const PID_Init_Config_s s_pid_cfg_vx = {
    .kp = CHASSIS_FORCE_VX_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .deadband = CHASSIS_FORCE_XY_DEADBAND,
    .out_lpf_rc = CHASSIS_FORCE_XY_LPF_RC,
    .config_mask = PID_ENABLE_DEADBAND | PID_ENABLE_OUTPUT_FILTER,
};

static const PID_Init_Config_s s_pid_cfg_vy = {
    .kp = CHASSIS_FORCE_VY_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .deadband = CHASSIS_FORCE_XY_DEADBAND,
    .out_lpf_rc = CHASSIS_FORCE_XY_LPF_RC,
    .config_mask = PID_ENABLE_DEADBAND | PID_ENABLE_OUTPUT_FILTER,
};

static const PID_Init_Config_s s_pid_cfg_w = {
    .kp = CHASSIS_FORCE_W_KP,
    .ki = CHASSIS_FORCE_W_KI,
    .kd = 0.0f,
    .integral_limit = CHASSIS_FORCE_W_INTEGRAL_LIMIT,
    .out_lpf_rc = CHASSIS_FORCE_W_LPF_RC,
    /* 不开 DEADBAND：它会把 i_out 清零（见 def.h 的说明） */
    .config_mask = PID_ENABLE_INTEGRAL_LIMIT | PID_ENABLE_OUTPUT_FILTER,
};

void ChassisForceLoopInit(const ChassisForceLoopCfg_s *cfg)
{
    s_loop.cfg = *cfg;
    s_loop.cmd_prev = (vector3_t){0.0f, 0.0f, 0.0f};

    LibPIDInit(&s_loop.pid_vx, &s_pid_cfg_vx);
    LibPIDInit(&s_loop.pid_vy, &s_pid_cfg_vy);
    LibPIDInit(&s_loop.pid_w, &s_pid_cfg_w);
    ChassisForceLoopReset();
}

void ChassisForceLoopReset(void)
{
    LibPIDReset(&s_loop.pid_vx);
    LibPIDReset(&s_loop.pid_vy);
    LibPIDReset(&s_loop.pid_w);
}

/* 前馈：摩擦（沿指令方向）+ 惯性 + 重力/坡道 */
static vector3_t ChassisForceFeedforward(const ChassisForceCfg_s *c, const vector3_t *cmd, const vector3_t *cmd_prev, float dt_s)
{
    /* 指令加速度（惯性前馈）：dt 非正时置 0，避免除零/首拍尖峰 */
    float a_vx = 0.0f;
    float a_vy = 0.0f;
    float a_w = 0.0f;
    if (dt_s > 0.0f)
    {
        a_vx = (cmd->x - cmd_prev->x) / dt_s;
        a_vy = (cmd->y - cmd_prev->y) / dt_s;
        a_w = (cmd->z - cmd_prev->z) / dt_s;
    }

    float ff_x = c->mass * a_vx + c->gravity_x;
    float ff_y = c->mass * a_vy + c->gravity_y;
    float ff_w = c->inertia_z * a_w;
    if (cmd->x > 0.0f)
    {
        ff_x += c->friction_x;
    }
    else if (cmd->x < 0.0f)
    {
        ff_x -= c->friction_x;
    }
    if (cmd->y > 0.0f)
    {
        ff_y += c->friction_y;
    }
    else if (cmd->y < 0.0f)
    {
        ff_y -= c->friction_y;
    }
    if (cmd->z > 0.0f)
    {
        ff_w += c->friction_w;
    }
    else if (cmd->z < 0.0f)
    {
        ff_w -= c->friction_w;
    }

    return (vector3_t){ff_x, ff_y, ff_w};
}

vector3_t ChassisForceLoopUpdate(const ChassisForceLoopInput_s *in, uint8_t enabled)
{
    vector3_t wrench = {0.0f, 0.0f, 0.0f};

    if (enabled != 0u)
    {
        /* 反馈：三个 lib_pid 统一计算（前馈作为 feedforward 参数传入）。
         * 平动通道的死区/输出低通见 def.h：轮速正解在舵轮偏转/高转速下的噪声远大于
         * 真实平移，不抑制会产生与偏航力同量级的假力。
         * yaw 通道用 PI：纯 P 留下与偏航阻力成正比的稳态误差（实测 w 漂移）。 */
        const vector3_t ff = ChassisForceFeedforward(s_loop.cfg.force_cfg, &in->cmd, &s_loop.cmd_prev, in->dt);
        wrench.x = LibPIDCalculate(&s_loop.pid_vx, in->cmd.x, in->twist_fb.x, ff.x, in->dt);
        wrench.y = LibPIDCalculate(&s_loop.pid_vy, in->cmd.y, in->twist_fb.y, ff.y, in->dt);
        wrench.z = LibPIDCalculate(&s_loop.pid_w, in->cmd.z, in->twist_fb.z, ff.z, in->dt);
    }
    else
    {
        /* 离开 RUN：积分/滤波状态从 0 重新起（与原实现每拍 Reset 一致） */
        ChassisForceLoopReset();
    }

    /* 无论是否计算，都记录本拍指令：
     * 非 RUN 期间指令仍可能变化（SAFE 会清零），下一拍进 RUN 的惯性前馈要用它。 */
    s_loop.cmd_prev = in->cmd;
    return wrench;
}
