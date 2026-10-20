#include "app_chassis_force.h"
#include "app_cfg.h"
#include "app_chassis_force_def.h" // 宏 + 类型（实体定义在本文件）
#include "app_chassis_feedback.h"
#include "app_chassis_fsm.h"
#include "app_chassis_force_loop.h"
//
#include "app.h"       // gimbal2chassis_data_t / chassis2gimbal_data_t / robot_mode
#include "robot_def.h" // IMU 安装方向与位置宏
#include "lib_math.h"
#include "bsp_sys_status.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "drvs_djimotor_broadcast.h"
#include "drvs_lkmotor_broadcast.h"
#include "drvlib_bmi088_kalman.h"
#include "drvlib_motor.h"
#include "lib_chassis.h" // LibChassisTorque_s
#include "lib_chassis_half_rudder.h"

#include "drv_comm.h"
#include "comm_media_can_idseq.h"
#include "comm_proto_ext.h"
#include "drv_vofa.h"

#include <string.h>

/* 换向决策里用 g_chassis_feedback.wheel[i] 按舵轮序号取轮向反馈：两套轮序必须一致 */
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_L == (int)APP_WHEEL_L, "wheel order mismatch: L");
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_R == (int)APP_WHEEL_R, "wheel order mismatch: R");

/* 日志实例（校准超时/故障上报；LOG_UART 未开时不发射代码） */
LOG_INSTANCE_DEF(g_chassis_log, "chassis", 20);

/*============================ 全局变量 / 常量 ============================*/
/* 原 app_chassis_force_def.c 的实体定义集中在这里：
 * 类型与宏在 app_chassis_force_def.h，实体只在真正使用它们的 .c 中定义。 */

/*---------------- 云台↔底盘通信 ----------------*/
/* CAN1 / IDSEQ / PROTO_EXT。底盘 tx_id=0x110 段、rx_id=0x100 段（云台侧对调）；
 * 线上帧 = [0xA5][seq][汉明编码区 E][CRC8][0x5A]，payload 就是下面两个业务结构体。
 * CAN1 上还有经典 CAN 的 RS05，故介质必须 CLASSIC，不能开 FD。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, EXT, EXT, gimbal2chassis_data_t, 13, chassis2gimbal_data_t, 12, UNPACK_IN_ISR);

gimbal2chassis_data_t g_gimbal2chassis_data = {0};      // 云台→底盘：速度指令 + 档位
volatile robot_mode g_gimbal_mode_rx = robot_mode_stop; // 接收回调写的档位原始副本
chassis2gimbal_data_t g_chassis2gimbal_data = {0};      // 底盘→云台：速度反馈

/*---------------- 常量 ----------------*/
/* 4x4 齐次变换：p_chassis = R · p_imu + t。
 * R 的三列 = IMU 的 x/y/z 轴在底盘系中的方向（宏在 robot_def.h）：
 *     x_imu = (−1, 1, 0)/√2   y_imu = (1, 1, 0)/√2   z_imu = (0, 0, −1)
 *     t     = IMU 中心在底盘系的位置。
 * 陀螺/加速度是方向量，用 Lib_Math_Mat4MulDir（不带平移）；
 * 点是位置量，用 Lib_Math_Mat4MulPoint（带平移）。 */
const matrix4_t g_imu_to_chassis = {
    .data =
        {
            {CHASSIS_IMU_X_AXIS_X, CHASSIS_IMU_Y_AXIS_X, CHASSIS_IMU_Z_AXIS_X, CHASSIS_IMU_POS_X},
            {CHASSIS_IMU_X_AXIS_Y, CHASSIS_IMU_Y_AXIS_Y, CHASSIS_IMU_Z_AXIS_Y, CHASSIS_IMU_POS_Y},
            {CHASSIS_IMU_X_AXIS_Z, CHASSIS_IMU_Y_AXIS_Z, CHASSIS_IMU_Z_AXIS_Z, CHASSIS_IMU_POS_Z},
            {0.0f, 0.0f, 0.0f, 1.0f},
        },
};

/* 底盘车架上那颗 BMI088 的静态零偏（2026-10-17，ignore/chassis_data/imu.csv）
 * 录制条件：静置 112s、约 33℃、IMU 倒装（+z 朝下）。地面轻微不平按水平处理；
 * 陀螺零偏与安装倾角无关，故这里只标零偏，acc 的六位置标定尚未做。 */
const BMI088_Calib_s g_bmi088_calib = {
    .gyro = {.bias = {-0.000721f, -0.000027f, -0.000445f}},
};

/*---------------- 运行状态 ----------------*/
/* 反馈处理结果：底盘系 IMU、驱动轮反馈、正解状态、twist。
 * 滤波/差分等内部状态封装在 app_chassis_feedback.c。 */
ChassisFeedback_s g_chassis_feedback;

/*---------------- 力控 ----------------*/
/* 闭环（PID + 前馈）在 app_chassis_force_loop.c，方案开关见 app_chassis_force_def.h 的
 * CHASSIS_CLOSED_LOOP_SCHEME 与 CHASSIS_FORCE_* 宏。
 * 前馈三组系数（摩擦/惯性/重力）按用户要求先全 0；
 * 标定后在这里改，或运行期用调试器改。 */
ChassisForceCfg_s g_chassis_force_cfg = {
    .friction_x = 0.0f, // N
    .friction_y = 0.0f, // N
    .friction_w = 0.0f, // N·m

    .mass = 0.0f,      // kg
    .inertia_z = 0.0f, // kg·m²

    .gravity_x = 0.0f, // N
    .gravity_y = 0.0f, // N
};

/* 实测底盘速度：vx/vy 来自轮速正解，w 由 IMU 底盘系 z 轴角速度覆盖 */
vector3_t g_chassis_twist_fb;
/* 力控输出广义力 (fx, fy, mw) */
vector3_t g_chassis_wrench;
/* 力分配输出：wheel_torque = 各轮**轮缘**驱动力矩 [N·m]（沿当前舵角，正=沿轮向+），
 * lateral_force = 舵轮侧向接触力 [N]。轮序 = 左/右舵；下发前还要经换向决策。 */
LibChassisTorque_s g_chassis_torque;

/* 换向决策：0 初始化 = combo 0（两轮都不反）、无历史方案、未锁定 */
ChassisCostSel_s g_chassis_cost_sel;

/*---------------- 校准 ----------------*/

uint8_t g_rudder_l_gate_state; // 上一拍光电门电平（判 1→0 下降沿）
uint8_t g_rudder_r_gate_state;
uint8_t g_rudder_l_inited; // 1 = 左舵已在光电门沿完成零点标定
uint8_t g_rudder_r_inited;

float g_rudder_l_setref; // 本拍舵向给定（电机侧 rad 或速度，取决于状态）
float g_rudder_r_setref;
float g_wheel_l_setref; // 本拍驱动轮给定力矩 (Nm)
float g_wheel_r_setref;

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

/*============================ 光电门 ============================*/
/* 左舵 PA0 → GPIO_PWM_1，右舵 PA2 → GPIO_PWM_2（DM_MC02 映射，见旧 app 的实机注释）。
 * 回调置 NULL，在控制任务里轮询（不占 EXTI 分发）；取 1→0 下降沿作零点。 */
GPIO_INSTANCE_DEF(rudder_l_gate_io);
GPIO_INSTANCE_DEF(rudder_r_gate_io);

/*============================ 云台↔底盘通信 ============================*/
/* 实例、收发数据与类型都在本文件/def.h；
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

/*============================ 杆臂补偿 ============================*/
/* a_imu = a_center + α×r + ω×(ω×r)（r = IMU 相对底盘中心的位置）
 * ⇒ a_center = a_imu − α×r − ω×(ω×r)
 * α 由底盘系 gyro 差分得到；差分对噪声敏感，先经一阶低通再乘 r。
 * 滤波/限幅参数见 app_chassis_force_def.h 的 CHASSIS_IMU_ALPHA_*；
 * 安装矩阵/IMU 标定值/观测状态都在本文件。 */

/*============================ 云台链路在线 ============================*/
/* 看门狗实例内嵌在 media 基类里，收帧时由 comm 层统一喂狗。
 * daemon_reload 为 0 时后端会把阈值提升为 100ms；未挂 daemon 则按在线处理。 */
static uint8_t GimbalCommIsOnline(void)
{
    const CommMedia *media = (const CommMedia *)gimbal_comm.media; // 首成员即 CommMedia 基类
    return (uint8_t)((media != NULL && media->daemon != NULL) ? DaemonIsOnline(media->daemon) : 1);
}

/*============================ 舵角：速度逆解 + 力投影 ============================*/
/**
 * @brief 余弦优化系数：max(cos(目标舵角 − 实际舵角), 0)
 * @param angle_error 舵向误差绝对值 (rad)
 * @return 驱动轮力矩降额系数，0~1；误差 ≥90° 时钳 0，不反向出力
 * @note 与旧代码 Steer_Motor_Kinematics_Nearest_Transposition 的
 *       Target_Wheel_Omega *= max(cos(Δ), 0) 对齐。
 */
static float ChassisCosineAlign(float angle_error)
{
    const float c = Lib_Math_Cos(angle_error);
    return (c > 0.0f) ? c : 0.0f;
}

/**
 * @brief 在 φ 与 φ+180° 两个等价轮向中，选离当前舵角最近的解
 * @param wheel   轮序
 * @param phi_des 速度逆解给出的期望轮向 (rad，任意区间)
 * @param phi_cur 当前舵角 (rad，连续)
 * @return 目标舵角 (rad，与 phi_cur 连续的表示)
 * @note 角度死区 CHASSIS_COST_ANGLE_DEADBAND 用于平手时保持上一拍分支，
 *       避免在 |Δθ|≈90° 边界逐拍翻转。
 */
static float ChassisNearestSteer(uint8_t wheel, float phi_des, float phi_cur)
{
    ChassisCostSel_s *sel = &g_chassis_cost_sel;
    const float d_fwd = Lib_Math_WrapAngleNegPIToPI(phi_des - phi_cur);
    const float d_rev = Lib_Math_WrapAngleNegPIToPI(phi_des + M_PI - phi_cur);
    const float abs_fwd = Lib_Math_Fabs(d_fwd);
    const float abs_rev = Lib_Math_Fabs(d_rev);

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
        rev = (sel->valid != 0u) ? sel->reverse[wheel] : 0u;
    }

    sel->reverse[wheel] = rev;
    const float dphi = (rev != 0u) ? d_rev : d_fwd;
    sel->angle_error[wheel] = Lib_Math_Fabs(dphi);
    return phi_cur + dphi;
}

/**
 * @brief 速度逆解定舵角 + 把广义力投影到这两个轮向，只更新轮向力矩
 * @param cmd    云台速度指令（车体系 vx 前 / vy 左 / w 逆时针）
 * @param st     本拍实际状态（舵角反馈）
 * @param wrench 期望广义力 W = (fx [N], fy [N], mw [N·m])
 * @note 架构：舵角完全由 (vx,vy,w) 的速度逆解决定；力分配不再用
 *       atan2(f_lat, f_drive) 改舵角，也不使用侧向力。
 * @note 力分配用**目标舵角**构造 J 的驱动行，输出的 wheel_torque 是沿目标
 *       轮向的带符号轮缘力矩；侧向行仍参与最小范数分配，但结果被丢弃。
 * @note 两轮中某一轮的回转中心速度低于 LIB_CHASSIS_STEER_SPEED_EPS 时，
 *       速度逆解不给方向：该轮保持当前舵角，力仍沿当前轮向出力。
 * @note 分支：φ 与 φ+180° 等价，选离当前角更近的；目标角与当前角连续。
 */
static void ChassisVelocitySteerSelect(const gimbal2chassis_data_t *cmd, const LibChassisState_s *st, vector3_t wrench)
{
    ChassisCostSel_s *sel = &g_chassis_cost_sel;
    LibChassisRef_s ref;
    const vector3_t twist_cmd = {cmd->vx, cmd->vy, cmd->w};

    /* 1) 速度逆解：期望轮向 = 该轮回转中心的接触点速度方向 */
    LibChassisInverse(&s_chassis, twist_cmd, &ref);

    LibChassisState_s st_target = *st;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if (ref.wheel_speed[i] <= 0.0f)
        {
            /* 回转中心速度≈0：速度逆解不定义方向，保持当前舵角 */
            st_target.steer_angle[i] = st->steer_angle[i];
            sel->angle_target[i] = st->steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->reverse[i] = (sel->valid != 0u) ? sel->reverse[i] : 0u;
        }
        else
        {
            const float phi_t = ChassisNearestSteer(i, ref.steer_target[i], st->steer_angle[i]);
            st_target.steer_angle[i] = phi_t;
            sel->angle_target[i] = phi_t;
        }
    }

    /* 2) 力分解：用目标舵向构造驱动行，只取驱动行力矩。
     *    侧向行只是让 (JᵀJ) 可逆、保持最小范数分配的形状；其输出 lateral_force
     *    在本架构里不参与舵向，也不再换算成舵向电机力矩。 */
    LibChassisTorque_s alloc;
    if (LibChassisAllocateTorque(&s_chassis, &st_target, wrench, &alloc) != 0)
    {
        /* 几何退化：本轮不出力，但舵向仍按速度逆解走 */
        g_chassis_torque = (LibChassisTorque_s){0};
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->wheel_torque[i] = 0.0f;
            sel->wheel_align[i] = ChassisCosineAlign(sel->angle_error[i]);
        }
    }
    else
    {
        g_chassis_torque = alloc;
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->wheel_torque[i] = alloc.wheel_torque[i];
            /* 余弦优化：力按目标舵向分配，舵向还没追上时按误差余弦降额，
             * 避免驱动轮在错误方向上顶满力。90° 及以上钳 0（不反向出力）。 */
            sel->wheel_align[i] = ChassisCosineAlign(sel->angle_error[i]);
        }
    }

    sel->locked = 0u;
    sel->valid = 1u;
    for (uint8_t k = 0u; k < 4u; k++)
    {
        sel->cost[k] = 0.0f; // 不再做组合代价，遥测通道保留
    }
    sel->combo = (uint8_t)((sel->reverse[LIB_CHASSIS_HALF_RUDDER_L] & 0x1u) | ((sel->reverse[LIB_CHASSIS_HALF_RUDDER_R] & 0x1u) << 1));
}

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

/*============================ 顶层状态机回调（app 侧动作） ============================*/
/* 状态迁移/计时/派发在 app_chassis_fsm.c；这里只提供四个条件采样和进入动作，
 * 模块不直接碰电机/通信实例。
 *
 * 状态 = 底盘"现在能干的事"，不是云台下发的档位：
 *   CAL   上电校准：两舵轮扫描找光电门零点（此时不理会云台在线与否）
 *   FAULT 校准失败（连续超时）：失能 + 给定清零，终态（需复位重启再标）
 *   SAFE  已标定但不可动：云台掉线 或 云台下发 stop —— 失能 + 给定清零 + 作废指令
 *   RUN   已标定且可动：力控/解算入口（正解 → 广义力 → 分配 → 换向决策 → 下发）
 * ⚠ 校准是"上电即可做、与云台无关"的第一步：云台常没上电，故 CAL 不受 online 影响。 */

/*---------------- 状态条件采样 ----------------*/
static uint8_t ChassisInputCalibrated(void *user)
{
    (void)user;
    return (uint8_t)(g_rudder_l_inited && g_rudder_r_inited);
}

static uint8_t ChassisInputOnline(void *user)
{
    (void)user;
    return (uint8_t)GimbalCommIsOnline();
}

static uint8_t ChassisInputStopped(void *user)
{
    (void)user;
    /* 档位取 g_rudder_mode_rx（接收回调写的原始副本），**不能取 g_gimbal2chassis_data.enabled**：
     * 采样发生在状态动作之后（SAFE 先把指令清零作废，再走到这里），若判据用同一份数据，
     * SAFE 每拍都会读到自己刚写下的 stop ⇒ stopped 恒 1 ⇒ 永远出不去。
     * 档位是"对端说了什么"，只能由接收回调更新，与"本拍要不要执行"必须分开存。 */
    return (uint8_t)(g_gimbal_mode_rx == robot_mode_stop);
}

/*---------------- 状态进入动作 ----------------*/
/* 进入运行态：舵向从"纯速度环扫描"切回"位置+速度环"（只改掩码，不重配 PID）。 */
static void ChassisActionRunEntry(void *user)
{
    (void)user;
    rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
    rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
}

/* 进入故障（终态）：上报一次。失能 + 给定清零在主循环 FAULT 分支里做。 */
static void ChassisActionFaultEntry(void *user)
{
    (void)user;
    BSPLOG(&g_chassis_log, LOG_LEVEL_ERROR, "cal fault: no photogate edge after %u retries x %u ticks; chassis disabled, reset MCU to retry",
           (unsigned)CHASSIS_CAL_MAX_RETRIES, (unsigned)CHASSIS_CAL_TIMEOUT_TICKS);
}

/* 单次校准超时：只重试，不判死。 */
static void ChassisActionCalTimeout(uint8_t retries, void *user)
{
    (void)user;
    BSPLOG(&g_chassis_log, LOG_LEVEL_WARNING, "cal timeout #%u/%u, keep scanning", (unsigned)retries, (unsigned)CHASSIS_CAL_MAX_RETRIES);
}

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

    /* ---- 反馈处理：配置 + 复位 ----
     * IMU 安装位置由模块从齐次变换算出（原点经旋转+平移），滤波/差分状态在模块内部。 */
    ChassisFeedbackCfg_s feedback_cfg = {
        .imu_to_chassis = &g_imu_to_chassis,
        .chassis = &s_chassis,
        .wheel_dir = {(float)CHASSIS_WHEEL_L_DIR, (float)CHASSIS_WHEEL_R_DIR},
        .steer_gear_ratio_inv = CHASSIS_STEER_GEAR_RATIO_INV,
        .drive_reduction = CHASSIS_DRIVE_REDUCTION,
    };
    ChassisFeedbackInit(&feedback_cfg);

    /* ---- 闭环：PID 反馈 + 前馈 ----
     * 前馈系数引用全局配置，运行期仍可用调试器改。 */
    const ChassisForceLoopCfg_s force_loop_cfg = {
        .force_cfg = &g_chassis_force_cfg,
    };
    ChassisForceLoopInit(&force_loop_cfg);

    /* ---- 顶层状态机：上电从校准起，等两个光电门标定完再判在线/使能 ----
     * 迁移/计时/派发在 app_chassis_fsm.c，这里只提供条件采样与进入动作。 */
    const ChassisFsmCfg_s chassis_fsm_cfg = {
        .is_calibrated = ChassisInputCalibrated,
        .is_online = ChassisInputOnline,
        .is_stopped = ChassisInputStopped,
        .on_cal_timeout = ChassisActionCalTimeout,
        .on_run_entry = ChassisActionRunEntry,
        .on_fault_entry = ChassisActionFaultEntry,
        .user = NULL,
    };
    ChassisFsmInit(&chassis_fsm_cfg);
}

void AppChassisRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;

    /* IMU 纯观测：每拍更新。vofa_enable=1 时 Update 内部会顺带发一帧 VOFA，
     * 直接用 VOFA+ 录 CSV，喂 bmi088_calib.py 标定陀螺零偏。 */
    BMI088KalmanUpdate(&bmi088);
    const BMI088Kalman_Data_t imu = BMI088KalmanGetData(&bmi088);

    /* 反馈处理：IMU 安装矩阵旋转 → 底盘系、角加速度差分+低通、杆臂补偿到质心 */
    const ChassisFeedbackImuInput_s feedback_imu = {
        .gyro = {imu.gyro[0], imu.gyro[1], imu.gyro[2]},
        .acc = {imu.acc[0], imu.acc[1], imu.acc[2]},
        .yaw_rate = imu.yaw_rate,
        .dt = imu.dt,
        .valid = imu.valid,
    };
    ChassisFeedbackUpdateImu(&feedback_imu, &g_chassis_feedback);

    /*============================ 电机反馈 ============================*/
    /* 每拍都读、都喂给控制核：失能期间也必须持续处理反馈，否则手动转动舵向电机后
     * 多圈累加与真实位置脱节，再使能的第一拍位置会跳。
     * 方向：反馈一律先取协议原始量，舵向由 drvlib 的 feedback_direction 统一修正；
     *       驱动轮不走 drvlib，由 feedback 模块按 CHASSIS_WHEEL_x_DIR 换算到 app 正方向。 */
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
    const ChassisFeedbackWheelInput_s feedback_wheel = {
        .wheel = {
            [APP_WHEEL_L] =
                {
                    .speed = wheel_l_data.speed,
                    .torque = wheel_l_data.torque,
                    .current = wheel_l_data.current,
                    .temperature = wheel_l_data.temperature,
                    .error = wheel_l_data.echo,
                },
            [APP_WHEEL_R] =
                {
                    .speed = wheel_r_data.speed,
                    .torque = wheel_r_data.torque,
                    .current = wheel_r_data.current,
                    .temperature = wheel_r_data.temperature,
                    .error = wheel_r_data.echo,
                },
        },
    };
    ChassisFeedbackUpdateWheels(&feedback_wheel, &g_chassis_feedback);

    /*============================ 按状态生成给定 ============================*/
    /* 顺序约定：① 状态给定（舵向在本拍喂反馈 + 算下发量）→ ② 校准判光电门（用 ① 刚刷新
     * 的 data.position）→ ③ 采样条件并（变化时）派发 → ④ 下发。
     * ⚠ 每拍每个舵向只跑一次 SetRef；状态变化从下一拍生效。 */
    const ChassisState_e state = ChassisFsmCurrent();
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
            /* 舵向：决策给的是舵轮侧 rad，控制核要电机侧 rad */
            rudder_l_ref = g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_L] * CHASSIS_STEER_GEAR_RATIO;
            rudder_r_ref = g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_R] * CHASSIS_STEER_GEAR_RATIO;

            /* 轮向：带符号轮缘力矩 × max(cos(目标−实际), 0)（余弦优化）。
             * 比例取上一拍决策算好的值（2ms 滞后可忽略）；误差 ≥90° 时降为 0。 */
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

    /*============================ 反馈正解 + 力控 ============================*/
    /* 轮速正解：舵角取本拍 SetRef 刚刷新的 data.position（电机侧 rad），
     * 轮速用 app 正方向；w 由 IMU 覆盖，均在 feedback 模块内完成。 */
    const float steer_motor_pos[LIB_CHASSIS_HALF_RUDDER_NUM] = {
        [LIB_CHASSIS_HALF_RUDDER_L] = (float)rudder_l_ctrl.data.position,
        [LIB_CHASSIS_HALF_RUDDER_R] = (float)rudder_r_ctrl.data.position,
    };
    ChassisFeedbackUpdateKinematics(steer_motor_pos, &g_chassis_feedback);
    g_chassis_twist_fb = g_chassis_feedback.twist;

    /* 速度指令 → 广义力 → 舵角/轮向力矩。
     * 闭环在 app_chassis_force_loop.c：非 RUN 时模块内部清 PID 状态并输出 0，
     * 但每拍仍记录指令供下一拍算惯性前馈。
     * 舵角：由 (vx,vy,w) 的速度逆解直接给出，力分配不再改变它；
     * 轮向：用目标舵向构造 J 的驱动行，把 3 个广义力投影到这两个方向上，
     *       只下发驱动行力矩，侧向力丢弃。结果见 g_chassis_cost_sel，
     *       下一拍由 RUN 分支下发。 */
    const ChassisForceLoopInput_s force_loop_in = {
        .cmd = {g_gimbal2chassis_data.vx, g_gimbal2chassis_data.vy, g_gimbal2chassis_data.w},
        .twist_fb = g_chassis_twist_fb,
        .dt = dt_s,
    };
    g_chassis_wrench = ChassisForceLoopUpdate(&force_loop_in, (uint8_t)(state == CHASSIS_STATE_RUN));
    if (state == CHASSIS_STATE_RUN)
    {
        ChassisVelocitySteerSelect(&g_gimbal2chassis_data, &g_chassis_feedback.state, g_chassis_wrench);
    }
    else
    {
        g_chassis_torque = (LibChassisTorque_s){0};
        /* 离开运行态即作废换向历史：下次使能重新决策（不沿用急停前的方案） */
        g_chassis_cost_sel = (ChassisCostSel_s){0};
    }

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
        VofaSetChannel(2, g_chassis_feedback.state.steer_angle[LIB_CHASSIS_HALF_RUDDER_L] * r2d);                         // 左舵实际 (deg)
        VofaSetChannel(3, Lib_Math_WrapAngleNegPIToPI(g_chassis_cost_sel.angle_target[LIB_CHASSIS_HALF_RUDDER_R]) * r2d); // 右舵目标 (deg)
        VofaSetChannel(4, g_chassis_feedback.state.steer_angle[LIB_CHASSIS_HALF_RUDDER_R] * r2d);                         // 右舵实际 (deg)
        VofaSetChannel(5, g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_L]);                                    // 左轮目标 (N·m)
        VofaSetChannel(6, g_chassis_feedback.wheel[APP_WHEEL_L].torque * CHASSIS_DRIVE_REDUCTION);                        // 左轮实测 (N·m)
        VofaSetChannel(7, g_chassis_cost_sel.wheel_torque[LIB_CHASSIS_HALF_RUDDER_R]);                                    // 右轮目标 (N·m)
        VofaSetChannel(8, g_chassis_feedback.wheel[APP_WHEEL_R].torque * CHASSIS_DRIVE_REDUCTION);                        // 右轮实测 (N·m)
        VofaSetChannel(9, (float)g_chassis_cost_sel.combo);                                                               // 换向方案 0~3
        VofaSetChannel(10, g_chassis_cost_sel.cost[0]);                                                                   // 组合代价 (s)
        VofaSetChannel(11, g_chassis_cost_sel.cost[1]);
        VofaSetChannel(12, g_chassis_cost_sel.cost[2]);
        VofaSetChannel(13, g_chassis_cost_sel.cost[3]);
        VofaSetChannel(14, (float)g_chassis_cost_sel.locked);                          // 历史力矩锁定（当前恒 0）
        VofaSetChannel(15, (float)state);                                              // 状态机状态
        VofaSetChannel(16, g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_L]); // 左轮余弦优化系数
        VofaSetChannel(17, g_chassis_cost_sel.wheel_align[LIB_CHASSIS_HALF_RUDDER_R]); // 右轮余弦优化系数
        /* CH18~28：命令 / 反馈 / 力控中间量。换向乱转的定位关键是
         * "期望力方向为什么转" —— 对比 CH18~20（指令）与 CH21~23（反馈），
         * 再看 CH24~26 是否跟着转，就能区分指令在转、反馈在转还是分配在转。 */
        VofaSetChannel(18, g_gimbal2chassis_data.vx);                                              // 指令 vx (m/s)
        VofaSetChannel(19, g_gimbal2chassis_data.vy);                                              // 指令 vy (m/s)
        VofaSetChannel(20, g_gimbal2chassis_data.w);                                               // 指令 w (rad/s)
        VofaSetChannel(21, g_chassis_twist_fb.x);                                                  // 反馈 vx (m/s，轮速正解)
        VofaSetChannel(22, g_chassis_twist_fb.y);                                                  // 反馈 vy (m/s，轮速正解)
        VofaSetChannel(23, g_chassis_twist_fb.z);                                                  // 反馈 w (rad/s，IMU)
        VofaSetChannel(24, g_chassis_wrench.x);                                                    // 期望力 fx (N)
        VofaSetChannel(25, g_chassis_wrench.y);                                                    // 期望力 fy (N)
        VofaSetChannel(26, g_chassis_wrench.z);                                                    // 期望力矩 mw (N·m)
        VofaSetChannel(27, g_chassis_feedback.wheel[APP_WHEEL_L].speed / CHASSIS_DRIVE_REDUCTION); // 左驱动轮轮速 (rad/s)
        VofaSetChannel(28, g_chassis_feedback.wheel[APP_WHEEL_R].speed / CHASSIS_DRIVE_REDUCTION); // 右驱动轮轮速 (rad/s)
        VofaSend();
    }
}
