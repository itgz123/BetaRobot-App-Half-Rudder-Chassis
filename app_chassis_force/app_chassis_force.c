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

#include "drv_comm.h"
#include "comm_media_can_idseq.h"

#include <math.h>
#include <string.h>

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
 *   RUN   已标定且可动：力控/解算入口（当前控制尚未接入，舵向保持当前位置、轮向 0 力矩）
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
     * 位置模式    WRAP，归一化范围 ±π×减速比（舵轮一周 = 电机 8 圈）
     * position_span 单圈 2π（M3508 编码器跨度），供多圈累加
     * 方向        见 CHASSIS_RUDDER_x_DIR（motor/feedback 成对）
     * 低通        速度/电流 20ms（沿用旧整定）
     * Config 会自动给位置/速度环补 TRAPEZOID_INTEGRAL，WRAP 下再给位置环补误差归一化。 */
    DrvlibMotor_Config_s rudder_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED,
        .motor_direction = (DrvlibMotorDirection_e)CHASSIS_RUDDER_L_DIR,    // 输出方向
        .feedback_direction = (DrvlibMotorDirection_e)CHASSIS_RUDDER_L_DIR, // 反馈方向
        .position_mode = DRVLIB_MOTOR_POS_WRAP,
        .angle_limit_min = -M_PI * CHASSIS_STEER_GEAR_RATIO,
        .angle_limit_max = M_PI * CHASSIS_STEER_GEAR_RATIO,
        .position_span = 6.2831853f, // 单圈 2π
        .position_offset = 0.0f,     // 光电门标定时由 RudderCalibrateOffset 改写
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
                        .config_mask = PID_ENABLE_ERROR_NORMALIZE | // 启用误差归一化
                                       PID_ENABLE_OUTPUT_LIMIT,     // 输出限幅
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
        /* 1 = 模块每帧把 CH1~CH15 写好并自己发帧（布局见 drvlib_bmi088_kalman.h）。
         * 该布局就是标定脚本 drvlib/drvlib_bmi088_calib/bmi088_calib.py 的输入：
         * CH4~6 是扣掉旧零偏后的残差，脚本按"新零偏 = CH13~15 + CH4~6 均值"迭代。 */
        .vofa_enable = 1,
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
    g_wheel_fb[APP_WHEEL_L].current = wheel_l_data.current * (float)CHASSIS_WHEEL_L_DIR;
    g_wheel_fb[APP_WHEEL_L].temperature = wheel_l_data.temperature;
    g_wheel_fb[APP_WHEEL_L].error = wheel_l_data.echo;
    g_wheel_fb[APP_WHEEL_R].speed = wheel_r_data.speed * (float)CHASSIS_WHEEL_R_DIR;
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
         * 力控/解算尚未接入：两舵向位置环归零（0 = 光电门标定后的零点，按 robot_def
         * 约定应指向车体正前方），两驱动轮同时给 app 正方向的小力矩，用于核对轮向与
         * 舵向零点（见 CHASSIS_WHEEL_CAL_TORQUE）。轮向不对改 CHASSIS_WHEEL_x_DIR；
         * 舵轮没指向正前方就改 RUDDER_ZERO_OFFSET_x_DEG。 */
        rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
        rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
        DrvlibMotorEnable(&rudder_l_ctrl);
        DrvlibMotorEnable(&rudder_r_ctrl);
        rudder_l_ref = 0.0f; // 电机侧位置 0（标定后的零点）
        rudder_r_ref = 0.0f;
        g_wheel_l_setref = CHASSIS_WHEEL_CAL_TORQUE;
        g_wheel_r_setref = CHASSIS_WHEEL_CAL_TORQUE;
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

    /* 云台通信：每周期回一帧。当前还没有正解，vx/vy/w 反馈先保持 0；
     * 接入轮速正解后再填写。即使全 0 也必须发——云台侧靠这帧维持链路在线。 */
    CommSend(&gimbal_comm, (uint8_t *)&g_chassis2gimbal_data);
}
