/**
 * @file app_chassis_feedback.c
 * @brief 底盘反馈处理实现（方案 0 独立 / 方案 1 ESKF 融合）
 *
 * 处理顺序：
 *   1. IMU：lib_imu_transform 把陀螺/加速度旋转到底盘系并做杆臂补偿；
 *      方案 1 另按姿态扣重力得到 acc_lin，并用时间戳判数据新鲜度；
 *   2. 轮向：协议原始量 × CHASSIS_WHEEL_x_DIR → app 正方向（带帧时间戳）；
 *   3. 正解：舵角/轮速换到舵轮侧 → LibChassisForward → vx/vy，
 *      w 由 IMU 底盘系 z 轴角速度覆盖；
 *      方案 1 把轮速正解当量测喂 ESKF（见 app_chassis_eskf.c），输出替代 vx/vy。
 */

#include "app_chassis_feedback.h"

#include "app_chassis_force_def.h" // CHASSIS_IMU_ALPHA_* / CHASSIS_FEEDBACK_SCHEME
#include "bsp_sys_status.h"
#include "lib_imu_transform.h"
#include "lib_math.h"

#if (CHASSIS_FEEDBACK_SCHEME != 0) && (CHASSIS_FEEDBACK_SCHEME != 1)
#error "invalid CHASSIS_FEEDBACK_SCHEME"
#endif

#if CHASSIS_FEEDBACK_SCHEME == 1
#include "app_chassis_eskf.h"
#include "bsp_dwt.h"
#endif

/* 轮序必须一致：反馈正解直接用 app 轮序索引驱动轮 */
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_L == (int)APP_WHEEL_L, "wheel order mismatch: L");
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_R == (int)APP_WHEEL_R, "wheel order mismatch: R");

static struct
{
    ChassisFeedbackCfg_s cfg;
    LibImuTransform_Instance_s imu_tf; // IMU 系→底盘系 + 杆臂补偿（差分/滤波状态在模块内）
} s_feedback;

void ChassisFeedbackInit(const ChassisFeedbackCfg_s *cfg)
{
    s_feedback.cfg = *cfg;

    /* 从 4x4 齐次变换提取：
     *   平移 t          = IMU 原点在底盘系的位置；
     *   左上 3x3 的三列 = IMU x/y/z 轴在底盘系的方向（原样取用，不转置）。 */
    const matrix4_t imu_to_chassis = *cfg->imu_to_chassis;
    const LibImuTransform_Config_s tf_cfg = {
        .pos = Lib_Math_Mat4MulPoint(imu_to_chassis, (vector3_t){0.0f, 0.0f, 0.0f}),
        .x_axis = (vector3_t){imu_to_chassis.data[0][0], imu_to_chassis.data[1][0], imu_to_chassis.data[2][0]},
        .y_axis = (vector3_t){imu_to_chassis.data[0][1], imu_to_chassis.data[1][1], imu_to_chassis.data[2][1]},
        .z_axis = (vector3_t){imu_to_chassis.data[0][2], imu_to_chassis.data[1][2], imu_to_chassis.data[2][2]},
        .alpha_lpf_rc = CHASSIS_IMU_ALPHA_LPF_RC,
        .alpha_limit = CHASSIS_IMU_ALPHA_LIMIT,
    };
    BSP_ASSERT_APP_CALL(LibImuTransformInit(&s_feedback.imu_tf, &tf_cfg));

#if CHASSIS_FEEDBACK_SCHEME == 1
    ChassisEskfInit();
#endif
    ChassisFeedbackReset();
}

void ChassisFeedbackReset(void)
{
    LibImuTransformReset(&s_feedback.imu_tf);
#if CHASSIS_FEEDBACK_SCHEME == 1
    ChassisEskfReset();
#endif
}

void ChassisFeedbackUpdateImu(const ChassisFeedbackImuInput_s *in, ChassisFeedback_s *out)
{
    /* IMU 系 → 底盘系、角加速度差分/限幅/低通、杆臂补偿统一由 lib_imu_transform 完成。 */
    const LibImuTransform_Input_s tf_in = {
        .gyro = in->gyro,
        .acc = in->acc,
        .dt = in->dt,
        .valid = in->valid,
    };
    LibImuTransform_Output_s tf_out;
    (void)LibImuTransformUpdate(&s_feedback.imu_tf, &tf_in, &tf_out);

    out->imu.gyro = tf_out.gyro;
    out->imu.acc = tf_out.acc;
    out->imu.euler = in->euler;
    out->imu.yaw_rate = in->yaw_rate; // world 竖直轴，与安装方向无关
    out->imu.fresh = tf_out.fresh;
    out->imu.pos = tf_out.pos;
#if CHASSIS_FEEDBACK_SCHEME == 1
    /* 方案 1：数据超时也判无效，避免冻结的陀螺样本被当作真值 */
    {
        const uint64_t now_us = DWT_GetTimeUs();
        const uint8_t stale =
            (uint8_t)((in->timestamp_us == 0u) || (now_us < in->timestamp_us) || ((now_us - in->timestamp_us) >= (uint64_t)CHASSIS_ESKF_IMU_TIMEOUT_US));
        out->imu.valid = (uint8_t)((tf_out.valid != 0u) && (stale == 0u));
    }
#else
    out->imu.valid = tf_out.valid;
#endif

#if CHASSIS_FEEDBACK_SCHEME == 1
    /* 姿态去重力：世界 +z 在 IMU 系 = (−sin p, cos p·sin r, cos p·cos r)，
     * 再用同一安装矩阵转到车体系。euler 是 IMU 自身轴向（倒装时 roll≈π）。 */
    {
        float sr, cr, sp, cp;
        Lib_Math_SinCos(in->euler.roll, &sr, &cr);
        Lib_Math_SinCos(in->euler.pitch, &sp, &cp);
        const vector3_t up_imu = {-sp, cp * sr, cp * cr};
        const vector3_t up_chassis = Lib_Math_Mat4MulDir(*s_feedback.cfg.imu_to_chassis, up_imu);
        out->imu.acc_lin = Lib_Math_Vec3Sub(out->imu.acc, Lib_Math_Vec3Scale(up_chassis, CHASSIS_ESKF_G));
    }
#else
    out->imu.acc_lin = (vector3_t){0.0f, 0.0f, 0.0f};
#endif

    out->valid = out->imu.valid; // 融合方案的速度有效位由正解阶段填
}

void ChassisFeedbackUpdateWheels(const ChassisFeedbackWheelInput_s *in, ChassisFeedback_s *out)
{
    for (uint8_t i = 0; i < (uint8_t)APP_WHEEL_NUM; i++)
    {
        const float dir = s_feedback.cfg.wheel_dir[i];
        out->wheel[i].speed = in->wheel[i].speed * dir;
        out->wheel[i].torque = in->wheel[i].torque * dir;
        out->wheel[i].current = in->wheel[i].current * dir;
        out->wheel[i].temperature = in->wheel[i].temperature;
        out->wheel[i].error = in->wheel[i].error;
        out->wheel[i].timestamp_us = in->wheel[i].timestamp_us;
    }
}

#if CHASSIS_FEEDBACK_SCHEME == 1
/**
 * @brief 轮速帧是否新鲜：时间戳非 0 且距现在未超时
 */
static uint8_t ChassisWheelFrameFresh(uint64_t ts_us, uint64_t now_us)
{
    return (uint8_t)((ts_us != 0u) && (now_us >= ts_us) && ((now_us - ts_us) < (uint64_t)CHASSIS_ESKF_WHEEL_TIMEOUT_US));
}
#endif

void ChassisFeedbackUpdateKinematics(const float steer_motor_pos[LIB_CHASSIS_HALF_RUDDER_NUM], float task_dt_s, ChassisFeedback_s *out)
{
    (void)task_dt_s; /* 方案 0 不用，避免 -Wunused-parameter */

    /* 轮速正解：舵角取本拍 SetRef 刚刷新的 data.position（电机侧 rad），
     * 换算到舵轮侧；轮速用 app 正方向（= 沿当前舵角推动车体）。 */
    out->state = (LibChassisState_s){0};
    for (uint8_t i = 0; i < (uint8_t)LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        out->state.steer_angle[i] = steer_motor_pos[i] * s_feedback.cfg.steer_gear_ratio_inv;
        out->state.wheel_speed[i] = out->wheel[i].speed / s_feedback.cfg.drive_reduction;
    }

    vector3_t twist;
    const uint8_t forward_ok = (uint8_t)(LibChassisForward(s_feedback.cfg.chassis, &out->state, &twist) == 0);
    if (forward_ok != 0u)
    {
        out->twist = twist;
    }
    else
    {
        /* 几何退化（JᵀJ 奇异）：不喂假反馈 */
        out->twist = (vector3_t){0.0f, 0.0f, 0.0f};
    }

#if CHASSIS_FEEDBACK_SCHEME == 0
    /* 独立方案：w 反馈用 IMU 底盘系 z 轴角速度（与轮速正解解耦；IMU 不可用时置 0） */
    out->twist.z = (out->imu.valid != 0u) ? out->imu.gyro.z : 0.0f;
    out->velocity_valid = forward_ok;
    out->zupt = 0u;
    out->slip = 0u;
#else
    /* 融合方案：IMU 预测 + 轮速正解量测，w 仍取 IMU（不让滤波估计反馈到 yaw 环） */
    const uint64_t now_us = DWT_GetTimeUs();
    const uint64_t ts_l = out->wheel[APP_WHEEL_L].timestamp_us;
    const uint64_t ts_r = out->wheel[APP_WHEEL_R].timestamp_us;
    const uint8_t fresh_l = ChassisWheelFrameFresh(ts_l, now_us);
    const uint8_t fresh_r = ChassisWheelFrameFresh(ts_r, now_us);
    const uint8_t wheel_valid = (uint8_t)((fresh_l != 0u) && (fresh_r != 0u) && (forward_ok != 0u));

    float wheel_speed_abs_max = 0.0f;
    for (uint8_t i = 0; i < (uint8_t)LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        const float rim = Lib_Math_Fabs(out->state.wheel_speed[i]) * CHASSIS_WHEEL_RADIUS;
        if (rim > wheel_speed_abs_max)
        {
            wheel_speed_abs_max = rim;
        }
    }

    const ChassisEskfInput_s eskf_in = {
        .task_dt_s = task_dt_s,
        .acc_lin = out->imu.acc_lin,
        .gyro_z = out->imu.gyro.z,
        .yaw_rate = out->imu.yaw_rate,
        .imu_valid = out->imu.valid,
        .wheel_valid = wheel_valid,
        .wheel_timestamp_us = (ts_l > ts_r) ? ts_l : ts_r,
        .wheel_twist = twist,
        .wheel_speed_abs_max = wheel_speed_abs_max,
    };
    ChassisEskfOutput_s eskf_out;
    ChassisEskfUpdate(&eskf_in, &eskf_out);

    out->twist.x = eskf_out.velocity.x;
    out->twist.y = eskf_out.velocity.y;
    out->twist.z = (out->imu.valid != 0u) ? out->imu.gyro.z : 0.0f;
    out->velocity_valid = eskf_out.velocity_valid;
    out->zupt = eskf_out.zupt;
    out->slip = eskf_out.slip;
#endif
}
