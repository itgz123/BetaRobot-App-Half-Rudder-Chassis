#include "app_chassis_force.h"
#include "app_cfg.h"
#include "app_chassis_force_def.h" // 类型 + 全局变量（实体在 .def.c）
//
#include "lib_math.h"
#include "bsp_sys_status.h"
#include "drvs_djimotor_broadcast.h"
#include "drvs_lkmotor_broadcast.h"
#include "drvlib_bmi088_kalman.h"

#include "drv_comm.h"
#include "comm_media_can_idseq.h"

#include <string.h>

/*============================ 电机实例 ============================*/
// 舵向：DJI M3508（C620），id2 / id4，共用 0x200 广播帧
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(rudder_motor_group);

// 驱动轮：LK MF7015 直驱，id1 / id2，共用 0x280 转矩广播帧
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);
DRVS_LKMOTOR_BROADCAST_GROUP_DEF(wheel_motor_group);

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
}

void AppChassisRun(float dt, uint64_t time_stamp)
{
    (void)dt;
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

    /* 云台通信：每周期回一帧。当前还没有正解，vx/vy/w 反馈先保持 0；
     * 接入轮速正解后再填写。即使全 0 也必须发——云台侧靠这帧维持链路在线。 */
    CommSend(&gimbal_comm, (uint8_t *)&g_chassis2gimbal_data);
}
