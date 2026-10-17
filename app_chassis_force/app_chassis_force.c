#include "app_chassis_force.h"
#include "app_cfg.h"
#include "robot_def.h"
//
#include "lib_math.h"
#include "bsp_sys_status.h"
#include "drvs_djimotor_broadcast.h"
#include "drvs_lkmotor_broadcast.h"
#include "drvlib_bmi088_kalman.h"

/*============================ 电机实例 ============================*/
// 舵向：DJI M3508（C620），id2 / id4，共用 0x200 广播帧
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(rudder_motor_group);

// 驱动轮：LK MF7015 直驱，id1 / id2，共用 0x280 转矩广播帧
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
DRVS_LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);
DRVS_LKMOTOR_BROADCAST_GROUP_DEF(wheel_motor_group);

/*============================ 底盘 IMU ============================*/
BMI088_KALMAN_INSTANCE_DEF(bmi088);

/*============================ IMU → 底盘系 ============================*/
/* 4x4 齐次变换：p_chassis = R · p_imu + t。
 * R 的三列 = IMU 的 x/y/z 轴在底盘系中的方向（robot_def.h）：
 *     x_imu = (−1, 1, 0)/√2   y_imu = (1, 1, 0)/√2   z_imu = (0, 0, −1)
 *     t     = IMU 中心在底盘系的位置。
 * 陀螺/加速度是方向量，用 Lib_Math_Mat4MulDir（不带平移）；
 * 需要杆臂补偿时用 Lib_Math_Mat4MulPoint 把 IMU 系里的点转到地底盘系。 */
static const matrix4_t s_imu_to_chassis = {
    .data =
        {
            {-0.70710678f, 0.70710678f, 0.0f, CHASSIS_IMU_POS_X},
            {0.70710678f, 0.70710678f, 0.0f, CHASSIS_IMU_POS_Y},
            {0.0f, 0.0f, -1.0f, CHASSIS_IMU_POS_Z},
            {0.0f, 0.0f, 0.0f, 1.0f},
        },
};

/* IMU 量转到**底盘系**后的结果（旋转用安装矩阵，平移量见 pos） */
typedef struct
{
    vector3_t gyro; // 底盘系角速度 (rad/s)
    vector3_t acc;  // 底盘系比力 (m/s²)，测点在 IMU 处（未做杆臂补偿）
    vector3_t pos;  // IMU 安装点在底盘系的位置 (m)，固定
    float yaw_rate; // 底盘系绕竖直轴角速度 (rad/s)（world 系，与安装方向无关）
    uint8_t valid;  // 1 = 本拍姿态可用（已播种/有数据）
} AppChassisImu_s;
static AppChassisImu_s s_imu_chassis; // 底盘系观测，供力控使用

/* 底盘车架上那颗 BMI088 的静态零偏（2026-10-17，ignore/chassis_data/imu.csv）
 * 录制条件：静置 112s、约 33℃、IMU 倒装（+z 朝下）。地面轻微不平按水平处理；
 * 陀螺零偏与安装倾角无关，故这里只标零偏，acc 的六位置标定尚未做。 */
static const BMI088_Calib_s s_bmi088_calib = {
    .gyro = {.bias = {-0.000721f, -0.000027f, -0.000445f}},
};

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
        .calib = &s_bmi088_calib,
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
    s_imu_chassis.pos = Lib_Math_Mat4MulPoint(s_imu_to_chassis, (vector3_t){0.0f, 0.0f, 0.0f});
}

void AppChassisRun(float dt, uint64_t time_stamp)
{
    (void)dt;
    (void)time_stamp;

    /* IMU 纯观测：每拍更新。vofa_enable=1 时 Update 内部会顺带发一帧 VOFA，
     * 直接用 VOFA+ 录 CSV，喂 bmi088_calib.py 标定陀螺零偏。 */
    BMI088KalmanUpdate(&bmi088);
    const BMI088Kalman_Data_t imu = BMI088KalmanGetData(&bmi088);

    /* IMU 系 → 底盘系（纯旋转；加速度测点仍在 IMU 处，杆臂补偿后续再加） */
    const vector3_t gyro_imu = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    const vector3_t acc_imu = {imu.acc[0], imu.acc[1], imu.acc[2]};
    s_imu_chassis.gyro = Lib_Math_Mat4MulDir(s_imu_to_chassis, gyro_imu);
    s_imu_chassis.acc = Lib_Math_Mat4MulDir(s_imu_to_chassis, acc_imu);
    s_imu_chassis.yaw_rate = imu.yaw_rate; // world 竖直轴，与安装方向无关
    s_imu_chassis.valid = imu.valid;
}
