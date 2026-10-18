/**
 * @file app_chassis_force_def.c
 * @brief 力控底盘 app 的类型/全局变量**定义**
 *
 * 与 app_chassis_force_def.h 配对：类型和 extern 声明在 .h，实体定义在这里。
 * 业务逻辑（状态机、控制律、解算接线）不要写进本文件。
 */

#include "app_chassis_force_def.h"

#include "comm_media_can_idseq.h"
#include "comm_proto_ext.h"

/*============================ 云台↔底盘通信 ============================*/
/* CAN1 / IDSEQ / PROTO_EXT。底盘 tx_id=0x110 段、rx_id=0x100 段（云台侧对调）；
 * 线上帧 = [0xA5][seq][汉明编码区 E][CRC8][0x5A]，payload 就是下面两个业务结构体。
 * CAN1 上还有经典 CAN 的 RS05，故介质必须 CLASSIC，不能开 FD。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, EXT, EXT, gimbal2chassis_data_t, 13, chassis2gimbal_data_t, 12, UNPACK_IN_ISR);

gimbal2chassis_data_t g_gimbal2chassis_data = {0};      // 云台→底盘：速度指令 + 档位
volatile robot_mode g_gimbal_mode_rx = robot_mode_stop; // 接收回调写的档位原始副本
chassis2gimbal_data_t g_chassis2gimbal_data = {0};      // 底盘→云台：速度反馈

/*============================ 常量 ============================*/
const matrix4_t g_imu_to_chassis = {
    .data =
        {
            {CHASSIS_IMU_X_AXIS_X, CHASSIS_IMU_Y_AXIS_X, CHASSIS_IMU_Z_AXIS_X, CHASSIS_IMU_POS_X},
            {CHASSIS_IMU_X_AXIS_Y, CHASSIS_IMU_Y_AXIS_Y, CHASSIS_IMU_Z_AXIS_Y, CHASSIS_IMU_POS_Y},
            {CHASSIS_IMU_X_AXIS_Z, CHASSIS_IMU_Y_AXIS_Z, CHASSIS_IMU_Z_AXIS_Z, CHASSIS_IMU_POS_Z},
            {0.0f, 0.0f, 0.0f, 1.0f},
        },
};

const BMI088_Calib_s g_bmi088_calib = {
    .gyro = {.bias = {-0.000721f, -0.000027f, -0.000445f}},
};

/*============================ 运行状态 ============================*/
AppChassisImu_s g_imu_chassis; // 底盘系观测，供力控使用

vector3_t g_gyro_prev;     // 上一有效陀螺样本（底盘系）
vector3_t g_alpha_lpf;     // 低通后的角加速度（底盘系）
uint8_t g_gyro_prev_valid; // 0 = 还没有上一样本（首拍/掉线后）

/* 驱动轮反馈（每拍更新，失能也更新） */
AppWheelFeedback_s g_wheel_fb[APP_WHEEL_NUM];

/*============================ 力控 ============================*/
/* 反馈环参数已移到 app_chassis_force_def.h 的 CHASSIS_FORCE_* 宏（lib_pid 配置）。
 * 前馈三组系数（摩擦/惯性/重力）按用户要求先全 0。
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

vector3_t g_chassis_twist_fb;        // 实测底盘速度 (vx, vy, w)
vector3_t g_chassis_wrench;          // 力控输出 (fx, fy, mw)
LibChassisTorque_s g_chassis_torque; // 力分配输出（轮缘力矩 + 侧向力），当前不下发
vector3_t g_chassis_cmd_prev;        // 上一拍指令速度

/* 换向决策：0 初始化 = combo 0（两轮都不反）、无历史方案、未锁定 */
ChassisCostSel_s g_chassis_cost_sel;

/*============================ 校准 / 状态机 ============================*/
ChassisFsmCtx_s g_chassis_fsm_ctx;
LibFsmTableInstance_s g_chassis_fsm;

uint8_t g_rudder_l_gate_state; // 上一拍光电门电平
uint8_t g_rudder_r_gate_state;
uint8_t g_rudder_l_inited; // 1 = 左舵已标定
uint8_t g_rudder_r_inited;

float g_rudder_l_setref;
float g_rudder_r_setref;
float g_wheel_l_setref;
float g_wheel_r_setref;
