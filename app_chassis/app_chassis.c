#include "app_chassis.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h"
//
#include "drv_motor_base.h"
#include "drv_djimotor_broadcast.h"
#include "drv_lkmotor_broadcast.h"
//
#include "bsp_assert.h"

// 实例
// rudder_l/r：DJI M3508 (C620) 舵向电机
// wheel_l/r：LK MF7015 直驱轮电机
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);

void AppChassisInit(void)
{
    // 注册 CAN 实例（四个电机共用 CAN_1）
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastRegister(&rudder_l_motor));
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastRegister(&rudder_r_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_l_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_r_motor));

    // 配置 rudder_l（M3508，motor_id=2 → rx 0x202 / tx 0x200）
    DJIMotorBroadcast_Config_s rudder_l_cfg = {
        .can_e = CAN_1,
        .model = DJI_MODEL_M3508,
        .motor_id = 2,
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.02f,
        .position_offset = 0,
        .torque_constant = 1, // M3508 电流→力矩系数，待标定
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                 // TODO: 后续改为位置/速度环
            .feedback_direction = MOTOR_DIRECTION_NORMAL, // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,    // 输出方向
            .position_mode = MOTOR_POSITION_CONTINUOUS,   // 开环占位，不用位置环
            .angle_limit_max = 0,
            .angle_limit_min = 0,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        .pid_angle_setting = {},
        .pid_speed_setting = {},
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastConfig(&rudder_l_motor, &rudder_l_cfg));

    // 配置 rudder_r（M3508，motor_id=4 → rx 0x204 / tx 0x200）
    DJIMotorBroadcast_Config_s rudder_r_cfg = {
        .can_e = CAN_1,
        .model = DJI_MODEL_M3508,
        .motor_id = 4,
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.02f,
        .position_offset = 0,
        .torque_constant = 1, // M3508 电流→力矩系数，待标定
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                 // TODO: 后续改为位置/速度环
            .feedback_direction = MOTOR_DIRECTION_NORMAL, // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,    // 输出方向
            .position_mode = MOTOR_POSITION_CONTINUOUS,   // 开环占位，不用位置环
            .angle_limit_max = 0,
            .angle_limit_min = 0,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针1
        },
        .pid_angle_setting = {},
        .pid_speed_setting = {},
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastConfig(&rudder_r_motor, &rudder_r_cfg));

    // 配置 wheel_l（LK MF7015V1-24V-23T，广播模式 motor_id=1 → 槽位 0，回复 ID 0x141）
    // 型号据旧工程 scj(likong) 分支注释确认：Kt = 0.28 Nm/A（直驱无减速比），
    // 详见 drv/drv_motor/drv_lkmotor/lkmotor参数.md。
    // ⚠️ 电机须先用 LK 上位机设为广播模式、ID=1、总线波特率 ≥500Kbps 并保存重启。
    // 编码器实测整圈 raw 0~65535（转一圈恰好回绕一次），驱动固定按 16bit 处理。
    // iq 分辨率 33/4096 A/LSB 也是 MF 全系列固定值，驱动内常量化，不在此配。
    // ⚠️ TODO 标定：Kt 按手册值，建议台架复核。
    LKMotorBroadcast_Config_s wheel_l_cfg = {
        .can_e = CAN_1,
        .model = LK_MODEL_MF,
        .motor_id = 1,
        .torque_constant = 0.28f, // MF7015-24V-23T
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.004f,
        .position_offset = 0,
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                 // TODO: 后续改为速度/位置环
            .feedback_direction = MOTOR_DIRECTION_NORMAL, // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,    // 输出方向
            .position_mode = MOTOR_POSITION_CONTINUOUS,   // 开环占位，不用位置环
            .angle_limit_max = 0,
            .angle_limit_min = 0,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        .pid_angle_setting = {},
        .pid_speed_setting = {},
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(LKMotorBroadcastConfig(&wheel_l_motor, &wheel_l_cfg));

    // 配置 wheel_r（LK MF7015V1-24V-23T，广播模式 motor_id=2 → 槽位 1，回复 ID 0x142）
    LKMotorBroadcast_Config_s wheel_r_cfg = {
        .can_e = CAN_1,
        .model = LK_MODEL_MF,
        .motor_id = 2,
        .torque_constant = 0.28f, // MF7015-24V-23T，同 wheel_l
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.004f,
        .position_offset = 0,
        .controller_setting = {
            .loop_type = MOTOR_LOOP_OPEN,                 // TODO: 后续改为速度/位置环
            .feedback_direction = MOTOR_DIRECTION_NORMAL, // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,    // 输出方向
            .position_mode = MOTOR_POSITION_CONTINUOUS,   // 开环占位，不用位置环
            .angle_limit_max = 0,
            .angle_limit_min = 0,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        .pid_angle_setting = {},
        .pid_speed_setting = {},
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(LKMotorBroadcastConfig(&wheel_r_motor, &wheel_r_cfg));

    MotorEnable(&(rudder_l_motor.base));
    MotorEnable(&(rudder_r_motor.base));
    MotorEnable(&(wheel_l_motor.base));
    MotorEnable(&(wheel_r_motor.base));
}

ITCM_RAM void AppChassisRun(void)
{
    /* 收发骨架：ref 先给 0（开环=0 扭矩，安全）。
     * 必须每周期 SetRef + Send，原因有二：
     *   1) 驱动只在 Send→Calculate→GetData 里解析反馈并写入 data_all.data，
     *      不调用 GetData 则 data_all.data 恒为全 0（调试器看到的“全 0”即此）；
     *   2) C620 / LK 电机都要先收到控制帧才会回传反馈（LK 广播发 0x280 才回状态2）。
     * TODO: 后续接入轮速环 / 舵向位置环，替换下面的 ref 来源。 */
    MotorSetRef(&(rudder_l_motor.base), 0.0f);
    MotorSetRef(&(rudder_r_motor.base), 0.0f);
    MotorSetRef(&(wheel_l_motor.base), 0.0f);
    MotorSetRef(&(wheel_r_motor.base), 0.0f);

    /* 组发送：同一组每周期只需调用一次，否则 0x200/0x280 重复占用总线。
     *   - DJI 舵向组（CAN_1，rx 0x202/0x204）共用 tx 0x200 一帧；
     *   - LK 广播组（CAN_1，槽位 0/1）共用 0x280 一帧。 */
    MotorSend(&(rudder_l_motor.base));
    MotorSend(&(wheel_l_motor.base));
}
