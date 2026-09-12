#include "app_chassis.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h"
//
#include "drv_motor_base.h"
#include "drv_djimotor_broadcast.h"
#include "drv_lkmotor_broadcast.h"
#include "drv_vofa.h"
//
#include "drv_comm.h"
#include "comm_media_can_idseq.h"
#include "comm_proto_custom.h"
//
#include "bsp_gpio.h"
#include "bsp_assert.h"
//
#include <string.h>

// 实例
// rudder_l/r：DJI M3508 (C620) 舵向电机
// wheel_l/r：LK MF7015 直驱轮电机
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);

/*============================ 光电门（M3508 增量编码器零点标定） ============================*/
// M3508 为增量编码器，上电无绝对零点，用两个光电门触发 EXTI 记录机械零点。
// 左舵 PA0 → GPIO_PWM_1，右舵 PA2 → GPIO_PWM_2（DM_MC02 映射）；
// CubeMX 已配为上升沿触发（GPIO_MODE_IT_RISING）并使能 EXTI0/EXTI2 NVIC。
// parent 指向对应电机的 MotorBase_s（基类为首成员，可直接强转）。
GPIO_INSTANCE_DEF(rudder_l_gate_io);
GPIO_INSTANCE_DEF(rudder_r_gate_io);

/**
 * @brief 光电门 EXTI 回调（左/右舵共用，通过 gpio_inst->parent 区分）
 * @note ISR 上下文。触发瞬间把当前反馈位置标定为机械零点：
 *         position = (position_cnt*2π + position_single + position_offset) * 反馈方向
 *       令括号内为 0，则该点即零点（结果与反馈方向无关）。
 * @note TODO 此处读的是任务上次 GetData 的缓存值，存在一个控制周期(2ms)的滞后；
 *       若标定精度不够，改为在任务中收到触发标志后重新 GetData 再标定。
 */
static void RudderPhotogateCallback(GPIOInstance *gpio_inst)
{
    MotorBase_s *motor = (MotorBase_s *)gpio_inst->parent;
    if (motor == NULL)
    {
        return;
    }
    // 1. 另外回调只做标定、不解除武装：光电门每圈都会再次触发并重写 position_offset。如果只想在上电标定一次，需要加一个"已标定"标志位或用 GPIOConfig 把回调置空。
    // 2. 回调里的数据滞后一个控制周期（2ms）：读的是任务上一次 MotorGetData 的缓存值。代码里已用 TODO 标注——上电标定时电机转速低，一般够用；若精度不够，应改成 ISR 只置标志、在 AppChassisRun 里重新MotorGetData 再标定。
    // motor->position_offset = -(float)((double)motor->data_all.position_cnt * M_2PI +
    //                                   (double)motor->data_all.position_single);
}

/*============================ 云台通信 ============================*/
/* 云台通信对话：CAN1 / IDSEQ / PROTO_CUSTOM（帧 = [0xA5][seq][payload N][CRC8][0x5A]）。
 * 底盘侧 tx_id=0x110 段、rx_id=0x100 段（云台侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 分别为 chassis2gimbal_data_t(12B) / gimbal2chassis_data_t(13B)；
 * CAN1 上挂有经典 CAN 的 RS05，故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, CUSTOM, CUSTOM, gimbal2chassis_data_t, 13, chassis2gimbal_data_t, 12, UNPACK_IN_ISR);

static gimbal2chassis_data_t gimbal_rx_data = {0}; // 云台→底盘（on_frame 同步拷贝）
static chassis2gimbal_data_t gimbal_tx_data = {0}; // 底盘→云台（业务填写后 CommSend）

/* 云台接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，须同步拷贝） */
static void GimbalRecvOnFrame(const uint8_t *payload)
{
    memcpy(&gimbal_rx_data, payload, sizeof(gimbal_rx_data));
}

void AppChassisInit(void)
{
    // 注册 CAN 实例（四个电机共用 CAN_2）
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastRegister(&rudder_l_motor));
    BSP_ASSERT_APP_CALL(DJIMotorBroadcastRegister(&rudder_r_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_l_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_r_motor));

    // 配置 rudder_l（M3508，motor_id=2 → rx 0x202 / tx 0x200）
    DJIMotorBroadcast_Config_s rudder_l_cfg = {
        .can_e = CAN_2,
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
        .can_e = CAN_2,
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
        .can_e = CAN_2,
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
        .can_e = CAN_2,
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

    // 注册并配置两个光电门（EXTI 上升沿，回调记录零点）
    rudder_l_gate_io.parent = &(rudder_l_motor.base);
    rudder_r_gate_io.parent = &(rudder_r_motor.base);
    BSP_ASSERT_APP_CALL(GPIORegister(&rudder_l_gate_io));
    BSP_ASSERT_APP_CALL(GPIORegister(&rudder_r_gate_io));

    GPIO_Config_s gate_l_cfg = {
        .gpio_e = GPIO_PWM_1, // PA0
        .callback = RudderPhotogateCallback,
    };
    GPIO_Config_s gate_r_cfg = {
        .gpio_e = GPIO_PWM_2, // PA2
        .callback = RudderPhotogateCallback,
    };
    BSP_ASSERT_APP_CALL(GPIOConfig(&rudder_l_gate_io, &gate_l_cfg));
    BSP_ASSERT_APP_CALL(GPIOConfig(&rudder_r_gate_io, &gate_r_cfg));

    // 云台通信（CAN1 / IDSEQ / CUSTOM）：登记 + 配置介质（ID 段/帧格式）+ 链路看门狗。
    // 一个实例即可双向：底盘在 tx_id 段发、rx_id 段收（idseq 后端已支持收发异段）。
    CommMediaCanIdseqConfig_s gimbal_comm_media_cfg = {
        .can_e = CAN_1,
        .tx_id = 0x110, /* 底盘→云台 ID 段基址 */
        .rx_id = 0x100, /* 云台→底盘 ID 段基址 */
        .frame_type = CAN_STANDARD_DATA_FRAME,
        .mode = CAN_FRAME_FORMAT_CLASSIC, /* CAN1 上有经典 CAN 的 RS05，不可用 FD */
        .timeout_ms = 1,
    };
    CommConfig_s gimbal_comm_cfg = {
        .media_cfg = &gimbal_comm_media_cfg, /* CAN 后端必须有介质配置（ID 段在此写入） */
        .on_frame = GimbalRecvOnFrame,
        .daemon_fault = DAEMON_FAULT_NONE,
        .daemon_reload = 10, /* 对端每 2ms 发一帧，超 10ms 判离线 */
    };
    CommRegister(&gimbal_comm);
    CommConfig(&gimbal_comm, &gimbal_comm_cfg);
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

    // 云台通信：每周期发一帧（接收已由 CAN 中断写入 gimbal_rx_data）
    CommSend(&gimbal_comm, (uint8_t *)&gimbal_tx_data);
}
