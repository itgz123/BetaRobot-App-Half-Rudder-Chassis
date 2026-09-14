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
#include "lib_math.h"
//
#include "bsp_gpio.h"
#include "bsp_assert.h"
//
#include <string.h>

/* 宏 */
#define STEER_GEAR_RATIO 8.0f       // 减速比：电机 8 圈 = 舵轮 1 圈
#define STEER_GEAR_RATIO_INV 0.125f // 减速比：电机 8 圈 = 舵轮 1 圈
// 校准光电门
#define jiaozhun_speed 8.0f

/* 变量 */
// 实例
// rudder_l/r：DJI M3508 (C620) 舵向电机
// wheel_l/r：LK MF7015 直驱轮电机
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);

/*============================ 光电门（M3508 增量编码器零点标定） ============================*/
// M3508 为增量编码器，上电无绝对零点，用两个光电门在转动中找机械零点。
// 左舵 PA0 → GPIO_PWM_1，右舵 PA2 → GPIO_PWM_2（DM_MC02 映射）。
// 遮光片是半遮半透的圆环：电机每转一圈，光电门电平"低→高→低"各变化一次，
// 取其中一个跳变（上升沿）当零点即可——每圈有且仅有一个，且与电机旋向无关
// （左右舵若镜像安装、旋向相反，同一个判据照样成立）。
// CubeMX 里这两个脚仍配着上升沿 EXTI，但本方案改为在控制任务里轮询：
// 回调置 NULL（不占 EXTI 分发），EXTI 触发时 BSP 层判空后直接返回，不影响轮询。
GPIO_INSTANCE_DEF(rudder_l_gate_io);
GPIO_INSTANCE_DEF(rudder_r_gate_io);

/*============================ 云台通信 ============================*/
/* 云台通信对话：CAN1 / IDSEQ / PROTO_CUSTOM（帧 = [0xA5][seq][payload N][CRC8][0x5A]）。
 * 底盘侧 tx_id=0x110 段、rx_id=0x100 段（云台侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 分别为 chassis2gimbal_data_t(12B) / gimbal2chassis_data_t(13B)；
 * CAN1 上挂有经典 CAN 的 RS05，故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, CUSTOM, CUSTOM, gimbal2chassis_data_t, 13, chassis2gimbal_data_t, 12, UNPACK_IN_ISR);

static gimbal2chassis_data_t gimbal2chassis_data = {0}; // 云台→底盘（on_frame 同步拷贝）
static chassis2gimbal_data_t chassis2gimbal_data = {0}; // 底盘→云台（业务填写后 CommSend）

static float rudder_l_motor_setref = 0;
static float rudder_r_motor_setref = 0;
// static float wheel_l_motor_setref = 0;
// static float wheel_r_motor_setref = 0;

static uint8_t rudder_l_state = 0;
static uint8_t rudder_r_state = 0;
static uint8_t rudder_l_inited = 0;
static uint8_t rudder_r_inited = 0;

/* 位置反馈是"电机侧 rad"（舵轮 1 圈 = 电机 STEER_GEAR_RATIO 圈），
 * 而机械偏置按舵轮角度标定，故换算到电机侧要乘减速比。 */
#define STEER_WHEEL_DEG_TO_MOTOR_RAD(deg) ((deg) * (M_PI / 180.0f) * STEER_GEAR_RATIO)

/**
 * @brief 光电门跳变沿处重标定零点：把当前读数改成"正前方 + 机械偏置"对应的位置
 * @param rudder 舵向电机（M3508，增量编码器，上电无绝对零点）
 * @param offset_deg 跳变沿 → 正前方 的舵轮角度偏置（°，见 robot_def.h）
 * @note 位置由驱动算出：position = wrap((多圈累加 + position_offset) * feedback_direction)，
 *       即 position_offset 每加 Δ，读数就沿 feedback_direction 方向变 Δ，
 *       故要让读数等于 target，增量须除以 feedback_direction。
 * @note 跳变沿由控制任务（2ms）轮询得到，采样滞后最多 2ms；标定转速 8rad/s（电机侧）
 *       对应最大零点误差 0.016rad(电机侧) ≈ 0.115°(舵轮)，可忽略，不做补偿。
 */
static void RudderCalibrateOffset(MotorBase_s *rudder, float offset_deg)
{
    float position_now = (float)MotorGetData(rudder).position; // 触发瞬间读数（rad，电机侧）
    float target = STEER_WHEEL_DEG_TO_MOTOR_RAD(offset_deg);   // 正前方应有的读数
    rudder->position_offset += (target - position_now) / (float)rudder->setting.feedback_direction;
}

static void RudderCalibrateOffset_l()
{
    RudderCalibrateOffset(&(rudder_l_motor.base), RUDDER_ZERO_OFFSET_L_DEG);
    rudder_l_inited = 1;
}
static void RudderCalibrateOffset_r()
{
    RudderCalibrateOffset(&(rudder_r_motor.base), RUDDER_ZERO_OFFSET_R_DEG);
    rudder_r_inited = 1;
}
static uint8_t lunxunjioazhun()
{
    if (rudder_l_inited && rudder_r_inited)
    {
        rudder_l_motor.base.setting.loop_type = MOTOR_LOOP_SPEED | MOTOR_LOOP_ANGLE;
        rudder_r_motor.base.setting.loop_type = MOTOR_LOOP_SPEED | MOTOR_LOOP_ANGLE;
        return 1;
    }

    // 控制旋转
    rudder_l_motor.base.setting.loop_type = MOTOR_LOOP_SPEED;
    rudder_r_motor.base.setting.loop_type = MOTOR_LOOP_SPEED;
    MotorEnable(&(rudder_l_motor.base));
    MotorEnable(&(rudder_r_motor.base));
    MotorSetRef(&(rudder_l_motor.base), jiaozhun_speed);
    MotorSetRef(&(rudder_r_motor.base), jiaozhun_speed);

    // 判断是否结束
    if ((rudder_l_state == 1) && (0 == GPIORead(&rudder_l_gate_io)))
    {
        RudderCalibrateOffset_l();
    }
    if ((rudder_r_state == 1) && (0 == GPIORead(&rudder_r_gate_io)))
    {
        RudderCalibrateOffset_r();
    }

    // 记录上次状态
    rudder_l_state = GPIORead(&rudder_l_gate_io);
    rudder_r_state = GPIORead(&rudder_r_gate_io);

    return 0;
}
/* 云台接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，须同步拷贝） */
static void GimbalRecvOnFrame(const uint8_t *payload)
{
    memcpy(&gimbal2chassis_data, payload, sizeof(gimbal2chassis_data));
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
            .loop_type = MOTOR_LOOP_ANGLE | MOTOR_LOOP_SPEED, // TODO: 后续改为位置/速度环
            .feedback_direction = MOTOR_DIRECTION_NORMAL,     // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,        // 输出方向
            .position_mode = MOTOR_POSITION_WRAP,             // 开环占位，不用位置环
            .angle_limit_max = M_PI * STEER_GEAR_RATIO,
            .angle_limit_min = -M_PI * STEER_GEAR_RATIO,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        // 速度环整定依据 ignore/vofa+.csv（20s，0/30/60 rad/s 阶跃，2ms 周期，kp=0.12/ki=0.15 采得）：
        //   实测超调 30~38%，0→30 后还残留约 1rad/s 的慢摆（整定 1.2~4.8s）。
        //   由该数据辨识被控对象：Kt/J≈495 rad/s²/A、B/J≈3 /s、恒值负载≈1.2A、回路延迟≈6ms，
        //   叠加 20ms 速度低通（speed_lpf_rc）；闭环复现超调 33%，与实测一致。
        //   根因是"20ms 低通 + 6ms 延迟"的相位滞后——纯 PI 只能靠降 kp 压超调，代价是变慢
        //   （kp≤0.06 才压到 12%，且负载/模型一漂就回到 25%+）。
        //   故改用微分先行（仅对反馈微分，不受目标阶跃冲击）补阻尼，并小幅提高 ki 加快积分收敛：
        //   kp 0.12→0.10、ki 0.15→0.20、kd 0→0.002、微分滤波 rc=0.004。
        //   辨识模型仿真：平均超调 33.8%→0.8%，平均整定 441ms→178ms；在负载 0.85~1.8A、
        //   Kt/J 400~640、B/J 2~4.5、延迟 4~10ms 的全范围内，最坏超调仍 <4%。
        //   kd 量纲：d_out = kd*(last_measure-measure)/dt，kd=0.002 即"每 2ms 速度变化 1rad/s 给 1A 阻尼"；
        //   实测速度噪声仅 ~0.045rad/s，经 4ms 微分滤波后噪声电流 <0.03A，可忽略。
        //   ⚠️ 积分语义：lib_pid 已修正为 i_out += ki*error*dt，ki 单位是"每秒"，不随调用频率变。
        .pid_speed_setting = {
            .kp = 0.10,                                      // 比例系数（0.12→0.10，让位给微分阻尼）
            .ki = 0.20,                                      // 积分系数 [1/s]（0.15→0.20，加快慢摆收敛）
            .kd = 0.002,                                     // 微分系数（配合微分先行补相位裕度）
            .integral_limit = 2.5,                           // 积分限幅阈值（实测峰值 i_out≈1.47，负载 +50% 仍有余量）
            .coef_a = 20,                                    // 变速积分参数 A (0 = 禁用)
            .coef_b = 20,                                    // 变速积分参数 B
            .d_lpf_rc = 0.004,                               // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,                                 // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,                                   // 死区范围 (0 = 禁用)
            .error_normalize_range = 0,                      // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,                                    // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,                                    // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL |   // 梯形积分
                           PID_ENABLE_INTEGRAL_LIMIT |       // 积分限幅
                           PID_ENABLE_CHANGING_INTEGRATION | // 变速积分
                           PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行（避免目标阶跃的微分冲击）
                           PID_ENABLE_DERIVATIVE_FILTER,     // 微分滤波
        },
        .pid_angle_setting = {
            .kp = 50,                                  // 比例系数（0.12→0.10，让位给微分阻尼）
            .ki = 0,                                   // 积分系数 [1/s]（0.15→0.20，加快慢摆收敛）
            .kd = 0.3,                                 // 微分系数（配合微分先行补相位裕度）
            .integral_limit = 0,                       // 积分限幅阈值（实测峰值 i_out≈1.47，负载 +50% 仍有余量）
            .coef_a = 0,                               // 变速积分参数 A (0 = 禁用)
            .coef_b = 0,                               // 变速积分参数 B
            .d_lpf_rc = 0,                             // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,                           // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,                             // 死区范围 (0 = 禁用)
            .error_normalize_range = 0,                // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,                              // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,                              // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = PID_ENABLE_ERROR_NORMALIZE, // 启用误差归一化

        },
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
            .loop_type = MOTOR_LOOP_ANGLE | MOTOR_LOOP_SPEED, // TODO: 后续改为位置/速度环
            .feedback_direction = MOTOR_DIRECTION_NORMAL,     // 反馈方向
            .motor_direction = MOTOR_DIRECTION_NORMAL,        // 输出方向
            .position_mode = MOTOR_POSITION_WRAP,             // 开环占位，不用位置环
            .angle_limit_max = M_PI * STEER_GEAR_RATIO,
            .angle_limit_min = -M_PI * STEER_GEAR_RATIO,
            .speed_feedforward_src = MOTOR_FEEDFORWARD_DISABLE,    // 速度前馈来源
            .position_feedforward_src = MOTOR_FEEDFORWARD_DISABLE, // 位置前馈来源
            .speed_feedforward_ptr = NULL,                         // 速度前馈指针
            .position_feedforward_ptr = NULL,                      // 位置前馈指针
            .angle_src = MOTOR_FEEDBACK_MOTOR,                     // 角度反馈来源
            .speed_src = MOTOR_FEEDBACK_MOTOR,                     // 速度反馈来源
            .angle_external_ptr = NULL,                            // 外部角度反馈指针
            .speed_external_ptr = NULL,                            // 外部速度反馈指针
        },
        // 速度环整定依据 ignore/vofa+.csv（20s，0/30/60 rad/s 阶跃，2ms 周期，kp=0.12/ki=0.15 采得）：
        //   实测超调 30~38%，0→30 后还残留约 1rad/s 的慢摆（整定 1.2~4.8s）。
        //   由该数据辨识被控对象：Kt/J≈495 rad/s²/A、B/J≈3 /s、恒值负载≈1.2A、回路延迟≈6ms，
        //   叠加 20ms 速度低通（speed_lpf_rc）；闭环复现超调 33%，与实测一致。
        //   根因是"20ms 低通 + 6ms 延迟"的相位滞后——纯 PI 只能靠降 kp 压超调，代价是变慢
        //   （kp≤0.06 才压到 12%，且负载/模型一漂就回到 25%+）。
        //   故改用微分先行（仅对反馈微分，不受目标阶跃冲击）补阻尼，并小幅提高 ki 加快积分收敛：
        //   kp 0.12→0.10、ki 0.15→0.20、kd 0→0.002、微分滤波 rc=0.004。
        //   辨识模型仿真：平均超调 33.8%→0.8%，平均整定 441ms→178ms；在负载 0.85~1.8A、
        //   Kt/J 400~640、B/J 2~4.5、延迟 4~10ms 的全范围内，最坏超调仍 <4%。
        //   kd 量纲：d_out = kd*(last_measure-measure)/dt，kd=0.002 即"每 2ms 速度变化 1rad/s 给 1A 阻尼"；
        //   实测速度噪声仅 ~0.045rad/s，经 4ms 微分滤波后噪声电流 <0.03A，可忽略。
        //   ⚠️ 积分语义：lib_pid 已修正为 i_out += ki*error*dt，ki 单位是"每秒"，不随调用频率变。
        .pid_speed_setting = {
            .kp = 0.10,                                      // 比例系数（0.12→0.10，让位给微分阻尼）
            .ki = 0.20,                                      // 积分系数 [1/s]（0.15→0.20，加快慢摆收敛）
            .kd = 0.002,                                     // 微分系数（配合微分先行补相位裕度）
            .integral_limit = 2.5,                           // 积分限幅阈值（实测峰值 i_out≈1.47，负载 +50% 仍有余量）
            .coef_a = 20,                                    // 变速积分参数 A (0 = 禁用)
            .coef_b = 20,                                    // 变速积分参数 B
            .d_lpf_rc = 0.004,                               // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,                                 // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,                                   // 死区范围 (0 = 禁用)
            .error_normalize_range = 0,                      // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,                                    // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,                                    // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL |   // 梯形积分
                           PID_ENABLE_INTEGRAL_LIMIT |       // 积分限幅
                           PID_ENABLE_CHANGING_INTEGRATION | // 变速积分
                           PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行（避免目标阶跃的微分冲击）
                           PID_ENABLE_DERIVATIVE_FILTER,     // 微分滤波
        },
        .pid_angle_setting = {
            .kp = 50,                                  // 比例系数（0.12→0.10，让位给微分阻尼）
            .ki = 0,                                   // 积分系数 [1/s]（0.15→0.20，加快慢摆收敛）
            .kd = 0.3,                                 // 微分系数（配合微分先行补相位裕度）
            .integral_limit = 0,                       // 积分限幅阈值（实测峰值 i_out≈1.47，负载 +50% 仍有余量）
            .coef_a = 0,                               // 变速积分参数 A (0 = 禁用)
            .coef_b = 0,                               // 变速积分参数 B
            .d_lpf_rc = 0,                             // 微分滤波时间常数 RC (0 = 禁用)
            .out_lpf_rc = 0,                           // 输出滤波时间常数 RC (0 = 禁用)
            .deadband = 0,                             // 死区范围 (0 = 禁用)
            .error_normalize_range = 0,                // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
            .out_max = 0,                              // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .out_min = 0,                              // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)
            .config_mask = PID_ENABLE_ERROR_NORMALIZE, // 启用误差归一化

        },
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

    // 注册并配置两个光电门（EXTI 上升沿，回调触发即写偏置）
    /* ⚠️ 侧别↔引脚：实机曾出现"转右舵只进 RudderPhotogate_l_Callback"，而当时 l 回调
     *    绑的是 PA2(GPIO_PWM_2)，说明右舵门在 PA2、左舵门在 PA0，故按下面填；
     *    改线后把这两行的 gpio_e 对调即可（实例名/回调/被标定电机三者保持同侧）。 */
    rudder_l_gate_io.parent = &(rudder_l_motor.base);
    rudder_r_gate_io.parent = &(rudder_r_motor.base);
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
    BSP_ASSERT_APP_CALL(CommRegister(&gimbal_comm));
    BSP_ASSERT_APP_CALL(CommConfig(&gimbal_comm, &gimbal_comm_cfg));
}

ITCM_RAM void AppChassisRun(void)
{
    // 判断
    if (lunxunjioazhun()) // 轮询校准
    {
        if (gimbal2chassis_data.mode == g2c_stop)
        {
            MotorDisable(&(rudder_l_motor.base));
            MotorDisable(&(rudder_r_motor.base));
            MotorDisable(&(wheel_l_motor.base));
            MotorDisable(&(wheel_r_motor.base));
        }
        else if (gimbal2chassis_data.mode == g2c_normal)
        {
            MotorEnable(&(rudder_l_motor.base));
            MotorEnable(&(rudder_r_motor.base));
            MotorEnable(&(wheel_l_motor.base));
            MotorEnable(&(wheel_r_motor.base));
            rudder_r_motor_setref = 0;
            rudder_l_motor_setref = 0;
        }
        else if (gimbal2chassis_data.mode == g2c_gyro)
        {
            MotorEnable(&(rudder_l_motor.base));
            MotorEnable(&(rudder_r_motor.base));
            MotorEnable(&(wheel_l_motor.base));
            MotorEnable(&(wheel_r_motor.base));
            rudder_r_motor_setref = -6;
            rudder_l_motor_setref = -6;
        }
        else if (gimbal2chassis_data.mode == g2c_hole)
        {
            MotorEnable(&(rudder_l_motor.base));
            MotorEnable(&(rudder_r_motor.base));
            MotorEnable(&(wheel_l_motor.base));
            MotorEnable(&(wheel_r_motor.base));
            rudder_r_motor_setref = 6;
            rudder_l_motor_setref = 6;
        }

        // 设置
        MotorSetRef(&(rudder_l_motor.base), rudder_l_motor_setref);
        MotorSetRef(&(rudder_r_motor.base), rudder_r_motor_setref);
        MotorSetRef(&(wheel_l_motor.base), 0.0f);
        MotorSetRef(&(wheel_r_motor.base), 0.0f);
    }

    // 电机发送
    MotorSend(&(rudder_l_motor.base));
    MotorSend(&(wheel_l_motor.base));

    // 发送
    // rudder_r 速度环观测通道（ch1~8 沿用旧布局，便于复用已存的 VOFA+ 配置）：
    //   1 位置  2 速度  3 力矩  4 积分项 i_out  5 比例项 p_out  6 PID 总输出
    //   7 积分限幅(常量)  8 实际 ref（正常=setref，寻零期间=扫描速度给定）
    // ch9~12 为本次整定补充：整定是否到位可直接看 9/12，若后续加微分再看 10。
    VofaSetChannel(1, rudder_r_motor.base.data_all.data.position);
    VofaSetChannel(2, rudder_r_motor.base.data_all.data.speed);
    VofaSetChannel(3, rudder_r_motor.base.data_all.data.torque);
    VofaSetChannel(4, rudder_r_motor.base.controller.pid_speed.i_out);
    VofaSetChannel(5, rudder_r_motor.base.controller.pid_speed.p_out);
    VofaSetChannel(6, rudder_r_motor.base.controller.pid_speed.output);
    VofaSetChannel(7, rudder_r_motor.base.controller.pid_speed.integral_limit);
    VofaSetChannel(8, rudder_r_motor.base.controller.ref);                // 驱动实际用的 ref（寻零时是扫描给定）
    VofaSetChannel(9, rudder_r_motor.base.controller.pid_speed.error);    // 误差 ref-measure
    VofaSetChannel(10, rudder_r_motor.base.controller.pid_speed.d_out);   // 微分项（kd=0 时应恒 0）
    VofaSetChannel(11, rudder_r_motor.base.controller.pid_speed.measure); // PID 实际用的反馈速度
    VofaSetChannel(12, rudder_r_motor.base.controller.output);            // 最终电流/力矩命令（已含方向与量纲换算）
    VofaSetChannel(13, rudder_r_motor.base.controller.pid_angle.error);
    VofaSetChannel(14, rudder_r_motor.base.controller.pid_angle.output);
    VofaSend();

    CommSend(&gimbal_comm, (uint8_t *)&chassis2gimbal_data); // 云台通信：每周期发一帧（接收已由 CAN 中断写入 gimbal_rx_data）
}
