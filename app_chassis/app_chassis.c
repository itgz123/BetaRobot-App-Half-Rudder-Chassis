#include "app_chassis.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h"
#include "app_chassis_def.h" // 本 app 的可调参数（传动比/校准/限幅/各环 PID）
//
#include "drv_motor_base.h"
#include "drv_lkmotor_broadcast.h"
#include "drv_vofa.h"
//
#include "drvs_djimotor_broadcast.h" // DJI 广播纯协议层（舵向 2 个 M3508）
#include "drvlib_motor.h"            // 三级级联 PID 控制核（舵向位置/速度环）
#include "drvlib_bmi088_kalman.h"    // 底盘 IMU：通信 + 标定修正 + 线性卡尔曼（纯观测）
//
#include "drv_comm.h"
#include "comm_media_can_idseq.h"
#include "comm_proto_ext.h"
//
#include "lib_math.h"
#include "lib_chassis_half_rudder.h"
#include "lib_fsm_table.h" // 表驱动状态机（底盘顶层状态机）
//
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "bsp_sys_status.h"
//
#include <string.h>

/* 日志实例（校准/故障上报） */
LOG_INSTANCE_DEF(g_chassis_log, "chassis", 20);

/* 本文件的可调参数（传动比 / 校准 / 限幅 / 各环 PID）都在 app_chassis_def.h；
 * 依赖这些参数的单位换算宏就近放在用它的地方（STEER_WHEEL_DEG_TO_MOTOR_RAD、
 * MOTOR_RADPS_TO_WHEEL_RADPS 等）——它们是代码不是参数，故不搬。 */

/* 变量 */
// 实例
// rudder_l/r：DJI M3508 (C620) 舵向电机 —— drvs 纯协议（收发）+ drvlib 三级 PID（闭环）
// wheel_l/r：LK MF7015 直驱轮电机 —— 仍走旧 drv_motor 广播栈
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_l_motor);
DRVS_DJIMOTOR_BROADCAST_INSTANCE_DEF(rudder_r_motor);
DRVS_DJIMOTOR_BROADCAST_GROUP_DEF(rudder_motor_group); // 两舵向共一组（CAN_2，id2/id4 → 0x201~0x204 同帧下发）
DRVLIB_MOTOR_INSTANCE_DEF(rudder_l_ctrl);
DRVLIB_MOTOR_INSTANCE_DEF(rudder_r_ctrl);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_l_motor);
LKMOTOR_BROADCAST_INSTANCE_DEF(wheel_r_motor);

/*============================ 姿态传感器（BMI088） ============================*/
/* 底盘 IMU：和云台同一套栈（drv_bmi088 只做通信 + drvlib_bmi088_kalman 做标定修正与姿态），
 * 但**是另一颗器件、装在底盘车架上**，标定值不能沿用云台那份（见下面 s_bmi088_calib）。
 *
 * 用途有两个：
 *   ① 观测：ch28/29 记世界系偏航角速度与航向角、ch30/31 记指令 w 与正解 w（见
 *      AppChassisRun 末尾的通道说明）。
 *   ② **偏航轴闭环**（2026-10-17 加入）：用原始陀螺在"世界向上"的投影 dot(gyro,u)
 *      做 yaw rate 阻尼 + 航向保持，修正量在 CHASSIS_WIRE_TWIST_SIGN 之前注入 w
 *      （见 ChassisYawLoopStep 与 app_chassis_def.h 的偏航闭环一节）。
 *
 * ⚠ 看门狗仍不挂离线动作（DAEMON_FAULT_NONE）：偏航闭环对 IMU 的依赖是**可旁路**的
 *   —— 连续 CHASSIS_YAW_IMU_TIMEOUT_MS 没有新样本、或姿态没 seed 成功时，闭环自动
 *   退化成原来的开环，底盘照跑。所以 IMU 掉线不应把车拦停。
 * ⚠ 装法对 yaw 无影响、对倾角有：yaw_rate 是航向角的导数，IMU 绕 z 装偏多少都只差一个
 *   常量偏置；但绕 x/y 的安装倾角会进 ψ̇ 的运动学换算（除以 cosθ），故须像云台那样
 *   尽量装平（KF 会用加速度计自己认出静态倾角，不需要手填）。 */
BMI088_KALMAN_INSTANCE_DEF(bmi088);

/* ─────────── PC 端标定结果：写死进固件 ───────────
 * ⚠ 这是**底盘车架上那颗** BMI088 的标定，与 app_gimbal.c 里那份（云台底座那颗）不是
 *   同一颗器件，数值不可互抄。拟合流程与取值口径见 app_gimbal.c 同名结构体的注释：
 *   上电静置录一段 → drvlib/drvlib_bmi088_calib/bmi088_calib.py（残差法：旧值 + 录到的残差）
 *   → 填进 .gyro.bias → 重新编译烧录。
 * 未标定（全 0）时的代价：yaw_rate 带一个零偏（量级 ~0.1°/s，把静止段的读数取均值即得），
 * 它积分出的航向角会缓慢漂。本阶段只用它判"转没转、转多快"，够用；将来若要把 euler.yaw
 * 当绝对航向用（例如喂给底盘航向环），必须先重标。 */
static const BMI088_Calib_s s_bmi088_calib = {
    .gyro =
        {
            /* 2026-10-17 静止段中位（`w 给0还是漂移.csv` 的 [12] 段）：gx 恰好 1 LSB，
             * gy/gz 小于 1LSB(0.061°/s)。航向是纯积分，填上它可把零偏漂移压到 <0.06°/s。 */
            .bias = {-0.001065f, 0.0f, 0.0f}, // ← 标定后填这里；其余项缺省 0 = 不修正
        },
};

/*============================ 偏航轴闭环 ============================*/
/* 反馈/控制律/参数见 app_chassis_def.h 的"偏航轴闭环"一节。这里只放状态机：
 *   - Init 在进入 RUN 时调用（清积分项、重新播种角速度低通）；
 *   - Step 每拍在 HalfRudderSolve 之前调用，返回**修正后的 wire w**；
 *   - IMU 无数据/未 seed/超时 ⇒ 自动旁路，返回原指令（开环）。
 * 符号：w_cmd 与 yaw_rate 都是物理/线协议约定（w>0 = 俯视逆时针），
 *       所以"实测正转 → 减一点 w"就是负反馈；修正量在 CHASSIS_WIRE_TWIST_SIGN
 *       之前注入，与云台指令同一条链路。 */
typedef struct
{
    float rate_lpf;   // 低通后的物理 yaw rate (rad/s)
    float integ;      // 速率误差积分 [rad/s]，等价于"航向误差"折算成的 w 修正
    float err;        // 本拍速率误差 w_cmd − yaw_rate (rad/s)，供 VOFA
    float corr;       // 本拍施加的 w 修正 (rad/s，已过变化率限制)，供 VOFA
    float corr_last;  // 上一拍的修正量，用于变化率限制
    uint64_t last_ts; // 上次看到的陀螺时间戳 (us)，用于判新鲜度
    uint32_t stale_ms; // 连续没有新样本的时长 (ms)
    uint8_t seeded;    // 0 = 角速度低通/积分器还没播种
} ChassisYawLoop_s;

static ChassisYawLoop_s s_yaw_loop;

static void ChassisYawLoopInit(void)
{
    s_yaw_loop.rate_lpf = 0.0f;
    s_yaw_loop.integ = 0.0f;
    s_yaw_loop.err = 0.0f;
    s_yaw_loop.corr = 0.0f;
    s_yaw_loop.corr_last = 0.0f;
    s_yaw_loop.last_ts = 0;
    s_yaw_loop.stale_ms = 0;
    s_yaw_loop.seeded = 0;
}

/**
 * @brief 偏航闭环一拍：返回修正后的 wire w（物理约定，w>0 = 俯视逆时针）
 * @param w_cmd   云台下发的 w（物理约定）
 * @param imu     本拍 IMU 输出（用 gyro 原始量，不用 KF 的 yaw_rate）
 * @param dt_ms   控制周期（ms）
 * @param steer_l 左舵实测轮角 [rad]（舵轮侧，不是电机侧）
 * @param steer_r 右舵实测轮角 [rad]
 * @note IMU 不可用时原样返回 w_cmd（开环），不会因为观测件掉线把车拦停。
 * @note 权限门控：两轮 |cos(舵角)| 都要 > CHASSIS_YAW_MIN_AUTHORITY 才闭环。
 *       半舵底盘 `v_i = v·g ∓ a·w·cosδ`，cosδ≈0（横移）时差模**没有**偏航权限，
 *       此时硬修只会把轮向从平移方向拽开（首份日志的 ±2.5rad/s 极限环就是这么来的）。
 *       门控关断时返回原 w_cmd、冻结积分（不重置，切回有权限时接着用）。
 */
static float ChassisYawLoopStep(float w_cmd, const BMI088Kalman_Data_t *imu, float dt_ms,
                                float steer_l, float steer_r)
{
    const float dt_s = dt_ms * 0.001f;

    s_yaw_loop.err = 0.0f;
    s_yaw_loop.corr = 0.0f;

    if ((imu == 0) || (imu->time_stamp_g == 0) || (imu->valid == 0))
    {
        ChassisYawLoopInit();
        return w_cmd;
    }

    // 新鲜度：每秒 1000 个样本 ⇒ 500Hz 任务每拍都该见到新时间戳
    if (imu->time_stamp_g != s_yaw_loop.last_ts)
    {
        s_yaw_loop.last_ts = imu->time_stamp_g;
        s_yaw_loop.stale_ms = 0;
    }
    else
    {
        s_yaw_loop.stale_ms += (uint32_t)dt_ms;
        if (s_yaw_loop.stale_ms > CHASSIS_YAW_IMU_TIMEOUT_MS)
        {
            ChassisYawLoopInit();
            return w_cmd;
        }
    }

    // 物理 yaw rate = 机体系陀螺在"世界向上"方向的投影（安装常量，见 app_chassis_def.h）
    float rate = imu->gyro[0] * CHASSIS_IMU_UP_X +
                 imu->gyro[1] * CHASSIS_IMU_UP_Y +
                 imu->gyro[2] * CHASSIS_IMU_UP_Z;

    if (!s_yaw_loop.seeded)
    {
        // 首拍直接吃进当前角速度，避免低通从 0 爬升时产生一段假阻尼
        s_yaw_loop.rate_lpf = rate;
        s_yaw_loop.integ = 0.0f;
        s_yaw_loop.seeded = 1;
    }

    if (CHASSIS_YAW_RATE_LPF_RC > 0.0f)
    {
        const float alpha = dt_s / (CHASSIS_YAW_RATE_LPF_RC + dt_s);
        s_yaw_loop.rate_lpf += alpha * (rate - s_yaw_loop.rate_lpf);
        rate = s_yaw_loop.rate_lpf;
    }

    // 偏航权限：两轮都要 |cos(舵角)| 足够大，差模才推得动车体。
    // 横移（舵角≈±90°）时门控关断：只观测、不修正，避免把轮向从平移方向拽开。
    const float authority_l = Lib_Math_Cos(steer_l);
    const float authority_r = Lib_Math_Cos(steer_r);
    if ((Lib_Math_Fabs(authority_l) < CHASSIS_YAW_MIN_AUTHORITY) ||
        (Lib_Math_Fabs(authority_r) < CHASSIS_YAW_MIN_AUTHORITY))
    {
        s_yaw_loop.err = w_cmd - rate;
        s_yaw_loop.corr = 0.0f;
        s_yaw_loop.corr_last = 0.0f; // 重新开环时从 0 平滑起修，别攒着猛甩
        return w_cmd;
    }

    // 速率误差 PI：P 阻尼、I 消稳态漂移（I 本身就是 ∫(w_cmd−yaw_rate)dt，
    // 即"指令航向 − 实测航向"，只是不显式积分两个角、避免 wrap 的边界问题）。
    const float err = w_cmd - rate;
    const float corr_raw = CHASSIS_YAW_RATE_KP * err + s_yaw_loop.integ;

    // 条件积分抗饱和：输出已顶到限幅、且误差还在往同方向推时，冻结积分。
    if (!(((corr_raw > CHASSIS_YAW_CORR_MAX) && (err > 0.0f)) ||
          ((corr_raw < -CHASSIS_YAW_CORR_MAX) && (err < 0.0f))))
    {
        s_yaw_loop.integ += CHASSIS_YAW_RATE_KI * err * dt_s;
        if (s_yaw_loop.integ > CHASSIS_YAW_INTEGRAL_MAX)
        {
            s_yaw_loop.integ = CHASSIS_YAW_INTEGRAL_MAX;
        }
        else if (s_yaw_loop.integ < -CHASSIS_YAW_INTEGRAL_MAX)
        {
            s_yaw_loop.integ = -CHASSIS_YAW_INTEGRAL_MAX;
        }
    }

    float corr = CHASSIS_YAW_RATE_KP * err + s_yaw_loop.integ;
    if (corr > CHASSIS_YAW_CORR_MAX)
    {
        corr = CHASSIS_YAW_CORR_MAX;
    }
    else if (corr < -CHASSIS_YAW_CORR_MAX)
    {
        corr = -CHASSIS_YAW_CORR_MAX;
    }

    /* 变化率限制：陀螺噪声/换向瞬间 err 会跳，P 项一拍就能把轮向打翻。
     * 首份日志里修正量在 ±1.0 之间 bang-bang、舵角跟着 0/π↔±90° 来回扫，
     * 就是没有这层限制。限幅后修正量按 CHASSIS_YAW_CORR_SLEW 线性逼近目标。 */
    const float max_step = CHASSIS_YAW_CORR_SLEW * dt_s;
    if (corr > s_yaw_loop.corr_last + max_step)
    {
        corr = s_yaw_loop.corr_last + max_step;
    }
    else if (corr < s_yaw_loop.corr_last - max_step)
    {
        corr = s_yaw_loop.corr_last - max_step;
    }

    s_yaw_loop.err = err;
    s_yaw_loop.corr = corr;
    s_yaw_loop.corr_last = corr;
    return w_cmd + corr;
}

/*============================ 光电门（M3508 增量编码器零点标定） ============================*/
// M3508 为增量编码器，上电无绝对零点，用两个光电门在转动中找机械零点。
// 左舵 PA0 → GPIO_PWM_1，右舵 PA2 → GPIO_PWM_2（DM_MC02 映射）。
// 遮光片是半遮半透的圆环：电机每转一圈，光电门电平"低→高→低"各变化一次，
// 取其中一个跳变（本代码用下降沿 1→0）当零点即可——每圈有且仅有一个，且与电机旋向无关
// （左右舵若镜像安装、旋向相反，同一个判据照样成立）。
// CubeMX 里这两个脚仍配着上升沿 EXTI，但本方案改为在控制任务里轮询：
// 回调置 NULL（不占 EXTI 分发），EXTI 触发时 BSP 层判空后直接返回，不影响轮询。
GPIO_INSTANCE_DEF(rudder_l_gate_io);
GPIO_INSTANCE_DEF(rudder_r_gate_io);

/*============================ 云台通信 ============================*/
/* 云台通信对话：CAN1 / IDSEQ / PROTO_EXT（帧 = [0xA5][seq][汉明编码区 E][CRC8][0x5A]）。
 * 在 CUSTOM 的定界 + CRC 之上先过一级扩展缩短汉明码（SECDED，m=7）：每块纠 1 位、检 2 位，
 * CAN 差分对上的偶发位翻转不再直接丢掉这一拍控制帧；CRC 仍保留，用来复核汉明的纠错结果
 * （见 comm_proto_ext.h）。分块与帧长全在编译期定死，运行期只按已算好的 cfg 逐块编解码。
 * 云台侧须同用 PROTO_EXT（PROTO_EXT 的帧格式与 PROTO_CUSTOM 不同，两端协议名必须一致）。
 * 底盘侧 tx_id=0x110 段、rx_id=0x100 段（云台侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 分别为 gimbal2chassis_data_t(13B) / chassis2gimbal_data_t(12B)：
 *   13B → 1 块 × 14B 编码区，整帧 18B（3 个 CAN 包）
 *   12B → 1 块 × 13B 编码区，整帧 17B（3 个 CAN 包）
 * CAN1 上挂有经典 CAN 的 RS05，故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, EXT, EXT, gimbal2chassis_data_t, 13, chassis2gimbal_data_t, 12, UNPACK_IN_ISR);

static gimbal2chassis_data_t gimbal2chassis_data = {0}; // 云台→底盘（on_frame 同步拷贝，SAFE 态会清零=作废）
/* 云台档位的独立副本，**只由接收回调写**（app 侧只读）：状态机的 stopped 判据取自它，
 * 不能取 gimbal2chassis_data.enabled —— SAFE 态的动作会把那份指令清零作废，而采样在
 * ChassisFsmUpdate 里发生在动作之后（同一拍先写后读）⇒ 用同一份数据会让 SAFE 自己
 * 按死自己的退出条件（stopped 恒 1），卡在 SAFE 出不来。详见 ChassisFsmUpdate 注释。 */
static volatile robot_mode gimbal_mode_rx = robot_mode_stop;
static chassis2gimbal_data_t chassis2gimbal_data = {0}; // 底盘→云台（业务填写后 CommSend）

static float rudder_l_motor_setref = 0;
static float rudder_r_motor_setref = 0;
static float wheel_l_motor_setref = 0;
static float wheel_r_motor_setref = 0;

/* VOFA 发帧分频计数（控制 500Hz → 遥测 250Hz，见 app_chassis_def.h 的 CHASSIS_VOFA_SEND_DIV） */
static uint8_t s_vofa_send_div = 0;

static uint8_t rudder_l_state = 0;
static uint8_t rudder_r_state = 0;
static uint8_t rudder_l_inited = 0;
static uint8_t rudder_r_inited = 0;

/* 位置反馈是"电机侧 rad"（舵轮 1 圈 = 电机 STEER_GEAR_RATIO 圈），
 * 而机械偏置按舵轮角度标定，故换算到电机侧要乘减速比。 */
#define STEER_WHEEL_DEG_TO_MOTOR_RAD(deg) ((deg) * (M_PI / 180.0f) * STEER_GEAR_RATIO)

/**
 * @brief 光电门跳变沿处重标定零点：把当前读数改成"正前方 + 机械偏置"对应的位置
 * @param rudder 舵向控制核（drvlib_motor；位置后处理在它内部，M3508 增量编码器上电无绝对零点）
 * @param offset_deg 跳变沿 → 正前方 的舵轮角度偏置（°，见 robot_def.h）
 * @note 位置由控制核算出：position = wrap((多圈累加 + position_offset) * feedback_direction)，
 *       即 position_offset 每加 Δ，读数就沿 feedback_direction 方向变 Δ，
 *       故要让读数等于 target，增量须除以 feedback_direction。
 * @note 控制核把处理后的位置**缓存**在 data 里、只在 SetRef 时刷新（旧 drv_motor 是每次
 *       读反馈都重算）。标定发生在 SetRef 之后，若不顺手把本拍读数改成新偏置下的值，
 *       下一拍（已切 RUN）HalfRudderSolve 会拿旧偏置的位置去解算舵角。故这里同时改
 *       data.position —— 偏置加 delta 使读数沿 feedback_direction 变 (target-position_now)，
 *       新读数即 target（与下一拍重算结果同余，差整数个跨也不影响：舵角按 mod 2π 用）。
 * @note 跳变沿由控制任务（2ms）轮询得到，采样滞后最多 2ms；标定转速 8rad/s（电机侧）
 *       对应最大零点误差 0.016rad(电机侧) ≈ 0.115°(舵轮)，可忽略，不做补偿。
 */
static void RudderCalibrateOffset(DrvlibMotorInstance *rudder, float offset_deg)
{
    float position_now = (float)rudder->data.position;       // 触发瞬间读数（rad，电机侧，已含方向/偏置/归一化）
    float target = STEER_WHEEL_DEG_TO_MOTOR_RAD(offset_deg); // 正前方应有的读数
    rudder->cfg.position_offset += (target - position_now) / (float)rudder->cfg.feedback_direction;
    rudder->data.position += (double)(target - position_now); // 同步本拍缓存读数，见上（mod 2π 等价）
}

static void RudderCalibrateOffset_l(void)
{
    RudderCalibrateOffset(&rudder_l_ctrl, RUDDER_ZERO_OFFSET_L_DEG);
    rudder_l_inited = 1;
}
static void RudderCalibrateOffset_r(void)
{
    RudderCalibrateOffset(&rudder_r_ctrl, RUDDER_ZERO_OFFSET_R_DEG);
    rudder_r_inited = 1;
}
/*============================ 半舵运动学（lib_chassis 内核） ============================*/
/* 几何与解算全部交给 lib/lib_chassis 的 chassis_half_rudder 封装，本文件只做
 * "电机侧 ↔ 轮侧"的单位换算与槽位映射，不再自带任何闭式解。
 *   逆解：底盘速度 (vx,vy,w) → 2 组 {目标舵角, 驱动速度}
 *   正解：2 组 {驱动速度, 当前舵角} → 底盘速度（里程计：2 组舵轮给 4 个方程解 3 个未知量）
 * 内核输出的舵轮量是"期望轮向 + 轮速大小"（纯几何、无跨帧状态）：在 φ 与 φ+180° 两个
 * 等价解里挑哪个、驱动要不要反转，是**离散决策**必须看整车（两轮同拍翻）才不抖，
 * 故落在本层（见 ChassisRudderDecide）；内核那边不做选择，也就不需要记任何东西。
 *
 * 约定（与内核一致）：舵角用"舵轮侧 rad"（0 = 正前方，即光电门标定后的零点），
 *   车体系 x+ 前、y+ 左、w+ 逆时针；驱动速度在文件里进出都是"电机侧 rad/s"
 *   （可直接下发/读反馈），只在跨内核接口时换算一次（见下面两个宏）。
 * 本层只做运动学与换向决策：舵向位置环（含积分）与换向的执行都在电机驱动里。
 *
 * ⚠️ 符号约定（首次上车务必台架核对；错了就改对应电机的 motor_direction/feedback_direction）：
 *   ① 舵向：电机位置读数增大 == 舵轮俯视逆时针（与车体系 y+ 同向）时，舵角无需取反；
 *   ② 轮向：驱动速度为正 == 沿当前舵角方向推动车体（不是"俯视逆时针"），
 *      故左右轮安装镜像时，两者所需的 motor_direction 往往相反。
 */
static LibChassisInstance_s s_chassis; // 内核实例：几何在 AppChassisInit 里一次性配好

/* 单位换算：内核的 wheel_speed 是"轮缘角速度 [rad/s]"，与电机侧差一个减速比。
 * MF7015 直驱（减速比 1）时两式都是恒等，但保留成宏——换带减速箱的轮子时语义不用改。 */
#define MOTOR_RADPS_TO_WHEEL_RADPS(v) ((v) / CHASSIS_DRIVE_REDUCTION)
#define WHEEL_RADPS_TO_MOTOR_RADPS(v) ((v) * CHASSIS_DRIVE_REDUCTION)

/**
 * @brief 采集两组舵轮的当前反馈，填成内核状态
 * @param st 输出：内核状态（舵角 + 轮缘角速度）
 * @note 舵角：电机侧 rad → 舵轮侧 rad。舵向控制核是 DRVLIB_MOTOR_POS_WRAP，读数被归一化在 ±π，
 *       不是严格连续值——本层只用它做"±180° 取最近解"的选择，对 2π 取模等价；
 *       解出的目标是"当前角 + 最近角差"，可以超出 ±π，驱动侧 setpoint 与 PID 误差都会做归一化，
 *       实际误差仍是最短角差，故这里直接当连续角用没有问题。
 * @note 轮速：电机侧 rad/s → 轮缘角速度 rad/s。只有正解用它做里程计，逆解不看这个量。
 * @note 舵角取控制核处理后的多圈位置（rudder_*_ctrl.data.position）——由 AppChassisRun 里
 *       本拍/上一拍的 DrvlibMotorSetRef 刷新（见那里的调用顺序说明）。
 */
static void ChassisReadState(LibChassisState_s *st)
{
    st->steer_angle[LIB_CHASSIS_HALF_RUDDER_L] = (float)rudder_l_ctrl.data.position * STEER_GEAR_RATIO_INV;
    st->steer_angle[LIB_CHASSIS_HALF_RUDDER_R] = (float)rudder_r_ctrl.data.position * STEER_GEAR_RATIO_INV;
    st->wheel_speed[LIB_CHASSIS_HALF_RUDDER_L] = MOTOR_RADPS_TO_WHEEL_RADPS((float)MotorGetData(&(wheel_l_motor.base)).speed);
    st->wheel_speed[LIB_CHASSIS_HALF_RUDDER_R] = MOTOR_RADPS_TO_WHEEL_RADPS((float)MotorGetData(&(wheel_r_motor.base)).speed);
}

/*============================ 舵轮换向决策（档2：短时域代价预测） ============================*/
/* 内核只给"期望轮向 + 轮速大小"（纯几何、无状态）；在 φ_des 与 φ_des+180° 两个等价解里
 * 挑哪个、驱动轮要不要反转、静止时保持哪个朝向，都是**离散决策**，放本层。
 *
 * 判据 = 短时域预测的代价（FCS-MPC 的退化形态，见 app_chassis.md §2.2）：把 4 个候选
 * （两轮各取 0/1）各自演一遍过渡过程，取代价最小者——
 *   舵角：从实测角以 ω_max 走最短弧到候选目标，到位后保持（时间最优/砰-砰）；
 *   轮速：按余弦律 s_i = V_i·cos(d_i − φ_i(t)) 跟着舵角走（§2.1：这个式子与分支恒等，
 *         两个候选给的轮速一模一样，差别**只在 φ(t) 走哪条弧**，故不钳负）；
 *   整车：每步喂 LibChassisForward 得 ξ̂ —— 正解里含"轮面不侧滑"约束，过渡期舵轮斜着推
 *         时的推力损失、以及两轮不对称产生的偏航，都被自动算进去，不用给工况分类；
 *   代价：J(c) = ∫₀ᵀ ‖ξ̂(t;c) − ξ*‖²_W dt，T = 最慢一轮的到位时间（makespan）。
 * ⚠ **不能只比"谁转到位更快"**：两轮对称起步的纯平移，四个候选的 T 完全相同（都转 180°−|e|，
 *   只是反向）——时间是零信息量，能分开好坏的只有过渡期的偏航（§2.2 表1/表2）。这里的
 *   "时间"没有丢：T 取候选**自己的** makespan，演得慢的那个自然积得多。
 * ⚠ 决策仍是**两轮一起做**，但不再靠"平移/自旋"分类：比的是"两轮合成 twist vs 指令 twist"，
 *   一个翻一个不翻会立刻以偏航的形式反映到 J 里（§2.2）。档1 的 SYNC 判据随之删除。
 * ⚠ 决策必须两轮同拍落地：先算完 4 个候选再一起 ChassisRudderApply。
 * ⚠ 状态同样由控制任务独占（无锁），不得在别的上下文读改写。 */
static float s_steer_target_last[LIB_CHASSIS_HALF_RUDDER_NUM]; // 本拍下发的目标舵角（舵轮侧连续角）
static int8_t s_steer_reverse[LIB_CHASSIS_HALF_RUDDER_NUM];    // 本拍是否采用反向解（每拍重算，不是记忆）
static uint8_t s_steer_seeded;                                 // bit i = 轮 i 的首拍锚定已完成
/* 本拍实际乘进轮速的余弦系数（不是控制状态，只是留一份给 VOFA 观测：
 * 上面存的是"想要转到哪"，它才是"这拍真打了多少折扣"）。 */
static float s_steer_align[LIB_CHASSIS_HALF_RUDDER_NUM];

/**
 * @brief 写下一轮的目标舵角与分支：把 |角差| 按选择结果折算到 φ_des 或 φ_des±180°
 * @param idx 轮序号
 * @param e   不翻转时的角差 wrap(phi_des − phi_cur) [rad]
 * @param rev 本拍分支：1 = 取反向解（目标舵角再偏 ±180°、驱动反转）
 * @param phi_cur 当前舵角 [rad]（舵轮侧实测，按连续角用）
 */
static void ChassisRudderApply(uint8_t idx, float e, int8_t rev, float phi_cur)
{
    if (rev != 0)
        e -= (e > 0.0f) ? (float)M_PI : -(float)M_PI;

    s_steer_reverse[idx] = rev;
    s_steer_target_last[idx] = phi_cur + e;
}

/* 预测一个候选所需的本拍量：每轮的期望轮向 d_i、轮速大小 V_i、实测舵角 φ_i；
 * moving 是本拍在转的轮（内核给 wheel_speed = 0 的轮不转也不出力）。 */
typedef struct
{
    float d[LIB_CHASSIS_HALF_RUDDER_NUM];
    float v[LIB_CHASSIS_HALF_RUDDER_NUM];
    float phi[LIB_CHASSIS_HALF_RUDDER_NUM];
    uint8_t moving;
} ChassisRudderPred_s;

/**
 * @brief 预测候选分支 (rev_l, rev_r) 的代价 J = ∫₀ᵀ ‖ξ̂(t) − ξ*‖²_W dt
 * @param inst   内核实例（正解要用它，会被重算雅可比——本函数在 HalfRudderSolve 的那次正解之前跑，
 *               故留下的中间状态随后即被覆盖，不影响反馈回填）
 * @param twist  指令 twist（车体系，**与内核同一个坐标系**：调用方已经取过负号）
 * @param p      本拍预测输入（期望轮向/轮速大小/实测舵角/哪几个轮在动）
 * @param rev_l  左轮候选分支（0 = 正解，1 = 反解）
 * @param rev_r  右轮候选分支
 * @param vscale 线速度误差的归一键（|v*|，见 ChassisRudderDecide）
 * @param wscale 偏航误差的归一键（|w*|，见 ChassisRudderDecide）
 * @return 代价（越小越好）；0 = 两轮本拍都已在候选目标角 1° 内（J 非负 ⇒ 必为全局最优）；
 *         CHASSIS_STEER_COST_INFEASIBLE = 该候选下正解几何退化，不可比
 */
static float ChassisRudderCost(LibChassisInstance_s *inst, const vector3_t *twist,
                               const ChassisRudderPred_s *p, uint8_t rev_l, uint8_t rev_r,
                               float vscale, float wscale)
{
    const uint8_t rev[LIB_CHASSIS_HALF_RUDDER_NUM] = {rev_l, rev_r};
    float delta[LIB_CHASSIS_HALF_RUDDER_NUM];
    float travel = 0.0f; // 两轮的共同到位时间由最慢的一轮决定（makespan）
    float t_end, dt, j = 0.0f;
    uint8_t i, k;

    for (i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        /* 候选目标 = 期望轮向，反向解再偏 ±180°（两解等价，只是让舵轮走另一条弧） */
        const float target = p->d[i] - ((rev[i] != 0) ? (float)M_PI : 0.0f);

        // 本拍不转的轮：行程为 0（目标角由 Apply 保持，不参与预测）
        delta[i] = ((p->moving & (uint8_t)(1u << i)) != 0u)
                       ? Lib_Math_WrapAngleNegPIToPI(target - p->phi[i])
                       : 0.0f;
        if (FABS(delta[i]) > travel)
            travel = FABS(delta[i]);
    }

    /* 两轮都已在目标角附近：演出来也只有这点残差，且 J 非负 ⇒ 0 是最小值，直接返回。
     * 稳态跟踪走这条快路径（实测角度环误差 rms 0.05°），不跑下面的循环。 */
    if (travel < CHASSIS_STEER_COST_ZERO_RAD)
        return 0.0f;

    t_end = travel / CHASSIS_STEER_OMEGA_MAX_RAD; // 候选自己的 makespan：转得慢的积得多
    dt = t_end / (float)CHASSIS_STEER_COST_STEPS;

    for (k = 0; k < CHASSIS_STEER_COST_STEPS; k++)
    {
        const float t = ((float)k + 0.5f) * dt; // 中点法：采样点全落在过渡期内（含收尾）
        LibChassisState_s st = {0};
        vector3_t xi;
        float dvx, dvy, dw;

        for (i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            float phi = p->phi[i];

            if (FABS(delta[i]) > 1e-6f)
            {
                float frac = t * CHASSIS_STEER_OMEGA_MAX_RAD / FABS(delta[i]);
                if (frac > 1.0f)
                    frac = 1.0f; // 到位后保持（砰-砰：全程 ω_max，不做减速收尾）
                phi += delta[i] * frac;
            }

            st.steer_angle[i] = phi;
            /* 余弦律：轮速随舵角连续缩放，过 90° 后自然翻负（轮子反向出力）——不钳负，
             * 钳了就与"两个解恒等"矛盾，翻转会被切成两段（§2.1）。 */
            st.wheel_speed[i] = p->v[i] * Lib_Math_Cos(p->d[i] - phi);
        }

        if (LibChassisForward(inst, &st, &xi) != 0)
            return CHASSIS_STEER_COST_INFEASIBLE; // 几何退化（JᵀJ 奇异）：该候选没得比

        dvx = (xi.x - twist->x) / vscale;
        dvy = (xi.y - twist->y) / vscale;
        dw = (xi.z - twist->z) / wscale;
        j += CHASSIS_STEER_COST_W_V * (dvx * dvx + dvy * dvy) + CHASSIS_STEER_COST_W_W * (dw * dw);
    }

    return j * dt;
}

/**
 * @brief 换向决策（整套）：内核输出 → 两轮各自的目标舵角 + 分支
 * @param inst  内核实例（预测要跑正解）
 * @param twist 本拍指令 twist（车体系）
 * @param ref   内核逆解输出（steer_target = 期望轮向，wheel_speed = 轮速大小）
 * @param st    内核状态（取实测舵角）
 * @note 枚举 4 个候选，取 J 最小者；枚举顺序把**上一拍的分支**排在第一个并用严格 <
 *       ⇒ 平手时保持不动。镜像对称工况（如 ψ=90° 平移 + 两轮对称起步）下两个同分支候选的
 *       J **逐位相等**（实测 rel ≤ 7e-16），没有这条就会逐拍翻转、目标角每拍跳 180°。
 *       这不是迟滞带：只在**精确**平手时生效，且选定后状态朝该分支走、J 差随之变大
 *       ⇒ 不会来回跳，也不会像档1 那样把车锁在坏分支上（J 差一点点就换）。
 * @note 首拍锚到当前实测舵角（别一上来就把舵甩到别处），之后由本层接管。
 * @note speed == 0（内核按 LIB_CHASSIS_STEER_SPEED_EPS 判）：保持**上次目标舵角**、驱动停转。
 *       不拿实测舵角当目标：它是带噪声的归一化读数，拿它当目标等于把噪声直接喂进位置环。
 *       该轮在预测里 V=0、行程 0：不出力、只贡献"轮面不侧滑"那条约束行（它确实还压在地上）。
 */
static void ChassisRudderDecide(LibChassisInstance_s *inst, const vector3_t *twist,
                                const LibChassisRef_s *ref, const LibChassisState_s *st)
{
    ChassisRudderPred_s p = {0};
    float e[LIB_CHASSIS_HALF_RUDDER_NUM] = {0};
    float arm = 0.0f, vscale, wscale, best_j = 0.0f;
    uint8_t rev_cur[LIB_CHASSIS_HALF_RUDDER_NUM], rev_best[LIB_CHASSIS_HALF_RUDDER_NUM];
    uint8_t i, k;

    for (i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if ((s_steer_seeded & (uint8_t)(1u << i)) == 0u)
        {
            s_steer_target_last[i] = st->steer_angle[i];
            s_steer_reverse[i] = 0;
            s_steer_seeded |= (uint8_t)(1u << i);
        }
        p.d[i] = ref->steer_target[i];
        p.v[i] = ref->wheel_speed[i];
        p.phi[i] = st->steer_angle[i];
        e[i] = Lib_Math_WrapAngleNegPIToPI(ref->steer_target[i] - st->steer_angle[i]); // (-π, π]
        if (ref->wheel_speed[i] > 0.0f)
            p.moving |= (uint8_t)(1u << i);
    }

    if (p.moving == 0u)
        return; // 两轮都不转：保持上次目标舵角（§2.5 的"两轮保持同一目标角"仍留给后续）

    /* 归一键：线速度取"指令"与"自旋在轮上产生的轮速"里大的那个——纯自旋指令 vx=vy=0，
     * 不这么取就会除以 ~0（vscale 下限兜底）；偏航取 |w*|，纯平移 w*=0 时用常量当尺子。 */
    for (i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        const float ay = FABS(inst->cfg.wheel[i].y);
        if (ay > arm)
            arm = ay;
    }
    vscale = FABS(twist->x);
    if (FABS(twist->y) > vscale)
        vscale = FABS(twist->y);
    if (FABS(twist->z) * arm > vscale)
        vscale = FABS(twist->z) * arm;
    if (vscale < CHASSIS_STEER_COST_SCALE_MIN)
        vscale = CHASSIS_STEER_COST_SCALE_MIN;
    wscale = (FABS(twist->z) > CHASSIS_STEER_COST_SCALE_MIN) ? FABS(twist->z) : CHASSIS_STEER_COST_W_SCALE_MIN;

    /* 4 个候选：k 的低位异或左轮上一拍分支、高位异或右轮 —— k=0 恰好就是"保持上一拍"，
     * 配合下面的严格 `<` 实现"平手不动"（见函数注释）。 */
    rev_cur[LIB_CHASSIS_HALF_RUDDER_L] = rev_best[LIB_CHASSIS_HALF_RUDDER_L] =
        (uint8_t)s_steer_reverse[LIB_CHASSIS_HALF_RUDDER_L];
    rev_cur[LIB_CHASSIS_HALF_RUDDER_R] = rev_best[LIB_CHASSIS_HALF_RUDDER_R] =
        (uint8_t)s_steer_reverse[LIB_CHASSIS_HALF_RUDDER_R];

    for (k = 0; k < 4u; k++)
    {
        const uint8_t rl = (uint8_t)((k & 1u) ^ rev_cur[LIB_CHASSIS_HALF_RUDDER_L]);
        const uint8_t rr = (uint8_t)(((k >> 1) & 1u) ^ rev_cur[LIB_CHASSIS_HALF_RUDDER_R]);
        const float j = ChassisRudderCost(inst, twist, &p, rl, rr, vscale, wscale);

        if (k == 0u || j < best_j)
        {
            best_j = j;
            rev_best[LIB_CHASSIS_HALF_RUDDER_L] = rl;
            rev_best[LIB_CHASSIS_HALF_RUDDER_R] = rr;
        }
    }

    for (i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if ((p.moving & (uint8_t)(1u << i)) == 0u)
            continue; // 本拍不转：保持上次目标舵角
        ChassisRudderApply(i, e[i], (int8_t)rev_best[i], st->steer_angle[i]);
    }
}

/**
 * @brief 舵轮一个轮：内核输出 → 两个电机侧给定（舵角 + 驱动速度）
 * @param idx          轮序号
 * @param ref          内核逆解输出（steer_target = 期望轮向，wheel_speed = 轮速大小）
 * @param st           内核状态（取实测舵角）
 * @param steer_setref 输出：舵向电机位置给定 [电机侧 rad]
 * @param wheel_setref 输出：驱动电机速度给定 [电机侧 rad/s]（内核只给大小，符号在这里定）
 * @note 舵角参考与分支由 ChassisRudderDecide 提前算好（两轮联合决策），这里只取出。
 * @note 轮速带**余弦优化**：再乘一个 cos(参考舵角 − 实际舵角)，见下面注释。
 */
static void ChassisRudderOut(uint8_t idx, const LibChassisRef_s *ref, const LibChassisState_s *st, float *steer_setref, float *wheel_setref)
{
    float speed_wheel = ref->wheel_speed[idx]; // 轮缘速度大小（≥ 0），反向解时下面取负
    int8_t reverse = s_steer_reverse[idx];
    float steer_ref, align;

    // 舵角参考（舵轮侧连续角）：本拍换向决策定下的、位置环要跟的那个角
    steer_ref = s_steer_target_last[idx];
    *steer_setref = steer_ref * STEER_GEAR_RATIO;

    /* 余弦优化：驱动轮只有"轮面正对着希望的运动方向"时，推力才全部用在那个方向上；
     * 舵向还没转到位时轮面偏了 θ，真正有用的分量只有 cos θ，其余把车往别处拽（侧滑/擦胎）。
     * 故把轮速按 cos(参考舵角 − 实际舵角) 缩放：大角度回转过程中轮子不再全速硬推，
     * 而是随舵角到位平滑起速；到位后 cos θ ≈ 1，与不做优化时完全一致。
     * 小角度下 cos 很平（θ=10° 才降 1.5%），跟踪误差与舵角噪声几乎不会被放大。
     * ⚠ 角差必须在**舵轮侧**算：cos 不是按比例的，换成电机侧算（角差再 ×8）得到的是另一个角。
     * ⚠ **不钳负**（§2.1）：乘上 (−1)^rev 后总系数恒等于 V·cos(d − φ)，与分支无关——这才是
     *   "两个等价解推力完全相同"的代数来源。钳 0 会破坏这个恒等：舵角越过 90° 之后本该反向
     *   出力（轮子从 +V 扫到 −V、过零），钳了就是一段**无力死区**，翻转被切成两段。
     *   代价函数（ChassisRudderCost）用的就是这个不钳负的式子，两边必须同一个假设。
     * ⚠ cos 以 2π 为周期，而参考是连续角、实测被归一化在 ±π：两者相差整数个 2π 等价，
     *   故不必先归一化角差。 */
    align = Lib_Math_Cos(steer_ref - st->steer_angle[idx]);
    s_steer_align[idx] = align; // 留档给 VOFA（ch26/27），与实际乘进去的是同一个数（可负）
    *wheel_setref = WHEEL_RADPS_TO_MOTOR_RADPS(((reverse != 0) ? -speed_wheel : speed_wheel) * align);
}

/**
 * @brief 半舵解算一整套：读反馈 → 逆解出四个电机给定 → 正解回填速度反馈
 * @param cmd 云台下发的底盘速度指令（m/s、rad/s，量纲已在云台侧换算）
 */
static void HalfRudderSolve(const gimbal2chassis_data_t *cmd)
{
    LibChassisState_s st;
    ChassisReadState(&st);

    // 逆解 → 期望轮向 + 轮速大小（轮侧量；跨接口时换回电机侧，舵轮的等价解选择见上）
    // ⚠ 整条 twist 的符号补偿只在这一处，原因见 app_chassis_def.h 的 CHASSIS_WIRE_TWIST_SIGN
    //   （驱动轮轮速正方向与舵角参考方向相反 ⇒ 模型的 x/y/w 全是物理量的负）。
    LibChassisRef_s ref;
    vector3_t twist = {.x = CHASSIS_WIRE_TWIST_SIGN * cmd->vx,
                       .y = CHASSIS_WIRE_TWIST_SIGN * cmd->vy,
                       .z = CHASSIS_WIRE_TWIST_SIGN * cmd->w};
    LibChassisInverse(&s_chassis, twist, &ref);

    // 换向决策必须先走：它要同时看两轮才能保证同拍翻（见 ChassisRudderDecide），
    // 之后每个轮再各自把结果翻成两个电机侧给定。
    ChassisRudderDecide(&s_chassis, &twist, &ref, &st);
    ChassisRudderOut(LIB_CHASSIS_HALF_RUDDER_L, &ref, &st, &rudder_l_motor_setref, &wheel_l_motor_setref);
    ChassisRudderOut(LIB_CHASSIS_HALF_RUDDER_R, &ref, &st, &rudder_r_motor_setref, &wheel_r_motor_setref);

    // 正解 → 速度反馈（随 CommSend 回传云台）
    // 用"实际舵角 + 实际驱动速度"，故轮子处于反向解（朝 φ、反转）时结果同样正确。
    // 两组回转中心重合时 JᵀJ 奇异，内核返回 -1 且输出已清零（车体方位不可分辨）。
    vector3_t odom;
    if (LibChassisForward(&s_chassis, &st, &odom) != 0)
        odom = (vector3_t){0};
    // 正解在内部约定下算，回传前按线协议符号翻回去（与入口补偿严格对称，三轴一起）
    chassis2gimbal_data.vx = CHASSIS_WIRE_TWIST_SIGN * odom.x;
    chassis2gimbal_data.vy = CHASSIS_WIRE_TWIST_SIGN * odom.y;
    chassis2gimbal_data.w = CHASSIS_WIRE_TWIST_SIGN * odom.z;
}

/* 云台接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，须同步拷贝） */
static void GimbalRecvOnFrame(const uint8_t *payload)
{
    gimbal2chassis_data_t frame;

    memcpy(&frame, payload, sizeof(frame));
    gimbal_mode_rx = frame.enabled; // 档位：状态机判据，全程不被本文件的其他代码回写
    gimbal2chassis_data = frame;    // 指令：SAFE 态可以把它清零作废，不影响上面的判据
}

/**
 * @brief 云台链路是否在线
 * @retval 1 在线（daemon_reload ms 内收到过完整合法帧）；0 掉线
 * @note 看门狗实例内嵌在 media 基类里（见 comm_media.h），收帧时由 comm 层统一喂狗，
 *       阈值为 CommConfig.daemon_reload（本实例 10ms，对端每 2ms 发一帧）。
 *       未登记看门狗（daemon == NULL）按"在线"处理：不监控就不该拦控制。
 * @note DaemonIsOnline 判的是 temp_count > 0；DaemonConfig 把 temp_count 初始化成
 *       reload_count，故上电后前 10ms 报在线、之后（仍未收到帧）落回离线。
 *       这段窗口由 gimbal2chassis_data.enabled 的初值（robot_mode_stop）兜底，
 *       不会出现"没收到任何帧却带速起步"。
 */
static uint8_t GimbalCommIsOnline(void)
{
    CommMedia *media = (CommMedia *)gimbal_comm.media; // 首成员即 CommMedia 基类，直接转
    return (media != NULL && media->daemon != NULL) ? DaemonIsOnline(media->daemon) : 1;
}

/*============================ 底盘顶层状态机（lib_fsm_table） ============================*/
/* 状态 = 底盘"现在能干的事"，不是云台下发的档位：
 *   CAL   上电校准：两舵轮扫描找光电门零点（此时不理会云台在线与否）
 *   FAULT 校准失败（超时）：失能 + 给定清零，终态（需复位重启再标）
 *   SAFE  已标定但不可动：云台掉线 或 云台下发 stop —— 失能 + 给定清零 + 作废指令
 *   RUN   已标定且可动：使能 + 逆解/正解
 * 产出：本状态下四电机的使能与给定（见 AppChassisRun 的 switch）。
 * 事件只有一个"每拍更新"：状态完全由 (是否标定, 是否超时, 是否在线, 是否 stop) 决定，
 * 与云台状态机同构——守卫划分状态空间、表顺序即优先级。
 * ⚠ 校准是"上电即可做、与云台无关"的第一步：云台常没上电，故 CAL 不受 online 影响；
 *   若把它一并拦掉，底盘会永远停在"未标定"状态起不了步（旧代码的同一顾虑）。 */
typedef enum : uint8_t
{
    chassis_state_cal_e = 0, // 上电校准中
    chassis_state_fault_e,   // 校准超时（故障）
    chassis_state_safe_e,    // 未使能 / 云台掉线
    chassis_state_run_e,     // 使能运行
    chassis_state_count_e,   // 状态值域上界（= 动作表项数）
} chassis_state_e;

typedef enum : uint8_t
{
    chassis_ev_update_e = 0, // 每拍更新（把下面四个条件重新汇总）
} chassis_event_e;

typedef struct
{
    uint8_t cal_done;    // 两舵轮均已标定
    uint8_t cal_fault;   // 校准故障（连续超时到上限，终态）
    uint8_t cal_retries; // 已发生的单次超时次数（未标定且计时超限就 +1 并重新计时）
    uint8_t online;      // 云台链路在线
    uint8_t stopped;     // 云台下发模式为 stop
    uint32_t cal_ticks;  // 本轮校准已持续拍数（单次超时判据）
    uint8_t key_last;    // 上次派发时的条件打包（变化才派发，见 ChassisFsmUpdate）
} ChassisFsmCtx_s;

static ChassisFsmCtx_s chassis_fsm_ctx;
static LibFsmTableInstance_s chassis_fsm;

/*---------------- 守卫 ----------------*/
/* 守卫必须无副作用且幂等：同 from/event 的多条规则会依次求值（见 lib_fsm_table.h）。
 * 下面四个条件互斥，故表内先后只决定"故障优先、校准次之"这一条语义。 */
static bool ChassisGuardFault(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((ChassisFsmCtx_s *)ctx)->cal_fault != 0;
}
static bool ChassisGuardCal(void *ctx, const void *event_data)
{
    (void)event_data;
    ChassisFsmCtx_s *c = (ChassisFsmCtx_s *)ctx;
    return (c->cal_done == 0) && (c->cal_fault == 0);
}
static bool ChassisGuardSafe(void *ctx, const void *event_data)
{
    (void)event_data;
    ChassisFsmCtx_s *c = (ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && ((c->online == 0) || (c->stopped != 0));
}
static bool ChassisGuardRun(void *ctx, const void *event_data)
{
    (void)event_data;
    ChassisFsmCtx_s *c = (ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && (c->online != 0) && (c->stopped == 0);
}

/*---------------- 转移表 ----------------*/
/* 源状态一律通配：四条守卫已把 (标定 × 在线 × stop) 完全划分，落点唯一；
 * 表顺序 = 优先级，故障排最前（一旦超时，无论别的条件如何都进故障）。 */
static const LibFsmTableTransition_s chassis_fsm_table[] = {
    {LIB_FSM_TABLE_ANY_STATE, chassis_ev_update_e, chassis_state_fault_e, ChassisGuardFault, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, chassis_ev_update_e, chassis_state_cal_e, ChassisGuardCal, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, chassis_ev_update_e, chassis_state_safe_e, ChassisGuardSafe, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, chassis_ev_update_e, chassis_state_run_e, ChassisGuardRun, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
};
LIB_FSM_TABLE_CHECK_TABLE(chassis_fsm_table);

/*---------------- 进入动作 ----------------*/
/* 校准结束进入运行的这一拍：舵向从"纯速度环扫描"切到"位置+速度环"，此后才由半舵解算
 * 接管舵角。放进入动作里，只在跨状态那一刻执行一次。
 * ⚠ loop_type 在 drvlib 里是实例配置——Config 时按 ANGLE|SPEED 建好两个环，校准期由
 *   ChassisCalibrateTick 收窄成 SPEED，这里再放开；只改掩码，不重配 PID。 */
static void ChassisRunEntry(void *ctx)
{
    (void)ctx;
    rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
    rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED;
    // 偏航闭环重新起算：积分项清零，低通在下一拍用当前角速度播种
    ChassisYawLoopInit();
}

/* 进入故障（终态）：上报一次。失能 + 给定清零在 AppChassisRun 的 FAULT 分支里做。
 * 打日志是为了让"上电后转一会儿就不动"这类现象能直接看到原因，不再静默卡死。 */
static void ChassisFaultEntry(void *ctx)
{
    (void)ctx;
    BSPLOG(&g_chassis_log, LOG_LEVEL_ERROR, "cal fault: no photogate edge after %u retries x %u ticks; chassis disabled, reset MCU to retry",
           (unsigned)CHASSIS_CAL_MAX_RETRIES, (unsigned)CHASSIS_CAL_TIMEOUT_TICKS);
}

static const LibFsmTableStateActions_s chassis_fsm_actions[] = {
    [chassis_state_cal_e] = {NULL, NULL},
    [chassis_state_fault_e] = {ChassisFaultEntry, NULL},
    [chassis_state_safe_e] = {NULL, NULL},
    [chassis_state_run_e] = {ChassisRunEntry, NULL},
};
LIB_FSM_TABLE_CHECK_ACTIONS(chassis_fsm_actions, chassis_state_count_e);

/**
 * @brief 四电机统一使能/失能
 * @note 舵向在 drvlib（软件使能标志，失能顺带 PIDReset），车轮仍在旧 drv_motor
 */
static void ChassisMotorsEnable(void)
{
    DrvlibMotorEnable(&rudder_l_ctrl);
    DrvlibMotorEnable(&rudder_r_ctrl);
    MotorEnable(&(wheel_l_motor.base));
    MotorEnable(&(wheel_r_motor.base));
}
static void ChassisMotorsDisable(void)
{
    DrvlibMotorDisable(&rudder_l_ctrl); // 置 disable + PIDReset，与旧 MotorDisable 同义
    DrvlibMotorDisable(&rudder_r_ctrl);
    MotorDisable(&(wheel_l_motor.base));
    MotorDisable(&(wheel_r_motor.base));
}

/**
 * @brief 校准判据：光电门跳变（1→0 下降沿）处重标零点并置完成标志
 * @note 必须在舵向 SetRef **之后**调用：标定读的是控制核本拍刚刷新过的 data.position，
 *       用上一拍的会多出 2ms 采样滞后（标定转速下约 0.3° 舵轮，不可忽略）。
 * @note 两个都标定完 → rudder_*_inited 置位，下一拍由状态机切到运行态；
 *       计时超限仍未标定 → 由 ChassisFsmUpdate 判故障。
 */
static void ChassisCalibPollGate(void)
{
    // 光电门跳变（1→0 下降沿）处重标零点
    if ((rudder_l_state == 1) && (0 == GPIORead(&rudder_l_gate_io)))
    {
        RudderCalibrateOffset_l();
    }
    if ((rudder_r_state == 1) && (0 == GPIORead(&rudder_r_gate_io)))
    {
        RudderCalibrateOffset_r();
    }
    // 记录本次电平，供下拍判边沿
    rudder_l_state = GPIORead(&rudder_l_gate_io);
    rudder_r_state = GPIORead(&rudder_r_gate_io);
}

/**
 * @brief 每拍汇总条件，仅在条件变化时派发一次更新事件
 * @note 状态完全由四个布尔量决定；若每拍都派发，同状态也会走一次外部自转移
 *       （退出 + 进入各跑一遍），将来挂上状态动作就会每拍重跑，故只在变化时派发。
 * @note 故障为终态：进入后不再派发（需复位重启才能重标）。
 */
static void ChassisFsmUpdate(void)
{
    ChassisFsmCtx_s *c = &chassis_fsm_ctx;

    /* 校准计时：仅在校准态累加。单次超时只重试（重新计时 + 计数 + 上报），不直接判死——
     * 上电找不到零点最常见的失败是漏采到某一圈的光电门沿，下一圈就能补上；
     * 连续 CHASSIS_CAL_MAX_RETRIES 次才判故障（那时基本是传感器/线束/挡片真坏了）。 */
    if (LibFsmTableCurrent(&chassis_fsm) == chassis_state_cal_e)
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
    c->cal_done = (uint8_t)(rudder_l_inited && rudder_r_inited);
    c->cal_fault = (uint8_t)((c->cal_done == 0) && (c->cal_retries >= CHASSIS_CAL_MAX_RETRIES));
    c->online = GimbalCommIsOnline();
    /* 档位取 gimbal_mode_rx（接收回调写的原始副本），**不能取 gimbal2chassis_data.enabled**：
     * 采样发生在状态动作之后（同一拍里 chassis_state_safe_e 先把 gimbal2chassis_data 清零
     * 作废，再走到这里），若判据用同一份数据，SAFE 每拍都会读到"自己刚写下的 stop" ⇒
     * stopped 恒 1 ⇒ ChassisGuardRun 永远不成立，SAFE 变成出不去（只在 ISR 恰好插进
     * "清零→采样"那零点几微秒的窗口时才能偶然逃出去）。档位是"对端说了什么"，
     * 只能由接收回调更新，与"本拍要不要执行这条指令"（SAFE 清零）必须分开存。 */
    c->stopped = (uint8_t)(gimbal_mode_rx == robot_mode_stop);

    // 打包成一个字节比较：任一条件变化就派发一次
    uint8_t key = (uint8_t)(c->cal_done | (c->cal_fault << 1) | (c->online << 2) | (c->stopped << 3));
    if ((key != c->key_last) && (LibFsmTableCurrent(&chassis_fsm) != chassis_state_fault_e))
    {
        LibFsmTableDispatch(&chassis_fsm, chassis_ev_update_e, NULL);
    }
    c->key_last = key;
}

void AppChassisInit(void)
{
    // 半舵运动学内核：几何尺寸全部来自 robot_def.h（两组回转中心位置 + 驱动轮半径）。
    // 内侧按"每组 1 个舵轮 2 行约束"组装，2 组共 4 行 3 未知量，正解即最小二乘里程计。
    LibChassisHalfRudderGeom_s chassis_geom = {
        .l_pos_x = CHASSIS_RUDDER_L_POS_X,
        .l_pos_y = CHASSIS_RUDDER_L_POS_Y,
        .r_pos_x = CHASSIS_RUDDER_R_POS_X,
        .r_pos_y = CHASSIS_RUDDER_R_POS_Y,
        .radius = CHASSIS_WHEEL_RADIUS,
    };
    BSP_ASSERT_APP_CALL(LibChassisHalfRudderInit(&s_chassis, &chassis_geom));

    // 注册 CAN 实例（四个电机共用 CAN_2）
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&rudder_l_motor));
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastRegister(&rudder_r_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_l_motor));
    BSP_ASSERT_APP_CALL(LKMotorBroadcastRegister(&wheel_r_motor));

    /* ---- 舵向协议层（drvs_djimotor_broadcast）----
     * 两个 M3508 同挂 CAN_2 的 0x201~0x204 组：id2 → rx 0x202、id4 → rx 0x204，
     * tx 均为 0x200，故两舵向共用一帧下发（一个广播组 rudder_motor_group）。 */
    DrvsDJIMotorBroadcastConfig_s rudder_l_cfg = {
        .group = &rudder_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 2,
        .torque_constant = 1, // 转矩常数 Kt (Nm/A)，M3508 待标定（沿用旧 drv_motor 的 Kt=1 占位）
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    DrvsDJIMotorBroadcastConfig_s rudder_r_cfg = {
        .group = &rudder_motor_group,
        .can_e = CAN_2,
        .model = DRVS_DJI_MODEL_M3508,
        .motor_id = 4,
        .torque_constant = 1, // 同 rudder_l
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1,
    };
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&rudder_l_motor, &rudder_l_cfg));
    BSP_ASSERT_APP_CALL(DrvsDJIMotorBroadcastConfig(&rudder_r_motor, &rudder_r_cfg));

    /* ---- 舵向控制核（drvlib_motor）：左右舵同参数，一份 cfg 配两个实例 ----
     * 复刻旧 drv_motor 的语义：
     *   loop_type   ANGLE|SPEED 两环都建（校准期由 ChassisCalibrateTick 临时收窄成 SPEED）
     *   位置模式    WRAP，归一化范围 ±π×减速比（舵轮一周 = 电机 STEER_GEAR_RATIO 圈）
     *   position_span 单圈 2π（M3508 编码器跨度），供多圈累加
     *   方向        左右舵旧值均为 NORMAL（feedback/motor 须成对；只反一个会变成正反馈）
     *   低通        速度/电流 20ms（旧 speed_lpf_rc=0.02）
     * Config 会自动给位置/速度环补 TRAPEZOID_INTEGRAL，WRAP 下再给位置环补误差归一化。 */
    DrvlibMotor_Config_s rudder_ctrl_cfg = {
        .loop_type = DRVLIB_MOTOR_LOOP_ANGLE | DRVLIB_MOTOR_LOOP_SPEED,
        .motor_direction = DRVLIB_MOTOR_DIRECTION_NORMAL,    // 输出方向
        .feedback_direction = DRVLIB_MOTOR_DIRECTION_NORMAL, // 反馈方向
        .position_mode = DRVLIB_MOTOR_POS_WRAP,
        .angle_limit_min = -M_PI * STEER_GEAR_RATIO,
        .angle_limit_max = M_PI * STEER_GEAR_RATIO,
        .position_span = 6.2831853f, // 单圈 2π
        .position_offset = 0,        // 光电门标定时由 RudderCalibrateOffset 改写
        .lpf_enable = DRVLIB_MOTOR_LPF_ENABLE,
        .lpf_rc = 0.02f,
        .angle_feedforward_src = DRVLIB_MOTOR_FF_DISABLE, // 前馈来源
        .speed_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .current_feedforward_src = DRVLIB_MOTOR_FF_DISABLE,
        .angle_feedforward_ptr = NULL, // 前馈指针
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        /* PID 参数与整定依据见 app_chassis_def.h（CHASSIS_RUDDER_PID_*）；
         * 这里只留各环的使能位（config_mask）——那是"开哪些功能"的接线，不是调参量。 */
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
                                       PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行（避免目标阶跃的微分冲击）
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
                                       PID_ENABLE_OUTPUT_LIMIT,     // 启用输出限幅（位置环输出不再无限大）
                    },
            },
        .dt_max = 0.01f,  // 单帧 dt 上限 (s)
        .vofa_enable = 0, // 本 app 的 VOFA 通道自定义，不用控制核内置 CH1~CH3
    };
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&rudder_l_ctrl, &rudder_ctrl_cfg));
    BSP_ASSERT_APP_CALL(DrvlibMotorConfig(&rudder_r_ctrl, &rudder_ctrl_cfg)); // 左右舵同参数

    // 避震方向看过去逆时针转动为正方向
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
        .controller_setting =
            {
                .loop_type = MOTOR_LOOP_SPEED,                 // TODO: 后续改为速度/位置环
                .feedback_direction = MOTOR_DIRECTION_REVERSE, // 反馈方向
                .motor_direction = MOTOR_DIRECTION_REVERSE,    // 输出方向
                .position_mode = MOTOR_POSITION_CONTINUOUS,    // 开环占位，不用位置环
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
        /* PID 参数与整定依据见 app_chassis_def.h（CHASSIS_WHEEL_PID_SPEED_*，左右轮共用）；
         * config_mask 留在这里，是"开哪些功能"的接线，不是调参量。 */
        .pid_speed_setting =
            {
                .kp = CHASSIS_WHEEL_PID_SPEED_KP,
                .ki = CHASSIS_WHEEL_PID_SPEED_KI,
                .kd = CHASSIS_WHEEL_PID_SPEED_KD,
                .integral_limit = CHASSIS_WHEEL_PID_SPEED_INTEGRAL_LIMIT,
                .coef_a = CHASSIS_WHEEL_PID_SPEED_COEF_A,
                .coef_b = CHASSIS_WHEEL_PID_SPEED_COEF_B,
                .d_lpf_rc = CHASSIS_WHEEL_PID_SPEED_D_LPF_RC,
                .out_lpf_rc = CHASSIS_WHEEL_PID_SPEED_OUT_LPF_RC,
                .deadband = CHASSIS_WHEEL_PID_SPEED_DEADBAND,
                .error_normalize_range = CHASSIS_WHEEL_PID_SPEED_ERR_NORM_RANGE,
                .out_max = CHASSIS_WHEEL_PID_SPEED_OUT_MAX,
                .out_min = CHASSIS_WHEEL_PID_SPEED_OUT_MIN,
                .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL |   // 梯形积分
                               PID_ENABLE_INTEGRAL_LIMIT |       // 积分限幅
                               PID_ENABLE_CHANGING_INTEGRATION | // 变速积分
                               PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行（kd=0，暂不生效，留作扩展）
                               PID_ENABLE_DERIVATIVE_FILTER,     // 微分滤波
            },
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(LKMotorBroadcastConfig(&wheel_l_motor, &wheel_l_cfg));

    // 避震方向看过去逆时针转动为正方向
    // 配置 wheel_r（LK MF7015V1-24V-23T，广播模式 motor_id=2 → 槽位 1，回复 ID 0x142）
    LKMotorBroadcast_Config_s wheel_r_cfg = {
        .can_e = CAN_2,
        .model = LK_MODEL_MF,
        .motor_id = 2,
        .torque_constant = 0.28f, // MF7015-24V-23T，同 wheel_l
        .speed_lpf_enable = MOTOR_SPEED_LPF_ENABLE,
        .speed_lpf_rc = 0.004f,
        .position_offset = 0,
        .controller_setting =
            {
                .loop_type = MOTOR_LOOP_SPEED,                // TODO: 后续改为速度/位置环
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
        /* PID 参数与整定依据见 app_chassis_def.h（CHASSIS_WHEEL_PID_SPEED_*，左右轮共用）；
         * config_mask 留在这里，是"开哪些功能"的接线，不是调参量。 */
        .pid_speed_setting =
            {
                .kp = CHASSIS_WHEEL_PID_SPEED_KP,
                .ki = CHASSIS_WHEEL_PID_SPEED_KI,
                .kd = CHASSIS_WHEEL_PID_SPEED_KD,
                .integral_limit = CHASSIS_WHEEL_PID_SPEED_INTEGRAL_LIMIT,
                .coef_a = CHASSIS_WHEEL_PID_SPEED_COEF_A,
                .coef_b = CHASSIS_WHEEL_PID_SPEED_COEF_B,
                .d_lpf_rc = CHASSIS_WHEEL_PID_SPEED_D_LPF_RC,
                .out_lpf_rc = CHASSIS_WHEEL_PID_SPEED_OUT_LPF_RC,
                .deadband = CHASSIS_WHEEL_PID_SPEED_DEADBAND,
                .error_normalize_range = CHASSIS_WHEEL_PID_SPEED_ERR_NORM_RANGE,
                .out_max = CHASSIS_WHEEL_PID_SPEED_OUT_MAX,
                .out_min = CHASSIS_WHEEL_PID_SPEED_OUT_MIN,
                .config_mask = PID_ENABLE_TRAPEZOID_INTEGRAL |   // 梯形积分
                               PID_ENABLE_INTEGRAL_LIMIT |       // 积分限幅
                               PID_ENABLE_CHANGING_INTEGRATION | // 变速积分
                               PID_ENABLE_DERIVATIVE_ON_MEAS |   // 微分先行（kd=0，暂不生效，留作扩展）
                               PID_ENABLE_DERIVATIVE_FILTER,     // 微分滤波
            },
        .reload_count = 100,
        .fault_action = DAEMON_FAULT_NONE,
        .timeout_ms = 1, // CAN 发送超时(ms)
    };
    BSP_ASSERT_APP_CALL(LKMotorBroadcastConfig(&wheel_r_motor, &wheel_r_cfg));

    DrvlibMotorEnable(&rudder_l_ctrl); // 舵向：drvlib 软件使能
    DrvlibMotorEnable(&rudder_r_ctrl);
    MotorEnable(&(wheel_l_motor.base)); // 车轮：旧 drv_motor 广播栈
    MotorEnable(&(wheel_r_motor.base));

    // 注册并配置两个光电门（改为控制任务轮询：回调置 NULL，不占 EXTI 分发）
    /* ⚠️ 侧别↔引脚：实机曾出现"转右舵只进 RudderPhotogate_l_Callback"，而当时 l 回调
     *    绑的是 PA2(GPIO_PWM_2)，说明右舵门在 PA2、左舵门在 PA0，故按下面填；
     *    改线后把这两行的 gpio_e 对调即可（实例名/回调/被标定电机三者保持同侧）。 */
    rudder_l_gate_io.parent = &rudder_l_motor; // parent 仅作身份绑定；回调为 NULL，轮询不用它
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

    // 云台通信（CAN1 / IDSEQ / EXT）：登记 + 配置介质（ID 段/帧格式）+ 链路看门狗。
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
        .daemon_fault = DAEMON_FAULT_BUZZER_SHORT,
        .daemon_reload = 10, /* 对端每 2ms 发一帧，超 10ms 判离线 */
    };
    BSP_ASSERT_APP_CALL(CommRegister(&gimbal_comm));
    BSP_ASSERT_APP_CALL(CommConfig(&gimbal_comm, &gimbal_comm_cfg));

    /* ---- 底盘 IMU（BMI088 + 线性卡尔曼）：纯观测，见文件上面那两个常量/实例的注释 ----
     * 硬件参数与噪声参数逐项抄自 app_gimbal.c（同一辆车、同一套栈、同一种安装环境），
     * 差别只有三处：标定值（器件属性，见 s_bmi088_calib）、daemon_fault（观测件不拦控制）、
     * vofa_enable（必须为 0，见下）。参数取值的依据见 drvlib_bmi088_kalman.h 头注释。
     * ⚠ vofa_enable 保持 0：它会让 drvlib 自己把数据写进 CH1~CH15 并**自己发一帧**，
     *   而本 app 的 ch1~27 已经给了舵向控制核、帧也由 AppChassisRun 统一发（同一时刻只能
     *   一路在写缓冲区，混写就是互相覆盖）。本 app 要的 6 路自己写进 ch28~33。 */
    BSP_ASSERT_APP_CALL(BMI088KalmanRegister(&bmi088));
    BMI088Kalman_Config_s bmi088_cfg = {
        .imu =
            {
                .spi_e = SPI_BMI088,
                .cs_acc_e = GPIO_BMI088_CS_ACCEL,
                .cs_gyro_e = GPIO_BMI088_CS_GYRO,
                .int_acc_e = GPIO_BMI088_INT_ACCEL,
                .int_gyro_e = GPIO_BMI088_INT_GYRO,
                .daemon_reload = 20,                // 20ms 没喂狗判掉线（IMU 自由运行 500Hz~1kHz）
                .daemon_fault = DAEMON_FAULT_NONE,  // 观测件：掉线只影响这几路通道，不停车
                .acc_range = BMI088_ACC_RANGE_3G,   // ±3g，只做低频倾角校正（同云台）
                .acc_bwp = BMI088_ACC_BWP_NORMAL,   // 正常带宽（ODR>400Hz 才必须切 OSR 模式）
                .acc_odr = BMI088_ACC_ODR_400,      // 400Hz：倾角校正够用
                .gyro_range = BMI088_GYRO_RANGE_2000,
                .gyro_conf = BMI088_GYRO_CONF_1000_116, // ODR=1000Hz/BW=116Hz（任务 2ms，2000Hz 用不上）
                .work_mode = BMI088_MODE_INT,           // DRDY EXTI 里发起采样，时间戳不含 SPI 往返
                .spi_mode = BSP_DMA_MODE,               // INT 模式下只能用 IT/DMA（BLOCK 会在中断里死等）
                .spi_timeout_ms = 10,
            },
        .calib = &s_bmi088_calib,
        .q_tilt = 1e-5f,
        .q_bias = 0.0f, // 0 = 关掉零偏残差状态（与云台同：标定值覆盖慢漂，再开一个状态会互相打架）
        .r_tilt = 4e-4f,
        .dt_max = 0.01f,        // 任务被拖延时钳位单步积分
        .temp_lpf_alpha = 0.01f, // ≈0.2s@500Hz；bias_tempco 未标定时无影响
        .vofa_enable = 0,        // 帧由本 app 统一发，见上
    };
    BSP_ASSERT_APP_CALL(BMI088KalmanConfig(&bmi088, &bmi088_cfg));

    // 底盘顶层状态机：上电从"校准"起（旧代码的 lunxunjioazhun 第一拍也是从零状态开始扫），
    // 等两个光电门标定完再判在线/使能决定进 RUN 还是 SAFE。
    LibFsmTableConfig_s chassis_fsm_cfg = {
        .initial_state = chassis_state_cal_e,
        .transitions = chassis_fsm_table,
        .transition_count = sizeof(chassis_fsm_table) / sizeof(chassis_fsm_table[0]),
        .state_actions = chassis_fsm_actions, // 仅 RUN 的进入动作（切舵向环型）
        .state_count = chassis_state_count_e,
        .ctx = &chassis_fsm_ctx,
        .trace_fn = NULL,
    };
    LibFsmTableInit(&chassis_fsm, &chassis_fsm_cfg);
    LibFsmTableStart(&chassis_fsm);
}

/**
 * @brief 把一个舵向控制核的一整套观测量写进连续 12 个 VOFA 通道（自外环向内环再到实际反馈）
 * @param ch_base       起始通道：写 ch_base ~ ch_base+11
 * @param ctrl          舵向控制核（drvlib_motor）
 * @param steer_ref_motor 本拍喂给控制核的舵角给定（电机侧 rad；CAL 态是扫描速度，见调用处）
 * @param raw_speed     本拍 drvs 原始转速 (rad/s)：控制核未使能时 data 是旧值，用它顶替
 * @param raw_current   本拍 drvs 原始电流 (A)：同上
 * @note 通道顺序：
 *   +0  舵角给定(舵轮侧 rad)   +1  舵角实际(舵轮侧 rad)   +2  角度环误差(舵轮侧 rad)
 *   +3  角度环 p_out(rad/s)    +4  角度环 d_out(rad/s)    +5  角度环输出 = 速度给定(rad/s)
 *   +6  速度实际(rad/s)        +7  速度环误差(rad/s)
 *   +8  速度环 p_out(A)        +9  速度环 i_out(A)        +10 下发力矩/电流(A)
 *   +11 实际电流(A)
 * @note 舵角类（+0/+1/+2）换算到舵轮侧（÷STEER_GEAR_RATIO）便于对着机械角读；
 *       其余是控制核原生量纲（rad/s、A）。+10 是位置+速度环的最终输出，即下发电流。
 * @note 控制核未使能（SAFE/FAULT）时它不喂反馈、data 会**冻结**在最后一拍（曾出现"失能了
 *       还在显示 24rad/s / 1.5A"的假象）：这种状态下环路项全写 0，速度/电流改用 drvs 本拍
 *       原始量（仍然真实）。所以看到一群 0 + 真实转速 = 舵向失能，不是故障。
 */
static void ChassisVofaRudder(uint8_t ch_base, const DrvlibMotorInstance *ctrl, float steer_ref_motor, float raw_speed, float raw_current)
{
    if (ctrl->enable == 0)
    {
        for (uint8_t i = 0; i < 12; i++)
        {
            VofaSetChannel(ch_base + i, 0.0f);
        }
        VofaSetChannel(ch_base + 6, raw_speed);
        VofaSetChannel(ch_base + 11, raw_current);
        return;
    }

    const PIDInstance *pid_angle = &ctrl->pid[DRVLIB_MOTOR_STAGE_ANGLE];
    const PIDInstance *pid_speed = &ctrl->pid[DRVLIB_MOTOR_STAGE_SPEED];
    VofaSetChannel(ch_base + 0, steer_ref_motor * STEER_GEAR_RATIO_INV);            // 舵角给定
    VofaSetChannel(ch_base + 1, (float)ctrl->data.position * STEER_GEAR_RATIO_INV); // 舵角实际
    VofaSetChannel(ch_base + 2, pid_angle->error * STEER_GEAR_RATIO_INV);           // 角度环误差
    VofaSetChannel(ch_base + 3, pid_angle->p_out);                                  // 角度环 p_out
    VofaSetChannel(ch_base + 4, pid_angle->d_out);                                  // 角度环 d_out
    VofaSetChannel(ch_base + 5, pid_angle->output);                                 // 角度环输出=速度给定
    VofaSetChannel(ch_base + 6, ctrl->data.speed);                                  // 速度实际
    VofaSetChannel(ch_base + 7, pid_speed->error);                                  // 速度环误差
    VofaSetChannel(ch_base + 8, pid_speed->p_out);                                  // 速度环 p_out
    VofaSetChannel(ch_base + 9, pid_speed->i_out);                                  // 速度环 i_out
    VofaSetChannel(ch_base + 10, pid_speed->output);                                // 下发力矩/电流
    VofaSetChannel(ch_base + 11, ctrl->data.current);                               // 实际电流
}

ITCM_RAM void AppChassisRun(float dt, uint64_t time_stamp)
{
    (void)time_stamp;
    const float dt_s = dt * 0.001f; // drvlib 的 dt 要秒；APP_TASK_DEF 给的 dt 是毫秒（见 bsp_app.h）

    /* 底盘 IMU：每拍更新一次（模块内部按陀螺时间戳差分算 dt，与调用抖动无关）。
     * 结果用于两处：① 观测通道 ch28/29/36~43；② RUN 态的偏航闭环（ChassisYawLoopStep）。
     * valid=0（还没用加速度计播种 / 这拍没数据）时 data 保持上一次的值或全 0；
     * 闭环会用 valid/time_stamp_g 自行判可用性并旁路。 */
    BMI088KalmanUpdate(&bmi088);
    const BMI088Kalman_Data_t imu_data = BMI088KalmanGetData(&bmi088);

    /* 采两舵向反馈：drvs 纯协议原始量 → drvlib 反馈结构（方向/多圈/偏置/低通都交给控制核） */
    DrvsDJIMotorBroadcastData_s rudder_l_data = DrvsDJIMotorBroadcastGetData(&rudder_l_motor);
    DrvsDJIMotorBroadcastData_s rudder_r_data = DrvsDJIMotorBroadcastGetData(&rudder_r_motor);
    DrvlibMotorFeedback_s rudder_l_fb = {
        .position = rudder_l_data.position,
        .speed = rudder_l_data.speed,
        .current = rudder_l_data.current,
        .timestamp_us = rudder_l_data.timestamp_us,
    };
    DrvlibMotorFeedback_s rudder_r_fb = {
        .position = rudder_r_data.position,
        .speed = rudder_r_data.speed,
        .current = rudder_r_data.current,
        .timestamp_us = rudder_r_data.timestamp_us,
    };

    /* 本拍两舵向下发量（电流 A；Kt=1 下与 drvs 要的 Nm 数值相等）。失能态保持 0。 */
    float rudder_l_cmd = 0.0f;
    float rudder_r_cmd = 0.0f;

    /* 本拍状态：由上一拍的派发决定，本拍内不再变（下一拍的派发在末尾，见下）。
     * 顺序：① 按状态给出给定（舵向在这里喂本拍反馈 + 算下发量）→ ② 校准判光电门（用 ①
     *       刚刷新的 data.position）→ ③ 采样条件并（变化时）派发 → ④ 下发。
     * ⚠ 每拍每个舵向只跑一次 SetRef（① 里各调一次）；② 不能在 ① 之前（标定要本拍位置），
     *   ③ 放到最后则状态变化下一拍生效（与旧 lunxunjioazhun 的 1 拍节奏一致）。 */
    const chassis_state_e state = (chassis_state_e)LibFsmTableCurrent(&chassis_fsm);

    switch (state)
    {
    case chassis_state_cal_e:
        // 扫描：舵向用纯速度环（位置环留到运行态），两舵轮以恒定速度转着找光电门
        rudder_l_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_SPEED;
        rudder_r_ctrl.cfg.loop_type = DRVLIB_MOTOR_LOOP_SPEED;
        DrvlibMotorEnable(&rudder_l_ctrl);
        DrvlibMotorEnable(&rudder_r_ctrl);
        rudder_l_cmd = DrvlibMotorSetRef(&rudder_l_ctrl, jiaozhun_speed, &rudder_l_fb, dt_s);
        rudder_r_cmd = DrvlibMotorSetRef(&rudder_r_ctrl, jiaozhun_speed, &rudder_r_fb, dt_s);
        rudder_l_motor_setref = jiaozhun_speed; // 供 VOFA 观测
        rudder_r_motor_setref = jiaozhun_speed;
        wheel_l_motor_setref = 0; // 校准期车轮不动（其余流程照旧下发 0）
        wheel_r_motor_setref = 0;
        break;

    case chassis_state_fault_e:
        /* 校准超时（传感器坏/线束断/挡片装反/堵转）：失能 + 给定清零，终态。
         * 不自动重试：多半是硬件真坏了，重标也不会好，需人工排查后复位重启。 */
        ChassisMotorsDisable();
        rudder_l_motor_setref = 0;
        rudder_r_motor_setref = 0;
        wheel_l_motor_setref = 0;
        wheel_r_motor_setref = 0;
        break;

    case chassis_state_safe_e:
        /* 云台链路看门狗：掉线与云台主动失能走同一条停车路径（四个给定清零 + 四电机失能）。
         * 判据在守卫里（ChassisGuardSafe）：掉线时最后一帧下的无论是"使能 + 速度"还是别的，
         * 一律不予执行——否则车会捧着几秒前的那帧速度指令一直跑下去。 */
        ChassisMotorsDisable();
        // 给定一并清零，避免下次使能瞬间残留上一拍的舵角/速度
        rudder_l_motor_setref = 0;
        rudder_r_motor_setref = 0;
        wheel_l_motor_setref = 0;
        wheel_r_motor_setref = 0;
        /* 收到的指令也一并作废，防止"链路刚恢复"那一拍：喂狗（置在线）与解包写
         * gimbal2chassis_data 之间有微秒级窗口——本任务若正好插在中间，会拿着掉线
         * 前的旧帧当新帧执行。清零后该窗口最多执行一拍"停车"，失效方向是安全的
         * （若插空失败、读到的是 ISR 刚写好的新帧，那本就是一帧可用指令）。
         * ⚠ 只清"指令"，勿动"档位"：状态机的 stopped 判据必须取 gimbal_mode_rx，
         *   若改回取这里的 gimbal2chassis_data.enabled，SAFE 会按死自己的退出条件
         *   而卡死（见 gimbal_mode_rx 与 ChassisFsmUpdate 的注释）。 */
        gimbal2chassis_data.enabled = robot_mode_stop;
        gimbal2chassis_data.vx = 0;
        gimbal2chassis_data.vy = 0;
        gimbal2chassis_data.w = 0;
        break;

    case chassis_state_run_e:
    {
        ChassisMotorsEnable();
        /* 偏航闭环：在**物理/线协议约定**下修正 w（CHASSIS_WIRE_TWIST_SIGN 之前）。
         * 修正后的指令只用于本拍解算；gimbal2chassis_data 本身不改，ch30 仍记原始指令。 */
        gimbal2chassis_data_t cmd_eff = gimbal2chassis_data;
        cmd_eff.w = ChassisYawLoopStep(cmd_eff.w, &imu_data, dt,
                                       (float)rudder_l_ctrl.data.position * STEER_GEAR_RATIO_INV,
                                       (float)rudder_r_ctrl.data.position * STEER_GEAR_RATIO_INV);
        HalfRudderSolve(&cmd_eff); // 逆解出四个给定 + 正解回填速度反馈
        /* 舵向：位置+速度环算出下发力矩交给 drvs（Kt=1，A 与 Nm 数值相同）。
         * ⚠ 顺序：HalfRudderSolve 读的是上一拍 SetRef 刷新的 data.position（本拍反馈在
         *    SetRef 里才处理）——即几何解算的舵角滞后一拍(2ms)。舵向变化慢，可忽略；
         *    想要零滞后就得把"处理反馈"和"跑环"拆开，drvlib 没提供该接口。 */
        rudder_l_cmd = DrvlibMotorSetRef(&rudder_l_ctrl, rudder_l_motor_setref, &rudder_l_fb, dt_s);
        rudder_r_cmd = DrvlibMotorSetRef(&rudder_r_ctrl, rudder_r_motor_setref, &rudder_r_fb, dt_s);
        break;
    }

    default:
        // chassis_state_count_e 不是合法状态，枚举新增状态忘了加分支会落到这里
        break;
    }

    // 校准判据：用上面 SetRef 刚刷新的 data.position（见函数注释），早一拍位置就旧了
    if (state == chassis_state_cal_e)
    {
        ChassisCalibPollGate();
    }

    // 采样四个条件 + 变化才派发；状态变化从下一拍生效
    ChassisFsmUpdate();

    /* 下发：舵向走 drvs（一次组帧发两台 3508，故两个 SetRef 必须都在 GroupSend 之前完成）；
     * 车轮仍走旧 drv_motor 广播栈（每个广播组任一成员调用即发整组帧）。失能态 cmd=0。 */
    DrvsDJIMotorBroadcastSetRef(&rudder_l_motor, rudder_l_cmd);
    DrvsDJIMotorBroadcastSetRef(&rudder_r_motor, rudder_r_cmd);
    DrvsDJIMotorBroadcastGroupSend(&rudder_motor_group);
    MotorSetRef(&(wheel_l_motor.base), wheel_l_motor_setref);
    MotorSetRef(&(wheel_r_motor.base), wheel_r_motor_setref);

    // 电机发送
    MotorSend(&(wheel_l_motor.base));

    /* 46 路：舵向/状态机/余弦系数 27 路 + 偏航观测组 8 路 + IMU 原始量 8 路
     * + 偏航闭环 3 路（通道数见 app_cfg.h 的 VOFA_CHANNELS）。
     *   ch1~12  左舵 12 项
     *   ch13~24 右舵 12 项
     *   ch25    底盘状态机当前状态（0=CAL 1=FAULT 2=SAFE 3=RUN）
     *   ch26/27 左右舵的余弦优化系数（本拍实际乘进轮速的那个值）
     *   ch28~35 偏航观测组（见下面"偏航观测组"那段）
     *   ch36~43 IMU 原始量（陀螺/加计机体系 + roll/pitch，用于诊断安装方向）
     *   ch44    偏航闭环实测 yaw rate（物理投影 + 低通）[rad/s]
     *   ch45    偏航闭环速率误差 w_cmd−yaw_rate（物理约定）[rad/s]
     *   ch46    偏航闭环施加到 w 的修正 [rad/s]（ch30 + ch46 = 实际解算用的 w）
     * 读法：每组里 ch+0/+1 重合 = 舵角跟得上；+2→0 且 +5 不饱和 = 位置环正常；
     *      +7→0 且 +10 是被控对象真正吃到的电流；+10 与 +11 长期不等查 Kt/方向。
     *      ch26/27：1 = 舵角已对齐（轮速不打折）、→0 = 轮面正垂直于目标方向、<0 = 轮面已偏过
     *      垂直（轮子在反向出力，翻转后半程的正常形态，见 ChassisRudderOut）。
     *      只做小幅修正时它一直贴着 1 属正常——cos 在小角度很平（差 10° 才降 1.5%）；
     *      要看到它明显压低轮速，得是大角度回转（如直行切横移）。
     * ⚠ CAL 态只有速度环在跑（+0 是扫描速度给定）；SAFE/FAULT 舵向失能，各路为 0。 */
    ChassisVofaRudder(1, &rudder_l_ctrl, rudder_l_motor_setref, rudder_l_data.speed, rudder_l_data.current);
    ChassisVofaRudder(13, &rudder_r_ctrl, rudder_r_motor_setref, rudder_r_data.speed, rudder_r_data.current);
    VofaSetChannel(25, (float)LibFsmTableCurrent(&chassis_fsm));
    /* 余弦系数只在 RUN 态由 HalfRudderSolve → ChassisRudderOut 更新；其余状态轮速本来就恒 0、
     * 系数无从谈起，这里显式清零，免得看到上一段运行态留下的旧数（与舵向失能时那 12 路清 0 同理）。 */
    if (state != chassis_state_run_e)
    {
        s_steer_align[LIB_CHASSIS_HALF_RUDDER_L] = 0.0f;
        s_steer_align[LIB_CHASSIS_HALF_RUDDER_R] = 0.0f;
        // 偏航闭环不在 RUN 态跑，显示量清零（状态本身在进 RUN 时 Init）
        s_yaw_loop.rate_lpf = 0.0f;
        s_yaw_loop.err = 0.0f;
        s_yaw_loop.corr = 0.0f;
        s_yaw_loop.corr_last = 0.0f;
    }
    VofaSetChannel(26, s_steer_align[LIB_CHASSIS_HALF_RUDDER_L]);
    VofaSetChannel(27, s_steer_align[LIB_CHASSIS_HALF_RUDDER_R]);

    /* ─────────── 偏航观测组（ch28~35）：把"车到底转没转"变成直接测量 ───────────
     *   ch28  IMU 世界系偏航角速度 yaw_rate (rad/s)  —— 车**真的**在转多快（陀螺实测）
     *   ch29  IMU 世界系航向角 euler.yaw (rad)       —— 上面积出来的航向（无绝对参考，只看变化量）
     *   ch30  指令 w（云台下发，rad/s）              —— 命令里到底带没带自旋
     *   ch31  正解 w（odom.z，rad/s）                —— 按"轮面不侧滑"从四轮实际状态算出的 w
     *   ch32/33 左/右驱动轮实际轮缘角速度 (rad/s)    —— 两轮出力的大小与方向（带符号）
     *   ch34/35 左/右驱动轮速度给定（轮缘 rad/s）    —— 与 ch32/33 同量纲，直接看跟随误差
     * 三种"w"并排就是这一组的用处：
     *   ① ch30 与 ch28 一起看，能一刀切开"命令的自旋"与"舵向过渡诱导的偏航"——
     *      ch30≈0 而 ch28≠0 就是后者（这正是"控制前后左右时 w 明显转动"要判的那一刀）；
     *   ② ch31 与 ch28 不符 ⇒ 两轮实际状态解释不了实测偏航，即"轮面不侧滑"前提不成立
     *      （侧滑/擦胎），ch32/33 再给出是哪一轮、往哪个方向出的力；
     *   ③ ch34/35 与 ch32/33 对照 ⇒ 把"命令本来就有差模"与"速度环没跟住"分开：给定边
     *      自己就有差模是逆解/换向解的事，给定无差模而实际有差模才轮到速度环/滑移。
     * ⚠ ch28/29 恒等于 0 = IMU 没就绪（valid=0：没播种 / SPI 或 DRDY 没通 / 看门狗掉线），
     *   不是"车没转"；真实陀螺静置时也是非零的（零偏 + 噪声）。
     * ⚠ ch28 的零偏未标定（s_bmi088_calib 全 0）：静止段读到的均值就是该补的零偏。
     * ⚠ ch28/29 乘了 CHASSIS_IMU_YAW_SIGN（安装方向校正，见 app_chassis_def.h）；这个符号
     *   必须用一次受控转动确认后才能用于闭环，单看日志里的相关性定不下来。
     * ⚠ ch30 在 SAFE 态被显式清零（那里的指令已作废，见 chassis_state_safe_e 分支）；
     *   ch31 只在 RUN 态由 HalfRudderSolve 更新，其余状态显式写 0，免得看到上一段的旧数。
     *   给定 ch34/35 在非 RUN 态本来就已被清 0（见各状态分支），无需额外处理。 */
    VofaSetChannel(28, CHASSIS_IMU_YAW_SIGN * imu_data.yaw_rate);
    VofaSetChannel(29, CHASSIS_IMU_YAW_SIGN * imu_data.euler.yaw);
    VofaSetChannel(30, gimbal2chassis_data.w);
    VofaSetChannel(31, (state == chassis_state_run_e) ? chassis2gimbal_data.w : 0.0f);
    VofaSetChannel(32, MOTOR_RADPS_TO_WHEEL_RADPS((float)MotorGetData(&(wheel_l_motor.base)).speed));
    VofaSetChannel(33, MOTOR_RADPS_TO_WHEEL_RADPS((float)MotorGetData(&(wheel_r_motor.base)).speed));
    VofaSetChannel(34, MOTOR_RADPS_TO_WHEEL_RADPS(wheel_l_motor_setref));
    VofaSetChannel(35, MOTOR_RADPS_TO_WHEEL_RADPS(wheel_r_motor_setref));

    /* ─────────── IMU 原始量（ch36~43）：诊断安装方向 ───────────
     *   ch36~38  机体系陀螺 x/y/z (rad/s，已按标定修正)
     *   ch39~41  机体系加速度 x/y/z (m/s²，已按标定修正)。按 BMI088 手册 §8.3：
     *            静止时**朝上的轴读 +g**。本车实测 (−0.12,−0.03,−9.70) ⇒ +z 轴朝下
     *            （IMU 装倒），KF 因此把 roll 收敛到 ≈180°；yaw_rate 的符号仍是对的。
     *   ch42/43  KF 的 roll/pitch (rad)
     * 为什么需要：ch28 的 yaw_rate 是**世界系欧拉角速率**，由 KF 的 roll/pitch 把机体系
     * 陀螺投影出来（`(sinφ·ωy+cosφ·ωz)/cosθ`）。实测发现本车 IMU 的 +z 轴朝下（见上），
     * KF 的 roll 因此收敛到 ≈180°、并在 ±180° 附近漂移：roll 处于 ±90° 之外时
     * ch28 ≈ −gz（与物理 yaw 同号，正确）；一旦 roll 漂过 ±90° 的分支，符号就会翻错
     * （实测约 6% 的帧）。这不是"装反"，改 CHASSIS_IMU_YAW_SIGN 修不了。原始量能直接看出：
     *   ① 静止段 acc 哪个分量 ≈ ±9.8 ⇒ 哪根机体系轴朝上（安装方向一目了然）；
     *   ② 自旋段 roll/pitch 是否在漂/跳、陀螺各轴是否只有一根在转；
     *   ③ 更稳的 yaw 修法：`yaw_rate = dot(gyro, u_rest)`，u_rest = 静止 acc 归一化
     *      （= 世界向上在机体系的方向，本车为 −z），完全绕开 KF 的 roll 分支；
     *      或先用固定的安装旋转把原始量旋到"平装"机体系再喂 KF。 */
    VofaSetChannel(36, imu_data.gyro[0]);
    VofaSetChannel(37, imu_data.gyro[1]);
    VofaSetChannel(38, imu_data.gyro[2]);
    VofaSetChannel(39, imu_data.acc[0]);
    VofaSetChannel(40, imu_data.acc[1]);
    VofaSetChannel(41, imu_data.acc[2]);
    VofaSetChannel(42, imu_data.euler.roll);
    VofaSetChannel(43, imu_data.euler.pitch);

    /* 偏航闭环观测量：ch44 实测角速度、ch45 速率误差、ch46 本拍修正。
     * 判读：稳态直行时 ch44→0、ch45→0；有恒定扰动时 ch44 会被压小但 ch46 非零
     * （积分项在出力，ch46 与其差 = K_P·ch45）；若 ch44 长期非零而 ch46 贴限幅，
     * 查方向/增益；若一停车就朝一个方向自转，查 ch44 的零偏和符号。 */
    VofaSetChannel(44, s_yaw_loop.rate_lpf);
    VofaSetChannel(45, s_yaw_loop.err);
    VofaSetChannel(46, s_yaw_loop.corr);

    /* 遥测隔拍发（250Hz）：46 路每帧 192B，500Hz 会超出 921600baud 的带宽（见 app_cfg.h）。
     * 通道值每拍都刷新，这里只决定哪一拍把当前快照推出去。 */
    if (++s_vofa_send_div >= CHASSIS_VOFA_SEND_DIV)
    {
        s_vofa_send_div = 0;
        VofaSend();
    }

    CommSend(&gimbal_comm,
             (uint8_t *)&chassis2gimbal_data); // 云台通信：每周期发一帧（接收已由 CAN 中断写入 gimbal_rx_data）
}
