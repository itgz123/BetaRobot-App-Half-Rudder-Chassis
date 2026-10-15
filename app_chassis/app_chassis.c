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
 * 等价解里挑哪个、驱动要不要反转，是**离散决策**、需要迟滞才不抖，故落在本层
 * （见 ChassisRudderSteer）；内核那边不做选择，也就不需要记任何东西。
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

/*============================ 舵轮换向（最近 180° 等价解 + 迟滞） ============================*/
/* 内核只给"期望轮向 + 轮速大小"（纯几何、无状态）；在 φ_des 与 φ_des+180° 两个等价解里
 * 挑哪个、驱动轮要不要反转、静止时保持哪个朝向，都是**离散决策**，放本层。
 * 判据是"|角差| 是否超过 90°"，而这条线恰好是两种解**等贵**的地方——轮面垂直于期望速度
 * 方向时（对称工况、被反推、舵角反馈噪声），取舍会在噪声下逐拍翻转：目标舵角每拍跳 ±180°、
 * 驱动符号跟着翻，位置环（kp=50）被反复踢。故判据必须带迟滞，而迟滞要跨帧记忆。
 * ⚠ 这三项与 s_chassis 一样由控制任务独占（无锁），不得在别的上下文读改写。 */
static float s_steer_target_last[LIB_CHASSIS_HALF_RUDDER_NUM]; // 上次下发的目标舵角（舵轮侧连续角）
static int8_t s_steer_reverse[LIB_CHASSIS_HALF_RUDDER_NUM];    // 迟滞状态：上一拍是否采用反向解
static uint8_t s_steer_seeded;                                 // bit i = 轮 i 的上面两项已初始化

/**
 * @brief 舵轮换向决策：内核的"期望轮向 + 轮速大小" → 目标舵角 + 带符号轮速
 * @param idx     轮序号（< LIB_CHASSIS_HALF_RUDDER_NUM，取本层的迟滞状态）
 * @param phi_des 内核给的期望轮向 [rad]（舵轮侧；本拍 speed == 0 时无意义）
 * @param phi_cur 当前舵角 [rad]（舵轮侧实测，按连续角用）
 * @param speed   内核给的轮缘速度**大小** [rad/s]（恒 ≥ 0；0 表示内核判为"不需转向"）
 * @param reverse 输出：1=选了反向解、驱动轮需反转；0=不反转。可为 NULL
 * @return 目标舵角 [rad]（舵轮侧连续角）
 * @note speed == 0（内核按 LIB_CHASSIS_STEER_SPEED_EPS 判）：保持**上次目标舵角**、驱动停转。
 *       不拿实测舵角当目标：它是带噪声的归一化读数，拿它当目标等于把噪声直接喂进位置环。
 * @note speed > 0：判据 |wrap(phi_des − phi_cur)| 是否超过 90°，阈值按上一拍的取舍分两档
 *       （± CHASSIS_STEER_HYST_RAD，见 app_chassis_def.h），带宽内沿用上一拍的选择；
 *       选反向解时角差再补 ±180°（即"舵朝 φ_des+180°、轮子反转"这个等价解）。
 * @note 首拍锚到当前实测舵角（别一上来就把舵甩到别处），之后由本层记忆接管。
 */
static float ChassisRudderSteer(uint8_t idx, float phi_des, float phi_cur, float speed, int8_t *reverse)
{
    uint8_t bit = (uint8_t)(1u << idx);
    float e, th;
    int8_t rev;

    if ((s_steer_seeded & bit) == 0u)
    {
        s_steer_target_last[idx] = phi_cur;
        s_steer_reverse[idx] = 0;
        s_steer_seeded |= bit;
    }

    if (speed <= 0.0f)
    {
        if (reverse != NULL)
            *reverse = 0;
        return s_steer_target_last[idx];
    }

    e = Lib_Math_WrapAngleNegPIToPI(phi_des - phi_cur); // (-π, π]
    th = (s_steer_reverse[idx] != 0) ? (M_PI_2 - CHASSIS_STEER_HYST_RAD) : (M_PI_2 + CHASSIS_STEER_HYST_RAD);
    rev = (FABS(e) > th) ? 1 : 0;

    if (rev != 0)
        e -= (e > 0.0f) ? (float)M_PI : -(float)M_PI;

    s_steer_reverse[idx] = rev;
    s_steer_target_last[idx] = phi_cur + e;
    if (reverse != NULL)
        *reverse = rev;
    return s_steer_target_last[idx];
}

/**
 * @brief 舵轮一个轮：内核输出 → 两个电机侧给定（舵角 + 驱动速度）
 * @param idx          轮序号
 * @param ref          内核逆解输出（steer_target = 期望轮向，wheel_speed = 轮速大小）
 * @param st           内核状态（取实测舵角）
 * @param steer_setref 输出：舵向电机位置给定 [电机侧 rad]
 * @param wheel_setref 输出：驱动电机速度给定 [电机侧 rad/s]（内核只给大小，符号在这里定）
 */
static void ChassisRudderOut(uint8_t idx, const LibChassisRef_s *ref, const LibChassisState_s *st, float *steer_setref, float *wheel_setref)
{
    float speed_wheel = ref->wheel_speed[idx]; // 轮缘速度大小（≥ 0），反向解时下面取负
    int8_t reverse = 0;

    *steer_setref = ChassisRudderSteer(idx, ref->steer_target[idx], st->steer_angle[idx], speed_wheel, &reverse) * STEER_GEAR_RATIO;
    *wheel_setref = WHEEL_RADPS_TO_MOTOR_RADPS((reverse != 0) ? -speed_wheel : speed_wheel);
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
    LibChassisRef_s ref;
    vector3_t twist = {.x = -cmd->vx, .y = -cmd->vy, .z = cmd->w};
    LibChassisInverse(&s_chassis, twist, &ref);
    ChassisRudderOut(LIB_CHASSIS_HALF_RUDDER_L, &ref, &st, &rudder_l_motor_setref, &wheel_l_motor_setref);
    ChassisRudderOut(LIB_CHASSIS_HALF_RUDDER_R, &ref, &st, &rudder_r_motor_setref, &wheel_r_motor_setref);

    // 正解 → 速度反馈（随 CommSend 回传云台）
    // 用"实际舵角 + 实际驱动速度"，故轮子处于反向解（朝 φ、反转）时结果同样正确。
    // 两组回转中心重合时 JᵀJ 奇异，内核返回 -1 且输出已清零（车体方位不可分辨）。
    vector3_t odom;
    if (LibChassisForward(&s_chassis, &st, &odom) != 0)
        odom = (vector3_t){0};
    chassis2gimbal_data.vx = odom.x;
    chassis2gimbal_data.vy = odom.y;
    chassis2gimbal_data.w = odom.z;
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
        ChassisMotorsEnable();
        HalfRudderSolve(&gimbal2chassis_data); // 逆解出四个给定 + 正解回填速度反馈
        /* 舵向：位置+速度环算出下发力矩交给 drvs（Kt=1，A 与 Nm 数值相同）。
         * ⚠ 顺序：HalfRudderSolve 读的是上一拍 SetRef 刷新的 data.position（本拍反馈在
         *    SetRef 里才处理）——即几何解算的舵角滞后一拍(2ms)。舵向变化慢，可忽略；
         *    想要零滞后就得把"处理反馈"和"跑环"拆开，drvlib 没提供该接口。 */
        rudder_l_cmd = DrvlibMotorSetRef(&rudder_l_ctrl, rudder_l_motor_setref, &rudder_l_fb, dt_s);
        rudder_r_cmd = DrvlibMotorSetRef(&rudder_r_ctrl, rudder_r_motor_setref, &rudder_r_fb, dt_s);
        break;

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

    /* 25 路全部给两个舵向控制核，看位置/速度双环的控制效果（顺序见 ChassisVofaRudder）。
     *   ch1~12  左舵 12 项
     *   ch13~24 右舵 12 项
     *   ch25    底盘状态机当前状态（0=CAL 1=FAULT 2=SAFE 3=RUN）
     * 读法：每组里 ch+0/+1 重合 = 舵角跟得上；+2→0 且 +5 不饱和 = 位置环正常；
     *      +7→0 且 +10 是被控对象真正吃到的电流；+10 与 +11 长期不等查 Kt/方向。
     * ⚠ CAL 态只有速度环在跑（+0 是扫描速度给定）；SAFE/FAULT 舵向失能，各路为 0。 */
    ChassisVofaRudder(1, &rudder_l_ctrl, rudder_l_motor_setref, rudder_l_data.speed, rudder_l_data.current);
    ChassisVofaRudder(13, &rudder_r_ctrl, rudder_r_motor_setref, rudder_r_data.speed, rudder_r_data.current);
    VofaSetChannel(25, (float)LibFsmTableCurrent(&chassis_fsm));
    VofaSend();

    CommSend(&gimbal_comm,
             (uint8_t *)&chassis2gimbal_data); // 云台通信：每周期发一帧（接收已由 CAN 中断写入 gimbal_rx_data）
}
