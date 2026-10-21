/**
 * @file app_chassis_force_def.h
 * @brief 力控底盘 app 的**宏 + 类型**
 *
 * 实体定义（常量、运行状态、通信实例）全部在 app_chassis_force.c；
 * 本文件只保留：
 *   - 控制/整定/方案相关的宏（机械/安装参数在 robot_def.h）；
 *   - 跨函数共享的 plain struct / enum 类型。
 *
 * ⚠ 电机/IMU/通信的驱动实例仍在 app_chassis_force.c：
 *   Drvs*_INSTANCE_DEF / BMI088_KALMAN_INSTANCE_DEF 展开的是 static 对象，
 *   头文件里无法 extern 给别的编译单元用，只能用在使用它的那个 .c 里。
 */

#ifndef __APP_CHASSIS_FORCE_DEF_H
#define __APP_CHASSIS_FORCE_DEF_H

#include "robot_def.h"               // 机械/传动参数（减速比、电机方向、速度上限）
#include "lib_math.h"                // vector3_t / M_PI
#include "lib_chassis_half_rudder.h" // LIB_CHASSIS_HALF_RUDDER_NUM / 轮序

/*============================================
 *              方案开关
 *============================================*/
/* 反馈方案：
 *   0 = 独立：IMU 给 yaw rate，轮速正解给 vx/vy（默认）
 *   1 = 融合：6 态 ESKF（vx/vy、加速度零偏、航向角、yaw rate 零偏）
 *       IMU 加速度预测 + 轮速正解速度量测 + ZUPT/ZARU，见 app_chassis_eskf.c */
#define CHASSIS_FEEDBACK_SCHEME 0
#if CHASSIS_FEEDBACK_SCHEME == 1
#error "不要用eskf，当前有bug，会失控"
#endif

/* 闭环方案：
 *   0 = PID 反馈 + 可配置前馈（当前实现；前馈系数默认全 0）
 *   1 = 反馈 + RLS/功率约束的整体规划（预留，尚未实现，编译会报错提醒） */
#define CHASSIS_CLOSED_LOOP_SCHEME 0

/* 输出方案（±180° 等价解怎么选）：
 *   0 = 直接最短路径：φ / φ+180° 里选离当前舵角更近的，平手保持上一拍分支
 *   1 = 4 组合代价函数：角度误差 + 力矩误差 + 两舵反向惩罚，带切换迟滞 */
#define CHASSIS_OUTPUT_SCHEME 1

/*============================================
 *              超级电容（YQ2027）
 *============================================*/
/* 只做接入：周期下发使能命令、每拍取反馈存进 g_super_cap_data，
 * 不做功率限制/能量管理。命令内容参考 half_rudder_2027 的 chassis.c：
 * 非比赛模式下固定发 70W 裁判限制；反馈固定为 0x052（1/64 W 精度）。 */
#define CHASSIS_SUPER_CAP_SEND_PERIOD_MS 20
#define CHASSIS_SUPER_CAP_REFEREE_POWER_LIMIT_W 70

/*============================================
 *              校准/整定参数
 *============================================*/
/* 舵向位置环 WRAP 窗口 = 舵轮整圈 360°（电机侧 ±π×减速比，跨度 16π = 舵轮 2π）。
 * 换向决策会在 φ 与 φ+180° 两个等价解之间选，两者都落在这个窗口内：
 * 舵轮可以连续绕整圈，不会被 ±180° 的"机械端点"挡住。
 * 窗口端点取 ±180° 是为了让 0（车体正前方）落在窗口正中，跨窗次数最少。 */
#define CHASSIS_STEER_WRAP_MIN (-M_PI * CHASSIS_STEER_GEAR_RATIO)
#define CHASSIS_STEER_WRAP_MAX (M_PI * CHASSIS_STEER_GEAR_RATIO)

/* 舵轮侧角度（°）→ 电机侧 rad：位置反馈在 drvlib_motor 里是"电机侧 rad"，
 * 机械偏置按舵轮角度标定，换算到电机侧要乘减速比。 */
#define STEER_WHEEL_DEG_TO_MOTOR_RAD(deg) ((deg) * (M_PI / 180.0f) * CHASSIS_STEER_GEAR_RATIO)

/* 上电校准（光电门找零点）：两舵轮以恒定速度扫描，捕捉下降沿。
 * 单次超时只重试（漏采一圈很常见），连续 CHASSIS_CAL_MAX_RETRIES 次才判故障。
 * 5000 拍 × 2ms = 10s。 */
#define CHASSIS_STEER_ZERO_SCAN_SPEED (24.0f) // 扫描速度（电机侧 rad/s）
#define CHASSIS_CAL_TIMEOUT_TICKS (5000u)
#define CHASSIS_CAL_MAX_RETRIES (3u)

/* 校准后方向验证（云台使能时生效）：
 * 两驱动轮同时给 app 正方向的**小力矩** (Nm)，用来核对轮向与舵向零点：
 *   两轮同号且 DIR 正确 ⇒ 车沿舵轮正前方直行；
 *   原地打转 ⇒ 某一轮 CHASSIS_WHEEL_x_DIR 反了；
 *   后退/斜行 ⇒ 舵向零点偏置（RUDDER_ZERO_OFFSET_x_DEG）或舵向方向有问题。
 * 这里给的是 app 正方向，下发时仍会乘 CHASSIS_WHEEL_x_DIR，走完整符号链。 */
#define CHASSIS_WHEEL_CAL_TORQUE (0.5f)

/* 舵向环路 PID：数值原样取自旧 app（app_chassis/app_chassis_def.h），
 * 包括角度环的"无积分、D 不加滤波"。力控阶段未接入前只有校准速度环真正生效。 */
#define CHASSIS_RUDDER_PID_SPEED_KP (0.10f)
#define CHASSIS_RUDDER_PID_SPEED_KI (0.20f)
#define CHASSIS_RUDDER_PID_SPEED_KD (0.002f)
#define CHASSIS_RUDDER_PID_SPEED_INTEGRAL_LIMIT (2.5f)
#define CHASSIS_RUDDER_PID_SPEED_COEF_A (20.0f)
#define CHASSIS_RUDDER_PID_SPEED_COEF_B (20.0f)
#define CHASSIS_RUDDER_PID_SPEED_D_LPF_RC (0.004f)
#define CHASSIS_RUDDER_PID_SPEED_OUT_LPF_RC (0.0f)
#define CHASSIS_RUDDER_PID_SPEED_DEADBAND (0.0f)
#define CHASSIS_RUDDER_PID_SPEED_ERR_NORM_RANGE (0.0f)
#define CHASSIS_RUDDER_PID_SPEED_OUT_MAX (0.0f)
#define CHASSIS_RUDDER_PID_SPEED_OUT_MIN (0.0f)

#define CHASSIS_RUDDER_PID_ANGLE_KP (50.0f)
#define CHASSIS_RUDDER_PID_ANGLE_KI (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_KD (0.3f)
#define CHASSIS_RUDDER_PID_ANGLE_INTEGRAL_LIMIT (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_COEF_A (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_COEF_B (0.0f)
/* 位置环微分滤波：舵角反馈带编码器量化噪声，不滤会让 D 项放大噪声。
 * 只在 PID 打开 DERIVATIVE_ON_MEAS + DERIVATIVE_FILTER 时生效，见下面的 config_mask。 */
#define CHASSIS_RUDDER_PID_ANGLE_D_LPF_RC (0.008f)
#define CHASSIS_RUDDER_PID_ANGLE_OUT_LPF_RC (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_DEADBAND (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_ERR_NORM_RANGE (0.0f) /* WRAP 下 Config 自动填 */
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MAX (CHASSIS_STEER_VMAX_MOTOR)
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MIN (-CHASSIS_STEER_VMAX_MOTOR)

/*============================================
 *              IMU / 杆臂补偿参数
 *============================================*/
#ifndef CHASSIS_IMU_ALPHA_LPF_RC
#define CHASSIS_IMU_ALPHA_LPF_RC 0.02f // 角加速度低通时间常数 (s)，≈20ms
#endif
#ifndef CHASSIS_IMU_ALPHA_LIMIT
#define CHASSIS_IMU_ALPHA_LIMIT 100.0f // 角加速度限幅 (rad/s²)，防差分尖峰
#endif

/*============================================
 *              ESKF 融合参数（方案 1）
 *============================================*/
/* 6 态误差 [vx, vy, bax, bay, theta, bg]，量测 [vx, vy, yaw_rate - bg]。
 * 全部给 #ifndef 守卫，便于局部覆盖（同 CHASSIS_IMU_ALPHA_* 的风格）。 */
#ifndef CHASSIS_ESKF_G
#define CHASSIS_ESKF_G 9.80665f // 重力加速度 (m/s²)
#endif
#ifndef CHASSIS_ESKF_DT_MAX_S
#define CHASSIS_ESKF_DT_MAX_S 0.010f // 预测步长上限 (s)，防任务抖动导致大积分
#endif

/* 初始协方差 P0（误差态初值恒 0，这里只给不确定度） */
#ifndef CHASSIS_ESKF_P0_VEL
#define CHASSIS_ESKF_P0_VEL 0.25f // (m/s)²
#endif
#ifndef CHASSIS_ESKF_P0_ACC_BIAS
#define CHASSIS_ESKF_P0_ACC_BIAS 0.04f // (m/s²)²
#endif
#ifndef CHASSIS_ESKF_P0_YAW
#define CHASSIS_ESKF_P0_YAW 0.01f // rad²
#endif
#ifndef CHASSIS_ESKF_P0_GYRO_BIAS
#define CHASSIS_ESKF_P0_GYRO_BIAS 1.0e-6f // (rad/s)²
#endif

/* 过程噪声密度：Q = diag(q·dt) */
#ifndef CHASSIS_ESKF_Q_VEL
#define CHASSIS_ESKF_Q_VEL 0.05f // m²/s³
#endif
#ifndef CHASSIS_ESKF_Q_ACC_BIAS
#define CHASSIS_ESKF_Q_ACC_BIAS 1.0e-6f // (m/s²)²/s
#endif
#ifndef CHASSIS_ESKF_Q_YAW
#define CHASSIS_ESKF_Q_YAW 1.0e-6f // rad²/s
#endif
#ifndef CHASSIS_ESKF_Q_GYRO_BIAS
#define CHASSIS_ESKF_Q_GYRO_BIAS 1.0e-9f // (rad/s)²/s
#endif
#ifndef CHASSIS_ESKF_Q_NO_IMU_SCALE
#define CHASSIS_ESKF_Q_NO_IMU_SCALE 100.0f // IMU 无效时速度/航向 Q 放大倍数
#endif

/* 量测噪声方差 */
#ifndef CHASSIS_ESKF_R_WHEEL
#define CHASSIS_ESKF_R_WHEEL 2.5e-3f // (m/s)²
#endif
#ifndef CHASSIS_ESKF_R_WHEEL_W
#define CHASSIS_ESKF_R_WHEEL_W 4.0e-3f // (rad/s)²，轮速正解 yaw rate
#endif
#ifndef CHASSIS_ESKF_R_ZUPT
#define CHASSIS_ESKF_R_ZUPT 1.0e-4f // (m/s)²
#endif
#ifndef CHASSIS_ESKF_R_ZARU
#define CHASSIS_ESKF_R_ZARU 1.0e-6f // (rad/s)²，静止零角速度
#endif

/* 新息门控（卡方 2 自由度） */
#ifndef CHASSIS_ESKF_NIS_INFLATE
#define CHASSIS_ESKF_NIS_INFLATE 9.21f // 99%
#endif
#ifndef CHASSIS_ESKF_NIS_REJECT
#define CHASSIS_ESKF_NIS_REJECT 13.82f // 99.9%
#endif
#ifndef CHASSIS_ESKF_R_INFLATE
#define CHASSIS_ESKF_R_INFLATE 10.0f // 可疑帧的 R 放大倍数
#endif

/* 打滑判据：轮速正解 yaw rate 与 IMU 世界 yaw rate 的差 (rad/s) */
#ifndef CHASSIS_ESKF_SLIP_W_DIFF
#define CHASSIS_ESKF_SLIP_W_DIFF 0.5f
#endif
#ifndef CHASSIS_ESKF_SLIP_W_REJECT
#define CHASSIS_ESKF_SLIP_W_REJECT 1.0f
#endif

/* 数据新鲜度 */
#ifndef CHASSIS_ESKF_IMU_TIMEOUT_US
#define CHASSIS_ESKF_IMU_TIMEOUT_US 20000u // 20ms
#endif
#ifndef CHASSIS_ESKF_WHEEL_TIMEOUT_US
#define CHASSIS_ESKF_WHEEL_TIMEOUT_US 20000u // 20ms
#endif

/* ZUPT/ZARU 静止判据 */
#ifndef CHASSIS_ESKF_ZUPT_WHEEL_EPS
#define CHASSIS_ESKF_ZUPT_WHEEL_EPS 0.03f // 轮缘速度 (m/s)
#endif
#ifndef CHASSIS_ESKF_ZUPT_GYRO_EPS
#define CHASSIS_ESKF_ZUPT_GYRO_EPS 0.05f // |yaw_rate - bg| (rad/s)
#endif
#ifndef CHASSIS_ESKF_ZUPT_ACC_EPS
#define CHASSIS_ESKF_ZUPT_ACC_EPS 0.30f // |acc_lin.xy| (m/s²)
#endif
#ifndef CHASSIS_ESKF_ZUPT_ENTER_TICKS
#define CHASSIS_ESKF_ZUPT_ENTER_TICKS 50u // 500Hz 下 100ms
#endif

/* velocity_valid 保持窗口：最后一次被接受的量测之后 */
#ifndef CHASSIS_ESKF_VEL_HOLD_TICKS
#define CHASSIS_ESKF_VEL_HOLD_TICKS 50u
#endif
#ifndef CHASSIS_ESKF_V_MAX
#define CHASSIS_ESKF_V_MAX 5.0f // 输出速度兜底钳位 (m/s)
#endif

/*============================================
 *              类型
 *============================================*/
/* IMU 量转到**底盘系**后的结果（旋转用安装矩阵，平移量见 pos） */
typedef struct
{
    vector3_t gyro;    // 底盘系角速度 (rad/s)
    vector3_t acc;     // 底盘系比力 (m/s²)，已做杆臂补偿到 底盘中心
    vector3_t acc_lin; // 底盘系线性加速度 (m/s²)：acc 按姿态扣除重力（方案 1）
    euler_t euler;     // 驱动原始欧拉角 (rad)：roll/pitch/yaw（IMU 自身轴向）
    vector3_t pos;     // IMU 安装点在底盘系的位置 (m)，固定
    float yaw_rate;    // 世界系绕竖直轴角速度 (rad/s)（与安装方向无关）
    uint8_t valid;     // 1 = 本拍姿态可用（已播种/有数据）
    uint8_t fresh;     // 1 = 本拍消费到新的陀螺样本
} AppChassisImu_s;

/* 驱动轮反馈（已按 CHASSIS_WHEEL_x_DIR 换算到 app 正方向）。
 * 力控本身不需要 drvlib_motor：轮向只做"力矩符号 + 反馈符号"，没有速度闭环。 */
typedef enum : uint8_t
{
    APP_WHEEL_L = 0,
    APP_WHEEL_R,
    APP_WHEEL_NUM,
} AppWheel_e;

typedef struct
{
    float speed;           // 轮速 (rad/s)，app 正方向（沿当前舵角推动车体为正）
    float torque;          // 电机轴力矩 (N·m)= iq×Kt，app 正方向（换向代价用）
    float current;         // 电流 (A)
    int8_t temperature;    // 温度 (℃)
    uint8_t error;         // 错误码（LK 为 echo，DJI 为 error）
    uint64_t timestamp_us; // CAN 帧到达时间戳 (us)，0 = 尚未收到（方案 1 去重用）
} AppWheelFeedback_s;

/*============================================
 *              顶层状态机
 *============================================*/
/* 状态 = 底盘"现在能干的事"，不是云台下发的档位：
 *   CAL   上电校准：两舵轮扫描找光电门零点（此时不理会云台在线与否）
 *   FAULT 校准失败（连续超时）：失能 + 给定清零，终态（需复位重启再标）
 *   SAFE  已标定但不可动：云台掉线 或 云台下发 stop —— 失能 + 给定清零 + 作废指令
 *   RUN   已标定且可动：力控/解算入口（正解 → 广义力 → 分配 → 换向决策 → 下发） */
typedef enum : uint8_t
{
    CHASSIS_STATE_CAL = 0, // 上电校准中
    CHASSIS_STATE_FAULT,   // 校准超时（故障）
    CHASSIS_STATE_SAFE,    // 未使能 / 云台掉线
    CHASSIS_STATE_RUN,     // 使能运行
    CHASSIS_STATE_COUNT,   // 状态值域上界（= 动作表项数）
} ChassisState_e;

typedef enum : uint8_t
{
    CHASSIS_EV_UPDATE = 0, // 每拍更新（把下面四个条件重新汇总）
} ChassisEvent_e;

typedef struct
{
    uint8_t cal_done;    // 两舵轮均已标定
    uint8_t cal_fault;   // 校准故障（连续超时到上限，终态）
    uint8_t cal_retries; // 已发生的单次超时次数（未标定且计时超限就 +1 并重新计时）
    uint8_t online;      // 云台链路在线
    uint8_t stopped;     // 云台下发模式为 stop
    uint32_t cal_ticks;  // 本轮校准已持续拍数（单次超时判据）
    uint8_t key_last;    // 上次派发时的条件打包（变化才派发）
} ChassisFsmCtx_s;

/*============================================
 *              力控：速度误差 → 广义力
 *============================================*/
/* 输入：云台速度指令 (vx, vy, w) 与实测速度。
 * 输出：底盘广义力 W = (fx [N], fy [N], mw [N·m])，车体系 x+前 / y+左 / w+逆时针。
 *   反馈：三个 lib_pid 实例（参数见下方 CHASSIS_FORCE_* 宏）
 *   前馈：摩擦（沿指令速度方向）+ 惯性/惯量（指令加速度）+ 重力/坡道分量。
 *   三组前馈系数当前全部为 0，结构先留好，标定后直接填数。 */
typedef struct
{
    /* 前馈：摩擦（Coulomb，沿指令速度方向；指令为 0 时不施加） */
    float friction_x; // N
    float friction_y; // N
    float friction_w; // N·m

    /* 前馈：惯性 / 转动惯量（用指令差分加速度） */
    float mass;      // kg
    float inertia_z; // kg·m²

    /* 前馈：重力/坡道在车体系的等效力（平面默认 0） */
    float gravity_x; // N
    float gravity_y; // N
} ChassisForceCfg_s;

/* 实体定义在 app_chassis_force.c。 */

/*============================================
 *              舵向：速度逆解 + ±180° 分支
 *============================================*/
/* 当前架构（2026-10-19）：
 *   1. 舵角只由 (vx,vy,w) 的速度逆解给出（LibChassisInverse）；
 *   2. 力分解只在上述两个轮向上投影，输出驱动行力矩，
 *      **不使用侧向力、不用力方向反过来改舵角**。
 *
 * 每组舵轮的驱动方向仍有 ±180° 两个等价解：
 *     (φ, +τ)  或  (φ+180°, −τ)
 * 两解产生的车体合力完全相同，只是舵向要多转多少、轮向力矩要变多少不同。
 * 分支选择由 CHASSIS_OUTPUT_SCHEME 切换：
 *   - 0：选**离当前舵角更近**的解（平手保持上一拍分支）；
 *   - 1：对 4 种反向组合做代价比较，角度/力矩/两舵反向都计入。
 *
 * 下面这组代价系数在 CHASSIS_OUTPUT_SCHEME=1 时生效；
 * CHASSIS_COST_ANGLE_DEADBAND 在两种方案里都用于分支平手迟滞。 */
#define CHASSIS_COST_RUDDER (0.35f)           // 舵向代价系数 (s/rad)
#define CHASSIS_COST_WHEEL (0.20f)            // 轮向力矩误差代价系数 (s/(N·m))
#define CHASSIS_COST_DIFF (0.15f)             // 两舵轮转向相反的额外代价 (s)
#define CHASSIS_COST_SWITCH_MARGIN (0.15f)    // 组合切换迟滞 (s)
#define CHASSIS_COST_TORQUE_LOCK (0.20f)      // 两轮均无力向时的力矩锁定阈值 (N·m)
#define CHASSIS_COST_ANGLE_DEADBAND (0.0175f) // ±180° 分支平手迟滞/角度死区 (rad，舵轮侧 ≈1°)

/* 驱动轮出力比例 wheel_align = max(cos(目标舵角 − 实际舵角), 0)。
 * 力分配按**目标**舵向算轮缘力矩；舵向位置环追上之前，实际轮向与目标有偏差，
 * 按误差余弦降额，避免驱动轮在错误方向上顶满力（与旧代码的余弦优化一致）。
 * 误差 ≥90° 时钳 0：换向/大角度纠偏过程中驱动轮不出力，不反向拖拽。 */

/* 反馈环参数（lib_pid）。
 * 平动：P + 死区 + 输出低通（积分保持 0：轮速正解的偏置主要来自侧滑，
 *       积分会把假平移也积进去）。
 * yaw：PI。纯 P 时指令 w 与实际 w 始终差一个由偏航阻力决定的稳态误差
 *      （实测 w ≈ 0.186·vx + 0.053 rad/s，P 只能把它压小、压不到 0），
 *      故加积分消除稳态漂移。Ki 与调用周期无关（i_term = Ki·e·dt）。
 *
 * ⚠ yaw 不能开 PID_ENABLE_DEADBAND：lib_pid 的死区分支会把 i_out 清零并
 *   直接返回 0，积分永远攒不起来，死区变成"松手不管"。w 的噪声只有
 *   ±0.005 rad/s，P 项贡献 ±0.01 N·m，不加死区即可。
 * ⚠ 平动的死区同样会在死区内把前馈一起清零；当前三组前馈系数为 0，
 *   标定摩擦/重力前馈时须重新评估（可改为只对误差扣减的死区）。 */
#define CHASSIS_FORCE_VX_KP (50.0f)       // N/(m/s)
#define CHASSIS_FORCE_VY_KP (50.0f)       // N/(m/s)
#define CHASSIS_FORCE_XY_DEADBAND (0.02f) // m/s，平移速度误差死区
#define CHASSIS_FORCE_XY_LPF_RC (0.020f)  // s，输出一阶低通

#define CHASSIS_FORCE_W_KP (6.0f)             // N·m/(rad/s)
#define CHASSIS_FORCE_W_KI (12.0f)            // N·m/(rad/s)/s（每秒累积，与调用频率无关）
#define CHASSIS_FORCE_W_INTEGRAL_LIMIT (0.6f) // N·m，积分限幅（实测偏航阻力约 0.35 N·m）
/* mw 输出低通：30 → 70 ms。
 * 纯 vy 日志实测 yaw 环在 ~4 Hz 自持（舵 ±90° 时 ∂φ/∂Mw ≈ 20~35°/N·m），
 * 30 ms 对 4 Hz 只衰减到 0.80，几乎没作用；70 ms 衰减到 ~0.50。
 * 注意一阶低通在 4 Hz 会再引入 ~60° 相位滞后，可能把极限环频率压低而非消除，
 * 本值先作为对照实验，日志确认后再决定是否换成更高阶或直接限幅。 */
#define CHASSIS_FORCE_W_LPF_RC (0.070f) // s，输出一阶低通

/**
 * @brief 舵向/轮向给定（每拍在 RUN 态更新）
 * @note angle_target 来自速度逆解（LibChassisInverse）+ 最近的 ±180° 分支：
 *       舵轮侧连续角 (rad)，下发前 ×CHASSIS_STEER_GEAR_RATIO。
 * @note wheel_torque 是力分配沿**目标舵向**的带符号轮缘力矩 (N·m)，
 *       下发电机前 ÷CHASSIS_DRIVE_REDUCTION 再乘 CHASSIS_WHEEL_x_DIR。
 * @note reverse[i] = 1 表示该轮选了 φ+180° 的反向等价解（驱动轮反转）。
 * @note combo 位定义：bit0 = 左舵反向，bit1 = 右舵反向（0..3）。
 * @note cost[] 是 4 组合代价（CHASSIS_OUTPUT_SCHEME=1 时有效，=0 时恒 0）。
 * @note locked = 1 表示两轮都无力向且力矩很小，保持当前舵角、不出力
 *       （仅 CHASSIS_OUTPUT_SCHEME=1 使用）。
 * @note RUN 态用这些量驱动舵向与轮向（舵向 100Hz 遥测见 app_cfg.h 通道表）。 */
typedef struct
{
    float angle_target[LIB_CHASSIS_HALF_RUDDER_NUM]; // 目标舵角 (rad，舵轮侧，速度逆解)
    float wheel_torque[LIB_CHASSIS_HALF_RUDDER_NUM]; // 沿目标舵向的带符号轮缘力矩 (N·m)
    float angle_error[LIB_CHASSIS_HALF_RUDDER_NUM];  // 本拍目标与当前舵角之差 (rad)
    float cost[4];                                   // 4 组合代价（方案 1）
    float wheel_align[LIB_CHASSIS_HALF_RUDDER_NUM];  // 余弦优化系数 max(cos(目标−实际舵角), 0)
    uint8_t reverse[LIB_CHASSIS_HALF_RUDDER_NUM];    // 1 = 选反向解
    uint8_t combo;                                   // 反向位组合（0..3）
    uint8_t valid;                                   // 1 = 至少完成过一次决策
    uint8_t locked;                                  // 1 = 无力向且小力矩锁定（方案 1）
} ChassisCostSel_s;

/* 以上类型的实体均定义在 app_chassis_force.c。 */

#endif // !__APP_CHASSIS_FORCE_DEF_H
