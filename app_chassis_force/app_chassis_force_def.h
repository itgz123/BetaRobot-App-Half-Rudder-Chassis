/**
 * @file app_chassis_force_def.h
 * @brief 力控底盘 app 的**类型声明 + 全局变量声明**
 *
 * app_chassis_force.c 只放业务逻辑（状态机、控制律、解算接线）；
 * 本文件与 app_chassis_force_def.c 集中放跨函数/跨文件共享的类型和状态：
 *   - 类型：IMU 观测结果 AppChassisImu_s；
 *   - 常量：IMU→底盘安装变换、BMI088 标定值；
 *   - 运行状态：底盘系 IMU 观测、杆臂补偿中间量、云台通信收发数据。
 *
 * ⚠ 电机/IMU/通信的**驱动实例**仍留在 app_chassis_force.c：
 *   Drvs*_INSTANCE_DEF / BMI088_KALMAN_INSTANCE_DEF 展开的是 static 对象，
 *   头文件里无法 extern 给别的编译单元用，只能用在使用它的那个 .c 里。
 */

#ifndef __APP_CHASSIS_FORCE_DEF_H
#define __APP_CHASSIS_FORCE_DEF_H

#include "app.h"       // 板间线协议类型 gimbal2chassis_data_t / chassis2gimbal_data_t / robot_mode
#include "robot_def.h" // IMU 安装方向与位置宏
#include "lib_math.h"
#include "lib_fsm_table.h"
#include "lib_chassis.h" // LibChassisTorque_s（力分配输出）
#include "drvlib_bmi088_kalman.h" // BMI088_Calib_s
#include "drv_comm.h"             // CommInstance

/*============================================
 *              电机方向（2026-10-18 实测）
 *============================================
 * 测试方法：四个电机分别发正力矩，观察反馈符号。
 * 结果：四个电机正力矩方向一致；反馈只有 wheel_l 为负（左右驱动轮镜像安装）。
 * 约定（与旧 app 相同，feedback/motor 必须成对取反，只反一个会变正反馈）：
 *   舵向 +：舵轮俯视逆时针（与车体系 +w 同向）
 *   轮向 +：沿当前舵角方向推动车体（不是"俯视逆时针"） */
#define CHASSIS_RUDDER_L_DIR (1)
#define CHASSIS_RUDDER_R_DIR (1)
#define CHASSIS_WHEEL_L_DIR (1)
#define CHASSIS_WHEEL_R_DIR (1)

/*============================================
 *              机械/整定参数
 *============================================*/
/* 舵向减速比：电机转 CHASSIS_STEER_GEAR_RATIO 圈 = 舵轮 1 圈。
 * 位置反馈在 drvlib_motor 里是"电机侧 rad"；换到舵轮侧时乘 INV。 */
#define CHASSIS_STEER_GEAR_RATIO (8.0f)
#define CHASSIS_STEER_GEAR_RATIO_INV (0.125f)

/* 上电校准（光电门找零点）：两舵轮以恒定速度扫描，捕捉下降沿。
 * 单次超时只重试（漏采一圈很常见），连续 CHASSIS_CAL_MAX_RETRIES 次才判故障。
 * 5000 拍 × 2ms = 10s。 */
#define CHASSIS_STEER_ZERO_SCAN_SPEED (24.0f) // 扫描速度（电机侧 rad/s）
#define CHASSIS_CAL_TIMEOUT_TICKS (5000u)
#define CHASSIS_CAL_MAX_RETRIES (3u)

/* 舵向位置环输出限幅 = 速度给定上限（电机侧 rad/s）。
 * M3508 空载约 50rad/s，再大只会把速度环按在饱和里。 */
#define CHASSIS_STEER_VMAX_MOTOR (50.0f)

/* 校准后方向验证（云台使能时生效）：
 * 两驱动轮同时给 app 正方向的**小力矩** (Nm)，用来核对轮向与舵向零点：
 *   两轮同号且 DIR 正确 ⇒ 车沿舵轮正前方直行；
 *   原地打转 ⇒ 某一轮 CHASSIS_WHEEL_x_DIR 反了；
 *   后退/斜行 ⇒ 舵向零点偏置（RUDDER_ZERO_OFFSET_x_DEG）或舵向方向有问题。
 * 这里给的是 app 正方向，下发时仍会乘 CHASSIS_WHEEL_x_DIR，走完整符号链。 */
#define CHASSIS_WHEEL_CAL_TORQUE (0.5f)

/* 舵向环路 PID（整定值沿用旧 app；力控阶段未接入前只有校准速度环真正生效） */
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
#define CHASSIS_RUDDER_PID_ANGLE_D_LPF_RC (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_OUT_LPF_RC (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_DEADBAND (0.0f)
#define CHASSIS_RUDDER_PID_ANGLE_ERR_NORM_RANGE (0.0f) /* WRAP 下 Config 自动填 */
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MAX (CHASSIS_STEER_VMAX_MOTOR)
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MIN (-CHASSIS_STEER_VMAX_MOTOR)

/*============================================
 *              类型
 *============================================*/
/* IMU 量转到**底盘系**后的结果（旋转用安装矩阵，平移量见 pos） */
typedef struct
{
    vector3_t gyro; // 底盘系角速度 (rad/s)
    vector3_t acc;  // 底盘系比力 (m/s²)，已做杆臂补偿到 底盘中心
    vector3_t pos;  // IMU 安装点在底盘系的位置 (m)，固定
    float yaw_rate; // 底盘系绕竖直轴角速度 (rad/s)（world 系，与安装方向无关）
    uint8_t valid;  // 1 = 本拍姿态可用（已播种/有数据）
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
    float speed;        // 轮速 (rad/s)，app 正方向（沿当前舵角推动车体为正）
    float current;      // 电流 (A)
    int8_t temperature; // 温度 (℃)
    uint8_t error;      // 错误码（LK 为 echo，DJI 为 error）
} AppWheelFeedback_s;

/*============================================
 *              顶层状态机
 *============================================*/
/* 状态 = 底盘"现在能干的事"，不是云台下发的档位：
 *   CAL   上电校准：两舵轮扫描找光电门零点（此时不理会云台在线与否）
 *   FAULT 校准失败（连续超时）：失能 + 给定清零，终态（需复位重启再标）
 *   SAFE  已标定但不可动：云台掉线 或 云台下发 stop —— 失能 + 给定清零 + 作废指令
 *   RUN   已标定且可动：力控/解算入口（当前控制尚未接入，舵向保持当前位置、轮向 0 力矩） */
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
 *   反馈：kp × (指令 − 实测)
 *   前馈：摩擦（沿指令速度方向）+ 惯性/惯量（指令加速度）+ 重力/坡道分量。
 *   三组前馈系数当前全部为 0，结构先留好，标定后直接填数。 */
typedef struct
{
    /* 反馈增益（初值，待整定） */
    float kp_vx; // N/(m/s)
    float kp_vy; // N/(m/s)
    float kp_w;  // N·m/(rad/s)

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

extern ChassisForceCfg_s g_chassis_force_cfg;

/* 实测底盘速度：vx/vy 来自轮速正解，w 由 IMU 底盘系 z 轴角速度覆盖 */
extern vector3_t g_chassis_twist_fb;
/* 力控输出广义力 (fx, fy, mw) */
extern vector3_t g_chassis_wrench;
/* 力分配输出：wheel_torque = 各轮**轮缘**驱动力矩 [N·m]（沿当前舵角，正=沿轮向+），
 * lateral_force = 舵轮侧向接触力 [N]。当前只计算、不下发电机；轮序 = 左/右舵。 */
extern LibChassisTorque_s g_chassis_torque;
/* 上一拍速度指令（算惯性前馈的指令加速度） */
extern vector3_t g_chassis_cmd_prev;

/*============================================
 *              常量
 *============================================*/
/* 4x4 齐次变换：p_chassis = R · p_imu + t。
 * R 的三列 = IMU 的 x/y/z 轴在底盘系中的方向（宏在 robot_def.h）：
 *     x_imu = (−1, 1, 0)/√2   y_imu = (1, 1, 0)/√2   z_imu = (0, 0, −1)
 *     t     = IMU 中心在底盘系的位置。
 * 陀螺/加速度是方向量，用 Lib_Math_Mat4MulDir（不带平移）；
 * 点是位置量，用 Lib_Math_Mat4MulPoint（带平移）。 */
extern const matrix4_t g_imu_to_chassis;

/* 底盘车架上那颗 BMI088 的静态零偏（2026-10-17，ignore/chassis_data/imu.csv）
 * 录制条件：静置 112s、约 33℃、IMU 倒装（+z 朝下）。地面轻微不平按水平处理；
 * 陀螺零偏与安装倾角无关，故这里只标零偏，acc 的六位置标定尚未做。 */
extern const BMI088_Calib_s g_bmi088_calib;

/*============================================
 *              云台↔底盘通信
 *============================================*/
extern CommInstance gimbal_comm; // COMM_DEF 定义的实例（def.c）

extern gimbal2chassis_data_t g_gimbal2chassis_data; // 云台→底盘：速度指令 + 档位
extern volatile robot_mode g_gimbal_mode_rx;        // 接收回调写的档位原始副本
extern chassis2gimbal_data_t g_chassis2gimbal_data; // 底盘→云台：速度反馈

/*============================================
 *              运行状态
 *============================================*/
extern AppChassisImu_s g_imu_chassis; // 底盘系观测，供力控使用

/* 杆臂补偿状态：
 * a_imu = a_center + α×r + ω×(ω×r)（r = IMU 相对底盘中心的位置）
 * ⇒ a_center = a_imu − α×r − ω×(ω×r)
 * α 由底盘系 gyro 差分得到；差分对噪声敏感，先经一阶低通再乘 r。 */
extern vector3_t g_gyro_prev;     // 上一有效陀螺样本（底盘系）
extern vector3_t g_alpha_lpf;     // 低通后的角加速度（底盘系）
extern uint8_t g_gyro_prev_valid; // 0 = 还没有上一样本（首拍/掉线后）

/* 驱动轮反馈（每拍更新，失能也更新） */
extern AppWheelFeedback_s g_wheel_fb[APP_WHEEL_NUM];

/*============================================
 *              校准 / 状态机状态
 *============================================*/
extern ChassisFsmCtx_s g_chassis_fsm_ctx;
extern LibFsmTableInstance_s g_chassis_fsm;

extern uint8_t g_rudder_l_gate_state; // 上一拍光电门电平（判 1→0 下降沿）
extern uint8_t g_rudder_r_gate_state;
extern uint8_t g_rudder_l_inited; // 1 = 左舵已在光电门沿完成零点标定
extern uint8_t g_rudder_r_inited;

extern float g_rudder_l_setref; // 本拍舵向给定（电机侧 rad 或速度，取决于状态）
extern float g_rudder_r_setref;
extern float g_wheel_l_setref; // 本拍驱动轮给定力矩 (Nm)
extern float g_wheel_r_setref;

#endif // !__APP_CHASSIS_FORCE_DEF_H
