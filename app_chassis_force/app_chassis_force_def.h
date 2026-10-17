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
#include "drvlib_bmi088_kalman.h" // BMI088_Calib_s
#include "drv_comm.h"             // CommInstance

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

#endif // !__APP_CHASSIS_FORCE_DEF_H
