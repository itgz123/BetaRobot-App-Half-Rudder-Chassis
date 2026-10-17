#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

/*============================ 半舵底盘 机械参数 ============================*/
/* 舵向光电门机械零点偏置（左/右舵各一个）
 * 光电门跳变沿只是机械安装上的一个固定角度，不等于车体正前方，两者相差一个常量：
 *   偏置 > 0：从跳变沿出发，沿位置正方向（读数增大的方向）再转过该角度才到正前方；
 *   偏置 < 0：反方向转。
 * 单位：舵轮输出端角度（°）。左右舵镜像安装，符号/数值往往不同，务必分别标定。
 * 标定：上电校准完成、舵轮停在光电门处后，手工把舵轮摆到正前方，读此时的位置
 *       （电机侧 rad，看 VOFA 通道 1），则
 *       偏置(°) = 位置(rad) / STEER_GEAR_RATIO * 180 / π */
#define RUDDER_ZERO_OFFSET_L_DEG (180.0f)
#define RUDDER_ZERO_OFFSET_R_DEG (-120.0f)

/*============================ 半舵底盘 运动学参数 ============================*/
/* 坐标系：x+ 向前，y+ 向左，w+ 逆时针（俯视）。
 * 以下参数由 app_chassis.c 读入，用来配置 lib_chassis 的半舵内核（LibChassisHalfRudderInit）；
 * 正/逆解本身在 lib_chassis 里，不在本工程。单位统一为 m / rad。 */

// 驱动轮：LK MF7015 直驱，无减速箱（减速比 1），故驱动速度 = 接触点线速度 / 轮半径
#define CHASSIS_WHEEL_RADIUS (0.05025f) // 驱动轮半径 (m)
#define CHASSIS_DRIVE_REDUCTION (1.0f)  // 驱动电机减速比（MF7015 直驱 = 1）

// 舵轮组回转中心在车体系中的位置 (m)。解算槽位 [0]=左舵、[1]=右舵。
// 两组前后位置相同（都装在中线附近）时 x 填同一个值，y 为 ±半轮距。
#define CHASSIS_RUDDER_L_POS_X (0.0f)      // 左舵回转中心 x（前+）
#define CHASSIS_RUDDER_L_POS_Y (0.20225f)  // 左舵回转中心 y（左+）
#define CHASSIS_RUDDER_R_POS_X (0.0f)      // 右舵回转中心 x（前+）
#define CHASSIS_RUDDER_R_POS_Y (-0.20225f) // 右舵回转中心 y（左+）

/*============================ 半舵底盘 imu和底盘中心位置关系 ============================*/
// 底盘系：前x左y上w（俯视逆时针w>0）
// imu系中心在底盘系坐标(-117.0,-95.3,0)(mm)
// imu系的方向在底盘方向
// imu系x轴方向向量(-1,1,0)
// imu系y轴方向向量(1,1,0)
// imu系z轴方向向量(0,0,-1)
// 即 R_chassis<-imu 的三列 = [(-1,1,0)/√2, (1,1,0)/√2, (0,0,-1)]
// （app 里用这些宏构造 4x4 齐次变换，一次把旋转 + 平移都带上）
/* IMU 各轴单位方向在底盘系中的分量：第一列 = IMU x 轴，第二列 = y 轴，第三列 = z 轴 */
#define CHASSIS_IMU_X_AXIS_X (-0.70710678f) // IMU x 轴在底盘 x 上的分量
#define CHASSIS_IMU_X_AXIS_Y (0.70710678f)  // IMU x 轴在底盘 y 上的分量
#define CHASSIS_IMU_X_AXIS_Z (0.0f)         // IMU x 轴在底盘 z 上的分量
#define CHASSIS_IMU_Y_AXIS_X (0.70710678f)  // IMU y 轴在底盘 x 上的分量
#define CHASSIS_IMU_Y_AXIS_Y (0.70710678f)  // IMU y 轴在底盘 y 上的分量
#define CHASSIS_IMU_Y_AXIS_Z (0.0f)         // IMU y 轴在底盘 z 上的分量
#define CHASSIS_IMU_Z_AXIS_X (0.0f)         // IMU z 轴在底盘 x 上的分量
#define CHASSIS_IMU_Z_AXIS_Y (0.0f)         // IMU z 轴在底盘 y 上的分量
#define CHASSIS_IMU_Z_AXIS_Z (-1.0f)        // IMU z 轴在底盘 z 上的分量

#define CHASSIS_IMU_POS_X (-0.117f)  // IMU 中心在底盘系的 x (m)
#define CHASSIS_IMU_POS_Y (-0.0953f) // IMU 中心在底盘系的 y (m)
#define CHASSIS_IMU_POS_Z (0.0f)     // IMU 中心在底盘系的 z (m)

/*============================ 半舵底盘 电机方向 ============================*/
// 舵向电机电流给正值，轮子以底盘参考系转向为正值（俯视逆时针）
// 轮向方向和光电门有关

#endif // !__ROBOT_DEF_H
