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
 * 以下参数供 app_chassis.c 里的半舵正/逆解使用，单位统一为 m / rad。 */

// 驱动轮：LK MF7015 直驱，无减速箱（减速比 1），故驱动速度 = 接触点线速度 / 轮半径
#define CHASSIS_WHEEL_RADIUS (0.05025f) // 驱动轮半径 (m)
#define CHASSIS_DRIVE_REDUCTION (1.0f)  // 驱动电机减速比（MF7015 直驱 = 1）

// 舵轮组回转中心在车体系中的位置 (m)。解算槽位 [0]=左舵、[1]=右舵。
// 两组前后位置相同（都装在中线附近）时 x 填同一个值，y 为 ±半轮距。
#define CHASSIS_RUDDER_L_POS_X (0.0f)      // 左舵回转中心 x（前+）
#define CHASSIS_RUDDER_L_POS_Y (0.20225f)  // 左舵回转中心 y（左+）
#define CHASSIS_RUDDER_R_POS_X (0.0f)      // 右舵回转中心 x（前+）
#define CHASSIS_RUDDER_R_POS_Y (-0.20225f) // 右舵回转中心 y（左+）

#endif // !__ROBOT_DEF_H
