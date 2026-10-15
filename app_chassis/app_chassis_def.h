#ifndef __APP_CHASSIS_DEF_H
#define __APP_CHASSIS_DEF_H

/*============================ 半舵底盘 app 参数 ============================*/
/* 本文件集中放 app_chassis 的**可调常量**：传动比、上电校准、控制限幅、各环 PID 参数。
 * 调参只动这里；配置接线、状态机与控制逻辑留在 app_chassis.c。
 * 纯机械/运动学尺寸（驱动轮半径、回转中心位置、零点偏置）在 robot_def.h —— 那份是喂给
 * lib_chassis 半舵内核的几何量，本文件是不进内核的软件参数。 */

/*============================ 传动 ============================*/
/* 舵向减速比：电机转 STEER_GEAR_RATIO 圈 = 舵轮 1 圈。
 * 位置反馈送出前一律是"电机侧 rad"，凡按舵轮角度标的量（机械偏置、VOFA 通道）都要乘 INV。 */
#define STEER_GEAR_RATIO (8.0f)       // 减速比：电机 8 圈 = 舵轮 1 圈
#define STEER_GEAR_RATIO_INV (0.125f) // 减速比：电机 8 圈 = 舵轮 1 圈

/*============================ 上电校准（光电门找零点） ============================*/
// 扫描速度（电机侧 rad/s）
#define jiaozhun_speed (24.0f)
/* 校准单次超时：两光电门迟迟不跳变时，一轮别转太久。
 * ⚠ 单次超时**不再直接判死**，只重试（重新计时 + 计数 + 上报，见 ChassisFsmUpdate）：
 *   上电找不到零点最常见的失败是"某一圈漏采到光电门沿"，下一圈就能补上；
 *   连续 CHASSIS_CAL_MAX_RETRIES 次都超时才判故障（那多半是传感器/线束/挡片真有问题）。
 *   每次超时 = 5000 拍 × 2ms（任务周期见 app.c 的 CHASSIS_FREQ_MS）= 10s。 */
#define CHASSIS_CAL_TIMEOUT_TICKS (5000u)
#define CHASSIS_CAL_MAX_RETRIES (3u) // 连续 3 次超时（约 30s）判故障

/*============================ 舵向位置环输出限幅 ============================*/
/* 舵向位置环输出 = 速度给定（电机侧 rad/s）。
 * M3508 空载约 482rpm ≈ 50rad/s（舵轮侧 6.25rad/s ≈ 358°/s），给定再大也跑不到，
 * 只会把速度环按在饱和里、电流一直顶格（实测未限幅时输出峰值到 4373rad/s）。
 * 限到"电机物理极限"，位置环就不再是无限输出源。 */
#define CHASSIS_STEER_VMAX_MOTOR (50.0f)

/*============================ 舵向换向迟滞 ============================*/
/* 舵轮"最近 180° 等价解"（掉头 180° + 驱动反转）选择的迟滞带宽 (rad)。
 * 内核只给期望轮向，选哪个等价解是本层的事；判据是"|角差| 是否超过 90°"，而这条线
 * 恰是两种解等贵的地方——轮面垂直于期望速度方向（对称工况、被反推、舵角反馈噪声）
 * 时取舍会逐拍翻转：目标舵角每拍跳 ±180°、驱动符号跟着翻，位置环（kp=50）被反复踢。
 * 分档：上一拍未反向 → 要 |角差| > 90°+h 才切反向；已反向 → 回到 90°−h 以内才切回。
 * 中间 2h 的带宽里保持原状态。15° 的取法：够盖住舵角反馈噪声与指令抖动，又不至于让
 * "该掉头"的工况多拖 15°（带宽内两种解的角度代价差 ≤ 30°，对轮速大小无影响）。 */
#define CHASSIS_STEER_HYST_RAD (M_PI / 12.0f) // 15°

/*============================ 舵向（M3508）速度环 PID ============================*/
/* 整定依据 ignore/vofa+.csv（20s，0/30/60 rad/s 阶跃，2ms 周期，kp=0.12/ki=0.15 采得）：
 *   实测超调 30~38%，0→30 后还残留约 1rad/s 的慢摆（整定 1.2~4.8s）。
 *   由该数据辨识被控对象：Kt/J≈495 rad/s²/A、B/J≈3 /s、恒值负载≈1.2A、回路延迟≈6ms，
 *   叠加 20ms 速度低通（lpf_rc）；闭环复现超调 33%，与实测一致。
 *   根因是"20ms 低通 + 6ms 延迟"的相位滞后——纯 PI 只能靠降 kp 压超调，代价是变慢
 *   （kp≤0.06 才压到 12%，且负载/模型一漂就回到 25%+）。
 *   故改用微分先行（仅对反馈微分，不受目标阶跃冲击）补阻尼，并小幅提高 ki 加快积分收敛：
 *   kp 0.12→0.10、ki 0.15→0.20、kd 0→0.002、微分滤波 rc=0.004。
 *   辨识模型仿真：平均超调 33.8%→0.8%，平均整定 441ms→178ms；在负载 0.85~1.8A、
 *   Kt/J 400~640、B/J 2~4.5、延迟 4~10ms 的全范围内，最坏超调仍 <4%。
 *   kd 量纲：d_out = kd*(last_measure-measure)/dt，kd=0.002 即"每 2ms 速度变化 1rad/s 给 1A 阻尼"；
 *   实测速度噪声仅 ~0.045rad/s，经 4ms 微分滤波后噪声电流 <0.03A，可忽略。
 *   ⚠️ 积分语义：lib_pid 已修正为 i_out += ki*error*dt，ki 单位是"每秒"，不随调用频率变。 */
#define CHASSIS_RUDDER_PID_SPEED_KP (0.10f)           // 比例系数（0.12→0.10，让位给微分阻尼）
#define CHASSIS_RUDDER_PID_SPEED_KI (0.20f)           // 积分系数 [1/s]（0.15→0.20，加快慢摆收敛）
#define CHASSIS_RUDDER_PID_SPEED_KD (0.002f)          // 微分系数（配合微分先行补相位裕度）
#define CHASSIS_RUDDER_PID_SPEED_INTEGRAL_LIMIT (2.5) // 积分限幅阈值（实测峰值 i_out≈1.47，负载 +50% 仍有余量）
#define CHASSIS_RUDDER_PID_SPEED_COEF_A (20)          // 变速积分参数 A (0 = 禁用)
#define CHASSIS_RUDDER_PID_SPEED_COEF_B (20)          // 变速积分参数 B
#define CHASSIS_RUDDER_PID_SPEED_D_LPF_RC (0.004f)    // 微分滤波时间常数 RC (0 = 禁用)
#define CHASSIS_RUDDER_PID_SPEED_OUT_LPF_RC (0)       // 输出滤波时间常数 RC (0 = 禁用)
#define CHASSIS_RUDDER_PID_SPEED_DEADBAND (0)         // 死区范围 (0 = 禁用)
#define CHASSIS_RUDDER_PID_SPEED_ERR_NORM_RANGE (0)   // 误差归一化范围 (0 = 禁用)
#define CHASSIS_RUDDER_PID_SPEED_OUT_MAX (0)          // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
#define CHASSIS_RUDDER_PID_SPEED_OUT_MIN (0)          // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)

/*============================ 舵向（M3508）位置环 PID ============================*/
// 位置环：只上 P+D（无积分），误差归一化（WRAP 下由 Config 自动开）
/* kp=50：实测（ignore/chassis_data 的 kp20/kp50 对比，2ms 周期）干净段
 * |err| 均值 0.148→0.041rad、中位数 0.098→0.0096rad。⚠ 不要再往上加：
 * 串级里内环（速度环）带宽约 20~50rad/s，kp 即位置环穿越刚度，已接近内环
 * 极限（实测误差振荡 ~6 次/s 就是这量级）；继续加只会把参考跳变换成更大的
 * 电流尖峰，而不是更小的跟踪误差。 */
#define CHASSIS_RUDDER_PID_ANGLE_KP (50)                             // 比例系数 (1/s)
#define CHASSIS_RUDDER_PID_ANGLE_KI (0)                              // 积分系数 [1/s]
#define CHASSIS_RUDDER_PID_ANGLE_KD (0.3f)                           // 微分系数（补相位裕度）
#define CHASSIS_RUDDER_PID_ANGLE_INTEGRAL_LIMIT (0)                  // 积分限幅阈值（未启用 INTEGRAL_LIMIT，0 无影响）
#define CHASSIS_RUDDER_PID_ANGLE_COEF_A (0)                          // 变速积分参数 A (0 = 禁用)
#define CHASSIS_RUDDER_PID_ANGLE_COEF_B (0)                          // 变速积分参数 B
#define CHASSIS_RUDDER_PID_ANGLE_D_LPF_RC (0)                        // 微分滤波时间常数 RC (0 = 禁用)
#define CHASSIS_RUDDER_PID_ANGLE_OUT_LPF_RC (0)                      // 输出滤波时间常数 RC (0 = 禁用)
#define CHASSIS_RUDDER_PID_ANGLE_DEADBAND (0)                        // 死区范围 (0 = 禁用)
#define CHASSIS_RUDDER_PID_ANGLE_ERR_NORM_RANGE (0)                  // 误差归一化范围（WRAP 下 Config 会自动填）
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MAX (CHASSIS_STEER_VMAX_MOTOR)  // 输出上限 (rad/s)，见 CHASSIS_STEER_VMAX_MOTOR
#define CHASSIS_RUDDER_PID_ANGLE_OUT_MIN (-CHASSIS_STEER_VMAX_MOTOR) // 输出下限 (rad/s)

/*============================ 驱动轮（LK MF7015）速度环 PID ============================*/
/* 左右轮共用同一组参数（原配置两份逐字相同）——改这里两轮一起变；
 * 若某天要左右分别整定，再拆成 _L/_R 两组即可。 */
/* 整定依据 ignore/vofa+.csv（20s，给定恒 40rad/s，中途手动加摩擦，2ms 周期）——
 * 该 csv 是旧参数（kp=0.1/ki=1e-4/kd=1e-4/integral_limit=0/coef_a=coef_b=0）采得的，
 * 现场现象即"给定 40 一转手动摩擦就掉速、且松手也回不到 40"。
 *   ① 根因是积分被两处同时清零，i_out 全程恒 0：
 *      integral_limit=0 时 lib_pid 的 f_Integral_Limit 把任何积分增量截成 0；
 *      变速积分 coef_a=coef_b=0 时 f_Changing_Integration_Rate 在 error*i_out>0 时
 *      又把 i_term 置 0。于是速度环退化成纯 P：摩擦力矩 T 只能靠误差换，
 *      稳态误差 = T/kp。实测摩擦 1.04Nm 段误差 10.6rad/s（转速掉到 29.4rad/s），
 *      与 T/kp=10.4 完全吻合；实车摩擦大于手加的值，误差只会更大。
 *   ② 由闭环数据辨识被控对象（台架空载，无粘性阻尼，是"纯积分"）：
 *      w' = a*(u - T_load)，a≈975 rad/s²/Nm，回路延时≈3ms，再叠加 4ms 速度低通
 *      （speed_lpf_rc）。用该模型跑旧参数，掉速段误差 10.43 vs 实测 10.56，可信。
 *   ③ 整定：
 *      kp 0.10 保持——闭环辨识给出穿越 14.6Hz、相位裕度 54°，本来就不慢；
 *        kp=0.12 起动命令 4.84Nm 已超驱动限幅 4.51Nm（16.1A×0.28），kp=0.15 直接发散。
 *      ki 1e-4→1.0 [1/s]——唯一真正缺的东西。稳态误差 10.4→0.01rad/s，
 *        摩擦突加后约 0.2s 回到 ±1rad/s；相位裕度只从 54° 降到 48°。
 *      kd 保持 0——对象是纯积分，微分先行(对测量微分)在此等效于降低回路增益，
 *        仿真里 kd>0 反而掉速更深（与舵向 M3508 那个带阻尼的对象不同，别照搬）。
 *      integral_limit 0→4.0 [Nm]——必须大于摩擦力矩，否则积分饱和在限幅值上、
 *        误差无法归零；4.0Nm 已接近 MF7015-23T 峰值 3.8Nm，再大也没意义。
 *      coef_a/coef_b 0→20/20——|err|≤20 全积分、20~40 递减、≥40 不积分，
 *        兼作起动大误差(40rad/s)时的抗饱和，比关掉变速积分(超调 32%)好(27%)。
 *   ④ 回放实机记录的真实摩擦曲线验证：旧参数 48% 时间低于 39rad/s、30% 低于 35rad/s、
 *      最重摩擦段平均误差 10.43rad/s；新参数 10% 低于 39（且只在摩擦突加的瞬态）、
 *      4% 低于 35、最重摩擦段平均误差 -0.09rad/s。起动 u 峰值 4.04Nm（未触限幅）。
 *      裕度：a 700~1600、延时 2~5ms 范围内相位裕度 >35°、增益裕度 >7dB。 */
#define CHASSIS_WHEEL_PID_SPEED_KP (0.10f)           // 比例系数（闭环辨识复核：穿越 14.6Hz / 相位裕度 54°，保持）
#define CHASSIS_WHEEL_PID_SPEED_KI (1.0)             // 积分系数 [1/s]（1e-4→1.0，消除摩擦掉速，见上②③）
#define CHASSIS_WHEEL_PID_SPEED_KD (0)               // 微分系数（纯积分对象，微分先行只会等效降增益）
#define CHASSIS_WHEEL_PID_SPEED_INTEGRAL_LIMIT (4.0) // 积分限幅阈值 [Nm]（0→4.0；必须 > 摩擦力矩，否则掉速归不了零）
#define CHASSIS_WHEEL_PID_SPEED_COEF_A (20)          // 变速积分参数 A（0→20，20~40rad/s 误差区间递减积分）
#define CHASSIS_WHEEL_PID_SPEED_COEF_B (20)          // 变速积分参数 B（0→20，兼作起动抗饱和）
#define CHASSIS_WHEEL_PID_SPEED_D_LPF_RC (0)         // 微分滤波时间常数 RC (0 = 禁用)
#define CHASSIS_WHEEL_PID_SPEED_OUT_LPF_RC (0)       // 输出滤波时间常数 RC (0 = 禁用)
#define CHASSIS_WHEEL_PID_SPEED_DEADBAND (0)         // 死区范围 (0 = 禁用)
#define CHASSIS_WHEEL_PID_SPEED_ERR_NORM_RANGE (0)   // 误差归一化范围 (0 = 禁用, 需要 PID_ENABLE_ERROR_NORMALIZE)
#define CHASSIS_WHEEL_PID_SPEED_OUT_MAX (0)          // 输出上限 (需要 PID_ENABLE_OUTPUT_LIMIT)
#define CHASSIS_WHEEL_PID_SPEED_OUT_MIN (0)          // 输出下限 (需要 PID_ENABLE_OUTPUT_LIMIT)

#endif // !__APP_CHASSIS_DEF_H
