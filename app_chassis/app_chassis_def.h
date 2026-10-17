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

/*============================ 舵向换向判据（档2：短时域代价预测） ============================*/
/* 舵轮有两个等价解：(d, 正转) ≡ (d+180°, 反转)，推力 τ = φ + 180°·rev 完全相同 ⇒ 选哪个是
 * **代价问题**，不是对错问题。档1 把判据写成"收益 > 代价"的解析式（|e| 与 90°+裕量比大小），
 * 档2 换成把两个候选各演一遍、比过渡期整车偏得多：J(c) = ∫₀ᵀ ‖ξ̂(t;c) − ξ*‖²_W dt（见 .c）。
 * 上面那些阈值常量（FLIP_MARGIN/FLIP/SYNC）随之全部删除——判据里不再有"90°""裕量""平移/自旋
 * 分类"，它们只是 J 在对称工况下的退化形式（§2.4 的解析式现已是 J 的先验）。
 *
 * ω_max 不再单独标定：它就是舵向位置环输出的上限（速度给定上限）折算到舵轮侧。
 * 实机印证：日志里那次 183.5° 单轮磨盘，舵向速度稳定在 50rad/s（= 限幅值）⇒ 位置环确实
 * 一直在饱和跑，用"限幅值"当角速度上界与实测一致（= 358°/s）。 */
#define CHASSIS_STEER_OMEGA_MAX_RAD (CHASSIS_STEER_VMAX_MOTOR / STEER_GEAR_RATIO) // 6.25rad/s ≈ 358°/s

/* 预测采样：每个候选 N 步、步长自适应（= 该候选行程 / ω_max / N）。取固定步数而非固定步长，
 * 是为了让"转 20°"和"转 180°"两个候选的积分分辨率一样（否则慢的那些被多算了几倍的样本）。
 * 16 步对 0.1~0.5s 的过渡 = 6~30ms 一个样本，积分出的 J 只用于排序，足够。
 * 算量：4 候选 × 16 步 = 64 次正解，仅在两轮都在过渡时才跑；稳态下两轮都已在目标角 1° 内
 * （实测角度环误差 rms 0.05°），走 ZERO_RAD 快路径直接返回，不演。 */
#define CHASSIS_STEER_COST_STEPS (16u)
/* 代价权重（先按 |v*|、|w*| 归一化再加权）：偏航最贵 ⇒ w_w 最大。
 * 投影：把两轮快 3 倍地转到位的收益，也换不回过渡期一直拧着车的代价。 */
#define CHASSIS_STEER_COST_W_V (1.0f) // 线速度误差权重
#define CHASSIS_STEER_COST_W_W (3.0f) // 偏航误差权重（最恨自转）
/* 归一键的下限：纯自旋指令 vx=vy=0、纯平移指令 w*=0，不设下限就会除以 ~0 把代价放大几个
 * 数量级（纯平移时偏航误差用 1rad/s 当尺子，纯自旋时线速度用自旋在轮上产生的轮速 |w|·|y|）。 */
#define CHASSIS_STEER_COST_SCALE_MIN (1e-3f)
#define CHASSIS_STEER_COST_W_SCALE_MIN (1.0f)
/* 两轮行程都在 1° 内 ⇒ 该候选 J ≡ 0（J 非负，故必为全局最优）⇒ 直接选它、不演。
 * 稳态跟踪（角度环误差 rms 0.05°）全靠这条走快路径；阈值取 1° 远大于跟踪误差、远小于
 * 任何"需要决策"的角差，不会把真要比较的候选误判成"已到位"。 */
#define CHASSIS_STEER_COST_ZERO_RAD (0.0174533f) // 1°
/* 正解解不出（几何退化，JᵀJ 奇异）时给该候选的代价：足够大以落选，但仍是有限值。 */
#define CHASSIS_STEER_COST_INFEASIBLE (1e3f)

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

/*============================ 云台线协议 twist 符号补偿 ============================*/
/* 实车核对（2026-10-17）：云台 w>0 对应左转（逆时针）没问题；底盘驱动电机 L/R 也没接反
 * （拔左轮电源，掉线的是 rudder_l_motor）。真正的原因是**驱动轮轮速正方向与舵角参考方向
 * 相反**：模型里"轮速 +v 沿舵角 φ 滚动"，实车是沿 −φ 滚动 ⇒ 模型算出的整条 twist 是
 * 物理 twist 的负（x、y、w 三个分量一起反，不只是自转）。
 *   前进：wire vx>0 → 模型 twist.x<0 → 轮速 v<0 → 物理前进（所以一直"看起来是对的"，
 *         其实入口的 -cmd->vx 正是在补这一下；w 之前漏了负号，才单独表现为转向反）。
 * 所以收发边界对**整条 twist** 取反一次：
 *     twist      = CHASSIS_WIRE_TWIST_SIGN * (cmd->vx, cmd->vy, cmd->w)
 *     chassis2gimbal = CHASSIS_WIRE_TWIST_SIGN * odom
 * 只翻协议边界；lib_chassis 内部的逆解/正解/换向代价函数仍共用同一套内部约定，
 * 三者保持自洽（不要在内部各调一次，否则换向代价预测会用到另一套符号）。
 * 若要根治，可把两个驱动轮电机的 motor/feedback_direction 同时翻过来、再删掉本补偿；
 * 但当前改动已用日志（w.csv：指令 -1.135，IMU 实测 +1.24）与实车核对验证。
 * 云台侧若之后改符号，把这里改回 +1.0f 即可。 */
#define CHASSIS_WIRE_TWIST_SIGN (-1.0f)

/*============================ 偏航观测（底盘 IMU） ============================*/
/* IMU 安装方向校正：BMI088 的 z 轴与车体 z 同向填 +1.0f，反向（绕 z 装反 180°）填 -1.0f。
 * 判定方法（只需一次，10 秒）：车静止 → 手动把车绕竖直轴朝 +z（俯视逆时针）转约 90°
 *   → 看 VOFA ch28/ch29：数值随转动方向为正就保持 +1，为负就填 -1。再反向转一次复核。
 * ⚠ 只作用于**观测通道** ch28/29（KF 的 yaw_rate/euler）。偏航闭环不用这两个量，
 *   而是用下面 dot(gyro,u) 的物理投影——ch28 会被 KF roll 奇异翻符号（见 yaw 闭环一节）。
 * ⚠ 航向没有绝对参考（六轴），ch29 只应看变化量；符号翻转不改变这一点。 */
#define CHASSIS_IMU_YAW_SIGN (1.0f)

/*============================ 偏航轴闭环（yaw rate + 航向） ============================*/
/* 反馈量 = 原始陀螺在"世界向上"方向的投影 dot(gyro, u)。不用 imu_data.yaw_rate：
 * 它经过 KF 欧拉角（ψ̇ = (sinφ·ωy + cosφ·ωz)/cosθ）换算，本机 KF roll 会漂到 ±90°
 * 附近，符号/幅值都被姿态投影污染（`w 给0还是漂移.csv` 里 ch28 与物理同号仅 42.6%，
 * 而物理投影 vs 指令 100%、模型 vs 物理 96.5%）。
 * 六轴下"世界向上"是安装常量：静止 acc 方向（BMI088 手册 §8.3：静止时朝上的轴读 +1g）。
 * 本机静止 acc = (-0.131, -0.013, -9.685) m/s²（2026-10-17 日志）⇒ 归一化如下。
 * ⚠ 重新拆装 IMU 后必须重新量这 3 个数；装平时 x/y 分量约 1.4%/0.1%，
 *   对 yaw 测量的轴间泄漏 <0.1°/s。 */
#define CHASSIS_IMU_UP_X (-0.013523f)
#define CHASSIS_IMU_UP_Y (-0.001342f)
#define CHASSIS_IMU_UP_Z (-0.999909f)

/* 控制律（物理/线协议约定，w>0 = 俯视逆时针；在 CHASSIS_WIRE_TWIST_SIGN 之前注入）：
 *     e = w_cmd − yaw_rate
 *     I += K_I·e·dt        // I = ∫(w_cmd−yaw_rate)dt，就是"航向误差"折算成的 w 修正
 *     w_eff = w_cmd + K_P·e + I
 * P 提供角速度阻尼（压掉实测的来回晃）；I 消除稳态航向漂移——恒定扰动下 e→0，
 * I 停在抵消扰动所需的那个常值上。
 * 为什么用"速率误差 PI"而不是"航向误差 P"：后者对斜坡指令（持续转弯）有
 * e_ss = (K_RATE/K_ANGLE)·w 的稳态航向滞后，转完停指令时车还要多转那么多去补，
 * 会过冲；速率误差的积分在匀速转弯时 e=0、I 不再增长，没有这个问题。
 *
 * ⚠ 2026-10-17 首轮实车（`效果非常差.csv`）证明第一版参数过猛：K_P=1.2、限幅 1.0，
 *   在 |cosδ|≈0 的横移段仍然满力修，41% 的 RUN 帧修正贴 ±1.0；车在 cmd.w=0 下
 *   出现 ±2.5rad/s 的极限环（轮速给定被推到 ±80rad/s、舵角在 0/π 与 ±90° 之间来回扫）。
 *   根因不是符号，是**权限**：横移时差模对偏航本来就没有权限，硬修只会把轮向从
 *   平移方向拽开，越修越转；再加上舵向到位有几百 ms 延迟，P 项一拍就把方向打到对侧，
 *   形成 bang-bang。故本版：
 *     - 加权限门控 CHASSIS_YAW_MIN_AUTHORITY：两轮 |cos(实测舵角)| 都够大才闭环，
 *       否则原样返回 w_cmd（开环）并冻结积分；
 *     - K_P/K_I/限幅整体降到"只修慢漂"的量级；
 *     - 修正量加变化率限制，避免一拍把轮向打翻。
 * 稳定性：把偏航环近似成一阶滞后 G=1/(1+τs)，闭环特征方程 τs²+(1+K_P)s+K_I=0。
 * 两个 K 都为 0 即完全关闭。
 * ⚠ 陀螺零偏已按静止段写进 app_chassis.c 的 s_bmi088_calib：航向是纯积分，
 *   残余零偏会变成积分项的稳态偏移（1LSB=0.061°/s，远小于原来的开环 7°/s）。 */
#define CHASSIS_YAW_RATE_KP (0.5f)       // 速率误差比例（yaw 阻尼，无量纲）
#define CHASSIS_YAW_RATE_KI (2.0f)       // 速率误差积分 [1/s]（= 航向项，消稳态漂移）
#define CHASSIS_YAW_INTEGRAL_MAX (0.15f) // 积分项限幅 [rad/s]
#define CHASSIS_YAW_CORR_MAX (0.30f)     // 总修正限幅 [rad/s]（≈17°/s：只修漂，不抢舵）
#define CHASSIS_YAW_CORR_SLEW (2.0f)     // 修正量变化率上限 [rad/s²]（0→0.3 约 0.15s）
#define CHASSIS_YAW_MIN_AUTHORITY (0.5f) // 两轮 |cos(舵角)| 都 > 它才闭环（横移时 ≈0，直接开环）
#define CHASSIS_YAW_RATE_LPF_RC (0.020f) // 角速度低通 [s]（压掉舵向 ~28Hz 极限环在陀螺上的分量；0 = 不用低通）
/* IMU 数据新鲜度：ODR=1kHz、控制 500Hz ⇒ 每拍都该有新样本。
 * 连续超过该时间没有新样本（或从未 seed 成功）就整段旁路，退化成原来的开环。 */
#define CHASSIS_YAW_IMU_TIMEOUT_MS (20u)

/* VOFA 发帧分频：控制任务 500Hz，遥测隔拍发 = 250Hz。
 * 加通道后 46 路每帧 196B，500Hz 要 98kB/s（>921600baud 的全部带宽，必然丢帧）；
 * 隔拍后 49kB/s = 53%，留出余量。通道值仍每拍刷新，只是每两拍取一次快照。 */
#define CHASSIS_VOFA_SEND_DIV (2u)

#endif // !__APP_CHASSIS_DEF_H
