/**
 * @file app_chassis_output.c
 * @brief 输出实现：速度逆解 + ±180° 分支 + 力投影 + 余弦降额
 *
 * 架构：
 *   1. 舵角完全由 (vx,vy,w) 的速度逆解决定；
 *   2. 力分配只在上述两个目标轮向上投影，只取驱动行力矩；
 *   3. φ 与 φ+180° 两个等价解的分支选择由 CHASSIS_OUTPUT_SCHEME 切换：
 *        - 0：选离当前角更近的，平手保持上一拍分支；
 *        - 1：4 组合代价函数（角度 + 力矩 + 两舵反向惩罚，带切换迟滞）。
 *
 * 方案 1 与 2026-10-18 旧版的区别：旧版舵角由力方向 atan2 给出，只在当前舵角
 * 下分配一次再换 ±180°；本架构舵角来自速度逆解，故每个组合按自己的目标舵角
 * 重新分配，力矩误差直接比较“目标方向上的电机力矩”与实测电机力矩。
 */

#include "app_chassis_output.h"

#if (CHASSIS_OUTPUT_SCHEME != 0) && (CHASSIS_OUTPUT_SCHEME != 1)
#error "invalid CHASSIS_OUTPUT_SCHEME"
#endif

/* 换向决策用 ChassisCostSel_s 按舵轮序号存状态，两套轮序必须一致 */
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_L == (int)APP_WHEEL_L, "wheel order mismatch: L");
_Static_assert((int)LIB_CHASSIS_HALF_RUDDER_R == (int)APP_WHEEL_R, "wheel order mismatch: R");

static struct
{
    LibChassisInstance_s *chassis;
} s_output;

void ChassisOutputInit(const ChassisOutputCfg_s *cfg)
{
    s_output.chassis = cfg->chassis;
}

void ChassisOutputReset(ChassisOutput_s *out)
{
    *out = (ChassisOutput_s){0};
}

/**
 * @brief 余弦优化系数：max(cos(目标舵角 − 实际舵角), 0)
 * @param angle_error 舵向误差 (rad)
 * @return 驱动轮力矩降额系数，0~1；误差 ≥90° 时钳 0，不反向出力
 * @note 与旧代码 Steer_Motor_Kinematics_Nearest_Transposition 的
 *       Target_Wheel_Omega *= max(cos(Δ), 0) 对齐。
 */
static float ChassisCosineAlign(float angle_error)
{
    const float c = Lib_Math_Cos(angle_error);
    return (c > 0.0f) ? c : 0.0f;
}

#if CHASSIS_OUTPUT_SCHEME == 0

/**
 * @brief 在 φ 与 φ+180° 两个等价轮向中，选离当前舵角最近的解
 * @param sel     输出状态（读写 reverse / angle_error）
 * @param wheel   轮序
 * @param phi_des 速度逆解给出的期望轮向 (rad，任意区间)
 * @param phi_cur 当前舵角 (rad，连续)
 * @return 目标舵角 (rad，与 phi_cur 连续的表示)
 * @note 角度死区 CHASSIS_COST_ANGLE_DEADBAND 用于平手时保持上一拍分支，
 *       避免在 |Δθ|≈90° 边界逐拍翻转。
 */
static float ChassisNearestSteer(ChassisCostSel_s *sel, uint8_t wheel, float phi_des, float phi_cur)
{
    const float d_fwd = Lib_Math_WrapAngleNegPIToPI(phi_des - phi_cur);
    const float d_rev = Lib_Math_WrapAngleNegPIToPI(phi_des + M_PI - phi_cur);
    const float abs_fwd = Lib_Math_Fabs(d_fwd);
    const float abs_rev = Lib_Math_Fabs(d_rev);

    uint8_t rev;
    if (abs_rev < (abs_fwd - CHASSIS_COST_ANGLE_DEADBAND))
    {
        rev = 1u;
    }
    else if (abs_fwd < (abs_rev - CHASSIS_COST_ANGLE_DEADBAND))
    {
        rev = 0u;
    }
    else
    {
        /* 平手：保持上一拍分支；首次决策（valid=0）取正向 */
        rev = (sel->valid != 0u) ? sel->reverse[wheel] : 0u;
    }

    sel->reverse[wheel] = rev;
    const float dphi = (rev != 0u) ? d_rev : d_fwd;
    sel->angle_error[wheel] = Lib_Math_Fabs(dphi);
    return phi_cur + dphi;
}

void ChassisOutputUpdate(const ChassisOutputInput_s *in, ChassisOutput_s *out)
{
    ChassisCostSel_s *sel = &out->sel;
    LibChassisRef_s ref;

    /* 1) 速度逆解：期望轮向 = 该轮回转中心的接触点速度方向 */
    LibChassisInverse(s_output.chassis, in->twist_cmd, &ref);

    LibChassisState_s st_target = in->state;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if (ref.wheel_speed[i] <= 0.0f)
        {
            /* 回转中心速度≈0：速度逆解不定义方向，保持当前舵角 */
            st_target.steer_angle[i] = in->state.steer_angle[i];
            sel->angle_target[i] = in->state.steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->reverse[i] = (sel->valid != 0u) ? sel->reverse[i] : 0u;
        }
        else
        {
            const float phi_t = ChassisNearestSteer(sel, i, ref.steer_target[i], in->state.steer_angle[i]);
            st_target.steer_angle[i] = phi_t;
            sel->angle_target[i] = phi_t;
        }
    }

    /* 2) 力分解：用目标舵向构造驱动行，只取驱动行力矩。
     *    侧向行只是让 (JᵀJ) 可逆、保持最小范数分配的形状；其输出 lateral_force
     *    在本架构里不参与舵向，也不再换算成舵向电机力矩。 */
    LibChassisTorque_s alloc;
    if (LibChassisAllocateTorque(s_output.chassis, &st_target, in->wrench, &alloc) != 0)
    {
        /* 几何退化：本轮不出力，但舵向仍按速度逆解走 */
        out->torque = (LibChassisTorque_s){0};
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->wheel_torque[i] = 0.0f;
            sel->wheel_align[i] = ChassisCosineAlign(sel->angle_error[i]);
        }
    }
    else
    {
        out->torque = alloc;
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->wheel_torque[i] = alloc.wheel_torque[i];
            /* 余弦优化：力按目标舵向分配，舵向还没追上时按误差余弦降额，
             * 避免驱动轮在错误方向上顶满力。90° 及以上钳 0（不反向出力）。 */
            sel->wheel_align[i] = ChassisCosineAlign(sel->angle_error[i]);
        }
    }

    sel->locked = 0u;
    sel->valid = 1u;
    for (uint8_t k = 0u; k < 4u; k++)
    {
        sel->cost[k] = 0.0f; // 最短路径方案不做组合代价，遥测通道保留
    }
    sel->combo = (uint8_t)((sel->reverse[LIB_CHASSIS_HALF_RUDDER_L] & 0x1u) | ((sel->reverse[LIB_CHASSIS_HALF_RUDDER_R] & 0x1u) << 1));
}

#else // CHASSIS_OUTPUT_SCHEME == 1

/* 分配失败时组合代价的哨兵值（正常代价远小于它） */
#define CHASSIS_COST_INF (1.0e30f)

/**
 * @brief 按组合位构造目标舵角，并给出各轮的连续角度误差
 * @param in       输出输入
 * @param ref      速度逆解结果
 * @param combo    反向位组合（bit0=L, bit1=R）
 * @param dir_mask 有速度方向的轮（bit=1）；无方向的轮保持当前舵角
 * @param st_target 输出：目标状态（舵角为连续表示）
 * @param dphi     输出：各轮目标与当前舵角之差 (rad，已 wrap 到 ±π)
 */
static void ChassisBuildComboTarget(const ChassisOutputInput_s *in,
                                    const LibChassisRef_s *ref,
                                    uint8_t combo,
                                    uint8_t dir_mask,
                                    LibChassisState_s *st_target,
                                    float dphi[LIB_CHASSIS_HALF_RUDDER_NUM])
{
    *st_target = in->state;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if ((dir_mask & (uint8_t)(1u << i)) != 0u)
        {
            const uint8_t rev = (uint8_t)((combo >> i) & 0x1u);
            const float phi_t = ref->steer_target[i] + ((rev != 0u) ? M_PI : 0.0f);
            dphi[i] = Lib_Math_WrapAngleNegPIToPI(phi_t - in->state.steer_angle[i]);
            st_target->steer_angle[i] = in->state.steer_angle[i] + dphi[i];
        }
        else
        {
            dphi[i] = 0.0f;
            st_target->steer_angle[i] = in->state.steer_angle[i];
        }
    }
}

void ChassisOutputUpdate(const ChassisOutputInput_s *in, ChassisOutput_s *out)
{
    ChassisCostSel_s *sel = &out->sel;
    LibChassisRef_s ref;

    /* 1) 速度逆解：期望轮向 = 该轮回转中心的接触点速度方向 */
    LibChassisInverse(s_output.chassis, in->twist_cmd, &ref);

    uint8_t dir_mask = 0u;
    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        if (ref.wheel_speed[i] > 0.0f)
        {
            dir_mask |= (uint8_t)(1u << i);
        }
    }

    /* 无方向的轮没有 ±180° 可选，沿用上一拍的反向位，避免无意义翻转 */
    const uint8_t keep = (sel->valid != 0u) ? (uint8_t)(sel->combo & (uint8_t)(~dir_mask)) : 0u;

    /* 1a) 两轮都没有速度方向：没有分支可选。
     *     力也很小时按历史力矩锁定逻辑保持当前舵角并停止出力。 */
    if (dir_mask == 0u)
    {
        LibChassisState_s st_now = in->state;
        LibChassisTorque_s alloc_now;
        const uint8_t alloc_ok = (uint8_t)(LibChassisAllocateTorque(s_output.chassis, &st_now, in->wrench, &alloc_now) == 0);
        const uint8_t small = (uint8_t)((alloc_ok != 0u) &&
                                        (Lib_Math_Fabs(alloc_now.wheel_torque[LIB_CHASSIS_HALF_RUDDER_L]) < CHASSIS_COST_TORQUE_LOCK) &&
                                        (Lib_Math_Fabs(alloc_now.wheel_torque[LIB_CHASSIS_HALF_RUDDER_R]) < CHASSIS_COST_TORQUE_LOCK));

        if (small != 0u)
        {
            sel->locked = 1u;
            out->torque = (LibChassisTorque_s){0};
            for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
            {
                sel->angle_target[i] = in->state.steer_angle[i];
                sel->angle_error[i] = 0.0f;
                sel->wheel_torque[i] = 0.0f;
                sel->wheel_align[i] = 1.0f;
                sel->reverse[i] = (uint8_t)((keep >> i) & 0x1u);
            }
            return; // valid 保持不变：首次锁定时 RUN 分支仍按“未决策”保持当前舵角
        }

        sel->locked = 0u;
        sel->valid = 1u;
        sel->combo = keep;
        if (alloc_ok != 0u)
        {
            out->torque = alloc_now;
        }
        else
        {
            out->torque = (LibChassisTorque_s){0};
        }
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            sel->angle_target[i] = in->state.steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->wheel_torque[i] = (alloc_ok != 0u) ? alloc_now.wheel_torque[i] : 0.0f;
            sel->wheel_align[i] = 1.0f;
            sel->reverse[i] = (uint8_t)((keep >> i) & 0x1u);
        }
        for (uint8_t k = 0u; k < 4u; k++)
        {
            sel->cost[k] = 0.0f;
        }
        return;
    }

    /* 2) 4 组合代价：角度误差 + 力矩误差 + 两舵反向惩罚。
     *    每个组合都用自己的目标舵向重新做一次力分配，保证比较的是该组合
     *    真正会下发的力矩；无方向的轮保持当前舵角、反转位沿用上一拍。
     *
     *    与旧版的一个取舍：旧版有“单轮力矩小就不转舵”的冻结，因为旧版舵角
     *    由力方向决定；本架构舵角由速度逆解决定，只要该轮有速度方向就必须
     *    转到位，故只保留“两轮都没有速度方向”时的整体力矩锁定。 */
    float combo_cost[4];
    uint8_t combo_ok[4];

    for (uint8_t raw = 0u; raw < 4u; raw++)
    {
        const uint8_t combo = (uint8_t)((raw & dir_mask) | keep);
        LibChassisState_s st_target;
        float dphi[LIB_CHASSIS_HALF_RUDDER_NUM];
        ChassisBuildComboTarget(in, &ref, combo, dir_mask, &st_target, dphi);

        LibChassisTorque_s alloc;
        if (LibChassisAllocateTorque(s_output.chassis, &st_target, in->wrench, &alloc) != 0)
        {
            combo_cost[raw] = CHASSIS_COST_INF;
            combo_ok[raw] = 0u;
            sel->cost[raw] = CHASSIS_COST_INF;
            continue;
        }

        int8_t rot[LIB_CHASSIS_HALF_RUDDER_NUM] = {0};
        float c = 0.0f;
        for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
        {
            const uint8_t has_dir = (uint8_t)((dir_mask >> i) & 0x1u);

            float theta_e = 0.0f;
            if (has_dir != 0u)
            {
                theta_e = Lib_Math_Fabs(dphi[i]) - CHASSIS_COST_ANGLE_DEADBAND;
                if (theta_e < 0.0f)
                {
                    theta_e = 0.0f;
                }
                if (dphi[i] > CHASSIS_COST_ANGLE_DEADBAND)
                {
                    rot[i] = 1;
                }
                else if (dphi[i] < -CHASSIS_COST_ANGLE_DEADBAND)
                {
                    rot[i] = -1;
                }
            }

            /* alloc 的力矩符号已经是“沿该组合目标轮向”的电机力矩：
             * 反向解指向 φ+180°，要产生同样的车体力，力矩符号自然为负。
             * 直接与实测电机力矩相减，就是换向需要反转的力矩量。 */
            const float tau_e = Lib_Math_Fabs(alloc.wheel_torque[i] - in->wheel_torque_fb[i]);
            c += theta_e * CHASSIS_COST_RUDDER + tau_e * CHASSIS_COST_WHEEL;
        }

        if ((rot[LIB_CHASSIS_HALF_RUDDER_L] != 0) && (rot[LIB_CHASSIS_HALF_RUDDER_R] != 0) &&
            (rot[LIB_CHASSIS_HALF_RUDDER_L] != rot[LIB_CHASSIS_HALF_RUDDER_R]))
        {
            c += CHASSIS_COST_DIFF;
        }

        combo_cost[raw] = c;
        combo_ok[raw] = 1u;
        sel->cost[raw] = c;
    }

    /* 3) 选择：首次直接取最优；之后只有优于当前超过迟滞才切换 */
    uint8_t best = 0u;
    for (uint8_t raw = 1u; raw < 4u; raw++)
    {
        if (combo_cost[raw] < combo_cost[best])
        {
            best = raw;
        }
    }
    const uint8_t best_eff = (uint8_t)((best & dir_mask) | keep);
    if (sel->valid == 0u)
    {
        sel->combo = best_eff;
    }
    else if ((best_eff != sel->combo) && (combo_ok[best] != 0u) &&
             ((combo_cost[sel->combo] - combo_cost[best]) > CHASSIS_COST_SWITCH_MARGIN))
    {
        sel->combo = best_eff;
    }

    /* 4) 按选中的组合重新构造目标并分配力矩（与代价计算使用同一套公式） */
    LibChassisState_s st_target;
    float dphi[LIB_CHASSIS_HALF_RUDDER_NUM];
    ChassisBuildComboTarget(in, &ref, sel->combo, dir_mask, &st_target, dphi);

    LibChassisTorque_s alloc;
    const uint8_t alloc_ok = (uint8_t)(LibChassisAllocateTorque(s_output.chassis, &st_target, in->wrench, &alloc) == 0);
    if (alloc_ok != 0u)
    {
        out->torque = alloc;
    }
    else
    {
        out->torque = (LibChassisTorque_s){0};
    }

    for (uint8_t i = 0; i < LIB_CHASSIS_HALF_RUDDER_NUM; i++)
    {
        const uint8_t rev = (uint8_t)((sel->combo >> i) & 0x1u);
        const uint8_t has_dir = (uint8_t)((dir_mask >> i) & 0x1u);

        if (has_dir != 0u)
        {
            const float theta = Lib_Math_Fabs(dphi[i]);
            /* 死区内不做微小修正，避免舵轮在目标附近抖 */
            sel->angle_target[i] = (theta <= CHASSIS_COST_ANGLE_DEADBAND) ? in->state.steer_angle[i] : (in->state.steer_angle[i] + dphi[i]);
            sel->angle_error[i] = theta;
            sel->wheel_align[i] = ChassisCosineAlign(dphi[i]);
        }
        else
        {
            sel->angle_target[i] = in->state.steer_angle[i];
            sel->angle_error[i] = 0.0f;
            sel->wheel_align[i] = 1.0f;
        }

        sel->wheel_torque[i] = (alloc_ok != 0u) ? alloc.wheel_torque[i] : 0.0f;
        sel->reverse[i] = rev;
    }

    sel->locked = 0u;
    sel->valid = 1u;
}

#endif // CHASSIS_OUTPUT_SCHEME
