/**
 * @file app_chassis_fsm.h
 * @brief 底盘顶层状态机：CAL / FAULT / SAFE / RUN
 *
 * 状态只由四个条件决定：(是否标定, 是否超时, 云台是否在线, 档位是否 stop)。
 * 模块负责校准计时、守卫、转移和"条件变化才派发"；具体动作（切环路、日志、
 * 失能）由 app 通过回调提供，模块不持有电机/通信实例。
 *
 * 状态枚举与上下文类型见 app_chassis_force_def.h。
 */

#ifndef __APP_CHASSIS_FSM_H
#define __APP_CHASSIS_FSM_H

#include "app_chassis_force_def.h"

typedef struct
{
    /* 四个状态条件（回调必须无副作用、可重复调用） */
    uint8_t (*is_calibrated)(void *user); // 两舵轮均已标定
    uint8_t (*is_online)(void *user);     // 云台链路在线
    uint8_t (*is_stopped)(void *user);    // 云台下发 stop

    /* 状态事件 */
    void (*on_cal_timeout)(uint8_t retries, void *user); // 单次校准超时，retries 已 +1
    void (*on_run_entry)(void *user);                    // 进入 RUN
    void (*on_fault_entry)(void *user);                  // 进入 FAULT（终态）

    void *user;
} ChassisFsmCfg_s;

void ChassisFsmInit(const ChassisFsmCfg_s *cfg);
void ChassisFsmUpdate(void);
ChassisState_e ChassisFsmCurrent(void);

#endif // !__APP_CHASSIS_FSM_H
