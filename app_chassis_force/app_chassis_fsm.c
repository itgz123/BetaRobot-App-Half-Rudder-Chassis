/**
 * @file app_chassis_fsm.c
 * @brief 底盘顶层状态机实现（lib_fsm_table）
 *
 * 行为与拆分前一致：
 *   - CAL：上电校准，不理会云台在线与否；
 *   - FAULT：连续 CHASSIS_CAL_MAX_RETRIES 次超时，终态；
 *   - SAFE：已标定但掉线或 stop；
 *   - RUN：已标定、在线且非 stop。
 * 状态完全由四个条件决定，事件只有一个"每拍更新"；条件变化才派发，
 * 避免同状态每拍走一次外部自转移（退出 + 进入）。
 */

#include "app_chassis_fsm.h"

#include "lib_fsm_table.h"

#include <stdbool.h>

static ChassisFsmCfg_s s_cfg;
static ChassisFsmCtx_s s_ctx;
static LibFsmTableInstance_s s_fsm;

static uint8_t ChassisFsmCallInput(uint8_t (*fn)(void *), void *user)
{
    return (fn != NULL) ? fn(user) : 0u;
}

/*---------------- 守卫 ----------------*/
/* 守卫必须无副作用且幂等；四个条件互斥，表内先后只决定"故障优先、校准次之"。 */
static bool ChassisGuardFault(void *ctx, const void *event_data)
{
    (void)event_data;
    return ((ChassisFsmCtx_s *)ctx)->cal_fault != 0;
}

static bool ChassisGuardCal(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done == 0) && (c->cal_fault == 0);
}

static bool ChassisGuardSafe(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && ((c->online == 0) || (c->stopped != 0));
}

static bool ChassisGuardRun(void *ctx, const void *event_data)
{
    (void)event_data;
    const ChassisFsmCtx_s *c = (const ChassisFsmCtx_s *)ctx;
    return (c->cal_done != 0) && (c->online != 0) && (c->stopped == 0);
}

/*---------------- 转移表 ----------------*/
/* 源状态一律通配：四条守卫已把 (标定 × 在线 × stop) 完全划分，落点唯一；
 * 表顺序 = 优先级，故障排最前。 */
static const LibFsmTableTransition_s chassis_fsm_table[] = {
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_FAULT, ChassisGuardFault, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_CAL, ChassisGuardCal, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_SAFE, ChassisGuardSafe, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
    {LIB_FSM_TABLE_ANY_STATE, CHASSIS_EV_UPDATE, CHASSIS_STATE_RUN, ChassisGuardRun, NULL, LIB_FSM_TABLE_TRANS_EXTERNAL},
};
LIB_FSM_TABLE_CHECK_TABLE(chassis_fsm_table);

/*---------------- 状态动作 ----------------*/
static void ChassisFsmRunEntry(void *ctx)
{
    (void)ctx;
    if (s_cfg.on_run_entry != NULL)
    {
        s_cfg.on_run_entry(s_cfg.user);
    }
}

static void ChassisFsmFaultEntry(void *ctx)
{
    (void)ctx;
    if (s_cfg.on_fault_entry != NULL)
    {
        s_cfg.on_fault_entry(s_cfg.user);
    }
}

static const LibFsmTableStateActions_s chassis_fsm_actions[] = {
    [CHASSIS_STATE_CAL] = {NULL, NULL},
    [CHASSIS_STATE_FAULT] = {ChassisFsmFaultEntry, NULL},
    [CHASSIS_STATE_SAFE] = {NULL, NULL},
    [CHASSIS_STATE_RUN] = {ChassisFsmRunEntry, NULL},
};
LIB_FSM_TABLE_CHECK_ACTIONS(chassis_fsm_actions, CHASSIS_STATE_COUNT);

void ChassisFsmInit(const ChassisFsmCfg_s *cfg)
{
    s_cfg = *cfg;
    s_ctx = (ChassisFsmCtx_s){0};

    const LibFsmTableConfig_s fsm_cfg = {
        .initial_state = CHASSIS_STATE_CAL,
        .transitions = chassis_fsm_table,
        .transition_count = sizeof(chassis_fsm_table) / sizeof(chassis_fsm_table[0]),
        .state_actions = chassis_fsm_actions,
        .state_count = CHASSIS_STATE_COUNT,
        .ctx = &s_ctx,
        .trace_fn = NULL,
    };
    LibFsmTableInit(&s_fsm, &fsm_cfg);
    LibFsmTableStart(&s_fsm);
}

/**
 * @brief 每拍汇总条件，仅在条件变化时派发一次更新事件
 * @note 状态完全由四个布尔量决定；若每拍都派发，同状态也会走一次外部自转移
 *       （退出 + 进入各跑一遍），将来挂上状态动作就会每拍重跑，故只在变化时派发。
 * @note 故障为终态：进入后不再派发（需复位重启才能重标）。
 */
void ChassisFsmUpdate(void)
{
    ChassisFsmCtx_s *c = &s_ctx;

    /* 校准计时：仅在校准态累加。单次超时只重试（重新计时 + 计数 + 上报），不直接判死——
     * 上电找不到零点最常见的失败是漏采到某一圈的光电门沿，下一圈就能补上；
     * 连续 CHASSIS_CAL_MAX_RETRIES 次才判故障（那时基本是传感器/线束/挡片真坏了）。 */
    if (LibFsmTableCurrent(&s_fsm) == CHASSIS_STATE_CAL)
    {
        c->cal_ticks++;
        if ((c->cal_done == 0) && (c->cal_ticks >= CHASSIS_CAL_TIMEOUT_TICKS))
        {
            c->cal_ticks = 0;
            c->cal_retries++;
            if (s_cfg.on_cal_timeout != NULL)
            {
                s_cfg.on_cal_timeout(c->cal_retries, s_cfg.user);
            }
        }
    }

    // 采样四个条件
    c->cal_done = ChassisFsmCallInput(s_cfg.is_calibrated, s_cfg.user);
    c->cal_fault = (uint8_t)((c->cal_done == 0) && (c->cal_retries >= CHASSIS_CAL_MAX_RETRIES));
    c->online = ChassisFsmCallInput(s_cfg.is_online, s_cfg.user);
    c->stopped = ChassisFsmCallInput(s_cfg.is_stopped, s_cfg.user);

    // 打包成一个字节比较：任一条件变化就派发一次
    const uint8_t key = (uint8_t)(c->cal_done | (c->cal_fault << 1) | (c->online << 2) | (c->stopped << 3));
    if ((key != c->key_last) && (LibFsmTableCurrent(&s_fsm) != CHASSIS_STATE_FAULT))
    {
        LibFsmTableDispatch(&s_fsm, CHASSIS_EV_UPDATE, NULL);
    }
    c->key_last = key;
}

ChassisState_e ChassisFsmCurrent(void)
{
    return (ChassisState_e)LibFsmTableCurrent(&s_fsm);
}
