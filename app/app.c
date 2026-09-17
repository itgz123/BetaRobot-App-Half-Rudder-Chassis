#include "app.h"
#include "app_cfg.h"
//
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "lib_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "app_chassis.h"

/* 日志实例定义 */
LOG_INSTANCE_DEF(g_app_log, "app", 255); // app 层日志实例

// 任务STACK大小
#define CHASSIS_STACK_SIZE 1024
// 任务频率设置
#define CHASSIS_FREQ_MS 2 // 底盘
/* 任务实例定义 */
TASK_INSTANCE_DEF(chassis_task, CHASSIS_STACK_SIZE);

ITCM_RAM static __attribute__((noreturn)) void StartChassisTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    TickType_t xLastWakeTime = xTaskGetTickCount();            // 周期锚点(绝对唤醒时刻)
    const TickType_t xPeriod = pdMS_TO_TICKS(CHASSIS_FREQ_MS); // 任务周期(tick)
    BSPLOG(&g_app_log, LOG_LEVEL_INFO, "CHASSIS Task Start");
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 固定周期唤醒，避免 vTaskDelay 的周期漂移
        start = DWT_GetTimeUs();
        AppChassisRun();
        dt = DWT_GetTimeUs() - start;
        if (dt > 1000 * CHASSIS_FREQ_MS)
            BSPLOG(&g_app_log, LOG_LEVEL_ERROR, "CHASSIS Task is being DELAY! dt = %llu(us)", dt);
    }
}

static void create_queue(void)
{
}

void function_in_main_c(void)
{
    // 初始化
    __disable_irq(); // 关闭中断
    BSPInit();
    DWT_Init();
    BSPLogInit(); // 初始化日志依赖的 bsp 外设（DWT，幂等）
    DaemonInit();
    VofaInit();
    // app
    AppChassisInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&chassis_task, &(Task_Init_Config_s){.func = StartChassisTask, .priority = 2});
    __enable_irq(); // 开启中断
}
