#include "app.h"
#include "app_cfg.h"
//
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "app_cmd.h"
#include "app_chassis.h"

/* 日志实例定义 */
LOG_INSTANCE_DEF(g_app_log, "app", 0); // app 层日志实例

/* 队列实例定义 */
QUEUE_INSTANCE_DEF(chassis2cmd_queue, 1, chassis2cmd_data_t);
QUEUE_INSTANCE_DEF(cmd2chassis_queue, 1, cmd2chassis_data_t);

/* 队列句柄（非static，供其他模块通过 extern 访问） */
QueueHandle_t chassis2cmd_queue_handle = NULL;
QueueHandle_t cmd2chassis_queue_handle = NULL;

// 任务STACK大小
#define CMD_STACK_SIZE 1024
#define CHASSIS_STACK_SIZE 1024
// 任务频率设置
#define CMD_FREQ_MS 2     // 遥控
#define CHASSIS_FREQ_MS 2 // 底盘
/* 任务实例定义 */
TASK_INSTANCE_DEF(cmd_task, CMD_STACK_SIZE);
TASK_INSTANCE_DEF(chassis_task, CHASSIS_STACK_SIZE);

ITCM_RAM static __attribute__((noreturn)) void StartCmdTask(void *argument)
{
    static uint64_t start;
    static uint64_t dt;
    TickType_t xLastWakeTime = xTaskGetTickCount();        // 周期锚点(绝对唤醒时刻)
    const TickType_t xPeriod = pdMS_TO_TICKS(CMD_FREQ_MS); // 任务周期(tick)
    BSPLOG(&g_app_log, LOG_LEVEL_INFO, "CMD Task Start");
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 固定周期唤醒，避免 vTaskDelay 的周期漂移
        start = DWT_GetTimeUs();
        AppCmdRun();
        dt = DWT_GetTimeUs() - start;
        if (dt > 1000 * CMD_FREQ_MS)
            BSPLOG(&g_app_log, LOG_LEVEL_ERROR, "CMD Task is being DELAY! dt = %llu(us)", dt);
    }
}

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
    chassis2cmd_queue_handle = QueueRegister(&chassis2cmd_queue);
    cmd2chassis_queue_handle = QueueRegister(&cmd2chassis_queue);
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
    AppCmdInit();

    // 创建队列
    create_queue();

    // 注册任务
    TaskRegister(&cmd_task, &(Task_Init_Config_s){.func = StartCmdTask, .priority = 2});
    TaskRegister(&chassis_task, &(Task_Init_Config_s){.func = StartChassisTask, .priority = 2});
    __enable_irq(); // 开启中断
}
