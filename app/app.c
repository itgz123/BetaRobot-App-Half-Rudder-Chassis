#include "app.h"
#include "app_cfg.h"
//
#include "bsp_log.h"
#include "bsp_app.h"
#include "bsp_dwt.h"
#include "lib_math.h"
#include "bsp_freertos.h"
#include "bsp_map.h"
//
#include "drv_daemon.h"
#include "drv_vofa.h"
//
#include "app_chassis_force.h"

/* 日志实例定义 */
LOG_INSTANCE_DEF(g_app_log, "app", 255); // app 层日志实例

// 任务STACK大小
#define CHASSIS_STACK_SIZE 1024
#define DAEMON_STACK_SIZE 512
// 任务频率设置
#define CHASSIS_FREQ_MS 2 // 底盘
/* 任务实例定义 */
TASK_INSTANCE_DEF(chassis_task, CHASSIS_STACK_SIZE);
TASK_INSTANCE_DEF(daemon_task, DAEMON_STACK_SIZE);

/* 任务函数：周期任务框架（固定周期唤醒 + 执行耗时监控 + 超时计数） */
APP_TASK_DEF(Chassis, CHASSIS_FREQ_MS, AppChassisRun)

/* daemon 周期主体：drv_daemon 不创建任务，由 app 提供任务时基（周期见 DAEMON_FREQ_MS） */
static void AppDaemonRun(float dt, uint64_t time_stamp)
{
    (void)dt;
    (void)time_stamp;
    DaemonTask();
}
APP_TASK_DEF(Daemon, DAEMON_FREQ_MS, AppDaemonRun)

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
    TaskRegister(&daemon_task, &(Task_Init_Config_s){.func = StartDaemonTask, .priority = 0});
    TaskRegister(&chassis_task, &(Task_Init_Config_s){.func = StartChassisTask, .priority = 2});
    __enable_irq(); // 开启中断
}
