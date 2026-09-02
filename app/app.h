#ifndef __APP_H
#define __APP_H

#include "bsp_freertos.h"

/*============================================
 *              任务创建函数
 *============================================*/
void function_in_main_c(void);

/*============================================
 *              队列句柄外部声明
 *============================================*/
extern QueueHandle_t cmd2chassis_queue_handle;
extern QueueHandle_t chassis2cmd_queue_handle;

/*============================================
 *              任务间通信
 *============================================*/
typedef struct
{
    uint8_t temp_unused;
} cmd2chassis_data_t;
typedef struct
{
    uint8_t temp_unused;
} chassis2cmd_data_t;

#endif // !__APP_H
