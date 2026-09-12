#include "app_cmd.h"
#include "app_cfg.h"
#include "app.h"
#include "robot_def.h"
//
#include "drv_comm.h"
#include "comm_media_can_idseq.h"
#include "comm_proto_custom.h"
//
#include <string.h>

/* 云台通信对话：CAN1 / IDSEQ / PROTO_CUSTOM（帧 = [0xA5][seq][payload 1B][CRC8][0x5A]）。
 * 底盘侧 tx_id= 段、rx_id=0x110 段（云台侧对调）；ID 段与 CAN1 现有占用
 * （yaw RS05 0x001/0x0FD）及后续 3508 波盘（0x1FF/0x200/0x201~0x208）均不重叠。
 * 收发 payload 同为 gimbal_chassis_data_t（1B 占位）；CAN1 上挂有经典 CAN 的 RS05，
 * 故 mode 必须 CLASSIC（不可用 FD）。 */
COMM_DEF(gimbal_comm, MEDIA_CAN_IDSEQ, CUSTOM, CUSTOM, gimbal_chassis_data_t, 4, chassis_gimbal_data_t, 28, UNPACK_IN_ISR);

static gimbal_chassis_data_t gimbal_rx_data = {0}; // 云台→底盘（on_frame 同步拷贝）
static gimbal_chassis_data_t gimbal_tx_data = {0}; // 底盘→云台（业务填写后 CommSend）

/* 云台接收出帧回调（UNPACK_IN_ISR：payload 指向接收缓冲，回调返回后即被覆盖，须同步拷贝） */
static void GimbalRecvOnFrame(const uint8_t *payload)
{
    memcpy(&gimbal_rx_data, payload, sizeof(gimbal_rx_data));
}

void AppCmdInit(void)
{
    // 云台通信（CAN1 / IDSEQ / CUSTOM）：登记 + 配置介质（ID 段/帧格式）+ 链路看门狗。
    // 一个实例即可双向：底盘在 tx_id 段发、rx_id 段收（idseq 后端已支持收发异段）。
    CommMediaCanIdseqConfig_s gimbal_comm_media_cfg = {
        .can_e = CAN_1,
        .tx_id = 0x110, /* 底盘→云台 ID 段基址 */
        .rx_id = 0x100, /* 云台→底盘 ID 段基址 */
        .frame_type = CAN_STANDARD_DATA_FRAME,
        .mode = CAN_FRAME_FORMAT_CLASSIC, /* CAN1 上有经典 CAN 的 RS05，不可用 FD */
        .timeout_ms = 1,
    };
    CommConfig_s gimbal_comm_cfg = {
        .media_cfg = &gimbal_comm_media_cfg, /* CAN 后端必须有介质配置（ID 段在此写入） */
        .on_frame = GimbalRecvOnFrame,
        .daemon_fault = DAEMON_FAULT_NONE,
        .daemon_reload = 10, /* 对端每 2ms 发一帧，超 10ms 判离线 */
    };
    CommRegister(&gimbal_comm);
    CommConfig(&gimbal_comm, &gimbal_comm_cfg);
}

ITCM_RAM void AppCmdRun(void)
{
    // 云台通信：每周期发一帧（接收已由 CAN 中断写入 gimbal_rx_data）
    gimbal_tx_data.placeholder += 0.001; /* TODO 填实际数据（如车体速度 v_x/v_y/v_z） */
    CommSend(&gimbal_comm, (uint8_t *)&gimbal_tx_data);
}
