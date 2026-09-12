//
// 航模 SBUS 遥控器驱动（接收机 SBUS 协议）
//
// 通过 SBUS_UART (USART1, 100000 baud) 接收 SBUS 帧，解析 16 通道后
// 按"按键分配"映射到工程现有的 Remote 结构体。
// 协议与示例代码(ESP32 工程 components/Hardware/Remote_Control.c)保持一致：
//   帧结构 [0]=0x0F, [1..22]=16通道x11bit, [23]=flags, [24]=0x00
//   通道中位 1024，满偏 ±671 -> ±1.0
//

#ifndef KAWASHIRO_FRAME_G474_REMOTE_FS6X_H
#define KAWASHIRO_FRAME_G474_REMOTE_FS6X_H

#include "stm32g4xx_hal.h"

/* ============================== SBUS 帧定义 ============================== */
#define SBUS_FRAME_LEN       25      // 一帧 25 字节
#define SBUS_HEADER          0x0F    // 帧头
#define SBUS_END             0x00    // 帧尾（第 25 字节）
#define SBUS_CHANNEL_NUM     16      // 通道数

/* ============================= 通道值换算 ================================ */
#define REMOTE_RC_MID        1024.0f // 通道中位值
#define REMOTE_RC_RANGE      671.0f  // 满偏量程（±671 -> ±1.0）

/* 开关通道阈值（归一化后 -1~1） */
#define REMOTE_SW_LOW       -0.5f    // <= 此值 = 开关低档
#define REMOTE_SW_HIGH       0.5f    // >= 此值 = 开关高档

/* ============================== 按键分配 ================================= */
/* SBUS 通道号 0~15 对应遥控器 CH1~CH16，按实际遥控器调整即可 */
#define REMOTE_CH_RIGHT_X    0       // 摇杆 -> Remote.Right_X（云台 Yaw）
#define REMOTE_CH_RIGHT_Y    1       // 摇杆 -> Remote.Right_Y（云台 Pitch）
#define REMOTE_CH_LEFT_Y     2       // 摇杆 -> Remote.Left_Y （底盘 Vy）
#define REMOTE_CH_LEFT_X     3       // 摇杆 -> Remote.Left_X （底盘 Vx）
#define REMOTE_CH_SA         4       // 开关 SA -> Remote.Mode（低档=急停，其它=正常）
#define REMOTE_CH_SB         5       // 开关 SB -> Remote.Pause（低档=安全）
#define REMOTE_CH_SC         6       // 开关 SC -> Remote.Custom_L（高档触发）
#define REMOTE_CH_SD         7       // 开关 SD -> Remote.Custom_R（高档触发）
#define REMOTE_CH_LO         8       // 旋钮 LO -> Remote.Wheel（拨轮）
#define REMOTE_CH_RO         9       // 旋钮 RO -> Remote.Trigger（高档开火）
#define REMOTE_CH_SE         10      // 备用
#define REMOTE_CH_SF         11      // 备用
#define REMOTE_CH_SG         12      // 备用
#define REMOTE_CH_SH         13      // 备用

extern uint16_t sbus_ch[SBUS_CHANNEL_NUM];

/* SBUS 帧解析 + 按键分配，结果写入工程现有的 Remote 结构体 */
void Remote_Control_GetData(uint8_t *DataBuff);

#endif //KAWASHIRO_FRAME_G474_REMOTE_FS6X_H
