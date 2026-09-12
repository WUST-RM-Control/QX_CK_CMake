//
// 航模 SBUS 遥控器驱动（接收机 SBUS 协议）
//
// 移植自示例代码: ESP32 工程 components/Hardware/Remote_Control.c
// 符号约定(取反)与原示例保持一致，确保摇杆/开关方向一致。
//

#include "Remote_FS6X.h"
#include "Remote_Control.h"

uint16_t sbus_ch[SBUS_CHANNEL_NUM];

/**
 * @brief 从 25 字节 SBUS 帧解析出 16 个通道（每通道 11 bit，小端位流）
 * @param buf SBUS 帧，buf[0] 必须为 0x0F
 */
static void SBUS_Decode(const uint8_t *buf)
{
    for (int i = 0; i < SBUS_CHANNEL_NUM; i++)
    {
        int start = i * 11;       // 通道起始 bit（相对 buf[1] 的 bit0）
        int byte  = start >> 3;   // 起始字节
        int bit   = start & 7;    // 字节内起始 bit

        sbus_ch[i]  = ((uint16_t)buf[1 + byte] >> bit)
                    | ((uint16_t)buf[2 + byte] << (8 - bit))
                    | ((uint16_t)buf[3 + byte] << (16 - bit));
        sbus_ch[i] &= 0x07FF;
    }
}

/**
 * @brief 通道归一化到 [-1,1]（中位 1024，满偏 671）并限幅
 * @param ch  11 位通道原始值
 * @param inv 是否取反（与原示例代码符号保持一致）
 */
static float RC_Normalize(uint16_t ch, int inv)
{
    float v = (ch - REMOTE_RC_MID) / REMOTE_RC_RANGE;

    if      (v >  1.0f) v =  1.0f;
    else if (v < -1.0f) v = -1.0f;

    return inv ? -v : v;
}

/**
 * @brief SBUS 数据解包 + 按键分配
 * @param DataBuff SBUS 帧（SBUS_UART 收到的数据）
 */
void Remote_Control_GetData(uint8_t *DataBuff)
{
    /* 帧头帧尾校验 */
    if (DataBuff[0] != SBUS_HEADER || DataBuff[SBUS_FRAME_LEN - 1] != SBUS_END)
        return;

    SBUS_Decode(DataBuff);

    /* ---------- 摇杆四轴（符号与示例代码一致） ---------- */
    Remote.Right_X = RC_Normalize(sbus_ch[REMOTE_CH_RIGHT_X], 0);
    Remote.Right_Y = RC_Normalize(sbus_ch[REMOTE_CH_RIGHT_Y], 1);
    Remote.Left_Y  = RC_Normalize(sbus_ch[REMOTE_CH_LEFT_Y],  1);
    Remote.Left_X  = RC_Normalize(sbus_ch[REMOTE_CH_LEFT_X],  0);

    /* ---------- 开关/旋钮通道（符号与示例代码一致） ---------- */
    float sa = RC_Normalize(sbus_ch[REMOTE_CH_SA], 1);
    float sb = RC_Normalize(sbus_ch[REMOTE_CH_SB], 1);
    float sc = RC_Normalize(sbus_ch[REMOTE_CH_SC], 1);
    float sd = RC_Normalize(sbus_ch[REMOTE_CH_SD], 1);
    float lo = RC_Normalize(sbus_ch[REMOTE_CH_LO], 1);
    float ro = RC_Normalize(sbus_ch[REMOTE_CH_RO], 1);

    /* ---------- 按键分配 ---------- */
    /* SA 总开关：低档 -> 急停(Remote_Mode_C)，其它 -> 正常(Remote_Mode_N) */
    Remote.Mode = (sa <= REMOTE_SW_LOW) ? Remote_Mode_C : Remote_Mode_N;

    /* SB 安全开关：低档 -> Pause=1（禁止发射），其它 -> Pause=0 */
    Remote.Pause = (sb <= REMOTE_SW_LOW) ? 1 : 0;

    /* SC/SD：高档触发自定义键 */
    Remote.Custom_L = (sc >= REMOTE_SW_HIGH) ? 1 : 0;
    Remote.Custom_R = (sd >= REMOTE_SW_HIGH) ? 1 : 0;

    /* LO 旋钮 -> 拨轮（自定义功能菜单），RO 旋钮 -> 扳机（高档开火） */
    Remote.Wheel   = lo;
    Remote.Trigger = (ro >= REMOTE_SW_HIGH) ? 1 : 0;

    /* ---------- 连接状态刷新 ---------- */
    Remote.If_Remote_Connect    = 1;
    Remote.Remote_GetData_Ticker = 0;
    Remote.If_Remote_Data_New   = 1;
}
