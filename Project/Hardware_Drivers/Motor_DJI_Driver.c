#include "Motor_DJI_Driver.h"

//电机-大疆-发送电流控制帧（FDCAN，控制帧ID，电机1电流，电机2电流，电机3电流，电机4电流）
void Motor_DJI_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, int16_t ID1_Currnet, int16_t ID2_Currnet, int16_t ID3_Currnet, int16_t ID4_Currnet)
{
    uint8_t Motor_Tx_Data[8];
    Motor_Tx_Data[0] = ID1_Currnet >> 8;
    Motor_Tx_Data[1] = ID1_Currnet & 0xFF;
    Motor_Tx_Data[2] = ID2_Currnet >> 8;
    Motor_Tx_Data[3] = ID2_Currnet & 0xFF;
    Motor_Tx_Data[4] = ID3_Currnet >> 8;
    Motor_Tx_Data[5] = ID3_Currnet & 0xFF;
    Motor_Tx_Data[6] = ID4_Currnet >> 8;
    Motor_Tx_Data[7] = ID4_Currnet & 0xFF;
    
    CAN_Send_Data_STD(hfdcan, CAN_ID, Motor_Tx_Data);
}

//电机-大疆-存储电机反馈数据（电机反馈数组，电机数据结构体）
void Motor_DJI_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    /*===| 协议解包 |===*/
    Motor_Data_Struct->Encoder = (int16_t)(Data[0] << 8 | Data[1]);
    Motor_Data_Struct->Speed_RPM = (int16_t)(Data[2] << 8 | Data[3]);
    Motor_Data_Struct->Torque = (int16_t)(Data[4] << 8 |Data[5]);
    Motor_Data_Struct->Temperature = (int16_t)(Data[6]);
    
    Motor_Data_Struct->Angle = (Motor_Data_Struct->Encoder - 4096.0f) * 180.0f / 4096.0f;
    
    /*===| 得到总角度值 |===*/
    if      (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last >  4096)    Motor_Data_Struct->Round--;
    else if (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last < -4096)    Motor_Data_Struct->Round++;
    Motor_Data_Struct->Total_Angle = 360.0f * (Motor_Data_Struct->Round + Motor_Data_Struct->Encoder / 8192.0f) - Motor_Data_Struct->Total_Angle_Offset;
    
    /*===| 记录编码器值 |===*/
    Motor_Data_Struct->Encoder_Last = Motor_Data_Struct->Encoder;
    Motor_Data_Struct->Angle_Last = Motor_Data_Struct->Angle;
    
    Motor_Data_Struct->Ticker = 0;
    Motor_Data_Struct->If_Online = 1; 
}