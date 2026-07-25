//======================================================================
//                      Gimbal[云台控制]
//      驱动相应电机使云台陀螺仪达到RoboControl的Pitch，Yaw目标角度
//======================================================================

#include "Gimbal.h"
#include "RoboControl.h"
#include "INS.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Aim.h"
#include "Motor_Compensation.h"
#include "Vofa.h"
#include "DWT.h"

/*===| 云台控制数据结构体 |===*/
Gimbal_Control_StructTypeDef Gimbal_Control_Struct;



void Gimbal_Task(void *argument)
{
    /*===| 云台PID参数初始化 |===*/
    PID_Init(&Gimbal_Control_Struct.Yaw_Angle_PID_Struct,   20,   0,    550,   0,    0,   150);
    PID_Init(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct,   10,   1,  0,  65 , 1000, 16384);
    PID_Init(&Gimbal_Control_Struct.Pitch_Angle_PID_Struct,  5,   0,    200,   0,    0,   100);
    PID_Init(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct, 400,   3,    0,   250, 4000, 16384);

//    Motor_Compensation_Task();
    
//    FLASH_ErasePage(100);
//    FLASH_ErasePage(101);
//    FLASH_ErasePage(102);
//    FLASH_ErasePage(103);
//    for(uint16_t i = 0; i<1024; i++)
//    {
//    FLASH_programword(0x08000000 + 2048*(100+128) + 8*i, ((uint64_t *)(Config.Motor_Yaw_Cal_Data))[i]);
//    }

    for(uint16_t i = 0; i<1024; i++)
    {
        memcpy(&((uint64_t *)(Config.Motor_Yaw_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048*(100+128) + 8*i), 8);
    }
 
//    FLASH_ErasePage(104);
//    FLASH_ErasePage(105);
//    FLASH_ErasePage(106);
//    FLASH_ErasePage(107);
//    for(uint16_t i = 0; i<1024; i++)
//    {
//        FLASH_programword(0x08000000 + 2048*(104+128) + 8*i, ((uint64_t *)(Config.Motor_Pitch_Cal_Data))[i]);
//    }
	
    for(uint16_t i = 0; i<1024; i++)
    {
        memcpy(&((uint64_t *)(Config.Motor_Pitch_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048*(104+128) + 8*i), 8);
    }
    
    for(;;)
    {
        /*===| 得到当前Pitch和Yaw数据 |===*/
        Gimbal_Control_Struct.Pitch_Feedback  =  INS_Data_Gimbal.Pitch;
        Gimbal_Control_Struct.Yaw_Feedback    = INS_Data_Gimbal.YawTotalAngle;

    
        
        /*===| 得到云台控制参数 ===*/
        Gimbal_Control_Struct.Gimbal_State = RoboControl_Struct.Gimbal_State;
        
        /*===| 得到自瞄位置与当前位置的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Vision_Yaw_Err = INS_Data_Gimbal.Yaw - Aim_Receive_Struct.yaw;
        if(Gimbal_Control_Struct.Vision_Yaw_Err > 180.0f) Gimbal_Control_Struct.Vision_Yaw_Err = Gimbal_Control_Struct.Vision_Yaw_Err - 360;
        else if(Gimbal_Control_Struct.Vision_Yaw_Err < -180.0f) Gimbal_Control_Struct.Vision_Yaw_Err = 360 + Gimbal_Control_Struct.Vision_Yaw_Err;
        
        if(Gimbal_Control_Struct.Gimbal_State == Gimbal_State_Aim && Aim_Receive_Struct.appear)
        {
            RoboControl_Struct.Robo_Target_Yaw   = Gimbal_Control_Struct.Yaw_Feedback - Gimbal_Control_Struct.Vision_Yaw_Err;
            RoboControl_Struct.Robo_Target_Pitch = Aim_Receive_Struct.pitch;   
        }

    
                           
        Gimbal_Control_Struct.Pitch_Target = RoboControl_Struct.Robo_Target_Pitch;  
        Gimbal_Control_Struct.Yaw_Target = RoboControl_Struct.Robo_Target_Yaw + 30.0f * sin(VOFA_RxData[0] * DWT_GetTimeline_s());

        /*===| 得到Yaw与底盘正方向的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Yaw_Err = Motor.Yaw.Angle - Gimbal_Median_Yaw;
        if(Gimbal_Control_Struct.Yaw_Err >= 180) Gimbal_Control_Struct.Yaw_Err = Gimbal_Control_Struct.Yaw_Err - 360;
        if(Gimbal_Control_Struct.Yaw_Err <= -180) Gimbal_Control_Struct.Yaw_Err =Gimbal_Control_Struct.Yaw_Err + 360;
        RoboControl_Struct.Yaw_Err = Gimbal_Control_Struct.Yaw_Err;
		

        /*===| 串级PID串级计算得到Yaw电机的电流大小 |===*/
        PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Angle_PID_Struct, Gimbal_Control_Struct.Yaw_Target,                  Gimbal_Control_Struct.Yaw_Feedback);
        PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct, Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output, Motor.Yaw.Speed_RPM - Chassis_Control_Struct.Odometer_Chassis_Wz / PI * 190);
//        PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct, Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,  INS_Data_Gimbal.Yaw_Speed);
        
        /*===| 串级PID串级计算得到Pitch电机的电流大小 |===*/
        PID_Position_Calculate(&Gimbal_Control_Struct.Pitch_Angle_PID_Struct, Gimbal_Control_Struct.Pitch_Target,                  Gimbal_Control_Struct.Pitch_Feedback);
        PID_Position_Calculate(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct, Gimbal_Control_Struct.Pitch_Angle_PID_Struct.Output, -Motor.Pitch.Total_Angle_Speed_RPM);
                  
    //    USB_Printf("%f,%f,%f,%f,%f\n", 
    //    Gimbal_Control_Struct.Yaw_Target, 
    //    Gimbal_Control_Struct.Yaw_Feedback, 
    //    Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,
    //    Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output / 10000.0f,
    //    Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Error);

        /*===|电流限幅|===*/
        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output=RoboControl_Struct.Smooth_Start_K * Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output 
                                                        + Motor_Compensation_Get_Data(Motor.Yaw.Angle, Motor.Yaw.Speed_RPM, &Yaw_Compensation_Config);
        Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output=RoboControl_Struct.Smooth_Start_K*-Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output 
                                                        + Motor_Compensation_Get_Data(Motor.Pitch.Angle, Motor.Pitch.Total_Angle_Speed_RPM, &Pitch_Compensation_Config);
        Limit_float(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output,16384.0f,-16384.0f);
        Limit_float(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output,16384.0f,-16384.0f);
        /*===| 发送电流参数 |===*/
        if(RoboControl_Struct.Robo_Enable && INS_Data_Gimbal.If_INS_Init)
        {
            if(Gimbal_Control_Struct.If_Setting_Zero)
            {
                Motor_DM_CMD_SetZero(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID);
            }
            else
            {
                Motor_DM_CMD_MIT(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID,0,0,0,0,Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output );
            }

            Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN, Gimbal_Pitch_Send_CAN_ID, 0,Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output , 0 ,0);

        }
        else 
        {
            Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN, Gimbal_Pitch_Send_CAN_ID, 0, 0 ,0 ,0);
            Motor_DM_CMD_MIT(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID,0,0,0,0,0);
        }
        osDelay(1);
    }
}
