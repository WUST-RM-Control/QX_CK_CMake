//========================================================
//                      Commnuicate[双板通信]
//      双板通信的发送和解包
//      当前有；云台Vx,Vy,Wz,YawErr，以及一些状态标志位
//========================================================

#include "Communicate.h"
#include "RoboControl.h"
#include "INS.h"
#include "Shoot.h"
#include "SuperCap_Driver.h"
#include "Vofa.h"

/*===| 通信错误看门狗---如果10ms内没有收到双板通信数据，则重置底盘状态 |===*/
uint8_t Board_Commnuicate_Error_Ticker;


/**
 * @brief 通信任务
 */
void Communicate_Task(void *argument)
{
    int i = 0;
    memcpy(VOFA_message + VOFA_TXDATA_SIZE * 4, VOFA_tail, 4);
    osDelay(50);
    
    for(;;)
    {
        /*===| 发送超电控制帧 |===*/
        SuperCap_Send_Control_Data();
//        USB_Printf("%f,%f,%f,%f,%f,%f\n",
//                                         Motor_Data_Struct[Shoot_Fric_First_Left_ID].Speed_RPM * Fric_RPM_to_m_s,
//                                        -Motor_Data_Struct[Shoot_Fric_First_Right_ID].Speed_RPM * Fric_RPM_to_m_s,
//                                         Motor_Data_Struct[Shoot_Fric_First_Middle_ID].Speed_RPM * Fric_RPM_to_m_s,
//                                         Motor_Data_Struct[Shoot_Fric_Second_Left_ID].Speed_RPM * Fric_RPM_to_m_s,
//                                         Motor_Data_Struct[Shoot_Fric_Second_Right_ID].Speed_RPM * Fric_RPM_to_m_s,
//                                        -Motor_Data_Struct[Shoot_Fric_Second_Middle_ID].Speed_RPM * Fric_RPM_to_m_s
//                );
//        USB_Printf("%f,%f,%f,%f,%f,%f\n", 
//        Gimbal_Control_Struct.Yaw_Target,
//        Gimbal_Control_Struct.Yaw_Feedback,
//        Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Error,
//        Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,
//        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Feedback,
//        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output/100.0f
//        );

        if(i % 10 == 0)
        {
            //100Hz
            
        }
        if(i % 20 == 0)
        {
            //50Hz
            VOFA_fdata[0] = Gimbal_Control_Struct.Yaw_Feedback;
            VOFA_fdata[1] = Gimbal_Control_Struct.Yaw_Target;
            VOFA_fdata[2] = Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output;
            memcpy((char *)VOFA_message, (char *)VOFA_fdata, VOFA_TXDATA_SIZE * 4);
            HAL_UART_Transmit_IT(VOFA_UART, VOFA_message, sizeof(VOFA_message));
        
        }
        if(i % 100 == 0)
        {
            //10Hz



            
        }
        if(i % 1000 == 0)
        {
            //1Hz



            i = 0;
        }


        i++;
        osDelay(1);
    }

        
}



