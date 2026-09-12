    //====================================================================================
//                           RoboControl[机器状态控制]
//      根据操控意愿把遥控器数据处理成机器人
//      整体[RoboEnable,Vx,Vy,Wz,Yaw,Pitch等],各个模块[Gimbal,Shoot,Chassis等]的状态,
//      各个模块再根据状态执行相应的功能
//      其中包括状态设置，冲突处理，错误检测与处理等    
//
//      机器人状态结构体：RoboControl_Struct
//====================================================================================

#include "RoboControl.h"
#include "INS.h"
#include "Buzzer.h"
#include "Remote_Control.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DrEmpower.h"
#include "Aim.h"

/*===| 机器人整体状态数据结构体 |===*/
RoboControl_StructTypeDef RoboControl_Struct;

/*===| 计算Wz的PID结构体 |===*/
PID_Struct_TypeDef RoboGimbal_Wz_PID_Struct_Angle;
PID_Struct_TypeDef RoboGimbal_Wz_PID_Struct;

/*==| 电机Pitch绝对角度限位 |==*/
#define Pitch_Angle_Limit_Max -41
#define Pitch_Angle_Limit_Min -8.5

// /*===| 自定义控制器数据 |===*/
// typedef __PACKED_STRUCT
// {
//     float Angle[7];
//     uint8_t K1;
//     uint8_t K2;
// } CustomerData_StrDef;
// CustomerData_StrDef CustomerData;
// CustomerData_StrDef CustomerData_Last;

uint32_t Robo_Task_DWT_Count;
float Robo_Task_Dt;
void Robo_Task(void *argument)
{
    /*===| Yaw参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Yaw = 0;
    
    /*===| Pitch参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Pitch = 0;
    
    /*===| Wz_PID初始化 |===*/
    // PID_Init(&RoboGimbal_Wz_PID_Struct, 0.15, 0, 10, 0, 0, Robo_Wz_MaxSpeed);
    PID_Init(&RoboGimbal_Wz_PID_Struct_Angle, 5, 0, 50, 0, 0, 100);
    PID_Init(&RoboGimbal_Wz_PID_Struct,       0.15, 0,  0.01, 0, 1, Robo_Wz_MaxSpeed);

    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Chassis_Speed_Level = 3;
    RoboControl_Struct.SuperCap_State = 0;
    Referee_Data_Init();
    
    //// 初始发弹频率
    // Shoot_Control_Struct.Shoot_Frequency = 12;
    for(;;)
    {
        Robo_Task_Dt = DWT_GetDeltaT(&Robo_Task_DWT_Count);
        
        
        /*===| 如果遥控器连接 |===*/
        if(Remote.If_Remote_Connect)
        {
            /*===| 选择控制模式：摇杆|键鼠|自定义控制器 |===*/
            if(Remote.Mode != Remote_Mode_S && RoboControl_Struct.Controler != Customer) RoboControl_Struct.Controler = Joystick;
            else if(Remote.Mode == Remote_Mode_S && RoboControl_Struct.Controler != Customer) RoboControl_Struct.Controler = KeyboardMouse;   
            
            /*===| 判断是否停机 |===*/     
            if(Remote.Mode == Remote_Mode_C && RoboControl_Struct.Robo_Enable == 1) Robo_Stop();   
            else if(Remote.Mode != Remote_Mode_C && RoboControl_Struct.Robo_Enable == 0) Robo_Restart(); 
        }
        else
        {
            Robo_Stop(); 
        }
        
        
        /*===| 遥控方式 |===*/
        /*===| 遥控器摇杆控制 |===*/
        if(RoboControl_Struct.Controler == Joystick)
        {
            RemoteControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
//                if(Remote_ReleaseSingle_Pause) RoboControl_Struct.Controler = Customer;
                RemoteControl_Bool();
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0;
            }
        }
        /*===| 遥控器键鼠控制 |===*/
        else if(RoboControl_Struct.Controler == KeyboardMouse)
        {
            KeyControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
//                if(Remote_ReleaseSingle_B) RoboControl_Struct.Controler = Customer; 
                KeyControl_Bool();
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0; 
            }
        }
        /*===| 自定义控制器控制 |===*/
        else if(RoboControl_Struct.Controler == Customer)
        {
            RemoteControl_Float();
            KeyControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
                if(Remote_ReleaseSingle_Pause) RoboControl_Struct.Controler = Joystick;
                if(Remote_ReleaseSingle_B) RoboControl_Struct.Controler = KeyboardMouse;
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0;
            }
        }
        /*===| 控制参数限幅 |===*/        
        Limit_float(&RoboControl_Struct.Robo_Target_Pitch, INS_Data_Gimbal.Pitch + (Motor.Pitch.Angle-(Pitch_Angle_Limit_Max)), INS_Data_Gimbal.Pitch + (Motor.Pitch.Angle-(Pitch_Angle_Limit_Min)));
        
        /*===| 根据底盘运动状态得到Wz |===*/   
        Get_Chassis_Wz();
        
        // /*===| 控制参数低通滤波 |===*/         
        // Control_Filter();

        osDelay(1);

        
        /*===| 平滑启停 |===*/    
        if(RoboControl_Struct.Robo_Enable)
            RoboControl_Struct.Smooth_Start_K += 0.001f;
            // RoboControl_Struct.Smooth_Start_K += 0.00033333f;
        else 
            RoboControl_Struct.Smooth_Start_K -= 0.001f;
        Limit_float(&RoboControl_Struct.Smooth_Start_K, 1, 0);
        
        osDelay(1);
    }
}


/**
 * @brief 根据底盘运动状态得到Wz
 */
void Get_Chassis_Wz(void)
{
    /*===| 底盘关闭 |===*/
   if(RoboControl_Struct.Chassis_State == Chassis_OFF)
   { 
       RoboControl_Struct.Robo_Target_Wz = 0;
   }
   /*===| 底盘静止 |===*/
   else if(RoboControl_Struct.Chassis_State == Chassis_STATIC)
   {
        RoboControl_Struct.Robo_Target_Wz = 0;
   }
   /*===| 底盘跟随模式 |===*/
   else if(RoboControl_Struct.Chassis_State == Chassis_FOLLOW)
   {
    //    PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
    //    RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct_Angle, 0, RoboControl_Struct.Yaw_Err);
        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, RoboGimbal_Wz_PID_Struct_Angle.Output, Chassis_Control_Struct.Odometer_Chassis_Wz);
        RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
   }
   /*===| 底盘侧向跟随模式 |===*/
   else if(RoboControl_Struct.Chassis_State == Chassis_F_SIDE)
   {
       PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 2.0f * (RoboControl_Struct.Chassis_Follow_45_Direction_Flag - 0.5f) * 45, RoboControl_Struct.Yaw_Err);
       RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
   }
   /*===| 底盘小陀螺 |===*/
   else if(RoboControl_Struct.Chassis_State == Chassis_SPIN)
   {
       static float Sin_Tick;
       Sin_Tick += 2*PI * Robo_Task_Dt;
       if(Sin_Tick > 2*PI) Sin_Tick = 0;
       
       /*===| 小陀螺得到Wz,每次切换正反转 |===*/
       if(RoboControl_Struct.SPIN_Direction_Flag)
       {
           RoboControl_Struct.Robo_Target_Wz = 0.8f * Robo_Wz_MaxSpeed + 0.2f * sinf(Sin_Tick) * Robo_Wz_MaxSpeed;
       }                                                         
       else 
       {
           RoboControl_Struct.Robo_Target_Wz = -0.8f * Robo_Wz_MaxSpeed + 0.2f * sinf(Sin_Tick) * Robo_Wz_MaxSpeed;
       }
       /*===| 根据档位限制旋转速度，遥控器控制状态不限制 |===*/
       RoboControl_Struct.Robo_Target_Wz = RoboControl_Struct.Robo_Target_Wz * (1.0f + RoboControl_Struct.Chassis_Speed_Level) / 6.0f;
           
       Motor.Yaw.Round = 0;
   }
   /*===| 冲刺模式 |===*/
   else if(RoboControl_Struct.Chassis_State == Chassis_DASH)
   {
       /*===| 如果底盘没有回正，则等待底盘回到正方向 |===*/
       if(RoboControl_Struct.Yaw_Err > 5.0f || RoboControl_Struct.Yaw_Err < -5.0f) 
       {
           RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
           PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
           RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
       }
       else
       {
           RoboControl_Struct.Chassis_State = Chassis_DASH;
           RoboControl_Struct.Robo_Target_Wz = 0;
       }
   }  
}


/**
 * @brief 输入参数低通滤波
 */
void Control_Filter(void)
{
    RoboControl_Struct.Robo_Target_Vx    = 0.4f * RoboControl_Struct.Robo_Target_Vx    + (1 - 0.4f)  * RoboControl_Struct.Robo_Target_Vx_Last   ;
    RoboControl_Struct.Robo_Target_Vy    = 0.4f * RoboControl_Struct.Robo_Target_Vy    + (1 - 0.4f)  * RoboControl_Struct.Robo_Target_Vy_Last   ;
    RoboControl_Struct.Robo_Target_Wz    = 0.9f * RoboControl_Struct.Robo_Target_Wz    + (1 - 0.9f)  * RoboControl_Struct.Robo_Target_Wz_Last   ;
    RoboControl_Struct.Robo_Target_Yaw   = 0.8f * RoboControl_Struct.Robo_Target_Yaw   + (1 - 0.8f)  * RoboControl_Struct.Robo_Target_Yaw_Last  ;
    RoboControl_Struct.Robo_Target_Pitch = 0.8f * RoboControl_Struct.Robo_Target_Pitch + (1 - 0.8f)  * RoboControl_Struct.Robo_Target_Pitch_Last;

    RoboControl_Struct.Robo_Target_Vx_Last    =  RoboControl_Struct.Robo_Target_Vx   ;
    RoboControl_Struct.Robo_Target_Vy_Last    =  RoboControl_Struct.Robo_Target_Vy   ;              
    RoboControl_Struct.Robo_Target_Wz_Last    =  RoboControl_Struct.Robo_Target_Wz   ;              
    RoboControl_Struct.Robo_Target_Yaw_Last   =  RoboControl_Struct.Robo_Target_Yaw  ;              
    RoboControl_Struct.Robo_Target_Pitch_Last =  RoboControl_Struct.Robo_Target_Pitch; 
}


/**
 * @brief 遥控器控制
 */
void RemoteControl_Float(void)
{
    if(Remote.Wheel < 0.7f && Remote.Wheel > -0.7f)
    {
        RoboControl_Struct.Robo_Target_Vx = Robo_Vx_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f * Remote.Left_X;
        RoboControl_Struct.Robo_Target_Vy = Robo_Vy_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f * Remote.Left_Y;
        RoboControl_Struct.Robo_Target_Yaw    -= Robo_Task_Dt * 300.0f * Remote.Right_X;
        RoboControl_Struct.Robo_Target_Pitch  += Robo_Task_Dt * 80.0f  * Remote.Right_Y;  
		}
}

void RemoteControl_Bool(void)
{
    /*===| 侧滑轮往上进入自定义功能选项1，往下是选项2，取消摇杆控制移动，通过摇杆选择触发一次定义的八个功能 |===*/ 
	if(Remote.Wheel > 0.7f)
	{
		/*===| 自定义功能-左摇杆向右 |===*/
		if      (Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f)
		{
				/*===| 底盘跟随模式 |===*/
				RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
		}
		/*===| 自定义功能-左摇杆向左 |===*/
		else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f)
		{
				/*===| 底盘静止模式 |===*/
				RoboControl_Struct.Chassis_State = Chassis_STATIC;
		}
		/*===| 自定义功能-左摇杆向上 |===*/
		if      (Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f)
		{
		}
		/*===| 自定义功能-左摇杆向下 |===*/
		else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f)
		{
			
		}
		/*===| 自定义功能-右摇杆向右 |===*/
		if      (Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f)
		{
				/*===| 速度等级+1 |===*/
				RoboControl_Struct.Chassis_Speed_Level += 1;
				if(RoboControl_Struct.Chassis_Speed_Level > 5) RoboControl_Struct.Chassis_Speed_Level = 5;
		}
		/*===| 自定义功能-右摇杆向左 |===*/
		else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f)
		{
				/*===| 速度等级-1 |===*/
				RoboControl_Struct.Chassis_Speed_Level -= 1;
				if(RoboControl_Struct.Chassis_Speed_Level < 1) RoboControl_Struct.Chassis_Speed_Level = 1;
		}
		/*===| 自定义功能-右摇杆向上 |===*/
		if      (Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f)
		{
				/*===| 开超电 |===*/
				RoboControl_Struct.SuperCap_State = 1;
				Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
		}
		/*===| 自定义功能-右摇杆向下 |===*/
		else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f)
		{
				/*===| 关超电 |===*/
				RoboControl_Struct.SuperCap_State = 0;
				Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_OFF);
		}
	}
	else if(Remote.Wheel < -0.7f )
	{
		/*===| 自定义功能-左摇杆向右 |===*/
		if      (Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f)
		{
		}
		/*===| 自定义功能-左摇杆向左 |===*/
		else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f)
		{
		}
		/*===| 自定义功能-左摇杆向上 |===*/
		if      (Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f)
		{
			
		}
		/*===| 自定义功能-左摇杆向下 |===*/
		else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f)
		{	    	
		}
		/*===| 自定义功能-右摇杆向右 |===*/
		if      (Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f)
		{
			   

		}
		/*===| 自定义功能-右摇杆向左 |===*/
		else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f)
		{
				
		}
		/*===| 自定义功能-右摇杆向上 |===*/
		if      (Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f)
		{
					/*===| 开自瞄 |===*/
					RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
					Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
		}
		/*===| 自定义功能-右摇杆向下 |===*/
		else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f)
		{
				/*===| 关自瞄 |===*/
				RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
				RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
				RoboControl_Struct.Robo_Target_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
				Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
		}
	}

	
	/*===| 单击左侧自定义按键开关小陀螺 |===*/
	if(Remote_Release_Pause && Remote_PressSingle_Custom_L)
    {
		if(RoboControl_Struct.Chassis_State != Chassis_SPIN)
		{
			RoboControl_Struct.Chassis_State = Chassis_SPIN;
			RoboControl_Struct.SPIN_Direction_Flag=!RoboControl_Struct.SPIN_Direction_Flag;
		}					
		else
			RoboControl_Struct.Chassis_State = Chassis_FOLLOW;		
    }
    
	/*===| 按住Pause并单击Triger按键开关摩擦轮 |===*/
	if(Remote_Press_Pause && Remote_PressSingle_Trigger)
	{
		if(RoboControl_Struct.Shoot_State == Shoot_State_Off)
			RoboControl_Struct.Shoot_State = Shoot_State_Ready;
		else
			RoboControl_Struct.Shoot_State = Shoot_State_Off;
	}
    
	/*===| 按下开火键单发,长按连发 |===*/
    if(Remote_Release_Pause)
	{
		if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
		{
			if (Remote_Release_Pause && Remote_Press_Trigger)
			{
				//如果打开了自瞄，则检测自瞄
				if (RoboControl_Struct.Gimbal_State != Gimbal_State_Aim ||
					( RoboControl_Struct.Gimbal_State == Gimbal_State_Aim && Aim_If_Allow_Shoot()))
				{
					RoboControl_Struct.Shoot_State = Shoot_State_Continue;
				}
				else
				{
					RoboControl_Struct.Shoot_State = Shoot_State_Ready;
				}
			}
			else
			{
				RoboControl_Struct.Shoot_State = Shoot_State_Ready;
			}
		}
	}
	
}


/**
 * @brief 键盘控制
 */
void KeyControl_Float(void)
{
    /*===| 底盘和云台运动控制 |===*/
    if(Remote_Release_Ctrl)
    {
        RoboControl_Struct.Robo_Target_Vx = Robo_Vx_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f * (Remote.Keyboard_D - Remote.Keyboard_A);
        RoboControl_Struct.Robo_Target_Vy = Robo_Vy_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f * (Remote.Keyboard_W - Remote.Keyboard_S);
    }
    if(RoboControl_Struct.Gimbal_State == Gimbal_State_Slow)
    {
        RoboControl_Struct.Robo_Target_Yaw    -= Robo_Task_Dt * 50.0f * Remote.Mouse_Vx / 500.0f;
        RoboControl_Struct.Robo_Target_Pitch  += Robo_Task_Dt * 50.0f * Remote.Mouse_Vy / 500.0f;
    }
    else
    {
        RoboControl_Struct.Robo_Target_Yaw    -= Robo_Task_Dt * 400.0f * Remote.Mouse_Vx / 500.0f;
        RoboControl_Struct.Robo_Target_Pitch  += Robo_Task_Dt * 400.0f * Remote.Mouse_Vy / 500.0f;
    }
}


/*===| 按住Ctrl进入瞄准模式 |===*/
/*===| 按X改底盘跟随模式 |===*/
/*===| 按住右键自瞄 |===*/
/*===| 按住Shift开启小陀螺 |===*/
/*===| 按Q/E向左/右转90度 |===*/
/*===| 按R开关摩擦轮 |===*/
/*===| 左键连发，如果有自瞄就有火控连发 |===*/
/*===| Ctrl+鼠标左键按下，发射一发 |===*/
/*===| 按V开关超电 |===*/
/*===| 按F切换速度档位 |===*/
/*===| ctrl+wasd调整自瞄偏置 |===*/
/*===| 按B重刷UI |===*/
/*===| 按G重启 |===*/
void KeyControl_Bool(void)
{
    /*===| 按住Ctrl进入瞄准模式 |===*/
    if(Remote_PressSingle_Ctrl)
    {
        // RoboControl_Struct.Gimbal_State = Gimbal_State_Slow;
        RoboControl_Struct.Chassis_State = Chassis_STATIC;
    }
    else if(Remote_ReleaseSingle_Ctrl)
    {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
        RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }

    /*===| 按X改底盘跟随模式 |===*/
    if(Remote_PressSingle_X) 
    {
        if(RoboControl_Struct.Chassis_State != Chassis_STATIC) RoboControl_Struct.Chassis_State = Chassis_STATIC;
        else RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }
    /*===| 按住右键自瞄 |===*/
    if(Remote_PressSingle_Mouse_R)
    {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
        Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
    }
    if(Remote_Release_Mouse_R && RoboControl_Struct.Gimbal_State == Gimbal_State_Aim)
    {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
        RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
        RoboControl_Struct.Robo_Target_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
        Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
    }
    /*===| 按住Shift开启小陀螺 |===*/
    if(Remote_PressSingle_Shift)
    {
        RoboControl_Struct.SPIN_Direction_Flag=!RoboControl_Struct.SPIN_Direction_Flag;
    }
    if(Remote_Press_Shift && RoboControl_Struct.Chassis_State != Chassis_SPIN)
    {
        RoboControl_Struct.Chassis_State = Chassis_SPIN;
    }
    else if(Remote_Release_Shift && RoboControl_Struct.Chassis_State == Chassis_SPIN)
    {
        RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }
        
        
        
//    if(Remote_PressSingle_C) 
//    {
//        if(RoboControl_Struct.Chassis_State != Chassis_SPIN)
//        {
//            RoboControl_Struct.Chassis_State = Chassis_SPIN;
//            RoboControl_Struct.SPIN_Direction_Flag=!RoboControl_Struct.SPIN_Direction_Flag;
//        }					
//        else
//        RoboControl_Struct.Chassis_State = Chassis_FOLLOW;		
//    }


    /*===| 按Q/E向左/右转90度 |===*/
    if(Remote_PressSingle_Q) RoboControl_Struct.Robo_Target_Yaw += 90;
    if(Remote_PressSingle_E) RoboControl_Struct.Robo_Target_Yaw -= 90;
    
    /*===| 按R开关摩擦轮 |===*/
    if(Remote_PressSingle_R)
    {
        if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
        {
            RoboControl_Struct.Shoot_State = Shoot_State_Off;
        }
        else
        {
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }

    if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
    {
        /*===| 左键连发，如果有自瞄就有火控连发 |===*/
        if(Remote_Press_Mouse_L)
        {
            if(RoboControl_Struct.Gimbal_State == Gimbal_State_Aim)
            {
                if(Aim_If_Allow_Shoot()) RoboControl_Struct.Shoot_State = Shoot_State_Continue; 
                else RoboControl_Struct.Shoot_State = Shoot_State_Ready; 
            }
            else
            {
                RoboControl_Struct.Shoot_State = Shoot_State_Continue; 
            }
        }
        else
        {
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }
    /*===| Ctrl+鼠标左键按下，发射一发 |===*/
    if(Remote_PressSingle_Mouse_L && RoboControl_Struct.Gimbal_State == Shoot_State_Ready && Remote_Press_Ctrl)
    {
        Shoot_Single(1);
    }
    /*===| 按V开关超电 |===*/
    if(Remote_PressSingle_V)
    {
        RoboControl_Struct.SuperCap_State = !RoboControl_Struct.SuperCap_State;
        if(RoboControl_Struct.SuperCap_State) Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
        else Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_OFF);
    }

    /*===| 按F切换速度档位 |===*/
    if(Remote_PressSingle_F)
    {
        RoboControl_Struct.Chassis_Speed_Level++;
        if(RoboControl_Struct.Chassis_Speed_Level >= 6) RoboControl_Struct.Chassis_Speed_Level = 1;
    } 
    
    /*===| ctrl+wasd调整自瞄偏置 |===*/
    if(Remote_Press_Ctrl)
    {
        if(Remote_PressSingle_W) Aim_Add_Operator_Offset(0, 0.2);
        if(Remote_PressSingle_S) Aim_Add_Operator_Offset(0, -0.2);
        if(Remote_PressSingle_A) Aim_Add_Operator_Offset(0.2, 0);
        if(Remote_PressSingle_D) Aim_Add_Operator_Offset(-0.2, 0);
    } 
    
    /*===| 按B重刷UI |===*/
    if(Remote_Press_B)
    {
        RoboControl_Struct.Refresh_UI_Flag = 1;
    }
    else
    {
        RoboControl_Struct.Refresh_UI_Flag = 0;
    }
    
    /*===| 按G重启 |===*/
    if(Remote_PressSingle_G)
    {
        Robo_Restart();
    } 
}


/**
 * @brief 关闭机器人
 */
void Robo_Stop(void)
{
    RoboControl_Struct.Robo_Enable = 0;

    RoboControl_Struct.Robo_Target_Vx = 0;
    RoboControl_Struct.Robo_Target_Vy = 0;
    RoboControl_Struct.Robo_Target_Wz = 0;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.SuperCap_State = 0;
    RoboControl_Struct.Chassis_State = Chassis_OFF;
}


/**
 * @brief 重启机器人
 */
void Robo_Restart(void)
{
    /*===| 重启后Pitch回到0，Yaw更新到当前位置 |===*/
    RoboControl_Struct.Robo_Target_Yaw = INS_Data_Gimbal.YawTotalAngle;
    RoboControl_Struct.Robo_Target_Pitch = 0;
    
    
    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Smooth_Start_K = 0;

    RoboControl_Struct.Robo_Enable = 1;
}

