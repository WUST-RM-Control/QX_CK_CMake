#include "UI_Task.h"
#include "RoboControl.h"
#include "Chassis.h"
#include "Referee_Unpack.h"
#include "INS.h"
#include "Aim.h"
#include "Gimbal.h"

extern float Shoot_Fric_First_Left_Speed   ;
extern float Shoot_Fric_First_Right_Speed  ;
extern uint8_t If_PC_Online;

float CAM_Angle_X = 64.4;
float CAM_Angle_Y = 40;
float CAM_X_ADD = 13;
float CAM_Y_ADD = -45;

int UI_SELF_ID;

UI_State_StructTypedef UI_State;

void UI_Init()
{
    ui_init_g_Dynamic();
    ui_init_g_Static();
}

void UI_Update()
{
    ui_update_g_Dynamic();
}

void UI_Task(void const * argument)
{
	while(Robo_State.robot_id == 0) osDelay(1000);
	UI_SELF_ID = Robo_State.robot_id;
	
	UI_Init();
    
	while(1)
	{
        UI_SELF_ID = Robo_State.robot_id;
        
        /*===| 如果要求重刷UI，则重新初始化 |===*/
        if(RoboControl_Struct.Refresh_UI_Flag)
        {
            UI_Init();
        }
        else
        {
            /*===| 获取UI显示数据 |===*/
            UI_State.Pitch       = INS_Data_Gimbal.Pitch;                      
            UI_State.Yaw         = -RoboControl_Struct.Robo_Target_Yaw;
            UI_State.YawErr      = RoboControl_Struct.Yaw_Err;   
            UI_State.Shoot_State = RoboControl_Struct.Shoot_State;
            UI_State.Fric1_Speed = fabs(Motor.Fric1.Speed_RPM * Fric_RPM_to_m_s);
            UI_State.Fric2_Speed = fabs(Motor.Fric2.Speed_RPM * Fric_RPM_to_m_s); 
            UI_State.Bullet      = Robo_BulletRemain.projectile_allowance_42mm;
            UI_State.SuperCap_IF_Enable = RoboControl_Struct.SuperCap_State;
            UI_State.SuperCap_K = (RoboControl_Struct.SuperCap_V - 6.0f) / 20.0f; //超电安全电压:8~25.5V
            if(UI_State.SuperCap_K < 0) UI_State.SuperCap_K = 0;
            UI_State.Chassis_Motor1_Speed = fabs(Motor.Chassis1.Speed_RPM * Chassis_RPM_to_m_s);
            UI_State.Chassis_Motor2_Speed = fabs(Motor.Chassis2.Speed_RPM * Chassis_RPM_to_m_s);
            UI_State.Chassis_Motor3_Speed = fabs(Motor.Chassis3.Speed_RPM * Chassis_RPM_to_m_s);
            UI_State.Chassis_Motor4_Speed = fabs(Motor.Chassis4.Speed_RPM * Chassis_RPM_to_m_s);
            UI_State.Chassis_State = RoboControl_Struct.Chassis_State;
            UI_State.Chassis_Speed_Level = RoboControl_Struct.Chassis_Speed_Level;
            
            
            /*===| Pitch |===*/
            
            ui_g_Dynamic_Pitch->start_angle = 270 + UI_State.Pitch;
            ui_g_Dynamic_Pitch->end_angle   = 271 + UI_State.Pitch;
            ui_g_Dynamic_Pitch_Num->number = UI_State.Pitch;
            
            /*===| Armor|===*/
            ui_g_Dynamic_Armor->start_x = 1633 + 150 * cos((UI_State.YawErr - 10 + 90) * 2.0f*PI/360.0f);
            ui_g_Dynamic_Armor->start_y = 703  + 150 * sin((UI_State.YawErr - 10 + 90) * 2.0f*PI/360.0f);
            ui_g_Dynamic_Armor->end_x   = 1633 + 150 * cos((UI_State.YawErr + 10 + 90) * 2.0f*PI/360.0f);
            ui_g_Dynamic_Armor->end_y   = 703  + 150 * sin((UI_State.YawErr + 10 + 90) * 2.0f*PI/360.0f);
            
            /*===| Fric |===*/
            ui_g_Dynamic_Fric_First_L->start_angle  = 0 ;
            ui_g_Dynamic_Fric_First_R->start_angle  = 0 ;
            ui_g_Dynamic_Fric_First_M->start_angle  = 0 ;
            ui_g_Dynamic_Fric_Second_L->start_angle = 0 ;
            ui_g_Dynamic_Fric_Second_R->start_angle = 0 ;
            ui_g_Dynamic_Fric_Second_M->start_angle = 0 ;
            ui_g_Dynamic_Fric_First_L->end_angle    = 1 + 330 * (UI_State.Fric1_Speed / Shoot_Fric_First_Left_Speed);
            ui_g_Dynamic_Fric_First_R->end_angle    = 1 + 330 * (UI_State.Fric2_Speed / Shoot_Fric_First_Right_Speed);
            ui_g_Dynamic_Fric_First_M->end_angle    = 0;
            ui_g_Dynamic_Fric_Second_L->end_angle   = 0;
            ui_g_Dynamic_Fric_Second_R->end_angle   = 0;
            ui_g_Dynamic_Fric_Second_M->end_angle   = 0;
            
            if(Shoot_Control_Struct.Shoot_State == Shoot_State_Ready)
            {
                ui_g_Dynamic_Fric_First_L->color = 6;
                ui_g_Dynamic_Fric_First_M->color = 6;
                ui_g_Dynamic_Fric_First_R->color = 6;
            }
            if(Shoot_Control_Struct.Shoot_State == Shoot_State_Unload)
            {
                ui_g_Dynamic_Fric_First_L->color = 5;
                ui_g_Dynamic_Fric_First_M->color = 5;
                ui_g_Dynamic_Fric_First_R->color = 5;
            }
        
            /*===| Chassis_Speed |===*/
            ui_g_Dynamic_Chassis_1->start_angle = 0;
            ui_g_Dynamic_Chassis_2->start_angle = 0;
            ui_g_Dynamic_Chassis_3->start_angle = 0;
            ui_g_Dynamic_Chassis_4->start_angle = 0;
            ui_g_Dynamic_Chassis_1->end_angle =   1 + 358 * (UI_State.Chassis_Motor1_Speed / (Robo_Vx_MaxSpeed * 1));
            ui_g_Dynamic_Chassis_2->end_angle =   1 + 358 * (UI_State.Chassis_Motor2_Speed / (Robo_Vx_MaxSpeed * 1));
            ui_g_Dynamic_Chassis_3->end_angle =   1 + 358 * (UI_State.Chassis_Motor3_Speed / (Robo_Vx_MaxSpeed * 1));
            ui_g_Dynamic_Chassis_4->end_angle =   1 + 358 * (UI_State.Chassis_Motor4_Speed / (Robo_Vx_MaxSpeed * 1));
            
            /*===| SuperCap |===*/
            ui_g_Dynamic_SuperCap->start_angle = 129 - UI_State.SuperCap_K * 79;
            if(UI_State.SuperCap_K > 0.3f)     ui_g_Dynamic_SuperCap->color = 6;
            else                                    ui_g_Dynamic_SuperCap->color = 5;
            
            if(UI_State.SuperCap_IF_Enable == 0)    ui_g_Dynamic_SuperCap->color = 4;
            
            /*===| Speed_Level |===*/
            ui_g_Dynamic_Speed->number = UI_State.Chassis_Speed_Level;
            
            /*===| Aim |===*/
            if(Gimbal_Control_Struct.Gimbal_State == Gimbal_State_Aim)
            {
                ui_g_Dynamic_Aim->color = 5; 
            }
            else
            {
                ui_g_Dynamic_Aim->color = 1;
            }
            
            if(If_PC_Online == 0)
            {
                ui_g_Dynamic_Aim->color = 3;
            }
            
            if(Aim_Receive_Struct.appear)
            {
                float Pos_Pitch = Aim_Receive_Struct.pitch;
                
                ui_g_Dynamic_Aim->start_x = 1920/2 - 1920/2 *(-Gimbal_Control_Struct.Vision_Yaw_Err) / CAM_Angle_X - 30 + CAM_X_ADD;
                ui_g_Dynamic_Aim->start_y = 1080/2 + 1080/2 *(Pos_Pitch - Gimbal_Control_Struct.Pitch_Feedback) / CAM_Angle_Y + 30 + CAM_Y_ADD;
                ui_g_Dynamic_Aim->end_x   = 1920/2 - 1920/2 *(-Gimbal_Control_Struct.Vision_Yaw_Err) / CAM_Angle_X + 30 + CAM_X_ADD;
                ui_g_Dynamic_Aim->end_y   = 1080/2 + 1080/2 *(Pos_Pitch - Gimbal_Control_Struct.Pitch_Feedback) / CAM_Angle_Y - 30 + CAM_Y_ADD; 
            }
            else    
            {
                ui_g_Dynamic_Aim->start_x = 911;
                ui_g_Dynamic_Aim->start_y = 686;
                ui_g_Dynamic_Aim->end_x = 1011;
                ui_g_Dynamic_Aim->end_y = 786;
            } 
            
            /*===| 自瞄偏置 |===*/
            String_Printf(ui_g_Dynamic_Yaw_Offset->string, "Y:%02.1f", Aim_Send_Struct.operator_yaw_offset);
            String_Printf(ui_g_Dynamic_Pitch_Offset->string, "P:%02.1f", Aim_Send_Struct.operator_pitch_offset);
            
            /*===| 更新UI |===*/     
            UI_Update();
        }
        
    }	
}

void String_Printf(char *String_Arr, const char *fmt,...)
{
    static va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)String_Arr, fmt, ap);
    va_end(ap);
}
