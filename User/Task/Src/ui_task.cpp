#include "ui_task.h"
#include "balance_chassis.h"
#include "dvc_GraphicsSendTask.h"

uint8_t Flag111 = 0;
void UITask(){

  GraphicSendtask();

  JudgeReceiveData.robot_id = Balance_Chassis.Referee.Get_ID();
  JudgeReceiveData.Chassis_Control_Type = Balance_Chassis.Get_Chassis_Control_Type();
  JudgeReceiveData.Pitch_Angle = Balance_Chassis.Gimbal_Pitch_Angle; // pitch角度
  JudgeReceiveData.Gimbal_Control_Type = Balance_Chassis.Gimbal_Control_Type;
  JudgeReceiveData.Minipc_Mode = Balance_Chassis.MiniPC_Aim;
  JudgeReceiveData.Leg_Length = (Balance_Chassis.Left_Leg.Get_Length() + Balance_Chassis.Right_Leg.Get_Length()) / 2.0f;
  JudgeReceiveData.Supercap_Voltage = Balance_Chassis.Supercap.Get_Supercap_Proportion(); // 超电压
  JudgeReceiveData.Fric_Status = Balance_Chassis.Fric_Status;
  JudgeReceiveData.Chassis_Gimbal_Diff = Balance_Chassis.Motor_Yaw.Get_Now_Angle(); // 底盘角度

  if (Balance_Chassis.Referee_UI_Refresh_Status == Referee_UI_Refresh_Status_ENABLE &&
    Balance_Chassis.Last_Referee_UI_Refresh_Status == Referee_UI_Refresh_Status_DISABLE && Init_Cnt == 0){
    Init_Cnt = 30;
  }

  Balance_Chassis.Last_Referee_UI_Refresh_Status = Balance_Chassis.Referee_UI_Refresh_Status;

  // if(Init_Cnt == 0){
  //   Init_Cnt = 255;
  // }

    
}