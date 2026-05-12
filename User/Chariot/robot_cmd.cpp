#include "robot_cmd.h"

#include "balance_chassis.h"

void CMDProcessTask()
{
  if(Balance_Chassis.Emergency_Stop_Flag){                //整车急停
    Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    return;
  }

  if(Balance_Chassis.Target_CMD_Data.Target_Control_Type == Chassis_Control_Type_DISABLE){
    Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    return;
  }
  else{
    if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE){
      Balance_Chassis.Set_Reserve_Status(Reserve_Disable);
      Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_RESERVE); 
      return;
    }
    else if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_RESERVE){
      return;
    }
    else if(Balance_Chassis.IS_NORMAL()){
      Balance_Chassis.Set_Chassis_Control_Type(Balance_Chassis.Target_CMD_Data.Target_Control_Type);
    }
    else{   //只剩Jump_1
      
    }
  }

  // 所有都正常的情况下
  float tmp_target_v = 0.0f;
  float tmp_target_omega = 0.0f;
  float tmp_target_yaw = Balance_Chassis.Get_Target_Yaw_Angle();
  float tmp_target_length = Balance_Chassis.Get_Target_Length();

  tmp_target_v = Balance_Chassis.Target_CMD_Data.Target_Velocity_X;

  if(Balance_Chassis.Left_Leg.Get_Air_Status() == Leg_Air || Balance_Chassis.Right_Leg.Get_Air_Status() == Leg_Air){
    tmp_target_omega = 0.0f;
    // tmp_target_v = Balance_Chassis.Get_True_Vx();
    tmp_target_yaw = Balance_Chassis.Get_Yaw_Angle();                 //保持在当前角度
    tmp_target_length = Balance_Chassis.Get_Target_Length();          //维持当前目标腿长
  }
  else{
    // tmp_target_v = -Balance_Chassis.Target_CMD_Data.Target_Velocity_X;

    if(fabs(tmp_target_v) < 0.5f){
      tmp_target_length += Balance_Chassis.Target_CMD_Data.Target_Delta_Length;
    }
    else{
      tmp_target_length = Balance_Chassis.Get_Target_Length();          //高速运动下不调整腿长
    }

    if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW){
      tmp_target_omega = 0.0f;
      tmp_target_yaw = Reference_Rad;     //标定云台相对底盘逆时针为正
    }
    else if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN){

      //目标角速度斜坡输入
      if(fabs(SPIN_OMEGA - Balance_Chassis.Get_Target_Omega()) * 1000.0f / CMDProcess_TASK_DT > 7.0f){
        tmp_target_omega = Balance_Chassis.Get_Target_Omega() + ((SPIN_OMEGA-Balance_Chassis.Get_Target_Omega()) > 0 ? 1.0f : -1.0f) * 7.0f * CMDProcess_TASK_DT / 1000.0f;        //斜坡逼近目标转速
      }
      else{
        tmp_target_omega = SPIN_OMEGA;
      }

      tmp_target_yaw = Balance_Chassis.Get_Yaw_Angle() + tmp_target_omega * CMDProcess_TASK_DT / 1000.0f;                           //小陀螺下不输出Yaw角度这一项
    }
    else if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_UNFLLOW){
      tmp_target_omega = 0.0f;
      tmp_target_yaw += Balance_Chassis.Target_CMD_Data.Target_Delta_Yaw;         //
    }
    else{         //Chassis_Control_Type_JUMP_1
      tmp_target_omega = 0.0f;
      tmp_target_yaw = Balance_Chassis.Get_Yaw_Angle(); 
    }

    //目标速度的斜坡输入
    if(fabs(tmp_target_v - Balance_Chassis.Get_Target_Vx()) * 1000.0f / CMDProcess_TASK_DT < 2.0f){
      tmp_target_v = tmp_target_v;
    }
    else{
      //加速度过大，斜坡逼近目标
      tmp_target_v = Balance_Chassis.Get_Target_Vx() + 2.0f * CMDProcess_TASK_DT * 1.0f / 1000.0f * ((tmp_target_v - Balance_Chassis.Get_Target_Vx()) > 0 ? 1.0f : -1.0f);
    }

  }

  if (tmp_target_yaw > PI)
  {
    tmp_target_yaw -= 2.0f * PI;
  }
  else if (tmp_target_yaw < -PI)
  {
    tmp_target_yaw += 2.0f * PI;
  }

  Math_Constrain(&tmp_target_v, -V_MAX, V_MAX);
  Math_Constrain(&tmp_target_length, Length_MIN, Length_MAX);

  Balance_Chassis.Set_Target_V(tmp_target_v);
  Balance_Chassis.Set_Target_Omega(tmp_target_omega);
  Balance_Chassis.Set_Target_Yaw_Angle(tmp_target_yaw);
  Balance_Chassis.Set_Target_Length(tmp_target_length);
}