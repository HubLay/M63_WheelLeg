#include "robot_cmd.h"

#include "balance_chassis.h"

uint8_t qqqq = 0;

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

      //小陀螺停下或者起立的时候判断一下最好的随动方向
      if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN 
      && Balance_Chassis.Target_CMD_Data.Target_Control_Type == Chassis_Control_Type_FLLOW){
        float tmp_delta_rad = -Balance_Chassis.Motor_Yaw.Get_Now_Radian() + Reference_Rad;
        tmp_delta_rad = Normalize_Angle_Radian_PI_to_PI(tmp_delta_rad);

        if(fabsf(tmp_delta_rad) < PI / 2.0f){
          Balance_Chassis.Chassis_Forward = 1.0f;
        }
        else{
          Balance_Chassis.Chassis_Forward = -1.0f;
        }

      }

      Balance_Chassis.Set_Chassis_Control_Type(Balance_Chassis.Target_CMD_Data.Target_Control_Type);
    }
    else{   //只剩Jump_1
      
    }
  }

  if(Balance_Chassis.Switch_Chassis_Forward == 1 && Balance_Chassis.Last_Switch_Chassis_Forward == 0){
    Balance_Chassis.Chassis_Forward *= -1.0f;
  }

  //在这里更新last，保证每次switch状态都能更新
  Balance_Chassis.Last_Switch_Chassis_Forward = Balance_Chassis.Switch_Chassis_Forward;   

  if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN){
    Balance_Chassis.Chassis_Forward = 1.0f;
  }

  // 所有都正常的情况下
  float tmp_target_v = 0.0f;
  float tmp_target_omega = 0.0f;
  float tmp_target_yaw = Balance_Chassis.Get_Target_Yaw_Angle();
  float tmp_target_length = Balance_Chassis.Get_Target_Length();

  tmp_target_v = Balance_Chassis.Target_CMD_Data.Target_Velocity_X * Balance_Chassis.Chassis_Forward;

  if(Balance_Chassis.Left_Leg.Get_Air_Status() == Leg_Air || Balance_Chassis.Right_Leg.Get_Air_Status() == Leg_Air){
    tmp_target_omega = 0.0f;
    tmp_target_yaw = Balance_Chassis.Get_Yaw_Angle();                 //保持在当前角度
    tmp_target_length = Balance_Chassis.Get_Target_Length();          //维持当前目标腿长
  }
  else{

    if(fabs(tmp_target_v) < 0.5f){
      tmp_target_length += Balance_Chassis.Target_CMD_Data.Target_Delta_Length;
    }
    else{
      tmp_target_length = Balance_Chassis.Get_Target_Length();          //高速运动下不调整腿长
    }

    if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW){
      tmp_target_omega = 0.0f;
      if(Balance_Chassis.Chassis_Forward == 1.0f){
        tmp_target_yaw = Reference_Rad;     //标定云台相对底盘逆时针为正
      }
      else if(Balance_Chassis.Chassis_Forward == -1.0f){
        tmp_target_yaw = Reference_Rad - PI;     //标定云台相对底盘逆时针为正
      }
    }
    else if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN){

      tmp_target_omega = SPIN_OMEGA;

      if(fabs(SPIN_OMEGA - Balance_Chassis.Get_Target_Omega()) * 1000.0f / CMDProcess_TASK_DT > 8.0f){
        tmp_target_omega = Balance_Chassis.Get_Target_Omega() + ((SPIN_OMEGA - Balance_Chassis.Get_Target_Omega()) > 0 ? 1.0f:-1.0f) * 8.0f * CMDProcess_TASK_DT / 1000.0f;
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
    if(Balance_Chassis.Chassis_Forward == 1.0f){
      if (tmp_target_v - Balance_Chassis.Get_Target_Vx() > 0.0f)
      {
        if ((tmp_target_v - Balance_Chassis.Get_Target_Vx()) * 1000.0f / CMDProcess_TASK_DT < Acc_Max_1)
        {
          tmp_target_v = tmp_target_v;
        }
        else
        {
          tmp_target_v = Balance_Chassis.Get_Target_Vx() + Acc_Max_1 * CMDProcess_TASK_DT * 1.0f / 1000.0f;
        }
      }
      else
      {
        if ((tmp_target_v - Balance_Chassis.Get_Target_Vx()) * 1000.0f / CMDProcess_TASK_DT > -ACC_Max_2)
        {
          tmp_target_v = tmp_target_v;
        }
        else
        {
          tmp_target_v = Balance_Chassis.Get_Target_Vx() + ACC_Max_2 * CMDProcess_TASK_DT * -1.0f / 1000.0f;
        }
      }
    }
    else if(Balance_Chassis.Chassis_Forward == -1.0f){
      if (tmp_target_v - Balance_Chassis.Get_Target_Vx() > 0.0f)
      {
        if ((tmp_target_v - Balance_Chassis.Get_Target_Vx()) * 1000.0f / CMDProcess_TASK_DT < ACC_Max_2)
        {
          tmp_target_v = tmp_target_v;
        }
        else
        {
          tmp_target_v = Balance_Chassis.Get_Target_Vx() + ACC_Max_2 * CMDProcess_TASK_DT * 1.0f / 1000.0f;
        }
      }
      else
      {
        if ((tmp_target_v - Balance_Chassis.Get_Target_Vx()) * 1000.0f / CMDProcess_TASK_DT > -Acc_Max_1)
        {
          tmp_target_v = tmp_target_v;
        }
        else
        {
          tmp_target_v = Balance_Chassis.Get_Target_Vx() + Acc_Max_1 * CMDProcess_TASK_DT * -1.0f / 1000.0f;
        }
      }
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

  //Shift加速操作
  float tmp_v_max = 0;
  if(Balance_Chassis.Sprint_Status == Sprint_Status_ENABLE){
    tmp_v_max = V_MAX_2;
  }
  else{
    tmp_v_max = V_MAX;
  }

  if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN){
    Math_Constrain(&tmp_target_length, Length_MIN_SPIN, Length_MAX_SPIN);
    Math_Constrain(&tmp_target_v, -V_MAX_SPIN, V_MAX_SPIN);
  }
  else{
    Math_Constrain(&tmp_target_length, Length_MIN, Length_MAX);
    Math_Constrain(&tmp_target_v, -tmp_v_max, tmp_v_max);

    if(Balance_Chassis.Get_Target_Length() > 0.28f){
      Math_Constrain(&tmp_target_v, -1.8f, 1.8f);
    }

  }

  Balance_Chassis.Set_Jump_Enable_Flag(((Balance_Chassis.Target_CMD_Data.Complex_Flag & 0x02) != 0));

  Balance_Chassis.Set_Target_V(tmp_target_v);
  Balance_Chassis.Set_Target_Omega(tmp_target_omega);
  Balance_Chassis.Set_Target_Yaw_Angle(tmp_target_yaw);
  Balance_Chassis.Set_Target_Length(tmp_target_length);

  if(Balance_Chassis.IS_NORMAL()){
    
  }
  
}