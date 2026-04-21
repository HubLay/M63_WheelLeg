#include "robot_cmd.h"

#include "balance_chassis.h"

uint8_t Test_Flag = 0;

void CMDProcessTask()
{
  Test_Flag = 0;
  // 任意一个电机离线或者云台指令掉线都进入急停
  if (!Get_Device_Status(Balance_Chassis.Left_Leg.Front_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Left_Leg.Back_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Left_Leg.Wheel_Motor.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Front_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Back_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Wheel_Motor.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.DR16.daemon_instance))
  {
    Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    if (!Get_Device_Status(Balance_Chassis.DR16.daemon_instance))
      Test_Flag |= (1 << 0);

    if (!Get_Device_Status(Balance_Chassis.Left_Leg.Front_Joint.daemon_instance))
      Test_Flag |= (1 << 1);

    if (!Get_Device_Status(Balance_Chassis.Left_Leg.Back_Joint.daemon_instance))
      Test_Flag |= (1 << 2);

    if (!Get_Device_Status(Balance_Chassis.Left_Leg.Wheel_Motor.daemon_instance))
      Test_Flag |= (1 << 3);

    if (!Get_Device_Status(Balance_Chassis.Right_Leg.Front_Joint.daemon_instance))
      Test_Flag |= (1 << 4);

    if (!Get_Device_Status(Balance_Chassis.Right_Leg.Back_Joint.daemon_instance))
      Test_Flag |= (1 << 5);

    if (!Get_Device_Status(Balance_Chassis.Right_Leg.Wheel_Motor.daemon_instance))
      Test_Flag |= (1 << 6);

    return;
  }

  // 理论上这里应该使用订阅转发机制，保证同一层级之间的模块可以转发消息
  Balance_Chassis.CMD_Data.Left_X = Balance_Chassis.DR16.Get_Left_X();
  Balance_Chassis.CMD_Data.Left_Y = Balance_Chassis.DR16.Get_Left_Y();
  Balance_Chassis.CMD_Data.Right_X = Balance_Chassis.DR16.Get_Right_X();
  Balance_Chassis.CMD_Data.Right_Y = Balance_Chassis.DR16.Get_Right_Y();

  // 失能模式下，所有电机正常的话
  if (Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
  {
    // 失能模式下恢复正常，先进入自救
    Balance_Chassis.Set_Reserve_Status(Reserve_Disable);
    Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_RESERVE);

    return;
  }

  float tmp_target_yaw = Balance_Chassis.Get_Target_Yaw_Angle();

  if(Balance_Chassis.IS_NORMAL()){
    if(Balance_Chassis.DR16.Get_Left_Switch() == DR16_Switch_Status_UP){
      Balance_Chassis.Set_Spin_Omega(8.0f);
      Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
    }
    else if(Balance_Chassis.DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE){
      if(Balance_Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN){
        tmp_target_yaw = Balance_Chassis.Get_Yaw_Angle();
      }
      Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_UNFLLOW);
    }
    else if(Balance_Chassis.DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN){
      Balance_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_UNFLLOW);
    }
  }

  // 所有都正常的情况下
  float tmp_target_v = 0.0f;
  float tmp_target_length = Balance_Chassis.Get_Target_Length();

  tmp_target_v = Balance_Chassis.CMD_Data.Left_Y * V_MAX;
  tmp_target_yaw -= Balance_Chassis.CMD_Data.Right_X * Yaw_Angle_Resolution;
  tmp_target_length += Balance_Chassis.CMD_Data.Right_Y * Length_Angle_Resolution;

  if (tmp_target_yaw > 180.0f)
  {
    tmp_target_yaw -= 360.0f;
  }
  else if (tmp_target_yaw < -180.0f)
  {
    tmp_target_yaw += 360.0f;
  }

  Math_Constrain(&tmp_target_v, -V_MAX, V_MAX);
  Math_Constrain(&tmp_target_length, Length_MIN, Length_MAX);

  Balance_Chassis.Set_Target_V(tmp_target_v);
  Balance_Chassis.Set_Target_Yaw_Angle(tmp_target_yaw);
  Balance_Chassis.Set_Target_Length(tmp_target_length);
}