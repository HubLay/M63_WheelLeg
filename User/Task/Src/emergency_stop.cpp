#include "emergency_stop.h"
#include "balance_chassis.h"

/**
 * @file emergency_stop.cpp
 * @author cjw
 * @brief 紧急停止相关函数
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
**/
uint8_t Test_Flag = 0;
void Emergency_StopTask()
{
  // 这里可以添加一些紧急停止的处理逻辑，例如监测某些传感器数据或者状态，并根据情况执行紧急停止操作
  // 例如，如果检测到某个电机离线或者云台指令掉线，可以调用相应的函数来执行紧急停止
  Test_Flag = 0;

  if(Balance_Chassis.Target_CMD_Data.Target_Control_Type == Chassis_Control_Type_DISABLE){
    Balance_Chassis.Emergency_Stop_Flag = 0;     //目标状态为失能的时候允许解除急停
  }

  // 任意一个电机离线或者云台指令掉线都进入急停
  if (!Get_Device_Status(Balance_Chassis.Left_Leg.Front_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Left_Leg.Back_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Left_Leg.Wheel_Motor.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Front_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Back_Joint.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Right_Leg.Wheel_Motor.daemon_instance) ||
      !Get_Device_Status(Balance_Chassis.Gimbal_Daemon))
  {
    Balance_Chassis.Emergency_Stop_Flag = 1;

    if (!Get_Device_Status(Balance_Chassis.Gimbal_Daemon))
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
  }

  if(Balance_Chassis.IS_NORMAL()){      //机体正常运动下检测机体是否正常
    if(fabs(Balance_Chassis.Get_Roll_Angle()) * 57.3f > 35.0f ||
       fabs(Balance_Chassis.Get_Pitch_Angle()) * 57.3f > 50.0f ||
       fabs(Balance_Chassis.Left_Leg.Get_Theta()) > 1.0f || 
       fabs(Balance_Chassis.Right_Leg.Get_Theta()) > 1.0f)
    {
        Balance_Chassis.Emergency_Stop_Flag = 1;
    }
  }
}
