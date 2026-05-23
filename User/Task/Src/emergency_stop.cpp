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
uint8_t Err_Status = 0;
uint32_t Left_Motor_Err_Torque_Cnt = 0, Right_Motor_Err_Torque_Cnt = 0;
void Emergency_StopTask()
{
  // 这里可以添加一些紧急停止的处理逻辑，例如监测某些传感器数据或者状态，并根据情况执行紧急停止操作
  // 例如，如果检测到某个电机离线或者云台指令掉线，可以调用相应的函数来执行紧急停止

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

    //电机输出力矩不正常
    if(fabs(Balance_Chassis.Left_Leg.Get_LQR_Wheel_T()) == 4.9f){
      Left_Motor_Err_Torque_Cnt ++;
    }
    else{
      Left_Motor_Err_Torque_Cnt = 0;
    }

    if(fabs(Balance_Chassis.Right_Leg.Get_LQR_Wheel_T()) == 4.9f){
      Right_Motor_Err_Torque_Cnt ++;
    }
    else{
      Right_Motor_Err_Torque_Cnt = 0;
    }

    if(fabs(Balance_Chassis.Get_Roll_Angle()) * 57.3f > 35.0f ||
       fabs(Balance_Chassis.Get_Pitch_Angle()) * 57.3f > 35.0f ||
       fabs(Balance_Chassis.Left_Leg.Get_Theta()) > 1.2f || 
       fabs(Balance_Chassis.Right_Leg.Get_Theta()) > 1.2f)
      //  Left_Motor_Err_Torque_Cnt * EMERGENCY_STOP_TASK_DT > 500 ||
      //  Right_Motor_Err_Torque_Cnt * EMERGENCY_STOP_TASK_DT > 500)
    {
        Balance_Chassis.Emergency_Stop_Flag = 1;
    }

    if(fabs(Balance_Chassis.Get_Roll_Angle()) * 57.3f > 35.0f)  Err_Status |= 0x01;
    if(fabs(Balance_Chassis.Get_Pitch_Angle()) * 57.3f > 40.0f) Err_Status |= (1<<1);
    if(fabs(Balance_Chassis.Left_Leg.Get_Theta()) > 1.2f)       Err_Status |= (1<<2);
    if(fabs(Balance_Chassis.Right_Leg.Get_Theta()) > 1.2f)      Err_Status |= (1<<3);
    // if(Left_Motor_Err_Torque_Cnt * EMERGENCY_STOP_TASK_DT > 500) Err_Status |= (1<<4);
    // if(Right_Motor_Err_Torque_Cnt * EMERGENCY_STOP_TASK_DT > 500) Err_Status |= (1<<5);

  }

  if(Balance_Chassis.Emergency_Stop_Flag == 0){
    Err_Status = 0;
    Test_Flag = 0;
  }

}
