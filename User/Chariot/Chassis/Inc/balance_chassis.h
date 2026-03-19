#ifndef __BALANCE_CHASSIS_H
#define __BALANCE_CHASSIS_H

#include "Leg.h"

#include "kalman_filter.h"
#include "dvc_dr16.h"
#include "dvc_imu.h"

class Class_Balance_Chassis;

extern Class_Balance_Chassis Balance_Chassis;

#ifdef UNFLLOW_ENABLE

typedef struct {
  float Left_X;
  float Left_Y;
  float Right_X;
  float Right_Y;
  uint8_t Control_Type;
}CMD_Data_s;

#endif

//具体到底盘的实现

enum Enum_Chassis_Control_Type : uint8_t
{
  Chassis_Control_Type_DISABLE = 0,
  Chassis_Control_Type_FLLOW,
  Chassis_Control_Type_SPIN,
  Chassis_Control_Type_JUMP,
  Chassis_Control_Type_UNFLLOW,           //适用于无云台下的正常运动调试
  Chassis_Control_Type_RESERVE,           //倒地自启

};

enum Enum_Reserve_Status : uint8_t
{
  Reserve_Disable,                        //没有倒地
  Reserve_Status_1,                       //Pitch翻车,摆正车体
  Reserve_Status_2,                       //摆杆摆正过程
  Reserve_Complete,                       //自救完成
};

//腿组装成为底盘
class Class_Balance_Chassis{
  public:

    Class_IMU IMU;              //整车姿态
    
    Class_Leg Left_Leg;         //注意左右腿建模方向不一致
    Class_Leg Right_Leg;

    Class_PID Tp_PID;           //防劈叉PID
    Class_PID Roll_PID;
    Class_PID Turn_Angle_PID;   //转向角度环
    Class_PID Turn_Omega_PID;   //转向速度环

    Class_DR16 DR16;

    void Init();

    inline float Get_aver_v();
    inline float Get_True_Vx();
    inline float Get_True_X();
    inline float Get_Target_Vx();
    inline float Get_Target_Length();
    inline float Get_Target_Yaw_Angle();
    inline float Get_Pitch_Angle();
    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    void Set_Target_V(float __Target_Vx);
    void Set_Target_Length(float __Target_Length);
    void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    void Set_Reserve_Status(Enum_Reserve_Status __Reserve_Status);
    void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);

    void CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data);

    void TIM_Calculate_PeriodElapsedCallback();             //周期性调用的逻辑函数

    DaemonInstance *Gimbal_Daemon;

    #ifdef UNFLLOW_ENABLE
    CMD_Data_s CMD_Data;
    #endif

  protected:

    float X = 0.0f;
    float Vx = 0.0f;
    float Accel_X = 0.0f, Accel_Z = 0.0f;
    float Compensite_F0 = 0.0f;                             //转向前馈补偿力
    float Pitch_Angle = 0.0f, GyroPitch = 0.0f;             //rad
    float Roll_Angle = 0.0f, GyroRoll = 0.0f;               //rad
    float Yaw_Angle = 0.0f, GyroYaw = 0.0f;                 //angle

    float Target_X = 0.0f, Target_Vx = 0.0f;
    float Target_theta = 0.0f;
    float Target_Length = 0.16f;
    float Target_Yaw_Angle = 0.0f;
    float Target_Roll_Angle = 0.0f;
    float Target_Pitch_Angle = 0.0f;

    float Target_Omega = 0.0f;

    KalmanFilter_t V_EstimateKF;

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Reserve_Status Reserve_Status = Reserve_Disable;

  private:
    void LengthControl();         // 腿长控制，Roll保持机体水平
    void SynthesizeMotion();      // 转向和抗劈叉
    void SpeedUpdata();
    void ParamUpdata();
    void SpeedEstimate();
    void Reserve_FSM();           //倒地自起状态机

    void NormalOutput();
    void ReserveOutput();
    void Chassis_Disable();

    void V_EstimateKF_Init();

    //相关中间变量小写
    float aver_v;
    float theta_error;
    uint32_t Chassis_Stable_Count = 0;
};

inline float Class_Balance_Chassis::Get_aver_v()
{
  return aver_v;
}

inline float Class_Balance_Chassis::Get_True_Vx()
{
  return Vx;
}

inline float Class_Balance_Chassis::Get_True_X()
{
  return X;
}

inline float Class_Balance_Chassis::Get_Target_Vx()
{
  return Target_Vx;
}

inline float Class_Balance_Chassis::Get_Target_Length()
{
  return Target_Length;
}

inline float Class_Balance_Chassis::Get_Target_Yaw_Angle()
{
  return Target_Yaw_Angle;
}

inline float Class_Balance_Chassis::Get_Pitch_Angle()
{
  return Pitch_Angle;
}

inline Enum_Chassis_Control_Type Class_Balance_Chassis::Get_Chassis_Control_Type()
{
  return Chassis_Control_Type;
}


#endif