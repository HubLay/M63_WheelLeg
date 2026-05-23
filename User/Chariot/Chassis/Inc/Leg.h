#ifndef __LEG_H
#define __LEG_H

#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"              //记得检查达妙数据接收喝发送，使能是否正常
#include "my_kalman.h"

enum Enum_Leg_Air_Status{
  Leg_UnAir = 0,
  Leg_Air,
};

typedef enum{
  Left,
  Right
}Enum_Leg;

extern void PrintfTask();

class Class_Leg{

  friend void PrintfTask();
  friend class Class_Balance_Chassis;

  public:

    Class_PID Length_PID;
    Class_PID dLength_PID;

    Class_DJI_Motor_C620 Wheel_Motor;
    Class_DM_Motor_8009P Front_Joint;               //自身坐标系下的前关节电机
    Class_DM_Motor_8009P Back_Joint;                //自身坐标系下的后关节电机 
  
    void Init();
    void VMC_Calc();
    void LQR_Calc();
    void Leg_V_Calc();
    void Length_Calc();
    void ForceSlove();
    void ParamUpdata();                             //更新VMC相关的参数
    void VMCProject();                              //VMC参数的映射

    float Get_Theta();

    template <Enum_Leg T>
    void Torque_Output();

    void Disable();

    inline float Get_LQR_Tp();
    inline float Get_LQR_Wheel_T();
    inline float Get_Length();
    inline float Get_Target_L0();
    inline Enum_Leg_Air_Status Get_Air_Status();

  protected:

    //重要的变量
    float Pitch = 0.0f;
    float GyroPitch = 0.0f;
    float theta = 0.0f, d_theta = 0.0f;
    float L0 = 0.0f, d_L0 = 0.0f;           //m
    float FN = 0.0f, Accel_Z = 0.0f;
    float d_alpha_true, d_theta_true, d_L0_true;

    float Target_L0 = 0.16f;

    float Tp, F0, Wheel_T, T_Front, T_Back;

    float Wheel_Speed  = 0.0f;

    uint8_t Status = 0;
    uint32_t Status_Count = 0;

    //相关中间变量小写
    float w_ed = 0.0f, leg_v = 0.0f, leg_v_true = 0.0f;                    //没有进行融合的腿部速度，修正后的机体轮速
    float pitch_offset = 0.0f;
    float theta_offset = 0.0f, d_theta_offset = 0.0f;

    my_kalman Wheel_Speed_Kalman;
    my_kalman leg_v_Kalman;
    my_kalman d_L0_Kalman, d_alpha_Kalman, d_theta_Kalman;
    my_kalman FN_KF;

    //vmc 相关变量
    float l1, l2, l3, l4, l5;     //m
    float XB, YB, XD, YD, XC, YC;
    float d_XC, d_YC;
    float alpha, d_alpha;       //-PI -- PI
    float lBD;            //BD两点的距离
    float phi1, phi2, phi3, phi4, phi0;
    float d_phi1, d_phi4, d_phi0;
    float A0, B0, C0; 
    float dd_theta, dd_L0;
    float j11, j12, j21, j22;         //雅可比矩阵的关节系数
    float last_d_theta, last_d_L0;

    float dt;                   //任务的运行周期 

    uint8_t first_flag = 0;

    Enum_Leg_Air_Status Air_Status = Leg_UnAir;         //离地标志位

    float LQR_K_calc(const float *coe,float len);

};

inline float Class_Leg::Get_LQR_Wheel_T()
{
  return (Wheel_T);
}

inline float Class_Leg::Get_Length()
{
  return (L0);
}

inline float Class_Leg::Get_Target_L0()
{
  return Target_L0;
}

inline Enum_Leg_Air_Status Class_Leg::Get_Air_Status()
{
  return Air_Status;
}

template <Enum_Leg T>
void Class_Leg::Torque_Output()
{
  Front_Joint.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_TORQUE);
  Back_Joint.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_TORQUE);
  Wheel_Motor.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

  if(T == Left){
    Front_Joint.Set_Output_Torque(T_Front);
    Back_Joint.Set_Output_Torque(T_Back);
  }
  else if(T == Right){        //输出轴方向不一致，需要加负号
    Front_Joint.Set_Output_Torque(-T_Front);
    Back_Joint.Set_Output_Torque(-T_Back);
  }
  else{
    Front_Joint.Set_Output_Torque(0.0f);
    Back_Joint.Set_Output_Torque(0.0f);
  }
  
  Wheel_Motor.Set_Target_Torque(Wheel_T);
  // Front_Joint.Set_Output_Torque(0.0f);
  // Back_Joint.Set_Output_Torque(0.0f);
  // Wheel_Motor.Set_Target_Torque(0.0f);

  Front_Joint.TIM_Process_PeriodElapsedCallback();
  Back_Joint.TIM_Process_PeriodElapsedCallback();
  Wheel_Motor.TIM_PID_PeriodElapsedCallback();              //实际不是PID，是一个线性映射
}


#endif