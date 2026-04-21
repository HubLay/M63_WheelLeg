#ifndef __LEG_H
#define __LEG_H

#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"              //记得检查达妙数据接收喝发送，使能是否正常
#include "my_kalman.h"

const float Poly_Coefficient[12][4] = {
    {-102.8302, 129.9481, -75.4332, -2.4717},  // a11
    {-1.7440, 2.1554, -6.7381, -0.0226},       // a12
    {-0.9324, 0.8300, -0.2028, -2.4342},       // a13
    {-0.0635, 0.1230, -0.6927, -2.9844},       // a14
    {-541.8021, 607.9104, -254.0789, 34.9168}, // a15
    {-28.9754, 34.0174, -15.3526, 3.4914},     // a16
    {32.8254, -36.4382, 13.2004, 3.3826},      // a21
    {0.5478, -0.3824, -1.6099, 0.1500},        // a22
    {-23.0344, 25.5361, -10.3895, 1.1512},     // a23
    {-27.0292, 29.9520, -12.3588, 1.3272},     // a24
    {159.4684, -158.6926, 52.9696, 217.3252},  // a25（无科学计数法，直接原值填入）
    {32.8830, -36.7975, 15.4845, 5.3032}       // a26
};

enum Enum_Leg_Air_Status{
  Leg_UnAir = 0,
  Leg_Air,
};

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
    float X = 0.0f, Vx = 0.0f;             //m   m/s
    float theta = 0.0f, d_theta = 0.0f;
    float L0 = 0.0f, d_L0 = 0.0f;           //m
    float FN = 0.0f, Accel_Z = 0.0f;
    float d_alpha_true, d_theta_true, d_L0_true;

    float Target_X = 0.0f;
    float Target_Vx = 0.0f;
    float Target_theta = 0.0f;
    float Target_dtheta = 0.0f;
    float Target_Pitch = 0.0f;
    float Target_L0 = 0.14f;

    float Tp, F0, Wheel_T, T_Front, T_Back;
    float LQR_K[12], LQR_Tp[7], LQR_Wheel_T[7];             //第七位是lqr计算输出之和

    float Wheel_Speed  = 0.0f;

    //相关中间变量小写
    float w_ed = 0.0f, leg_v = 0.0f, leg_v_true = 0.0f;                    //没有进行融合的腿部速度，修正后的机体轮速
    float pitch_offset = 0.0f;
    float theta_offset = 0.0f, d_theta_offset = 0.0f;

  private:
    my_kalman Wheel_Speed_Kalman, leg_v_Kalman;
    my_kalman d_L0_Kalman, d_alpha_Kalman, d_theta_Kalman;

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

inline float Class_Leg::Get_LQR_Tp()
{
  return (LQR_Tp[6]);
}

inline float Class_Leg::Get_LQR_Wheel_T()
{
  return (LQR_Wheel_T[6]);
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

#endif