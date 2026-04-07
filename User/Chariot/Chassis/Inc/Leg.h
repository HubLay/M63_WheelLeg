#ifndef __LEG_H
#define __LEG_H

#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"              //记得检查达妙数据接收喝发送，使能是否正常
#include "my_kalman.h"

const float Poly_Coefficient[12][4] = {
    {-117.8182, 147.5706, -81.4761, -0.7362},  // a11
    {-2.4241, 3.3451, -6.8757, 0.0938},        // a12
    {-14.6635, 15.3427, -5.5542, -1.0088},     // a13
    {-19.8497, 20.9435, -8.1377, -1.5422},     // a14
    {-364.5918, 443.4160, -207.6234, 44.7239}, // a15
    {-12.7319, 17.6359, -9.7807, 3.1466},      // a16
    {136.3975, -116.1036, 18.5507, 15.8494},   // a21
    {15.2673, -15.2873, 3.3829, 1.0890},       // a22
    {-74.7571, 89.0693, -40.1957, 7.7662},     // a23
    {-111.1846, 131.4041, -58.8783, 11.3638},  // a24
    {1728.2, -1828.7, 677.0310, 77.8553},      // a25（“1.7282e+03”对应1728.2，“-1.8287e+03”对应-1828.7）
    {111.2040, -124.2318, 50.5259, -1.1936}    // a26
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
    float alpha, d_alpha;
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

inline Enum_Leg_Air_Status Class_Leg::Get_Air_Status()
{
  return Air_Status;
}

#endif