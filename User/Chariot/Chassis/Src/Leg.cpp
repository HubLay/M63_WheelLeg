#include "Leg.h"
#include "config.h"
#include "drv_can.h"

void Class_Leg::Init()
{
  // 左上第一条边为l1，逆时针编号
  l5 = 0.0f; // AE长度 m
  l1 = 0.216f;
  l2 = 0.258f;
  l3 = 0.258f;
  l4 = 0.216f;

  dt = ROBOT_TASK_DT;

  Front_Joint.TIM_Process_PeriodElapsedCallback();                //处理一次数据防止上电时刻0的数组线性映射到力矩不为0
  Back_Joint.TIM_Process_PeriodElapsedCallback();

  Length_PID.Init(25.0f, 0.0f, 0.08f, 0.0f, 0.0f, 1.5f);
  dLength_PID.Init(110.0f, 0.0f, 0.0f, 0.0f, 0.0f, 200.0f);         //腿太硬了回没有缓冲，容易进入离地

  kalman_Init(&d_L0_Kalman, 0.99f, 0.01f, 0.0f, 1.0f);
  kalman_Init(&d_alpha_Kalman, 0.99f, 0.01f, 0.0f, 1.0f);
  kalman_Init(&d_theta_Kalman, 0.99f, 0.01f, 0.0f, 1.0f);
  kalman_Init(&Wheel_Speed_Kalman, 0.99f, 0.002f, 0.0f, 1.0f);
  kalman_Init(&leg_v_Kalman, 0.99f, 0.01f, 0.0f, 1.0f);
  kalman_Init(&FN_KF, 0.99f,0.0005f,0.0f,1.0f);
}

void Class_Leg::VMC_Calc()
{
  // 如果定义车体符合右手系，这里VMC定义的应该是左腿x朝前，右腿x朝后
  // 两边腿的VMC X轴方向不一致 导致角度的正方向什么也不一样 面朝外侧逆时针方向为正
  // 有关x方向的变量应该刚好是倒过来的
  YD = l4 * arm_sin_f32(phi4);      // D的y坐标
  YB = l1 * arm_sin_f32(phi1);      // B的y坐标
  XD = l5 + l4 * arm_cos_f32(phi4); // D的x坐标
  XB = l1 * arm_cos_f32(phi1);      // B的x坐标

  lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));

  A0 = 2.0f * l2 * (XD - XB);
  B0 = 2.0f * l2 * (YD - YB);
  C0 = l2 * l2 + lBD * lBD - l3 * l3;
  phi2 = 2.0f * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
  phi3 = atan2f(YB - YD + l2 * arm_sin_f32(phi2), XB - XD + l2 * arm_cos_f32(phi2));
  // C点直角坐标
  XC = l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(phi2);
  YC = l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(phi2);
  // C点极坐标
  L0 = sqrt((XC - l5 / 2.0f) * (XC - l5 / 2.0f) + YC * YC); // 虚拟杆长L0
  phi0 = atan2f(YC, (XC - l5 / 2.0f));                      // phi0用于计算lqr需要的theta

  // 等效杆长与机体Y轴夹角
  alpha = phi0 - PI / 2.0f;     //左右腿x正方向不同，但公式是一样的，实际上左右腿的角度在相同姿态下会因为这个差一个负号

  if (alpha > PI)               //alpha的限幅
    alpha -= 2.0f * PI;
  else if (alpha < -PI)
    alpha += 2.0f * PI;

  // 关注一下theta的计算
  // 两边腿的VMC X轴方向不一致，左腿超前右腿朝后  导致角度的正方向什么也不一样   面朝外侧逆时针方向为正
  theta = phi0 - Pitch - PI / 2.0f; // 得到状态变量1

  if (theta > PI)
    theta -= 2.0f * PI;
  else if (theta < -PI)
    theta += 2.0f * PI;

  // 计算雅可比矩阵
  j11 = (l1 * arm_sin_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / arm_sin_f32(phi3 - phi2);
  j12 = (l1 * arm_cos_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / (L0 * arm_sin_f32(phi3 - phi2));
  j21 = (l4 * arm_sin_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / arm_sin_f32(phi3 - phi2);
  j22 = (l4 * arm_cos_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / (L0 * arm_sin_f32(phi3 - phi2));

  // d_phi0计算应改为雅可比矩阵求解
  d_L0 = j11 * d_phi1 + j21 * d_phi4;
  d_phi0 = j12 * d_phi1 + j22 * d_phi4;

  if(first_flag == 0){
    first_flag   = 1;
    last_d_L0    = d_L0;          //可以试试不加这个处理
    last_d_theta = d_theta;
  }

  //计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  d_alpha = d_phi0;
  d_theta = d_phi0 - GyroPitch; // 得到状态变量2

  dd_L0 = (d_L0 - last_d_L0) / dt; // 腿长L0的二阶导数
  last_d_L0 = d_L0;

  dd_theta = (d_theta - last_d_theta) / dt;
  last_d_theta = d_theta;

  Kalman_PeriodElapsedCallback(&d_L0_Kalman, d_L0);
  Kalman_PeriodElapsedCallback(&d_alpha_Kalman, d_alpha);
  Kalman_PeriodElapsedCallback(&d_theta_Kalman, d_theta);

  d_L0_true    = Kalman_Get_Out(d_L0_Kalman);
  d_alpha_true = Kalman_Get_Out(d_alpha_Kalman);
  d_theta_true = Kalman_Get_Out(d_theta_Kalman);
}

void Class_Leg::LQR_Calc()
{

}

void Class_Leg::Leg_V_Calc()
{
  //HK建模，Pitch方向和哈工程相反了，所以直接在Pitch上加一个负号就可以
  w_ed = Wheel_Speed + d_alpha + GyroPitch;           
  leg_v = w_ed * Wheel_Diameter + L0 * d_theta * arm_cos_f32(theta) + d_L0_true * arm_sin_f32(theta);

  //速度滤波                  
  Kalman_PeriodElapsedCallback(&leg_v_Kalman, leg_v);
  leg_v_true = Kalman_Get_Out(leg_v_Kalman);
}

void Class_Leg::Length_Calc()
{
  Length_PID.Set_Now(L0);
  dLength_PID.Set_Now(d_L0_true);
  
  Length_PID.Set_Target(Target_L0);
  Length_PID.TIM_Adjust_PeriodElapsedCallback();
  dLength_PID.Set_Target(Length_PID.Get_Out());
  dLength_PID.TIM_Adjust_PeriodElapsedCallback();
}

/**
 * @brief 支持力解算
 */
void Class_Leg::ForceSlove()
{
  // static uint8_t Status = 0;
  static float Pre_FN = 0.0f;

  //腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  float tmp_FN = F0*arm_cos_f32(theta) + Tp*arm_sin_f32(theta)/L0 + 0.588f * (9.81f + Accel_Z);

  FN = tmp_FN;//0.5f * tmp_FN + 0.5f * Pre_FN;

  Pre_FN = FN;

  Kalman_PeriodElapsedCallback(&FN_KF, FN);

  switch(Status){
    case(0):                        //正常不离地状态
    {
      Air_Status = Leg_UnAir;
      if(FN < 30){
        Status = 2;                 //可能离地状态
        Status_Count = 0;
      }

      Status_Count += ROBOT_TASK_DT;

      break;
    }
    case(1):                      //疑似离地状态
    {
      Air_Status = Leg_UnAir;
      if(FN < 30){
        Status = 2;                 //真正离地
        Status_Count = 0;
      }

      if (FN > 100 || Status_Count > 100)              //小于50后的100ms内没有小于30，认为是误判了
      {
        Status = 0;
        Status_Count = 0;
      }

      Status_Count += ROBOT_TASK_DT;

      break;
    }
    case(2):                        //真正离地状态
    {
      Air_Status = Leg_Air;
      if(FN > 100){
        Status = 3;                 //切到可能落地状态
        Status_Count = 0;
      }

      Status_Count += ROBOT_TASK_DT;

      break;
    }
    case(3):
    {
      Air_Status = Leg_UnAir;
      if(Status_Count < 50){               //落地后的一段时间内存在力的波动，疑似落地的100ms内都不进行检测
        
      }
      else{
        //100ms后如果还存在小力的情况就是误判了
        if(FN < 30){
          Status = 2;                       //力太小了认为80是误判，切回离地
          Status_Count = 0;
        }
        else if(FN > 100 || Status_Count > 1000){       //大力或者时间过长
          Status = 0;
          Status_Count = 0;
        }
      }

      Status_Count += ROBOT_TASK_DT;

      break;
    }
  }

  // if(FN < 10.0f){                 //两个参数和逻辑有待加强
  //   Air_Status = Leg_Air;
  // }

  // if(FN > 80.0f){
  //   Air_Status = Leg_UnAir;
  // }

  Air_Status = Leg_UnAir;

}

void Class_Leg::ParamUpdata()
{

}

void Class_Leg::VMCProject()
{
  T_Back  = j11 * F0 + j12 * Tp;
  T_Front = j21 * F0 + j22 * Tp;

  Math_Constrain(&T_Front, -JOINT_MAX_TORQUE, JOINT_MAX_TORQUE);
  Math_Constrain(&T_Back, -JOINT_MAX_TORQUE, JOINT_MAX_TORQUE);

}

float Class_Leg::Get_Theta()
{
  return theta;
}

void Class_Leg::Disable()
{
  Tp = 0.0f;
  F0 = 0.0f;
  T_Back  = 0.0f;
  T_Front = 0.0f;
  Wheel_T = 0.0f;

  Target_L0 = 0.16f;

  Air_Status = Leg_UnAir;

  Status = 0;
  Status_Count = 0;

  Length_PID.Set_Integral_Error(0.0f);
  dLength_PID.Set_Integral_Error(0.0f);
}

float Class_Leg::LQR_K_calc(const float *coe, float len)
{
  return (coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3]);
}
