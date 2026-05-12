#include "balance_chassis.h"

#include "user_lib.h"
#include "drv_math.h"
#include "cantransmit_msg.h"

#define RAD_TO_ANGLE (180.0f / 3.14159f)

// K矩阵拟合系数 K_Fit_Coefficients[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
float K_Fit_Coefficients[40][6] = {
    {    -44.7449f,      83.0465f,      23.5788f,     -52.6408f,      -145.66f,      111.942f},  // K[0][0]
    {    -43.4401f,      93.7871f,      18.5033f,     -55.9121f,      -153.92f,      109.328f},  // K[0][1]
    {     -2.2144f,    -0.590354f,     -21.4238f,      3.31819f,     -11.3079f,      31.5807f},  // K[0][2]
    {   -0.497581f,      1.75651f,     -9.34315f,     -1.85265f,     -5.00304f,        12.77f},  // K[0][3]
    {    -8.81871f,      26.6869f,      52.2256f,     -23.7558f,      22.0716f,     -45.3951f},  // K[0][4]
    {   -0.412043f,      1.00717f,      3.31747f,    -0.291058f,      5.11934f,     -4.15519f},  // K[0][5]
    {    -61.0335f,      24.7508f,      58.3257f,     -8.12879f,     -74.6519f,      4.01228f},  // K[0][6]
    {    -4.11377f,      1.28282f,      5.19048f,    -0.186851f,     -5.06408f,     -5.21313f},  // K[0][7]
    {     32.0326f,     -98.3406f,      498.736f,      137.261f,     -118.744f,     -511.028f},  // K[0][8]
    {    -4.14535f,      8.57038f,      49.5709f,     0.338818f,     -39.9634f,     -34.2765f},  // K[0][9]
    {    -44.7449f,      23.5788f,      83.0465f,      111.942f,      -145.66f,     -52.6408f},  // K[1][0]
    {    -43.4401f,      18.5033f,      93.7871f,      109.328f,      -153.92f,     -55.9121f},  // K[1][1]
    {      2.2144f,      21.4238f,     0.590354f,     -31.5807f,      11.3079f,     -3.31819f},  // K[1][2]
    {    0.497581f,      9.34315f,     -1.75651f,       -12.77f,      5.00304f,      1.85265f},  // K[1][3]
    {    -61.0335f,      58.3257f,      24.7508f,      4.01228f,     -74.6519f,     -8.12879f},  // K[1][4]
    {    -4.11377f,      5.19048f,      1.28282f,     -5.21313f,     -5.06408f,    -0.186851f},  // K[1][5]
    {    -8.81871f,      52.2256f,      26.6869f,     -45.3951f,      22.0716f,     -23.7558f},  // K[1][6]
    {   -0.412043f,      3.31747f,      1.00717f,     -4.15519f,      5.11934f,    -0.291058f},  // K[1][7]
    {     32.0326f,      498.736f,     -98.3406f,     -511.028f,     -118.744f,      137.261f},  // K[1][8]
    {    -4.14535f,      49.5709f,      8.57038f,     -34.2765f,     -39.9634f,     0.338818f},  // K[1][9]
    {      1.5708f,     -28.4458f,      65.5473f,      13.6783f,      21.3859f,     -80.5686f},  // K[2][0]
    {      2.5827f,     -26.0126f,      57.1145f,      12.1334f,      19.9444f,     -69.3755f},  // K[2][1]
    {    -2.04562f,     -5.17052f,      8.79167f,      7.85765f,     -4.16703f,     -4.98122f},  // K[2][2]
    {    -0.52838f,     -2.43403f,      3.04645f,      3.71641f,     -2.28742f,    -0.888392f},  // K[2][3]
    {     2.60946f,      13.6202f,      2.00759f,     -20.6908f,      13.0971f,     -14.6459f},  // K[2][4]
    {    0.161577f,     0.827147f,    0.0153682f,    -0.239261f,     0.221777f,    -0.783744f},  // K[2][5]
    {     6.47369f,     -18.1457f,      66.9148f,      23.1999f,     -17.7973f,     -49.2663f},  // K[2][6]
    {    0.459614f,     -1.07074f,      3.84056f,      1.71955f,     -2.17206f,    -0.130101f},  // K[2][7]
    {     33.9836f,     -56.0609f,     -25.9569f,      44.9086f,      47.8081f,     -27.6183f},  // K[2][8]
    {     4.84801f,     -12.6003f,     0.201247f,      10.3361f,      9.76675f,     -8.27195f},  // K[2][9]
    {      1.5708f,      65.5473f,     -28.4458f,     -80.5686f,      21.3859f,      13.6783f},  // K[3][0]
    {      2.5827f,      57.1145f,     -26.0126f,     -69.3755f,      19.9444f,      12.1334f},  // K[3][1]
    {     2.04562f,     -8.79167f,      5.17052f,      4.98122f,      4.16703f,     -7.85765f},  // K[3][2]
    {     0.52838f,     -3.04645f,      2.43403f,     0.888392f,      2.28742f,     -3.71641f},  // K[3][3]
    {     6.47369f,      66.9148f,     -18.1457f,     -49.2663f,     -17.7973f,      23.1999f},  // K[3][4]
    {    0.459614f,      3.84056f,     -1.07074f,    -0.130101f,     -2.17206f,      1.71955f},  // K[3][5]
    {     2.60946f,      2.00759f,      13.6202f,     -14.6459f,      13.0971f,     -20.6908f},  // K[3][6]
    {    0.161577f,    0.0153682f,     0.827147f,    -0.783744f,     0.221777f,    -0.239261f},  // K[3][7]
    {     33.9836f,     -25.9569f,     -56.0609f,     -27.6183f,      47.8081f,      44.9086f},  // K[3][8]
    {     4.84801f,     0.201247f,     -12.6003f,     -8.27195f,      9.76675f,      10.3361f}   // K[3][9]
};

// 平衡点偏移拟合系数 Offset_Fit_Coefficients[3][6]
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq

float Offset_Fit_Coefficients[3][6] = {
    {     0.18946f,    -0.746021f, -8.24177e-16f,      0.83315f,  1.29131e-15f,  7.07297e-16f},  // theta_l_eq
    {     0.18946f, -1.15491e-15f,    -0.746021f,  1.47241e-15f,  1.23133e-15f,      0.83315f},  // theta_r_eq
    {       -0.02f,   1.1471e-16f,  7.03616e-17f, -1.11679e-16f,  -1.4005e-16f,  -1.0636e-17f}   // theta_b_eq
};

float vaEstimateKF_F[4] = {1.0f, 0.002f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.2f, 0.0f, 
                           0.0f, 200.0f};    
// Q矩阵初始值     Q11减小更信任用加速度估计出来的速度，Q22增大，对加速度的滤波效果减小
//R11 增大R11减小对轮速的信任度，但是要足够大，太大了会造成转向结束震荡（因为转向时速度实际上应该更信任轮子，实际上是速度滞后太大了不收敛到真实的速度
//在不打滑的前提下，轮速应该是更准确的，所以在没问题的前提下应该更信任轮子
float vaEstimateKF_R[4] = {2000.0f, 0.0f, 
                            0.0f,  0.005f}; 	//v a观测噪声
													 
float vaEstimateKF_H[4] = {1.0f, 0.0f,
                            0.0f, 1.0f};	// 设置矩阵H为常量

Class_Balance_Chassis Balance_Chassis;

extern void Gimbal_Offline_CallbackFunction(void *Param);
extern uint8_t DM_Motor_CAN_Message_Enter[8];

void Class_Balance_Chassis::Init()
{
  IMU.Init();

  DR16.Init(&huart5,&huart1);

  V_EstimateKF_Init();            //卡尔曼速度观测初始化

  Motor_Yaw.Init(&hfdcan3, DJI_Motor_ID_0x205);

  //使用香港大学的建模，右腿前电机ID为A2
  Left_Leg.Wheel_Motor.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);
  Left_Leg.Front_Joint.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Left_Leg.Back_Joint.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Right_Leg.Wheel_Motor.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);         //4.925 = 20 * 0.3 * 15.765 / (3591/187)
  Right_Leg.Front_Joint.Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Right_Leg.Back_Joint.Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Left_Leg.Init();
  Right_Leg.Init();

  CanTransmit_TaskRegister(1, 0xF1, CAN1_0xxf1_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(2, 0xF2, CAN1_0xxf2_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(3, 0xF1, CAN2_0xxf1_Tx_Data, &hfdcan2);
  CanTransmit_TaskRegister(4, 0xF2, CAN2_0xxf2_Tx_Data, &hfdcan2);

  Daemon_Init_Config_s daemon_init_config_instance = Get_DaemonInitConfig_s(50, NULL, Gimbal_Offline_CallbackFunction);
  Gimbal_Daemon = DaemonRegister(daemon_init_config_instance);

  TD_Init(&Target_Vx_Td, 15, 1, 0.002);

  Tp_PID.Init(80.0f, 0.0f, 2.0f, 0.0f, 0.0f, 30.0f);
  Roll_PID.Init(600.0f, 0.0f, 0.0f,0.0f,0.0f,150.0f);
  Turn_Angle_PID.Init(0.15f, 0.0f, 0.0f);
  Turn_Omega_PID.Init(0.8f, 0.0f, 0.0f);

  //Tp_PID.Set_D_Extern_Status(PID_D_Extern_ENABLE);
  Roll_PID.Set_D_Extern_Status(PID_D_Extern_ENABLE);
  //Turn可以考虑开不开
}

void Class_Balance_Chassis::Set_Target_V(float __Target_Vx){
  Target_Vx = __Target_Vx;
}

void Class_Balance_Chassis::Set_Target_Length(float __Target_Length)
{
  Target_Length = __Target_Length;
}

void Class_Balance_Chassis::Set_Target_Omega(float __Target_Omega)
{
  Target_Omega = __Target_Omega;
}

void Class_Balance_Chassis::Set_Spin_Omega(float __Spin_Omega)
{
  Spin_Omega = __Spin_Omega;
}

void Class_Balance_Chassis::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
  Target_Yaw_Angle = __Target_Yaw_Angle;
}

void Class_Balance_Chassis::Set_Reserve_Status(Enum_Reserve_Status __Reserve_Status)
{
  Reserve_Status = __Reserve_Status;
}

void Class_Balance_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
  Chassis_Control_Type = __Chassis_Control_Type;
}

void Class_Balance_Chassis::Reset_Chassis_Stable_Count()
{
  Chassis_Stable_Count = 0;
}

void Class_Balance_Chassis::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{
  DaemonReload(Gimbal_Daemon);

  #ifdef UNFLLOW_ENABLE
  uint8_t control_type;
  int8_t tmp_dr16_left_x, tmp_dr16_left_y, tmp_dr16_right_x, tmp_dr16_right_y, tmp_sum;

  memcpy(&tmp_dr16_left_x,&Rx_Data[0],sizeof(int8_t));
  memcpy(&tmp_dr16_left_y,&Rx_Data[1],sizeof(int8_t));
  memcpy(&tmp_dr16_right_x,&Rx_Data[2],sizeof(int8_t));
  memcpy(&tmp_dr16_right_y,&Rx_Data[3],sizeof(int8_t));
  memcpy(&tmp_sum,&Rx_Data[4],sizeof(int8_t));
  memcpy(&control_type, &Rx_Data[7], sizeof(uint8_t));

  if(tmp_sum != (tmp_dr16_left_x + tmp_dr16_left_y + tmp_dr16_right_x + tmp_dr16_right_y)){
    return;
  }

  #elif defined(NORMAL_CHASSIS)

  memcpy(&Gimbal_To_Chassis_Data, Rx_Data, 8);

  Target_CMD_Data.Target_Velocity_X   = Math_Int_To_Float(Gimbal_To_Chassis_Data.tmp_Velocity_X, -450, 450, -4.0f, 4.0f);
  Target_CMD_Data.Target_Delta_Length = Gimbal_To_Chassis_Data.dr16_left_y * Length_Angle_Resolution / 100.0f;
  Target_CMD_Data.Target_Delta_Yaw    = 0.0f;
  Target_CMD_Data.Target_Control_Type = Gimbal_To_Chassis_Data.control_type;

  #endif

}

float test_tp = 0.0f;
uint8_t start_flag = 0;
uint32_t Test_cnt = 0;
float tmp_target_length = 0.20f;
void Class_Balance_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
  //控制量的更改和模式什么的一般在中断里边实现
  Test_cnt ++;

  ParamUpdata();

  SpeedEstimate();      //进行速度的观测

  Left_Leg.VMC_Calc();
  Right_Leg.VMC_Calc();

  SpeedUpdata();        //速度传参更新

  LQR_Calc();

  if(Chassis_Control_Type == Chassis_Control_Type_DISABLE){
    Chassis_Disable();
  }
  else if(Chassis_Control_Type == Chassis_Control_Type_RESERVE){
    Reserve_FSM();            //自救的逻辑还需要完善，比如腿在前方的时候
    LengthControl();          //腿长和Roll
    ReserveOutput();          //自救模式下的力矩输出

    Left_Leg.VMCProject();
    Right_Leg.VMCProject();
  }
  else if(Chassis_Control_Type == Chassis_Control_Type_JUMP_1){
    JUMP_1_FSM();
    LengthControl();
    JUMP_1_Output();

    Left_Leg.VMCProject();
    Right_Leg.VMCProject(); 

  }
  else{             //随动 小陀螺 跳跃 不随动
    SynthesizeMotion();       //转向和防劈叉
    LengthControl();          //腿长和Roll
    NormalOutput();

    Left_Leg.VMCProject();
    Right_Leg.VMCProject();   

    Chassis_Stable_Count++;
    //刚起立时腿部不太稳定，可能会错误进入离地
    if(Chassis_Stable_Count >= 500){
      //用脚踩Pitch会导致摆角向后摆，为了维持腿长会使腿触地只有一个圆周的范围，所以会导致离地
      Left_Leg.ForceSlove();
      Right_Leg.ForceSlove();

      //在腿部稳定的情况下，如果开启了上台阶模式，进入到达台阶的判断
      if(Jump_Enable_Flag == 1 && Chassis_Control_Type == Chassis_Control_Type_FLLOW){
        if(Left_Leg.L0 > 0.25f && Left_Leg.theta > 0.4f && Right_Leg.L0 > 0.25f && Right_Leg.theta > 0.4f){
          Chassis_Control_Type = Chassis_Control_Type_JUMP_1;     //磕上台阶
        }
      }

      if(Chassis_Stable_Count == 500){
        // X = Target_X = 0.0f;
      }

    }
    else{
      Left_Leg.Air_Status = Leg_UnAir;
      Right_Leg.Air_Status = Leg_UnAir;
      #ifdef UNFLLOW_ENABLE
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;
      #elif defined(NORMAL_CHASSIS)
      Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      #endif
    }

  }

  // if(start_flag){
  //   //测试代码    
  //   Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = tmp_target_length;

  //   Left_Leg.Air_Status = Leg_Air;
  //   Right_Leg.Air_Status = Leg_Air;

  //   SynthesizeMotion();       //转向和防劈叉
  //   LengthControl();          //腿长和Roll
  //   NormalOutput();

  //   Left_Leg.VMCProject();
  //   Right_Leg.VMCProject();
  // }
  // else{
  //   Chassis_Disable();
  //   Left_Leg.VMCProject();
  //   Right_Leg.VMCProject();
  // }

  Left_Leg.Torque_Output<Left>();
  Right_Leg.Torque_Output<Right>();

}

float tmp_Pitch_Err;
float tmp_wt;
float LQR_Tp_L[10];
void Class_Balance_Chassis::LQR_Calc()
{
  Target_l_theta = Offset_Fit_Coefficients[0][0] + Offset_Fit_Coefficients[0][1]*Left_Leg.L0 + Offset_Fit_Coefficients[0][2]*Right_Leg.L0 
                 + Offset_Fit_Coefficients[0][3]*Left_Leg.L0 * Left_Leg.L0 + Offset_Fit_Coefficients[0][4]*Left_Leg.L0 * Right_Leg.L0 
                 + Offset_Fit_Coefficients[0][5]*Right_Leg.L0 * Right_Leg.L0;

  Target_r_theta = Offset_Fit_Coefficients[1][0] + Offset_Fit_Coefficients[1][1]*Left_Leg.L0 + Offset_Fit_Coefficients[1][2]*Right_Leg.L0 
                 + Offset_Fit_Coefficients[1][3]*Left_Leg.L0 * Left_Leg.L0 + Offset_Fit_Coefficients[1][4]*Left_Leg.L0 * Right_Leg.L0 
                 + Offset_Fit_Coefficients[1][5]*Right_Leg.L0 * Right_Leg.L0;

  Target_Pitch_Angle = Offset_Fit_Coefficients[2][0] + Offset_Fit_Coefficients[2][1]*Left_Leg.L0 + Offset_Fit_Coefficients[2][2]*Right_Leg.L0 
                 + Offset_Fit_Coefficients[2][3]*Left_Leg.L0 * Left_Leg.L0 + Offset_Fit_Coefficients[2][4]*Left_Leg.L0 * Right_Leg.L0 
                 + Offset_Fit_Coefficients[2][5]*Right_Leg.L0 * Right_Leg.L0;

  // Target_l_theta = 0.0f;
  // Target_r_theta = 0.0f;
  // Target_Pitch_Angle = 0.0f;

  if(Chassis_Control_Type == Chassis_Control_Type_SPIN){
    Target_l_theta = -0.0f;
    Target_r_theta = -0.0f;
  }

  if(Left_Leg.Air_Status == Leg_Air){
    Target_l_theta = -0.0f;
  }

  if(Right_Leg.Air_Status == Leg_Air){
    Target_r_theta = -0.0f;
  }

  Get_Polyfit_K();

  tmp_Pitch_Err = (Target_Pitch_Angle - Pitch_Angle);
  float a =  0.2f * Vx + 0.35f;
  Math_Constrain(&tmp_Pitch_Err, -a, a);

  //T(r-b)
  LQR_Out[0] = K[0][0] * (Target_X - X) + K[0][1] * (True_Target_Vx - Vx) + K[0][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[0][3] * (Target_Omega - GyroYaw) + K[0][4] * (Target_l_theta - Left_Leg.theta) + K[0][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[0][6] * (Target_r_theta - Right_Leg.theta) + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[0][8] * tmp_Pitch_Err + K[0][9] * (Target_Picth_Omega - GyroPitch);

  //T(l-b)
  LQR_Out[1] = K[1][0] * (Target_X - X) + K[1][1] * (True_Target_Vx - Vx) + K[1][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[1][3] * (Target_Omega - GyroYaw) + K[1][4] * (Target_l_theta - Left_Leg.theta) + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[1][6] * (Target_r_theta - Right_Leg.theta) + K[1][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[1][8] * tmp_Pitch_Err + K[1][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wr-r)
  LQR_Out[2] = K[2][0] * (Target_X - X) + K[2][1] * (True_Target_Vx - Vx) + K[2][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[2][3] * (Target_Omega - GyroYaw) + K[2][4] * (Target_l_theta - Left_Leg.theta) + K[2][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[2][6] * (Target_r_theta - Right_Leg.theta) + K[2][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[2][8] * tmp_Pitch_Err + K[2][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wl-l)
  LQR_Out[3] = K[3][0] * (Target_X - X) + K[3][1] * (True_Target_Vx - Vx) + K[3][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[3][3] * (Target_Omega - GyroYaw) + K[3][4] * (Target_l_theta - Left_Leg.theta) + K[3][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[3][6] * (Target_r_theta - Right_Leg.theta) + K[3][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[3][8] * tmp_Pitch_Err + K[3][9] * (Target_Picth_Omega - GyroPitch);

  tmp_wt = LQR_Out[3];
  LQR_Tp_L[0] = K[1][0] * (Target_X - X);
  LQR_Tp_L[1] = K[1][1] * (True_Target_Vx - Vx);
  LQR_Tp_L[2] = K[1][2] * (Target_Yaw_Angle - Yaw_Angle);
  LQR_Tp_L[3] = K[1][3] * (Target_Omega - GyroYaw);
  LQR_Tp_L[4] = K[1][4] * (Target_l_theta - Left_Leg.theta);
  LQR_Tp_L[5] = K[1][5] * (Target_l_dtheta - Left_Leg.d_theta);
  LQR_Tp_L[6] = K[1][6] * (Target_r_theta - Right_Leg.theta);
  LQR_Tp_L[7] = K[1][7] * (Target_r_dtheta - Right_Leg.d_theta);
  LQR_Tp_L[8] = K[1][8] * tmp_Pitch_Err;
  LQR_Tp_L[9] = K[1][9] * (Target_Picth_Omega - GyroPitch);
  
  if(Left_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[1] = K[1][4] * (Target_l_theta - Left_Leg.theta) + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta);
    LQR_Out[3] = 0.0f;
  }

  if(Right_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[0] = K[0][6] * (Target_r_theta - Right_Leg.theta) + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta);
    LQR_Out[2] = 0.0f;
  }

}

void Class_Balance_Chassis::LengthControl()
{
  //对于Roll的补偿还有另一种计算的方式
  Roll_PID.Set_Now(Roll_Angle);
  Roll_PID.Set_D_Extern_Value(-GyroRoll);       //error的变换率 （Target - Now）
  Roll_PID.Set_Target(0.0f);
  Roll_PID.TIM_Adjust_PeriodElapsedCallback();

  // if(Left_Leg.Air_Status == Leg_UnAir && Right_Leg.Air_Status == Leg_UnAir && IS_NORMAL()){
  //   float tan_beta = ((Right_Leg.L0 - Left_Leg.L0) * arm_cos_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_sin_f32(Roll_Angle)) / ((Left_Leg.L0 - Right_Leg.L0) * arm_sin_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_cos_f32(Roll_Angle));
  //   float Compensite_Length = 2.0f * Chassis_Half_Width * tan_beta;
  //   // Compensite_Length = 0.0f;

  //   Left_Leg.Target_L0 = Target_Length - Compensite_Length / 2.0f;
  //   Right_Leg.Target_L0 = Target_Length + Compensite_Length / 2.0f;
  // }
  // else{
  //   if(Chassis_Control_Type == Chassis_Control_Type_RESERVE || Chassis_Control_Type == Chassis_Control_Type_JUMP_1){

  //   }
  //   else{
  //     Left_Leg.Target_L0 = Target_Length;
  //     Right_Leg.Target_L0 = Target_Length;
  //   }
  // }

  //腿的补偿加个斜坡？
  //单边腿离地也正常补偿Roll的平衡，双边离地不补偿
  float tmp_left_target_l0 = Left_Leg.Target_L0, tmp_right_target_l0 = Right_Leg.Target_L0;

  if(Left_Leg.Air_Status == Leg_Air && Right_Leg.Air_Status == Leg_Air){
    tmp_left_target_l0 = Target_Length;
    tmp_right_target_l0 = Target_Length;
  }
  else{
    if(IS_NORMAL()){
      float tan_beta = ((Right_Leg.L0 - Left_Leg.L0) * arm_cos_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_sin_f32(Roll_Angle)) / ((Left_Leg.L0 - Right_Leg.L0) * arm_sin_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_cos_f32(Roll_Angle));
      float Compensite_Length = 2.0f * Chassis_Half_Width * tan_beta;
      // Compensite_Length = 0.0f;

      tmp_left_target_l0 = Target_Length - Compensite_Length / 2.0f;
      tmp_right_target_l0 = Target_Length + Compensite_Length / 2.0f;
    }
    else{
      // tmp_left_target_l0 = Target_Length;
      // tmp_right_target_l0 = Target_Length;
    }
  }

  //离地和不离地反复切换的时候因为坡面上的补偿加上单边腿离地不是同时处理F，会导致Roll偏，导致腿长补偿突然变大，可能会导致震荡，暂时先加个斜坡限制一下腿长的变化率，等后面有时间了再优化一下这个逻辑
  // if(fabs(tmp_left_target_l0 - Left_Leg.Target_L0) * 1000.0f/ROBOT_TASK_DT < 2.0f){
  //   Left_Leg.Target_L0 = tmp_left_target_l0;
  // }
  // else{
  //   Left_Leg.Target_L0 = Left_Leg.Target_L0 + 2.0f * ROBOT_TASK_DT * ((tmp_left_target_l0 - Left_Leg.Target_L0) > 0 ? 1.0f : -1.0f) / 1000.0f;
  // }

  // if(fabs(tmp_right_target_l0 - Right_Leg.Target_L0) * 1000.0f/ROBOT_TASK_DT < 2.0f){
  //   Right_Leg.Target_L0 = tmp_right_target_l0;
  // }
  // else{
  //   Right_Leg.Target_L0 = Right_Leg.Target_L0 + 2.0f * ROBOT_TASK_DT * ((tmp_right_target_l0 - Right_Leg.Target_L0) > 0 ? 1.0f : -1.0f) / 1000.0f;
  // }

  Left_Leg.Target_L0 = tmp_left_target_l0;
  Right_Leg.Target_L0 = tmp_right_target_l0;



  // Left_Leg.Target_L0 = Right_Leg.Target_L0 = Target_Length;

  Left_Leg.Length_Calc();
  Right_Leg.Length_Calc();

  float L1 = (Right_Leg.L0 + Left_Leg.L0) / 2.0f;						    //质心理论上在车体中心不变
	float F1 = Robot_Mg * Vx * GyroYaw;			//质心受到的向心力

  if(Chassis_Control_Type == Chassis_Control_Type_FLLOW){
    Compensite_F0 = 1.0f * L1 * F1 / Chassis_Width;							//0.40m是车宽     转向向心力的补偿 0.7是人为的缩放因子
  }
  else{
    Compensite_F0 = 0.0f;
  }
}

void Class_Balance_Chassis::SynthesizeMotion()
{

}

void Class_Balance_Chassis::SpeedUpdata()
{

}

void Class_Balance_Chassis::ParamUpdata()
{
  #ifdef UNFLLOW_ENABLE
  Yaw_Angle   = IMU.Get_Rad_Yaw();
  #elif defined(NORMAL_CHASSIS)
  Yaw_Angle   = Motor_Yaw.Get_Now_Radian();
  Yaw_Angle = Normalize_Angle_Radian_PI_to_PI(Yaw_Angle);
  #endif  

  Roll_Angle  = -IMU.Get_Rad_Roll();
  Pitch_Angle = IMU.Get_Rad_Pitch();
  Accel_X   = -IMU.Get_Accel_Y_b();              //注意看是不是对应了
  Accel_Z   = IMU.Get_Accel_Z_b();
  GyroYaw   = IMU.Get_Gyro_Yaw();
  GyroRoll  = -IMU.Get_Gyro_Roll();              //Roll和Pitch陀螺仪速度应该换一下（可能）
  GyroPitch = IMU.Get_Gyro_Pitch();

  //IMU安装方式前Y，右X，这也是IMU解算算法的坐标系
  Left_Leg.GyroPitch  = GyroPitch;
  Left_Leg.Accel_Z    = Accel_Z;
  
  //香港大学建模两腿都是X朝前
  Right_Leg.GyroPitch = GyroPitch;
  Right_Leg.Accel_Z   = Accel_Z;

  //小陀螺行进
  if(Chassis_Control_Type == Chassis_Control_Type_SPIN){
    //限制小陀螺的最大速度
    Math_Constrain(&Target_Vx, -V_MAX_SPIN, V_MAX_SPIN);

    #ifdef UNFLLOW_ENABLE
    float tmp_delta_yaw = Normalize_Angle_Radian_0_to_2PI(Yaw_Angle * PI / 180.0f);
    #endif

    #ifdef NORMAL_CHASSIS
    float tmp_delta_yaw = -Motor_Yaw.Get_Now_Radian() + Reference_Rad;
    tmp_delta_yaw = Normalize_Angle_Radian_0_to_2PI(tmp_delta_yaw);
    #endif

    True_Target_Vx = Target_Vx * arm_sin_f32(tmp_delta_yaw);
    // Target_X = 0.0f;
  }
  else{
    // Target_X = 0.0f;
    True_Target_Vx = Target_Vx;

    // True_Target_Vx = Target_Vx;

  }

  Target_X = Target_X + True_Target_Vx * ROBOT_TASK_DT / 1000.0f;

  //功率限制限制速度
  Power_Control_Task(&True_Target_Vx);
  Angle_Continuity_Process_Rad(&Target_Yaw_Angle, Yaw_Angle);

  Math_Constrain(&Target_Length, Length_MIN, Length_MAX);
  
  Left_Leg.ParamUpdata();
  Right_Leg.ParamUpdata();

  //腿部数据的更新
  //左腿       phi1 phi4的对应实际上是看腿部的摆杆，而不是电机的前后
  Left_Leg.phi1 = Left_Leg.Back_Joint.Get_Now_Angle();                //原本应该+pi/2.0f 但是因为解包的时候偏移了+pi，所以应该-pi + pi/2.0 = -pi/2.0
  Left_Leg.phi4 = -PI  + Left_Leg.Front_Joint.Get_Now_Angle();

  Left_Leg.phi1 = Normalize_Angle_Radian_PI_to_PI(Left_Leg.phi1);
  Left_Leg.phi4 = Normalize_Angle_Radian_PI_to_PI(Left_Leg.phi4);

  Left_Leg.d_phi1 = Left_Leg.Back_Joint.Get_Now_Omega();
  Left_Leg.d_phi4 = Left_Leg.Front_Joint.Get_Now_Omega();

  //右腿
  Right_Leg.phi1 = - Right_Leg.Back_Joint.Get_Now_Angle();                //原本应该+pi/2.0f 但是因为解包的时候偏移了+pi，所以应该-pi + pi/2.0 = -pi/2.0
  Right_Leg.phi4 = -PI - Right_Leg.Front_Joint.Get_Now_Angle();

  Right_Leg.phi1 = Normalize_Angle_Radian_PI_to_PI(Right_Leg.phi1);
  Right_Leg.phi4 = Normalize_Angle_Radian_PI_to_PI(Right_Leg.phi4);

  Right_Leg.d_phi1 = -Right_Leg.Back_Joint.Get_Now_Omega();
  Right_Leg.d_phi4 = -Right_Leg.Front_Joint.Get_Now_Omega();

  //轮子自身力矩都遵循右手系，面朝Y负方向看过去逆时针为正（这里加负号和电机安装方式相关）
  Kalman_PeriodElapsedCallback(&Left_Leg.Wheel_Speed_Kalman, -Left_Leg.Wheel_Motor.Get_Now_Omega_Radian());
  Kalman_PeriodElapsedCallback(&Right_Leg.Wheel_Speed_Kalman, Right_Leg.Wheel_Motor.Get_Now_Omega_Radian());

  Left_Leg.Wheel_Speed = Kalman_Get_Out(Left_Leg.Wheel_Speed_Kalman);
  Right_Leg.Wheel_Speed = Kalman_Get_Out(Right_Leg.Wheel_Speed_Kalman);

}

void Class_Balance_Chassis::SpeedEstimate()
{
  Left_Leg.Leg_V_Calc();
  Right_Leg.Leg_V_Calc();

  if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){
    aver_v = (Left_Leg.leg_v_true + Right_Leg.leg_v_true) / 2.0f;
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air && Right_Leg.Get_Air_Status() == Leg_Air){
    aver_v = Vx;
  }
  else if(Right_Leg.Get_Air_Status() == Leg_Air){
    aver_v = (Left_Leg.leg_v_true + Vx) / 2.0f;
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air){
    aver_v = (Right_Leg.leg_v_true + Vx) / 2.0f;
  }
  
  V_EstimateKF.MeasuredVector[0] = aver_v;
  V_EstimateKF.MeasuredVector[1] = Accel_X;
  Kalman_Filter_Update(&V_EstimateKF, NULL);

  Vx = V_EstimateKF.FilteredValue[0];

  X = X + Vx * ROBOT_TASK_DT * 1.0f/ 1000.0f;

  // if(fabs(Target_Vx) < 0.1f){
  //   X = X + Vx * ROBOT_TASK_DT / 1000.0f;
  //   Target_X = 0.0f;
  // }
  // else{
  //   X = 0.0f;
  //   Target_X = 0.0f;
  // }

}

void Class_Balance_Chassis::Power_Control_Task(float *Target_Vx)
{
  
}

void Class_Balance_Chassis::NormalOutput()
{
  //最终力的输出
  if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){
    if(Chassis_Control_Type == Chassis_Control_Type_FLLOW){
      Left_Leg.F0  = 100.0f / arm_cos_f32(Left_Leg.theta) + Left_Leg.dLength_PID.Get_Out() - Compensite_F0;         //F0是机体受到的向上的力
      Right_Leg.F0 = 100.0f / arm_cos_f32(Right_Leg.theta) + Right_Leg.dLength_PID.Get_Out() + Compensite_F0; 
    }
    else{
      //小陀螺不用补偿前馈
      Left_Leg.F0  = 100.0f / arm_cos_f32(Left_Leg.theta) + Left_Leg.dLength_PID.Get_Out() + Roll_PID.Get_Out();         //F0是机体受到的向上的力
      Right_Leg.F0 = 100.0f / arm_cos_f32(Right_Leg.theta) + Right_Leg.dLength_PID.Get_Out() - Roll_PID.Get_Out(); 
    }
    
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air && Right_Leg.Get_Air_Status() == Leg_Air){
    Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out(); 
  }
  else if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_Air){
    Left_Leg.F0  = 100.0f / arm_cos_f32(Left_Leg.theta) + Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out(); 
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air && Right_Leg.Get_Air_Status() == Leg_UnAir){
    Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = 100.0f / arm_cos_f32(Right_Leg.theta) + Right_Leg.dLength_PID.Get_Out(); 
  }

  Left_Leg.Tp = -LQR_Out[1];
  Left_Leg.Wheel_T = -LQR_Out[3];

  Right_Leg.Tp = -LQR_Out[0];
  Right_Leg.Wheel_T = -LQR_Out[2];

  //对应上转速的建模，面朝Y负方向，逆时针为正，电机安装方式导致左轮需要加负号
  Left_Leg.Wheel_T  = -Left_Leg.Wheel_T;
  Right_Leg.Wheel_T = Right_Leg.Wheel_T;

  // Left_Leg.Wheel_T  = 0.0f;
  // Right_Leg.Wheel_T = 0.0f;

  Math_Constrain(&Left_Leg.F0, -300.0f, 300.0f);
  Math_Constrain(&Left_Leg.Wheel_T, -4.9f, 4.9f);

  Math_Constrain(&Right_Leg.F0, -300.0f, 300.0f);
  Math_Constrain(&Right_Leg.Wheel_T, -4.9f, 4.9f);
}

void Class_Balance_Chassis::JUMP_1_Output()
{
  Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
  Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out();

  Math_Constrain(&Left_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Left_Leg.Wheel_T, -4.2f, 4.2f);

  Math_Constrain(&Right_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Right_Leg.Wheel_T, -4.2f, 4.2f);
}

void Class_Balance_Chassis::ReserveOutput()
{
  Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
  Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out();

  Math_Constrain(&Left_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Left_Leg.Wheel_T, -4.2f, 4.2f);

  Math_Constrain(&Right_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Right_Leg.Wheel_T, -4.2f, 4.2f);
}

void Class_Balance_Chassis::Chassis_Disable()
{
  Target_X = X = 0.0f;
  Chassis_Stable_Count = 0;
  Target_Omega = 0.0f;
  Target_Yaw_Angle = Yaw_Angle;
  jump_state = JUMP_BACK_SWING;
  Reserve_Status = Reserve_Disable;

  Left_Leg.Disable();
  Right_Leg.Disable();

  Tp_PID.Set_Integral_Error(0.0f);
  Roll_PID.Set_Integral_Error(0.0f);
  Turn_Omega_PID.Set_Integral_Error(0.0f);
  Turn_Angle_PID.Set_Integral_Error(0.0f);
}

void Class_Balance_Chassis::Get_Polyfit_K()
{
  for (int n = 0; n < 40; n++) {
    int row = n / 10;
    int col = n % 10;
    K[row][col] = K_Fit_Coefficients[n][0] 
                + K_Fit_Coefficients[n][1] * Left_Leg.L0
                + K_Fit_Coefficients[n][2] * Right_Leg.L0
                + K_Fit_Coefficients[n][3] * Left_Leg.L0 * Left_Leg.L0 
                + K_Fit_Coefficients[n][4] * Left_Leg.L0 * Right_Leg.L0 
                + K_Fit_Coefficients[n][5] * Right_Leg.L0 * Right_Leg.L0;
  }
}

void Class_Balance_Chassis::V_EstimateKF_Init()
{
  Kalman_Filter_Init(&V_EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
	memcpy(V_EstimateKF.F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
  memcpy(V_EstimateKF.P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
  memcpy(V_EstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
  memcpy(V_EstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
  memcpy(V_EstimateKF.H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void Class_Balance_Chassis::Reserve_FSM()
{
  static uint16_t status_cnt = 0;
  static uint8_t Left_Alpha_Status = 0, Right_Alpha_Status = 0;     //切到1的时候代表偏角过大，先速度环在切回角度环
  static float resver_alpha_Kp = 40.0f, reserve_d_alpha_Kp = 6.0f;
  static float resver_alpha_Kd = 0.2f;
  static float Target_Left_Alpha = 0.0f, Target_Right_Alpha = 0.0f;     //左右坐标系不一致
  switch (Reserve_Status)
  {
    case(Reserve_Disable):
    {
      status_cnt = 0;
      Left_Alpha_Status = Right_Alpha_Status = 0;
      Left_Leg.Disable();
      Right_Leg.Disable();
      if(Pitch_Angle > -70.0f / 57.3f && Pitch_Angle < 70.0f / 57.3f){
        Reserve_Status = Reserve_Status_2;        //切换到Pitch正常的状态
      }
      else{
        Reserve_Status = Reserve_Status_1;        //切换到Pitch不正常的状态
      }
      break;
    }
    case(Reserve_Status_1):
    {
      if(Pitch_Angle < -30.0f / 57.3f || Pitch_Angle > 30.0f / 57.3f){                //仍然是翻车状态
        status_cnt = 0;
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.33f;

        Left_Leg.Wheel_T = 0.0f;
        Right_Leg.Wheel_T = 0.0f;
        Left_Leg.Tp  = reserve_d_alpha_Kp * (2.5f - Left_Leg.d_alpha);
        Right_Leg.Tp = reserve_d_alpha_Kp * (2.5f - Right_Leg.d_alpha);
      }
      else{
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.33f;

        Left_Leg.Wheel_T = 0.0f;
        Right_Leg.Wheel_T = 0.0f;
        Left_Leg.Tp  = 0.0f;
        Right_Leg.Tp = 0.0f;
      }

      if(fabs(Pitch_Angle) < 10.0f / 57.3f){
        status_cnt ++;
      }

      //状态切换时清零计数
      if(status_cnt > 50){
        status_cnt = 0;
        Reserve_Status = Reserve_Status_2; 
      }

      break;
    }
    case(Reserve_Status_2):{
      float Left_Error = 0.0f, Right_Error = 0.0f;

      if(Left_Leg.alpha < -1.20f && Left_Alpha_Status == 0){        //偏角过大了，先用速度环摆正
        Left_Alpha_Status = 1;
      }

      if(Right_Leg.alpha < -1.20f && Right_Alpha_Status == 0){
        Right_Alpha_Status = 1;
      }

      if(Left_Alpha_Status == 1){     //用速度环摆正
        Left_Leg.Target_L0 = 0.30f;
        Left_Leg.Tp = reserve_d_alpha_Kp * (-2.5f - Left_Leg.d_alpha);

        if(fabs(Left_Leg.alpha - 1.5f) < 0.05f){      //接近1.5rad认为可以切换角度换控制
          Left_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process_Rad(&Target_Left_Alpha, Left_Leg.alpha);
        float Left_Error = Target_Left_Alpha - Left_Leg.alpha;
        Left_Leg.Tp  = resver_alpha_Kp * Left_Error - resver_alpha_Kd * Left_Leg.d_alpha;
        Left_Leg.Target_L0 = 0.16f;
      }

      if(Right_Alpha_Status == 1){     //用速度环摆正
        Right_Leg.Target_L0 = 0.30f;
        Right_Leg.Tp = reserve_d_alpha_Kp * (-2.5f - Right_Leg.d_alpha);

        if(fabs(Right_Leg.alpha - 1.5f) < 0.05f){
          Right_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process_Rad(&Target_Right_Alpha, Right_Leg.alpha);
        Right_Error = Target_Right_Alpha - Right_Leg.alpha;
        Right_Leg.Tp = resver_alpha_Kp * Right_Error - resver_alpha_Kd * Right_Leg.d_alpha;
        Right_Leg.Target_L0 = 0.16f;
      }

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;

      //左右腿都是角度换算了，并且都在合理范围内了，就认为自救完成了
      if(fabs(Right_Error) < 0.3f && fabs(Left_Error) < 0.3f && Left_Alpha_Status == 0 && Right_Alpha_Status == 0){
        status_cnt ++;
      }

      if(status_cnt > 500){
        status_cnt = 0;
        Left_Alpha_Status = 0;
        Right_Alpha_Status = 0;
        Reserve_Status = Reserve_Complete; 
      }

      break;
    }
    case(Reserve_Complete):
    {
      X = 0.0f;
      Vx = 0.0f;

      Target_X  = 0.0f;
      Target_Vx = 0.0f;
      Target_Yaw_Angle = Yaw_Angle;
      Chassis_Stable_Count = 0;
      Target_Roll_Angle = 0.0f;
      Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.20f;
      Reserve_Status = Reserve_Disable;

      // Left_Leg.Disable();
      // Right_Leg.Disable();

      Angle_Continuity_Process_Rad(&Target_Left_Alpha, Left_Leg.alpha);
      Angle_Continuity_Process_Rad(&Target_Right_Alpha, Right_Leg.alpha);

      float Left_Error = Target_Left_Alpha - Left_Leg.alpha;
      float Right_Error = Target_Right_Alpha - Right_Leg.alpha;

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;
      Left_Leg.Tp  = resver_alpha_Kp * Left_Error;
      Right_Leg.Tp = resver_alpha_Kp * Right_Error;

      Math_Constrain(&Left_Leg.Tp, -15.0f, 15.0f);
      Math_Constrain(&Right_Leg.Tp, -15.0f, 15.0f);

      Left_Leg.Air_Status = Leg_UnAir;
      Right_Leg.Air_Status = Leg_UnAir;

      #ifdef UNFLLOW_ENABLE
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;        //切回正常lqr
      #else
      Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      #endif

      // Chassis_Control_Type = Chassis_Control_Type_DISABLE;
      
      break;
    }
  
    default:
    {
      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;
      Left_Leg.Tp  = 0.0f;
      Right_Leg.Tp = 0.0f;

      Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.20f;
      break;
    }
  }
}

void Class_Balance_Chassis::JUMP_1_FSM()
{
  static uint16_t state_cnt = 0;

  // 参数（建议后续调参）
  float L_LONG = 0.32f;
  float L_SHORT = 0.16f;

  float BACK_TARGET_ALPHA_L = 2.2f;   // 左腿后摆
  float BACK_TARGET_ALPHA_R = 2.2f;

  float NORMAL_ALPHA_L = 0.0f;
  float NORMAL_ALPHA_R = 0.0f;

  float Left_Target_Alpha_Omega = 3.0f;         //向后转动
  float Right_Target_Alpha_Omega = 3.0f;

  float KP_Omega_ALPHA = 8.0f;
  float KP_ALPHA = 40.0f;
  float KD_ALPHA = 0.2f;

  //清零轮电机的输出
  Left_Leg.Wheel_T = 0.0f;
  Right_Leg.Wheel_T = 0.0f;

  Left_Leg.Air_Status = Leg_Air;
  Right_Leg.Air_Status = Leg_Air;

  Chassis_Stable_Count = 0;

  switch(jump_state)
  {
    case JUMP_BACK_SWING:
    {
      // 保持长腿
      Target_Length = L_LONG;
      Left_Leg.Target_L0  = L_LONG;
      Right_Leg.Target_L0 = L_LONG;

      // // 角度连续处理
      Angle_Continuity_Process_Rad(&BACK_TARGET_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process_Rad(&BACK_TARGET_ALPHA_R, Right_Leg.alpha);

      float err_l = BACK_TARGET_ALPHA_L - Left_Leg.alpha;
      float err_r = BACK_TARGET_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_Omega_ALPHA * (Left_Target_Alpha_Omega - Left_Leg.d_alpha);
      Right_Leg.Tp = KP_Omega_ALPHA * (Right_Target_Alpha_Omega - Right_Leg.d_alpha);

      if(fabs(err_l) < 0.2f){
        Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha;
      }

      if(fabs(err_r) < 0.2f){
        Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha;
      }

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      // 到位判断
      if(fabs(err_l) < 0.2f && fabs(err_r) < 0.2f)
      {
        state_cnt += ROBOT_TASK_DT;
      }
      else{
        state_cnt = 0;
      }

      if(state_cnt > 500)
      {
        state_cnt = 0;
        jump_state = JUMP_RECOVER;
      }

      break;
    }
    case JUMP_RECOVER:
    {
      // 恢复到正常长度
      Target_Length = L_SHORT;
      Left_Leg.Target_L0  = L_SHORT;
      Right_Leg.Target_L0 = L_SHORT;

      Angle_Continuity_Process_Rad(&NORMAL_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process_Rad(&NORMAL_ALPHA_R, Right_Leg.alpha);

      float err_l = NORMAL_ALPHA_L - Left_Leg.alpha;
      float err_r = NORMAL_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      if(fabs(err_l) < 0.2f && fabs(err_r) < 0.2f)
      {
        state_cnt += ROBOT_TASK_DT;
      }
      else{
        state_cnt = 0;
      }

      if(state_cnt > 500)
      {
        state_cnt = 0;
        jump_state = JUMP_DONE;
      }

      break;
    }

    case JUMP_DONE:
    {
      // 清状态，回归正常控制
      state_cnt = 0;
      jump_state = JUMP_BACK_SWING;

      //相关状态重置
      X = Vx = 0.0f;
      Target_X = Target_Vx = 0.0f;
      Target_Roll_Angle = 0.0f;
      Target_Yaw_Angle = Yaw_Angle;

      Target_Length = 0.20f;
      Left_Leg.Target_L0  = 0.20f;
      Right_Leg.Target_L0 = 0.20f;

      // Left_Leg.Tp = 0.0f;
      // Right_Leg.Tp = 0.0f;

      Angle_Continuity_Process_Rad(&NORMAL_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process_Rad(&NORMAL_ALPHA_R, Right_Leg.alpha);

      float err_l = NORMAL_ALPHA_L - Left_Leg.alpha;
      float err_r = NORMAL_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      #ifdef UNFLLOW_ENABLE
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;
      #elif defined(NORMAL_CHASSIS)
      Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      #endif

      break;
    }

    default:{
      state_cnt = 0;
      jump_state = JUMP_BACK_SWING;
      break;
    }
  }
}