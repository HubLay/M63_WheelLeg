#include "balance_chassis.h"

#include "user_lib.h"
#include "drv_math.h"
#include "cantransmit_msg.h"

#define RAD_TO_ANGLE (180.0f / 3.14159f)

// K矩阵拟合系数 K_Fit_Coefficients[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
float K_Fit_Coefficients[40][6] = {
    {    -67.7913f,      138.026f,      31.6413f,     -70.3091f,     -261.571f,      188.276f},  // K[0][0]
    {    -48.3693f,       123.19f,      11.1365f,     -60.3559f,     -204.208f,      137.143f},  // K[0][1]
    {    -2.18963f,    -0.565473f,     -19.3753f,      2.49008f,     -9.00046f,      26.6187f},  // K[0][2]
    {   -0.498516f,      2.30725f,      -8.6469f,     -3.06739f,      -3.6309f,      10.9426f},  // K[0][3]
    {    -15.2925f,      41.0504f,      63.7288f,     -8.49533f,     -26.4061f,     -30.5724f},  // K[0][4]
    {   -0.359945f,      2.39944f,      3.96372f,     -1.12979f,       3.4962f,     -4.78026f},  // K[0][5]
    {    -73.1746f,      57.6674f,      47.8833f,     -32.2696f,     -109.139f,      42.7389f},  // K[0][6]
    {    -8.50039f,     0.902298f,      15.8656f,     0.404209f,      -4.8853f,     -13.8439f},  // K[0][7]
    {       62.16f,     -161.294f,       581.37f,      198.209f,     -91.5409f,     -622.552f},  // K[0][8]
    {    -1.06746f,      2.78754f,      37.3237f,      4.39946f,     -28.2828f,     -27.5039f},  // K[0][9]
    {    -67.7913f,      31.6413f,      138.026f,      188.276f,     -261.571f,     -70.3091f},  // K[1][0]
    {    -48.3693f,      11.1365f,       123.19f,      137.143f,     -204.208f,     -60.3559f},  // K[1][1]
    {     2.18963f,      19.3753f,     0.565473f,     -26.6187f,      9.00046f,     -2.49008f},  // K[1][2]
    {    0.498516f,       8.6469f,     -2.30725f,     -10.9426f,       3.6309f,      3.06739f},  // K[1][3]
    {    -73.1746f,      47.8833f,      57.6674f,      42.7389f,     -109.139f,     -32.2696f},  // K[1][4]
    {    -8.50039f,      15.8656f,     0.902298f,     -13.8439f,      -4.8853f,     0.404209f},  // K[1][5]
    {    -15.2925f,      63.7288f,      41.0504f,     -30.5724f,     -26.4061f,     -8.49533f},  // K[1][6]
    {   -0.359945f,      3.96372f,      2.39944f,     -4.78026f,       3.4962f,     -1.12979f},  // K[1][7]
    {       62.16f,       581.37f,     -161.294f,     -622.552f,     -91.5409f,      198.209f},  // K[1][8]
    {    -1.06746f,      37.3237f,      2.78754f,     -27.5039f,     -28.2828f,      4.39946f},  // K[1][9]
    {     6.14273f,     -52.6332f,      99.0239f,      22.9675f,      42.7346f,     -121.874f},  // K[2][0]
    {     5.80408f,     -31.8586f,      56.5506f,      12.0039f,      28.6164f,     -69.8499f},  // K[2][1]
    {    -2.01184f,     -5.17622f,      8.15991f,      7.88057f,     -4.57954f,     -4.17809f},  // K[2][2]
    {   -0.488698f,     -2.53496f,      2.94051f,      3.93064f,     -2.74001f,     -0.67159f},  // K[2][3]
    {     4.29775f,      16.8243f,      2.24424f,     -32.9657f,      26.2444f,     -21.2486f},  // K[2][4]
    {     0.29384f,      0.98516f,    -0.399201f,     -0.50251f,     0.192204f,    -0.253413f},  // K[2][5]
    {     8.67789f,     -28.9316f,      80.8722f,      33.3861f,     -20.2456f,     -61.7875f},  // K[2][6]
    {    0.914756f,     -1.29243f,      5.92855f,      1.94446f,     -2.18659f,     -3.24853f},  // K[2][7]
    {     41.5505f,     -59.8074f,     -45.6758f,      48.1899f,      55.2552f,     -15.1447f},  // K[2][8]
    {     4.00879f,     -9.66432f,     -1.52928f,      7.71735f,       8.2313f,     -4.56991f},  // K[2][9]
    {     6.14273f,      99.0239f,     -52.6332f,     -121.874f,      42.7346f,      22.9675f},  // K[3][0]
    {     5.80408f,      56.5506f,     -31.8586f,     -69.8499f,      28.6164f,      12.0039f},  // K[3][1]
    {     2.01184f,     -8.15991f,      5.17622f,      4.17809f,      4.57954f,     -7.88057f},  // K[3][2]
    {    0.488698f,     -2.94051f,      2.53496f,      0.67159f,      2.74001f,     -3.93064f},  // K[3][3]
    {     8.67789f,      80.8722f,     -28.9316f,     -61.7875f,     -20.2456f,      33.3861f},  // K[3][4]
    {    0.914756f,      5.92855f,     -1.29243f,     -3.24853f,     -2.18659f,      1.94446f},  // K[3][5]
    {     4.29775f,      2.24424f,      16.8243f,     -21.2486f,      26.2444f,     -32.9657f},  // K[3][6]
    {     0.29384f,    -0.399201f,      0.98516f,    -0.253413f,     0.192204f,     -0.50251f},  // K[3][7]
    {     41.5505f,     -45.6758f,     -59.8074f,     -15.1447f,      55.2552f,      48.1899f},  // K[3][8]
    {     4.00879f,     -1.52928f,     -9.66432f,     -4.56991f,       8.2313f,      7.71735f}   // K[3][9]
};

// 平衡点偏移拟合系数 Offset_Fit_Coefficients[3][6]
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq

float Offset_Fit_Coefficients[3][6] = {
    {    0.198437f,     -0.78094f, -9.05786e-16f,     0.871354f,  1.51919e-15f,  6.86025e-16f},  // theta_l_eq
    {    0.198437f, -1.63352e-15f,     -0.78094f,  1.89475e-15f,   1.8939e-15f,     0.871354f},  // theta_r_eq
    {        0.05f, -8.45955e-17f, -1.73195e-16f, -2.81855e-16f,  4.08282e-16f,  5.31802e-17f}   // theta_b_eq
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
float vaEstimateKF_R[4] = {1000.0f, 0.0f, 
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

  Supercap.Init(&hfdcan2, 60.0f);
  Referee.Init(&huart10);

  Daemon_Init_Config_s daemon_init_config_instance_1 = Get_DaemonInitConfig_s(100, NULL, NULL);
  Referee_Daemon = DaemonRegister(daemon_init_config_instance_1);

  Daemon_Init_Config_s daemon_init_config_instance_2 = Get_DaemonInitConfig_s(10, NULL, NULL);
  Supercap_Daemon = DaemonRegister(daemon_init_config_instance_2);

  CanTransmit_TaskRegister(1, 0xF1, CAN1_0xxf1_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(2, 0xF2, CAN1_0xxf2_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(3, 0xF1, CAN2_0xxf1_Tx_Data, &hfdcan2);
  CanTransmit_TaskRegister(4, 0xF2, CAN2_0xxf2_Tx_Data, &hfdcan2);

  Daemon_Init_Config_s daemon_init_config_instance = Get_DaemonInitConfig_s(50, NULL, Gimbal_Offline_CallbackFunction);
  Gimbal_Daemon = DaemonRegister(daemon_init_config_instance);

  TD_Init(&Target_Vx_Td, 15, 1, 0.002);

  Tp_PID.Init(80.0f, 0.0f, 2.0f, 0.0f, 0.0f, 30.0f);
  Roll_PID.Init(1000.0f, 0.0f, 10.0f,0.0f,0.0f,150.0f);
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
    if(Chassis_Stable_Count >= 100){
      //用脚踩Pitch会导致摆角向后摆，为了维持腿长会使腿触地只有一个圆周的范围，所以会导致离地
      Left_Leg.ForceSlove();
      Right_Leg.ForceSlove();

      //在腿部稳定的情况下，如果开启了上台阶模式，进入到达台阶的判断
      if(Jump_Enable_Flag == 1 && Chassis_Control_Type == Chassis_Control_Type_FLLOW){
        if(Left_Leg.L0 > 0.30f && Left_Leg.theta > 0.5f && Right_Leg.L0 > 0.30f && Right_Leg.theta > 0.5f){
          Chassis_Control_Type = Chassis_Control_Type_JUMP_1;     //磕上台阶
        }
      }

      if(Chassis_Stable_Count == 100){
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
  float a =  -0.20f * Vx + 0.50f;
  if(a < 0.1f){
    a = 0.1f;
  }

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

    }
  }

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

  Target_X = 0.0f;

  //功率限制限制速度
  Power_Control_Task(&True_Target_Vx);
  Angle_Continuity_Process_Rad(&Target_Yaw_Angle, Yaw_Angle);
  
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

  // X = X + Vx * ROBOT_TASK_DT * 1.0f/ 1000.0f;

  if(fabs(True_Target_Vx) < 0.1f){
    X = X + Vx * ROBOT_TASK_DT / 1000.0f;
    Target_X = 0.0f;
  }
  else{
    X = 0.0f;
    Target_X = 0.0f;
  }

}

void Class_Balance_Chassis::Power_Control_Task(float *__Target_Vx)
{
  Supercap.Set_Supercap_Mode(Supercap_ENABLE);
  if(Get_Device_Status(Referee_Daemon) == Device_Online){
    Limit_Power = Referee.Get_Chassis_Power_Max();
  }
  else{
    Limit_Power = 70.0f;
  }

  Supercap.Set_Limit_Power(Limit_Power);
  Supercap.TIM_Supercap_PeriodElapsedCallback();

  Limit_Power_Vx_Max = Limit_Power_Kp * sqrtf(Limit_Power);

  Math_Constrain(__Target_Vx, -Limit_Power_Vx_Max, Limit_Power_Vx_Max);

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
  True_Target_Vx = Target_Vx = X = 0.0f;
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
  static float resver_theta_Kp = 50.0f, reserve_d_alpha_Kp = 10.0f;
  static float resver_theta_Kd = 0.4f;
  static float Target_Left_theta = 0.15f, Target_Right_theta = 0.15f;     //左右坐标系不一致
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
        Left_Leg.Tp = reserve_d_alpha_Kp * (-5.0f - Left_Leg.d_alpha);

        if(fabs(Left_Leg.alpha - 1.5f) < 0.05f){      //接近1.5rad认为可以切换角度换控制
          Left_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process_Rad(&Target_Left_theta, Left_Leg.theta);
        float Left_Error = Target_Left_theta - Left_Leg.theta;
        Left_Leg.Tp  = resver_theta_Kp * Left_Error - resver_theta_Kd * Left_Leg.d_theta;
        Left_Leg.Target_L0 = 0.16f;
      }

      if(Right_Alpha_Status == 1){     //用速度环摆正
        Right_Leg.Target_L0 = 0.30f;
        Right_Leg.Tp = reserve_d_alpha_Kp * (-5.0f - Right_Leg.d_alpha);

        if(fabs(Right_Leg.alpha - 1.5f) < 0.05f){
          Right_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process_Rad(&Target_Right_theta, Right_Leg.theta);
        Right_Error = Target_Right_theta - Right_Leg.theta;
        Right_Leg.Tp = resver_theta_Kp * Right_Error - resver_theta_Kd * Right_Leg.d_theta;
        Right_Leg.Target_L0 = 0.16f;
      }

      Math_Constrain(&Left_Leg.Tp, -13.0f, 13.0f);
      Math_Constrain(&Right_Leg.Tp, -13.0f, 13.0f);

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;

      //左右腿都是角度换算了，并且都在合理范围内了，就认为自救完成了
      if(fabs(Right_Error) < 0.3f && fabs(Left_Error) < 0.3f && Left_Alpha_Status == 0 && Right_Alpha_Status == 0){
        status_cnt ++;
      }

      if(status_cnt > 100){
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

      Angle_Continuity_Process_Rad(&Target_Left_theta, Left_Leg.theta);
      Angle_Continuity_Process_Rad(&Target_Right_theta, Right_Leg.theta);

      float Left_Error = Target_Left_theta - Left_Leg.theta;
      float Right_Error = Target_Right_theta - Right_Leg.theta;

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;
      Left_Leg.Tp  = resver_theta_Kp * Left_Error;
      Right_Leg.Tp = resver_theta_Kp * Right_Error;

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

uint8_t cnt111;
void Class_Balance_Chassis::JUMP_1_FSM()
{
  static uint16_t state_cnt = 0;
  static uint8_t Left_Complete_Flag = 0, Right_Complete_Flag = 0;

  // 参数（建议后续调参）
  float L_LONG = 0.32f;
  float L_SHORT = 0.16f;

  float BACK_TARGET_ALPHA_L = 1.0f;   // 左腿后摆
  float BACK_TARGET_ALPHA_R = 1.0f;

  float NORMAL_Theta_L = 0.15f;
  float NORMAL_Theta_R = 0.15f;

  float Left_Target_Alpha_Omega = 13.0f;         //向后转动
  float Right_Target_Alpha_Omega = 13.0f;

  float KP_Omega_ALPHA = 8.0f;
  float KP_ALPHA = 50.0f;
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
        Left_Complete_Flag = 1;
      }
      if(Left_Complete_Flag){
        cnt111 ++;
        Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha;
      }

      if(fabs(err_r)){
        Right_Complete_Flag = 1;
      }
      if(Right_Complete_Flag){
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

      if(state_cnt > 100)
      {
        state_cnt = 0;
        jump_state = JUMP_RECOVER;
      }

      break;
    }
    case JUMP_RECOVER:
    {
      // 恢复到正常长度
      Left_Complete_Flag = 0;
      Right_Complete_Flag = 0;

      Target_Length = L_SHORT;
      Left_Leg.Target_L0  = L_SHORT;
      Right_Leg.Target_L0 = L_SHORT;

      Angle_Continuity_Process_Rad(&NORMAL_Theta_L, Left_Leg.theta);
      Angle_Continuity_Process_Rad(&NORMAL_Theta_R, Right_Leg.theta);

      float err_l = NORMAL_Theta_L - Left_Leg.theta;
      float err_r = NORMAL_Theta_R - Right_Leg.theta;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_theta;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_theta;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      if(fabs(err_l) < 0.2f && fabs(err_r) < 0.2f)
      {
        state_cnt += ROBOT_TASK_DT;
      }
      else{
        state_cnt = 0;
      }

      if(state_cnt > 200)
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
      Target_X = 0.3f;
      True_Target_Vx = Target_Vx = 0.0f;
      Target_Roll_Angle = 0.0f;
      Target_Yaw_Angle = Yaw_Angle;

      Target_Length = 0.20f;
      Left_Leg.Target_L0  = 0.20f;
      Right_Leg.Target_L0 = 0.20f;

      // Left_Leg.Tp = 0.0f;
      // Right_Leg.Tp = 0.0f;

      Angle_Continuity_Process_Rad(&NORMAL_Theta_L, Left_Leg.theta);
      Angle_Continuity_Process_Rad(&NORMAL_Theta_R, Right_Leg.theta);

      float err_l = NORMAL_Theta_L - Left_Leg.theta;
      float err_r = NORMAL_Theta_R - Right_Leg.theta;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_theta;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_theta;

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