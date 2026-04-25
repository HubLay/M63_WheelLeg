#include "balance_chassis.h"

#include "user_lib.h"
#include "drv_math.h"
#include "cantransmit_msg.h"

#define RAD_TO_ANGLE (180.0f / 3.14159f)

// K矩阵拟合系数 K_Fit_Coefficients[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2

float K_Fit_Coefficients[40][6] = {
    {    -20.3058f,      43.3473f,      27.8592f,     -40.4492f,     -53.4769f,       11.028f},  // K[0][0]
    {    -38.7981f,      80.3825f,      62.0245f,     -66.3855f,     -115.386f,      10.4703f},  // K[0][1]
    {   -0.599791f,     0.969982f,     -3.53717f,     0.216263f,     -6.16998f,      5.96969f},  // K[0][2]
    {   -0.234892f,      1.09479f,     -1.85676f,    -0.739107f,     -2.51527f,      2.90579f},  // K[0][3]
    {    -3.37153f,     -5.91669f,      307.344f,     -280.961f,      389.022f,     -403.749f},  // K[0][4]
    {     -1.4682f,     0.158036f,      13.0124f,      -15.018f,      33.9025f,      -18.507f},  // K[0][5]
    {    -115.047f,     -27.9595f,      231.484f,      47.1084f,     -284.135f,     -46.6293f},  // K[0][6]
    {    -3.40993f,       5.1633f,     -3.35554f,     -4.15231f,     -22.9544f,       12.182f},  // K[0][7]
    {     234.615f,     -430.824f,      983.362f,       551.93f,     -238.596f,     -1119.86f},  // K[0][8]
    {    -1.21419f,     -1.90499f,      43.8615f,      5.38723f,      -25.371f,     -38.0259f},  // K[0][9]
    {    -17.6837f,       15.359f,      49.4147f,        23.74f,     -41.8243f,     -52.6486f},  // K[1][0]
    {     -33.802f,      29.9699f,      99.7816f,      45.0126f,     -91.4686f,     -100.852f},  // K[1][1]
    {    0.594811f,      3.28915f,     -0.70858f,     -5.50633f,      6.03537f,    -0.562693f},  // K[1][2]
    {    0.239548f,      1.64477f,    -0.902688f,     -2.52549f,       2.1853f,     0.704731f},  // K[1][3]
    {    -151.043f,      279.358f,     -41.6657f,      31.0328f,     -470.589f,      146.001f},  // K[1][4]
    {    -3.55408f,     -5.23561f,      4.56802f,      18.9296f,     -31.0174f,    -0.182421f},  // K[1][5]
    {    -8.89639f,      262.203f,     -33.9195f,     -241.911f,      156.309f,     -107.575f},  // K[1][6]
    {    -1.29802f,       10.488f,     -0.57479f,     -10.5505f,      22.4316f,     -8.36176f},  // K[1][7]
    {     183.028f,      1089.04f,     -357.466f,     -1159.35f,     -376.084f,       547.67f},  // K[1][8]
    {    -1.60678f,       41.931f,     -1.03536f,     -31.8203f,     -29.9473f,      7.28382f},  // K[1][9]
    {    0.497721f,      0.25234f,       2.5638f,    0.0343306f,     -1.77306f,     -2.15693f},  // K[2][0]
    {      1.0865f,     0.497209f,      3.86878f,    -0.542899f,     -1.73716f,     -3.78608f},  // K[2][1]
    {   -0.595393f,    -0.565948f,     0.575492f,     0.522088f,    -0.131687f,    -0.372195f},  // K[2][2]
    {   -0.223963f,      -0.2752f,     0.274404f,       0.2511f,   -0.0734148f,    -0.168353f},  // K[2][3]
    {     3.42192f,      27.4372f,     -12.2853f,      -23.539f,      4.15749f,      1.50123f},  // K[2][4]
    {    0.202487f,      1.72227f,    -0.600093f,     0.187914f,    -0.585834f,    0.0363365f},  // K[2][5]
    {     3.95683f,     -14.6443f,      29.7733f,      8.18589f,    -0.997038f,       -23.17f},  // K[2][6]
    {    0.216353f,    -0.924573f,      2.14502f,      0.81368f,     -1.34494f,     0.373919f},  // K[2][7]
    {     16.7404f,     -30.9757f,     -33.2178f,      8.44866f,      55.8879f,      10.0589f},  // K[2][8]
    {     1.38764f,     -2.57431f,      -2.1105f,      1.00731f,      3.86834f,      0.32151f},  // K[2][9]
    {    0.485419f,      2.32874f,     0.486193f,      -1.8732f,     -1.85532f,    -0.235847f},  // K[3][0]
    {      1.0648f,      4.15793f,     0.200089f,     -3.76185f,     -2.28052f,     -0.15708f},  // K[3][1]
    {    0.595675f,    -0.567407f,     0.567346f,     0.365885f,     0.128877f,    -0.521392f},  // K[3][2]
    {    0.224054f,    -0.271975f,     0.274514f,     0.165333f,    0.0776299f,    -0.252423f},  // K[3][3]
    {     4.00611f,      34.6074f,     -17.3108f,     -24.2462f,     -6.70518f,      12.7716f},  // K[3][4]
    {    0.213294f,      2.30165f,     -1.01621f,     0.510047f,     -1.68601f,     0.994121f},  // K[3][5]
    {     3.51098f,     -9.47963f,      23.3419f,     -3.15192f,       9.8802f,     -23.2751f},  // K[3][6]
    {    0.204536f,    -0.504025f,      1.59676f,    -0.132365f,    -0.260374f,    0.0420083f},  // K[3][7]
    {     16.9753f,     -27.8701f,     -36.8292f,      2.44865f,      55.6332f,      16.5452f},  // K[3][8]
    {     1.38901f,     -1.83681f,     -2.83405f,  -0.00922064f,         3.83f,      1.34422f}   // K[3][9]
};

// 平衡点偏移拟合系数 Offset_Fit_Coefficients[3][6]
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq
float Offset_Fit_Coefficients[3][6] = {
    {    0.263778f,     -1.02246f, -1.08035e-15f,      1.12212f,  1.40525e-15f,  9.57244e-16f},  // theta_l_eq
    {    0.263778f, -1.72924e-15f,     -1.02246f,  2.03469e-15f,  1.82904e-15f,      1.12212f},  // theta_r_eq
    {    0.205574f, -1.49773e-15f,  -7.3906e-16f,  1.95703e-15f,  1.36727e-15f,  3.40354e-16f}   // theta_b_eq
};


float vaEstimateKF_F[4] = {1.0f, 0.001f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
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

  //使用香港大学的建模，右腿前电机ID为A2
  Left_Leg.Wheel_Motor.Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);
  Left_Leg.Front_Joint.Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Left_Leg.Back_Joint.Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Right_Leg.Wheel_Motor.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);         //4.925 = 20 * 0.3 * 15.765 / (3591/187)
  Right_Leg.Front_Joint.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Right_Leg.Back_Joint.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

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
  Roll_PID.Init(0.0f, 0.0f, 0.0f);
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

  // CMD_Data.Left_X  = (float)tmp_dr16_left_x / 100.0f;
	// CMD_Data.Left_Y  = (float)tmp_dr16_left_y / 100.0f;
  // CMD_Data.Right_X = (float)tmp_dr16_right_x / 100.0f;
	// CMD_Data.Right_Y = (float)tmp_dr16_right_y / 100.0f;
  // CMD_Data.Control_Type = (control_type & 0x03);
  #endif

}

float test_tp = 0.0f;
uint8_t start_flag = 0;
uint32_t Test_cnt = 0;
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
      if(Jump_Enable_Flag == 1 && Chassis_Control_Type == Chassis_Control_Type_UNFLLOW){
        if(Left_Leg.L0 > 0.30f && Left_Leg.alpha > 0.4f && Right_Leg.L0 > 0.30f && Right_Leg.alpha < -0.4f && Chassis_Control_Type == Chassis_Control_Type_UNFLLOW){
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
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;
    }

  }

  // if(start_flag){
  //   //测试代码    
  //   Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.20f;

  //   Left_Leg.Air_Status = Leg_Air;
  //   Right_Leg.Air_Status = Leg_Air;

  //   SynthesizeMotion();       //转向和防劈叉
  //   LengthControl();          //腿长和Roll
  //   NormalOutput();

  //   Left_Leg.VMCProject();
  //   Right_Leg.VMCProject();
  // }

  Left_Leg.Torque_Output<Left>();
  Right_Leg.Torque_Output<Right>();

}

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

  Get_Polyfit_K();

  Angle_Continuity_Process(&Target_Yaw_Angle, Yaw_Angle);

  //T(r-b)
  LQR_Out[0] = K[0][0] * (Target_X - X) + K[0][1] * (True_Target_Vx - Vx) + K[0][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[0][3] * (Target_Omega - GyroYaw) + K[0][4] * (Target_l_theta - Left_Leg.theta) + K[0][5] * (Target_l_dtheta - Left_Leg.d_theta_true)
              + K[0][6] * (Target_r_theta - Right_Leg.theta) + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta_true) 
              + K[0][8] * (Target_Pitch_Angle - Pitch_Angle) + K[0][9] * (Target_Picth_Omega - GyroPitch);

  //T(l-b)
  LQR_Out[1] = K[1][0] * (Target_X - X) + K[1][1] * (True_Target_Vx - Vx) + K[1][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[1][3] * (Target_Omega - GyroYaw) + K[1][4] * (Target_l_theta - Left_Leg.theta) + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta_true)
              + K[1][6] * (Target_r_theta - Right_Leg.theta) + K[1][7] * (Target_r_dtheta - Right_Leg.d_theta_true) 
              + K[1][8] * (Target_Pitch_Angle - Pitch_Angle) + K[1][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wr-r)
  LQR_Out[2] = K[2][0] * (Target_X - X) + K[2][1] * (True_Target_Vx - Vx) + K[2][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[2][3] * (Target_Omega - GyroYaw) + K[2][4] * (Target_l_theta - Left_Leg.theta) + K[2][5] * (Target_l_dtheta - Left_Leg.d_theta_true)
              + K[2][6] * (Target_r_theta - Right_Leg.theta) + K[2][7] * (Target_r_dtheta - Right_Leg.d_theta_true) 
              + K[2][8] * (Target_Pitch_Angle - Pitch_Angle) + K[2][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wl-l)
  LQR_Out[3] = K[3][0] * (Target_X - X) + K[3][1] * (True_Target_Vx - Vx) + K[3][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[3][3] * (Target_Omega - GyroYaw) + K[3][4] * (Target_l_theta - Left_Leg.theta) + K[3][5] * (Target_l_dtheta - Left_Leg.d_theta_true)
              + K[3][6] * (Target_r_theta - Right_Leg.theta) + K[3][7] * (Target_r_dtheta - Right_Leg.d_theta_true) 
              + K[3][8] * (Target_Pitch_Angle - Pitch_Angle) + K[3][9] * (Target_Picth_Omega - GyroPitch);
  
  if(Left_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[1] = K[1][4] * (Target_l_theta - Left_Leg.theta) + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta_true);
    LQR_Out[3] = 0.0f;
  }

  if(Right_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[0] = K[0][6] * (Target_r_theta - Right_Leg.theta) + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta_true);
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

  if(Left_Leg.Air_Status == Leg_UnAir && Right_Leg.Air_Status == Leg_UnAir && IS_NORMAL()){
    float tan_beta = ((Right_Leg.L0 - Left_Leg.L0) * arm_cos_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_sin_f32(Roll_Angle)) / ((Left_Leg.L0 - Right_Leg.L0) * arm_sin_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_cos_f32(Roll_Angle));
    float Compensite_Length = 2.0f * Chassis_Half_Width * tan_beta;
    // Compensite_Length = 0.0f;

    Left_Leg.Target_L0 = Target_Length - Compensite_Length / 2.0f;
    Right_Leg.Target_L0 = Target_Length + Compensite_Length / 2.0f;
  }
  else{
    // if(Chassis_Control_Type == Chassis_Control_Type_RESERVE){

    // }
    // else{
    //   Left_Leg.Target_L0 = Target_Length;
    //   Right_Leg.Target_L0 = Target_Length;
    // }
  }

  Left_Leg.Length_Calc();
  Right_Leg.Length_Calc();

  float L1 = (Right_Leg.L0 + Left_Leg.L0) / 2.0f;						    //质心理论上在车体中心不变
	float F1 = Robot_Mg * Vx * GyroYaw;			//质心受到的向心力

	Compensite_F0 = 1.5f * L1 * F1 / Chassis_Width;							//0.40m是车宽     转向向心力的补偿 0.7是人为的缩放因子
}

void Class_Balance_Chassis::SynthesizeMotion()
{

}

void Class_Balance_Chassis::SpeedUpdata()
{

}

void Class_Balance_Chassis::ParamUpdata()
{
  static float pre_Target_Length = 0.16f; 

  Yaw_Angle   = IMU.Get_Rad_Yaw();              
  Roll_Angle  = IMU.Get_Rad_Roll();
  Pitch_Angle = -IMU.Get_Rad_Pitch();
  Accel_X   = IMU.Get_Accel_Y_b();              //注意看是不是对应了
  Accel_Z   = IMU.Get_Accel_Z_b();
  GyroYaw   = IMU.Get_Gyro_Yaw();
  GyroRoll  = IMU.Get_Gyro_Roll();              //Roll和Pitch陀螺仪速度应该换一下（可能）
  GyroPitch = -IMU.Get_Gyro_Pitch();

  //IMU安装方式前Y，右X，这也是IMU解算算法的坐标系
  Left_Leg.GyroPitch  = GyroPitch;
  Left_Leg.Accel_Z    = Accel_Z;
  
  //香港大学建模两腿都是X朝前
  Right_Leg.GyroPitch = GyroPitch;
  Right_Leg.Accel_Z   = Accel_Z;

  // Target_X = 0.0f;          //速控的方案

  Math_Constrain(&Target_Vx, -V_MAX, V_MAX);

  //小陀螺行进
  if(Chassis_Control_Type == Chassis_Control_Type_SPIN){
    Math_Constrain(&Target_Vx, -V_MAX_SPIN, V_MAX_SPIN);
    float tmp_yaw_rad = Normalize_Angle_Radian_0_to_2PI(Yaw_Angle * PI / 180.0f);
    True_Target_Vx = Target_Vx * arm_cos_f32(tmp_yaw_rad);
    Target_X = 0.0f;
  }
  else{
    Target_X = Target_X + Target_Vx * ROBOT_TASK_DT / 1000.0f;
    True_Target_Vx = Target_Vx;
  }

  //功率限制限制速度
  Power_Control_Task(&True_Target_Vx);

  Math_Constrain(&Target_Length, Length_MIN, Length_MAX);

  if((Left_Leg.Get_Air_Status() == Leg_Air || Right_Leg.Get_Air_Status()) == Leg_Air && IS_NORMAL()){
    // Left_Leg.Target_L0 = Right_Leg.Target_L0 = Target_Length = 0.2f;
    Target_Omega = 0.0f;
    Target_Yaw_Angle = Yaw_Angle;
    Target_Length = pre_Target_Length;            //离地的时候不能变腿长
  }
  else{
    if(Chassis_Control_Type == Chassis_Control_Type_SPIN)
    {
      Target_Omega = Spin_Omega;
      Target_Yaw_Angle = Yaw_Angle;
    }
    else if(Chassis_Control_Type == Chassis_Control_Type_UNFLLOW){
      Target_Omega = 0.0f;
      Target_Yaw_Angle = Target_Yaw_Angle;
    }

    pre_Target_Length = Target_Length;
  }
  
  Left_Leg.ParamUpdata();
  Right_Leg.ParamUpdata();

  //腿部数据的更新
  //左腿
  Left_Leg.phi1 = -PI / 2.0f + Left_Leg.Back_Joint.Get_Now_Angle();                //原本应该+pi/2.0f 但是因为解包的时候偏移了+pi，所以应该-pi + pi/2.0 = -pi/2.0
  Left_Leg.phi4 = -PI / 2.0f + Left_Leg.Front_Joint.Get_Now_Angle();

  Left_Leg.d_phi1 = Left_Leg.Back_Joint.Get_Now_Omega();
  Left_Leg.d_phi4 = Left_Leg.Front_Joint.Get_Now_Omega();

  //右腿
  Right_Leg.phi1 = -PI / 2.0f - Right_Leg.Back_Joint.Get_Now_Angle();                //原本应该+pi/2.0f 但是因为解包的时候偏移了+pi，所以应该-pi + pi/2.0 = -pi/2.0
  Right_Leg.phi4 = -PI / 2.0f - Right_Leg.Front_Joint.Get_Now_Angle();

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
    aver_v = 0.0f;
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

  X = X + Vx * ROBOT_TASK_DT * 1.1f/ 1000.0f;

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
    Left_Leg.F0  = 69.0f / arm_cos_f32(Left_Leg.theta) + Left_Leg.dLength_PID.Get_Out() + Roll_PID.Get_Out() - Compensite_F0;         //F0是机体受到的向上的力
    Right_Leg.F0 = 69.0f / arm_cos_f32(Right_Leg.theta) + Right_Leg.dLength_PID.Get_Out() - Roll_PID.Get_Out() + Compensite_F0; 
  }
  else{
    Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out(); 
  }

  Left_Leg.Tp = -LQR_Out[1];
  Left_Leg.Wheel_T = -LQR_Out[3];

  Right_Leg.Tp = -LQR_Out[0];
  Right_Leg.Wheel_T = -LQR_Out[2];

  //对应上转速的建模，面朝Y负方向，逆时针为正，电机安装方式导致左轮需要加负号
  Left_Leg.Wheel_T  = -Left_Leg.Wheel_T;
  Right_Leg.Wheel_T = Right_Leg.Wheel_T;

  Math_Constrain(&Left_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Left_Leg.Wheel_T, -4.2f, 4.2f);

  Math_Constrain(&Right_Leg.F0, -200.0f, 200.0f);
  Math_Constrain(&Right_Leg.Wheel_T, -4.2f, 4.2f);
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
  static float resver_alpha_Kp = 30.0f, reserve_d_alpha_Kp = 8.0f;
  static float resver_alpha_Kd = 1.0f;
  static float Target_Left_Alpha = 0.40f, Target_Right_Alpha = 0.40f;     //左右坐标系不一致
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
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.35f;

        Left_Leg.Wheel_T = 0.0f;
        Right_Leg.Wheel_T = 0.0f;
        Left_Leg.Tp  = reserve_d_alpha_Kp * (2.5f - Left_Leg.d_alpha_true);
        Right_Leg.Tp = reserve_d_alpha_Kp * (2.5f - Right_Leg.d_alpha_true);
      }
      else{
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.35f;

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
        Left_Leg.Tp = reserve_d_alpha_Kp * (-2.5f - Left_Leg.d_alpha_true);

        if(fabs(Left_Leg.alpha - 1.5f) < 0.05f){      //接近1.5rad认为可以切换角度换控制
          Left_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process(&Target_Left_Alpha, Left_Leg.alpha);
        float Left_Error = Target_Left_Alpha - Left_Leg.alpha;
        Left_Leg.Tp  = resver_alpha_Kp * Left_Error - resver_alpha_Kd * Left_Leg.d_alpha_true;
        Left_Leg.Target_L0 = 0.20f;
      }

      if(Right_Alpha_Status == 1){     //用速度环摆正
        Right_Leg.Target_L0 = 0.30f;
        Right_Leg.Tp = reserve_d_alpha_Kp * (-2.5f - Right_Leg.d_alpha_true);

        if(fabs(Right_Leg.alpha - 1.5f) < 0.05f){
          Right_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process(&Target_Right_Alpha, Right_Leg.alpha);
        Right_Error = Target_Right_Alpha - Right_Leg.alpha;
        Right_Leg.Tp = resver_alpha_Kp * Right_Error - resver_alpha_Kd * Right_Leg.d_alpha_true;
        Right_Leg.Target_L0 = 0.20f;
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
      Target_Roll_Angle = 0.0f;
      Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.20f;
      Reserve_Status = Reserve_Disable;

      // Left_Leg.Disable();
      // Right_Leg.Disable();

      Angle_Continuity_Process(&Target_Left_Alpha, Left_Leg.alpha);
      Angle_Continuity_Process(&Target_Right_Alpha, Right_Leg.alpha);

      float Left_Error = Target_Left_Alpha - Left_Leg.alpha;
      float Right_Error = Target_Right_Alpha - Right_Leg.alpha;

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;
      Left_Leg.Tp  = resver_alpha_Kp * Left_Error;
      Right_Leg.Tp = resver_alpha_Kp * Right_Error;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      Left_Leg.Air_Status = Leg_UnAir;
      Right_Leg.Air_Status = Leg_UnAir;

      // Chassis_Control_Type = Chassis_Control_Type_RESERVE;
      // Chassis_Control_Type = Chassis_Control_Type_JUMP_1;

      #ifdef UNFLLOW_ENABLE
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;        //切回正常lqr
      #else
      Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      #endif
      
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

  float BACK_TARGET_ALPHA_L = 2.8f;   // 左腿后摆
  float BACK_TARGET_ALPHA_R = 2.8f;

  float NORMAL_ALPHA_L = 0.40f;
  float NORMAL_ALPHA_R = 0.40f;

  float Left_Target_Alpha_Omega = 2.5f;         //向后转动
  float Right_Target_Alpha_Omega = 2.5f;

  float KP_Omega_ALPHA = 8.0f;
  float KP_ALPHA = 20.0f;
  float KD_ALPHA = 0.3f;

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
      Left_Leg.Target_L0  = L_LONG;
      Right_Leg.Target_L0 = L_LONG;

      // // 角度连续处理
      Angle_Continuity_Process(&BACK_TARGET_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process(&BACK_TARGET_ALPHA_R, Right_Leg.alpha);

      float err_l = BACK_TARGET_ALPHA_L - Left_Leg.alpha;
      float err_r = BACK_TARGET_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_Omega_ALPHA * (Left_Target_Alpha_Omega - Left_Leg.d_alpha_true);
      Right_Leg.Tp = KP_Omega_ALPHA * (Right_Target_Alpha_Omega - Right_Leg.d_alpha_true);

      if(fabs(err_l) < 0.2f){
        Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha_true;
      }

      if(fabs(err_r) < 0.2f){
        Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha_true;
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
      Left_Leg.Target_L0  = L_SHORT;
      Right_Leg.Target_L0 = L_SHORT;

      Angle_Continuity_Process(&NORMAL_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process(&NORMAL_ALPHA_R, Right_Leg.alpha);

      float err_l = NORMAL_ALPHA_L - Left_Leg.alpha;
      float err_r = NORMAL_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_Omega_ALPHA * (-Left_Target_Alpha_Omega - Left_Leg.d_alpha_true);
      Right_Leg.Tp = KP_Omega_ALPHA * (-Right_Target_Alpha_Omega - Right_Leg.d_alpha_true);

      if(fabs(err_l) < 0.2f){
        Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha_true;
      }

      if(fabs(err_r) < 0.2f){
        Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha_true;
      }

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

      Angle_Continuity_Process(&NORMAL_ALPHA_L, Left_Leg.alpha);
      Angle_Continuity_Process(&NORMAL_ALPHA_R, Right_Leg.alpha);

      float err_l = NORMAL_ALPHA_L - Left_Leg.alpha;
      float err_r = NORMAL_ALPHA_R - Right_Leg.alpha;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha_true;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha_true;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      // Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;
      // Chassis_Control_Type = Chassis_Control_Type_JUMP_1;

      break;
    }

    default:{
      state_cnt = 0;
      jump_state = JUMP_BACK_SWING;
      break;
    }
  }
}