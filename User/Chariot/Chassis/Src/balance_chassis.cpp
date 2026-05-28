#include "balance_chassis.h"

#include "user_lib.h"
#include "drv_math.h"
#include "cantransmit_msg.h"

#define RAD_TO_ANGLE (180.0f / 3.14159f)

// K矩阵拟合系数 K_Fit_Coefficients[40][6]
// 第n个K元素: K_n = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
float K_Fit_Coefficients[40][6] = {
    {    -86.0654f,      208.196f,     -13.6151f,     -96.8805f,     -324.019f,      266.625f},  // K[0][0]
    {    -75.9694f,      218.589f,      -16.384f,     -86.8254f,     -340.663f,       235.33f},  // K[0][1]
    {    -3.84703f,     -4.89002f,     -28.6622f,      7.03207f,     -10.2345f,      37.8851f},  // K[0][2]
    {   -0.711998f,      5.40623f,     -12.5458f,     -8.66345f,    0.0847037f,      12.9111f},  // K[0][3]
    {    -6.08593f,      60.4018f,       62.339f,     -37.8672f,      8.35883f,     -73.9267f},  // K[0][4]
    {   -0.775625f,      2.40378f,      4.39168f,     0.709417f,      1.71046f,     -3.92949f},  // K[0][5]
    {     -188.03f,      6.03719f,      315.823f,      11.3315f,     -45.0586f,     -209.546f},  // K[0][6]
    {    -3.44061f,      2.58984f,     -2.54192f,     -1.74741f,     -3.69152f,      1.73302f},  // K[0][7]
    {     30.2958f,     -82.2591f,      424.255f,      106.797f,     -89.8547f,     -428.552f},  // K[0][8]
    {    -5.85738f,      18.9656f,      34.7914f,     -8.31388f,     -43.9869f,     -16.4344f},  // K[0][9]
    {    -86.0654f,     -13.6151f,      208.196f,      266.625f,     -324.019f,     -96.8805f},  // K[1][0]
    {    -75.9694f,      -16.384f,      218.589f,       235.33f,     -340.663f,     -86.8254f},  // K[1][1]
    {     3.84703f,      28.6622f,      4.89002f,     -37.8851f,      10.2345f,     -7.03207f},  // K[1][2]
    {    0.711998f,      12.5458f,     -5.40623f,     -12.9111f,   -0.0847037f,      8.66345f},  // K[1][3]
    {     -188.03f,      315.823f,      6.03719f,     -209.546f,     -45.0586f,      11.3315f},  // K[1][4]
    {    -3.44061f,     -2.54192f,      2.58984f,      1.73302f,     -3.69152f,     -1.74741f},  // K[1][5]
    {    -6.08593f,       62.339f,      60.4018f,     -73.9267f,      8.35883f,     -37.8672f},  // K[1][6]
    {   -0.775625f,      4.39168f,      2.40378f,     -3.92949f,      1.71046f,     0.709417f},  // K[1][7]
    {     30.2958f,      424.255f,     -82.2591f,     -428.552f,     -89.8547f,      106.797f},  // K[1][8]
    {    -5.85738f,      34.7914f,      18.9656f,     -16.4344f,     -43.9869f,     -8.31388f},  // K[1][9]
    {     4.72887f,     -45.6439f,       109.92f,      19.5555f,      21.5612f,     -118.107f},  // K[2][0]
    {     7.06986f,     -34.5044f,      74.2529f,      9.16089f,      27.4475f,     -84.0092f},  // K[2][1]
    {     -3.5195f,      -6.8466f,      12.1924f,      9.62517f,     -4.30225f,     -7.87206f},  // K[2][2]
    {   -0.617531f,     -3.34838f,      3.81316f,      5.29058f,     -4.75958f,    -0.329466f},  // K[2][3]
    {     3.61436f,      24.9964f,     -8.78831f,      -27.048f,     -4.07832f,      6.68051f},  // K[2][4]
    {    0.259609f,     0.792794f,    -0.219382f,     0.222556f,     0.165259f,    -0.621936f},  // K[2][5]
    {     16.4702f,     -13.1301f,       137.13f,       18.611f,     -20.4804f,     -129.761f},  // K[2][6]
    {    0.345977f,     -1.26199f,      5.12038f,      2.04062f,     -3.01689f,     0.460358f},  // K[2][7]
    {     30.7689f,     -45.2845f,     -24.9503f,      37.0548f,      38.2141f,     -20.7176f},  // K[2][8]
    {     3.87378f,     -10.0949f,     0.167977f,      7.84706f,      7.98542f,     -6.18537f},  // K[2][9]
    {     4.72887f,       109.92f,     -45.6439f,     -118.107f,      21.5612f,      19.5555f},  // K[3][0]
    {     7.06986f,      74.2529f,     -34.5044f,     -84.0092f,      27.4475f,      9.16089f},  // K[3][1]
    {      3.5195f,     -12.1924f,       6.8466f,      7.87206f,      4.30225f,     -9.62517f},  // K[3][2]
    {    0.617531f,     -3.81316f,      3.34838f,     0.329466f,      4.75958f,     -5.29058f},  // K[3][3]
    {     16.4702f,       137.13f,     -13.1301f,     -129.761f,     -20.4804f,       18.611f},  // K[3][4]
    {    0.345977f,      5.12038f,     -1.26199f,     0.460358f,     -3.01689f,      2.04062f},  // K[3][5]
    {     3.61436f,     -8.78831f,      24.9964f,      6.68051f,     -4.07832f,      -27.048f},  // K[3][6]
    {    0.259609f,    -0.219382f,     0.792794f,    -0.621936f,     0.165259f,     0.222556f},  // K[3][7]
    {     30.7689f,     -24.9503f,     -45.2845f,     -20.7176f,      38.2141f,      37.0548f},  // K[3][8]
    {     3.87378f,     0.167977f,     -10.0949f,     -6.18537f,      7.98542f,      7.84706f}   // K[3][9]
};
// 平衡点偏移拟合系数 Offset_Fit_Coefficients[3][6]
// [0]: theta_l_eq, [1]: theta_r_eq, [2]: theta_b_eq

float Offset_Fit_Coefficients[3][6] = {
    {    0.198437f,     -0.78094f, -9.05786e-16f,     0.871354f,  1.51919e-15f,  6.86025e-16f},  // theta_l_eq
    {    0.198437f, -1.63352e-15f,     -0.78094f,  1.89475e-15f,   1.8939e-15f,     0.871354f},  // theta_r_eq
    {          -0.0f,            0.0f,            0.0f,           -0.0f,           -0.0f,            0.0f}   // theta_b_eq
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
  Left_Leg.Front_Joint.Init(&hfdcan1, DM_Motor_ID_0x01, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Left_Leg.Back_Joint.Init(&hfdcan1, DM_Motor_ID_0x02, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Right_Leg.Wheel_Motor.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);         //4.925 = 20 * 0.3 * 15.765 / (3591/187)
  Right_Leg.Front_Joint.Init(&hfdcan2, DM_Motor_ID_0x01, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Right_Leg.Back_Joint.Init(&hfdcan2, DM_Motor_ID_0x02, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Left_Leg.Init();
  Right_Leg.Init();

  Supercap.Init(&hfdcan2, 60.0f);
  Referee.Init(&huart10);

  Daemon_Init_Config_s daemon_init_config_instance_1 = Get_DaemonInitConfig_s(100, NULL, NULL);
  Referee_Daemon = DaemonRegister(daemon_init_config_instance_1);

  Daemon_Init_Config_s daemon_init_config_instance_2 = Get_DaemonInitConfig_s(10, NULL, NULL);
  Supercap_Daemon = DaemonRegister(daemon_init_config_instance_2);

  CanTransmit_TaskRegister(1, 0x04, CAN1_0xxf1_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(2, 0x05, CAN1_0xxf2_Tx_Data, &hfdcan1);
  CanTransmit_TaskRegister(3, 0x04, CAN2_0xxf1_Tx_Data, &hfdcan2);
  CanTransmit_TaskRegister(4, 0x05, CAN2_0xxf2_Tx_Data, &hfdcan2);

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

void Class_Balance_Chassis::Set_Jump_Enable_Flag(uint8_t __Jump_Enable_Flag)
{
  Jump_Enable_Flag = __Jump_Enable_Flag;
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
  Target_CMD_Data.Complex_Flag        = Gimbal_To_Chassis_Data.Complex_Flag;
  Target_CMD_Data.Target_Control_Type = Gimbal_To_Chassis_Data.control_type;

  MiniPC_Aim                = ((Target_CMD_Data.Complex_Flag & 0x50) >> 5);  
  Gimbal_Control_Type       = (Enum_Gimbal_Control_Type)((Target_CMD_Data.Complex_Flag & 0x18) >> 3);
  Sprint_Status             = (Enum_Sprint_Status)((Target_CMD_Data.Complex_Flag & (1<<7)) >> 7);
  Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)((Target_CMD_Data.Complex_Flag & (1<<2)) != 0);
  Fric_Status               = (Enum_Friction_Control_Type)(Gimbal_To_Chassis_Data.Complex_Flag_1 & 0x01);
  Gimbal_Pitch_Angle        = Gimbal_To_Chassis_Data.Pitch_uint8;

  Switch_Chassis_Forward    = ((Balance_Chassis.Target_CMD_Data.Complex_Flag & 0x01));

  #endif

}

float test_tp = 0.0f;
uint8_t start_flag = 0;
uint32_t Test_cnt = 0;
float tmp_target_length = 0.20f;
float Left_Right_Err;
void Class_Balance_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
  //控制量的更改和模式什么的一般在中断里边实现
  Test_cnt ++;

  CAN_Chassis_To_Gimbal_Data_Process();

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
        if(Left_Leg.L0 > 0.33f && Left_Leg.theta > 0.45f && Right_Leg.L0 > 0.33f && Right_Leg.theta > 0.45f){
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

  // if(Target_CMD_Data.Target_Control_Type == Chassis_Control_Type_FLLOW){
  //   //测试代码    
  //   // Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = tmp_target_length;
  //   Reserve_FSM();            //自救的逻辑还需要完善，比如腿在前方的时候
  //   LengthControl();          //腿长和Roll
  //   ReserveOutput();          //自救模式下的力矩输出

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

float tmp_X_Err, tmp_Pitch_Err, tmp_Left_Theta_Err, tmp_Right_Theta_Err;
float tmp_wt;
float LQR_Tp_L[10];
float pitch_max;
float pitch_min;
float T_other;
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
    Target_l_theta = -0.05f;
  }

  if(Right_Leg.Air_Status == Leg_Air){
    Target_r_theta = -0.05f;
  }

  Get_Polyfit_K();

  tmp_X_Err = (Target_X - X);
  tmp_Pitch_Err = (Target_Pitch_Angle - Pitch_Angle);
  tmp_Left_Theta_Err = (Target_l_theta - Left_Leg.theta);
  tmp_Right_Theta_Err = (Target_r_theta - Right_Leg.theta);
  // float a =  -0.20f * Vx + 0.50f;
  // if(a < 0.1f){
  //   a = 0.1f;
  // }
  Math_Constrain(&tmp_X_Err, -0.5f, 0.5f);
  Math_Constrain(&tmp_Pitch_Err, -0.3f, 0.3f);
  Math_Constrain(&tmp_Left_Theta_Err, -0.6f, 0.6f);
  Math_Constrain(&tmp_Right_Theta_Err, -0.6f, 0.6f);

  LQR_Output_Calc(tmp_X_Err, tmp_Left_Theta_Err, tmp_Right_Theta_Err, tmp_Pitch_Err);
  
  // if(IS_NORMAL()){
  //   if(K[2][8] * tmp_Pitch_Err * LQR_Out[2] > 0.0f){     //说明转矩较大是Pitch引起的

  //     float T_Max = 4.9f;
  //     T_other = LQR_Out[2] - K[2][8] * tmp_Pitch_Err;

  //     if(K[2][8] > 0.0f){
  //       pitch_max = (T_Max - T_other) / K[2][8];
  //       pitch_min = (-T_Max - T_other) / K[2][8];
  //     }
  //     else{
  //       pitch_max = (-T_Max - T_other) / K[2][8];
  //       pitch_min = (T_Max - T_other) / K[2][8];
  //     }

  //     if(pitch_max < 0.05f){
  //       pitch_max = 0.05f;
  //     }
  //     if(pitch_min > -0.05f){
  //       pitch_min = -0.05f;
  //     }

  //     if(fabs(tmp_Pitch_Err) > 0.15f){
  //       if(tmp_Pitch_Err > pitch_max){
  //         tmp_Pitch_Err = pitch_max;
  //       }
  //       else if(tmp_Pitch_Err < pitch_min){
  //         tmp_Pitch_Err = pitch_min;
  //       }
  //       // LQR_Output_Calc(tmp_Pitch_Err);
  //     }
  //     else{
  //       // Emergency_Stop_Flag = 1;
  //     }

  //   }
  // }

  tmp_wt = LQR_Out[2];
  LQR_Tp_L[0] = K[2][0] * (Target_X - X);
  LQR_Tp_L[1] = K[2][1] * (True_Target_Vx - Vx);
  LQR_Tp_L[2] = K[2][2] * (Target_Yaw_Angle - Yaw_Angle);
  LQR_Tp_L[3] = K[2][3] * (Target_Omega - GyroYaw);
  LQR_Tp_L[4] = K[2][4] * (Target_l_theta - Left_Leg.theta);
  LQR_Tp_L[5] = K[2][5] * (Target_l_dtheta - Left_Leg.d_theta);
  LQR_Tp_L[6] = K[2][6] * (Target_r_theta - Right_Leg.theta);
  LQR_Tp_L[7] = K[2][7] * (Target_r_dtheta - Right_Leg.d_theta);
  LQR_Tp_L[8] = K[2][8] * tmp_Pitch_Err;
  LQR_Tp_L[9] = K[2][9] * (Target_Picth_Omega - GyroPitch);
  
  if(Left_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[1] = K[1][4] * (Target_l_theta - Left_Leg.theta) + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta);
    LQR_Out[3] = 0.0f;
  }

  if(Right_Leg.Get_Air_Status() == Leg_Air){
    LQR_Out[0] = K[0][6] * (Target_r_theta - Right_Leg.theta) + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta);
    LQR_Out[2] = 0.0f;
  }

}

void Class_Balance_Chassis::LQR_Output_Calc(float __tmp_x_err, float __tmp_left_theta_err, float __tmp_right_theta_err, float __tmp_pitch_err)
{
  //T(r-b)
  LQR_Out[0] = K[0][0] * __tmp_x_err + K[0][1] * (True_Target_Vx - Vx) + K[0][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[0][3] * (Target_Omega - GyroYaw) + K[0][4] * __tmp_left_theta_err + K[0][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[0][6] * __tmp_right_theta_err + K[0][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[0][8] * __tmp_pitch_err + K[0][9] * (Target_Picth_Omega - GyroPitch);

  //T(l-b)
  LQR_Out[1] = K[1][0] * __tmp_x_err + K[1][1] * (True_Target_Vx - Vx) + K[1][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[1][3] * (Target_Omega - GyroYaw) + K[1][4] * __tmp_left_theta_err + K[1][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[1][6] * __tmp_right_theta_err + K[1][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[1][8] * __tmp_pitch_err + K[1][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wr-r)
  LQR_Out[2] = K[2][0] * __tmp_x_err + K[2][1] * (True_Target_Vx - Vx) + K[2][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[2][3] * (Target_Omega - GyroYaw) + K[2][4] * __tmp_left_theta_err + K[2][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[2][6] * __tmp_right_theta_err + K[2][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[2][8] * __tmp_pitch_err + K[2][9] * (Target_Picth_Omega - GyroPitch);
  
  //T(wl-l)
  LQR_Out[3] = K[3][0] * __tmp_x_err + K[3][1] * (True_Target_Vx - Vx) + K[3][2] * (Target_Yaw_Angle - Yaw_Angle)
              + K[3][3] * (Target_Omega - GyroYaw) + K[3][4] * __tmp_left_theta_err + K[3][5] * (Target_l_dtheta - Left_Leg.d_theta)
              + K[3][6] * __tmp_right_theta_err + K[3][7] * (Target_r_dtheta - Right_Leg.d_theta) 
              + K[3][8] * __tmp_pitch_err + K[3][9] * (Target_Picth_Omega - GyroPitch);
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

  // tmp_left_target_l0 = Target_Length;
  // tmp_right_target_l0 = Target_Length;

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
  // Compensite_F0 = 0.0f;
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

  if(Left_Leg.Air_Status == Leg_Air || Right_Leg.Air_Status == Leg_Air){
    Target_X = 0.0f;
    X = 0.0f;

    Target_Vx = 0.0f;
    Vx = 0.0f;
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
    Left_Leg.F0  = 35.0f;//Left_Leg.dLength_PID.Get_Out() * 0.3f;
    Right_Leg.F0 = 35.0f;//Right_Leg.dLength_PID.Get_Out() * 0.3f; 
  }
  else if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_Air){
    Left_Leg.F0  = 100.0f / arm_cos_f32(Left_Leg.theta) + Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = 35.0f;//Right_Leg.dLength_PID.Get_Out() * 0.3f; 
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air && Right_Leg.Get_Air_Status() == Leg_UnAir){
    Left_Leg.F0  = 35.0f;//Left_Leg.dLength_PID.Get_Out() * 0.3f;
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

void Class_Balance_Chassis::CAN_Chassis_To_Gimbal_Data_Process()
{
  Chassis_To_Gimbal_Data.Chassis_Control_Type = Chassis_Control_Type;

  memcpy(CAN_Chassis_Tx_Data, &Chassis_To_Gimbal_Data, 8);
}

uint32_t status_cnt = 0;
uint32_t xxx1 = 0;
float reserve_right_theta_I = 0.0f, reserve_left_theta_I = 0.0f;
void Class_Balance_Chassis::Reserve_FSM()
{
  static uint8_t Theta_Synchronize = 0, Theta_Synchronize_FLag = 0, Pre_Theta_Synchronize = 0;   // 正在进行同步  2 保证同步只进入一次
  static uint8_t Left_Alpha_Status = 0, Right_Alpha_Status = 0;     //切到1的时候代表偏角过大，先速度环在切回角度环
  static float resver_theta_Kp = 50.0f, reserve_d_alpha_Kp = 3.0f, reserve_d_alpha_Kp_1 = 5.0f;
  static float Theta_Synchronize_Kp = 2.0f;
  static float resver_theta_Kd = 0.7f;
  static float Target_Left_theta = 0.10f, Target_Right_theta = 0.10f;     //左右坐标系不一致

  Left_Leg.Air_Status = Leg_UnAir;
  Right_Leg.Air_Status = Leg_UnAir;

  switch (Reserve_Status)
  {
    case(Reserve_Disable):
    {
      xxx1 = 2;
      status_cnt = 0;
      reserve_right_theta_I = 0;
      reserve_left_theta_I = 0;
      Theta_Synchronize = 0;
      Theta_Synchronize_FLag = 0;
      Pre_Theta_Synchronize = 0;
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
      if(Pitch_Angle < -50.0f / 57.3f || Pitch_Angle > 50.0f / 57.3f){                //仍然是翻车状态
        status_cnt = 0;
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.33f;

        Left_Leg.Wheel_T = 0.0f;
        Right_Leg.Wheel_T = 0.0f;

        Left_Right_Err = Normalize_Angle_Radian_PI_to_PI(Left_Leg.alpha - Right_Leg.alpha);

        if(fabs(Left_Right_Err) > 0.3 && Theta_Synchronize_FLag == 0){
          Theta_Synchronize = 1;
        }
        else{
          Theta_Synchronize = 0;
          Theta_Synchronize_FLag = 1;
        }

        if(Theta_Synchronize){
          xxx1 = 5;
            Left_Leg.Target_L0 = 0.16f;
            reserve_left_theta_I += (-4.0f - Left_Leg.d_alpha) * 0.02f;
            Left_Leg.Tp = reserve_d_alpha_Kp * (-4.0f - Left_Leg.d_alpha) + reserve_left_theta_I * 1.2f;
            Right_Leg.Tp = reserve_d_alpha_Kp * (0.0f - Right_Leg.d_alpha);
        }
        else{
          xxx1 = 6;
          if(Theta_Synchronize == 0 && Pre_Theta_Synchronize == 1){     //从正在同步切换到不需要同步了，清零积分
            reserve_right_theta_I = 0.0f;
            reserve_left_theta_I = 0.0f;
          }

          reserve_left_theta_I += (4.0f - Left_Leg.d_alpha) * 0.02f;
          reserve_right_theta_I += (4.0f - Right_Leg.d_alpha) * 0.02f;

          Left_Leg.Tp  = reserve_d_alpha_Kp_1 * (4.0f - Left_Leg.d_alpha) + reserve_left_theta_I * 2.0f - Theta_Synchronize_Kp * Left_Right_Err;
          Right_Leg.Tp = reserve_d_alpha_Kp_1 * (4.0f - Right_Leg.d_alpha) + reserve_right_theta_I * 2.0f + Theta_Synchronize_Kp * Left_Right_Err;
        }
      }
      else{
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.33f;

        Left_Leg.Wheel_T = 0.0f;
        Right_Leg.Wheel_T = 0.0f;
        Left_Leg.Tp  = 0.0f;
        Right_Leg.Tp = 0.0f;
        reserve_right_theta_I = 0.0f;
        reserve_left_theta_I = 0.0f;
      }

      Pre_Theta_Synchronize = Theta_Synchronize;

      if(fabs(Pitch_Angle) < 50.0f / 57.3f){
        status_cnt ++;
      }

      //状态切换时清零计数
      if(status_cnt > 300){
        status_cnt = 0;
        Reserve_Status = Reserve_Status_2; 
      }

      Math_Constrain(&Left_Leg.Tp, -30.0f, 30.0f);
      Math_Constrain(&Right_Leg.Tp, -30.0f, 30.0f);

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
        Left_Leg.Target_L0 = 0.23f;
        Left_Leg.Tp = reserve_d_alpha_Kp * (-5.0f - Left_Leg.d_alpha);

        if(fabs(Left_Leg.alpha - 1.5f) < 0.05f){      //接近1.5rad认为可以切换角度换控制
          Left_Alpha_Status = 0;
        }
      }
      else{
        Angle_Continuity_Process_Rad(&Target_Left_theta, Left_Leg.theta);
        Left_Error = Target_Left_theta - Left_Leg.theta;
        Left_Leg.Tp  = resver_theta_Kp * Left_Error - resver_theta_Kd * Left_Leg.d_theta;
        Left_Leg.Target_L0 = 0.16f;
      }

      if(Right_Alpha_Status == 1){     //用速度环摆正
        Right_Leg.Target_L0 = 0.23f;
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

      Math_Constrain(&Left_Leg.Tp, -20.0f, 20.0f);
      Math_Constrain(&Right_Leg.Tp, -20.0f, 20.0f);

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;

      //左右腿都是角度换算了，并且都在合理范围内了，就认为自救完成了
      if(fabs(Right_Error) < 0.3f && fabs(Left_Error) < 0.3f && Left_Alpha_Status == 0 && Right_Alpha_Status == 0){
        status_cnt ++;
      }

      xxx1 = 1;

      if(status_cnt > 100){
        status_cnt = 0;
        xxx1 = 10;
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

      xxx1 = 20;

      Target_X  = 0.0f;
      Target_Vx = 0.0f;
      Theta_Synchronize = 0;
      Theta_Synchronize_FLag = 0;
      Pre_Theta_Synchronize = 0;
      reserve_right_theta_I = 0.0f;
      reserve_left_theta_I = 0.0f;
      Target_Yaw_Angle = Yaw_Angle;
      Chassis_Stable_Count = 0;
      Target_Roll_Angle = 0.0f;
      Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.18f;
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

      //起来的时候判断一下最优的随动方向
      float tmp_delta_rad = -Balance_Chassis.Motor_Yaw.Get_Now_Radian() + Reference_Rad;
      tmp_delta_rad = Normalize_Angle_Radian_PI_to_PI(tmp_delta_rad);

      if(fabsf(tmp_delta_rad) < PI / 2.0f){
        Balance_Chassis.Chassis_Forward = 1.0f;
      }
      else{
        Balance_Chassis.Chassis_Forward = -1.0f;
      }

      V_EstimateKF_Init();    

      #ifdef UNFLLOW_ENABLE
      Chassis_Control_Type = Chassis_Control_Type_UNFLLOW;        //切回正常lqr
      #else
      Chassis_Control_Type = Chassis_Control_Type_FLLOW;
      #endif

      // Chassis_Control_Type = Chassis_Control_Type_RESERVE;
      
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

  float NORMAL_Theta_L = 0.08f;
  float NORMAL_Theta_R = 0.08f;

  float Left_Target_Alpha_Omega = 10.0f;         //向后转动
  float Right_Target_Alpha_Omega = 10.0f;

  float KP_Omega_ALPHA = 10.0f;
  float KP_ALPHA = 50.0f;
  float KD_ALPHA = 0.5f;

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

      if(fabs(err_l) < 0.4f){
        Left_Complete_Flag = 1;
      }
      if(Left_Complete_Flag){
        cnt111 ++;
        Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_alpha;
      }

      if(fabs(err_r) < 0.4f){
        Right_Complete_Flag = 1;
      }
      if(Right_Complete_Flag){
        Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_alpha;
      }

      Math_Constrain(&Left_Leg.Tp, -20.0f, 20.0f);
      Math_Constrain(&Right_Leg.Tp, -20.0f, 20.0f);

      // 到位判断
      if(fabs(err_l) < 0.2f && fabs(err_r) < 0.2f)
      {
        state_cnt += ROBOT_TASK_DT;
      }
      else{
        state_cnt = 0;
      }

      if(state_cnt > 50)
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

      Math_Constrain(&Left_Leg.Tp, -20.0f, 20.0f);
      Math_Constrain(&Right_Leg.Tp, -20.0f, 20.0f);

      if(fabs(err_l) < 0.2f && fabs(err_r) < 0.2f)
      {
        state_cnt += ROBOT_TASK_DT;
      }
      else{
        state_cnt = 0;
      }

      if(state_cnt > 150)
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
      Target_X = 0.0f;
      True_Target_Vx = Target_Vx = 0.0f;
      Target_Roll_Angle = 0.0f;
      Target_Yaw_Angle = Yaw_Angle;

      Target_Length = 0.20f;
      Left_Leg.Target_L0  = 0.20f;
      Right_Leg.Target_L0 = 0.20f;
      V_EstimateKF_Init();          //重启状态

      // Left_Leg.Tp = 0.0f;
      // Right_Leg.Tp = 0.0f;

      Angle_Continuity_Process_Rad(&NORMAL_Theta_L, Left_Leg.theta);
      Angle_Continuity_Process_Rad(&NORMAL_Theta_R, Right_Leg.theta);

      float err_l = NORMAL_Theta_L - Left_Leg.theta;
      float err_r = NORMAL_Theta_R - Right_Leg.theta;

      Left_Leg.Tp  = KP_ALPHA * err_l - KD_ALPHA * Left_Leg.d_theta;
      Right_Leg.Tp = KP_ALPHA * err_r - KD_ALPHA * Right_Leg.d_theta;

      Math_Constrain(&Left_Leg.Tp, -20.0f, 20.0f);
      Math_Constrain(&Right_Leg.Tp, -20.0f, 20.0f);

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