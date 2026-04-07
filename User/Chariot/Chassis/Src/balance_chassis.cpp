#include "balance_chassis.h"

#include "user_lib.h"
#include "drv_math.h"
#include "cantransmit_msg.h"

#define RAD_TO_ANGLE (180.0f / 3.14159f)


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

  Left_Leg.Wheel_Motor.Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);
  Left_Leg.Front_Joint.Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Left_Leg.Back_Joint.Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

  Right_Leg.Wheel_Motor.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 15.765f, 4.925f);         //4.925 = 20 * 0.3 * 15.765 / (3591/187)
  Right_Leg.Front_Joint.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);
  Right_Leg.Back_Joint.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_TORQUE, 0, 45.0f, JOINT_MAX_TORQUE);

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
uint8_t Test_Flag_2 = 1;
void Class_Balance_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
  //控制量的更改和模式什么的一般在中断里边实现
  Test_Flag_2 = 0;

  ParamUpdata();

  SpeedEstimate();      //进行速度的观测

  Left_Leg.VMC_Calc();
  Right_Leg.VMC_Calc();

  SpeedUpdata();        //速度传参更新

  Left_Leg.LQR_Calc();
  Right_Leg.LQR_Calc();

  if(Chassis_Control_Type == Chassis_Control_Type_DISABLE){
    Chassis_Disable();
  }
  else if(Chassis_Control_Type == Chassis_Control_Type_RESERVE){
    Reserve_FSM();
    LengthControl();          //腿长和Roll
    ReserveOutput();          //自救模式下的力矩输出

    Left_Leg.VMCProject();
    Right_Leg.VMCProject();
  }
  else{             //随动 小陀螺 跳跃 不随动
    SynthesizeMotion();       //转向和防劈叉
    LengthControl();          //腿长和Roll
    NormalOutput();

    Left_Leg.VMCProject();
    Right_Leg.VMCProject();   

    if(Chassis_Stable_Count >= 2000){
      Left_Leg.ForceSlove();
      Right_Leg.ForceSlove();
    }
    else{
      Left_Leg.Air_Status = Leg_UnAir;
      Right_Leg.Air_Status = Leg_UnAir;
      Chassis_Stable_Count++;
    }

  }

  if(start_flag){
    //测试代码
    // LengthControl();
    // SynthesizeMotion();       //转向和防劈叉
    // Right_Leg.Tp = Right_Leg.Get_LQR_Tp() + Tp_PID.Get_Out();
    // Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out();
    // Left_Leg.Tp = Left_Leg.Get_LQR_Tp() + Tp_PID.Get_Out();
    // Left_Leg.F0 = Left_Leg.dLength_PID.Get_Out();

    // Right_Leg.Wheel_T = 0.0f;//Turn_Omega_PID.Get_Out();

    // Left_Leg.VMCProject();
    // Right_Leg.VMCProject();

    // Reserve_FSM();
    // LengthControl();          //腿长和Roll
    // ReserveOutput();          //自救模式下的力矩输出

    // Left_Leg.VMCProject();
    // Right_Leg.VMCProject();

  }

  Left_Leg.Torque_Output();
  Right_Leg.Torque_Output();

}

void Class_Balance_Chassis::LengthControl()
{
  //对于Roll的补偿还有另一种计算的方式
  Roll_PID.Set_Now(Roll_Angle);
  Roll_PID.Set_D_Extern_Value(-GyroRoll);       //error的变换率 （Target - Now）
  Roll_PID.Set_Target(0.0f);
  Roll_PID.TIM_Adjust_PeriodElapsedCallback();

  if(Left_Leg.Air_Status == Leg_UnAir && Right_Leg.Air_Status == Leg_UnAir){
    float tan_beta = ((Right_Leg.L0 - Left_Leg.L0) * arm_cos_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_sin_f32(Roll_Angle)) / ((Left_Leg.L0 - Right_Leg.L0) * arm_sin_f32(Roll_Angle) + 2.0f * Chassis_Half_Width * arm_cos_f32(Roll_Angle));
    float Compensite_Length = 2.0f * Chassis_Half_Width * tan_beta;
    // Compensite_Length = 0.0f;

    Left_Leg.Target_L0 = Target_Length - Compensite_Length / 2.0f;
    Right_Leg.Target_L0 = Target_Length + Compensite_Length / 2.0f;
  }
  else{
    Left_Leg.Target_L0 = Target_Length;
    Right_Leg.Target_L0 = Target_Length;
  }

  Left_Leg.Length_Calc();
  Right_Leg.Length_Calc();

  float L1 = (Right_Leg.L0 + Left_Leg.L0) / 2.0f;						    //质心理论上在车体中心不变
	float F1 = Robot_Mg * Vx * GyroYaw;			//质心受到的向心力

	Compensite_F0 = 1.5f * L1 * F1 / Chassis_Width;							//0.40m是车宽     转向向心力的补偿 0.7是人为的缩放因子
}

void Class_Balance_Chassis::SynthesizeMotion()
{
  theta_error = Left_Leg.theta + Right_Leg.theta;
  Tp_PID.Set_Now(theta_error);
  Turn_Angle_PID.Set_Now(Yaw_Angle);
  Turn_Omega_PID.Set_Now(GyroYaw);

  Tp_PID.Set_Target(0.0f);
  Angle_Continuity_Process(&Target_Yaw_Angle, Yaw_Angle);
  Turn_Angle_PID.Set_Target(Target_Yaw_Angle);

  Tp_PID.TIM_Adjust_PeriodElapsedCallback();

  //Disable直接不输出了
  if(Chassis_Control_Type == Chassis_Control_Type_SPIN){
    Turn_Omega_PID.Set_Target(Target_Omega);
    Turn_Omega_PID.TIM_Adjust_PeriodElapsedCallback();
  }
  else{       //JUMP RESERVE FLLOW
    Turn_Angle_PID.TIM_Adjust_PeriodElapsedCallback();
    Turn_Omega_PID.Set_Target(Turn_Angle_PID.Get_Out());
    Turn_Omega_PID.TIM_Adjust_PeriodElapsedCallback();
  }

  

  if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){        //两个腿都在地上
    
  }
  else{
    
  }

}

void Class_Balance_Chassis::SpeedUpdata()
{
  Left_Leg.X   = X;
  Left_Leg.Vx  = Vx;
  Right_Leg.X  = -X;
  Right_Leg.Vx = -Vx;

  float theta_offset = 0.08f + (Target_Vx - Vx) * 0.08f;
  Math_Constrain(&theta_offset, -0.15f, 0.15f);

  Left_Leg.pitch_offset = -0.03f;
  Right_Leg.pitch_offset = 0.03f;
  Left_Leg.theta_offset = theta_offset;
  Right_Leg.theta_offset = -theta_offset;

}

void Class_Balance_Chassis::ParamUpdata()
{
  static float pre_Target_Length = 0.16f; 

  Yaw_Angle   = IMU.Get_Angle_Yaw();              
  Roll_Angle  = IMU.Get_Rad_Roll();
  Pitch_Angle = IMU.Get_Rad_Pitch();
  Accel_X   = IMU.Get_Accel_Y_n();              //注意看是不是对应了
  Accel_Z   = IMU.Get_Accel_Z_n();
  GyroYaw   = IMU.Get_Gyro_Yaw();
  GyroRoll  = IMU.Get_Gyro_Roll();              //Roll和Pitch陀螺仪速度应该换一下（可能）
  GyroPitch = IMU.Get_Gyro_Pitch();

  Target_X = 0.0f;          //速控的方案

  Math_Constrain(&Target_Vx, -V_MAX, V_MAX);
  Math_Constrain(&Target_Length, Length_MIN, Length_MAX);

  // TD_SetTarget(&Target_Vx_Td, Target_Vx);
  // TD_Update(&Target_Vx_Td);

  //IMU安装方式前Y，右X，这也是IMU解算算法的坐标系
  Left_Leg.Pitch      = Pitch_Angle;
  Left_Leg.Target_X   = Target_X;
  Left_Leg.Target_Vx  = Target_Vx;
  Left_Leg.GyroPitch  = GyroPitch;
  Left_Leg.Accel_Z    = Accel_Z;
  
  
  Right_Leg.Pitch     = -Pitch_Angle;
  Right_Leg.Target_X  = -Target_X;
  Right_Leg.Target_Vx = -Target_Vx;
  Right_Leg.GyroPitch = -GyroPitch;
  Right_Leg.Accel_Z   = Accel_Z;
  
  if(Left_Leg.Get_Air_Status() == Leg_Air){
    // Left_Leg.Target_L0 = 0.2f;
  }
  if(Right_Leg.Get_Air_Status() == Leg_Air){
    // Right_Leg.Target_L0 = 0.2f;
  }

  if(Left_Leg.Get_Air_Status() == Leg_Air || Right_Leg.Get_Air_Status() == Leg_Air){
    // Left_Leg.Target_L0 = Right_Leg.Target_L0 = Target_Length = 0.2f;
    Target_Yaw_Angle = Yaw_Angle;
    Target_Omega = 0.0f;

    Target_Length = pre_Target_Length;            //离地的时候不能变腿长

  }
  else{
    pre_Target_Length = Target_Length;
  }

  // Left_Leg.Target_L0 = Right_Leg.Target_L0 = Target_Length;

  // if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){
  //   // Left_Leg.Target_L0 = Right_Leg.Target_L0 = Target_Length;         //后续考虑的应该是加上Roll轴的考虑，目标腿长不应直接这样
  // }
  
  Left_Leg.ParamUpdata();
  Right_Leg.ParamUpdata();

}

void Class_Balance_Chassis::SpeedEstimate()
{
  Left_Leg.Leg_V_Calc();
  Right_Leg.Leg_V_Calc();

  if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){
    aver_v = (Left_Leg.leg_v_true - Right_Leg.leg_v_true) / 2.0f;
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air && Right_Leg.Get_Air_Status() == Leg_Air){
    aver_v = 0.0f;
  }
  else if(Right_Leg.Get_Air_Status() == Leg_Air){
    aver_v = Left_Leg.leg_v_true;
  }
  else if(Left_Leg.Get_Air_Status() == Leg_Air){
    aver_v = -Right_Leg.leg_v_true;
  }
  
  V_EstimateKF.MeasuredVector[0] = aver_v;
  V_EstimateKF.MeasuredVector[1] = Accel_X;
  Kalman_Filter_Update(&V_EstimateKF, NULL);

  Vx = V_EstimateKF.FilteredValue[0];

  if(fabs(Target_Vx) < 0.1){
    X = X + Vx * ROBOT_TASK_DT / 1000.0f;
  }
  else{
    X = 0.0f;
    Target_X = 0.0f;
  }

}

void Class_Balance_Chassis::NormalOutput()
{
  //最终力的输出
  if(Left_Leg.Get_Air_Status() == Leg_UnAir && Right_Leg.Get_Air_Status() == Leg_UnAir){
    Left_Leg.F0  = 69.0f + Left_Leg.dLength_PID.Get_Out() + Roll_PID.Get_Out() - Compensite_F0;         //F0是机体受到的向上的力
    Right_Leg.F0 = 69.0f + Right_Leg.dLength_PID.Get_Out() - Roll_PID.Get_Out() + Compensite_F0; 

    Left_Leg.Tp = Left_Leg.Get_LQR_Tp() + Tp_PID.Get_Out();
    Left_Leg.Wheel_T = Left_Leg.Get_LQR_Wheel_T() -  Turn_Omega_PID.Get_Out();        //顺时针切割x为正

    Right_Leg.Tp = Right_Leg.Get_LQR_Tp() + Tp_PID.Get_Out();
    Right_Leg.Wheel_T = Right_Leg.Get_LQR_Wheel_T() - Turn_Omega_PID.Get_Out();
  }
  else{
    Left_Leg.F0  = Left_Leg.dLength_PID.Get_Out();
    Right_Leg.F0 = Right_Leg.dLength_PID.Get_Out(); 

    Left_Leg.Tp = Left_Leg.Get_LQR_Tp();
    Left_Leg.Wheel_T = Left_Leg.Get_LQR_Wheel_T();        //注意坐标系是面朝腿外侧，顺时针

    Right_Leg.Tp = Right_Leg.Get_LQR_Tp();
    Right_Leg.Wheel_T = Right_Leg.Get_LQR_Wheel_T();
  }

  Left_Leg.Wheel_T  = -Left_Leg.Wheel_T;          //由于坐标系问题，3508逆时针为正，建模里是顺时针为正
  Right_Leg.Wheel_T = -Right_Leg.Wheel_T;

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
  X = 0.0f;
  Chassis_Stable_Count = 0;
  Target_Yaw_Angle = Yaw_Angle;
  Reserve_Status = Reserve_Disable;

  Left_Leg.Disable();
  Right_Leg.Disable();

  Tp_PID.Set_Integral_Error(0.0f);
  Roll_PID.Set_Integral_Error(0.0f);
  Turn_Omega_PID.Set_Integral_Error(0.0f);
  Turn_Angle_PID.Set_Integral_Error(0.0f);

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
  static float resver_alpha_Kp = 30.0f, reserve_d_alpha_Kp = 8.0f;
  static float Target_Left_Alpha = 0.25f, Target_Right_Alpha = -0.25f;     //左右坐标系不一致
  switch (Reserve_Status)
  {
    case(Reserve_Disable):
    {
      status_cnt = 0;
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
        Right_Leg.Tp = -reserve_d_alpha_Kp * (2.5f - Left_Leg.d_alpha_true);
      }
      else{
        Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.32f;

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

      Angle_Continuity_Process(&Target_Left_Alpha, Left_Leg.alpha);
      Angle_Continuity_Process(&Target_Right_Alpha, Right_Leg.alpha);

      float Left_Error = Target_Left_Alpha - Left_Leg.alpha;
      float Right_Error = Target_Right_Alpha - Right_Leg.alpha;

      Left_Leg.Wheel_T = 0.0f;
      Right_Leg.Wheel_T = 0.0f;
      Left_Leg.Tp  = resver_alpha_Kp * Left_Error;
      Right_Leg.Tp = resver_alpha_Kp * Right_Error;

      Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.18f;

      Math_Constrain(&Left_Leg.Tp, -10.0f, 10.0f);
      Math_Constrain(&Right_Leg.Tp, -10.0f, 10.0f);

      if(fabs(Right_Error) < 0.5f && fabs(Left_Error) < 0.5){
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
      Target_Length = Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.15f;
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

      Left_Leg.Target_L0 = Right_Leg.Target_L0 = 0.18f;
      break;
    }
  }
}
