#include "printf_task.h"

#include "stm32h7xx_hal.h"

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "balance_chassis.h"

char Mes[100];
extern UART_HandleTypeDef huart7;

extern float tmp_Pitch_Err;
extern float tmp_wt;
extern float LQR_Tp_L[10];
extern float pitch_max;
extern float pitch_min;
extern float T_other;

extern uint8_t Test_Flag;
extern uint8_t cnt111;

extern uint8_t Theta_Synchronize, Theta_Synchronize_FLag;
extern float Left_Right_Err;

void PrintfTask(){
//     sprintf(Mes, "%f,%f,%f,%f,%d\n",
//   Balance_Chassis.Get_True_Vx(),
// Balance_Chassis.Get_True_X(),
// Balance_Chassis.Left_Leg.F0,
// Balance_Chassis.Right_Leg.F0,
// Balance_Chassis.Get_Chassis_Control_Type());           //128根本不够

  // sprintf(Mes, "%f,%f,%f,%f,%f,%f,%f,%f,%d\n", 
  // Balance_Chassis.Left_Leg.Get_Theta(),
  // Balance_Chassis.Get_Target_Vx(),
  // Balance_Chassis.Get_True_Vx(),
  // Balance_Chassis.Compensite_F0,
  // Balance_Chassis.Left_Leg.Target_L0,
  // Balance_Chassis.Left_Leg.dLength_PID.Get_Out(),
  // Balance_Chassis.Get_Pitch_Angle(),
  // Balance_Chassis.Get_Roll_Angle(),
  // Balance_Chassis.Get_Chassis_Control_Type());           //128根本不够

  // sprintf(Mes, "%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
  // Balance_Chassis.Left_Leg.Target_L0,
  // Balance_Chassis.Left_Leg.alpha,
  // Balance_Chassis.Left_Leg.Wheel_Motor.Get_Target_Torque(),
  // Balance_Chassis.Left_Leg.Tp,
  // Balance_Chassis.Get_True_Vx(),
  // Balance_Chassis.Get_Pitch_Angle(),
  // Balance_Chassis.Get_Jump_State(),
  // cnt111,
  // Balance_Chassis.Get_Chassis_Control_Type());           //128根本不够

  // sprintf(Mes, "%f,%f,%f,%f,%f\n", tmp_Pitch_Err, pitch_max, pitch_min,tmp_wt,T_other);           //128根本不够

  // sprintf(Mes, "%f,%f,%d,%f,%d,%f,%d,%d\n",
  // Balance_Chassis.Left_Leg.theta,
  // Balance_Chassis.Get_True_Vx(),
  // Balance_Chassis.Supercap.Get_Buffer_Power(),
  // Balance_Chassis.Limit_Power_Vx_Max,
  // Balance_Chassis.Referee.Get_Chassis_Power_Max(),
  // Balance_Chassis.Supercap.Get_Chassis_Power(),
  // Balance_Chassis.Referee.Get_Chassis_Energy_Buffer(),
  // Balance_Chassis.Get_Chassis_Control_Type());           //128根本不够

  //   sprintf(Mes, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
  // LQR_Tp_L[0], LQR_Tp_L[1],LQR_Tp_L[2],LQR_Tp_L[3],LQR_Tp_L[4],LQR_Tp_L[5],LQR_Tp_L[6],LQR_Tp_L[7],LQR_Tp_L[8],LQR_Tp_L[9], tmp_wt);           //128根本不够
	
  // sprintf(Mes, "%f,%f,%f,%f,%d,%d,%d,%d\n",
  // Balance_Chassis.Left_Leg.Tp,
  // Balance_Chassis.Right_Leg.Tp,
  // Left_Right_Err,
  // Balance_Chassis.Get_Pitch_Angle(),
  // Balance_Chassis.Reserve_Status,
  // Theta_Synchronize,
  // Theta_Synchronize_FLag,
  // Balance_Chassis.Get_Chassis_Control_Type());           //128根本不够
  
  HAL_UART_Transmit_DMA(&huart7, (uint8_t *)Mes, strlen(Mes));
}