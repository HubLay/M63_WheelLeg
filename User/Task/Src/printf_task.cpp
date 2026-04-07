#include "printf_task.h"

#include "stm32h7xx_hal.h"

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "balance_chassis.h"

char Mes[100];
extern UART_HandleTypeDef huart7;

extern uint8_t Test_Flag, Test_Flag_2;

void PrintfTask(){
  sprintf(Mes, "%f,%d,%d,%f\n", Balance_Chassis.Left_Leg.FN, Balance_Chassis.Left_Leg.Air_Status,Balance_Chassis.Get_Chassis_Control_Type(), Balance_Chassis.Left_Leg.F0);           //128根本不够
	HAL_UART_Transmit_DMA(&huart7, (uint8_t *)Mes, strlen(Mes));
}