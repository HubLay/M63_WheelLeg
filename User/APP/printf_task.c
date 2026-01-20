#include "printf_task.h"

#include "chassisR_task.h"
#include "chassisL_task.h"

extern chassis_t chassis_move;
extern vmc_leg_t right;
extern vmc_leg_t left;

char Mes[100];
extern UART_HandleTypeDef huart7;

void printf_task(){
  sprintf(Mes, "%f,%f,%f,%f,%f,%f\n", chassis_move.joint_motor[0].Dt, chassis_move.joint_motor[2].Dt, right.Tp, left.Tp, chassis_move.leg_tp,right.theta);
	HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));
  vTaskDelay(2);
}