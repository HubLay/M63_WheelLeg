#include "printf_task.h"

#include "chassisR_task.h"
#include "chassisL_task.h"

extern chassis_t chassis_move;
extern vmc_leg_t right;
extern vmc_leg_t left;
extern uint8_t right_flag;
extern uint8_t left_flag;

extern float FnR, FnL;

extern float TP_out[6];

char Mes[100];
extern UART_HandleTypeDef huart7;

int Error_Count = 0;

void printf_task(){
  if(chassis_move.leg_tp > 100){
    //Error_Count ++;
  }
  sprintf(Mes, "%f,%f,%f,%f,%f,%d,%d\n", chassis_move.target_v, right.FN, left.FN, right.F0, left.F0, right_flag, left_flag);
	HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));
  vTaskDelay(2);
}