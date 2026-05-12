
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "dvc_dwt.h"
#include "robot_task.h"
#include "printf_task.h"
#include "emergency_stop.h"
#include "robot_cmd.h"
#include "balance_chassis.h"

TaskHandle_t CanTransmit_TaskHandle;
TaskHandle_t Daemon_TaskHandle;
TaskHandle_t Robot_TaskHandle;
TaskHandle_t CMD_TaskHandle;
TaskHandle_t Printf_TaskHandle;
TaskHandle_t Emergency_Stop_TaskHandle;

struct_DwtTime CanTransmit_dwt;
struct_DwtTime CMDProcess_dwt;
struct_DwtTime Daemon_dwt;
struct_DwtTime Robot_dwt;
struct_DwtTime Emergency_Stop_dwt;

void Printf_Task(void *Para){
  while (1)
  {

    PrintfTask();

    vTaskDelay(5);
  }
}

void CanTransmit_Task(void *Para){            //非一发一收模式的报文发送，怕达妙类型一发一收连续发送被仲裁
  while (1)
  {
    CanTransmit_dwt.dt = DWT_GetDeltaT(&CanTransmit_dwt.cnt_last);

    CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);          
    CAN_Send_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, 8); 

    vTaskDelay(CAN_TRANSMIT_TASK_DT);           //1000Hz
  }
}

void CMDProcess_Task(void *Para){
  while (1)
  {
    CMDProcess_dwt.dt = DWT_GetDeltaT(&CMDProcess_dwt.cnt_last);

    CMDProcessTask();

    vTaskDelay(CMDProcess_TASK_DT);       //200Hz 
  }
}

void Emergency_Stop_Task(void *Para){
  while (1)
  {
    /* code */
    Emergency_Stop_dwt.dt = DWT_GetDeltaT(&Emergency_Stop_dwt.cnt_last);

    Emergency_StopTask();

    vTaskDelay(EMERGENCY_STOP_TASK_DT);       //200Hz 
  }
  
}

void Daemon_Task(void *Para){
  while (1)
  {
    Daemon_dwt.dt = DWT_GetDeltaT(&Daemon_dwt.cnt_last);

    DaemonTask();

    vTaskDelay(DAEMON_TASK_DT);
  }
}

void Robot_Task(void *Para){

  while (1)
  {
    Robot_dwt.dt = DWT_GetDeltaT(&Robot_dwt.cnt_last);
    Balance_Chassis.TIM_Calculate_PeriodElapsedCallback();

    vTaskDelay(ROBOT_TASK_DT);
  }

}