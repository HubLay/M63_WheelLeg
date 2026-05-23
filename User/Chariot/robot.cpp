#include "FreeRTOS.h"
#include "task.h"

#include "robot.h"
#include "drv_can.h"
#include "drv_dwt.h"
#include "cantransmit_msg.h"
#include "balance_chassis.h"

#include "ins_task.h"
#include "robot_task.h"
#include "printf_task.h"

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
  switch (CAN_RxMessage->Header.Identifier){
    case(0x01):
    {
      Balance_Chassis.Left_Leg.Front_Joint.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x02):
    {
      Balance_Chassis.Left_Leg.Back_Joint.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x204):
    {
      Balance_Chassis.Left_Leg.Wheel_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
  }
}

void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
  switch (CAN_RxMessage->Header.Identifier){
    case(0x01):
    {
      Balance_Chassis.Right_Leg.Front_Joint.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x02):
    {
      Balance_Chassis.Right_Leg.Back_Joint.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x201):
    {
      Balance_Chassis.Right_Leg.Wheel_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x67):
    {
      DaemonReload(Balance_Chassis.Supercap_Daemon);
      Balance_Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
  }
}

void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier){
    case(0x205):
    {
      Balance_Chassis.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
      break;
    }
    case(0x77):
    {
      Balance_Chassis.CAN_Chassis_Rx_Gimbal_Callback(CAN_RxMessage->Data);
      break;
    }
  }
}

void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
  Balance_Chassis.DR16.DR16_UART_RxCpltCallback(Buffer);
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART10_Callback(uint8_t *Buffer, uint16_t Length)
{
  DaemonReload(Balance_Chassis.Referee_Daemon);
  Balance_Chassis.Referee.UART_RxCpltCallback(Buffer,Length);
}

void Device_SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length){

}

void Gimbal_Offline_CallbackFunction(void *Param){
  
}

void Robot_Init(){
  __disable_irq();

  DWT_Init(480);          //必须最先初始化

  CAN_Init(&hfdcan1, Chassis_Device_CAN1_Callback);
  CAN_Init(&hfdcan2, Chassis_Device_CAN2_Callback);
  CAN_Init(&hfdcan3, Chassis_Device_CAN3_Callback);

  SPI_Init(&hspi2, Device_SPI2_Callback);

  UART_Init(&huart5, DR16_UART5_Callback, 18);

  UART_Init(&huart10, Referee_UART10_Callback, 128);
  
  Balance_Chassis.Init();

  osTaskCreate();             //任务创建    每个电机或者需要的外设应该在初始化的时候注册守护任务

  __enable_irq();

}

void osTaskCreate(){

  //当这个任务优先级2的时候，会由于被别的长时间任务冲突，导致它的执行时间间隔不稳定，同时它本身执行时间很短，可以提升优先级保证正常执行
  xTaskCreate(CanTransmit_Task, "CanTransmit_Task", 128, NULL, 3, &CanTransmit_TaskHandle);             
  xTaskCreate(Daemon_Task, "Daemon_Task", 128, NULL, 2, &Daemon_TaskHandle);
  xTaskCreate(Robot_Task, "Robot_Task", 512, NULL, 2, &Robot_TaskHandle);
  xTaskCreate(Ins_Task, "Ins_Task", 512, NULL, 2, &Ins_TaskHandle);
  xTaskCreate(CMDProcess_Task, "CMDProcess_Task", 128, NULL, 2, &CMD_TaskHandle); 
  xTaskCreate(UI_Task, "UI_Task", 512, NULL, 2, &UI_TaskHandle);

  xTaskCreate(Printf_Task, "Printf_Task", 512, NULL, 1, &Printf_TaskHandle);
  xTaskCreate(Emergency_Stop_Task, "Emergency_Stop_Task", 128, NULL, 3, &Emergency_Stop_TaskHandle); 

  CanTransmit_TaskCreate();

}