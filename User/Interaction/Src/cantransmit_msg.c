#include "cantransmit_msg.h"

#define CAN_TRANSMIT_Dt 1         //ms

CanTransmit_Task_s *CanTransmit_Task[MAX_TRANSMIT_TASK_NUM];

void CanTransmit_TaskFunction(void *Param){
  while(1){
    CanTransmit_Task_s *CanTransmit_Task_Instance = (CanTransmit_Task_s *)Param;

    if (CanTransmit_Task_Instance == NULL)
    {
      continue;
    }

    CanTransmit_Task_Instance->Task_dt.dt = DWT_GetDeltaT(&(CanTransmit_Task_Instance->Task_dt.cnt_last));
    for (uint8_t i = 0; i < CanTransmit_Task_Instance->idx; i++)
    {
      uint16_t Transmit_ID = CanTransmit_Task_Instance->CanTransmit_Msg[i]->Transmit_ID;
      uint8_t *Transmit_Data = CanTransmit_Task_Instance->CanTransmit_Msg[i]->Transmit_Data;
      FDCAN_HandleTypeDef *hfdcan = CanTransmit_Task_Instance->CanTransmit_Msg[i]->hfdcan;
      CAN_Send_Data(hfdcan, Transmit_ID, Transmit_Data, 8);
    }

    vTaskDelay(CAN_TRANSMIT_Dt);
  }
}

/**
 * @brief 添加一个can发送消息任务
 * @param Task_ID 分配消息任务属于的id，0-7
 * @param Transmit_ID can报文的id 
 * @param Transmit_Data 发送数据对应的指针
 * @param hfdcan 绑定的can句柄
 * @return 
 */
uint8_t CanTransmit_TaskRegister(uint8_t Task_ID, uint16_t Transmit_ID, uint8_t *Transmit_Data, FDCAN_HandleTypeDef *hfdcan)
{
  if(Task_ID >= 8){
    return 0;
  }

  if(CanTransmit_Task[Task_ID] == NULL){
    CanTransmit_Task_s *Task_Instance = (CanTransmit_Task_s *)pvPortMalloc(sizeof(CanTransmit_Task_s));
    CanTransmit_Msg_s *Msg_Instance = (CanTransmit_Msg_s *)pvPortMalloc(sizeof(CanTransmit_Msg_s));

    memset(Task_Instance, 0, sizeof(CanTransmit_Task_s));
    memset(Msg_Instance, 0, sizeof(CanTransmit_Msg_s));

    CanTransmit_Task[Task_ID] = Task_Instance;
    Task_Instance->CanTransmit_Msg[Task_Instance->idx] = Msg_Instance;
    
    Msg_Instance->hfdcan = hfdcan;
    Msg_Instance->Transmit_ID = Transmit_ID;
    Msg_Instance->Transmit_Data =  Transmit_Data;
    Task_Instance->idx ++;

    return 1;
  }
  else{
    if(CanTransmit_Task[Task_ID]->idx >= 5){
      return 0;
    }

    CanTransmit_Msg_s *Msg_Instance = (CanTransmit_Msg_s *)pvPortMalloc(sizeof(CanTransmit_Msg_s));
    memset(Msg_Instance, 0, sizeof(CanTransmit_Msg_s));

    Msg_Instance->hfdcan = hfdcan;
    Msg_Instance->Transmit_ID = Transmit_ID;
    Msg_Instance->Transmit_Data =  Transmit_Data;
    CanTransmit_Task[Task_ID]->idx ++;
  }

}

void CanTransmit_TaskCreate()
{
  for(uint8_t i = 0; i<MAX_TRANSMIT_TASK_NUM; i++){
    if(CanTransmit_Task[i] == NULL){
      continue;
    }

    xTaskCreate(CanTransmit_TaskFunction, NULL, 128, CanTransmit_Task[i], 3, &CanTransmit_Task[i]->TaskHandle);

  }
}
