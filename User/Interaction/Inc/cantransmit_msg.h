#ifndef __CANTRANSMIT_MSG_H
#define __CANTRANSMIT_MSG_H

#ifdef __cplusplus
extern "C"{
#endif

#include "FreeRTOS.h"
#include "task.h"

#include "fdcan.h"
#include "stdint.h"
#include "dvc_dwt.h"
#include "string.h"
#include "drv_can.h"

#define MAX_TRANSMIT_TASK_NUM 8

typedef struct
{
  uint16_t Transmit_ID;
  uint8_t *Transmit_Data;
  FDCAN_HandleTypeDef *hfdcan;
}CanTransmit_Msg_s;

typedef struct
{
  struct_DwtTime Task_dt;
  TaskHandle_t TaskHandle;
  CanTransmit_Msg_s *CanTransmit_Msg[5];          //同一个任务id下，最多可以发送5个不同消息

  uint8_t idx;

}CanTransmit_Task_s;

void CanTransmit_TaskCreate();
uint8_t CanTransmit_TaskRegister(uint8_t Task_ID, uint16_t Transmit_ID, uint8_t *Transmit_Data, FDCAN_HandleTypeDef *hfdcan);

static void CanTransmit_TaskFunction(void *Param);


#ifdef __cplusplus
}
#endif

#endif