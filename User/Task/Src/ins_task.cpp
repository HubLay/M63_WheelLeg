#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "ins_task.h"
#include "dvc_dwt.h"
#include "balance_chassis.h"

TaskHandle_t Ins_TaskHandle;

struct_DwtTime Ins_dwt;
extern Class_Balance_Chassis Balance_Chassis;


void Ins_Task(void *Para){
  while (1)
  {
    Ins_dwt.dt = DWT_GetDeltaT(&Ins_dwt.cnt_last);          //统计任务运行间隔
    Balance_Chassis.IMU.TIM_Calculate_PeriodElapsedCallback();

    vTaskDelay(INS_TASK_DT);
  }
}