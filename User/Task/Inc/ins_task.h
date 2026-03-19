#ifndef __INS_TASK_H
#define __INS_TASK_H

//IMU的姿态解算任务

extern TaskHandle_t Ins_TaskHandle;

void Ins_Task(void *Para);


#endif