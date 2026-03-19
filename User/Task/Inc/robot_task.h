#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

//云台或底盘的核心任务，以及一些小功能任务

extern TaskHandle_t CanTransmit_TaskHandle;
extern TaskHandle_t Daemon_TaskHandle;
extern TaskHandle_t Robot_TaskHandle;
extern TaskHandle_t CMD_TaskHandle;
extern TaskHandle_t Printf_TaskHandle;

void CanTransmit_Task(void *Para);
void Daemon_Task(void *Para);
void CMDProcess_Task(void *Para);
void Robot_Task(void *Para);
void Printf_Task(void *Para);



#endif