#ifndef __ROBOT_H
#define __ROBOT_H

//实现各种硬件层的回调函数，以及任务创建，整车的初始化

#ifdef __cplusplus
extern "C"{
#endif

void Robot_Init();                      //系统任务以及配置初始化
void osTaskCreate();                    //整个系统的任务创建

#ifdef __cplusplus
}
#endif

#endif