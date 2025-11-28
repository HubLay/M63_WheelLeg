#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"
#include "Up_board.h"
extern void Remote_data_process(Up_borard_t *data,chassis_t *chassis,float dt);
 
extern void remote_task(void);

#endif



