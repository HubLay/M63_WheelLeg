/**
  *********************************************************************
  * @file      remote_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
	*            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "remote_task.h"
#include "cmsis_os.h"
extern chassis_t chassis_move;
extern INS_t INS;
uint32_t REMOTE_TIME=10;//ps2手柄任务周期是10ms
void remote_task(void)
{
			Up_borard.V_X=0;
			Up_borard.V_Y=0;
			Up_borard.mode=0;
		while(1)
	 {
		   Remote_data_process(&Up_borard,&chassis_move,(float)REMOTE_TIME/1000.0f);//处理数据，设置期望数据
		   osDelay(REMOTE_TIME);
	 }
}
extern vmc_leg_t right;			
extern vmc_leg_t left;	
void Remote_data_process(Up_borard_t *data,chassis_t *chassis,float dt)
{  
	if(data->mode==1) 
	{
		//手柄上的Start按键被按下
		chassis->start_flag=1;
		if(chassis->recover_flag==0
			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
		{
		  chassis->recover_flag=1;//需要自起
		}
	}	else if(data->mode!=1) 
	{
		//手柄上的Start按键被按下
		chassis->start_flag=0;
		chassis->recover_flag=0;
	}
	if(chassis->start_flag==1)
	{
		//启动
		chassis->target_v=data->V_Y;//往前大于0
		chassis->v_set=chassis->target_v;

		chassis->x_set=chassis->x_set + chassis->v_set*dt;
		chassis->turn_set=chassis->turn_set - data->V_X/20.f;//往右大于0		
	  			
		//腿长变化
	//	chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f);
		//chassis->roll_target= ((float)(data->lx-127))*(0.0025f);
	//	chassis->roll_set=chassis->roll_target;
		//slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);
		chassis->roll_set=0;
		//jump_key(chassis,data);

		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;

//		mySaturate(&chassis->leg_left_set,0.065f,0.18f);//腿长限幅
//		mySaturate(&chassis->leg_right_set,0.065f,0.18f);//腿长限幅
		

		if(fabsf(chassis->last_leg_left_set-chassis->leg_left_set)>0.0001f || fabsf(chassis->last_leg_right_set-chassis->leg_right_set)>0.0001f)
		{//遥控器控制腿长在变化
			right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
      left.leg_flag=1;	 			
		}
		
		chassis->last_leg_set=chassis->leg_set;
		chassis->last_leg_left_set=chassis->leg_left_set;
		chassis->last_leg_right_set=chassis->leg_right_set;
	}
	else if(chassis->start_flag==0)
	{//关闭					按理说应该观测器什么的全部置零，目标值什么的也不能更新，
	  chassis->v_set=0.0f;//清零
		chassis->x_set=chassis->x_filter;//保存
	  chassis->turn_set=chassis->total_yaw;//保存
	  chassis->leg_set=0.18f;//原始腿长
		chassis->stand_ready_flag=0;
		chassis->stand_ready_flag_r=0;
		chassis->stand_ready_ok_flag=0;
	}
}
