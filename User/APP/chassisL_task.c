/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"

#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"

vmc_leg_t left;

//float LQR_K_L[12]={  -1.5171 ,  -0.1347  , -2.4105,   -0.9858 ,   0.8685  ,  0.0783,
//    1.2392 ,   0.1251 ,   2.9650 ,   1.0868,   10.7689 ,   0.5026};

//Q=diag([20 0.1 80 110 700 1]);
//R=[90 0;0 4]; 
float LQR_K_L[12]={-19.7949 ,  -1.7312  , -4.5360,   -4.9155 ,  13.4709 ,   1.6190
 ,  40.1947  ,  4.1664 ,  13.9373  , 14.3136,   39.5619,    2.4759
};




//float LQR_K_L[12]={     -2.1340  , -0.2028 ,  -0.8838 ,  -1.3155 ,   1.2601 ,   0.1103,
//    2.3696  , 0.2669  ,  1.5572 ,   2.2473  , 12.2365   , 0.4599};
extern float Poly_Coefficient[12][4];

extern chassis_t chassis_move;
		
PidTypeDef LegL_Pid;
extern INS_t INS;

uint32_t CHASSL_TIME=1;				
void ChassisL_task(void)
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
  ChassisL_init(&chassis_move,&left,&LegL_Pid);//初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部
	
	while(1)
	{	
		chassisL_feedback_update(&chassis_move,&left,&INS);//更新数据
		
		chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_L,&LegL_Pid);//控制计算
   		
		// chassis_move.stop_flag = 1;
   if(chassis_move.start_flag==1&&chassis_move.wheel_motor[0].para.online_status==1&&chassis_move.wheel_motor[1].para.online_status&&chassis_move.stop_flag==0)	
		{
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);//left.torque_set[1]		左前
			osDelay(CHASSL_TIME);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);					//左后
			osDelay(CHASSL_TIME);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);//左边边轮毂电机
			osDelay(CHASSL_TIME);
		}
		else if(chassis_move.start_flag==0 || chassis_move.stop_flag == 1)	
		{
		  mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
			osDelay(CHASSL_TIME);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			osDelay(CHASSL_TIME);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边轮毂电机	
			osDelay(CHASSL_TIME);
		}
	}
}

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//发送id为8				左前
	
	wheel_motor_init(&chassis->wheel_motor[1],0x200,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值
	
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  osDelay(1);
	}
//	for(int j=0;j<10;j++)
//	{
//    enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//左边轮毂电机
//	  osDelay(1);
//	}
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
	vmc->d_phi1	=chassis->joint_motor[2].para.vel;
	vmc->d_phi4 =+chassis->joint_motor[3].para.vel;	
	chassis->myPithL=0.0f-ins->Pitch;//注意imu位置
	chassis->myPithGyroL=0.0f-ins->Gyro[0];
	
}

extern uint8_t right_flag;
uint8_t left_flag;
float T_out[6];
float TP_out[6];
float x_error=-0.03;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}

	chassis->wheel_motor[1].wheel_T=(LQR_K[0]*(vmcl->theta-0.0f)
																	+LQR_K[1]*(vmcl->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_set-chassis->x_filter-x_error)
																	+LQR_K[3]*(chassis->v_set-chassis->v_filter)
																	+LQR_K[4]*(chassis->myPithL-0.08f)
																	+LQR_K[5]*(chassis->myPithGyroL-0.0f));
	//右边髋关节输出力矩				
	vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
					+LQR_K[7]*(vmcl->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_set-chassis->x_filter-x_error)
					+LQR_K[9]*(chassis->v_set-chassis->v_filter)
					+LQR_K[10]*(chassis->myPithL-0.08f)
					+LQR_K[11]*(chassis->myPithGyroL-0.0f));

	T_out[0] = LQR_K[0] * (vmcl->theta - 0.0f);
	T_out[1] = LQR_K[1] * (vmcl->d_theta - 0.0f);
	T_out[2] = LQR_K[2] * (chassis->x_set - chassis->x_filter - x_error);
	T_out[3] = LQR_K[3] * (chassis->v_set - chassis->v_filter);
	T_out[4] = LQR_K[4] * (chassis->myPithL - 0.0f);
	T_out[5] = LQR_K[5] * (chassis->myPithGyroL - 0.0f);
	TP_out[0] = LQR_K[6] * (vmcl->theta - 0.0f);
	TP_out[1] = LQR_K[7] * (vmcl->d_theta - 0.0f);
	TP_out[2] = LQR_K[8] * (chassis->x_set - chassis->x_filter - x_error);
	TP_out[3] = LQR_K[9] * (chassis->v_set - chassis->v_filter);
	TP_out[4] = LQR_K[10] * (chassis->myPithL - 0.0f);
	TP_out[5] = LQR_K[11] * (chassis->myPithGyroL - 0.0f);

	if ((chassis->stand_ready_flag_r == 1) && (chassis->stand_ready_flag_l == 1))
	{
		vmcl->Tp=vmcl->Tp+chassis->leg_tp;//髋关节输出力矩
		vmcl->F0=70.0f/arm_cos_f32(vmcl->theta)+PID_Calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd
		
		chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
		chassis->wheel_motor[1].wheel_T=-chassis->wheel_motor[1].wheel_T;
	
	
	}
	else if((chassis->stand_ready_flag_r==0)||(chassis->stand_ready_flag_l==0))
	{
		vmcl->F0=PID_Calc(leg,vmcl->L0,0.14)*0.8;
		vmcl->Tp=60*(vmcl->alpha+3.1415926f/120.0f);
		chassis->wheel_motor[1].wheel_T=0;
		 mySaturate(&vmcl->Tp,-15.f,15.0f);	
		if((vmcl->alpha>(3.1415926f/2.0f))||(vmcl->alpha<(-3.1415926f/2.0f)))
		{
		vmcl->Tp=-2;
			
		}	
		
		if(vmcl->L0>0.17)vmcl->Tp=0;
	}
	if(chassis->stand_ready_flag==0)
	{	
	if((vmcl->alpha>(-3.1415926f/12.0f))&&(vmcl->alpha<3.1415926f/12.0f)&&(chassis->start_flag==1)&&(vmcl->d_alpha<0.08)&&(vmcl->d_alpha>-0.08))
	{
				if(chassis->stand_ready_time_l<1000)
			chassis->stand_ready_time_l++;
		if(chassis->stand_ready_time_l>=100)
		chassis->stand_ready_flag_l=1;
	}
	else
	{	chassis->stand_ready_flag_l=0;
	chassis->stand_ready_time_l=0;}

	}




//	//jump_loop_l(chassis,vmcl,leg); 
	left_flag=ground_detectionL(vmcl,ins);//左腿离地检测
////	
//	 if(chassis->recover_flag==0)	
//	 {//倒地自起不需要检测是否离地
		//车体姿态正常并且左右腿离地
		if((right_flag==1)&&(left_flag==1)&&(	chassis->stand_ready_ok_flag==1))//&&right_flag==1&&vmcl->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis->wheel_motor[1].wheel_T=0.0f;
			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
			
			chassis->x_filter=0.0f;//对位移清零
			chassis->x_set=chassis->x_filter+0.8;
			chassis->turn_set=chassis->total_yaw;
			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
		}
		else if(chassis->stand_ready_ok_flag==1)
		{//没有离地
			vmcl->leg_flag=0;//置为0
			vmcl->F0=vmcl->F0+chassis->roll_f0;//roll轴补偿取反然后加上去				
				
		}
//	 }
//	 else if(chassis->recover_flag==1)
//	 {
//		 vmcl->Tp=0.0f;
//	 }
	

	
  //额定扭矩
	mySaturate(&chassis->wheel_motor[1].wheel_T,-4.2f,4.2f);	
	mySaturate(&vmcl->F0,-200.0f,200.0f);//限幅 
	VMC_calc_2(vmcl);//计算期望的关节输出力矩
  mySaturate(&vmcl->torque_set[1],-20.0f,20.0f);	
	mySaturate(&vmcl->torque_set[0],-20.0f,20.0f);	
	
}



void jump_loop_l(chassis_t *chassis,vmc_leg_t *vmcl,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_l == 0)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.07f) ;//前馈+pd
			if(vmcl->L0<0.1f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 1)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.30f) ;//前馈+pd
			if(vmcl->L0>0.15f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 2)
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcl->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_l++;
			}
		}
		else
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) ;//前馈+pd
		}

	}
	else
	{
		vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) + chassis->now_roll_set;//前馈+pd
		
	}

}


