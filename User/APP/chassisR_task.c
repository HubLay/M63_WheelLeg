/**
	*********************************************************************
	* @file      chassisR_task.c/h
	* @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
	* @note
	* @history
	*
	@verbatim
	==============================================================================

	==============================================================================
	@endverbatim
	*********************************************************************
	*/

#include "chassisR_task.h"
#include "my_kalman.h"
#include "fdcan.h"
#include "cmsis_os.h"

#include <stdio.h>
// Q=diag([20 0.1 80 110 700 1]);
//     R=[90 0;0 4];
// float LQR_K_R[12]={
//-10.3692  , -0.8455 ,  -1.3771 ,  -1.7585   , 4.3970 ,   0.7893,
//     7.7685  ,  0.6771   , 1.7812  ,  2.1353  ,  9.4314  ,  1.1389
// };
// float LQR_K_R[12]={
//-11.4621  , -0.9226   ,-1.0515  , -1.8152  ,  6.0725  ,  0.9167
//    , 9.5542 ,   0.7866  ,  1.3424  ,  2.2247  , 16.4825  ,  1.5499
// };
// float LQR_K_R[12]={ -9.6745  , -0.7850  , -0.9891 ,  -1.3678  ,  4.7708   , 0.6105,
//    21.2186  ,  1.9576   , 4.5166   , 5.8310 ,  12.6958  ,  0.4720};
// float LQR_K_R[12]={-10.9788  , -0.8456  , -1.2608  , -1.6752 ,   4.1706 ,   0.7935,
//     7.6622   , 0.6137   , 1.4652   , 1.8303 ,   8.5417  ,  1.1779

//};
float LQR_K_R[12] = {-19.7949, -1.7312, -4.5360, -4.9155, 13.4709, 1.6190, 40.1947, 4.1664, 13.9373, 14.3136, 39.5619, 2.4759};
// a11-a26的12组4维数据
float Poly_Coefficient[12][4] = {
    {-105.1466, 137.9171, -91.1062, -1.3694},  // a11
    {0.1407, 0.8799, -8.6317, -0.0854},         // a12
    {-43.9729, 48.8673, -19.1419, -3.9064},    // a13
    {-22.9555, 26.5224, -12.9046, -3.4135},    // a14
    {-150.9675, 213.3057, -119.0827, 32.5858}, // a15
    {-5.2680, 9.1903, -6.2156, 2.8878},         // a16
    {177.1774, -165.7278, 40.9203, 13.0940},   // a21
    {12.0866, -11.6203, 1.7225, 1.7455},        // a22
    {-88.9649, 126.9058, -69.9800, 16.6340},   // a23
    {-89.6878, 116.9342, -59.4289, 13.3717},   // a24
    {717.0438, -800.8509, 320.5993, 23.5748},  // a25
    {50.0040, -59.9865, 26.9624, -0.3749}      // a26
};
// float Poly_Coefficient[12][4] = {
//     {-67.4309, 102.4276, -106.7760, -1.7936},  // a11
//     {7.2806, -12.2652, -11.4053, -0.1201},       // a12
//     {-94.2699, 117.6216, -53.1895, -9.8313},    // a13
//     {-19.3348, 30.6974, -23.5991, -6.1914},     // a14
//     {-62.4316, 117.4683, -84.5214, 32.3439},    // a15
//     {-3.1463, 5.8273, -4.0895, 3.2322},         // a16
//     {366.5643, -367.5410, 109.6442, 17.0662},   // a21
//     {39.0405, -42.7806, 15.1734, 2.3643},       // a22
//     {-51.1493, 172.4416, -151.6217, 52.0679},   // a23
//     {-84.5758, 139.7397, -90.9977, 28.3272},    // a24
//     {702.1978, -841.1999, 371.3393, 6.6606},    // a25
//     {52.6469, -68.4746, 34.3058, -2.9496}       // a26
// };

float theta_offset = 0.08f;					//串腿等效重心不在虚拟杆上，加一点偏移

vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;

chassis_t chassis_move;

PidTypeDef LegR_Pid; // 右腿的腿长pd
PidTypeDef Roll_Pid; // 横滚角补偿pd
PidTypeDef Tp_Pid;	 // 防劈叉补偿pd
PidTypeDef Turn_Pid; // 转向pd

uint32_t CHASSR_TIME = 1;
char Mes[100];
extern UART_HandleTypeDef huart7;

void ChassisR_task(void)
{
	while (INS.ins_flag == 0) // 等待机体姿态收敛
	{													// 等待加速度收敛
		osDelay(1);
	}
	
	ChassisR_init(&chassis_move, &right, &LegR_Pid); // 初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
	Pensation_init(&Roll_Pid, &Tp_Pid, &Turn_Pid);	 // 补偿pid初始化

	while (1)
	{
		// sprintf(Mes, "%f,%f,%f,%f\n", left.kalman_d_L0, left.d_L0, left.kalman_d_alpha, left.d_alpha);
		// HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));

		// sprintf(Mes, "%f,%f,%f,%f,%d\n", chassis_move.v_set, chassis_move.v_filter,chassis_move.x_set,chassis_move.x_filter,chassis_move.start_flag);
		// HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));

		// sprintf(Mes, "%f,%f,%f\n", left.Tp, right.Tp, INS.Pitch);
		// HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));

		// sprintf(Mes, "%f,%f,%f,%f,%d\n", left.F0, left.Tp, left.torque_set[0], left.torque_set[1],chassis_move.start_flag);
		// HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));

		chassisR_feedback_update(&chassis_move, &right, &INS); // 更新数据
		chassisR_control_loop(&chassis_move, &right, &INS, LQR_K_R, &LegR_Pid); // 控制计算

		//chassis_move.stop_flag = 1;

		if (chassis_move.start_flag == 1 && chassis_move.wheel_motor[0].para.online_status == 1 
			&& chassis_move.wheel_motor[1].para.online_status && chassis_move.stop_flag == 0)
		{
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[1]); // right.torque_set[1]   右后
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[0]); // right.torque_set[0]    右前
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T); // 右边轮毂电机
			osDelay(CHASSR_TIME);
		}
		else if (chassis_move.start_flag == 0 || chassis_move.stop_flag == 1)
		{
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 右边轮毂电机
			osDelay(CHASSR_TIME);
		}
	}
}

void ChassisR_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legr)
{
	const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0], 6, MIT_MODE); // 发送id为6				右前
	joint_motor_init(&chassis->joint_motor[1], 8, MIT_MODE); // 发送id为8				右后

	wheel_motor_init(&chassis->wheel_motor[0], 0x200, MIT_MODE); // 发送id为1

	VMC_init(vmc); // 给杆长赋值

	PID_init(legr, PID_POSITION, legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 腿长pid

	// 使能两个电机
	for (int j = 0; j < 10; j++)
	{
		enable_motor_mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode);
		osDelay(1);
	}
	for (int j = 0; j < 10; j++)
	{
		enable_motor_mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode);
		osDelay(1);
	}
}

void Pensation_init(PidTypeDef *roll, PidTypeDef *Tp, PidTypeDef *turn)
{ // 补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};				

	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
	vmc->phi1 = pi / 2.0f + chassis->joint_motor[0].para.pos;
	vmc->phi4 = pi / 2.0f + chassis->joint_motor[1].para.pos;
	vmc->d_phi1 = chassis->joint_motor[0].para.vel;
	vmc->d_phi4 = +chassis->joint_motor[1].para.vel;
	chassis->myPithR = ins->Pitch;
	chassis->myPithGyroR = ins->Gyro[0];

	chassis->total_yaw = ins->YawTotalAngle;
	chassis->roll = ins->Roll;
	chassis->theta_err = 0.0f - (vmc->theta + left.theta);

	if (ins->Pitch < (3.1415926f / 6.0f) && ins->Pitch > (-3.1415926f / 6.0f)) //+-30度以内的姿态
	{																																					 // 根据pitch角度判断倒地自起是否完成
		chassis->recover_flag = 0;
	}
}

uint8_t right_flag = 0;
extern uint8_t left_flag;
float target_alpha = 0;
extern float x_error;
void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcr, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
	VMC_calc_1_right(vmcr, ins, ((float)CHASSR_TIME) * 3.0f / 1000.0f); // 计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒

	for (int i = 0; i < 12; i++)
	{
		LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0);
	}

	// chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
	chassis->turn_T = Turn_Pid.Kp * (chassis->turn_set - chassis->total_yaw) - Turn_Pid.Kd * ins->Gyro[2]; // 这样计算更稳一点
	// chassis->roll_f0=PID_Calc(&Roll_Pid, chassis->roll,chassis->roll_set);//roll轴pid计算
	chassis->roll_f0 = Roll_Pid.Kp * (chassis->roll_set - chassis->roll) - Roll_Pid.Kd * ins->Gyro[1];
	chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f); // 防劈叉pid计算
	
	chassis->wheel_motor[0].wheel_T = (
													LQR_K[0] * (vmcr->theta - 0.0f - theta_offset) + 
													LQR_K[1] * (vmcr->kalman_d_theta - 0.0f) + 
													LQR_K[2] * (chassis->x_filter - chassis->x_set + x_error) + 
													LQR_K[3] * (chassis->v_filter - chassis->v_set) + 
													LQR_K[4] * (chassis->myPithR - 0.030f) + 
													LQR_K[5] * (chassis->myPithGyroR - 0.0f));

	// 右边髋关节输出力矩				Tp正方向均为从对应腿面看过去的逆时针
	vmcr->Tp = (
					LQR_K[6] * (vmcr->theta - 0.0f- theta_offset) + 
					LQR_K[7] * (vmcr->kalman_d_theta - 0.0f) + 
					LQR_K[8] * (chassis->x_filter - chassis->x_set + x_error) + // 注意由于左右腿VMCX轴方向不一致，这里的速度位移计算也是整体相差一个负号
				  LQR_K[9] * (chassis->v_filter - chassis->v_set) + 
					LQR_K[10] * (chassis->myPithR - 0.030f) + 
					LQR_K[11] * (chassis->myPithGyroR - 0.0f));

	if ((chassis->stand_ready_flag_r == 1) && (chassis->stand_ready_flag_l == 1))
	{
		vmcr->Tp = vmcr->Tp + chassis->leg_tp;																									 // 髋关节输出力矩
		vmcr->F0 = 69.0f / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_set); // 重力前馈+pd

		chassis->wheel_motor[0].wheel_T = chassis->wheel_motor[0].wheel_T - chassis->turn_T; // 轮毂电机输出力矩
		// 左右腿都是按朝自己VMC正方向来先建模，T是顺时针，与机体不同的在计算力矩那里会改
		chassis->wheel_motor[0].wheel_T = -chassis->wheel_motor[0].wheel_T; // 建模里定义的T是顺时针为正，和轮电机逆时针为正相反
		chassis->stand_ready_flag = 1;
		if (chassis->stand_ready_ok_flag == 0)
		{
			chassis->x_set = chassis->x_filter = 0.0f;				//起来的时候车会后退一段
			if ((vmcr->theta < (3.1415926f / 12.0f)) && (vmcr->theta > -3.1415926f / 12.0f))
			{
				// 摆杆竖直方向角度正常
				if (chassis->stand_ready_ok_time <= 500)
					chassis->stand_ready_ok_time++;
				if (chassis->stand_ready_ok_time >= 100)
				{
					// 站立完成
					chassis->stand_ready_ok_flag = 1;
				}
			}
			else
			{
				chassis->stand_ready_ok_time = 0;
			}
		}
		else
		{
			// 倒下了
			if ((vmcr->theta > (3.1415926f / 2.0f)) || (vmcr->theta < -3.1415926f / 2.0f))
			{
				//			chassis->stand_ready_flag=0;
				//			chassis->stand_ready_time_r=0;
				//			chassis->stand_ready_time_l=0;
				//			chassis->stand_ready_flag_r=0;
				//			chassis->stand_ready_flag_l=0;
				//			chassis->stand_ready_ok_time=0;
				//			chassis->stand_ready_ok_flag=0;
				//			chassis->v_set=0.0f;//清零
				//			chassis->x_set=chassis->x_filter+0.5;//保存
				//			chassis->turn_set=chassis->total_yaw;//保存
				//			chassis->leg_set=0.18f;//原始腿长
				chassis->stop_flag = 1;
			}
		}
	}
	else if ((chassis->stand_ready_flag_r == 0) || (chassis->stand_ready_flag_l == 0))
	{		//车体是倒下的状态   倒地自启
		vmcr->F0 = PID_Calc(leg, vmcr->L0, 0.14) * 0.8;
		vmcr->Tp = 60 * (vmcr->alpha - 3.1415926f / 120.0f);
		chassis->wheel_motor[0].wheel_T = 0;
		mySaturate(&vmcr->Tp, -15.f, 15.0f);
		if ((vmcr->alpha > (3.1415926f / 2.0f)) || (vmcr->alpha < (-3.1415926f / 2.0f)))
		{
			vmcr->Tp = 2;
		}

		if (vmcr->L0 > 0.17)
			vmcr->Tp = 0;
	}
	//判断右腿站立姿态完成 只会执行一次
	if (chassis->stand_ready_flag == 0)
	{
		if ((vmcr->alpha < (3.1415926f / 12.0f)) && (vmcr->alpha > -3.1415926f / 12.0f) && (chassis->start_flag == 1) && (vmcr->d_alpha < 0.08) && (vmcr->d_alpha > -0.08))
		{
			if (chassis->stand_ready_time_r < 1000)
				chassis->stand_ready_time_r++;
			if (chassis->stand_ready_time_r >= 100)
				chassis->stand_ready_flag_r = 1;
		}
		else
		{
			chassis->stand_ready_flag_r = 0;
			chassis->stand_ready_time_r = 0;
		}
	}

	//	vmcr->F0=vmcr->F0-chassis->roll_f0;
	//	jump_loop_r(chassis,vmcr,leg);
	right_flag = ground_detectionR(vmcr, ins); // 右腿离地检测

	//	 if(chassis->recover_flag==0)
	//	 {//倒地自起不需要检测是否离地
	if ((right_flag == 1) && (chassis->stand_ready_ok_flag == 1) && (left_flag == 1) && vmcr->leg_flag==0)
	{ // 当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
		chassis->wheel_motor[0].wheel_T = 0.0f;
		vmcr->Tp = LQR_K[6] * (vmcr->theta - 0.0f) + LQR_K[7] * (vmcr->d_theta - 0.0f);
		//	vmcr->F0=PID_Calc(leg,vmcr->L0,chassis->leg_set);
		chassis->x_filter = 0.0f;
		chassis->x_set = 0.0f;
		chassis->turn_set = chassis->total_yaw;
		vmcr->Tp = vmcr->Tp + chassis->leg_tp;
		chassis_move.stop_flag = 1;
	}
	else if (chassis->stand_ready_ok_flag == 1)
	{											// 没有离地
		vmcr->leg_flag = 0; // 置为0
		vmcr->F0 = vmcr->F0 - chassis->roll_f0;
		// roll轴补偿取反然后加上去
	}
		//  }
		//  else if(chassis->recover_flag==1)
		//  {
		// 	 vmcr->Tp=0.0f;
		//  }

	// 额定扭矩
	mySaturate(&vmcr->F0, -200.0f, 200.0f); // 限幅
	mySaturate(&chassis->wheel_motor[0].wheel_T, -4.2f, 4.2f);
	VMC_calc_2(vmcr); // 计算期望的关节输出力矩
	mySaturate(&vmcr->torque_set[1], -20.0f, 20.0f);
	mySaturate(&vmcr->torque_set[0], -20.0f, 20.0f);
}

//限幅函数
void mySaturate(float *in, float min, float max)
{
	if (*in < min)
	{
		*in = min;
	}
	else if (*in > max)
	{
		*in = max;
	}
}
void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcr, PidTypeDef *leg)
{
	if (chassis->jump_flag == 1)
	{
		if (chassis->jump_status_r == 0)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.07f); // 前馈+pd
			if (vmcr->L0 < 0.1f)
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if (chassis->jump_status_r == 1)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.3f); // 前馈+pd
			if (vmcr->L0 > 0.15f)
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if (chassis->jump_status_r == 2)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set); // 前馈+pd
			if (vmcr->L0 < (chassis->leg_right_set + 0.01f))
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set); // 前馈+pd
		}

		if (chassis->jump_status_r == 3 && chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set) - chassis->now_roll_set; // 前馈+pd
	}
}
