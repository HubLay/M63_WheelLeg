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

float Poly_Coefficient[12][4] = {{-163.696546537900, 217.776180568688, -129.916306711902, -2.51033725460129},
																 {8.38647365190998, -7.48808338858023, -9.17798651273725, 0.112167270663901},
																 {-53.3509006644520, 67.6995263580939, -31.1361267419782, -0.814205732736298},
																 {-36.9710434209510, 48.9927066538411, -25.1653831817525, -1.75955700088198},
																 {19.3464431207759, 30.7640922244089, -50.0493283629428, 21.3508892661774},
																 {-3.19549257687929, 7.62959958968457, -6.38207173622906, 2.53885607522964},
																 {294.552353922323, -198.349007891591, -19.3458746640293, 48.3156145760187},
																 {51.8616348511822, -56.2137038568800, 20.2293427102935, 2.03973683756331},
																 {29.9012015839142, 24.0389642287713, -50.4904026897986, 22.0493027810502},
																 {-63.4517347393634, 116.228724274838, -80.0952664925332, 25.3294724679455},
																 {586.629889815140, -725.068306200901, 325.273275412536, 1.07140123332722},
																 {61.4792216000901, -75.7437054495019, 34.3038639165768, -1.60602456397967}};
// 三次多项式拟合系数
// float Poly_Coefficient[12][4]={	{-92.0699600773357	,71.4868555011764	,-30.6921933953314,	-0.0958002007084524},
//																{1.60098559368738,	-1.13274122580887	,-2.82116637582756	,0.0295182225494464},
//																{-21.1867196303270,	12.3610554853386,	-2.51566924070518	,-0.755562691415545},
//																{-30.6461230131359,	18.4403464720723,	-4.42893895932222	,-1.07006891622098},
//																{-141.783593718613,	90.5321293186876,	-21.7824629497436	,2.47606043845602},
//																{-6.49940206537698,	4.32462034853376	,-1.11404205284405,	0.174401976130811},
//																{3.09605574012049	,7.37880709057633	,-6.81650374507351	,2.87205502564828},
//																{4.83479253102295	,-3.01643745917309,	0.586384046364414	,0.237251571342193},
//																{-224.130470954818	,141.560444143461	,-33.2264057886601,	3.39152168796696},
//																{-313.237104933888,	197.784837724870	,-46.4812453696717,	4.81419185198688},
//																{360.850043501824,	-212.560933205359	,44.0392588918109 ,	9.96626295435711},
//																{25.0961164366780	,-15.1533421691331,	3.28051567574422	,0.286235660771812}};
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
		
		chassisR_feedback_update(&chassis_move, &right, &INS); // 更新数据
		chassisR_control_loop(&chassis_move, &right, &INS, LQR_K_R, &LegR_Pid); // 控制计算

		// chassis_move.stop_flag = 1;

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
													LQR_K[0] * (vmcr->theta - 0.0f) + 
													LQR_K[1] * (vmcr->d_theta - 0.0f) + 
													LQR_K[2] * (chassis->x_filter - chassis->x_set + x_error) + 
													LQR_K[3] * (chassis->v_filter - chassis->v_set) + 
													LQR_K[4] * (chassis->myPithR + 0.08f) + 
													LQR_K[5] * (chassis->myPithGyroR - 0.0f));

	// 右边髋关节输出力矩
	vmcr->Tp = (
					LQR_K[6] * (vmcr->theta - 0.0f) + 
					LQR_K[7] * (vmcr->d_theta - 0.0f) + 
					LQR_K[8] * (chassis->x_filter - chassis->x_set + x_error) + // 注意由于左右腿VMCX轴方向不一致，这里的速度位移计算也是整体相差一个负号
				  LQR_K[9] * (chassis->v_filter - chassis->v_set) + 
					LQR_K[10] * (chassis->myPithR + 0.08f) + 
					LQR_K[11] * (chassis->myPithGyroR - 0.0f));

	if ((chassis->stand_ready_flag_r == 1) && (chassis->stand_ready_flag_l == 1))
	{
		vmcr->Tp = vmcr->Tp + chassis->leg_tp;																									 // 髋关节输出力矩
		vmcr->F0 = 70.0f / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_set); // 重力前馈+pd

		chassis->wheel_motor[0].wheel_T = chassis->wheel_motor[0].wheel_T - chassis->turn_T; // 轮毂电机输出力矩
		// 左右腿都是按朝自己VMC正方向来先建模，T是顺时针，与机体不同的在计算力矩那里会改
		chassis->wheel_motor[0].wheel_T = -chassis->wheel_motor[0].wheel_T; // 建模里定义的T是顺时针为正，和轮电机逆时针为正相反
		chassis->stand_ready_flag = 1;
		if (chassis->stand_ready_ok_flag == 0)
		{
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
	//判断右腿站立姿态完成
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
		chassis->x_set = chassis->x_filter + 0.8;
		chassis->turn_set = chassis->total_yaw;
		vmcr->Tp = vmcr->Tp + chassis->leg_tp;
	}
	else if (chassis->stand_ready_ok_flag == 1)
	{											// 没有离地
		vmcr->leg_flag = 0; // 置为0
		vmcr->F0 = vmcr->F0 - chassis->roll_f0;
		// roll轴补偿取反然后加上去
	}
	//	 }
	//	 else if(chassis->recover_flag==1)
	//	 {
	//		 vmcr->Tp=0.0f;
	//	 }

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
