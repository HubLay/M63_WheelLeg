/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
	* 					 原理来源于https://zhuanlan.zhihu.com/p/689921165
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "kalman_filter.h"
#include "my_kalman.h"
#include "cmsis_os.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体
my_kalman kalman_MotionAccel_n;
my_kalman kalman_Gyro0;
float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {350.0f, 0.0f, 
                            0.0f,  50.0f}; 	//观测噪声
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
extern INS_t INS;		
extern chassis_t chassis_move;																 															 
																 
extern vmc_leg_t right;			
extern vmc_leg_t left;	

float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//任务周期是3ms			
uint8_t mod50;																 
void 	Observe_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
		
	xvEstimateKF_Init(&vaEstimateKF);
	kalman_Init(&kalman_MotionAccel_n,0.3,0.01,0,1);
	kalman_Init(&kalman_Gyro0,0.5,0.01,0,1);
  while(1)
	{  
		kalman_set_now(&kalman_MotionAccel_n,INS.MotionAccel_n[1]);
		Recv_Adjust_PeriodElapsedCallback(&kalman_MotionAccel_n);
//		kalman_set_now(&kalman_Gyro0,INS.Gyro[0]);
//		Recv_Adjust_PeriodElapsedCallback(&kalman_Gyro0);
		wr= -chassis_move.wheel_motor[0].para.vel-INS.Gyro[0]+right.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb=wr*0.075f+right.L0*right.d_theta*arm_cos_f32(right.theta)+right.d_L0*arm_sin_f32(right.theta);//机体b系的速度
		
		wl= -chassis_move.wheel_motor[1].para.vel+INS.Gyro[0]+left.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb=wl*0.075f+left.L0*left.d_theta*arm_cos_f32(left.theta)+left.d_L0*arm_sin_f32(left.theta);//机体b系的速度
		
		aver_v=(vrb-vlb)/2.0f;//取平均（坐标系方向相反）
    xvEstimateKF_Update(&vaEstimateKF,kalman_MotionAccel_n.Out,aver_v);
	//	xvEstimateKF_Update(&vaEstimateKF,kalman_MotionAccel_b.Out,aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		chassis_move.v_filter=vel_acc[0];//得到卡尔曼滤波后的速度
		if(fabs(chassis_move.v_filter) < 0.001f){
			chassis_move.v_filter = 0;					//速度太小了不能进行累计，不然x会飘
		}
		chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
	//如果想直接用轮子速度，不做融合的话可以这样
	//chassis_move.v_filter=(chassis_move.wheel_motor[0].para.vel-chassis_move.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
	//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
	//电机在线状态检测
		mod50++;
		if(	mod50>=50)
		{
				if(chassis_move.wheel_motor[1].para.online_flag>chassis_move.wheel_motor[1].para.pre_online_flag)
				{
					chassis_move.wheel_motor[1].para.pre_online_flag=chassis_move.wheel_motor[1].para.online_flag;
					chassis_move.wheel_motor[1].para.online_status=1;
				}
				else if(chassis_move.wheel_motor[1].para.online_flag==chassis_move.wheel_motor[1].para.pre_online_flag)
				{
					chassis_move.wheel_motor[1].para.online_status=0;
				}
				if(chassis_move.wheel_motor[0].para.online_flag>chassis_move.wheel_motor[0].para.pre_online_flag)
				{
					chassis_move.wheel_motor[0].para.pre_online_flag=chassis_move.wheel_motor[0].para.online_flag;
					chassis_move.wheel_motor[0].para.online_status=1;
				}
				else if(chassis_move.wheel_motor[0].para.online_flag==chassis_move.wheel_motor[0].para.pre_online_flag)
				{
				chassis_move.wheel_motor[0].para.online_status=0;
				}
				mod50=0;
		}
		
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}


