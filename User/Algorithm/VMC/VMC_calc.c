#include "VMC_calc.h"

#include "my_kalman.h"

my_kalman kalman_right_d_theta_s;
my_kalman kalman_right_d_L0_s;
my_kalman kalman_right_d_alpha_s;
my_kalman kalman_left_d_theta_s;
my_kalman kalman_left_d_L0_s;
my_kalman kalman_left_d_alpha_s;

void VMC_init(vmc_leg_t *vmc)//给杆长赋值
{
	//左上第一条边为l1，逆时针编号
	vmc->l5=0.00f;//AE长度 //单位为m
	vmc->l1=0.215f;//单位为m
	vmc->l2=0.258f;//单位为m
	vmc->l3=0.258f;//单位为m
	vmc->l4=0.215f;//单位为m

	//Ozone调参
	kalman_Init(&kalman_left_d_L0_s, 0.99,0.01,0,1);
	kalman_Init(&kalman_left_d_theta_s, 0.90,0.01,0,1);
	kalman_Init(&kalman_left_d_alpha_s, 0.80,0.01,0,1);
	kalman_Init(&kalman_right_d_L0_s, 0.99,0.01,0,1);
	kalman_Init(&kalman_right_d_theta_s, 0.90,0.01,0,1);
	kalman_Init(&kalman_right_d_alpha_s, 0.80,0.01,0,1);


}


void VMC_calc_1_right(vmc_leg_t *vmc,INS_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
{	
	//如果定义车体符合右手系，这里VMC定义的应该是左腿x朝前，右腿x朝后
	//两边腿的VMC X轴方向不一致 导致角度的正方向什么也不一样 面朝外侧逆时针方向为正
	//有关x方向的变量应该刚好是倒过来的
		static float PitchR=0.0f;
	  static float PithGyroR=0.0f;
	  PitchR=ins->Pitch;								//IMU装的方向和解算方向不一致导致
	  PithGyroR=ins->Gyro[0];
	
	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标
			
		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2.0f*vmc->l2*(vmc->XD - vmc->XB);
		vmc->B0 = 2.0f*vmc->l2*(vmc->YD - vmc->YB);
		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
		vmc->phi2 = 2.0f*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C点直角坐标
		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
		//C点极坐标
		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);		//虚拟杆长L0
		vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta		
						
	  //等效杆长与机体Y轴夹角
	  vmc->alpha=pi/2.0f-vmc->phi0;				//左右腿x正方向不同，但公式是一样的，实际上左右腿的角度在相同姿态下会因为这个差一个负号

		if( vmc->alpha>pi) vmc->alpha-=2.0f*pi;
		else if( vmc->alpha<-pi) vmc->alpha+=2.0f*pi;	
		
		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0;
			vmc->first_flag=1;
		}

		//计算雅可比矩阵
		vmc->j11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
		vmc->j21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));

		//d_phi0计算应改为雅可比矩阵求解
		vmc->d_L0   = vmc->j11 * vmc->d_phi1 + vmc->j21 * vmc->d_phi4;
		vmc->d_phi0 = vmc->j12 * vmc->d_phi1 + vmc->j22 * vmc->d_phi4;
		
		//vmc->d_phi0 = (vmc->XC *vmc->d_YC - vmc->YC * vmc->d_XC ) / (vmc->XC*vmc->XC + vmc->YC *vmc->YC );

	//	vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
		vmc->d_alpha=0.0f-vmc->d_phi0;
		
		//关注一下theta的计算
		//两边腿的VMC X轴方向不一致，左腿超前右腿朝后  导致角度的正方向什么也不一样   面朝外侧逆时针方向为正
		vmc->theta=pi/2.0f-PitchR-vmc->phi0;//得到状态变量1

		if(vmc->theta > pi) vmc->theta -= 2.0f * pi;
		else if(vmc->theta < -pi) vmc->theta += 2.0f * pi;

		vmc->d_theta=(-PithGyroR-vmc->d_phi0);//得到状态变量2
		
		vmc->last_phi0=vmc->phi0;

    //vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;

		kalman_set_now(&kalman_right_d_L0_s, vmc->d_L0);
		kalman_set_now(&kalman_right_d_alpha_s, vmc->d_alpha);
		kalman_set_now(&kalman_right_d_theta_s, vmc->d_theta);

		Recv_Adjust_PeriodElapsedCallback(&kalman_right_d_L0_s);
		Recv_Adjust_PeriodElapsedCallback(&kalman_right_d_alpha_s);
		Recv_Adjust_PeriodElapsedCallback(&kalman_right_d_theta_s);

		vmc->kalman_d_L0    = kalman_right_d_L0_s.Out;
		vmc->kalman_d_alpha = kalman_right_d_alpha_s.Out;
		vmc->kalman_d_theta = kalman_right_d_theta_s.Out;

}


void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
{		
	  static float PitchL=0.0f;
	  static float PithGyroL=0.0f;
	  PitchL=0.0f-ins->Pitch;				
	  PithGyroL=0.0f-ins->Gyro[0];
	
		vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标
			
		vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2.0f*vmc->l2*(vmc->XD - vmc->XB);
		vmc->B0 = 2.0f*vmc->l2*(vmc->YD - vmc->YB);
		vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
		vmc->phi2 = 2.0f*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C点直角坐标
		vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
		vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
		//C点极坐标
		vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
					
	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta		
	  vmc->alpha=pi/2.0f-vmc->phi0 ;
			if( vmc->alpha>pi) vmc->alpha-=2.0f*pi;
		else if( vmc->alpha<-pi) vmc->alpha+=2.0f*pi;	
		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0 ;
			vmc->first_flag=1;
		}

		//计算雅可比矩阵
		vmc->j11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
		vmc->j21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
		
		//vmc->d_phi0=(vmc->phi0-vmc->last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
		
		vmc->d_L0   = vmc->j11 * vmc->d_phi1 + vmc->j21 * vmc->d_phi4;
		vmc->d_phi0 = vmc->j12 * vmc->d_phi1 + vmc->j22 * vmc->d_phi4;

		//vmc->d_phi0=(vmc->XC *vmc->d_YC - vmc->YC * vmc->d_XC ) / (vmc->XC*vmc->XC + vmc->YC *vmc->YC );

		vmc->d_alpha=0.0f-vmc->d_phi0 ;
		
		vmc->theta=pi/2.0f-PitchL-vmc->phi0;//得到状态变量1

		if(vmc->theta > pi) vmc->theta -= 2.0f * pi;
		else if(vmc->theta < -pi) vmc->theta += 2.0f * pi;

		vmc->d_theta=(-PithGyroL-vmc->d_phi0);//得到状态变量2
		
		vmc->last_phi0=vmc->phi0 ;

		//vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
    vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;

		kalman_set_now(&kalman_left_d_L0_s, vmc->d_L0);
		kalman_set_now(&kalman_left_d_alpha_s, vmc->d_alpha);
		kalman_set_now(&kalman_left_d_theta_s, vmc->d_theta);

		Recv_Adjust_PeriodElapsedCallback(&kalman_left_d_L0_s);
		Recv_Adjust_PeriodElapsedCallback(&kalman_left_d_alpha_s);
		Recv_Adjust_PeriodElapsedCallback(&kalman_left_d_theta_s);

		vmc->kalman_d_L0    = kalman_left_d_L0_s.Out;
		vmc->kalman_d_alpha = kalman_left_d_alpha_s.Out;
		vmc->kalman_d_theta = kalman_left_d_theta_s.Out;

}

void VMC_calc_2(vmc_leg_t *vmc)//计算期望的关节输出力矩
{	
	vmc->torque_set[0]=vmc->j11*vmc->F0+vmc->j12*vmc->Tp;//得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力 
	vmc->torque_set[1]=vmc->j21*vmc->F0+vmc->j22*vmc->Tp;//得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩 
}


int8_t Air_Time_R;
extern char Mes[100];
extern UART_HandleTypeDef huart7;
float FnR;
uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins)
{
	static int8_t Status = 0;
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f * (9.81f + ins->MotionAccel_n[2]);//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
// 	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
// +0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

	FnR=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f * (9.81f + ins->MotionAccel_n[2]
		- vmc->dd_L0 * arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

	if(vmc->FN < 45.0f)
	{//离地了
		Air_Time_R ++;
	}

	if(vmc->FN > 100.0f)
	{
		Air_Time_R = 0;
		Status = 0;
	}

	if(Status == 0 && Air_Time_R > 5){
		Status = 1;
	}

	// sprintf(Mes, "%d,%d,%f\n", Status, Air_Time_R, vmc->FN);
	// HAL_UART_Transmit_DMA(&huart7, Mes, strlen(Mes));

	return Status;

}

int8_t Air_Time_L;
float FnL;
uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins)
{
	static int8_t Status = 0;
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f * (9.81f + ins->MotionAccel_n[2]);//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
//	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
//+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

	FnL=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f * (9.81f + ins->MotionAccel_n[2]
		- vmc->dd_L0 * arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));


	if(vmc->FN < 45.0f)
	{//离地了
		Air_Time_L ++;
	}

	if(vmc->FN > 100.0f)
	{
		Air_Time_L = 0;
		Status = 0;
	}

	if(Status == 0 && Air_Time_L > 5){
		Status = 1;
	}

	return Status;
}

float LQR_K_calc(float *coe,float len)
{
   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}


