#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define ROLL_PID_KP 200.0f
#define ROLL_PID_KI 0.0f //不用积分项
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT  20.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f //不用积分项
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f //不用积分项
#define TURN_PID_KD 0.2f
#define TURN_PID_MAX_OUT  2.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f
#define Mg 13.0f
typedef struct
{
  Joint_Motor_t joint_motor[4];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float target_v;
	float x_set;//期望位置，单位是m
	float turn_set;//期望yaw轴弧度
	float target_turn;
	float leg_set;//期望腿长，单位是m
	float leg_lx_set;
	float target_leg_lx_set;
	float leg_left_set;
	float leg_right_set;
	float last_leg_set;
	float last_leg_left_set;
	float last_leg_right_set;
	float roll_set;
	float roll_target;
	float now_roll_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//遥控器启动标志
	
	uint8_t jump_flag;//跳跃标志
	uint8_t jump_flag2;//跳跃标志
	
	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志
	uint8_t leg_theta_flag;//腿是否达到可以自起的标志位
	
	uint32_t count_key;
	float jump_leg;
	uint32_t jump_time_r;
	uint32_t jump_time_l;
	uint8_t jump_status_r;
	uint8_t jump_status_l;
	
	//这些flag是当时调试用的，可以删除不用
	uint8_t stand_ready_flag_r;									//右腿站立完成标志
	uint16_t stand_ready_time_r;
	uint8_t stand_ready_flag_l;									//左腿站立完成标志
	uint16_t stand_ready_time_l;
	uint16_t stand_ok_time;
	uint8_t stand_ready_flag;										//用于单独判断某个腿是否完成站立的计数
	uint8_t stand_ready_ok_flag;								//车体站立标志，只用了右腿角度判断
	uint16_t stand_ready_ok_time;
	uint8_t stop_flag;						//行驶过程中车倒下
	
} chassis_t;


extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg);
void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,PidTypeDef *leg);
#endif




