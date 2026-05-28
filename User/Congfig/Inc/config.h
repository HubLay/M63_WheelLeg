/**
 * @file config.h
 * @author cjw
 * @brief 工程配置文件
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

//底盘或云台状态
//#define CHASSIS
#define GIMBAL

#define INS_TASK_DT      2                  //姿态估计任务周期      ms  
#define ROBOT_TASK_DT    2                  //机器人控制任务周期
#define DAEMON_TASK_DT   10                 //离线守护任务周期
#define UI_Task_DT       50                 //ui刷新周期
#define CAN_TRANSMIT_TASK_DT  1             //can发送任务周期
#define CMDProcess_TASK_DT 5                //控制命令处理周期  5ms
#define EMERGENCY_STOP_TASK_DT 10           //紧急停止任务周期

//功率控制相关
#define POWER_CONTROL 1 //启用功率控制
//#define BUFFER_LOOP

//遥控器选择
//#define USE_VT13
#define USE_DR16

/* 兵种/底盘类型选择*/
#define AGV      //舵轮底盘
//#define OMNI_WHEEL //全向轮底盘

//#define INFANTRY //步兵
//#define HERO  //英雄
//#define SENTRY //哨兵
#define BALANCE_CHASSIS //哨兵

/*轮组数据*/
#ifdef INFANTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef HERO
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#endif 

#ifdef SENTRY
#define Wheel_Diameter 0.12000000f // 轮子直径，单位为m
#define Chassis_Radius 0.46000000f // 底盘半径，单位为m
#endif

#ifdef BALANCE_CHASSIS

#define Robot_Mg       18.0f        //Kg
#define Chassis_Width  0.3789        //车宽 m
#define Chassis_Half_Width (Chassis_Width / 2.0f)
#define Wheel_Diameter 0.060f       // 轮子半径，单位为m

#define JOINT_MAX_TORQUE 35.0f      //关节电机的最大输出力矩，要和上位机对应

// #define UNFLLOW_ENABLE              //单底盘调试状态

#ifndef UNFLLOW_ENABLE
#define NORMAL_CHASSIS
#endif

#define V_MAX 1.8f
#define V_MAX_2 2.8f
#define V_MAX_SPIN 0.7f
#define Yaw_Angle_Resolution (1.5f / 57.3f)
#define Length_Angle_Resolution 0.008f
#define SPIN_OMEGA 8.0f
#define Acc_Max_1 3.0f
#define ACC_Max_2 3.0f

#define Reference_Rad (-0.5675f)

#define Length_MIN 0.16f
#define Length_MAX 0.37f

#define Length_MIN_SPIN 0.16f
#define Length_MAX_SPIN 0.27f

// #define H7_Offset_X -0.12f       //IMU距离车体中心的距离，IMU自身坐标系 m 量不出来硬调
// #define H7_Offset_Y -0.0f         //IMU距离车体中心的距离，IMU自身坐标系 m

#endif



/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
