#include "Up_board.h"

#include "fdcan.h"
#include "arm_math.h"
//	enum Enum_Sprint_Status Sprint_Status;
//    //弹仓开关
//  enum  Enum_Bulletcap_Status Bulletcap_Status;
//    //摩擦轮开关
//  enum  Enum_Fric_Status Fric_Status;
//    //自瞄锁住状态
//  enum Enum_MinPC_Aim_Status MiniPC_Aim_Status;
//    //迷你主机状态
//	enum Enum_MiniPC_Status MiniPC_Status;
//    //裁判系统UI刷新状态
//  enum Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status;
	//控制类型字节
    uint8_t control_type;
Up_borard_t Up_borard;
/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param Int_Min 整型最小值
 * @param Int_Max 整型最大值
 * @param Float_Min 浮点数最小值
 * @param Float_Max 浮点数最大值
 * @return float 浮点数
 */
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max)
{
    float tmp = (float)(x - Int_Min) / (float)(Int_Max - Int_Min);
    float out = tmp * (Float_Max - Float_Min) + Float_Min;
    return (out);
}
void upborad_fbdata(Up_borard_t *borad, uint8_t *rx_data,uint32_t data_len)
{ 
	
	
	 float gimbal_velocity_x, gimbal_velocity_y;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    enum Enum_Chassis_Control_Type chassis_control_type;
    //底盘和云台夹角（弧度制）
    float derta_angle;
    //float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y, tmp_omega, tmp_gimbal_pitch;
	if(data_len==FDCAN_DLC_BYTES_8)
	{//返回的数据有8个字节
		
    memcpy(&tmp_velocity_x,&rx_data[0],sizeof(uint16_t));
    memcpy(&tmp_velocity_y,&rx_data[2],sizeof(uint16_t));
    memcpy(&tmp_omega,&rx_data[4],sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch,&rx_data[5],sizeof(uint16_t));
    memcpy(&control_type,&rx_data[7],sizeof(uint8_t));
		gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x,0,0x7FFF,-1 *1,1);
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y,0,0x7FFF,-1 * 1,1);
		borad->V_X=gimbal_velocity_x;
		borad->V_Y=gimbal_velocity_y;
   // Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch,0,0x7FFF,-10.0f,30.0f);
		chassis_control_type = (control_type & 0x03);
		borad->Chassis_Control_Type= (control_type & 0x03);
    borad->Sprint_Status =(control_type>>2 & 0x01);
    borad->Bulletcap_Status = (control_type>>3 & 0x01);
   	borad-> Fric_Status = (control_type>>4 & 0x01);
    borad->MinPC_Aim_Status = (control_type>>5 & 0x01);
    borad->MiniPC_Status =(control_type>>6 & 0x01);
    borad->Referee_UI_Refresh_Status = (control_type>>7 & 0x01);
		
		borad->mode=chassis_control_type;
		
	}
}