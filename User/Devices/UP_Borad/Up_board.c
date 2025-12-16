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
	
	
    //底盘控制类型
    enum Enum_Chassis_Control_Type chassis_control_type;
    //float映射到int16之后的速度
    int8_t tmp_dr16_left_x, tmp_dr16_left_y, tmp_dr16_right_x, tmp_dr16_right_y;
	if(data_len==FDCAN_DLC_BYTES_8)
	{//返回的数据有8个字节
		
    memcpy(&tmp_dr16_left_x,&rx_data[0],sizeof(uint8_t));
    memcpy(&tmp_dr16_left_y,&rx_data[1],sizeof(uint8_t));
    memcpy(&tmp_dr16_right_x,&rx_data[2],sizeof(uint8_t));
    memcpy(&tmp_dr16_right_y,&rx_data[3],sizeof(uint8_t));
    memcpy(&control_type,&rx_data[7],sizeof(uint8_t));

		borad->Left_X  = (float)tmp_dr16_left_x / 100.0f;
		borad->Left_Y  = (float)tmp_dr16_left_y / 100.0f;
    borad->Right_X = (float)tmp_dr16_right_x / 100.0f;
		borad->Right_Y = (float)tmp_dr16_right_y / 100.0f;

		chassis_control_type = (control_type & 0x03);

		borad->mode=chassis_control_type;
		
	}
}