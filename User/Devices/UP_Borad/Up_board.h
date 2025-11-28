#ifndef __UP_BOARD_H__
#define __UP_BOARD_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"
/**
 * @brief µ×ÅÌ³å´Ì×´Ì¬Ã¶¾Ù
 *
 */
enum Enum_Sprint_Status 
{
    Sprint_Status_DISABLE=0, 
    Sprint_Status_ENABLE
};


/**
 * @brief µ×ÅÌ¿ØÖÆÀàÐÍ
 *
 */
enum Enum_Chassis_Control_Type 
{
    Chassis_Control_Type_DISABLE=0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_SPIN_NePositive
};
/* Exported types ------------------------------------------------------------*/

/**
 * @brief ÔÆÌ¨Pitch×´Ì¬Ã¶¾Ù
 *
 */
enum Enum_Pitch_Control_Status  
{
    Pitch_Status_Control_Free=0, 
    Pitch_Status_Control_Lock 
};

enum Enum_MinPC_Aim_Status 
{
    MinPC_Aim_Status_DISABLE=0,
    MinPC_Aim_Status_ENABLE
};

/**
 * @brief Ä¦²ÁÂÖ×´Ì¬
 *
 */
enum Enum_Fric_Status
{
    Fric_Status_CLOSE=0,
    Fric_Status_OPEN
};


/**
 * @brief µ¯²Õ×´Ì¬ÀàÐÍ
 *
 */
enum Enum_Bulletcap_Status 
{
    Bulletcap_Status_CLOSE=0,
    Bulletcap_Status_OPEN
};


/**
 * @brief µ×ÅÌÍ¨Ñ¶×´Ì¬
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE
};

/**
 * @brief ÔÆÌ¨Í¨Ñ¶×´Ì¬
 *
 */
enum Enum_Gimbal_Status 
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE
};
/**
 * @brief ÃÔÄãÖ÷»ú×´Ì¬
 *
 */
enum Enum_MiniPC_Status 
{
    MiniPC_Status_DISABLE = 0,
    MiniPC_Status_ENABLE
};
/**
 * @brief ²ÃÅÐÏµÍ³UIË¢ÐÂ×´Ì¬
 *
 */
enum Enum_Referee_UI_Refresh_Status 
{
    Referee_UI_Refresh_Status_DISABLE = 0,
    Referee_UI_Refresh_Status_ENABLE
};

typedef struct
{
	float V_X;
	float V_Y;
	float omega;
	float gimbal_pitch;
	enum Enum_Chassis_Control_Type Chassis_Control_Type;
	enum Enum_Sprint_Status Sprint_Status;
	enum Enum_Bulletcap_Status Bulletcap_Status;
	enum Enum_Fric_Status Fric_Status;
	enum Enum_MinPC_Aim_Status MinPC_Aim_Status;
	enum Enum_MiniPC_Status MiniPC_Status;
	enum Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status;
	uint8_t mode;
	uint8_t pre_mode;
}Up_borard_t;
extern Up_borard_t Up_borard;
void upborad_fbdata(Up_borard_t *borad, uint8_t *rx_data,uint32_t data_len);


#endif 
