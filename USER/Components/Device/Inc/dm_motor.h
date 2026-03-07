
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include "motor_def.h"
#include "stdint.h"



/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
#include "motor_def.h"

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}Motor_CANFrameInfo_typedef;


/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct
{

  bool Initlized;    /*!< init flag */
  uint8_t  State; 	 /*!< Motor Message */
  uint16_t  P_int;   /*!< Motor Positon  uint16 */
	uint16_t  V_int;   /*!< Motor Velocity uint16 */
	uint16_t  T_int;   /*!< Motor Torque   uint16 */
	float  Position;   /*!< Motor Positon  */
  float  Velocity;   /*!< Motor Velocity */
  float  Torque;     /*!< Motor Torque   */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS   */
	float  Temperature_Rotor; /*!< Motor Temperature_Rotor */
  float  Angle;

}DM_Motor_Data_Typedef;

/**
 * @brief 专用于达妙电机的初始化结构体
 */
typedef struct
{
    Motor_Type_e motor_type;                       // 电机型号 (来自 motor_def.h)
    DM_Motor_Control_Mode_Type_e control_mode;     // 控制模式
    DM_Motor_Param_Range_Typedef param_limits;     // 硬件参数极限
    CAN_Init_Config_s can_init_config;             // 底层 CAN 配置
} DM_Motor_Init_Config_s;


/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
typedef struct
{
  float Position;
	float Velocity;
	float KP;
	float KD;
	float Torque;
	float Angle;
}DM_Motor_Control_Info_Typedef;


/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{

	DM_Motor_Control_Mode_Type_e	Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;
	DM_Motor_Param_Range_Typedef Param_Range;
	DM_Motor_Data_Typedef Data;
	
	CANInstance *motor_can_instance;
	DM_Motor_Control_Info_Typedef Control_Info;
}DM_Motor_Info_Typedef;



/* Externs ------------------------------------------------------------------*/



extern DM_Motor_Info_Typedef* DM_Motor_Init(DM_Motor_Init_Config_s *config);

extern void DM_Motor_Command(DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);

extern void DM_Motor_CAN_TxMessage(DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);

extern void DM_Motor_Set_MIT_Target(DM_Motor_Info_Typedef *motor, float pos, float vel, float kp, float kd, float torq);
extern void DM_Motor_Set_PosVel_Target(DM_Motor_Info_Typedef *motor, float pos, float vel_limit);

extern void DM_Motor_Control(void);
extern void DM_Motor_Enable_All(void);
extern void DM_Motor_Disable_All(void);


#endif
