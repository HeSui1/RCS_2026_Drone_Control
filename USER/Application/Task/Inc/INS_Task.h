/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


typedef struct
{
	float q[4];     // 新增：四元数，顺序 w, x, y, z
	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度
	// 还需要增加角速度数据
	float Roll;
	float Pitch;
	float Yaw;
	float YawTotalAngle;
} attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct 
{
	float Pitch_Angle;
	float Yaw_Angle;
	float Yaw_TolAngle;
	float Roll_Angle;

  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

  float Angle[3];
	float Gyro[3];	
	float Accel[3];
	
	float Last_Yaw_Angle;
	int16_t YawRoundCount;

}INS_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info; 
extern attitude_t INS_Output;    // 外壳数据

attitude_t *INS_Init(void);
void INS_Task(void const * argument);

#endif //INS_TASK_H
