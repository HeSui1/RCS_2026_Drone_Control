/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control_COD.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "bsp_uart.h"
#include "gimbal.h"
static uint32_t last_loop_time_us = 0;

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */

 void CAN_Task(void const * argument)
{
 	static float motor_dt;
 	static float motor_start;
  TickType_t CAN_Task_SysTick = 0;
	
	while (pitch_dm_motor == NULL) 
    {
        osDelay(2); 
    }
	
//	DM_Motor_Command(pitch_dm_motor,Motor_Save_Zero_Position);    //设定电机零点
	

	DM_Motor_Enable_All();

  osDelay(30);	
	for(;;)
  {


	  CAN_Task_SysTick = osKernelSysTick();
		motor_start = DWT_GetTimeline_ms();
		
		// ==========================================
		// 频率硬锁限制 (2kHz 限流)
		uint32_t now_us = DWT_GetTimeline_us(); 
		if (now_us - last_loop_time_us < 500) 
		{
				continue; 
		}
		last_loop_time_us = now_us;
		// ==========================================
		
		DM_Motor_Control();
				
				
		DJIMotorControl();

		motor_dt = DWT_GetTimeline_ms() - motor_start;
		osDelay(1);
  }
 
}


