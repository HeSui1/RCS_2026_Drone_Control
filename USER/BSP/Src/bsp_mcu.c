/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.c
  * @brief          : MCU peripheral initialization functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_mcu.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "bsp_dwt.h"
#include "bmi088.h"
#include "usb_device.h"



/**
  * @brief Initializes the MCU.
  */
void MCU_Init(void)
{
  /* ----------------------- BSP Init ----------------------- */
  DWT_Init(168);
	BSP_PWM_Init();
	BSP_GPIO_Init();
  CANServiceInit();
  BSP_USART_Init();
	BSP_ADC_Init();
	MX_USB_DEVICE_Init();
  /* ----------------------- Device Init ----------------------- */
  BMI088_Init();
		// ============================================================
  // 【防疯转保护 1】硬件冻结
  // 告诉 STM32：当我在 Keil/IDE 里按下暂停时，FDCAN 也要立刻暂停！
  // ============================================================
  
  // 尝试使用标准宏（适用于 H723 单核）
//	#define __HAL_DBGMCU_FREEZE_FDCAN
  #ifdef __HAL_DBGMCU_FREEZE_FDCAN
      __HAL_DBGMCU_FREEZE_FDCAN();
 
  #endif
	
}
//------------------------------------------------------------------------------
