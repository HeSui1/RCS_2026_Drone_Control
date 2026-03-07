/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Remote_Control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_COD_H
#define REMOTE_CONTROL_COD_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
/* Exported defines -----------------------------------------------------------*/


// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15


/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
/**
 * @brief offset of remote control channel data
 */
//#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status of the keyboard.
 */
typedef enum
{
	UP,			/*!< up */
	SHORT_DOWN,	/*!< short time down */
	DOWN,		/*!< long time down */
	PRESS,		/*!< 0->1 */
	RELAX,		/*!< 1->0 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

// typedef struct
// {
// 	uint16_t Count;
// 	KeyBoard_Status_e Status;
// 	KeyBoard_Status_e last_Status;
// 	bool last_KEY_PRESS;
// 	bool KEY_PRESS;
// }KeyBoard_Info_Typedef;

// typedef struct
// {
// 	KeyBoard_Info_Typedef press_l;
// 	KeyBoard_Info_Typedef press_r;
// 	KeyBoard_Info_Typedef W;
// 	KeyBoard_Info_Typedef S;
// 	KeyBoard_Info_Typedef A;
// 	KeyBoard_Info_Typedef D;
// 	KeyBoard_Info_Typedef SHIFT;
// 	KeyBoard_Info_Typedef CTRL;
// 	KeyBoard_Info_Typedef Q;
// 	KeyBoard_Info_Typedef E;
// 	KeyBoard_Info_Typedef R;
// 	KeyBoard_Info_Typedef F;
// 	KeyBoard_Info_Typedef G;
// 	KeyBoard_Info_Typedef Z;
// 	KeyBoard_Info_Typedef X;
// 	KeyBoard_Info_Typedef C;
// 	KeyBoard_Info_Typedef V;
// 	KeyBoard_Info_Typedef B;
// }Remote_Pressed_Typedef;


/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
//
// /* Mouse Exported defines -----------------------------------------------------*/
// #define MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
// #define MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )
// #define MOUSE_Z_MOVE_SPEED    (remote_ctrl.mouse.z )
// #define MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
// #define MOUSE_PRESSED_RIGHT   (remote_ctrl.mouse.press_r)
//
// /* KeyBoard Exported defines --------------------------------------------------*/
// #define KeyBoard_W            (remote_ctrl.key.set.W)
// #define KeyBoard_S            (remote_ctrl.key.set.S)
// #define KeyBoard_A            (remote_ctrl.key.set.A)
// #define KeyBoard_D            (remote_ctrl.key.set.D)
// #define KeyBoard_SHIFT        (remote_ctrl.key.set.SHIFT)
// #define KeyBoard_CTRL         (remote_ctrl.key.set.CTRL)
// #define KeyBoard_Q            (remote_ctrl.key.set.Q)
// #define KeyBoard_E            (remote_ctrl.key.set.E)
// #define KeyBoard_R            (remote_ctrl.key.set.R)
// #define KeyBoard_F            (remote_ctrl.key.set.F)
// #define KeyBoard_G            (remote_ctrl.key.set.G)
// #define KeyBoard_Z            (remote_ctrl.key.set.Z)
// #define KeyBoard_X            (remote_ctrl.key.set.X)
// #define KeyBoard_C            (remote_ctrl.key.set.C)
// #define KeyBoard_V            (remote_ctrl.key.set.V)
// #define KeyBoard_B            (remote_ctrl.key.set.B)
//
// /* Exported functions prototypes ---------------------------------------------*/
// /**
//   * @brief  convert the remote control received message
//   */
 extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
// /**
//   * @brief  clear the remote control data while the device offline
//   */
 extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);
//
// /**
//   * @brief  report the cover status that acrroding the key_R swicthing
//   */
// extern bool Key_R(void);
// /**
//   * @brief  switch the shooter mode that acrroding the key_B
//   */
// extern bool Key_B(void);
//
// /**
//   * @brief  report the auto aim status that acrroding the mouse right swicthing
//   */
// extern bool Mouse_Pressed_Right(void);
// /**
//   * @brief  report the fire status that acrroding the mouse left swicthing
//   */
// extern bool Mouse_Pressed_Left(void);

// 待测试的位域结构体,可以极大提升解析速度
typedef union
{
	struct // 用于访问键盘状态
	{
		uint16_t w : 1;
		uint16_t s : 1;
		uint16_t d : 1;
		uint16_t a : 1;
		uint16_t shift : 1;
		uint16_t ctrl : 1;
		uint16_t q : 1;
		uint16_t e : 1;
		uint16_t r : 1;
		uint16_t f : 1;
		uint16_t g : 1;
		uint16_t z : 1;
		uint16_t x : 1;
		uint16_t c : 1;
		uint16_t v : 1;
		uint16_t b : 1;
	};
	uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
	struct
	{
		int16_t rocker_l_; // 左水平
		int16_t rocker_l1; // 左竖直
		int16_t rocker_r_; // 右水平
		int16_t rocker_r1; // 右竖直
		int16_t dial;      // 侧边拨轮

		uint8_t switch_left;  // 左侧开关
		uint8_t switch_right; // 右侧开关
	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

	uint8_t key_count[3][16];
} RC_ctrl_t;

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

extern RC_ctrl_t rc_data;

#endif //REMOTE_CONTROL_H


