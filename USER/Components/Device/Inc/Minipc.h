/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : HeSui
  * @date           : 2026/03/06
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 
// 告诉编译器取消字节对齐，保证和上位机的数据包完全一致
#pragma pack(push, 1)

// 发送给视觉的数据包 (对应上位机 GimbalToVision)
typedef struct {
  uint8_t  head[2];       // 固定为 'S', 'P'
  uint8_t  mode;          // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float    q[4];          // wxyz顺序四元数
  float    yaw;           // 云台当前yaw
  float    yaw_vel;       // 云台当前yaw速度
  float    pitch;         // 云台当前pitch
  float    pitch_vel;     // 云台当前pitch速度
  float    bullet_speed;  // 当前弹速
  uint16_t bullet_count;  // 子弹累计发射数量
  uint16_t crc16;         // CRC16校验位
} GimbalToVision_t;

// 接收视觉的数据包 (对应上位机 VisionToGimbal)
typedef struct {
  uint8_t  head[2];       // 固定为 'S', 'P'
  uint8_t  mode;          // 0: 不控制, 1: 控制云台不开火，2: 控制云台且开火
  float    yaw;           // 目标yaw
  float    yaw_vel;       // 目标yaw前馈速度
  float    yaw_acc;       // 目标yaw前馈加速度
  float    pitch;         // 目标pitch
  float    pitch_vel;     // 目标pitch前馈速度
  float    pitch_acc;     // 目标pitch前馈加速度
  uint16_t crc16;         // CRC16校验位
} VisionToGimbal_t;

#pragma pack(pop) // 恢复默认对齐方式

// 声明全局变量供其他文件（如云台控制任务）调用
extern GimbalToVision_t gimbal_tx_data;
extern VisionToGimbal_t vision_rx_data;

void MiniPC_Transmit_Info(void);
void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len);
#endif