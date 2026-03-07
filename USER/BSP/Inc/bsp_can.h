/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "fdcan.h"

// 最多能够支持的CAN设备数 (根据你的Conservetive配置，这里可以设为32，与硬件Filter数匹配)
#define CAN_MX_REGISTER_CNT 32
#define DEVICE_CAN_CNT 3           // H7通常有3个CAN

    /* can instance typedef */
#pragma pack(1)
    typedef struct _
    {
        FDCAN_HandleTypeDef *can_handle; // 句柄变为FDCAN
        FDCAN_TxHeaderTypeDef txconf;    // 发送配置变为FDCAN
        uint32_t tx_id;
        // uint32_t tx_mailbox;          // FDCAN不需要用户管理邮箱号，删除
        uint8_t tx_buff[64];             // 扩大为64字节以兼容FDCAN
        uint8_t rx_buff[64];             // 扩大为64字节
        uint32_t rx_id;
        uint8_t rx_len;
        // 接收回调函数
        void (*can_module_callback)(struct _ *);
        void *id;
    } CANInstance;
#pragma pack()

    /* CAN实例初始化结构体 */
    typedef struct
    {
        FDCAN_HandleTypeDef *can_handle;
        uint32_t tx_id;
        uint32_t rx_id;
        void (*can_module_callback)(CANInstance *);
        void *id;
    } CAN_Init_Config_s;

    /**
     * @brief 注册一个CAN实例
     */
    CANInstance *CANRegister(CAN_Init_Config_s *config);

    /**
     * @brief 发送CAN消息
     */
    uint8_t CANTransmit(CANInstance *_instance, float timeout);


    extern void CANServiceInit();

#endif