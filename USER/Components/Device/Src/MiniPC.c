/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions 
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "CRC.h"       // 引入工程原有的官方 CRC 库
#include <string.h>    // 需要用到 memcpy


// 实例化全局变量
GimbalToVision_t gimbal_tx_data;
VisionToGimbal_t vision_rx_data;



/**
  * @brief  发送云台状态给视觉
  * @note   在 FreeRTOS 的发送任务中，以固定频率(如 1ms 或 2ms)调用此函数
  */
void MiniPC_Transmit_Info(void)
{
    // 1. 强制写入帧头
    gimbal_tx_data.head[0] = 'S';
    gimbal_tx_data.head[1] = 'P';

    // 2. 利用官方自带的 Append 函数计算并追加 CRC16
    // 它会自动计算前面数据的内容，并将 16位 的校验码直接塞入结构体末尾的 crc16 变量中
    Append_CRC16_Check_Sum((uint8_t *)&gimbal_tx_data, sizeof(gimbal_tx_data));

    // 3. 通过 USB CDC 高速接口发送
    CDC_Transmit_HS((uint8_t *)&gimbal_tx_data, sizeof(gimbal_tx_data));
}


/**
  * @brief  解析来自视觉的数据包
  * @note   此函数在底层的 CDC_Receive_HS 中断回调中被调用
  */
void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len)
{
    // 1. 检查长度：如果收到的数据长度不等于视觉下发包的长度，说明发生粘包/断包，直接丢弃
    if (*Len != sizeof(VisionToGimbal_t)) {
        return;
    }

    // 2. 检查帧头：确保是 'S' 和 'P'
    if (Buff[0] != 'S' || Buff[1] != 'P') {
        return;
    }

    // 3. 校验 CRC16
    if (Verify_CRC16_Check_Sum(Buff, sizeof(VisionToGimbal_t))) {

        // 4. 校验通过，将底层缓冲区的数据安全地拷贝到全局变量，供云台 PID 控制使用
        memcpy(&vision_rx_data, Buff, sizeof(VisionToGimbal_t));

        // 可选提示：在这里可以清零一个全局的 "offline_counter" 变量，用来做视觉掉线保护
    }
}