/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control_COD.h"
#include "stdlib.h"
//#include "memory.h"
#include "bsp_dwt.h"
// 屏蔽 bsp_log，保留原代码结构
#define LOGINFO(fmt, ...)
#define LOGWARNING(fmt, ...)
#define LOGERROR(fmt, ...)

/* CAN实例存储池 */
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx = 0; // 全局CAN实例索引

/* 过滤器索引管理 (H7专用) */
// FDCAN1/2/3 分别维护自己的过滤器索引，从0开始
static uint32_t can1_filter_idx = 0;
static uint32_t can2_filter_idx = 0;
static uint32_t can3_filter_idx = 0;

/* ---------------- 辅助函数：DLC转换 ---------------- */
// H7的DataLength是枚举值(0x00080000)，需要和整数(8)互相转换

static uint32_t DLC_Bytes_To_Enum(uint8_t bytes) {
    if (bytes <= 8) return (bytes << 16); // FDCAN_DLC_BYTES_0 ~ 8 的规律
    // 对于 >8 的情况 (FDCAN)，暂时只处理常用值，DJI电机一般只用8
    return FDCAN_DLC_BYTES_8;
}

static uint8_t DLC_Enum_To_Bytes(uint32_t dlc_enum) {
    if (dlc_enum <= FDCAN_DLC_BYTES_8) return (dlc_enum >> 16);
    return 8; // 默认返回8
}

/* ---------------- 内部函数 ---------------- */

/**
 * @brief 添加过滤器 (H7 RAM模式)
 * 保留了原作者的负载均衡思想：奇数ID进FIFO0，偶数ID进FIFO1
 */
static void CANAddFilter(CANInstance *_instance)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式
    sFilterConfig.FilterID1 =  0x00000000;//_instance->rx_id;   // 必须匹配的ID
    sFilterConfig.FilterID2 =  0x00000000;//0x7FF;              // 掩码全1，表示精确匹配

    // 负载均衡策略：偶数ID -> FIFO1, 奇数ID -> FIFO0 (与原代码保持逻辑一致)
    // 注意：CubeMX里必须开启两个FIFO的中断
    if (_instance->rx_id & 1) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    } else {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    }

    // 分配过滤器索引
    if (_instance->can_handle == &hfdcan1) {
        sFilterConfig.FilterIndex = can1_filter_idx++;
    } else if (_instance->can_handle == &hfdcan2) {
        sFilterConfig.FilterIndex = can2_filter_idx++;
    } else if (_instance->can_handle == &hfdcan3) {
        sFilterConfig.FilterIndex = can3_filter_idx++;
    }

    // 写入硬件
    HAL_FDCAN_ConfigFilter(_instance->can_handle, &sFilterConfig);
    // 配置全局过滤：拒收非匹配帧
    HAL_FDCAN_ConfigGlobalFilter(_instance->can_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

void CANServiceInit()
{
    // 启动所有CAN并开启中断
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

/* ---------------- 外部接口 ---------------- */

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    // if (!idx) {
    //     CANServiceInit(); // 第一次注册时启动硬件
    // }
    if (idx >= CAN_MX_REGISTER_CNT) {
        while (1); // 错误：超过最大实例数
    }

    // 查重逻辑保留
    for (size_t i = 0; i < idx; i++) {
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle) {
            while (1); // 错误：ID冲突
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    // 配置发送头 (默认 Classic CAN)
    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = FDCAN_STANDARD_ID;
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8; // 默认8字节
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;  // 关闭BRS
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;   // 经典模式
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);
    can_instance[idx++] = instance;

    return instance;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    // H7 FDCAN 发送逻辑：直接写入 Tx FIFO，非阻塞
    // 我们配置了 32 深度的 Tx FIFO，很难发满，所以这里不进行 while 死等，提高实时性

    // 如果需要发送长度非8字节，需要在这里更新 DataLength
    // 默认为8，DJI电机不需要改动

	uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle);
	
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
    {
        // 发送失败 (FIFO满)
        return 0;
    }
    return 1;
}

/* ---------------- 回调函数 ---------------- */

/**
 * @brief 统一处理接收 FIFO 的数据
 */
static void CANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    FDCAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[64]; // 使用局部缓存，最大64字节

    // 循环读取直到FIFO为空
    while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) == HAL_OK)
        {
            // 软件路由：遍历查找匹配的实例
            for (size_t i = 0; i < idx; ++i)
            {
                if (_hcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
                {
                    if (can_instance[i]->can_module_callback != NULL)
                    {
                        // 转换长度枚举为实际字节数
                        uint8_t real_len = 8 ;
                        can_instance[i]->rx_len = real_len;
                        // 拷贝数据
                        memcpy(can_instance[i]->rx_buff, can_rx_buff, real_len);
                        // 回调
                        can_instance[i]->can_module_callback(can_instance[i]);
                    }
                    break; // 找到后跳出循环，处理下一帧
                }
            }
        }
    }
}

/* 重写 HAL 库的中断回调 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
    }
}