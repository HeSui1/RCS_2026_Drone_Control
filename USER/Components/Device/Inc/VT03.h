
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : VT03.h
 * @brief          : VT03 Remote Control Driver Interface
 * @author         : [CQN/GrassFan Wang]
 * @date           : 2026/04/18
 * @version        : v1.0
 ******************************************************************************
 * @attention      :
 * 此模块负责 VT03 遥控器的数据解包、CRC校验与离线监控。
 * 设计原则：高内聚，低耦合。对外仅暴露解析后的标准控制数据和更新接口。
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VT03_H
#define VT03_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines -----------------------------------------------------------*/

/**
 * @brief Length of VT03 received data frame (21 Bytes)
 * 2(sof) + 8(ch/mode/etc) + 6(mouse_xyz) + 1(mouse_btn) + 2(key) + 2(crc) = 21
 */
#define VT03_RX_BUF_NUM     21u

/**
 * @brief VT03 Offline timeout threshold (e.g., 50ms depending on calling freq)
 */
#define VT03_OFFLINE_LIMIT  50u

/* VT03 Switch State Definitions */
#define VT03_SW_UP          1u
#define VT03_SW_DOWN        2u
#define VT03_SW_MID         3u

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 原始数据包结构体 (严格按照通信协议定义的位域)
 */
typedef struct __attribute__((packed))
{
    uint8_t sof_1;      // 0xA9
    uint8_t sof_2;      // 0x53
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;
    uint64_t reserved1:3; // <--- 修正：填补 3 bits 内存空洞，确保 mouse_x 对齐到第 10 字节

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint8_t reserved2:2;  // <--- 修正：填补 2 bits 内存空洞，确保 key 对齐到第 17 字节

    uint16_t key;
    uint16_t crc16;
} VT03_Raw_Data_t;

/**
 * @brief 键盘按键位域联合体 (优化内存并提升解析速度)
 */
typedef union
{
    struct
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
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
    uint16_t val;
} VT03_Key_t;

/**
 * @brief 标准化遥控器控制数据结构体 (供上层应用直接读取)
 * @note  将原始的位域数据转换为标准的数据类型，方便上层做控制算法计算
 */
typedef struct
{
    struct
    {
        int16_t ch_0;     // 通道 0 (右水平)
        int16_t ch_1;     // 通道 1 (右竖直)
        int16_t ch_2;     // 通道 2 (左水平)
        int16_t ch_3;     // 通道 3 (左竖直)
        int16_t wheel;    // 拨轮

        uint8_t mode_sw;  // 模式拨杆
        uint8_t pause;    // 暂停键
        uint8_t fn_1;     // 自定义按键1
        uint8_t fn_2;     // 自定义按键2
        uint8_t trigger;  // 扳机
    } rc;

    struct
    {
        int16_t x; 	//鼠标横移速度。左负右正
        int16_t y; 	//竖
        int16_t z;	//滚轮
        uint8_t press_l; //左键
        uint8_t press_r; //右键
        uint8_t press_m; //中键
    } mouse;

    VT03_Key_t key;       // 键盘按键状态

} VT03_Ctrl_t;

/**
 * @brief VT03 遥控器运行状态信息结构体
 */
typedef struct
{
    VT03_Raw_Data_t raw_data; // 原始数据包备份
    VT03_Ctrl_t     parsed;   // 解析后的可用控制数据

    uint16_t online_cnt;      // 离线检测计数器
    bool     is_lost;         // 丢失标志位：true为掉线，false为在线
} VT03_Info_t;

/* Exported variables ---------------------------------------------------------*/

/**
 * @brief 暴露给底层的 DMA 双缓冲接收数组 (32字节)
 */
extern __attribute__((section (".AXI_SRAM"), aligned(32))) uint8_t VT03_MultiRx_Buf[2][32];

/**
 * @brief 暴露给上层的遥控器信息实例
 */
extern VT03_Info_t vt03_info;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化 VT03 遥控器模块 (预留，可用于初始化结构体状态)
 */
void VT03_Init(void);

/**
 * @brief  解析接收到的 VT03 遥控器数据 (应在 DMA IDLE 接收完成回调中调用)
 * @param  rx_buf: 指向接收缓冲区的指针
 */
void VT03_Parse(volatile const uint8_t *rx_buf);

/**
 * @brief  监控遥控器在线状态 (应在 1ms 定时器任务或 FreeRTOS 任务中循环调用)
 */
void VT03_Monitor(void);

#endif /* VT03_H */

