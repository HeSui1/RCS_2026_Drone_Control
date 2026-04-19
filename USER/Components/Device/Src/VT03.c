
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : VT03.c
 * @brief          : VT03 Remote Control Driver
 ******************************************************************************
 */
/* USER CODE END Header */

#include "VT03.h"
#include "CRC.h"   // 引入公共 CRC 算法库
#include <string.h>

/* 定义暴露给上层使用的全局结构体实例 */
VT03_Info_t vt03_info = {
    .online_cnt = 0,
    .is_lost = true
};

/**
 * @brief VT03 遥控器 UART RxDMA 双缓冲区
 * @note  放入 AXI_SRAM 并按 32 字节对齐。
 * 注意：内层数组大小写 32 而不是 VT03_RX_BUF_NUM (21)，
 * 是为了完美契合 D-Cache 的 32 字节 Cache Line 机制，防止 Invalidate 时发生内存踩踏。
 */
__attribute__((section (".AXI_SRAM"), aligned(32))) uint8_t VT03_MultiRx_Buf[2][32];

/**
 * @brief  解析接收到的 VT03 遥控器数据
 * @param  rx_buf: 指向接收缓冲区的指针 (通常是 DMA 双缓冲的某一个)
 */
void VT03_Parse(volatile const uint8_t *rx_buf)
{
    // 1. 指针判空保护
    if (rx_buf == NULL) return;

    // 2. 帧头校验 (说明书明确规定 Header1 为 0xA9, Header2 为 0x53)
    if (rx_buf[0] != 0xA9 || rx_buf[1] != 0x53) return;

    // 3. CRC16 校验 (总长 21 字节)
    if (Verify_CRC16_Check_Sum((uint8_t *)rx_buf, VT03_RX_BUF_NUM) == false)
    {
        return; // 校验失败，直接丢弃该帧，防止乱码导致机器人疯跑
    }

    // 4. 将有效数据拷贝到原始数据结构体中 (触发编译器的位域自动映射)
    memcpy(&vt03_info.raw_data, (const void *)rx_buf, VT03_RX_BUF_NUM);

    // 5. 数据标准化转换 (赋值给上层实际调用的 parsed 结构体)

    // 摇杆与拨轮：原范围 [364, 1024, 1684]，减去 1024 映射到 [-660, 0, 660]
    vt03_info.parsed.rc.ch_0  = (int16_t)vt03_info.raw_data.ch_0 - 1024;
    vt03_info.parsed.rc.ch_1  = (int16_t)vt03_info.raw_data.ch_1 - 1024;
    vt03_info.parsed.rc.ch_2  = (int16_t)vt03_info.raw_data.ch_2 - 1024;
    vt03_info.parsed.rc.ch_3  = (int16_t)vt03_info.raw_data.ch_3 - 1024;
    vt03_info.parsed.rc.wheel = (int16_t)vt03_info.raw_data.wheel - 1024;

    // 开关与按键：直接赋值
    vt03_info.parsed.rc.mode_sw = vt03_info.raw_data.mode_sw;
    vt03_info.parsed.rc.pause   = vt03_info.raw_data.pause;
    vt03_info.parsed.rc.fn_1    = vt03_info.raw_data.fn_1;
    vt03_info.parsed.rc.fn_2    = vt03_info.raw_data.fn_2;
    vt03_info.parsed.rc.trigger = vt03_info.raw_data.trigger;

    // 鼠标：说明书指出已是带符号的 int16_t，直接赋值即可
    vt03_info.parsed.mouse.x = vt03_info.raw_data.mouse_x;
    vt03_info.parsed.mouse.y = vt03_info.raw_data.mouse_y;
    vt03_info.parsed.mouse.z = vt03_info.raw_data.mouse_z;
    vt03_info.parsed.mouse.press_l = vt03_info.raw_data.mouse_left;
    vt03_info.parsed.mouse.press_r = vt03_info.raw_data.mouse_right;
    vt03_info.parsed.mouse.press_m = vt03_info.raw_data.mouse_middle;

    // 键盘：直接对 Union 赋值
    vt03_info.parsed.key.val = vt03_info.raw_data.key;

    // 6. 重置状态机 (只要能顺利走到这一步，说明收到了一帧健康的数据)
    vt03_info.online_cnt = VT03_OFFLINE_LIMIT;
    vt03_info.is_lost = false;
}

/**
 * @brief  监控遥控器在线状态 
 * @note   必须被 1ms 定时器 (如 SysTick 或 FreeRTOS 任务) 周期性循环调用
 */
void VT03_Monitor(void)
{
    if (vt03_info.online_cnt > 0)
    {
        vt03_info.online_cnt--; // 每次调用递减 1
    }
    else
    {
        // 计数器归零，说明超时未收到新数据，判定为离线
        vt03_info.is_lost = true;
        
        // 【极其重要】为了安全，离线时必须强制清空控制量，防止机器人暴走
        memset(&vt03_info.parsed, 0, sizeof(VT03_Ctrl_t)); 
    }
}

