#include "ws2812b.h"
#include "tim.h"  // 依赖底层的定时器接口

/* 
 * 硬件参数定义 (基于 160MHz 时钟, ARR=199) 
 */
#define WS2812_0_PULSE 64   // 逻辑 0 的占空比 (~32%)
#define WS2812_1_PULSE 128  // 逻辑 1 的占空比 (~64%)
#define WS2812_RESET_PULSES 300 // 复位信号需要的低电平周期数 (>50us)

/* 
 * 驱动内部缓存区 
 */
// 1. 用户颜色缓存区：保存每颗灯的 RGB 颜色
uint8_t rgb_buffer[WS2812B_NUM_LEDS][3];
static uint8_t global_brightness = 255;//定义初始亮度

// 2. DMA 发送缓存区：计算公式 -> 灯珠数 * 24位 + 复位周期
#define PWM_BUFFER_SIZE (WS2812B_NUM_LEDS * 24 + WS2812_RESET_PULSES)
__attribute__((section(".ARM.__at_0x24000000"))) uint16_t pwm_buffer[PWM_BUFFER_SIZE];

/**
  * @brief  WS2812B 驱动初始化
  */
void WS2812B_Init(void)
{
    // 这里严格来说属于 BSP 层的动作，为了方便暂时放在驱动初始化中。
    // 如果你以后有 bsp_power.c，可以将这句移过去。
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); 
    HAL_Delay(10); // 等待 5V 电源稳定
    
    // 初始化时把所有灯都设为关闭（黑色）
    for(int i = 0; i < WS2812B_NUM_LEDS; i++)
    {
        WS2812B_SetPixelColor(i, 0, 0, 0);
    }
    WS2812B_Show();
}

/**
  * @brief  设置指定灯珠的颜色 (并不会立即生效，需调用 Show)
  * @param  index: 灯珠的序号 (从 0 开始)
  * @param  r: 红色亮度 (0-255)
  * @param  g: 绿色亮度 (0-255)
  * @param  b: 蓝色亮度 (0-255)
  */
void WS2812B_SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < WS2812B_NUM_LEDS)
    {
        // WS2812B 的内部颜色顺序通常是 G, R, B
        rgb_buffer[index][0] = g;
        rgb_buffer[index][1] = r;
        rgb_buffer[index][2] = b;
    }
}

/**
  * @brief  将缓存区的颜色数据推送到硬件上显示
  */
void WS2812B_SetBrightness(uint8_t brightness)
{
    global_brightness = brightness;
}

// 3. 修改 WS2812B_Show 函数内部的颜色换算循环
void WS2812B_Show(void)
{
    uint32_t ptr = 0;

    for (int i = 0; i < WS2812B_NUM_LEDS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // 新代码: 将原始颜色按百分比缩放 (最大 65025/255 = 255)
            uint8_t color_byte = (rgb_buffer[i][j] * global_brightness) / 255; 
            
            for (int bit = 7; bit >= 0; bit--)
            {
                if (color_byte & (1 << bit))
                {
                    pwm_buffer[ptr++] = WS2812_1_PULSE;
                }
                else
                {
                    pwm_buffer[ptr++] = WS2812_0_PULSE;
                }
            }
        }
    }

    // 下面的填 0 复位信号和 DMA 发送代码
    for (int i = 0; i < WS2812_RESET_PULSES; i++) {
        pwm_buffer[ptr++] = 0;
    }
    SCB_CleanDCache_by_Addr((uint32_t *)pwm_buffer, sizeof(pwm_buffer));
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_buffer, PWM_BUFFER_SIZE);
}