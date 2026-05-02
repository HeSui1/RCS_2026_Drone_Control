#ifndef __WS2812B_H
#define __WS2812B_H

#include "main.h"

/* 定义灯条上灯珠的数量，以后如果你接了 10 颗灯，把这里改成 10 即可 */
#define WS2812B_NUM_LEDS  2 
// 亮度范围从 0 (全灭) 到 255 (最亮)
void WS2812B_SetBrightness(uint8_t brightness);

/* 驱动接口声明 */
void WS2812B_Init(void);
void WS2812B_SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_Show(void);

#endif /* __WS2812B_H */