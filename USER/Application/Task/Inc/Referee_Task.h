//
// Created by 29075 on 2026/5/2.
//

#ifndef COD_H7_TEMPLATE_REFEREE_H
#define COD_H7_TEMPLATE_REFEREE_H

#include "Referee_System.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
void UITaskInit(Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用，用于绘制静态UI
 */
void MyUIInit(void);

/**
 * @brief 裁判系统交互任务(UI刷新和多机通信)，需在 FreeRTOS 任务的 while(1) 中调用
 */
void UITask(void);



#endif //COD_H7_TEMPLATE_REFEREE_H在·