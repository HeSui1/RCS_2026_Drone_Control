#ifndef GIMBAL_H
#define GIMBAL_H


#include "dm_motor.h"

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();


extern DM_Motor_Info_Typedef *pitch_dm_motor;

#endif // GIMBAL_H
