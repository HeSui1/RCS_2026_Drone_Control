#include "bsp_mcu.h"
#include "Robot_Task.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
//#include "robot_task.h"



// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
//#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
//#include "robot_cmd.h"
#endif


void RobotInit()
{


#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

        GimbalInit();
        ShootInit();

#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    //    ChassisInit();
#endif

}

void Robot_Task(void const * argument)
{

    for(;;)
    {


#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

            GimbalTask();
            ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
        //    ChassisTask();
#endif


        osDelay(1);
    }

}
