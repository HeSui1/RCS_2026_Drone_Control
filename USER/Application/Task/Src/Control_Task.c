/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Remote_Control_COD.h"
//#include "PID.h"
//#include "Motor.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bsp_dwt.h"
#include "INS_Task.h"
#include "robot_def.h"
#include "MiniPC.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

//static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
//static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等


//static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
 Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{


//    vision_recv_data = VisionInit(&huart1); // 视觉通信串口
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}


/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
     angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    //    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    //    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
    //    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE);
        // chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f);
        // chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else;
        // chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

// 记录上一次的云台模式，用于判断模式切换的瞬间
static uint8_t last_gimbal_mode = GIMBAL_ZERO_FORCE;

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet()
{
    // ========================================================
    // 1. 控制运行模式设定 (右侧开关)
    // ========================================================
    if (switch_is_down(rc_data.rc.switch_right)) //右侧开关[下]: 摩擦轮常开，云台受控
    {
				gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
    }
    else if (switch_is_mid(rc_data.rc.switch_right)) // 右侧开关[中]: 摩擦轮关闭，云台受控
    {
				gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; 
        shoot_cmd_send.friction_mode = FRICTION_OFF;
			
    }
		else if (switch_is_up(rc_data.rc.switch_right)) // 右侧开关[上]: 安全待机状态 (云台掉电，摩擦轮关)
    {
       gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
			 shoot_cmd_send.friction_mode = FRICTION_OFF;			
    }


    if (switch_is_mid(rc_data.rc.switch_left)) // 左侧开关状态[中], 视觉模式
    {
// 1. 强制云台进入陀螺仪模式 (视觉通常需要世界坐标系下的绝对角度)
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        gimbal_cmd_send.yaw = vision_rx_data.yaw*57.29578;
        gimbal_cmd_send.pitch = vision_rx_data.pitch*57.29578;
        
        float pitch_max_limit = 17.f;  // 约仰角 17 度
        float pitch_min_limit = -17.f; // 约俯角 17 度
        if (gimbal_cmd_send.pitch > pitch_max_limit) 
        {
            gimbal_cmd_send.pitch = pitch_max_limit;
        }
        else if (gimbal_cmd_send.pitch < pitch_min_limit) 
        {
            gimbal_cmd_send.pitch = pitch_min_limit;
        }

        // 上位机 mode: 0: 不控制, 1: 控制云台不开火, 2: 控制云台且开火
        if (vision_rx_data.mode == 2) 
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE; // 触发连发拨弹
            shoot_cmd_send.friction_mode = FRICTION_OFF;// 确保摩擦轮开启
        }
        else 
        {
            shoot_cmd_send.load_mode = LOAD_STOP;      // 停止拨弹
        }
    }
    else if (switch_is_down(rc_data.rc.switch_left)) // 左侧开关状态[下], 纯遥控器手动控制
    { 
        // 摇杆增量叠加（在刚才对齐的基础上缓慢增加目标角度）
        gimbal_cmd_send.yaw += 0.0002f * (float)rc_data.rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.0002f * (float)rc_data.rc.rocker_l1;
        
        // Pitch 轴软件限位 (极其重要，防止摇杆一直推把云台线扯断)
        float pitch_max_limit = 11.f; 
        float pitch_min_limit = -18.f; 
        
        if (gimbal_cmd_send.pitch > pitch_max_limit) 
        {
            gimbal_cmd_send.pitch = pitch_max_limit;
        }
        else if (gimbal_cmd_send.pitch < pitch_min_limit) 
        {
            gimbal_cmd_send.pitch = pitch_min_limit;
        }
    }


//    // 摩擦轮控制：左上侧拨轮向上打(低于-100)打开摩擦轮
//    if (rc_data.rc.dial < -100) 
//        shoot_cmd_send.friction_mode = FRICTION_ON;
//    else
//        shoot_cmd_send.friction_mode = FRICTION_OFF;
//        
//    // 拨弹控制：左上侧拨轮向上打满(低于-500)开启连发
//    if (rc_data.rc.dial < -500)
//        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
//    else
//        shoot_cmd_send.load_mode = LOAD_STOP;
//        
//    // 射频控制：固定每秒 x 发
//    shoot_cmd_send.shoot_rate = 25;
// ========================================================
    // 3. 拨弹电机控制 (右侧摇杆，垂直r1控制发射，水平r0控制退弹)
    // ========================================================
    static bool single_shoot_lock = false; // 单发软开关的边沿检测标志位

    // 优先判断水平方向（退弹）
    if (rc_data.rc.rocker_r_ < -50) // 右摇杆向左推 (退弹/反转)
    {
        single_shoot_lock = false; // 解锁单发
        shoot_cmd_send.load_mode = LOAD_REVERSE;
        
        // 线性映射：摇杆死区 -50 到 极值 -660 映射到 0~25 的抽象速度等级
        float rate = (float)(-rc_data.rc.rocker_r_ - 50) / 610.0f * 25.0f;
        shoot_cmd_send.shoot_rate = (uint8_t)rate; 
    }
    // 再判断垂直方向（发射）
//    else if (rc_data.rc.rocker_r1 < -300) // 右摇杆向下打过一半 (触发单发)
//    {
////        if (!single_shoot_lock)
////        {
////            shoot_cmd_send.load_mode = LOAD_1_BULLET;
////            single_shoot_lock = true; // 上锁，必须摇杆回中才能解锁下一次单发
////        }
////        else
////        {
////            shoot_cmd_send.load_mode = LOAD_1_BULLET; // 维持单发指令模式
////        }
//    }
    else if (rc_data.rc.rocker_r1 > 50) // 右摇杆向上推 (线性连发)
    {
        single_shoot_lock = false; 
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;

        // 线性映射射频 (0~25 发/秒)
        float rate = (float)(rc_data.rc.rocker_r1 - 50) / 610.0f * 25.0f;
        shoot_cmd_send.shoot_rate = (uint8_t)rate; 
    }
    else // 摇杆回中 (-50 ~ 50 死区)
    {
        single_shoot_lock = false; 
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{

    // gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    // gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data.key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
         shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
         shoot_cmd_send.bullet_speed = 18;
        break;
    default:
         shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data.key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
         shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
         shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
         shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
         shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data.key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
         shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
         shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data.key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
         shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
         shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data.key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        // chassis_cmd_send.chassis_speed_buff = 40;
        // break;
    case 1:
        // chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        // chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        // chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data.key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief  紧急停止与状态机守护
 */
static void EmergencyHandler()
{
	static bool require_switch_cycle = false;
    // ========================================================
    // 1. 最高优先级：致命错误检测 (掉线、急停拨轮打满)
    // ========================================================
    if (remote_ctrl.rc_lost == true || rc_data.rc.dial > 300) 
    {
        robot_state = ROBOT_STOP; // 强制进入死锁状态
    }
    // ========================================================
    // 2. 解锁机制：硬件安全的前提下，右开关打到【上】才能解锁
    // ========================================================
if (robot_state == ROBOT_STOP && remote_ctrl.rc_lost == false && rc_data.rc.dial <= 300)
    {
        // 如果用户把开关拨到了【中】或【下】，说明操作手有意识地动了开关，解除死锁标志
        if (!switch_is_up(rc_data.rc.switch_right)) {
            require_switch_cycle = false; 
        }
        
        // 只有在死锁标志解除，且开关被切实打到【上】时，才真正解锁
        if (require_switch_cycle == false && switch_is_up(rc_data.rc.switch_right)) {
            robot_state = ROBOT_READY; 
					  shoot_cmd_send.shoot_mode = SHOOT_ON;
        }
    }

    if (robot_state == ROBOT_STOP)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        
        shoot_cmd_send.shoot_mode = SHOOT_OFF;

    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
		RobotCMDInit();
  uint32_t PreviousWakeTime = osKernelSysTick();
   const uint32_t TaskPeriod = 5; // 200Hz = 5ms 周期

	for(;;)
  {

	    // 从其他应用获取回传数据
#ifdef ONE_BOARD
//	    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
	    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
	     SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
	     SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

	    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
	    CalcOffsetAngle();
	    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
	    if (switch_is_down(rc_data.rc.switch_left) || switch_is_mid(rc_data.rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
	        RemoteControlSet();
	    else if (switch_is_up(rc_data.rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
	        MouseKeySet();

	    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况


	    // 推送消息,双板通信,视觉通信等
	    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
	    // PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
	    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
	     PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
	     PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);


	     osDelayUntil(&PreviousWakeTime, TaskPeriod);
  }
}


