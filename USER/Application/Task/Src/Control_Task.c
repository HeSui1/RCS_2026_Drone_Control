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
#include "VT03.h"
//#include "PID.h"
//#include "Motor.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bsp_dwt.h"
#include "INS_Task.h"
#include "robot_def.h"
#include "MiniPC.h"

#include "ws2812b.h"

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

static bool gimbal_enabled = false; // 云台使能软开关，初始为 false (失能)
static uint8_t last_fn_1_state = 0; // 用于检测 fn_1 是否产生按下动作 (边沿检测)

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

		// 初始化 WS2812B
    WS2812B_Init();
		WS2812B_SetBrightness(10);//亮度更改入口
    // 开机默认绿色，提示初始化成功
    WS2812B_SetPixelColor(0, 0, 255, 0); 
    WS2812B_Show();
	
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
 * @brief VT03 专属手动控制逻辑 (纯遥控器操作)
 */
static void RemoteControlSet()
{
    // ========================================================
    // 1. 软开关逻辑 (fn_1 按键边沿检测)
    // ========================================================
    // 只有在 fn_1 从 0 变成 1 的瞬间，才切换云台的使能状态
    if (vt03_info.parsed.rc.fn_1 == 1 && last_fn_1_state == 0)
    {
        gimbal_enabled = !gimbal_enabled; 
    }
    last_fn_1_state = vt03_info.parsed.rc.fn_1; // 更新历史状态

    // ========================================================
    // 2. 云台控制 (受软开关管辖)
    // ========================================================
    if (gimbal_enabled)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
				shoot_cmd_send.shoot_mode = SHOOT_ON;
        // 摇杆增量叠加 (系数 0.0002f 视底盘电机的实际灵敏度可调)
        // ch_3 控制 Yaw：负数向左，正数向右。(注：如果发现向左推摇杆云台却向右转，把这里的 += 改成 -= 即可)
        gimbal_cmd_send.yaw -= 0.0002f * (float)vt03_info.parsed.rc.ch_3; 
        
        // ch_1 控制 Pitch：正数向上，负数向下。
        gimbal_cmd_send.pitch += 0.0002f * (float)vt03_info.parsed.rc.ch_1;

        // Pitch 轴软件限位 (必须保留，防止扯断线)
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
    else
    {
        // 软开关为 false 时，云台绝对脱力
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
				shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    // ========================================================
    // 3. 摩擦轮控制 (mode_sw)
    // ========================================================
    if (vt03_info.parsed.rc.mode_sw == 2)
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }

    // ========================================================
    // 4. 拨弹与退弹控制 (trigger & wheel)
    // ========================================================
    if (vt03_info.parsed.rc.trigger == 1) // 扳机按住：连发
    {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 25; // 设定一个默认满速射频
    }
    else if (vt03_info.parsed.rc.wheel < -50) // 拨轮向下推过死区：退弹
    {
        shoot_cmd_send.load_mode = LOAD_REVERSE;
        // 线性映射：从 -50 到 -660 映射到 0~25 的速度等级
        float rate = (float)(-vt03_info.parsed.rc.wheel - 50) / 610.0f * 25.0f;
        shoot_cmd_send.shoot_rate = (uint8_t)rate;
    }
    else // 无操作时停止拨弹
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0;
    }
}

/**
 * @brief VT03 专属键鼠控制逻辑 (图传链路)
 */
static void MouseKeySet()
{
    // ========================================================
    // 1. 软开关逻辑 (复用 fn_1 使能云台)
    // ========================================================
    if (vt03_info.parsed.rc.fn_1 == 1 && last_fn_1_state == 0) {
        gimbal_enabled = !gimbal_enabled; 
    }
    last_fn_1_state = vt03_info.parsed.rc.fn_1;

    // 如果云台未使能，强制失能并退出计算
    if (!gimbal_enabled) {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
				shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        return; 
    }

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
		shoot_cmd_send.shoot_mode = SHOOT_ON;
    // ========================================================
    // 2. 鼠标控制云台 (鼠标位移增量直接叠加到目标角度)
    // ========================================================
    // 这里的 0.005f 是鼠标灵敏度，你需要根据上车后的实际手感去放大或缩小
    gimbal_cmd_send.yaw -= 0.005f * (float)vt03_info.parsed.mouse.x; 
    gimbal_cmd_send.pitch += 0.005f * (float)vt03_info.parsed.mouse.y;

    // Pitch 轴软件限位
    float pitch_max_limit = 11.f;
    float pitch_min_limit = -18.f;
    if (gimbal_cmd_send.pitch > pitch_max_limit) gimbal_cmd_send.pitch = pitch_max_limit;
    else if (gimbal_cmd_send.pitch < pitch_min_limit) gimbal_cmd_send.pitch = pitch_min_limit;


    // ========================================================
    // 3. 键盘按键边沿检测与状态切换
    // ========================================================
    static VT03_Key_t last_key = {0}; // 静态变量，记录上一帧的键盘状态
    VT03_Key_t cur_key = vt03_info.parsed.key; // 获取当前键盘状态

    // --- Z 键：切换弹速 (15 -> 18 -> 30) ---
    static uint8_t speed_state = 0;
    if (cur_key.z && !last_key.z) { // 检测到 Z 键刚被按下
        speed_state = (speed_state + 1) % 3;
    }
    if (speed_state == 0) shoot_cmd_send.bullet_speed = 15;
    else if (speed_state == 1) shoot_cmd_send.bullet_speed = 18;
    else shoot_cmd_send.bullet_speed = 30;

    // --- E 键：切换发射模式 (停止 -> 单发 -> 3连发 -> 持续连发) ---
    static uint8_t fire_mode = 0;
    if (cur_key.e && !last_key.e) {
        fire_mode = (fire_mode + 1) % 4;
    }
    if (fire_mode == 0) shoot_cmd_send.load_mode = LOAD_STOP;
    else if (fire_mode == 1) shoot_cmd_send.load_mode = LOAD_1_BULLET;
    else if (fire_mode == 2) shoot_cmd_send.load_mode = LOAD_3_BULLET;
    else shoot_cmd_send.load_mode = LOAD_BURSTFIRE;

    // --- R 键：开关弹舱 ---
    static uint8_t lid_state = 0;
    if (cur_key.r && !last_key.r) {
        lid_state = !lid_state;
    }
    shoot_cmd_send.lid_mode = lid_state ? LID_OPEN : LID_CLOSE;

    // --- F 键：开关摩擦轮 ---
    static uint8_t fric_state = 0;
    if (cur_key.f && !last_key.f) {
        fric_state = !fric_state;
    }
    shoot_cmd_send.friction_mode = fric_state ? FRICTION_ON : FRICTION_OFF;

// ========================================================
    // 3. 键盘按键边沿检测与状态切换 (WASD 飞手沟通灯语)
    // ========================================================

    // 计算灯带的中点
    uint16_t mid_led = WS2812B_NUM_LEDS / 2;

    // --- W 键：向上飞 (全亮青色/Cyan) ---
    if (cur_key.w && !last_key.w) { 
        for(uint16_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            WS2812B_SetPixelColor(i, 0, 255, 255); // 青色 (G=255, B=255)
        }
        WS2812B_Show();
    }

    // --- S 键：向下飞 (全亮橙黄色/Orange) ---
    if (cur_key.s && !last_key.s) {
        for(uint16_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            WS2812B_SetPixelColor(i, 255, 128, 0); // 橙黄色 (R=255, G=128, B=0)
        }
        WS2812B_Show();
    }

    // --- A 键：向左飞 (左半边亮紫色，右半边灭) ---
    if (cur_key.a && !last_key.a) {
        // 左半边亮紫色 (R=255, B=255)
        for(uint16_t i = 0; i < mid_led; i++) {
            WS2812B_SetPixelColor(i, 255, 0, 255); 
        }
        // 右半边熄灭 (制造方向感)
        for(uint16_t i = mid_led; i < WS2812B_NUM_LEDS; i++) {
            WS2812B_SetPixelColor(i, 0, 0, 0); 
        }
        WS2812B_Show();
    }

    // --- D 键：向右飞 (右半边亮绿色，左半边灭) ---
    if (cur_key.d && !last_key.d) {
        // 左半边熄灭
        for(uint16_t i = 0; i < mid_led; i++) {
            WS2812B_SetPixelColor(i, 0, 0, 0); 
        }
        // 右半边亮绿色 (G=255)
        for(uint16_t i = mid_led; i < WS2812B_NUM_LEDS; i++) {
            WS2812B_SetPixelColor(i, 0, 255, 0); 
        }
        WS2812B_Show();
    }

    // --- (可选) 松开所有按键时恢复待机颜色 (比如微弱的白光) ---
    // 如果你想让提示只在按住时有效，松开就恢复，可以加一段按键释放检测
    if (!cur_key.w && last_key.w || !cur_key.s && last_key.s || 
        !cur_key.a && last_key.a || !cur_key.d && last_key.d) {
        
        // 检查是不是全都松开了
        if(!cur_key.w && !cur_key.s && !cur_key.a && !cur_key.d) {
            for(uint16_t i = 0; i < WS2812B_NUM_LEDS; i++) {
                WS2812B_SetPixelColor(i, 10, 10, 10); // 微弱的白光代表待机
            }
            WS2812B_Show();
        }
    }

    last_key = cur_key;

		
		
    // ========================================================
    // 4. 鼠标左键发射逻辑
    // ========================================================
    // 如果按住了鼠标左键，强行覆盖 E 键的模式，开启最高射速连发
    if (vt03_info.parsed.mouse.press_l) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 25; 
    } 
    // 如果没按左键，但 E 键切到了持续连发模式，也维持连发
    else if (fire_mode == 3) {
        shoot_cmd_send.shoot_rate = 25;
    } 
    else {
        shoot_cmd_send.shoot_rate = 0; // 单发或三连发由底层任务去控制拨弹电机步数
    }

    // 注意：底盘 WASD 的运动控制还没写，留到之后单独处理底盘时再补充！
}


/**
 * @brief  紧急停止与状态机守护 (VT03版)
 */
static void EmergencyHandler()
{
    // ========================================================
    // 1. 最高优先级：致命错误检测 (VT03 掉线保护)
    // ========================================================
    if (vt03_info.is_lost == true) 
    {
        robot_state = ROBOT_STOP;   // 强制进入死锁状态
        gimbal_enabled = false;     // 出于安全考虑，掉线后必须强制重置软开关为失能！
    }
    // ========================================================
    // 2. 解锁机制：重新连上后恢复到就绪状态
    // ========================================================
    else if (robot_state == ROBOT_STOP && vt03_info.is_lost == false)
    {
        robot_state = ROBOT_READY;  
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        // 注意：恢复 READY 不代表云台马上有劲，必须等操作手再次按下 fn_1 才能使能
    }

    // ========================================================
    // 3. 死锁状态下的输出强制清零
    // ========================================================
    if (robot_state == ROBOT_STOP)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0;
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

// mode_sw 为 1 或 2 时，使用纯遥控器控制 (2为开摩擦轮状态)
        if (vt03_info.parsed.rc.mode_sw == 1 || vt03_info.parsed.rc.mode_sw == 2)
        {
            RemoteControlSet();
        }
        // mode_sw 为 0 时，使用键鼠图传链路控制
        else if (vt03_info.parsed.rc.mode_sw == 0)
        {
            MouseKeySet();
        }
		
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


