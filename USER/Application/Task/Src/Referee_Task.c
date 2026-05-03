#include "Referee_Task.h"
#include "Referee_System.h"
#include "referee_UI.h"
#include "cmsis_os.h"

uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用

/* === 引入你在 Referee_System_Info.c 中定义的全局变量 === */
extern Referee_System_Info_TypeDef Referee_System_Info;

Referee_Interactive_info_t Robot_UI_State;

/* 本地指针，用于方便地操作 UI 和裁判系统数据 */
static Referee_Interactive_info_t *Interactive_data;
static Referee_System_Info_TypeDef *referee_recv_info;

/* UI 图层相关的结构体实例 */

/* 只保留我们需要画的三个圈 */
static Graph_Data_t UI_aim_circles[3];

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态, 静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态, 动态先add才能change

static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

/* 内部函数声明 */
static void DeterminRobotID(void);
static void MyUIRefresh(Referee_System_Info_TypeDef *ref_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data);

/**
 * @brief 初始化裁判系统交互任务
 */
void UITaskInit(Referee_Interactive_info_t *UI_data)
{
    // 将操作指针指向全局的裁判系统数据结构体
    referee_recv_info = &Referee_System_Info;
    // 获取 UI 绘制需要的机器人状态数据 (如底盘模式、云台模式)
    Interactive_data = UI_data;
    // 标记初始化完成
    referee_recv_info->init_flag = 1;
}

/**
 * @brief 判断各种ID，计算出需要发送到的客户端(选手端)ID
 */
static void DeterminRobotID(void)
{
		referee_recv_info->robot_status.robot_id = 6;
    // id 小于 7 是红色, 大于 7 是蓝色, 0 为红色，1 为蓝色
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->robot_status.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->robot_status.robot_id;
    // 客户端 ID = 0x0100 + 本机 ID
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

/**
 * @brief UI 静态初始化，在 RTOS 任务死循环前调用一次
 */
void MyUIInit(void)
{
    if (!referee_recv_info->init_flag)
    {
        vTaskDelete(NULL); 
    }
    
    // 测试用强制赋值
    referee_recv_info->robot_status.robot_id = 6;
    while (referee_recv_info->robot_status.robot_id == 0)
    {
        osDelay(100);
    }
    DeterminRobotID(); 

    // 1. 清空残留
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
    osDelay(200); 
    
    // 2. 准备图形（1 个圆）
    UICircleDraw(&UI_aim_circles[0], "c1", UI_Graph_ADD, 1, UI_Color_Main, 3, 960, 540, 50);
    
    // 3. 发送（就这么简单直接！）
    UIGraphRefresh(&referee_recv_info->referee_id, &UI_aim_circles[0]);
    
    // 防丢包，重发一次
    osDelay(150);
    UIGraphRefresh(&referee_recv_info->referee_id, &UI_aim_circles[0]);
}

/**
 * @brief UI 动态刷新核心函数
 */
static void MyUIRefresh(Referee_System_Info_TypeDef *ref_info, Referee_Interactive_info_t *_Interactive_data)
{
//    // 1. 检查状态枚举是否发生变化，并更新脏标记 (flag)
//    UIChangeCheck(_Interactive_data);

//    // 2. 根据 flag 的情况，针对性地推送更改指令 (UI_Graph_Change)
//    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
//    {
//        switch (_Interactive_data->chassis_mode)
//        {
//        case CHASSIS_ZERO_FORCE:
//            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
//            break;
//        case CHASSIS_ROTATE:
//            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
//            break;
//        case CHASSIS_NO_FOLLOW:
//            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "nofollow ");
//            break;
//        case CHASSIS_FOLLOW_GIMBAL_YAW:
//            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
//            break;
//        }
//        UICharRefresh(&ref_info->referee_id, UI_State_dyn[0]);
//        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0; // 清除标记
//    }

//    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
//    {
//        switch (_Interactive_data->gimbal_mode)
//        {
//        case GIMBAL_ZERO_FORCE:
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
//            break;
//        case GIMBAL_FREE_MODE:
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
//            break;
//        case GIMBAL_GYRO_MODE:
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
//            break;
//        }
//        UICharRefresh(&ref_info->referee_id, UI_State_dyn[1]);
//        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
//    }

//    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
//    {
//        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
//        UICharRefresh(&ref_info->referee_id, UI_State_dyn[2]);
//        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
//    }

//    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
//    {
//        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
//        UICharRefresh(&ref_info->referee_id, UI_State_dyn[3]);
//        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
//    }

//    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
//    {
//        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
//        UICharRefresh(&ref_info->referee_id, UI_State_dyn[4]);
//        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
//    }

//    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
//    {
//        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
//        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
//        UIGraphRefresh(&ref_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
//        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
//    }
}

/**
 * @brief  模式切换检测：比对 current 和 last，产生更新 Flag
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }
    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }
    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }
    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }
    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }
    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }
}

/**
 * @brief  测试用函数，实现模式自动变化，用于在不上遥控器时检查裁判系统链路
 */
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data)
{
    static uint8_t count = 0;
    static uint16_t count1 = 0;
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    case 1:
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    case 2:
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    case 3:
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    default:
        break;
    }
}

/**
 * @brief 供 FreeRTOS 任务内部调用的单次更新逻辑
 */
void UITask(void)
{
    // 如果你在遥控器控制逻辑中已经能正常修改 _Interactive_data 的值，
    // 可以把 RobotModeTest 注释掉。初期调试链路时建议保留。
//    RobotModeTest(Interactive_data);

//    MyUIRefresh(referee_recv_info, Interactive_data);
}

/**
 * @brief FreeRTOS 任务主函数
 */
void Referee_Task(void const * argument)
{
    /* USER CODE BEGIN Referee_Task */

//    // 假设在任务启动前，或者这里，你声明了一个 Referee_Interactive_info_t 变量
//    // 你需要确保它在生命周期内有效，通常建议把它作为一个全局变量。
////    // 为了防止空指针，这里必须先初始化它。
//			UITaskInit(&Robot_UI_State);
//	
	// 如果你是离线测试（没连主裁判系统），必须保留这句强行声明身份
    referee_recv_info->robot_status.robot_id = 6;
    DeterminRobotID(); 

    // 先发一次清除指令，扫清战场
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
    osDelay(500);
	

//    // 执行一次静态 UI 绘制和清除
			MyUIInit();

//    /* Infinite loop */
    for(;;)
    {
//				UITask(); // 包含动态刷新逻辑
				UICircleDraw(&UI_aim_circles[0], "c1", UI_Graph_ADD, 1, UI_Color_Green, 5, 960, 540, 50);
        
        // 疯狂发送
        UIGraphRefresh(&referee_recv_info->referee_id, &UI_aim_circles[0]);
        
        // 每隔 1 秒发一次
        osDelay(1000);			
        osDelay(1); // 防止死循环，且在底层的 RefereeSend 中还有 115ms 延时
    }
//    /* USER CODE END Referee_Task */
}
		
		

