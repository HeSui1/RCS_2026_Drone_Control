#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "bsp_uart.h"

#include "ramp.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

Publisher_t *shoot_pub;
Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
Subscriber_t *shoot_sub;
Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static uint8_t last_load_mode = LOAD_STOP;    // 记录上一次的拨弹模式，用于检测边沿
static float loader_target_angle = 0;         // 记录拨盘的绝对目标角度
		
void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
							  .DeadBand = 30,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
							  .Improve = PID_Integral_Limit,
                .IntegralLimit = 100,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 1, // 10
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }


    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:			// 激活能量机关/干扰对方用
				DJIMotorOuterLoop(loader, ANGLE_LOOP); 
        if (last_load_mode != LOAD_1_BULLET) 
        {
            // 基于电机当前的实际角度，往前减一发子弹的角度
            loader_target_angle = loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE;
            hibernate_time = DWT_GetTimeline_ms(); 
            dead_time = 150;                       
        }                                                                     
        DJIMotorSetRef(loader, loader_target_angle);
				break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
				DJIMotorSetRef(loader, -shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题

				
    }

// ==========================================================
    // 摩擦轮斜坡控制逻辑开始
    // ==========================================================
    static float friction_l_current = 0; // 静态变量：保存左摩擦轮当前实际下发的转速
    static float friction_r_current = 0; // 静态变量：保存右摩擦轮当前实际下发的转速
    
    float target_speed = 0; // 临时变量：当前期望的目标速度

    // 1. 根据指令确定目标速度 target_speed
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            target_speed = 5000;
            break;
        case SMALL_AMU_18:
            target_speed = 6000;
            break;
        case SMALL_AMU_30:
            target_speed = 8000;
            break;
        default: 
            target_speed = 40000; // 默认值
            break;
        }
    }
    else 
    {
        target_speed = 0; // 关闭摩擦轮时，目标速度设为0
    }

    // 2. 调用 ramp 函数进行平滑过渡
    // 这里的 15.0f 是步长。假设从 7000 降到 0，每 5ms 降 15，大约需要 2.3 秒停稳。
    // 如果你觉得刹车太慢，可以把 15.0f 改成 20.0f 或 30.0f。
    friction_l_current = f_Ramp_Calc(friction_l_current, target_speed, 4.0f);
    friction_r_current = f_Ramp_Calc(friction_r_current, target_speed, 4.0f);

    // 3. 将平滑后的当前速度下发给底层 PID
    DJIMotorSetRef(friction_l, friction_l_current);
    DJIMotorSetRef(friction_r, friction_r_current);

    // ==========================================================
    // 摩擦轮斜坡控制逻辑结束
    // ==========================================================

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
//		USART_Vofa_Justfloat_Transmit(friction_l->measure.speed_aps,friction_l->motor_controller.pid_ref,(float)((int16_t)(friction_l->motor_can_instance->rx_buff[2] << 8 | friction_l->motor_can_instance->rx_buff[3]))*RPM_2_ANGLE_PER_SEC);		

}