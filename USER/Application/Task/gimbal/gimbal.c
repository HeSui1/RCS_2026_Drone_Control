#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "INS_Task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_uart.h"
#include "Config.h"
#include "Minipc.h"
attitude_t *gimba_IMU_data; // 云台IMU数据
DJIMotorInstance *yaw_motor;

DM_Motor_Info_Typedef *pitch_dm_motor;

Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
float pitch_target_rad =0;

extern GimbalToVision_t gimbal_tx_data;

void Update_Gimbal_To_Vision_Packet(void) {
// 1. 搬运四元数 (从 IMU 指针获取)
    // 坐标系轴向和正负号的问题，等实际上机调试视觉时再做修改
    gimbal_tx_data.q[0] = gimba_IMU_data->q[0]; // w
    gimbal_tx_data.q[1] = gimba_IMU_data->q[1]; // x
    gimbal_tx_data.q[2] = gimba_IMU_data->q[2]; // y
    gimbal_tx_data.q[3] = gimba_IMU_data->q[3]; // z

    // 2. 搬运欧拉角 (视觉除了四元数，通常也需要直接读取当前欧拉角做绝对位置参考)
    gimbal_tx_data.yaw   = gimba_IMU_data->Yaw;
    gimbal_tx_data.pitch = gimba_IMU_data->Pitch;

    // 3. 搬运角速度 (视觉做卡尔曼滤波预测非常依赖实时的角速度)
    // 注意：目前 INS_Output 里是 deg/s，如果视觉组发现预测乱飘，可能需要你在这里乘以 0.0174533f 转成 rad/s
    gimbal_tx_data.yaw_vel   = gimba_IMU_data->Gyro[IMU_GYRO_INDEX_YAW];
    gimbal_tx_data.pitch_vel = gimba_IMU_data->Gyro[IMU_GYRO_INDEX_PITCH];

    // 4. 模式与弹道数据
    // 【注意】: 视觉需要的 mode 是 (0: 空闲, 1: 自瞄, 2: 小符, 3: 大符)
    // 而你代码里的 gimbal_cmd_recv.gimbal_mode 是 (GIMBAL_ZERO_FORCE, GIMBAL_GYRO_MODE 等)
    // 你需要确保 cmd 任务里有一个专门发给视觉的模式变量，这里暂时假设你后续会在 cmd 里增加 vision_mode
    gimbal_tx_data.mode = 1; // 临时写死为 1 (自瞄) 方便初期测试，后续换成类似 gimbal_cmd_recv.vision_mode

    // 子弹速度和射击计数（目前写死，后期需要接入串口裁判系统解析出来的数据和拨弹轮数据）
    gimbal_tx_data.bullet_speed = 15.0f; 
    gimbal_tx_data.bullet_count = 0;     

    // 5. 触发发送
    // 该函数内部会自动计算 CRC16 验证码并调用 USB CDC 接口发送到底层
    MiniPC_Transmit_Info();
}

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5, // 8
                .Ki = 0.2,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 150,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 20,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
							  .DeadBand = 0,
            },

            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
						.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

        },
        .motor_type = GM6020};

    yaw_motor = DJIMotorInit(&yaw_config);
				
		    // PITCH
//		DM_Motor_Init_Config_s dm_pitch_config = {
//						.motor_type = DM3507,
//						.control_mode = MIT,          // 推荐使用 MIT 模式进行调试
//						.param_limits = {
//								.P_MAX = 12.5f,           // 依据达妙上位机配置 (通常是 12.5 或 PI)
//								.V_MAX = 50.0f,           // 依据达妙上位机配置
//								.T_MAX = 3.0f,            // 3507的峰值力矩
//						},
//						.can_init_config = {
//								.can_handle = &hfdcan1,   

//								.tx_id = 0x01,            //CAN ID 
//								.rx_id = 0x01,            //Master ID 
//						}
//				};
		DM_Motor_Init_Config_s dm_pitch_config = {
						.motor_type = DM3507,
						.control_mode = POSITION_VELOCITY,         
						.can_init_config = {
								.can_handle = &hfdcan1,   

								.tx_id = 0x02,            //CAN ID 
								.rx_id = 0x02,            //Master ID 
						}
//				};	
//		DM_Motor_Init_Config_s dm_pitch_config = {
//						.motor_type = DM4310,
//						.control_mode = POSITION_VELOCITY,         
//						.can_init_config = {
//								.can_handle = &hfdcan1,   

//								.tx_id = 0x01,            //CAN ID 
//								.rx_id = 0x01,            //Master ID 
//						}
				};						
				// 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动		
			pitch_dm_motor = DM_Motor_Init(&dm_pitch_config);
				
			gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
			gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
		}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
		pitch_target_rad =pitch_dm_motor->Data.Position+(gimbal_cmd_recv.pitch * 0.0174533f-gimba_IMU_data->Pitch * 0.0174533f);// gimbal_cmd_recv.pitch * 0.0174533f  ;//-(gimba_IMU_data->Pitch * 0.0174533f-pitch_dm_motor->Data.Position);
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);

		    if (pitch_dm_motor->Data.State == 1) {
            DM_Motor_Command(pitch_dm_motor, Motor_Disable);
        }

        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);

        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈


		// 【防洪防竞争】：分层处理使能与控制
        if (pitch_dm_motor->Data.State != 1) {
            // 如果还没使能，只发使能指令，不发控制指令
            DM_Motor_Command(pitch_dm_motor, Motor_Enable);
        } else {
            // 只有在成功使能后，才下发位置控制帧！防止和使能帧在邮箱里打架

            DM_Motor_Set_PosVel_Target(pitch_dm_motor, pitch_target_rad, 0.5f);
        }
        break;
				
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
			
			  DJIMotorEnable(yaw_motor);	

						
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);


        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈

				
		// 【防洪防竞争】：分层处理使能与控制
        if (pitch_dm_motor->Data.State != 1) {
            // 如果还没使能，只发使能指令，不发控制指令
            DM_Motor_Command(pitch_dm_motor, Motor_Enable);
        } else {
            // 只有在成功使能后，才下发位置控制帧！防止和使能帧在邮箱里打架

            DM_Motor_Set_PosVel_Target(pitch_dm_motor, pitch_target_rad, 0.5f);
        }
		
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
//		USART_Vofa_Justfloat_Transmit(gimbal_cmd_recv.yaw,gimbal_cmd_recv.pitch,0);
//		USART_Vofa_Justfloat_Transmit(-gimba_IMU_data->Yaw,yaw_motor->motor_controller.speed_PID.Output,-yaw_motor->motor_controller.angle_PID.Err);		
//		USART_Vofa_Justfloat_Transmit(gimba_IMU_data->Yaw,gimba_IMU_data->Pitch,gimba_IMU_data->Gyro[IMU_GYRO_INDEX_YAW]);		
// 观测通道 1: 遥控器下发的目标角度 (度)
    // 观测通道 2: IMU 测量的真实物理角度 (度)
    // 观测通道 3: 达妙电机的底层编码器位置 (将其乘以 -57.29 转换回“世界坐标”的度数)
    USART_Vofa_Justfloat_Transmit(-gimbal_cmd_recv.pitch, 
                                  gimba_IMU_data->Pitch, 
                                  pitch_dm_motor->Data.Velocity);
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
		
		Update_Gimbal_To_Vision_Packet();
		
}