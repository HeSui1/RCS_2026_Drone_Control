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

    gimbal_tx_data.q[0] = gimba_IMU_data->q[0]; // w
    gimbal_tx_data.q[1] = gimba_IMU_data->q[1]; // x
    gimbal_tx_data.q[2] = gimba_IMU_data->q[2]; // y
    gimbal_tx_data.q[3] = gimba_IMU_data->q[3]; // z

    gimbal_tx_data.yaw   = gimba_IMU_data->Yaw;
    gimbal_tx_data.pitch = gimba_IMU_data->Pitch;

    gimbal_tx_data.yaw_vel   = gimba_IMU_data->Gyro[IMU_GYRO_INDEX_YAW];
    gimbal_tx_data.pitch_vel = gimba_IMU_data->Gyro[IMU_GYRO_INDEX_PITCH];


    // 【注意】: 视觉需要的 mode 是 (0: 空闲, 1: 自瞄, 2: 小符, 3: 大符)

    gimbal_tx_data.mode = 1;

    gimbal_tx_data.bullet_speed = 15.0f; 
    gimbal_tx_data.bullet_count = 0;     

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
				

		DM_Motor_Init_Config_s dm_pitch_config = {
						.motor_type = DM3507,
						.control_mode = POSITION_VELOCITY,         
						.can_init_config = {
								.can_handle = &hfdcan1,   

								.tx_id = 0x02,            //CAN ID 
								.rx_id = 0x02,            //Master ID 
						}


				};						
				// 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动		
			pitch_dm_motor = DM_Motor_Init(&dm_pitch_config);
				
			gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
			gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
		}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
		pitch_target_rad =pitch_dm_motor->Data.Position-(gimbal_cmd_recv.pitch * -0.0174533f-gimba_IMU_data->Pitch * 0.0174533f);//gimbal_cmd_recv.pitch * 0.0174533f;// gimbal_cmd_recv.pitch * 0.0174533f  ;//-(gimba_IMU_data->Pitch * 0.0174533f-pitch_dm_motor->Data.Position);
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);


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

        if (pitch_dm_motor->Data.State != 1) {
            
            DM_Motor_Command(pitch_dm_motor, Motor_Enable);
        } else {

            DM_Motor_Set_PosVel_Target(pitch_dm_motor, pitch_target_rad, 0.5f);
        }
        break;
				
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
			
			  DJIMotorEnable(yaw_motor);	

						
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);


        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈


        if (pitch_dm_motor->Data.State != 1) {
            
            DM_Motor_Command(pitch_dm_motor, Motor_Enable);
        } else {
            DM_Motor_Set_PosVel_Target(pitch_dm_motor, pitch_target_rad, 0.5f);
        }
		
        break;
    default:
        break;
    }



    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    USART_Vofa_Justfloat_Transmit(-gimbal_cmd_recv.pitch, 
                                  gimba_IMU_data->Pitch, 
                                  pitch_dm_motor->Data.Velocity);
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
		
		Update_Gimbal_To_Vision_Packet();
		
}