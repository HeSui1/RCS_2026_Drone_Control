/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dm_motor.h"
#include <stdlib.h> // 包含 malloc
#include <string.h> // 包含 memset


// 【新增】定义最大支持的达妙电机数量
#define DM_MOTOR_CNT 10 

// 【新增】达妙电机实例池与全局索引
static DM_Motor_Info_Typedef *dm_motor_instance[DM_MOTOR_CNT] = {NULL}; 
static uint8_t dm_idx = 0;


static void DM_Motor_Decode(CANInstance *_instance);

/**
 * @brief 达妙电机初始化函数
 * @param config 初始化配置指针
 * @return 动态分配的 DM_Motor 实例指针
* 注意速度限制设为0时电机不被控制，软绵无力。
 */
DM_Motor_Info_Typedef* DM_Motor_Init(DM_Motor_Init_Config_s *config)
{
    // 1. 动态分配内存
    DM_Motor_Info_Typedef *instance = (DM_Motor_Info_Typedef *)malloc(sizeof(DM_Motor_Info_Typedef));
    memset(instance, 0, sizeof(DM_Motor_Info_Typedef));

    // 2. 加载基础配置参数
    instance->Control_Mode = config->control_mode;


    if (config->param_limits.P_MAX != 0.0f) 
    {
        instance->Param_Range = config->param_limits;
    } 
    else 
    {
        switch (config->motor_type) 
        {
						case DM4310:
                instance->Param_Range.P_MAX = 12.56637f;
                instance->Param_Range.V_MAX = 10.0f;
                instance->Param_Range.T_MAX = 10.0f;	
							break;
            case DM3507:
                instance->Param_Range.P_MAX = 12.566f;
                instance->Param_Range.V_MAX = 50.0f;
                instance->Param_Range.T_MAX = 5.0f;
                break;
            case DM8009: // 依据原作者的数据
                instance->Param_Range.P_MAX = 3.141593f;
                instance->Param_Range.V_MAX = 45.0f;
                instance->Param_Range.T_MAX = 54.0f;
                break;
            // 如果你还有 DM4310 或其他型号，可以在这里继续加 case
            default:
                // 默认给一个宽泛的值，防止解码报错
                instance->Param_Range.P_MAX = 12.5f;
                instance->Param_Range.V_MAX = 50.0f;
                instance->Param_Range.T_MAX = 10.0f;
                break;
        }
    }	
	
    instance->FDCANFrame.TxIdentifier = config->can_init_config.tx_id;
    instance->FDCANFrame.RxIdentifier = config->can_init_config.rx_id;

    // 3. 注册到 CAN 总线
    // 把当前电机的实例指针绑到 id 上，这样触发接收中断时，回调函数就能认出是哪个电机
    config->can_init_config.id = instance; 
    config->can_init_config.can_module_callback = DM_Motor_Decode; // 绑定解析回调
    
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 4. 标记初始化完成
    instance->Data.Initlized = true;
		
		if (dm_idx < DM_MOTOR_CNT) {
        dm_motor_instance[dm_idx++] = instance;
    } else {
        // 如果超过最大数量，可以加个错误打印或断言 (while(1))
    }

    return instance;
}



//------------------------------------------------------------------------------



float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static int float_to_uint(float x, float x_min, float x_max, int bits);

//------------------------------------------------------------------------------

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }

  float len = Max_Value - Min_Value;

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}
//------------------------------------------------------------------------------


/**
 * @brief  Transmit enable disable save zero position Command to DM motor 
 * @param  *DM_Motor：pointer to the DM_Motor instance
 * @param  CMD：Transmit Command  
 */
void DM_Motor_Command(DM_Motor_Info_Typedef *DM_Motor, DM_Motor_CMD_Type_e CMD)
{
    if (DM_Motor->motor_can_instance == NULL) return; // 安全检查

    CANInstance *instance = DM_Motor->motor_can_instance;

    // 恢复基础 ID (防止之前使用了位置/速度模式导致 ID 发生偏移)
    instance->txconf.Identifier = DM_Motor->FDCANFrame.TxIdentifier;

    // 达妙指令前7个字节固定为 0xFF
    memset(instance->tx_buff, 0xFF, 7); 

    switch(CMD){
        case Motor_Enable :
            instance->tx_buff[7] = 0xFC; 
            break;
        case Motor_Disable :
            instance->tx_buff[7] = 0xFD; 
            break;
        case Motor_Save_Zero_Position :
            instance->tx_buff[7] = 0xFE; 
            break;
        default:
            return;  
    }
    
    // 调用 BSP 层的发送接口
    CANTransmit(instance, 1);
}

/**
 * @brief  CAN Transmit DM motor Information
 * @param  *DM_Motor  pointer to the DM_Motor
 * @param  Postion Velocity KP KD Torgue: Target
 */
void DM_Motor_CAN_TxMessage(DM_Motor_Info_Typedef *DM_Motor, float Postion, float Velocity, float KP, float KD, float Torque)
{
    if(DM_Motor->motor_can_instance == NULL) return; // 安全检查

    CANInstance *instance = DM_Motor->motor_can_instance;
    uint8_t *tx_buff = instance->tx_buff; // 拿到发送缓冲区的指针，方便下面赋值

    if(DM_Motor->Control_Mode == MIT){
        uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;
         
        Postion_Tmp  = float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
        Velocity_Tmp = float_to_uint(Velocity, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);
        Torque_Tmp   = float_to_uint(Torque, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
        KP_Tmp = float_to_uint(KP, 0, 500, 12);
        KD_Tmp = float_to_uint(KD, 0, 5, 12);
         
        instance->txconf.Identifier = DM_Motor->FDCANFrame.TxIdentifier; // 确保使用基础 ID
         
        tx_buff[0] = (uint8_t)(Postion_Tmp >> 8);
        tx_buff[1] = (uint8_t)(Postion_Tmp);
        tx_buff[2] = (uint8_t)(Velocity_Tmp >> 4);
        tx_buff[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
        tx_buff[4] = (uint8_t)(KP_Tmp);
        tx_buff[5] = (uint8_t)(KD_Tmp >> 4);
        tx_buff[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
        tx_buff[7] = (uint8_t)(Torque_Tmp);

    } else if(DM_Motor->Control_Mode == POSITION_VELOCITY) {
        uint8_t *Postion_Tmp  = (uint8_t*) & Postion;
        uint8_t *Velocity_Tmp = (uint8_t*) & Velocity;
        
        // 【关键兼容】利用现成的 txconf 动态修改发送 ID
        instance->txconf.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;
        
        tx_buff[0] = *(Postion_Tmp);
        tx_buff[1] = *(Postion_Tmp + 1);
        tx_buff[2] = *(Postion_Tmp + 2);
        tx_buff[3] = *(Postion_Tmp + 3);
        tx_buff[4] = *(Velocity_Tmp);
        tx_buff[5] = *(Velocity_Tmp + 1);
        tx_buff[6] = *(Velocity_Tmp + 2);
        tx_buff[7] = *(Velocity_Tmp + 3);

    } else if(DM_Motor->Control_Mode == VELOCITY) {
        uint8_t *Velocity_Tmp = (uint8_t*) & Velocity;
        
        // 【关键兼容】利用现成的 txconf 动态修改发送 ID
        instance->txconf.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;
        
        tx_buff[0] = *(Velocity_Tmp);
        tx_buff[1] = *(Velocity_Tmp + 1);
        tx_buff[2] = *(Velocity_Tmp + 2);
        tx_buff[3] = *(Velocity_Tmp + 3);
        tx_buff[4] = 0;
        tx_buff[5] = 0;
        tx_buff[6] = 0;
        tx_buff[7] = 0;
    }
     
    // 打包完毕，调用你的 BSP 接口将数据推入硬件 FIFO
    CANTransmit(instance, 1);
}


void DM_Motor_Set_MIT_Target(DM_Motor_Info_Typedef *motor, float pos, float vel, float kp, float kd, float torq)
{
    if (motor == NULL) return;
    motor->Control_Info.Position = pos;
    motor->Control_Info.Velocity = vel;
    motor->Control_Info.KP = kp;
    motor->Control_Info.KD = kd;
    motor->Control_Info.Torque = torq;
}

void DM_Motor_Set_PosVel_Target(DM_Motor_Info_Typedef *motor, float pos, float vel_limit)
{
    if (motor == NULL) return;
    motor->Control_Info.Position = pos;
    motor->Control_Info.Velocity = vel_limit; // 在该模式下，这是最大速度限制
}


/**
 * @brief  【新增】遍历所有达妙电机，统一发送控制帧
 * @note   这个函数应该被放在 1kHz 的定时任务 (如 CAN_Task) 中循环调用
 */
void DM_Motor_Control(void)
{
    DM_Motor_Info_Typedef *motor;

    // 遍历所有已注册的达妙电机
    for (size_t i = 0; i < dm_idx; ++i)
    {
        motor = dm_motor_instance[i];
        
        // 安全检查：如果指针非空且已经初始化
        if (motor != NULL && motor->Data.Initlized == true && motor->Data.State == 1)
        {
            // 直接将该电机 Control_Info 中的参数喂给底层的 CAN 发送函数
            DM_Motor_CAN_TxMessage(motor, 
                                   motor->Control_Info.Position, 
                                   motor->Control_Info.Velocity, 
                                   motor->Control_Info.KP, 
                                   motor->Control_Info.KD, 
                                   motor->Control_Info.Torque);
        }
    }
}

/**
 * @brief  【新增】一键使能所有已注册的达妙电机
 * @note   建议在所有电机 Init 完成后，在 Task 的进入循环前调用
 */
void DM_Motor_Enable_All(void)
{
    DM_Motor_Info_Typedef *motor;

    for (size_t i = 0; i < dm_idx; ++i)
    {
        motor = dm_motor_instance[i];
        
        // 确保指针安全且电机已被成功初始化
        if (motor != NULL && motor->Data.Initlized == true)
        {
            DM_Motor_Command(motor, Motor_Enable);
        }
    }
}

/**
 * @brief  【新增】一键失能所有已注册的达妙电机 (为了代码的完整性和安全性)
 */
void DM_Motor_Disable_All(void)
{
    DM_Motor_Info_Typedef *motor;

    for (size_t i = 0; i < dm_idx; ++i)
    {
        motor = dm_motor_instance[i];
        
        if (motor != NULL && motor->Data.Initlized == true)
        {
            DM_Motor_Command(motor, Motor_Disable);
        }
    }
}


//------------------------------------------------------------------------------

/**
 * @brief  达妙电机CAN接收回调函数 (替代原来的 DM_Motor_Info_Update)
 * @note   挂载在 CANInstance 上，收到对应 ID 的报文时会自动被 BSP 层调用
 */
static void DM_Motor_Decode(CANInstance *_instance)
{
    // 1. 从 instance 提取我们在 Init 时强行绑定的 DM_Motor 实例指针
    DM_Motor_Info_Typedef *DM_Motor = (DM_Motor_Info_Typedef *)_instance->id;
    uint8_t *Rx_Buf = _instance->rx_buff;

    // 2. 原来的 if(*Identifier != DM_Motor->FDCANFrame.RxIdentifier) return; 被删除了！
    // 因为你的 bsp_can 已经完美保证了进到这里的绝对是属于这个电机的报文。

    // 3. 下面的解析逻辑原封不动
    DM_Motor->Data.State = Rx_Buf[0] >> 4;
    DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) << 8) | ((uint16_t)(Rx_Buf[2]));
    DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) << 4) | ((uint16_t)(Rx_Buf[4]) >> 4);
    DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4] & 0xF) << 8) | ((uint16_t)(Rx_Buf[5]));
    
    DM_Motor->Data.Torque   = uint_to_float(DM_Motor->Data.T_int, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
    DM_Motor->Data.Position = uint_to_float(DM_Motor->Data.P_int, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
    DM_Motor->Data.Velocity = uint_to_float(DM_Motor->Data.V_int, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);

    DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
    DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);
}

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){

    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){

    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
