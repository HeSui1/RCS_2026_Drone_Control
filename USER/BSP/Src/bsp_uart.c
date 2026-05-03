/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the  BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "Remote_Control_COD.h"
#include "Referee_System.h"
#include "VT03.h"
#include "cmsis_os.h"

/**
 * @brief 裁判系统数据发送函数 (适配你的工程)
 * @param send   要发送的数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	/* 1. 使用 HAL 库的 DMA 发送函数替代原作者的面向对象 BSP */
	// 注意：这里假设你连接电源管理模块/裁判系统的串口是 huart1
	
HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, send, tx_len, 1000);
    
    if (status != HAL_OK) {
        __NOP(); // 断点
    }
	
	/* 2. 保留原作者的延时逻辑，防止发包过快导致带宽超限或被服务器踢下线 */
	osDelay(115);
}



void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){
 
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;


		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		Rx_Buf[12] =  0x00;
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));
	

}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void){

	USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);
	
	USART_RxDMA_MultiBuffer_Init(&huart7, (uint32_t *)VT03_MultiRx_Buf[0], (uint32_t *)VT03_MultiRx_Buf[1], VT03_RX_BUF_NUM);
	
	USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

}

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxXferSize    = DataLength * 2;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
		
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);	
	
	
}



/**
  * @brief  USER USART5 Reception Event Callback.(SBUS remote_ctrl)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    /* Current memory buffer used is Memory 0 */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
		
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);
          
				  /* Switch Memory 0 to Memory 1*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					
				  /* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

				  /* Juge whether size is equal to the length of the received data */
					if(Size == SBUS_RX_BUF_NUM)
					{
						
						SCB_InvalidateDCache_by_Addr((uint32_t *)SBUS_MultiRx_Buf[0], 32);
						/* Memory 0 data update to remote_ctrl*/
						SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
					
					}
					
			}
			/* Current memory buffer used is Memory 1 */
			else{
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);
				 
				  /* Switch Memory 1 to Memory 0*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					/* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

					if(Size == SBUS_RX_BUF_NUM)
					{
						SCB_InvalidateDCache_by_Addr((uint32_t *)SBUS_MultiRx_Buf[1], 32);
						/* Memory 1 to data update to remote_ctrl*/
						SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
					}
					
			}
			
}

/**
  * @brief  USER USART1 Reception Event Callback.(Referee_System)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */

/**
 * @brief  USER USART1 Reception Event Callback.(VT03 Remote Control)
 */
//static void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* Current memory buffer used is Memory 0 */
//    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
//    {
//        /* Disable DMA */
//        __HAL_DMA_DISABLE(huart->hdmarx);
//        
//        /* Switch Memory 0 to Memory 1*/
//        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
//        
//        /* Reset the receive count (VT03_RX_BUF_NUM = 21) */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT03_RX_BUF_NUM * 2);

//        /* 严格判断数据长度是否为 21 字节 */
//        if(Size == VT03_RX_BUF_NUM)
//        {
//            // 清理 D-Cache 保证读取到的是 DMA 刚搬运进 RAM 的新鲜数据
//            SCB_InvalidateDCache_by_Addr((uint32_t *)VT03_MultiRx_Buf[0], 32); 
//            // 传入刚才写好的解析函数
//            VT03_Parse(VT03_MultiRx_Buf[0]);
//        }
//    }
//    /* Current memory buffer used is Memory 1 */
//    else
//    {
//        /* Disable DMA */
//        __HAL_DMA_DISABLE(huart->hdmarx);
//        
//        /* Switch Memory 1 to Memory 0*/
//        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
//        
//        /* Reset the receive count */
//        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT03_RX_BUF_NUM * 2);

//        if(Size == VT03_RX_BUF_NUM)
//        {
//            SCB_InvalidateDCache_by_Addr((uint32_t *)VT03_MultiRx_Buf[1], 32);
//            VT03_Parse(VT03_MultiRx_Buf[1]);
//        }
//    }
//}

static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
	{
		
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
				
				  if(Size >= 10){
						
						// === 新增：清理 D-Cache (注意长度是 160) ===
            SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_System_Info_MultiRx_Buf[0], 160);
						
						    Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
				
				        memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);

				        __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
          }
				
	}
	else
	{
				__HAL_DMA_DISABLE(huart->hdmarx);
				
				((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
		    if(Size >= 10){
        
						// === 新增：清理 D-Cache (注意长度是 160) ===
            SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_System_Info_MultiRx_Buf[1], 160);
					
			         Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
				
				       memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);

				       __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
      }
					
	}
  
}

/**
 * @brief  USER UART7 Reception Event Callback.(VT03 Remote Control)
 */
static void USER_UART7_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT03_RX_BUF_NUM * 2);

        if(Size == VT03_RX_BUF_NUM)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)VT03_MultiRx_Buf[0], 32); 
            VT03_Parse(VT03_MultiRx_Buf[0]);
        }
    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT03_RX_BUF_NUM * 2);

        if(Size == VT03_RX_BUF_NUM)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)VT03_MultiRx_Buf[1], 32);
            VT03_Parse(VT03_MultiRx_Buf[1]);
        }
    }
}


/**
  * @brief  USER USART10 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USER USART3 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USER USART2 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(huart == &huart5){
	
		  USER_USART5_RxHandler(huart,Size);
			
	 } 
	 
	 if(huart == &huart1){
	 
      USER_USART1_RxHandler(huart,Size);
			
	 }
	 if(huart == &huart7){
		 
        USER_UART7_RxHandler(huart, Size);
		 
    }
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}



