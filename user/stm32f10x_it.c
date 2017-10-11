/**
  ******************************************************************************
  * @file EXTI/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"
#include "system.h"
#include "movebase.h"
#include "time.h"

extern uint8_t g_RxUpBuf[];
extern uint8_t g_RxStation[];
extern uint8_t g_RxBufferC[];	
extern __IO uint8_t g_TxCounter;	
extern uint8_t g_RxCounter;

extern uint32_t up1;
extern  uint8_t read_star;
extern uint8_t num_motor_flag ;

extern  __IO uint32_t g_TimingDelay;

extern int R485End;
static unsigned int systick_num = 10;
/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup EXTI_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
    if(systick_num != 0)
	{
	    systick_num--;
		//g_TimingDelay++;
	}
	else
	{
	    systick_num = 20;
		g_TimingDelay++;
	}
}  
 //con8,for 485communication
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
		USART_ClearFlag(USART1,USART_IT_RXNE);	
	    /* Read one byte from the receive data register */
	    g_RxBufferC[R485End++] = USART_ReceiveData(USART1);
		if(R485End >= 8)
		{
		    R485End %= 8; 
			gHalData->FbMotor = 1;
		}
		
	 
    }
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE ) == SET)
    {
        USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR其实就是清除标志
    }

}
//bluetooth communication
#if 0
void USART2_IRQHandler(void)
{
	static int flagssss = 0;
    int start = 0;
	static int num = 0;
	
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE ) == SET)
    {
        USART_ClearFlag(USART2,USART_FLAG_ORE); //读SR其实就是清除标志
    }
//	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==SET)
//	{
//	    USART_ClearFlag(USART2,USART_FLAG_RXNE);//TCIE,TE,RE
//	}

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{	
		/* Read one byte from the receive data register */
		g_RxUpBuf[num++] = USART_ReceiveData(USART2);	
		if(num >= 35)
		{
		    start = 0;
			if(num > 70)
			{
			    num = 0;
			}
		    while((0x55 != g_RxUpBuf[start] || 0xaa != g_RxUpBuf[start+34] )&&(start < num - 34))
			{
	            start++;
				///////////////////////
			}
			if(num - start >= 35 && 0 == gInnerData->NeedUp)
			{
			    num = 0;
			    memcpy(gDownData,&g_RxUpBuf[start+1],33);
					memset(g_RxUpBuf,0,96);
			    gInnerData->NeedUp = 1;	
//				GPIO_ResetBits(GPIOD, GPIO_Pin_2);
//				flagssss++;
//				if(flagssss%3 == 0)
//				{					
//					GPIO_ResetBits(GPIOE, GPIO_Pin_1);
//				}
//				else
//				{
//					GPIO_SetBits(GPIOE, GPIO_Pin_1);
//				}	
				
//				GPIO_ResetBits(GPIOE, GPIO_Pin_1);
			}
		}
	}


}
#endif

void  USART2_IRQHandler(void)
{ 
  static  u8  temp_buff[100]="";
  static  u8  count=0;
	static u8 beat= 0;
	static u8 start=0;
  //temp  = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
  temp_buff[count++]  = USART_ReceiveData(USART2);
  
 // if(BUS485_Timeout > 300) count = 0;
	if(count >= 26)
	{
		while(temp_buff[start] != 0x55 && temp_buff[start+26] !=0xAA)
		{
			start++;
		}
		if(count - start >=26)
		{
			buletooth_motor_para.driver_motor_x_speed = *((int*)(&temp_buff[start+1]));
			buletooth_motor_para.driver_motor_y_speed = *((int*)(&temp_buff[start+5]));
			buletooth_motor_para.driver_motor_th_speed = *((int*)(&temp_buff[start+9]));
			memset(temp_buff,0,100);
			buletooth_motor_para.up_data_flag = 1;
			
			if(beat%10 == 0)
			{
				GPIO_ResetBits(GPIOE, GPIO_Pin_0);				
			}else
			{				
				GPIO_SetBits(GPIOE, GPIO_Pin_0);
			}
			beat++;
			count = 0;
			start= 0;
		}
		
	}
	else if(count > 80) count = 0;
}
	
//  if(!count &&  temp==0x55)
//  {
//    temp_buff[count]  = temp;
//    count=1;
//  }
//  else if(count)
//  {
//    temp_buff[count]  = temp;
//		if(count >= 26 && temp_buff[26] == 0xAA ) 
//		{
//			count = 0;
//			buletooth_motor_para.driver_motor_speed = *((int*)(&temp_buff[1]));
//			if(beat%10 == 0)
//			{
//				GPIO_ResetBits(GPIOE, GPIO_Pin_0);				
//			}else
//			{				
//				GPIO_SetBits(GPIOE, GPIO_Pin_0);
//			}
//			beat++;
//						
//		}
//		else if(count >= 26 && temp_buff[26] != 0xAA)
//		{
//			count = 0;
//		}
//    if(count) count++; 
//  }
}

#if 0
static char temp[200] ={0};
void USART2_IRQHandler(void)
{
	static int flagssss = 0;
  int start = 0;
	static int num = 0;
//	temp[num++] = USART_ReceiveData(USART2);
//	if(temp[0] == 0x55)
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
//		temp[num++] = USART_ReceiveData(USART2);
			if(num == 200)
			{
				num=0;
			}
		/*
		if(num >= 35)
		{
		    start = 0;
			if(num > 100)
			{
			    num = 0;
			}
		    while((0x55 != g_RxUpBuf[start] || 0xaa != g_RxUpBuf[start+34] )&&(start < num - 34))
			{
	            start++;
			}
			if(num - start >= 100 && 0 == gInnerData->NeedUp)
			{
			    num = 0;
			    memcpy(gDownData,&g_RxUpBuf[start+1],33);
					memset(g_RxUpBuf,0,96);
			    gInnerData->NeedUp = 1;	
			}
		}
		*/
	}		
}

#endif

//RFID communication
void UART4_IRQHandler(void)
{
    int temp = 0;
	static int num = 0;
	int start = 0;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{	
		/* Read one byte from the receive data register */
		g_RxStation[num++] = USART_ReceiveData(UART4);
		if(num>=16)
		{
		     while((0x02 != g_RxStation[start] || 0x0d != g_RxStation[start + 13] || 0x0a != g_RxStation[start + 14] || 0x03 != g_RxStation[start + 15])&&(start < num-14))
			 {
			     start++;
			 }
			 if(num - start >= 16)
			 {
			     num = 0;
				 gHalData->Station = 0;
				 for(temp = start + 4;temp < start + 11;temp++)
				 {
				     if(g_RxStation[temp] > 0x39)
					 {
						 gHalData->Station |= (g_RxStation[temp]- 0x37);
					 }
					 else
					 {
					     gHalData->Station |= (g_RxStation[temp]- 0x30);
					 }
					 gHalData->Station = gHalData->Station <<4;
				 }
				 gHalData->Station = gHalData->Station >>4;
			 }
		}
		if(num > 150)
		{
		    num = 0;
		}
	}

}
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval : None
  */
/**
  * @brief  This function handles USB Low Priority or CAN RX0 interrupts 
  *   requests.
  * @param  None
  * @retval : None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
//	static int flagssss = 0;
	static char flag =0;
    CanRxMsg RxMessage;
	RxMessage.StdId = 0;
	//CA	N_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	switch(RxMessage.StdId & 0x780)
	{
	    case 0x580:// 1011 SDO(TX)
		    switch(((int)RxMessage.Data[1])|((int)RxMessage.Data[2]<<8))
			{
			    case 0x6061:
				    gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[ACTUALTYPE].Value[0] = RxMessage.Data[4];
					break;
				case 0x6041:
				    gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[STATUSWORD].Value[0] = RxMessage.Data[4];
					gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[STATUSWORD].Value[1] = RxMessage.Data[5];
					break;
			}
			break;
		case 0x180: //0011 PDO(TX)
			
		  gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[STATUSWORD].Value[0] = RxMessage.Data[0];
			gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[STATUSWORD].Value[1] = RxMessage.Data[1];
			gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[ACTUALPOS].Value[0]	= RxMessage.Data[2];
			gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[ACTUALPOS].Value[1]	= RxMessage.Data[3];
			gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[ACTUALPOS].Value[2]	= RxMessage.Data[4];
			gHalData->WheelHal[(RxMessage.StdId & 0x7f) - 1].ObDict[ACTUALPOS].Value[3]	= RxMessage.Data[5];
		if(read_star == 1)
		{			
		if(flag ==0)
			{
				flag =1;
				num_motor_flag = (RxMessage.StdId & 0x7f) -1;
				up1 = *((int*)(&(RxMessage.Data[2])));
				
			}
		}
//			flagssss++;
//			if(flagssss%3 == 0)
//				{					
//					GPIO_ResetBits(GPIOE, GPIO_Pin_1);
//				}
//				else
//				{
//					GPIO_SetBits(GPIOE, GPIO_Pin_1);
//				}	
			break;
		default:
		    break;
	}
}
void USB_HP_CAN1_TX_IRQHandler(void)
{
    
}
////////////////键盘行线1中断
void EXTI2_IRQHandler(void)
{

}

void TIM6_IRQHandler(void)
{  
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
