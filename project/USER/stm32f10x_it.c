/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "system.h"

 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}
/**
  * @brief  This function handles USB Low Priority or CAN RX0 interrupts 
  *   requests.
  * @param  None
  * @retval : None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
//	static int flagssss = 0;
//	static char flag =0;
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
//			if(read_star == 1)
//			{			
//				if(flag ==0)
//				{
//					flag =1;
//					num_motor_flag = (RxMessage.StdId & 0x7f) -1;
//					up1 = *((int*)(&(RxMessage.Data[2])));
//				}
//			}
			break;
		default:
		    break;
	}
}
/*
#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
}
#endif
*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
