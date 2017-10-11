/****************************************************************************
* Copyright (C), 2013 ���ݺ��ϻ��������޹�˾         
*
* �ļ���: main.c
* ���ݼ���:	
*       
*	�ܶ���STM32������������ƶ�ƽ̨�ײ������������ 
	����MDK�汾��        3.8
	���ڹٷ������汾�� 3.5
	ʵ��485���Ա���������ͨ�ţ�pc5��ѹ�ɼ���CANͨ�źʹ���ͨ��
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2013-1-1    �������ļ�
*
***************************************************************************/
#include "movebase.h"
#include "crabdrive.h"
#include "time.h"


//��ֹ��ʾ����������֧�ֶ��ֽڵľ��桱����870����
#pragma diag_suppress 870

/* Private macro -------------------------------------------------------------*/
//#define TXBUFFERSIZE   (countof(g_TxBuffer) - 1)
//#define RXBUFFERSIZE   (countof(g_RxBuffer) - 1)
//#define countof(a)     (sizeof(a) / sizeof(*(a)))
                       
volatile uint8_t g_TxCounter = 0x00;
uint8_t g_RxCounter = 0x00; 

vu16 ADC_ConvertedValue;		                     //����ɼ����ĵ�Դ��ѹֵ

/* Private variables ---------------------------------------------------------*/
volatile uint32_t g_TimingDelay = 500;	             //������������ms


/****************************************************************************
* ��    �ƣ�int main(void)
* ��    �ܣ�������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
int main(void)
{
//    static int beating = 0;


	//��ʼ��STM32оƬ��ؼĴ���
	Device_Init();
	//��ʼ��ϵͳ�����Ͳ���
	System_Init();
	CanOpen_Init();
	//������ʹ��
	Drivers_Init();
	g_TimingDelay = 0;
	TIMx_PWMx_Init(TIM6,199,719,GPIOA,GPIO_Pin_1,0);//500HZ 

	GPIO_ResetBits(GPIOD, GPIO_Pin_3);
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	read_star = 1;
	//������ѭ��
	while(1)
	{
			move_process();
//		gHalData->WheelHal[0].CmdVel = 0.2;
//		gHalData->WheelHal[1].CmdVel = 0.2;
//		gHalData->WheelHal[2].CmdVel = 0.2;
//		gHalData->WheelHal[3].CmdVel = 0.2;
//		gHalData->WheelHal[4].CmdVel = 0.2;
//		gHalData->WheelHal[5].CmdVel = 0.2;
			Data_Send();
			Status_UpLoad();
	}
	/*
	while(1)
	{
	//	SentString("asdfasdf");
	    if(0 != g_TimingDelay)
		{
			 //���÷�ת�����ϲ����²��Ƿ�����������������
			 if(0 == beating % 2)
			 {
			     GPIO_SetBits(GPIOD, GPIO_Pin_2);
				 GPIO_SetBits(GPIOE, GPIO_Pin_0);
			 }
			 else
			 {	 if(0 == gInnerData->FatalErr)
			     {
			         GPIO_ResetBits(GPIOD, GPIO_Pin_2);
				 }
				 GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			 }
			 beating++;

		     g_TimingDelay = 0;
		   
		     //���ڳ����������
			// Obstacle();
			 	 
			 //�����������ٶ�
			 CalMotor();

			 //���մŵ���IO�����ݣ������浽hal��
			 //MagIO_Check();

			 //�����ƶ�ƽ̨״̬��λ�ã����浽������
			 //Status_Handle();

			 //������������ĸ������ٶȣ����浽hal��
			 Command_Handle();
			 gHalData->WheelHal[0].CmdVel = -0.8;
			 gHalData->WheelHal[1].CmdVel = 0.8;
			 gHalData->WheelHal[2].CmdVel = 0.8;
			 gHalData->WheelHal[3].CmdVel = 0.8;

			 //���ĸ������ٶȷ��͵�������
			 Data_Send();

			 //������Ҫ��״̬����ͨ�����ڷ������ϲ�
			 Status_UpLoad();	
		}
		
	}
	*/
}

