/****************************************************************************
* Copyright (C), 2013 常州汉迪机器人有限公司         
*
* 文件名: main.c
* 内容简述:	
*       
*	奋斗版STM32开发板基础的移动平台底层控制器主函数 
	基于MDK版本：        3.8
	基于官方外设库版本： 3.5
	实现485绝对编码器数据通信，pc5电压采集，CAN通信和串口通信
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2013-1-1    创建该文件
*
***************************************************************************/
#include "movebase.h"
#include "crabdrive.h"
#include "time.h"


//禁止显示“编译器不支持多字节的警告”——870警告
#pragma diag_suppress 870

/* Private macro -------------------------------------------------------------*/
//#define TXBUFFERSIZE   (countof(g_TxBuffer) - 1)
//#define RXBUFFERSIZE   (countof(g_RxBuffer) - 1)
//#define countof(a)     (sizeof(a) / sizeof(*(a)))
                       
volatile uint8_t g_TxCounter = 0x00;
uint8_t g_RxCounter = 0x00; 

vu16 ADC_ConvertedValue;		                     //保存采集到的电源电压值

/* Private variables ---------------------------------------------------------*/
volatile uint32_t g_TimingDelay = 500;	             //设置任务周期ms


/****************************************************************************
* 名    称：int main(void)
* 功    能：主函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
int main(void)
{
//    static int beating = 0;


	//初始化STM32芯片相关寄存器
	Device_Init();
	//初始化系统变量和参数
	System_Init();
	CanOpen_Init();
	//驱动器使能
	Drivers_Init();
	g_TimingDelay = 0;
	TIMx_PWMx_Init(TIM6,199,719,GPIOA,GPIO_Pin_1,0);//500HZ 

	GPIO_ResetBits(GPIOD, GPIO_Pin_3);
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	read_star = 1;
	//进入主循环
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
			 //设置反转，供上层检测下层是否死机；做紧急保护
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
		   
		     //用于超声波测距离
			// Obstacle();
			 	 
			 //计算电机反馈速度
			 CalMotor();

			 //接收磁导航IO的数据，并保存到hal层
			 //MagIO_Check();

			 //计算移动平台状态和位置，保存到交互层
			 //Status_Handle();

			 //根据命令计算四个轮子速度，保存到hal层
			 Command_Handle();
			 gHalData->WheelHal[0].CmdVel = -0.8;
			 gHalData->WheelHal[1].CmdVel = 0.8;
			 gHalData->WheelHal[2].CmdVel = 0.8;
			 gHalData->WheelHal[3].CmdVel = 0.8;

			 //将四个轮子速度发送到驱动器
			 Data_Send();

			 //根据需要将状态数据通过串口反馈给上层
			 Status_UpLoad();	
		}
		
	}
	*/
}

