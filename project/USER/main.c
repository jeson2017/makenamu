#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "can.h" 
#include "system.h"
#include "canopen.h"
#include "pstwo.h"
#include "drive.h"
#include "timer.h"
#include "rs485.h"
#include "magc.h"
 
 
 
/************************************************
 ALIENTEK精英STM32开发板实验25
 CAN 实验   
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
 



 int main(void)
 {	 
	u8 key;
	u8 runmode=0;
	u8 mode_change = 0;
//	u8 i=0,t=0;
//	u8 cnt=0;
//	u8 canbuf[8];
//	u8 res;
//	u8 mode=CAN_Mode_Normal;//CAN工作模式;CAN_Mode_Normal(0)：普通模式，CAN_Mode_LoopBack(1)：环回模式
	s16 speed,speed1,speed2; 
	s16 swerve;           //转弯量	  
	PS2_Init();
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
//	uart_init(115200);	 	//串口初始化为115200
	
	LED_Init();		  		//初始化与LED连接的硬件接口
//	LCD_Init();			   	//初始化LCD	
//	KEY_Init();				//按键初始化		 	
   RS485_Init(9600);	//初始化RS485
	 
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);//CAN初始化环回模式,波特率500Kbps    
	System_Init();
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN普通模式初始化, 波特率500Kbps 
	CanOpen_Init();
	Drivers_Init();
	TIM2_Int_Init(9,719);//100Khz的计数频率，计数到2000为10us 
	TIM3_Int_Init(99,7199);//10Khz的计数频率，计数到2000为10ms
	
/*
 	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(60,50,200,16,16,"WarShip STM32");	
	LCD_ShowString(60,70,200,16,16,"CAN TEST");	
	LCD_ShowString(60,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(60,110,200,16,16,"2015/1/11");
	LCD_ShowString(60,130,200,16,16,"LoopBack Mode");	 
	LCD_ShowString(60,150,200,16,16,"KEY0:Send WK_UP:Mode");//显示提示信息		
  POINT_COLOR=BLUE;//设置字体为蓝色	  
	LCD_ShowString(60,170,200,16,16,"Count:");			//显示当前计数值	
	LCD_ShowString(60,190,200,16,16,"Send Data:");		//提示发送的数据	
	LCD_ShowString(60,250,200,16,16,"Receive Data:");	//提示接收到的数据
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,0);//CAN普通模式初始化, 波特率500Kbps 
	POINT_COLOR=RED;//设置字体为红色 	
*/
	while(1)
	{
		// 处理一下要用到的LED显示灯
//		move_process();
//		if(runmode == 0){
			psinput();
//		}
		if(psinputstat == PS_ROLL){ // 模式切换
			runmode = 1;
		}else if(psinputstat == PS_SIG){ // 速度调整
			Para_Speed += 1;
			if(Para_Speed >20)
				Para_Speed = 20;
		}else if(psinputstat == PS_X){ // 速度调整
			Para_Speed =1;
			runmode = 0;
		}
		if(READMAGCDELAY >= 5){
			READ_MAGC_DATA();
			READMAGCDELAY = 0;
		}
		if(runmode == 0){
			move_process();
		}
		else{
			auto_run();
		}
		
		Data_Send();
		Status_UpLoad();
	}
	return 0;
}



