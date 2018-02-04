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
 ALIENTEK��ӢSTM32������ʵ��25
 CAN ʵ��   
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
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
//	u8 mode=CAN_Mode_Normal;//CAN����ģʽ;CAN_Mode_Normal(0)����ͨģʽ��CAN_Mode_LoopBack(1)������ģʽ
	s16 speed,speed1,speed2; 
	s16 swerve;           //ת����	  
	PS2_Init();
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
//	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
//	LCD_Init();			   	//��ʼ��LCD	
//	KEY_Init();				//������ʼ��		 	
   RS485_Init(9600);	//��ʼ��RS485
	 
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);//CAN��ʼ������ģʽ,������500Kbps    
	System_Init();
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN��ͨģʽ��ʼ��, ������500Kbps 
	CanOpen_Init();
	Drivers_Init();
	TIM2_Int_Init(9,719);//100Khz�ļ���Ƶ�ʣ�������2000Ϊ10us 
	TIM3_Int_Init(99,7199);//10Khz�ļ���Ƶ�ʣ�������2000Ϊ10ms
	
/*
 	POINT_COLOR=RED;//��������Ϊ��ɫ 
	LCD_ShowString(60,50,200,16,16,"WarShip STM32");	
	LCD_ShowString(60,70,200,16,16,"CAN TEST");	
	LCD_ShowString(60,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(60,110,200,16,16,"2015/1/11");
	LCD_ShowString(60,130,200,16,16,"LoopBack Mode");	 
	LCD_ShowString(60,150,200,16,16,"KEY0:Send WK_UP:Mode");//��ʾ��ʾ��Ϣ		
  POINT_COLOR=BLUE;//��������Ϊ��ɫ	  
	LCD_ShowString(60,170,200,16,16,"Count:");			//��ʾ��ǰ����ֵ	
	LCD_ShowString(60,190,200,16,16,"Send Data:");		//��ʾ���͵�����	
	LCD_ShowString(60,250,200,16,16,"Receive Data:");	//��ʾ���յ�������
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,0);//CAN��ͨģʽ��ʼ��, ������500Kbps 
	POINT_COLOR=RED;//��������Ϊ��ɫ 	
*/
	while(1)
	{
		// ����һ��Ҫ�õ���LED��ʾ��
//		move_process();
//		if(runmode == 0){
			psinput();
//		}
		if(psinputstat == PS_ROLL){ // ģʽ�л�
			runmode = 1;
		}else if(psinputstat == PS_SIG){ // �ٶȵ���
			Para_Speed += 1;
			if(Para_Speed >20)
				Para_Speed = 20;
		}else if(psinputstat == PS_X){ // �ٶȵ���
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



