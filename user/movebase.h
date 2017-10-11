/****************************************************************************
* Copyright (C), 2013 汉迪机器人
*
* 文件名: create.h
* 内容简述:
*       本程序包含了操作iRobot Create的函数声明
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2013-1-22   张干  创建该文件
****************************************************************************/

#ifndef __CREATE_H
#define __CREATE_H

#include "stdlib.h"
#include "stdio.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_fsmc.h"
#include "fsmc_sram.h" 
#include "misc.h"
#include  <stdarg.h>
#include "system.h"
#include "canopen.h"

#define Mulipl(x,y,z,u,v,w) (((x)*(u))+((y)*(v))+((z)*(w)))
#define MatM(x,y,z)  sqrt((x)*(x)+(y)*(y)+(z)*(z))

#define DIV_SPEED  (500.0)
/* functions ---------------------------------------------------------*/
int SendChar (char ch) ;
int TransUp (int ch) ;
void SentString(char*);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void SetPLL(void);
void Delay(__IO uint32_t nCount);  
void NVIC_Configuration(void);
void Usart1_Init(void);
void Usart2_Init(void);
void Usart4_Init(void);

void Device_Init(void);
void ADC_Configuration(void);
bool CAN_Configuration(void);


void MagIO_Check(void);
void Status_UpLoad(void);
void CalMotor(void);
void Obstacle(void);
void Drivers_Init(void);
void Drivers_Start(void);

typedef struct _Motor_param_
{
	int driver_motor_x_speed;
	int driver_motor_y_speed;
	int driver_motor_th_speed;
	char driver_motor_direction;
	double div_speed;
	char up_data_flag;
	
}Buletooth_Motor_param;



extern vu16 ADC_ConvertedValue;
extern Buletooth_Motor_param buletooth_motor_para;



#endif

