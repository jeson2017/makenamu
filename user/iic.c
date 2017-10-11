/****************************************************************************
* Copyright (C), 2013 常州汉迪机器人
* 
*
* 文件名: iic.c
* 内容简述:
*       本程序包含了iiC协议的底层驱动函数，I2C的实现是由软件模拟实现的
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2012-12-28 张干    创建该文件
*
*/


/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* I2C控制线的定义 */
/*#define SCL_H         GPIOB->BSRR = GPIO_Pin_10			   
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11*/
//可根据io口情况，随机选用I2C总线，只需修改此处和GPIO配置处即可。
#define SCL_H         GPIOC->BSRR = GPIO_Pin_0			   
#define SCL_L         GPIOC->BRR  = GPIO_Pin_0 
   
#define SDA_H         GPIOC->BSRR = GPIO_Pin_1
#define SDA_L         GPIOC->BRR  = GPIO_Pin_1

#define SCL_read      GPIOC->IDR  & GPIO_Pin_0
#define SDA_read      GPIOC->IDR  & GPIO_Pin_1


void I2C_delay(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
void I2C_FM_Init(void);
bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
extern void Delay(__IO uint32_t nCount);
u8 I2C_ReceiveByte(void);

/****************************************************************************
* 名    称：void I2C_Configuration(void)
* 功    能：I2C FM收音机模块TEA5767控制线的初始化 
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;    
  /* 配置PB10,PB11为I2C的 SCL SDL */
  /*GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                               
  GPIO_Init(GPIOB, &GPIO_InitStructure); */
  /*GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                               
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;                                               
  GPIO_Init(GPIOB, &GPIO_InitStructure); */

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                               
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;                                               
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  

}

/****************************************************************************
* 名    称：bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* 功    能：I2C写
* 入口参数：uint8_t* pBuffer --待写入的数组  uint8_t WriteAddr--器件地址  uint8_t NumByteToWrite--写入的字节数
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  	if(!I2C_Start())
	{
	    printf("Start over\r\n");
	    return FALSE;
	}
    I2C_SendByte(WriteAddr);         //写器件地址 
    if(!I2C_WaitAck())				 //等待应答
	{
	    printf("Wait over\r\n");
	    I2C_Stop();
	    return FALSE;
	}	
	 
	while(NumByteToWrite--)
	{
	    I2C_SendByte(* pBuffer);
	    I2C_WaitAck();
        pBuffer++;
	}
	I2C_Stop(); 	
	return TRUE;
}

/****************************************************************************
* 名    称：void I2C_delay(void)
* 功    能：I2C 控制延时函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_delay(void)
{	
   u8 i=2; 
   while(i) 
   { 
     i--; 
   } 
}
/****************************************************************************
* 名    称：bool I2C_Start(void)
* 功    能：I2C起始状态
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
bool I2C_Start(void)
{
	SDA_H;						//SDA置高
	SCL_H;						//SCL置高
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;						//SDA置低
	I2C_delay();
	return TRUE;
}
/****************************************************************************
* 名    称：void I2C_Stop(void)
* 功    能：I2C停止状态
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_Stop(void)
{
	SCL_L;				  
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}
/****************************************************************************
* 名    称：void I2C_Ack(void)
* 功    能：I2C ACK应答
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
/****************************************************************************
* 名    称：void I2C_NoAck(void)
* 功    能：I2C 无应答
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}
/****************************************************************************
* 名    称：bool I2C_WaitAck(void)
* 功    能：I2C等待应答
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
      return FALSE;
	}
	SCL_L;
	return TRUE;
}
/****************************************************************************
* 名    称：void I2C_SendByte(u8 SendByte)
* 功    能：
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}
/****************************************************************************
* 名    称：u8 I2C_ReceiveByte(void)
* 功    能：I2C接收字节
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
u8 I2C_ReceiveByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
        ReceiveByte<<=1;      
        SCL_L;
        I2C_delay();
	    SCL_H;
        I2C_delay();	
        if(SDA_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}

/****************************************************************************
* 名    称：bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
* 功    能：I2C 读
* 入口参数：u8* pBuffer-- 数组     u8 length--读出的字节数  u8 DeviceAddress--器件地址
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/	
bool I2C_Read(u8* pBuffer,   u8 length,   u8 DeviceAddress)
{		
    if(!I2C_Start())
	{
	    return FALSE;
    }
    I2C_SendByte(DeviceAddress);                            //器件地址 
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}  
	while(length--)
	{
		*pBuffer = I2C_ReceiveByte();
     	if(length == 1)
		{
		    I2C_NoAck();
		}
     	else
		{
		    I2C_Ack();
		} 
        pBuffer++;
	}
	I2C_Stop(); 	
	return TRUE;
}



