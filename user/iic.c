/****************************************************************************
* Copyright (C), 2013 ���ݺ��ϻ�����
* 
*
* �ļ���: iic.c
* ���ݼ���:
*       �����������iiCЭ��ĵײ�����������I2C��ʵ���������ģ��ʵ�ֵ�
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2012-12-28 �Ÿ�    �������ļ�
*
*/


/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* I2C�����ߵĶ��� */
/*#define SCL_H         GPIOB->BSRR = GPIO_Pin_10			   
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11*/
//�ɸ���io����������ѡ��I2C���ߣ�ֻ���޸Ĵ˴���GPIO���ô����ɡ�
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
* ��    �ƣ�void I2C_Configuration(void)
* ��    �ܣ�I2C FM������ģ��TEA5767�����ߵĳ�ʼ�� 
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;    
  /* ����PB10,PB11ΪI2C�� SCL SDL */
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
* ��    �ƣ�bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* ��    �ܣ�I2Cд
* ��ڲ�����uint8_t* pBuffer --��д�������  uint8_t WriteAddr--������ַ  uint8_t NumByteToWrite--д����ֽ���
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  	if(!I2C_Start())
	{
	    printf("Start over\r\n");
	    return FALSE;
	}
    I2C_SendByte(WriteAddr);         //д������ַ 
    if(!I2C_WaitAck())				 //�ȴ�Ӧ��
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
* ��    �ƣ�void I2C_delay(void)
* ��    �ܣ�I2C ������ʱ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
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
* ��    �ƣ�bool I2C_Start(void)
* ��    �ܣ�I2C��ʼ״̬
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_Start(void)
{
	SDA_H;						//SDA�ø�
	SCL_H;						//SCL�ø�
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;						//SDA�õ�
	I2C_delay();
	return TRUE;
}
/****************************************************************************
* ��    �ƣ�void I2C_Stop(void)
* ��    �ܣ�I2Cֹͣ״̬
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
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
* ��    �ƣ�void I2C_Ack(void)
* ��    �ܣ�I2C ACKӦ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
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
* ��    �ƣ�void I2C_NoAck(void)
* ��    �ܣ�I2C ��Ӧ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
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
* ��    �ƣ�bool I2C_WaitAck(void)
* ��    �ܣ�I2C�ȴ�Ӧ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
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
* ��    �ƣ�void I2C_SendByte(u8 SendByte)
* ��    �ܣ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
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
* ��    �ƣ�u8 I2C_ReceiveByte(void)
* ��    �ܣ�I2C�����ֽ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
u8 I2C_ReceiveByte(void)  //���ݴӸ�λ����λ//
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
* ��    �ƣ�bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
* ��    �ܣ�I2C ��
* ��ڲ�����u8* pBuffer-- ����     u8 length--�������ֽ���  u8 DeviceAddress--������ַ
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/	
bool I2C_Read(u8* pBuffer,   u8 length,   u8 DeviceAddress)
{		
    if(!I2C_Start())
	{
	    return FALSE;
    }
    I2C_SendByte(DeviceAddress);                            //������ַ 
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



