/****************************************************************************
* Copyright (C), 2013 ���ϻ�����
*
* �ļ���: system.c
* ���ݼ���:
*       
*       
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2013-6-6     �������ļ�
****************************************************************************/
//#include "movebase.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f10x_can.h"
#include "system.h"
#include "led.h"

BaseSysSt* gSysData;
BaseComSt* gComData;
BaseInnerSt* gInnerData;
BaseHalSt* gHalData;
BaseDownSt* gDownData;
BaseUpSt* gUpData;

uint8_t g_RxUpBuf[160] = {0,};
uint8_t g_RxBufferC[110] = {0,};
uint8_t g_RxStation[150] = {0,};
int R485End = 0;



/****************************************************************************
* ��    �ƣ�void RCC_Configuration(void)
* ��    �ܣ�ϵͳʱ������Ϊ72MHZ�� ����ʱ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void RCC_Configuration(void)
{
    SystemInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			   //���ù���ʹ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
  					  	  | RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO , ENABLE);
}
/****************************************************************************
* ��    �ƣ�void GPIO_Configuration(void)
* ��    �ܣ�ϵͳʱ������Ϊ72MHZ�� ����ʱ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_1;				     //LED1 V6����Ϊͨ���������  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	//���ڴ��������Ĵ�����⡣��16��IO��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//���ڿ�����ͣ�������͵�λ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14;				     //����Ϊͨ���������  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    //����������Ӧ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // ��������   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3  | GPIO_Pin_5  | GPIO_Pin_7;				     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
/****************************************************************************
* ��    �ƣ�void NVIC_Configuration(void)
* ��    �ܣ��ж�Դ���ú���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ���ȼ���1 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	       //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //�����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
//    NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;			       //�ж��ж�
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //�������ȼ�0  ��Χ��0��1
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			       //�����ȼ�0	��Χ��0-7
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);	  

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			       //����1�ж��ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //�������ȼ�0  ��Χ��0��1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //�����ȼ�0	��Χ��0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			       //����2�ж��ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //�������ȼ�0  ��Χ��0��1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //�����ȼ�1	��Χ��0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;			       //����2�ж��ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //�������ȼ�0  ��Χ��0��1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			       //�����ȼ�1	��Χ��0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			       // ��ʱ��6�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //�������ȼ�0  ��Χ��0��1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;			       //�����ȼ�1	��Χ��0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/****************************************************************************
* ��    �ƣ�void Delay(__IO uint32_t nCount)
* ��    �ܣ��ۻ���ʱ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
/****************************************************************************
* ��    �ƣ�void Usart1_Init(void)
* ��    �ܣ�����1��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����ͨ��232ת485ģ����Ƶ��
* ���÷������� 
****************************************************************************/
void Usart1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
 
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //ʹ�ܴ���1ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A�˿� 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //���ÿ�©����
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A�˿� 

    USART_InitStructure.USART_BaudRate = 9600;						//����9600bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);							//���ô��ڲ�������   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//�����ж�ʹ��
    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);	
  
}
/****************************************************************************
* ��    �ƣ�void Usart2_Init(void)
* ��    �ܣ�����2��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ������������ͨ��
* ���÷������� 
****************************************************************************/
void Usart2_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
 
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE);	                           //ʹ�ܴ���2ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	         		                           //USART2 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		                               //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		                               //A�˿� 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	         	                               //USART2 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	                           //���ÿ�©����
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	                               //A�˿� 

    USART_InitStructure.USART_BaudRate = 115200;						                   //����115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		                       //����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			                       //ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;				                       //��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					       //�շ�ģʽ

    /* Configure USART1 */
    USART_Init(USART2, &USART_InitStructure);							                   //���ô��ڲ�������   
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);					                       //�����ж�ʹ��
	USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    /* Enable the USART1 */
    USART_Cmd(USART2, ENABLE);	
}


/****************************************************************************
* ��    �ƣ�void Usart4_Init(void)
* ��    �ܣ�����1��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ��������IC����ȡ
* ���÷������� 
****************************************************************************/
void Usart4_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
 
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4 , ENABLE);	                           //ʹ�ܴ���2ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         		                           //USART2 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		                               //�����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);		    		                               //A�˿� 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	         	                               //USART2 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	                           //���ÿ�©����
    GPIO_Init(GPIOC, &GPIO_InitStructure);		         	                               //A�˿� 

    USART_InitStructure.USART_BaudRate = 9600;						                   //����115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		                       //����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			                       //ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;				                       //��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					       //�շ�ģʽ

    /* Configure USART1 */
    USART_Init(UART4, &USART_InitStructure);							                   //���ô��ڲ�������   
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);					                       //�����ж�ʹ�� 
    /* Enable the USART1 */
    USART_Cmd(UART4, ENABLE);	
}
/****************************************************************************
* ��    �ƣ�bool CAN_Configuration(void)
* ��    �ܣ��ж�ģʽ�µ�CAN�շ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
int CAN_Configuration(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;  
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);		     //�˿ڸ���ΪCAN1   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	                 //PB8:CAN-RX 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			     //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					 //PB9:CAN-TX 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			 //����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* CAN�Ĵ�����ʼ�� */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN��Ԫ��ʼ�� */
    CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹��� 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  �Զ�����ģʽ
    //CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  0-�Զ��ش�   1-����ֻ��һ��
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  0-�Զ��ش�   1-����ֻ��һ��
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  0-���ʱ�±��ĻḲ��ԭ�б���  1-���ʱ���±��Ķ���
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    //CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� 0-���ȼ�ȡ���ڱ��ı�ʾ�� 1-���ȼ�ȡ���ڷ��������˳��
    //CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;	   //BTR-SILM/LBKM   CAN����ģʽ
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;		   //BTR-TS1 ʱ���1 ռ����2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
  
#if CAN_BAUDRATE == 1000 /* 1MBps */
    CAN_InitStructure.CAN_Prescaler =6;			   //BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+2+3)/6=1Mbps
#elif CAN_BAUDRATE == 500 /* 500KBps */
    CAN_InitStructure.CAN_Prescaler =12;
#elif CAN_BAUDRATE == 250 /* 250KBps */
    CAN_InitStructure.CAN_Prescaler =24;
#elif CAN_BAUDRATE == 125 /* 125KBps */
    CAN_InitStructure.CAN_Prescaler =48;
#elif  CAN_BAUDRATE == 100 /* 100KBps */
    CAN_InitStructure.CAN_Prescaler =60;
#elif  CAN_BAUDRATE == 50 /* 50KBps */
    CAN_InitStructure.CAN_Prescaler =120;
#elif  CAN_BAUDRATE == 20 /* 20KBps */
    CAN_InitStructure.CAN_Prescaler =300;
#elif  CAN_BAUDRATE == 10 /* 10KBps */
    CAN_InitStructure.CAN_Prescaler =600;
#endif  

    CAN_Init(CAN1, &CAN_InitStructure);

    /* CAN��������ʼ�� */
    CAN_FilterInitStructure.CAN_FilterNumber=0;						//
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;		//FM1R  ��������0�Ĺ���ģʽ��
  																	//0: ��������x��2��32λ�Ĵ��������ڱ�ʶ������λģʽ�� 
																	//1: ��������x��2��32λ�Ĵ��������ڱ�ʶ���б�ģʽ��
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//FS1R ��������0(13��0)��λ��
  																	//0��������λ��Ϊ2��16λ�� 1��������λ��Ϊ����32λ��
  
    /* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */
    CAN_FilterInitStructure.CAN_FilterIdHigh=0;//(((u32)0x0021<<3)&0xFFFF0000)>>16;				//Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterIdLow=0;//(((u32)0x0021<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;//Ҫ���˵�ID��λ 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0;//0xffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;//0xffff;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;				//FFAx : ������λ������ ������ͨ����ĳ�������Ĺ��˺�
  																	//������ŵ��������FIFO�С� 0����������������FIFO0�� 1����������������FIFO1��
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;				//FACTx : ���������� �����ĳλ����1��������Ӧ�Ĺ�������ֻ�ж�FACTxλ��0��
  																	//���CAN_FMR�Ĵ�����FINITλ����1�󣬲����޸���Ӧ�Ĺ������Ĵ���
																	//x(CAN_FxR[0:1])�� 0�������������ã� 1�������������
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* CAN FIFO0 �����ж�ʹ�� */ 
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    return 0;
}
/****************************************************************************
* ��    �ƣ�void Device_Init( void )
* ��    �ܣ�initiate stm32's peripheral devices
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Device_Init( void )
{
    RCC_Configuration(); 
	GPIO_Configuration();
	NVIC_Configuration(); 
	if(1 == SysTick_Config(72000))
	{
	    //���ϵͳ��ʱ���ò��ɹ���������PE0����
	    while(1)
		{
		    GPIO_SetBits(GPIOE, GPIO_Pin_0);
			Delay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			Delay(1000000);
		}
	} 
	GPIO_ResetBits(GPIOE, GPIO_Pin_2);		 //����ָʾ�����־load1���ø�Ϊ��ͨ���õ�Ϊ�Ͽ�
	GPIO_SetBits(GPIOE, GPIO_Pin_3);		 //���ڿ��Ƽ̵������Կ�������������·load2
    Usart1_Init(); 							 //����1����232ת485ͨ��ģ�飬���Ƶ��
	Usart2_Init();							 //����2�����������������״̬
    Usart4_Init();							 //����4��ȡRFID����������Ϣ
	CAN_Configuration();					 //CAN�����ŷ�������ʵ������ͨ��
}
/****************************************************************************
* ��    �ƣ�void System_Init(void)
* ��    �ܣ�initiate the controller software environment,and config the inner data
* ��ڲ�����null
* ���ڲ�����null
* ˵    ����
* ���÷�����null 
****************************************************************************/
void System_Init(void)
{
    //�����ڴ棬����ʼ��ϵͳ����
	gSysData = (BaseSysSt*)malloc(sizeof(BaseSysSt));
//	memset(gSysData,0,sizeof(BaseSysSt));
	gComData = &(gSysData->BaseComData);
	gInnerData = &(gSysData->BaseInnerData);
	gHalData = &(gSysData->BaseHalData);
	gDownData = &(gSysData->BaseDownData);
	gUpData = &(gSysData->BaseUpData);

	//����С���ļ��β���
	gInnerData->Length = 0.53;				
	gInnerData->Width = 0.62;
	gInnerData->WheelPeri = 0.254*3.1415926*0.8;	 //ʵ��L��20cm
	gInnerData->GearScale = 40.0;
	gInnerData->MotorEncoder = 4000;
	gInnerData->MaxAcc = 0.3;
	gInnerData->MaxAccTh = 1;
	gInnerData->Period = 0.02;
	
	gInnerData->ThetaKp = 3.0;
	gInnerData->ThetaKi = 0.0000;

	gInnerData->MagDist = 0.01;

	gHalData->WheelHal[0].FlagForward = -1.0;
	gHalData->WheelHal[1].FlagForward = -1.0;
	gHalData->WheelHal[2].FlagForward = 1.0;
	gHalData->WheelHal[3].FlagForward = 1.0;

//	gHalData->WheelHal[6].FlagForward = 1;
//	gHalData->WheelHal[7].FlagForward = 1;

	gComData->FbPosx = 0;
	gComData->FbPosy = 0;
	gComData->FbPosthe = 0;

}
//��������֮��485ͨ�ŵ�crcУ�麯��
unsigned int RTU_CRC(unsigned char *data,unsigned char length)
{
     int j = 0;
	 unsigned int temp_crc = 0;
	 unsigned int reg_crc = 0xffff;
	 while(length--)
	 {
	     reg_crc ^= *data++;
		 for(j=0;j<8;j++)
		 {
		     if(reg_crc&0x01)
			 {
			     reg_crc=(reg_crc>>1)^0xa001;
			 }
			 else
			 {
			     reg_crc = reg_crc>>1;
			 }
		 }
	 }
	 temp_crc = reg_crc;
	 reg_crc = ((reg_crc<<8)&0xff00);
	 temp_crc = ((temp_crc>>8)&0x00ff);
	 reg_crc = reg_crc + temp_crc;
	 return reg_crc;
}
/****************************************************************************
* ��    �ƣ�void Drivers_Init(void)
* ��    �ܣ�initiate the drivers
* ��ڲ�����null
* ���ڲ�����null
* ˵    ����
* ���÷�����null 
****************************************************************************/
void Drivers_Init(void)
{
	int times = 5;
	int num = 0;
	int ready = 0;
	

	while(times > 0x0)
	{
	//	GPIO_SetBits(GPIOE, GPIO_Pin_0);
		LED0 = 1;
		Delay(3000000);
		LED0 = 0;
	//	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
		Delay(3000000);
		times--;
	}
	times = 30;
	while(times >0 && ready != MOTORNUM)	
	{
	  ready = 0;
		SendSDO(0,CONTROLWORD,0x80); // 		1xxx xxxx	Fault Reset
		for(num = 0;num < MOTORNUM;num++)
		{
		    switch(gHalData->WheelHal[num].PowerFlag)
			{
			    case 0:
				  SendSDO(num,IDRXPDO1,0x00000201+num);	
					Delay(20000);
					SendSDO(num,RXPDO1TYPE,1);	
					Delay(20000);
					SendSDO(num,NUMRXPDO1,2);	
					Delay(20000);
					SendSDO(num,RXPDO1OB1,0x60400010);	
					Delay(20000);
					SendSDO(num,RXPDO1OB2,0x60ff0020);	// �ٶ�ģʽ����
					Delay(20000);
//					SendSDO(num,RXPDO1OB2,0x206B0020);	// ��׼�ٶ�ģʽ����
//					Delay(20000);

					SendSDO(num,IDTXPDO1,0x40000181+num);	
					Delay(20000);
					SendSDO(num,TXPDO1TYPE,1);	
					Delay(20000);
					SendSDO(num,NUMTXPDO1,2);	
					Delay(20000);
					SendSDO(num,TXPDO1OB1,0x60410010);	
					Delay(20000);
					SendSDO(num,TXPDO1OB2,0x60640020);	
					Delay(20000);
					SendSDO(num,HEARTBEATTIME,10000);	
					Delay(20000);

					SendSDO(num,CMDTYPE,3);	  //  �������� 3 Profile Velocity Mode��1  Profile Position Mode. -2 Velocity Mode
					Delay(20000);
				  ReadSDO(num,ACTUALTYPE);
					Delay(20000);
					if(0x03 == gHalData->WheelHal[num].ObDict[ACTUALTYPE].Value[0])
					{
						gHalData->WheelHal[num].PowerFlag = 1;
					}
					break;
				case 1:
					SendSDO(num,CONTROLWORD,6);	 //0x 0000 0110  Shutdown
					Delay(20000);
					ReadSDO(num,STATUSWORD);
					Delay(20000);
					if(0x21 == (gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0] & 0x00ff))
					{
					    gHalData->WheelHal[num].PowerFlag = 2;
					}
					break;
				case 2:
					SendSDO(num,CONTROLWORD,15);	// 0x 0000 1111 Enable Operation
					Delay(20000);
					ReadSDO(num,STATUSWORD);
					Delay(20000);
					if(0x37 == (gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0] & 0x00ff))
					{
					    gHalData->WheelHal[num].PowerFlag = 3;
					}
					break;
				case 3:
				    ready++;
					break;
				default:
				    break;
			}
		}
		times--;
	}
	if(MOTORNUM == ready)
	{
	    PreOPtoOP();
		//�����ȴ��������ϵ�
		times = 0;
		while(times > 0x50)
	    {
				LED0 = 1;
//			GPIO_SetBits(GPIOE, GPIO_Pin_0);
			Delay(30000);
				LED0 = 0;
//			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			Delay(30000);
			times++;
		}
		SendSYNC();
		Delay(60000);
	}
	else
	{
		while(1)
	    {
//			GPIO_SetBits(GPIOE, GPIO_Pin_0);
			LED0 = 1;
			Delay(9000000);
//			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			LED0 = 0;
			Delay(9000000);
		}
	}

}

/****************************************************************************
* ��    �ƣ�void CalMotor(void)
* ��    �ܣ�calculate value based on motor status.
* ��ڲ�����null
* ���ڲ�����null
* ˵    ����
* ���÷�����null 
****************************************************************************/
void CalMotor(void)
{
    int num = 0;
	static int firstflag[4] = {0,};
	double actpos[4] = {0,};
	double basex = 0.0;
	double basey = 0.0;
	double baseth = 0.0;

	for(num = 0;num < MOTORNUM;num++)
	{
	     if((0x00 != gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0] & 0x08) || (0x00 == gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0]))
		 {
		      //motor error;
			  gInnerData->FatalErr |= 0x01;
		 }
		 
		 //gHalData->WheelHal[num].FbVel = *(int*)(&gHalData->WheelHal[num].ObDict[ACTUALVEL].Value[0]);
		 gHalData->WheelHal[num].FbVel = (*(int*)(&gHalData->WheelHal[num].ObDict[ACTUALVEL].Value[0])) * gInnerData->WheelPeri /(gInnerData->GearScale * 60  * gHalData->WheelHal[num].FlagForward);
         gHalData->WheelHal[num].FbPos = (double)(*((int *)&(gHalData->WheelHal[num].ObDict[ACTUALPOS].Value[0]))) * gInnerData->WheelPeri /(gInnerData->GearScale * gInnerData->MotorEncoder * gHalData->WheelHal[num].FlagForward);
		 if(0 == firstflag[num])
		 {
		     gHalData->WheelHal[num].LastFbpos	= gHalData->WheelHal[num].FbPos;
			 firstflag[num] = 1;
		 }
		 actpos[num] = (gHalData->WheelHal[num].FbPos - gHalData->WheelHal[num].LastFbpos);
		 gHalData->WheelHal[num].LastFbpos	= gHalData->WheelHal[num].FbPos;
	}

	//gComData->FbVx = (gHalData->WheelHal[0].FbVel + gHalData->WheelHal[1].FbVel + gHalData->WheelHal[2].FbVel + gHalData->WheelHal[3].FbVel)/4.0;
	//gComData->FbVy = (-gHalData->WheelHal[0].FbVel + gHalData->WheelHal[1].FbVel + gHalData->WheelHal[2].FbVel - gHalData->WheelHal[3].FbVel)/4.0;
	//gComData->FbVthe = (-gHalData->WheelHal[0].FbVel + gHalData->WheelHal[1].FbVel - gHalData->WheelHal[2].FbVel + gHalData->WheelHal[3].FbVel)/(2*(gInnerData->Length + gInnerData->Width));
    basex = (actpos[0] + actpos[1] + actpos[2] + actpos[3])/4.0;
	basey = (-actpos[0] + actpos[1] + actpos[2] - actpos[3])/4.0;
	baseth = (-actpos[0] + actpos[1] - actpos[2] + actpos[3])/(2*(gInnerData->Length + gInnerData->Width));
	gComData->FbPosx += basex*cos(gComData->FbPosthe) - basey*sin(gComData->FbPosthe);
	gComData->FbPosy += basex*sin(gComData->FbPosthe) + basey*cos(gComData->FbPosthe);
	gComData->FbPosthe += baseth;
	gComData->FbVx = (basex*cos(gComData->FbPosthe)- basey*sin(gComData->FbPosthe))/0.02;
	gComData->FbVy = (basex*sin(gComData->FbPosthe)+ basey*cos(gComData->FbPosthe))/0.02;
	gComData->FbVthe = baseth/0.02;
	//gComData->FbPosx += (actpos[0] + actpos[1] + actpos[2] + actpos[3])/4.0;
	//gComData->FbPosy += (-actpos[0] + actpos[1] + actpos[2] - actpos[3])/4.0;
	//gComData->FbPosthe += (-actpos[0] + actpos[1] - actpos[2] + actpos[3])/(2*(gInnerData->Length + gInnerData->Width));
}

/****************************************************************************
* ��    �ƣ�void Obstacle(void)
* ��    �ܣ�calculate value based on motor status.
* ��ڲ�����null
* ���ڲ�����null
* ˵    ����
* ���÷�����null 
****************************************************************************/
void Obstacle(void)
{
    return;
}

/****************************************************************************
* ��    �ƣ�void MagIO_Check(void)
* ��    �ܣ�
* ��ڲ�����gHalData->forwardIO[8],gHalData->leftIO[8]
* ���ڲ�����gInnerData->MagFIOX, gInnerData->OnMag, gInnerData->MagLIOX
* ˵    ��������16�������ȷ����������ڴ�����λ��
* ���÷������� 
****************************************************************************/
void MagIO_Check(void)
{
	return;
}
/****************************************************************************
* ��    �ƣ�void Status_Handle(void)
* ��    �ܣ�
* ��ڲ�����gHalData->Fbobstacle[8],gHalData->Station,gHalData->Pause,gHalData->Start,
* ���ڲ�����gInnerData->ScaleObst,gInnerData->Station,gHalData->InPosition,gComData->TurnFinish
* ˵    ����
* ���÷������� 
****************************************************************************/

void Status_Handle(void)
{	
    return;
}	
/****************************************************************************
* ��    �ƣ�void Command_Handle(void)
* ��    �ܣ�calculate the wheels' speed based on cmd velocity 
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Command_Handle(void)
{
	char temp = 0;        //////8λ
	int i = 0;            //////32λ
	double caltmp = 0.0;  ///////32λ
//	double tmpx = 0.0;
//	double tmpy = 0.0;
	double delx = 0.0;
	double dely = 0.0;
	int j = 0;
	int finishcount = 0;
	static int segstate = 0;
	static int finish[10] = {0,};
	static int finishnum = 0;
	static 	char flagssss=0;
	//������յ��ϲ�����
	gUpData->FbStatus = 0;
	if(1 == gInnerData->NeedUp)
	{
	  
		gUpData->FbStatus = 1;
	    //�Դ��ڽ��յ����ϲ����ݽ���CRCУ��
	    for(i=0;i<32;i++)
		{
		    temp += *(((char*)gDownData) + i);
		}
		if(temp == gDownData->CmdCRC)
		{


			flagssss++;
			if(flagssss%2 == 0)
			{					
				GPIO_ResetBits(GPIOE, GPIO_Pin_1);
			}
			else
			{
				GPIO_SetBits(GPIOE, GPIO_Pin_1);
			}	
			//У��ͨ���󣬸����������ͣ�������������
			gUpData->FbStatus = 2;
			if(gDownData->Cmd == 0x73)
		    {
				gInnerData->State = 0;
			    //С�˶��뷽ʽ�����ݵ�λ��Ӧ�ڴ��λ
				//����x������ٶȣ���λ����ÿ��
				gComData->CmdVx = (double)(gDownData->Angle)/1000.0;
				//����y������ٶȣ���λ����ÿ��
				gComData->CmdVy = (double)(gDownData->distx)/1000.0;
				//����theta������ٶȣ���λ����ÿ��
				gComData->CmdVthe = (double)(gDownData->disty)/1000.0;
			}
			else if(gDownData->Cmd == 0x72)
			{
			    gUpData->FbStatus = 3;
			    //�����ֿ�ģʽ�������Զ����ƽ׶�
				//���Զ�ģʽ�£����ǵ�kivaС���ƶ���ʽֻ��ǰ1����2��ԭ����ʱ����ת3��ԭ��˳ʱ����ת4��ֹͣ5��ԭ���������6�������
				//������ӽ����˹�·��㣬�����Ƿ�����ɨ��Ķ�ά���־�������Ƿ��޸ķ���λ�á�
				//tmpx = (int)(gComData->FbPosx+0.5);
				//tmpy = (int)(gComData->FbPosy+0.5);
				if(1 == gDownData->NewCode)//�ѵ���ά��
				{
					gComData->QrCodeX = gDownData->qrCodeX;
					gComData->QrCodeY = gDownData->qrCodeY;
					gComData->FbPosthe = gDownData->Angle*3.1415926/18000.0;
					gComData->FbPosx = gDownData->distx*0.01*0.021075*cos(gComData->FbPosthe) - gDownData->disty*0.01*0.021075*sin(gComData->FbPosthe) + gComData->QrCodeX;
					gComData->FbPosy = gDownData->distx*0.01*0.021075*sin(gComData->FbPosthe) + gDownData->disty*0.01*0.021075*cos(gComData->FbPosthe) + gComData->QrCodeY;
					
				}
				
				//��������ϵ���壬�������Ŀ��λ�òֵ������ṹ��					
				gComData->goalx = gDownData->goalx;//��ֵ
				gComData->goaly = gDownData->goaly;
				
				//����state�˶����¾��ۼƱ����������Ƿ���������С���˶���㡣����������ڲ�״̬λ��
				if(gDownData->State != segstate)
				{
					gComData->startx  = (int)(gComData->FbPosx+0.5);
					gComData->starty  = (int)(gComData->FbPosy+0.5);
					gComData->State = 0;
					gComData->Finished = 0;
					segstate = gDownData->State;
				}
				gUpData->FbErr =  segstate;
				//���㱾�˶�����·�̣�������Ŀ���x,yλ�ò����С������
				gComData->goall = (fabs(gComData->goalx) + fabs(gComData->goaly) )* 1.0;
				if(0 == gDownData->goalx * gDownData->goaly)
				{
				    if(gDownData->goalx > 0 && 0 == gDownData->goaly)
					{
					    gComData->goalth = 0;
					}
					else if(gDownData->goalx < 0 && 0 == gDownData->goaly)
					{
					    gComData->goalth = -3.1415926;
					}
					else if(0 == gDownData->goalx && gDownData->goaly > 0)
					{
					    gComData->goalth = 3.1415926/2.0;
					}
					else if(0 == gDownData->goalx && gDownData->goaly < 0)
					{
					    gComData->goalth = -3.1415926/2.0;
					}
				}
			}
			else if(gDownData->Cmd == 0x71)
			{
			    //��ʼ���׶ε�������ݶ�ά��ͼ����Ϣ����С��ȫ������
				if(1 == gDownData->NewCode)
				{
				    gComData->FbPosthe = gDownData->Angle*3.1415926/18000.0;
					gComData->FbPosx = gDownData->distx*0.01*0.02125*cos(gComData->FbPosthe) - gDownData->disty*0.01*0.02125*sin(gComData->FbPosthe) + gDownData->goalx;
					gComData->FbPosy = gDownData->distx*0.01*0.02125*sin(gComData->FbPosthe) + gDownData->disty*0.01*0.02125*cos(gComData->FbPosthe) + gDownData->goaly;
				}
			}
		}//if(temp == gDownData->CmdCRC)...
		//gInnerData->NeedUp = 2;
		gInnerData->NeedUp = 0;
	}
	//���ݿ������ڲ�״̬λ������С����ת����ֱ��
	if(0 == gComData->State)
	{
		if(fabs(gComData->goalth - gComData->FbPosthe) >2.35&&fabs(gComData->goalth - gComData->FbPosthe) <3.92 )
		{
			gComData->State = 6;
		}
		else
		{	
			gComData->State = 7;
		}
	}
	//rotate 
	if(gComData->State==6||gComData->State==7)
	{
		delx = gComData->FbPosx - (int)(gComData->FbPosx+0.5);
		dely = gComData->FbPosy - (int)(gComData->FbPosy+0.5);
		gComData->CmdVx = -(delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) * 5;
		gComData->CmdVy = -(dely*cos(gComData->FbPosthe) - delx*sin(gComData->FbPosthe)) * 5;
		if(gComData->FbPosthe >= 3.1415926)
		{
		    gComData->FbPosthe -= 3.1415926*2.0;
		}
		if(gComData->State == 7)
		{
			if(fabs(gComData->goalth - gComData->FbPosthe) < 3.1415926)
			{
		    	gComData->CmdVthe = (gComData->goalth - gComData->FbPosthe)*0.95;
			}
			else if(gComData->goalth > gComData->FbPosthe)
			{
		   		gComData->CmdVthe = (gComData->goalth - (gComData->FbPosthe + 3.1415926*2.0))*0.95;
				//gComData->CmdVthe = (3.1415926*2.0 - gComData->goalth + gComData->FbPosthe)*0.95;
			}
			else   //�߼���Ҫ����
			{
		   		 gComData->CmdVthe = (3.1415926 + gComData->goalth + gComData->FbPosthe)*0.95;
			}
		}
		if(gComData->State == 6)
		{
			if(fabs(gComData->goalth - gComData->FbPosthe) < 3.1415926)
			{
				gComData->CmdVthe = (gComData->FbPosthe-gComData->goalth)*0.95;
			}
			else
			{
				gComData->CmdVthe = (gComData->goalth-gComData->FbPosthe)*0.95;
			}
		}
		//���С����ת��Ŀ�곯�������ֱ�н׶�	 ���ڷ���xyû�е��ںõĻ���
		if(fabs(gComData->goalth - gComData->FbPosthe) < 0.01 || fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0)< 0.01)
		{
			gComData->State = 1;
			gComData->CmdVx = 0.0;///0.4//////////////////////////
		}
		//���С����ת��Ŀ�곯�����������ֱ�н׶�
		else if((fabs(gComData->goalth - gComData->FbPosthe) >3.13&&fabs(gComData->goalth - gComData->FbPosthe) <3.152)||(fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0) >3.13&&fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0) <3.152))
		{
			gComData->State = 5;
			gComData->CmdVx = 0.0;///0.4//////////////////////////
		}
	}
	if(gComData->State==1||gComData->State==5)
	{
	    //line driver
		//���ݷ���λ�ú�Ŀ��λ�ò�ֵ��Pi�Ĵ�С��������ת�ٶȡ�
		if(gComData->FbPosthe >= 3.1415926)
		{
		    gComData->FbPosthe -= 3.1415926*2.0;
		}
		if(1 == gComData->State)
		{
			if(fabs(gComData->goalth - gComData->FbPosthe) < 3.1415926)
			{
		    	gComData->CmdVthe = (gComData->goalth - gComData->FbPosthe)*0.95;
			}
			else if(gComData->goalth > gComData->FbPosthe)
			{
		    	gComData->CmdVthe = (gComData->goalth - (gComData->FbPosthe + 3.1415926*2.0))*0.95;
			}
			else
			{	
				gComData->CmdVthe = (3.1415926*2.0 + gComData->goalth - gComData->FbPosthe)*0.95;
		    //gComData->CmdVthe = (3.1415926*2.0 - gComData->goalth + gComData->FbPosthe)*0.95;
			}
		}
		if( 5== gComData->State)
		{
			if(fabs(gComData->goalth)<2.35619&&fabs(gComData->goalth)>0.78539)
			{
				gComData->CmdVthe = (-gComData->FbPosthe-gComData->goalth)*0.95;
			}
			else if(gComData->goalth<0)
			{
				gComData->CmdVthe = (gComData->goalth+3.1415926- gComData->FbPosthe)*0.95;
			}
			else
			{
				if(gComData->FbPosthe>0)
				{
					gComData->CmdVthe = (gComData->goalth+3.1415926- gComData->FbPosthe)*0.95;
	
				}
				else
				{
					gComData->CmdVthe = (gComData->goalth-3.1415926- gComData->FbPosthe)*0.95;
				}	
			}
		}
		//����ƫ���������ת������ϵ������С����������ϵ��y����������ٶ�
		delx = gComData->FbPosx - (int)(gComData->FbPosx+0.5);
		dely = gComData->FbPosy - (int)(gComData->FbPosy+0.5);
		
		gComData->CmdVy = -(dely*cos(gComData->FbPosthe) - delx*sin(gComData->FbPosthe)) * 1.8;
		//���ݸ���·�̸���С��������0.5m/s���ٶȣ�
		if(0.5*0.5/gInnerData->MaxAcc < gComData->goall)
		{
			gComData->CmdVx = 0.2;///0.4/////////////////////////
		}
		else
		{
		    gComData->CmdVx = sqrt(gComData->goall * gInnerData->MaxAcc)*0.5;//////////////
		}
		//���ݷ���λ�ü���С��Ŀǰ�Ѿ��н���·��
		caltmp = (gComData->FbPosx - gComData->startx)*(gComData->FbPosx - gComData->startx);
		caltmp += (gComData->FbPosy - gComData->starty)*(gComData->FbPosy - gComData->starty);
		gComData->progressl = sqrt(caltmp);
		//����ʣ��·���ж��Ƿ��٣�����Ŀ��㸽������PID���ڣ�ʹ֮׼ȷ����Ŀ���
		caltmp = gComData->CmdVx * gComData->CmdVx / (gInnerData->MaxAcc*2.0);
    if(gComData->progressl >= gComData->goall - caltmp - 0.05)
		{
		    gComData->CmdVx = 0.0;
		}
		if(gComData->State==1)
		{
			if(fabs(gComData->progressl - gComData->goall) < 0.05)
			{
		    	caltmp = -(delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) * 5;
				gComData->CmdVx += caltmp; 
			}
			else if(fabs(gComData->progressl - gComData->goall) < 0.15)
			{
		   	 	caltmp = -(delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) * 1;
				gComData->CmdVx += caltmp; 
			}
		}
		if(gComData->State==5)
		{
			if(fabs(gComData->progressl - gComData->goall) < 0.05)
			{
				caltmp = (delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) * 5;
				gComData->CmdVx += caltmp; 
			}
			else if(fabs(gComData->progressl - gComData->goall) < 0.15)
			{
				caltmp = (delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) * 1;
				gComData->CmdVx += caltmp; 
			}
		}
		//����ȫ��λ��ƫ��ж��Ƿ񵽴�Ŀ��㣬��������������˶�����ɱ�־��������Ϊ0
		if(fabs(delx*cos(gComData->FbPosthe) + dely*sin(gComData->FbPosthe)) <= 0.005 && fabs(gComData->progressl - gComData->goall) < 0.05)
		{
		    finish[finishnum] = 1;
		}
		else
		{
			finish[finishnum] = 0;
		}
		finishnum = (finishnum+1) % 10;
		for(j = 0;j < 10;j++)
		{
		    finishcount += finish[j];
		}
		if(10 == finishcount && 1 == gDownData->NewCode)
		{
		    gComData->Finished = 1;
		}
		else
		{
		    gComData->Finished = 0;
		}
		
	}	  
	
	
	gInnerData->ScaleObst = 1.0;
	//��С��y�����ٶȽ�������
	if(gComData->CmdVy > 0.2)////0.3
	{
		gComData->CmdVy = 0.2;////
    }
	else if(gComData->CmdVy < -0.2)////
	{
		gComData->CmdVy = -0.2	;/////
    }
	//��th�����ٶȽ��мӼ��ٿ���
	if(gComData->CmdVthe > gComData->LastCmdVthe + gInnerData->MaxAccTh * gInnerData->Period)
	{
	    gComData->CmdVthe = gComData->LastCmdVthe + gInnerData->MaxAccTh * gInnerData->Period;
	}
	else if(gComData->CmdVthe < gComData->LastCmdVthe - gInnerData->MaxAccTh * gInnerData->Period)
	{
	    gComData->CmdVthe = gComData->LastCmdVthe - gInnerData->MaxAccTh * gInnerData->Period;
	}
	if(gComData->CmdVthe > 0.4)//0.6
	{
	    gComData->CmdVthe = 0.4;///
	}
	else if(gComData->CmdVthe < -0.4)///
	{
	    gComData->CmdVthe = -0.4;////
	}
	gComData->LastCmdVthe	= gComData->CmdVthe;
	//��x�����ٶȽ��мӼ��ٿ���
	if(gComData->CmdVx > gComData->LastCmdVx + gInnerData->MaxAcc * gInnerData->Period)
	{
	    gComData->CmdVx = gComData->LastCmdVx + gInnerData->MaxAcc * gInnerData->Period;
	}
	else if(gComData->CmdVx < gComData->LastCmdVx - gInnerData->MaxAcc * gInnerData->Period)
	{
	    gComData->CmdVx = gComData->LastCmdVx - gInnerData->MaxAcc * gInnerData->Period;
	}
	gComData->LastCmdVx	= gComData->CmdVx;

	//�����ĸ����ӵ��ٶ�,��λ��m/s
	if(1== gComData->State)
	{
		gHalData->WheelHal[0].CmdVel = (gComData->CmdVx - gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[1].CmdVel = (gComData->CmdVx + gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[2].CmdVel = (gComData->CmdVx + gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[3].CmdVel = (gComData->CmdVx - gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;  
	}
	else if(5== gComData->State)
	{
		gHalData->WheelHal[0].CmdVel = (-gComData->CmdVx - gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[1].CmdVel = (-gComData->CmdVx + gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[2].CmdVel = (-gComData->CmdVx + gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[3].CmdVel = (-gComData->CmdVx - gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
	}		
	else
	{
		gHalData->WheelHal[0].CmdVel = (gComData->CmdVx - gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[1].CmdVel = (gComData->CmdVx + gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[2].CmdVel = (gComData->CmdVx + gComData->CmdVy - gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
		gHalData->WheelHal[3].CmdVel = (gComData->CmdVx - gComData->CmdVy + gComData->CmdVthe * (gInnerData->Length + gInnerData->Width)/2)*gInnerData->ScaleObst;
	}
}
/****************************************************************************
* ��    �ƣ�void Data_Send(void)
* ��    �ܣ�calculate the wheels' speed based on cmd velocity 
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Data_Send(void)
{
	int num = 0;

//	Delay(20000);
	for(num = 0; num < MOTORNUM; num++)
	{
	  if(0 == gInnerData->FatalErr)
		{
	        gHalData->WheelHal[num].CmdMotorvel = (int)(gHalData->WheelHal[num].CmdVel * gInnerData->GearScale * 60  * gHalData->WheelHal[num].FlagForward / gInnerData->WheelPeri);
			 
			//gHalData->WheelHal[num].CmdMotorvel = vel * gHalData->WheelHal[num].FlagForward;
		}
		else
		{
			gHalData->WheelHal[num].CmdMotorvel = 0;
//			GPIO_SetBits(GPIOE, GPIO_Pin_2);
		}
		SendPDO(num);
		gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0] = 0;
		Delay(20000); // ��һ����ʱ���������滻������־λ��������MOTORNUMʱ���㣬����ͬ���ź�
	}
}
/****************************************************************************
* ��    �ƣ�void Status_UpLoad(void)
* ��    �ܣ�send the controller's status to upper level
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void Status_UpLoad(void)
{
//	int i = 0;
//	char crc = 0;
//	static char num = 0;
   	
	SendSYNC();
//	Delay(20000);

	//�������CRCУ����ȷ�������ϲ���Ҫ���ݣ�������ݽ���У����ͳ�ȥ
//	num++;
	
//	gUpData->FbVx = gComData->startx;
//	gUpData->FbVy = gComData->starty;
//	gUpData->FbVthe = gComData->goalth*1000;
//	//gUpData->FbVx = gComData->FbVx*1000;
//	//gUpData->FbVy = gComData->FbVy*1000;
//	//gUpData->FbVthe = gComData->FbVthe*1000;

//	gUpData->Fbposx = gComData->FbPosx*1000;
//	gUpData->Fbposy = gComData->FbPosy*1000;
//	gUpData->Fbposth = gComData->FbPosthe*1000;

//	//gUpData->FbErr = (char)(gComData->CmdStation);
//	//gUpData->FbStatus = num;
//	gUpData->FbFlag = gComData->Finished;
//	gUpData->FbBeating = gDownData->CmdBeating+1;
//	//if(2 == gInnerData->NeedUp)
//	{
//		//���Ϳ�ʼ��־
//		SendChar(0x55);
//		for(i = 0;i < 28;i++)
//		{
//		    SendChar(*(((char*)&(gUpData->FbVx))+i));
//			crc += *(((char*)&(gUpData->FbVx))+i);
//		}
//		gUpData->FbCRC = crc;
////		SendChar(crc);
////		SendChar(0xaa);
//		//gInnerData->NeedUp = 0;
//	}
}

