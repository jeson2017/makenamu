/****************************************************************************
* Copyright (C), 2013 ���ݺ��ϻ��������޹�˾         
*
* �ļ���: movebase.c
* ���ݼ���:	
*          
*	�ܶ���STM32������������ƶ�ƽ̨�ײ�����������ܲ���ʵ��
	����MDK�汾��        3.8
	���ڹٷ������汾�� 3.5
	ʵ�ֶԸ������ܿ�����ú�ʵ�֣�ϵͳ��ʼ����
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2013-2-5   �Ÿ�    �������ļ�
*
***************************************************************************/
#include "movebase.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
#define CAN_BAUDRATE (500)

Buletooth_Motor_param buletooth_motor_para;


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
  
    NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;			       //�ж��ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //�������ȼ�0  ��Χ��0��1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			       //�����ȼ�0	��Χ��0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	  

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
* ��    �ƣ�bool CAN_Configuration(void)
* ��    �ܣ��ж�ģʽ�µ�CAN�շ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
bool CAN_Configuration(void)
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

    return (bool)0;
}
/****************************************************************************
* ��    �ƣ�void ADC_Configuration(void)
* ��    �ܣ�ADC ���ú���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/ 
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

    //����ADģ������˿�Ϊ���� 1·AD ����ͨ��
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);

  	/* DMA channel1 configuration ----------------------------------------------*/
	//ʹ��DMA
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			            //DMAͨ��1�ĵ�ַ 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	            //DMA���͵�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					            //���ͷ���
	DMA_InitStructure.DMA_BufferSize = 1;								            //�����ڴ��С��100��16λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				            //�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//ADC1ת����������16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				//���͵�Ŀ�ĵ�ַ��16λ���
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
	/* ����DMA1ͨ��1��������ж� */
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);


	//ʹ��DMAͨ��1
	DMA_Cmd(DMA1_Channel1, ENABLE); 
  
  
	//ADC����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;               //�涨��˳����й���ת����ADCͨ������Ŀ�������Ŀ��ȡֵ��Χ��1��16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [����ģʽͨ������]*/ 

	//ADC1 ����ͨ������
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);	  //ͨ��11����ʱ�� 55.5����
  	

	//ʹ��ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);
	//ʹ��ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// ��ʼ��ADC1У׼�Ĵ���
	ADC_ResetCalibration(ADC1);
	//���ADC1У׼�Ĵ�����ʼ���Ƿ����
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//��ʼУ׼ADC1
	ADC_StartCalibration(ADC1);
	//����Ƿ����У׼
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1ת������
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 
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
//Write character to Serial Port 
int SendChar (char ch)  
{                
    USART_SendData(USART2, (unsigned char) ch);
    while (!(USART2->SR & USART_FLAG_TXE));
    return (ch);
}

void SentString(char *p)
{
	while(*p !=NULL)
	{
		SendChar(*p);
		p++;
	}
	return;
}

int TransUp (int ch)  
{                
    USART_SendData(USART1, (unsigned char) ch);
    while (!(USART1->SR & USART_FLAG_TXE));
    return (ch);
}
/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART2, (u8) ch);

    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    return ch;
}

/***********************************************���ݺ��ϻ����˿Ƽ����޹�˾************************************************************/
