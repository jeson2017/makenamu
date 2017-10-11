/****************************************************************************
* Copyright (C), 2013 常州汉迪机器人有限公司         
*
* 文件名: movebase.c
* 内容简述:	
*          
*	奋斗版STM32开发板基础的移动平台底层控制器各功能部分实现
	基于MDK版本：        3.8
	基于官方外设库版本： 3.5
	实现对各个功能块的配置和实现：系统初始化，
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2013-2-5   张干    创建该文件
*
***************************************************************************/
#include "movebase.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
#define CAN_BAUDRATE (500)

Buletooth_Motor_param buletooth_motor_para;


/****************************************************************************
* 名    称：void Delay(__IO uint32_t nCount)
* 功    能：累积延时函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

/****************************************************************************
* 名    称：void NVIC_Configuration(void)
* 功    能：中断源配置函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 优先级组1 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	       //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
    NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;			       //中断中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //抢先优先级0  范围：0或1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			       //子优先级0	范围：0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	  

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			       //串口1中断中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢先优先级0  范围：0或1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //子优先级0	范围：0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			       //串口2中断中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //抢先优先级0  范围：0或1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			       //子优先级1	范围：0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;			       //串口2中断中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢先优先级0  范围：0或1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			       //子优先级1	范围：0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			       // 定时器6中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		   //抢先优先级0  范围：0或1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;			       //子优先级1	范围：0-7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/****************************************************************************
* 名    称：void GPIO_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_1;				     //LED1 V6配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	//用于磁条导航的磁条检测。共16个IO开，输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//用于控制暂停、启动和到位检测
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14;				     //配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    //用于心跳反应
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;       
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;     // 上拉输入   	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3  | GPIO_Pin_5  | GPIO_Pin_7;				     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
/****************************************************************************
* 名    称：void RCC_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void RCC_Configuration(void)
{
    SystemInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			   //复用功能使能
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
  					  	  | RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO , ENABLE);
}
/****************************************************************************
* 名    称：bool CAN_Configuration(void)
* 功    能：中断模式下的CAN收发
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
bool CAN_Configuration(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;  
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);		     //端口复用为CAN1   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	                 //PB8:CAN-RX 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			     //输入上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					 //PB9:CAN-TX 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			 //复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* CAN寄存器初始化 */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN单元初始化 */
    CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  自动唤醒模式
    //CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  0-自动重传   1-报文只传一次
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  0-自动重传   1-报文只传一次
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  0-溢出时新报文会覆盖原有报文  1-溢出时，新报文丢弃
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    //CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 0-优先级取决于报文标示符 1-优先级取决于发送请求的顺序
    //CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;	   //BTR-SILM/LBKM   CAN环回模式
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
    CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;		   //BTR-TS1 时间段1 占用了2个时间单元
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 时间段2 占用了3个时间单元
  
#if CAN_BAUDRATE == 1000 /* 1MBps */
    CAN_InitStructure.CAN_Prescaler =6;			   //BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+2+3)/6=1Mbps
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

    /* CAN过滤器初始化 */
    CAN_FilterInitStructure.CAN_FilterNumber=0;						//
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;		//FM1R  过滤器组0的工作模式。
  																	//0: 过滤器组x的2个32位寄存器工作在标识符屏蔽位模式； 
																	//1: 过滤器组x的2个32位寄存器工作在标识符列表模式。
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//FS1R 过滤器组0(13～0)的位宽。
  																	//0：过滤器位宽为2个16位； 1：过滤器位宽为单个32位。
  
    /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
    CAN_FilterInitStructure.CAN_FilterIdHigh=0;//(((u32)0x0021<<3)&0xFFFF0000)>>16;				//要过滤的ID高位 
    CAN_FilterInitStructure.CAN_FilterIdLow=0;//(((u32)0x0021<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;//要过滤的ID低位 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0;//0xffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;//0xffff;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;				//FFAx : 过滤器位宽设置 报文在通过了某过滤器的过滤后，
  																	//将被存放到其关联的FIFO中。 0：过滤器被关联到FIFO0； 1：过滤器被关联到FIFO1。
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;				//FACTx : 过滤器激活 软件对某位设置1来激活相应的过滤器。只有对FACTx位清0，
  																	//或对CAN_FMR寄存器的FINIT位设置1后，才能修改相应的过滤器寄存器
																	//x(CAN_FxR[0:1])。 0：过滤器被禁用； 1：过滤器被激活。
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* CAN FIFO0 接收中断使能 */ 
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    return (bool)0;
}
/****************************************************************************
* 名    称：void ADC_Configuration(void)
* 功    能：ADC 配置函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/ 
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

    //设置AD模拟输入端口为输入 1路AD 规则通道
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);

  	/* DMA channel1 configuration ----------------------------------------------*/
	//使能DMA
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			            //DMA通道1的地址 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	            //DMA传送地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					            //传送方向
	DMA_InitStructure.DMA_BufferSize = 1;								            //传送内存大小，100个16位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				            //传送内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//ADC1转换的数据是16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;				//传送的目的地址是16位宽度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
	/* 允许DMA1通道1传输结束中断 */
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);


	//使能DMA通道1
	DMA_Cmd(DMA1_Channel1, ENABLE); 
  
  
	//ADC配置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;               //规定了顺序进行规则转换的ADC通道的数目。这个数目的取值范围是1到16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [规则模式通道配置]*/ 

	//ADC1 规则通道配置
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);	  //通道11采样时间 55.5周期
  	

	//使能ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);
	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// 初始化ADC1校准寄存器
	ADC_ResetCalibration(ADC1);
	//检测ADC1校准寄存器初始化是否完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//开始校准ADC1
	ADC_StartCalibration(ADC1);
	//检测是否完成校准
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1转换启动
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 
}
/****************************************************************************
* 名    称：void Usart1_Init(void)
* 功    能：串口1初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：通过232转485模块控制电机
* 调用方法：无 
****************************************************************************/
void Usart1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
 
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //使能串口1时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A端口 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //复用开漏输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A端口 

    USART_InitStructure.USART_BaudRate = 9600;						//速率9600bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);							//配置串口参数函数   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//接收中断使能
    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);	
  
}
/****************************************************************************
* 名    称：void Usart2_Init(void)
* 功    能：串口2初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：用于蓝牙通信
* 调用方法：无 
****************************************************************************/
void Usart2_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
 
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE);	                           //使能串口2时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	         		                           //USART2 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		                               //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		                               //A端口 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	         	                               //USART2 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	                           //复用开漏输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	                               //A端口 

    USART_InitStructure.USART_BaudRate = 115200;						                   //速率115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		                       //数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			                       //停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				                       //无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					       //收发模式

    /* Configure USART1 */
    USART_Init(USART2, &USART_InitStructure);							                   //配置串口参数函数   
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);					                       //接收中断使能
	USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    /* Enable the USART1 */
    USART_Cmd(USART2, ENABLE);	
}


/****************************************************************************
* 名    称：void Usart4_Init(void)
* 功    能：串口1初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：用于IC卡读取
* 调用方法：无 
****************************************************************************/
void Usart4_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
 
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4 , ENABLE);	                           //使能串口2时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         		                           //USART2 TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		                               //复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);		    		                               //A端口 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	         	                               //USART2 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	                           //复用开漏输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);		         	                               //A端口 

    USART_InitStructure.USART_BaudRate = 9600;						                   //速率115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		                       //数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			                       //停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				                       //无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					       //收发模式

    /* Configure USART1 */
    USART_Init(UART4, &USART_InitStructure);							                   //配置串口参数函数   
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);					                       //接收中断使能 
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

/***********************************************常州汉迪机器人科技有限公司************************************************************/
