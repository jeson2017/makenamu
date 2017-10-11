/****************************************************************************
* Copyright (C), 2013 汉迪机器人
*
* 文件名: system.c
* 内容简述:
*       
*       
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2013-6-6   张干  创建该文件
****************************************************************************/
#include "movebase.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f10x_can.h"

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
* 名    称：void Device_Init( void )
* 功    能：initiate stm32's peripheral devices
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void Device_Init( void )
{
    RCC_Configuration(); 
	GPIO_Configuration();
	NVIC_Configuration(); 
	if(1 == SysTick_Config(72000))
	{
	    //如果系统定时配置不成功，则设置PE0闪灯
	    while(1)
		{
		    GPIO_SetBits(GPIOE, GPIO_Pin_0);
			Delay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			Delay(1000000);
		}
	} 
	GPIO_ResetBits(GPIOE, GPIO_Pin_2);		 //用于指示错误标志load1，置高为接通，置低为断开
	GPIO_SetBits(GPIOE, GPIO_Pin_3);		 //用于控制继电器，以控制驱动器主回路load2
    Usart1_Init(); 							 //串口1利用232转485通信模块，控制电机
	Usart2_Init();							 //串口2利用蓝牙接收命令反馈状态
    Usart4_Init();							 //串口4读取RFID读卡器的信息
	CAN_Configuration();					 //CAN口与伺服驱动器实现数据通信
}
/****************************************************************************
* 名    称：void System_Init(void)
* 功    能：initiate the controller software environment,and config the inner data
* 入口参数：null
* 出口参数：null
* 说    明：
* 调用方法：null 
****************************************************************************/
void System_Init(void)
{
    //分配内存，并初始化系统参数
	gSysData = (BaseSysSt*)malloc(sizeof(BaseSysSt));
	memset(gSysData,0,sizeof(BaseSysSt));
	gComData = &(gSysData->BaseComData);
	gInnerData = &(gSysData->BaseInnerData);
	gHalData = &(gSysData->BaseHalData);
	gDownData = &(gSysData->BaseDownData);
	gUpData = &(gSysData->BaseUpData);

	//设置小车的几何参数
	gInnerData->Length = 0.53;				
	gInnerData->Width = 0.62;
	gInnerData->WheelPeri = 0.254*3.1415926*0.8;	 //实际L：20cm
	gInnerData->GearScale = 40.0;
	gInnerData->MotorEncoder = 4000;
	gInnerData->MaxAcc = 0.3;
	gInnerData->MaxAccTh = 1;
	gInnerData->Period = 0.02;
	
	gInnerData->ThetaKp = 3.0;
	gInnerData->ThetaKi = 0.0000;

	gInnerData->MagDist = 0.01;

	gHalData->WheelHal[0].FlagForward = 1;
	gHalData->WheelHal[1].FlagForward = 1;
	gHalData->WheelHal[2].FlagForward = 1;
	gHalData->WheelHal[3].FlagForward = 1;
	gHalData->WheelHal[4].FlagForward = 1;
	gHalData->WheelHal[5].FlagForward = 1;
//	gHalData->WheelHal[6].FlagForward = 1;
//	gHalData->WheelHal[7].FlagForward = 1;

	gComData->FbPosx = 0;
	gComData->FbPosy = 0;
	gComData->FbPosthe = 0;

}
//与驱动器之间485通信的crc校验函数
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
* 名    称：void Drivers_Init(void)
* 功    能：initiate the drivers
* 入口参数：null
* 出口参数：null
* 说    明：
* 调用方法：null 
****************************************************************************/
void Drivers_Init(void)
{
	int times = 10;
	int num = 0;
	int ready = 0;
	

	while(times > 0x0)
	{
		GPIO_SetBits(GPIOE, GPIO_Pin_0);
		Delay(3000000);
		GPIO_ResetBits(GPIOE, GPIO_Pin_0);
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
					SendSDO(num,RXPDO1OB2,0x60ff0020);	// 速度模式下用
					Delay(20000);
//					SendSDO(num,RXPDO1OB2,0x206B0020);	// 标准速度模式下用
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

					SendSDO(num,CMDTYPE,3);	  //  命令类型 3 Profile Velocity Mode。1  Profile Position Mode. -2 Velocity Mode
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
		//快闪等待驱动器上电
		times = 0;
		while(times > 0x50)
	    {
			GPIO_SetBits(GPIOE, GPIO_Pin_0);
			Delay(30000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
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
			GPIO_SetBits(GPIOE, GPIO_Pin_0);
			Delay(9000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			Delay(9000000);
		}
	}

}

/****************************************************************************
* 名    称：void CalMotor(void)
* 功    能：calculate value based on motor status.
* 入口参数：null
* 出口参数：null
* 说    明：
* 调用方法：null 
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
* 名    称：void Obstacle(void)
* 功    能：calculate value based on motor status.
* 入口参数：null
* 出口参数：null
* 说    明：
* 调用方法：null 
****************************************************************************/
void Obstacle(void)
{
    return;
}

/****************************************************************************
* 名    称：void MagIO_Check(void)
* 功    能：
* 入口参数：gHalData->forwardIO[8],gHalData->leftIO[8]
* 出口参数：gInnerData->MagFIOX, gInnerData->OnMag, gInnerData->MagLIOX
* 说    明：根据16个输入点确定磁条相对于磁条的位置
* 调用方法：无 
****************************************************************************/
void MagIO_Check(void)
{
	return;
}
/****************************************************************************
* 名    称：void Status_Handle(void)
* 功    能：
* 入口参数：gHalData->Fbobstacle[8],gHalData->Station,gHalData->Pause,gHalData->Start,
* 出口参数：gInnerData->ScaleObst,gInnerData->Station,gHalData->InPosition,gComData->TurnFinish
* 说    明：
* 调用方法：无 
****************************************************************************/

void Status_Handle(void)
{	
    return;
}	
/****************************************************************************
* 名    称：void Command_Handle(void)
* 功    能：calculate the wheels' speed based on cmd velocity 
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void Command_Handle(void)
{
	char temp = 0;        //////8位
	int i = 0;            //////32位
	double caltmp = 0.0;  ///////32位
	double tmpx = 0.0;
	double tmpy = 0.0;
	double delx = 0.0;
	double dely = 0.0;
	int j = 0;
	int finishcount = 0;
	static int segstate = 0;
	static int finish[10] = {0,};
	static int finishnum = 0;
	static 	char flagssss=0;
	//如果接收到上层命令
	gUpData->FbStatus = 0;
	if(1 == gInnerData->NeedUp)
	{
	  
		gUpData->FbStatus = 1;
	    //对串口接收到的上层数据进行CRC校验
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
			//校验通过后，根据命令类型，处理命令数据
			gUpData->FbStatus = 2;
			if(gDownData->Cmd == 0x73)
		    {
				gInnerData->State = 0;
			    //小端对齐方式，数据低位对应内存低位
				//计算x方向的速度，单位毫米每秒
				gComData->CmdVx = (double)(gDownData->Angle)/1000.0;
				//计算y方向的速度，单位毫米每秒
				gComData->CmdVy = (double)(gDownData->distx)/1000.0;
				//计算theta方向的速度，单位弧度每秒
				gComData->CmdVthe = (double)(gDownData->disty)/1000.0;
			}
			else if(gDownData->Cmd == 0x72)
			{
			    gUpData->FbStatus = 3;
			    //脱离手控模式，进入自动控制阶段
				//在自动模式下，考虑到kiva小车移动方式只有前1、后2、原地逆时针旋转3、原地顺时针旋转4、停止5、原地升起货架6五种情况
				//计算最接近的人工路标点，根据是否是新扫描的二维码标志，决定是否修改反馈位置。
				//tmpx = (int)(gComData->FbPosx+0.5);
				//tmpy = (int)(gComData->FbPosy+0.5);
				if(1 == gDownData->NewCode)//搜到二维码
				{
					gComData->QrCodeX = gDownData->qrCodeX;
					gComData->QrCodeY = gDownData->qrCodeY;
					gComData->FbPosthe = gDownData->Angle*3.1415926/18000.0;
					gComData->FbPosx = gDownData->distx*0.01*0.021075*cos(gComData->FbPosthe) - gDownData->disty*0.01*0.021075*sin(gComData->FbPosthe) + gComData->QrCodeX;
					gComData->FbPosy = gDownData->distx*0.01*0.021075*sin(gComData->FbPosthe) + gDownData->disty*0.01*0.021075*cos(gComData->FbPosthe) + gComData->QrCodeY;
					
				}
				
				//根据坐标系定义，将命令的目标位置差赋值给计算结构体					
				gComData->goalx = gDownData->goalx;//△值
				gComData->goaly = gDownData->goaly;
				
				//根据state运动段新旧累计变量，决定是否重新设置小车运动起点。并设置相关内部状态位。
				if(gDownData->State != segstate)
				{
					gComData->startx  = (int)(gComData->FbPosx+0.5);
					gComData->starty  = (int)(gComData->FbPosy+0.5);
					gComData->State = 0;
					gComData->Finished = 0;
					segstate = gDownData->State;
				}
				gUpData->FbErr =  segstate;
				//计算本运动段总路程，并根据目标点x,y位置差决定小车朝向
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
			    //初始化阶段的命令，根据二维码图像信息计算小车全局坐标
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
	//根据控制器内部状态位，决定小车旋转还是直行
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
			else   //逻辑还要看看
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
		//如果小车旋转到目标朝向，则进入直行阶段	 存在风险xy没有调节好的话。
		if(fabs(gComData->goalth - gComData->FbPosthe) < 0.01 || fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0)< 0.01)
		{
			gComData->State = 1;
			gComData->CmdVx = 0.0;///0.4//////////////////////////
		}
		//如果小车旋转到目标朝向，则进入后后退直行阶段
		else if((fabs(gComData->goalth - gComData->FbPosthe) >3.13&&fabs(gComData->goalth - gComData->FbPosthe) <3.152)||(fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0) >3.13&&fabs(fabs(gComData->goalth - gComData->FbPosthe) - 3.1415926*2.0) <3.152))
		{
			gComData->State = 5;
			gComData->CmdVx = 0.0;///0.4//////////////////////////
		}
	}
	if(gComData->State==1||gComData->State==5)
	{
	    //line driver
		//根据反馈位置和目标位置差值与Pi的大小，决定旋转速度。
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
		//根据偏离和自身朝向转换坐标系，给定小车自身坐标系下y方向的命令速度
		delx = gComData->FbPosx - (int)(gComData->FbPosx+0.5);
		dely = gComData->FbPosy - (int)(gComData->FbPosy+0.5);
		
		gComData->CmdVy = -(dely*cos(gComData->FbPosthe) - delx*sin(gComData->FbPosthe)) * 1.8;
		//根据给定路程给定小车不超过0.5m/s的速度；
		if(0.5*0.5/gInnerData->MaxAcc < gComData->goall)
		{
			gComData->CmdVx = 0.2;///0.4/////////////////////////
		}
		else
		{
		    gComData->CmdVx = sqrt(gComData->goall * gInnerData->MaxAcc)*0.5;//////////////
		}
		//根据反馈位置计算小车目前已经行进的路程
		caltmp = (gComData->FbPosx - gComData->startx)*(gComData->FbPosx - gComData->startx);
		caltmp += (gComData->FbPosy - gComData->starty)*(gComData->FbPosy - gComData->starty);
		gComData->progressl = sqrt(caltmp);
		//根据剩余路程判断是否降速，并在目标点附近进行PID调节，使之准确到达目标点
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
		//根据全局位置偏差，判断是否到达目标点，如果到达则设置运动段完成标志，否则置为0
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
	//对小车y方向速度进行限制
	if(gComData->CmdVy > 0.2)////0.3
	{
		gComData->CmdVy = 0.2;////
    }
	else if(gComData->CmdVy < -0.2)////
	{
		gComData->CmdVy = -0.2	;/////
    }
	//对th方向速度进行加减速控制
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
	//对x方向速度进行加减速控制
	if(gComData->CmdVx > gComData->LastCmdVx + gInnerData->MaxAcc * gInnerData->Period)
	{
	    gComData->CmdVx = gComData->LastCmdVx + gInnerData->MaxAcc * gInnerData->Period;
	}
	else if(gComData->CmdVx < gComData->LastCmdVx - gInnerData->MaxAcc * gInnerData->Period)
	{
	    gComData->CmdVx = gComData->LastCmdVx - gInnerData->MaxAcc * gInnerData->Period;
	}
	gComData->LastCmdVx	= gComData->CmdVx;

	//计算四个轮子的速度,单位是m/s
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
* 名    称：void Data_Send(void)
* 功    能：calculate the wheels' speed based on cmd velocity 
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
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
			GPIO_SetBits(GPIOE, GPIO_Pin_2);
		}
		SendPDO(num);
		gHalData->WheelHal[num].ObDict[STATUSWORD].Value[0] = 0;
		Delay(20000); // 开一个定时器，把他替换掉，标志位计数等于MOTORNUM时清零，发送同步信号
	}
}
/****************************************************************************
* 名    称：void Status_UpLoad(void)
* 功    能：send the controller's status to upper level
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void Status_UpLoad(void)
{
	int i = 0;
	char crc = 0;
	static char num = 0;
   	
	SendSYNC();
//	Delay(20000);

	//如果数据CRC校验正确，并且上层需要数据，则对数据进行校验后发送出去
	num++;
	
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
//		//发送开始标志
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

