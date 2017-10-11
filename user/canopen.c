/****************************************************************************
* Copyright (C), 2013
***************************************************************************/
#include "movebase.h"


void ObjectConfig(ObjectDictSt* object,unsigned short index,unsigned char subindex,unsigned char size)
{
	object->Index = index;
	object->SubIndex = subindex;
	object->Size = size;
}

void CanOpen_Init(void)
{
    int num = 0;

    for(num = 0;num < MOTORNUM;num++)
	{
	    ObjectConfig(&(gHalData->WheelHal[num].ObDict[CONTROLWORD]),0x6040,0,16);	   //控制字	 16代表位
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[STATUSWORD]),0x6041,0,16);	   //状态字
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[CMDTYPE]),0x6060,0,8);	       //命令运行模式
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[ACTUALTYPE]),0x6061,0,8);	       //当前运行模式
		
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[Velocit_TARGETVEL]),0x206B,0,32);	       //速度模式给定命令速度
		
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[TARGETVEL]),0x60ff,0,32);	       //给定命令速度
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[ACTUALTARGETVEL]),0x606b,0,32);  //当前命令速度
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[ACTUALVEL]),0x606c,0,32);	       //当前实际速度
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[TARGETPOS]),0x607a,0,32);	       //给定命令位置
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[ACTUALTARGETPOS]),0x6062,0,32);  //当前命令位置
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[ACTUALPOS]),0x6064,0,32);	       //当前实际位置

		ObjectConfig(&(gHalData->WheelHal[num].ObDict[MAXVEL]),0x607f,0,32);	   //最大速度
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[IDRXPDO1]),0x1400,1,32);	   //接收RXPDO1的COB-ID  0x00000202
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[RXPDO1TYPE]),0x1400,2,8);	   //接收RXPDO1的类型 0x01
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[NUMRXPDO1]),0x1600,0,8);	   //接收RXPDO1的映射的对象数量	0x02
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[RXPDO1OB1]),0x1600,1,32);	   //接收RXPDO1的第一个映射对象	0x60400010
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[RXPDO1OB2]),0x1600,2,32);	   //接收RXPDO1的第二个映射对象	0x60ff0020
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[IDTXPDO1]),0x1800,1,32);	   //发送TXPDO1的COB-ID	0x40000182
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[TXPDO1TYPE]),0x1800,2,8);	   //发送TXPDO1的类型 0x01
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[NUMTXPDO1]),0x1A00,0,8);	   //发送TXPDO1的映射的对象数量	0x02
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[TXPDO1OB1]),0x1A00,1,32);	   //发送TXPDO1的第一个映射对象	0x60410010
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[TXPDO1OB2]),0x1A00,2,32);	   //发送TXPDO1的第二个映射对象	0x606C0020
		ObjectConfig(&(gHalData->WheelHal[num].ObDict[HEARTBEATTIME]),0x1017,0,16);	   //最大速度
	}
}

void SendSDO(int drivernum,int objectnum,int value)
{
    CanTxMsg TxMsg;

	TxMsg.StdId = 0x0600 | (drivernum + 1);
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 8;

    switch(gHalData->WheelHal[drivernum].ObDict[objectnum].Size)
	{
	    case 8:
		    gHalData->WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x00ff;
			TxMsg.Data[0] = 0x2f;
			break;
		case 16:
		    gHalData->WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			TxMsg.Data[0] = 0x2b;
			break;
		case 24:
		    gHalData->WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[2]=(value&0x00ff0000)>>16;
			TxMsg.Data[0] = 0x27;
			break;
		case 32:
		    gHalData->WheelHal[drivernum].ObDict[objectnum].Value[0]=value&0x000000ff;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[1]=(value&0x0000ff00)>>8;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[2]=(value&0x00ff0000)>>16;
			gHalData->WheelHal[drivernum].ObDict[objectnum].Value[3]=(value&0xff000000)>>24;
			TxMsg.Data[0] = 0x23;
			break;
		default:
		    break;
	}

	TxMsg.Data[1] = gHalData->WheelHal[drivernum].ObDict[objectnum].Index&0x00ff;
	TxMsg.Data[2] = (gHalData->WheelHal[drivernum].ObDict[objectnum].Index&0xff00)>>8;
	TxMsg.Data[3] = gHalData->WheelHal[drivernum].ObDict[objectnum].SubIndex;
	TxMsg.Data[4] = gHalData->WheelHal[drivernum].ObDict[objectnum].Value[0];
	TxMsg.Data[5] = gHalData->WheelHal[drivernum].ObDict[objectnum].Value[1];
	TxMsg.Data[6] = gHalData->WheelHal[drivernum].ObDict[objectnum].Value[2];
	TxMsg.Data[7] = gHalData->WheelHal[drivernum].ObDict[objectnum].Value[3];
	CAN_Transmit(CAN1, &TxMsg);
}

void ReadSDO(int drivernum,int objectnum)
{
    CanTxMsg TxMsg;

	TxMsg.StdId = 0x0600 | (drivernum + 1);
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 8;
	TxMsg.Data[0] = 0x40;

	TxMsg.Data[1] = gHalData->WheelHal[drivernum].ObDict[objectnum].Index&0x00ff;
	TxMsg.Data[2] = (gHalData->WheelHal[drivernum].ObDict[objectnum].Index&0xff00)>>8;
	TxMsg.Data[3] = gHalData->WheelHal[drivernum].ObDict[objectnum].SubIndex;
	CAN_Transmit(CAN1, &TxMsg);
}


void PreOPtoOP(void)
{
	CanTxMsg TxMsg;

	TxMsg.StdId = 0x00;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] =  0x01;
	TxMsg.Data[1] =  0x00;
	CAN_Transmit(CAN1, &TxMsg);
}

void OPtoPreOP(void)
{
	CanTxMsg TxMsg;

	TxMsg.StdId = 0x00;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] =  0x80;
	TxMsg.Data[1] =  0x00;
	CAN_Transmit(CAN1, &TxMsg);
}

void SendSYNC(void)
{
	CanTxMsg TxMsg;

	TxMsg.StdId = 0x80;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] =  0x80;
	TxMsg.Data[1] =  0x00;
	CAN_Transmit(CAN1, &TxMsg);
}

void SendPDO(int drivernum)
{
    CanTxMsg TxMsg;

    TxMsg.StdId = 0x0200 | (drivernum + 1);
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 6;
	TxMsg.Data[0] = 0x0f;

	TxMsg.Data[1] = 0x00;
	TxMsg.Data[2] = gHalData->WheelHal[drivernum].CmdMotorvel&0x000000ff;
	TxMsg.Data[3] = (gHalData->WheelHal[drivernum].CmdMotorvel&0x0000ff00)>>8;
	TxMsg.Data[4] = (gHalData->WheelHal[drivernum].CmdMotorvel&0x00ff0000)>>16;
	TxMsg.Data[5] = (gHalData->WheelHal[drivernum].CmdMotorvel&0xff000000)>>24;
	CAN_Transmit(CAN1, &TxMsg);
}



















