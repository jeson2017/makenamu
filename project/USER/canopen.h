/****************************************************************************
* Copyright (C), 2013
***************************************************************************/
#ifndef __CANOPEN_H
#define __CANOPEN_H

enum object
{
    CONTROLWORD = 0,
	STATUSWORD,
	CMDTYPE,
	ACTUALTYPE,
	TARGETVEL,
	Velocit_TARGETVEL,
	ACTUALTARGETVEL,
	ACTUALVEL,
	TARGETPOS,
	ACTUALTARGETPOS,
	ACTUALPOS,
	MAXVEL,
	IDRXPDO1,
	RXPDO1TYPE,
	NUMRXPDO1,
	RXPDO1OB1,
	RXPDO1OB2,
	IDTXPDO1,
	TXPDO1TYPE,
	NUMTXPDO1,
	TXPDO1OB1,
	TXPDO1OB2,
	HEARTBEATTIME	
};

typedef struct ObjectDictStruct
{
	unsigned short Index;
	unsigned char SubIndex;
	unsigned char Size;
	unsigned char Value[4]; 
}ObjectDictSt;

void CanOpen_Init(void);
void SendSDO(int drivernum,int objectnum,int value);
void ReadSDO(int drivernum,int objectnum);
void PreOPtoOP(void);
void SendSYNC(void);
void SendPDO(int drivernum);





#endif











