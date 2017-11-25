/****************************************************************************
* Copyright (C), 2013
***************************************************************************/
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "string.h"
#include "canopen.h"
//#include "movebase.h"

#define MINDOUBLE (10e-5)
#define MAXDOUBLE (10e8)
#define MOTORNUM (4)

//���ڽ�������ͨ���յ����ϲ�����
typedef struct BaseDownStruct
{
    int Angle;		             	//
	int distx;		            	//
	int disty;	        	     	//
	int goalx;
	int goaly;
	int qrCodeX;
	int qrCodeY;
	char NewCode;
	char State;                     //���ڷ���С������״̬
	char Cmd;						//���Ա��淢�͸�С�����������ͣ������ֶ����ƺ��Զ����Ƶȣ�
	char CmdBeating;				//���Ա������²����������Ŀǰû����
	char CmdCRC;  					//���Ա�������ͨ�ſ�������ͨ�����ݵ�CRCУ����
}BaseDownSt;

//���ڷ���С������ϵͳ״̬
typedef struct BaseUpStruct
{
    
	int FbVx;						//���Ա��淴����С��x�ٶȣ�
	int FbVy;						//���Ա��淴����С��y�ٶȣ�
	int FbVthe;						//���Ա��淴����С��theta�ٶȣ�
	int Fbposx;
	int Fbposy;
	int Fbposth;
	char FbErr;                     //���Ա��浼��С��ֹͣ�Ĵ���
	char FbStatus;                  //���Ա���С��״̬
	char FbFlag;                    //��ʱ����
    char FbBeating;                 //��������
    char FbCRC;                     //У��
}BaseUpSt;

//�����ڲ������õ������״̬����
typedef struct BaseComStruct
{
    double CmdVx;					//���Ա�������ϲ������ȡ��x�����ٶȣ���λm/s
	double CmdVy;					//���Ա�������ϲ������ȡ��y�����ٶȣ���λm/s
	double CmdVthe;					//���Ա�������ϲ������ȡ��theta�����ٶȣ���λrad/s
	double LastCmdVx;
	double LastCmdVthe;
	double PosErr;					//���Ա���С����y�����ϵ�����ۼƣ�����pid����
	double PosThetaErr;				//���Ա���С����theta�����ϵ�����ۼƣ�����pid����
	double goalx;
	double goaly;
	double goalth;
	double goall;
	double progressl;
	double startx;
	double starty;
	double MaxVel;
	unsigned char State;
	unsigned char Finished;
	unsigned char CmdStation;		//���Ա���С����һ��Ŀ��վ��
	unsigned char TurnFinish;		//���Ա���С��ԭ����ת��ϵı�־
	unsigned char TurnFlag;			//���Ա���С��ԭ����ת��־��
	
	double FbVx;					//��ʱδ��
	double FbVy;					//��ʱδ��
	double FbVthe;					//��ʱδ��
	double FbPosx;
	double FbPosy;
	double FbPosthe;  

	double QrCodeX;
	double QrCodeY;
}BaseComSt;

//�����ڲ�������ڲ�����
typedef struct BaseInnerStruct
{
	double Length;				        //��ʾ�������ľ൥λm
	double Width;					    //��ʾһ�����Ӽ�൥λm
	double WheelPeri;					//��ʾ�����ܳ���λm
	double GearScale;					//��ʾ��������ӵĳ��ֱ�32
	double MotorEncoder;
	double MagFIOX;						//���Ա�ʾǰ��������������Ӧ���Ĵ���λ��
	double MagBIOX;						//���Ա�ʾ�󷽴�����������Ӧ���Ĵ���λ��
	double MagLIOY;						//���Ա�ʾ�󷽴�����������Ӧ���Ĵ���λ��
	double MagRIOY;						//���Ա�ʾ�ҷ�������������Ӧ���Ĵ���λ��
	double MagDist;						//���Ա�ʾ����������IO�ڼ��
	double MagFBefore;                  //���Ա�ʾ��һ��ǰ��������������Ӧ���Ĵ���λ��
	double MagBBefore;					//���Ա�ʾ��һ�κ󷽴�����������Ӧ���Ĵ���λ��
	double MagLBefore;					//���Ա�ʾ��һ���󷽴�����������Ӧ���Ĵ���λ��
	double MagRBefore;					//���Ա�ʾ��һ���ҷ�������������Ӧ���Ĵ���λ��
	double ThetaKp;                     //���Ա���pid���ڱ���ϵ��
	double ThetaKi;                     //���Ա���pid���ڻ���ϵ��
	double ScaleObst;					//���Ա����˶�ϵ��
	double MaxAcc;
	double MaxAccTh;
	double Period;
	unsigned int AvgObstacle[8];		//���Ա���8��������̽ͷ�⵽�ľ���
	unsigned char NeedUp;				//��ʾ�Ƿ���Ҫ�ϴ�����
	unsigned char DebugFlag;            //���ڵ��Ա�־������

	unsigned char FatalErr;             //���ڱ�ʶ�������󣬵��Ӧ����ֹͣ����ʹ�ܣ�
//	unsigned char Beating;				//�����ж����²�ͨ���Ƿ�ʱ
//	unsigned char ComBeat;              //�����ж����²�ͨ���Ƿ�ʱ
	unsigned char Station;				//���Ա������hal���ȡ��ID���Ŷ���Ӧ��վ���
	unsigned char State;				//���ڱ�ʾ���ֶ�״̬�����Զ�״̬
	unsigned char OnMag;				//���ڱ�ʾ�Ƿ��⵽����
	unsigned char InPosition;			//���ڱ�ʾ�Ƿ񵽴�Ŀ���
	unsigned char FlagStation;			//���ڱ�ʾ�Ƿ���ҪѰ��վ��
}BaseInnerSt;

//�ĸ����ӵ���ز���
typedef struct WheelStruct
{
    double CmdVel;					//���Ա��������ٶȣ���λm/s
	double FbVel;
	int CmdMotorvel;				//���Ա�����ת�٣���λr/min
	int FlagForward;			    //���Ա�ʾ���������
	double FbPos;
	double LastFbpos;
	unsigned char PowerFlag; 
	ObjectDictSt ObDict[24]; 		//���ڴ洢�����ֵ�
}WheelSt;

//���Ա���ײ�����
typedef struct BaseHalStruct
{
	unsigned int Station; 			//���Ա�ʾID����������ȡ��ID������
	unsigned char forwardIO[8];		//���Ա�ʾǰ��8��IO״̬
	unsigned char backIO[8];		//���Ա�ʾ��8��IO״̬
	unsigned char leftIO[8];		//���Ա�ʾ��8��IO״̬
	unsigned char rightIO[8];		//���Ա�ʾ�ҷ�8��IO״̬
	unsigned char Pause;			//���Ա�����ϲ��ȡ������ͣ��־λ
	unsigned char Start;			//���Ա�����ϲ��ȡ����������־λ
	unsigned char ForwardOb;
	unsigned char LeftOb;
	unsigned char InPosition;		//���Ա����Ƿ񵽴�Ŀ���ĵ�λ��־
	unsigned char FbMotor;			//���Ա�������������485ͨ���Ƿ��з���
	unsigned char Fbobstacle[8];	//���Ա����ȡ����8��������̽ͷ���ֵ

	WheelSt WheelHal[MOTORNUM];			//���Ա���4�����ӵ��������
}BaseHalSt;

typedef struct BaseSysStruct
{
    BaseComSt BaseComData;			//���Ա�����Ҫ���²�ͨ�ŵĲ���
	BaseInnerSt BaseInnerData;		//���Ա����ڲ������õ����ò�������������
	BaseHalSt BaseHalData;			//���Ա���ӵײ㴫������ȡ�Ĳ���
	BaseDownSt BaseDownData;		//���Ա�������ͨ�Ż�ȡ������
	BaseUpSt BaseUpData;			//���Ա�������ͨ���ϴ�������
}BaseSysSt;

extern BaseSysSt* gSysData;
extern BaseComSt* gComData;
extern BaseInnerSt* gInnerData;
extern BaseHalSt* gHalData;
extern BaseDownSt* gDownData;
extern BaseUpSt* gUpData;

extern void System_Init(void);
extern void Status_Handle(void);
extern void Command_Handle(void);
extern void Data_Send(void);
extern void Status_UpLoad(void);
extern void Drivers_Init(void);
#endif

