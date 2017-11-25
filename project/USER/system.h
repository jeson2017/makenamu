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

//用于接收蓝牙通信收到的上层命令
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
	char State;                     //用于发送小车运行状态
	char Cmd;						//用以保存发送给小车的命令类型，比如手动控制和自动控制等；
	char CmdBeating;				//用以保存上下层控制心跳，目前没用上
	char CmdCRC;  					//用以保存蓝牙通信控制命令通信内容的CRC校验码
}BaseDownSt;

//用于反馈小车控制系统状态
typedef struct BaseUpStruct
{
    
	int FbVx;						//用以保存反馈的小车x速度；
	int FbVy;						//用以保存反馈的小车y速度；
	int FbVthe;						//用以保存反馈的小车theta速度；
	int Fbposx;
	int Fbposy;
	int Fbposth;
	char FbErr;                     //用以保存导致小车停止的错误
	char FbStatus;                  //用以保存小车状态
	char FbFlag;                    //暂时保留
    char FbBeating;                 //反馈心跳
    char FbCRC;                     //校验
}BaseUpSt;

//用于内部计算用的命令和状态反馈
typedef struct BaseComStruct
{
    double CmdVx;					//用以保存根据上层命令获取的x方向速度，单位m/s
	double CmdVy;					//用以保存根据上层命令获取的y方向速度，单位m/s
	double CmdVthe;					//用以保存根据上层命令获取的theta方向速度，单位rad/s
	double LastCmdVx;
	double LastCmdVthe;
	double PosErr;					//用以保存小车在y方向上的误差累计，用于pid调节
	double PosThetaErr;				//用以保存小车在theta方向上的误差累计，用于pid调节
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
	unsigned char CmdStation;		//用以保存小车下一个目标站点
	unsigned char TurnFinish;		//用以保存小车原地旋转完毕的标志
	unsigned char TurnFlag;			//用以保存小车原地旋转标志符
	
	double FbVx;					//暂时未用
	double FbVy;					//暂时未用
	double FbVthe;					//暂时未用
	double FbPosx;
	double FbPosy;
	double FbPosthe;  

	double QrCodeX;
	double QrCodeY;
}BaseComSt;

//用于内部计算的内部参数
typedef struct BaseInnerStruct
{
	double Length;				        //表示轮组中心距单位m
	double Width;					    //表示一组轮子间距单位m
	double WheelPeri;					//表示轮子周长单位m
	double GearScale;					//表示电机到轮子的齿轮比32
	double MotorEncoder;
	double MagFIOX;						//用以表示前方磁条传感器感应到的磁条位置
	double MagBIOX;						//用以表示后方磁条传感器感应到的磁条位置
	double MagLIOY;						//用以表示左方磁条传感器感应到的磁条位置
	double MagRIOY;						//用以表示右方磁条传感器感应到的磁条位置
	double MagDist;						//用以表示磁条传感器IO口间距
	double MagFBefore;                  //用以表示上一次前方磁条传感器感应到的磁条位置
	double MagBBefore;					//用以表示上一次后方磁条传感器感应到的磁条位置
	double MagLBefore;					//用以表示上一次左方磁条传感器感应到的磁条位置
	double MagRBefore;					//用以表示上一次右方磁条传感器感应到的磁条位置
	double ThetaKp;                     //用以保存pid调节比例系数
	double ThetaKi;                     //用以保存pid调节积分系数
	double ScaleObst;					//用以保存运动系数
	double MaxAcc;
	double MaxAccTh;
	double Period;
	unsigned int AvgObstacle[8];		//用以保存8个超声波探头测到的距离
	unsigned char NeedUp;				//表示是否需要上传数据
	unsigned char DebugFlag;            //用于调试标志，备份

	unsigned char FatalErr;             //用于标识致命错误，电机应立即停止并下使能；
//	unsigned char Beating;				//用于判断上下层通信是否超时
//	unsigned char ComBeat;              //用于判断上下层通信是否超时
	unsigned char Station;				//用以保存根据hal层读取的ID卡号而对应的站点号
	unsigned char State;				//用于表示是手动状态或者自动状态
	unsigned char OnMag;				//用于表示是否检测到磁条
	unsigned char InPosition;			//用于表示是否到达目标点
	unsigned char FlagStation;			//用于表示是否需要寻找站点
}BaseInnerSt;

//四个轮子的相关参数
typedef struct WheelStruct
{
    double CmdVel;					//用以保存轮子速度，单位m/s
	double FbVel;
	int CmdMotorvel;				//用以保存电机转速，单位r/min
	int FlagForward;			    //用以表示电机正方向；
	double FbPos;
	double LastFbpos;
	unsigned char PowerFlag; 
	ObjectDictSt ObDict[24]; 		//用于存储对象字典
}WheelSt;

//用以保存底层数据
typedef struct BaseHalStruct
{
	unsigned int Station; 			//用以表示ID卡读卡器读取的ID卡编码
	unsigned char forwardIO[8];		//用以表示前方8个IO状态
	unsigned char backIO[8];		//用以表示后方8个IO状态
	unsigned char leftIO[8];		//用以表示左方8个IO状态
	unsigned char rightIO[8];		//用以表示右方8个IO状态
	unsigned char Pause;			//用以保存从上层读取到的暂停标志位
	unsigned char Start;			//用以保存从上层读取到的启动标志位
	unsigned char ForwardOb;
	unsigned char LeftOb;
	unsigned char InPosition;		//用以保存是否到达目标点的到位标志
	unsigned char FbMotor;			//用以保存与驱动器的485通信是否有返回
	unsigned char Fbobstacle[8];	//用以保存读取到的8个超声波探头测距值

	WheelSt WheelHal[MOTORNUM];			//用以保存4个轮子的相关数据
}BaseHalSt;

typedef struct BaseSysStruct
{
    BaseComSt BaseComData;			//用以保存需要上下层通信的参数
	BaseInnerSt BaseInnerData;		//用以保存内部计算用的配置参数和其他参数
	BaseHalSt BaseHalData;			//用以保存从底层传感器获取的参数
	BaseDownSt BaseDownData;		//用以保存蓝牙通信获取的数据
	BaseUpSt BaseUpData;			//用以保存蓝牙通信上传的数据
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

