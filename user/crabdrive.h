#ifndef __CRABDRIVE_H__
#define __CRABDRIVE_H__

#include "movebase.h"
extern int up1 ;
extern uint8_t read_star;
extern uint8_t num_motor_flag ;

typedef enum 
{
	NoneedTrun = 0, 
	NeedTrun90 ,
	RightLiftSame90, //�����ƶ���
	RightLiftOther90, //ԭ����ת��
	Trun90Over,
	NeedTrunRight45,
	NeedTrunLife45,
	From90to45,
	TrunRight45Over,
	TrunLift45Over,
	SameRun,
	OtherRun,
	NullState,
	ParallelState,
	CmdRunOver,
} ProcessState;


void move_process(void);
	
#endif
