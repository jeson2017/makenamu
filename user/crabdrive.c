#include "crabdrive.h"

uint8_t turn_state = 0; //  0:水平   1:90   2:45right 3:45lift
uint8_t cmd_flag =0; //0: 完成 1：正在进行
//uint8_t need_turn =0;// 0:不需要转向 1：需要转向
ProcessState RunState ;
ProcessState SaveState;
int up1 = 0;
uint8_t read_star =0;
uint8_t num_motor_flag = 10;


void move_process()
{
		int i =0;
		static int down1[4] = {0};
		for(i =0;i<4;i++)
		{
			 down1[i] = (*((int *)&(gHalData->WheelHal[i].ObDict[ACTUALPOS].Value[0])));
		}
		if(buletooth_motor_para.up_data_flag == 1)
		{
			if(cmd_flag == 0)
			{				
				if(buletooth_motor_para.driver_motor_y_speed == 0 && buletooth_motor_para.driver_motor_th_speed == 0 && buletooth_motor_para.driver_motor_x_speed != 0)
				{
					RunState = NoneedTrun;
					SaveState = NoneedTrun;
				}
				if(buletooth_motor_para.driver_motor_y_speed == 0 && buletooth_motor_para.driver_motor_th_speed == 0 && buletooth_motor_para.driver_motor_x_speed == 0)
				{
					RunState = NullState;
					SaveState = NullState;
				}
				if((buletooth_motor_para.driver_motor_y_speed != 0 && buletooth_motor_para.driver_motor_x_speed == 0) || (buletooth_motor_para.driver_motor_th_speed != 0 && buletooth_motor_para.driver_motor_x_speed == 0))
				{
					RunState = NeedTrun90;
					if(buletooth_motor_para.driver_motor_y_speed != 0 && buletooth_motor_para.driver_motor_x_speed == 0)
						SaveState = RightLiftSame90;
					if(buletooth_motor_para.driver_motor_th_speed != 0 && buletooth_motor_para.driver_motor_x_speed == 0)
						SaveState = RightLiftOther90;
				}
				if(buletooth_motor_para.driver_motor_y_speed > 0 && buletooth_motor_para.driver_motor_x_speed != 0)
				{
					RunState = NeedTrunLife45;
					SaveState = NeedTrunLife45;
				}
				if(buletooth_motor_para.driver_motor_y_speed < 0 && buletooth_motor_para.driver_motor_x_speed != 0)
				{
					RunState = NeedTrunRight45;
					SaveState = NeedTrunRight45;
				}
			}
			
			switch(RunState)
			{
				case NullState: // 没有指令，连接上没有发送指令,一步动作
					for(i = 0;i < MOTORNUM;i++)
					{
						gHalData->WheelHal[i].CmdVel = 0;
					}
					cmd_flag = 0;
					break;
				case NoneedTrun: //判断是否为平行状态，如果不平行矫正后运行
					if(turn_state == 0)
					{
						RunState = SameRun;
						turn_state = 0;
						cmd_flag = 1;
					}
					else if( turn_state == 1 || turn_state == 3) // 90 度  左45度
					{
						gHalData->WheelHal[0].CmdVel = -0.005;
						gHalData->WheelHal[1].CmdVel = -0.005;
						gHalData->WheelHal[2].CmdVel = -0.005;
						gHalData->WheelHal[3].CmdVel = -0.005;
						cmd_flag = 1;
						RunState = ParallelState;
					}
					else if(turn_state == 2 )  //右45度
					{
						gHalData->WheelHal[0].CmdVel = 0.005;
						gHalData->WheelHal[1].CmdVel = 0.005;
						gHalData->WheelHal[2].CmdVel = 0.005;
						gHalData->WheelHal[3].CmdVel = 0.005;
						cmd_flag = 1;
						RunState =ParallelState;
					}
					
					break;
				case ParallelState:
					if(abs(down1[num_motor_flag] - up1) <= 100 ) // reset the number 
					{
						gHalData->WheelHal[0].CmdVel = 0;
						gHalData->WheelHal[1].CmdVel = 0;
						gHalData->WheelHal[2].CmdVel = 0;
						gHalData->WheelHal[3].CmdVel = 0;
						RunState = SameRun;
						turn_state = 0;
						cmd_flag = 1;
					}
					break;
				case NeedTrun90:
					if(turn_state != 1)
					{
						gHalData->WheelHal[0].CmdVel = 0.005;
						gHalData->WheelHal[1].CmdVel = 0.005;
						gHalData->WheelHal[2].CmdVel = 0.005;
						gHalData->WheelHal[3].CmdVel = 0.005;
						
						RunState =Trun90Over;	
					}
					else 
					{
						RunState =SameRun;		
					}						
					cmd_flag = 1;
					break;
				case Trun90Over:
					if(abs(down1[num_motor_flag] - up1) >= 995 )//&& abs(down1[num_motor_flag] - up1) <= 1005)
					{
						
						gHalData->WheelHal[0].CmdVel = 0;
						gHalData->WheelHal[1].CmdVel = 0;
						gHalData->WheelHal[2].CmdVel = 0;
						gHalData->WheelHal[3].CmdVel = 0;
						turn_state = 1;
//						if(buletooth_motor_para.driver_motor_th_speed != 0)
//						 RunState =OtherRun;
//						else 
							RunState =SameRun;
					}
					
					break;
				case NeedTrunRight45:
					if(turn_state != 2)
					{
						cmd_flag = 1;
						if(turn_state == 1)
						{
						gHalData->WheelHal[0].CmdVel = -0.005;
						gHalData->WheelHal[1].CmdVel = -0.005;
						gHalData->WheelHal[2].CmdVel = -0.005;
						gHalData->WheelHal[3].CmdVel = -0.005;
//						RunState = From90to45;
						}
					}
					RunState =TrunRight45Over;
					break;
				case TrunRight45Over:
					if(abs(down1[num_motor_flag] - up1) >= 495  && down1[num_motor_flag] < up1)//&& abs(down1[num_motor_flag] - up1) <= 1005)
					{
						gHalData->WheelHal[0].CmdVel = 0;
						gHalData->WheelHal[1].CmdVel = 0;
						gHalData->WheelHal[2].CmdVel = 0;
						gHalData->WheelHal[3].CmdVel = 0;						
						turn_state = 2;
						RunState =SameRun;
					}
					break;
				case NeedTrunLife45:
					if(turn_state == 2 || turn_state == 0)
					{
						cmd_flag = 1;
						gHalData->WheelHal[0].CmdVel = 0.005;
						gHalData->WheelHal[1].CmdVel = 0.005;
						gHalData->WheelHal[2].CmdVel = 0.005;
						gHalData->WheelHal[3].CmdVel = 0.005;
						RunState = TrunLift45Over;
					}
					else if(turn_state == 1)
					{
						cmd_flag = 1;
						gHalData->WheelHal[0].CmdVel = -0.005;
						gHalData->WheelHal[1].CmdVel = -0.005;
						gHalData->WheelHal[2].CmdVel = -0.005;
						gHalData->WheelHal[3].CmdVel = -0.005;
						RunState = From90to45;
					}
					
					break;
				case From90to45:
					if(abs(down1[num_motor_flag] - up1) < 505 )//&& abs(down1[num_motor_flag] - up1) <= 1005)
					{
						gHalData->WheelHal[0].CmdVel = 0;
						gHalData->WheelHal[1].CmdVel = 0;
						gHalData->WheelHal[2].CmdVel = 0;
						gHalData->WheelHal[3].CmdVel = 0;
						
						turn_state = 3;
						RunState =SameRun;
					}
					break;
				case TrunLift45Over:
					if(abs(down1[num_motor_flag] - up1) >= 495 && down1[num_motor_flag] > up1 )//&& abs(down1[num_motor_flag] - up1) <= 1005)
					{
						gHalData->WheelHal[0].CmdVel = 0;
						gHalData->WheelHal[1].CmdVel = 0;
						gHalData->WheelHal[2].CmdVel = 0;
						gHalData->WheelHal[3].CmdVel = 0;
						
						turn_state = 2;
						RunState =SameRun;
					}
					break;
				case SameRun:
					if(SaveState ==NoneedTrun )
					{
						gHalData->WheelHal[4].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
						gHalData->WheelHal[5].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
					}
					if(SaveState == RightLiftSame90)
					{
						gHalData->WheelHal[4].CmdVel = (buletooth_motor_para.driver_motor_y_speed/DIV_SPEED);
						gHalData->WheelHal[5].CmdVel = (buletooth_motor_para.driver_motor_y_speed/DIV_SPEED);
					}
					if(SaveState == RightLiftOther90)
					{
						gHalData->WheelHal[4].CmdVel = (buletooth_motor_para.driver_motor_th_speed/DIV_SPEED);
						gHalData->WheelHal[5].CmdVel = -(buletooth_motor_para.driver_motor_th_speed/DIV_SPEED);
					}
					cmd_flag = 0;
//					gHalData->WheelHal[6].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[7].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
					break;
				case OtherRun:
					if(buletooth_motor_para.driver_motor_th_speed > 0)
					{
//					gHalData->WheelHal[4].CmdVel = -(buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[5].CmdVel = -(buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[6].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[7].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
						cmd_flag = 0;
					}
					if(buletooth_motor_para.driver_motor_th_speed < 0)
					{
//					gHalData->WheelHal[4].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[5].CmdVel = (buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[6].CmdVel = -(buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
//					gHalData->WheelHal[7].CmdVel = -(buletooth_motor_para.driver_motor_x_speed/DIV_SPEED);
						cmd_flag = 0;
					}
					break;
//				case CmdRunOver:
//					SaveState = CmdRunOver;
//					break;
				default :
					cmd_flag = 0;
					break;
			}			
			buletooth_motor_para.up_data_flag = 0;
		}

	}
