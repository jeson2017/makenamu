#include "time.h"



/*==============================================================================
??????,????PEM?
TIMx:???? ?:TIM2
arr:????? ?:199
psc:?????? ?:71
GPIOx: ??????? ?:GPIOA
GPIO_Pin_x:????????  ?:GPIO_Pin_1
CHx:PWM??????  ?:1
==============================================================================*/
void TIMx_PWMx_Init(TIM_TypeDef* TIMx,u16 arr,u16 psc,GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,u8 CHx)
{		 					 
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef         TIM_OCInitStructure;     
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //??????TIMx???????????  ??? 71:1MHZ  0.001MS/?
	TIM_TimeBaseStructure.TIM_Period = arr; //???????????????????????????	 80K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //??????:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//????!!
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //??TIM_TimeBaseInitStruct?????????TIMx???????
  TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(TIMx, ENABLE); //??TIMx?ARR????????
           
 	TIM_Cmd(TIMx, ENABLE);  //??TIMx??
//				if(CHx>0  &&  CHx<5)
//        {
//          GPIOx_Init(GPIOx,GPIO_Pin_x,GPIO_Mode_AF_PP,0);//????????????,??PWM????
//          TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //CH1 PWM2??	
//          TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
//          TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	
//          TIM_OCInitStructure.TIM_Pulse = 0; //????????????????
//          TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //?????????CCR1_Val?????
//          TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//          TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//          TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;        
//          
//          switch(CHx)
//          {
//          case  1:
//            TIM_OC1Init(TIMx, &TIM_OCInitStructure);  //??TIM_OCInitStruct???????????TIMx
//            TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);  //CH1 ?????
//            break;
//          case  2:
//            TIM_OC2Init(TIMx, &TIM_OCInitStructure);  //??TIM_OCInitStruct???????????TIMx
//  	    TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);  //CH2 ?????
//            break;
//          case  3:
//            TIM_OC3Init(TIMx, &TIM_OCInitStructure);  //??TIM_OCInitStruct???????????TIMx
//            TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);  //CH3 ?????
//            break;
//          case  4:
//            TIM_OC4Init(TIMx, &TIM_OCInitStructure);  //??TIM_OCInitStruct???????????TIMx
//            TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);  //CH4 ?????
//            break;
//          default:
//            break;
//          }
//        }
//         
//				if((TIMx ==  TIM1  ||  TIMx  ==  TIM8)&&(CHx>0  &&  CHx<5))
//          TIM_CtrlPWMOutputs(TIMx, ENABLE);//??PWM??
					
}
