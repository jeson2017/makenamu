#ifndef __TIME_H__
#define __TIME_H__
#include "stm32f10x_tim.h"

void TIMx_PWMx_Init(TIM_TypeDef* TIMx,u16 arr,u16 psc,GPIO_TypeDef* GPIOx,u16 GPIO_Pin_x,u8 CHx);

#endif

