#include "pwm.h"
/*************************************************************************
*功能描述：  PWM输出初始化
*入口参数：  Per重装值，Psc分频值
*返回的值：  无
***************************************************************************/
void PWM_Init_TIM1(u16 Psc, u16 Per)
{			
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;


		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO, ENABLE);	 


		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);							 


		TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = Per;
		TIM_TimeBaseInitStruct.TIM_Prescaler = Psc;
		TIM_TimeBaseInit(TIM1 ,&TIM_TimeBaseInitStruct);


		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; 
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse =0;
		TIM_OC1Init(TIM1,&TIM_OCInitStruct);
		TIM_OC4Init(TIM1,&TIM_OCInitStruct);
		
		
		TIM_CtrlPWMOutputs(TIM1,ENABLE); //主输出使能，这个必须开，高级定时器不开的话是无法输出PWM波形的！！！！！！
		
		
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		
		TIM_Cmd(TIM1, ENABLE);
}
