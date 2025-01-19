#include "exti.h"

/*************************************************************************
*功能描述：  MPU6050引脚触发外部中断初始化
*入口参数：  无
*返回的值：  无
***************************************************************************/

void MPU6050_EXTI_Init(void)
{			
		GPIO_InitTypeDef  GPIO_InitStructure;	
		EXTI_InitTypeDef EXTI_InitStruct;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO , ENABLE);	 


		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
	
		EXTI_InitStruct.EXTI_Line = EXTI_Line2;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_Init(&EXTI_InitStruct);
	
}
