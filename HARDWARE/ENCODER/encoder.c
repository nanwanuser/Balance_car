#include "encoder.h"





/*************************************************************************
*功能描述：  编码器接口初始化
*入口参数：  无
*返回的值：  无
***************************************************************************/
void Encoder_TIM2_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_ICInitTypeDef TIM_ICInitStruct;
		
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);									                                //开启时钟  GPIOA,TIM2
																																					                              
																																					                              
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;									                              	//浮空输入
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;                                               
		GPIO_Init(GPIOA,&GPIO_InitStructure);							 										                              	//初始化GPIOA ——Pin_0,Pin1
																																					                              
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;				                                      //不分频
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		                                      //向上计数
		TIM_TimeBaseInitStruct.TIM_Period = 65535;											                                      //设置重装值
		TIM_TimeBaseInitStruct.TIM_Prescaler = 0;                                                             
		TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);									                                      //初始化定时器
		
		
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12 ,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //配置为编码器模式
		
		TIM_ICStructInit(&TIM_ICInitStruct);
		TIM_ICInitStruct.TIM_ICFilter = 10;              																							        //滤波等级
		TIM_ICInit(TIM2, &TIM_ICInitStruct);             																							        //初始化输入捕获
																									
																									
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);            																							        //清除溢出更新中断标志位
																																																				  
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);       																							        //配置溢出更新中断标志位
																																																				  
		TIM_SetCounter(TIM2, 0);                         																							        //清零定时器计数值
																																																				  
		TIM_Cmd(TIM2, ENABLE);                           																							        //开启定时器
}





/*************************************************************************
*功能描述：  编码器接口初始化
*入口参数：  无
*返回的值：  无
***************************************************************************/
void Encoder_TIM4_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_ICInitTypeDef TIM_ICInitStruct;
		
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
		
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Period = 65535;
		TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
		TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
		
		
		TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12 ,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
		TIM_ICStructInit(&TIM_ICInitStruct);
		TIM_ICInitStruct.TIM_ICFilter = 10;
		TIM_ICInit(TIM4, &TIM_ICInitStruct);
		
		
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		
		TIM_SetCounter(TIM4, 0);
		
		TIM_Cmd(TIM4, ENABLE);
}





/*************************************************************************
*功能描述：  单位时间读取编码器计数
*入口参数：  定时器
*返回的值：  速度值
***************************************************************************/

int Read_Speed(int TIMx)
{	
		int value_1;
		switch(TIMx)
		{
			case 2:value_1 = (short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2, 0);break;
			case 4:value_1 = (short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4, 0);break;
			default: value_1 = 0;
		}
		return value_1;
}





/*************************************************************************
*功能描述：  清除定时器中断标志位
*入口参数：  无
*返回的值：  无
***************************************************************************/

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}




/*************************************************************************
*功能描述：  清除定时器中断标志位
*入口参数：  无
*返回的值：  无
***************************************************************************/
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
