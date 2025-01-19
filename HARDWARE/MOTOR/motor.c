#include "motor.h"

/*************************************************************************
*功能描述：  电机引脚控制正反转初始化
*入口参数：  无
*返回的值：  无
***************************************************************************/
void Motor_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 


		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/*************************************************************************
*功能描述：  PWM限幅
*入口参数：  A电机的控制输出量，B电机的控制输出量
*返回的值：  无
***************************************************************************/

void Limit(int *motoA, int *motoB)
{
	if(*motoA > PWM_MAX)*motoA = PWM_MAX;
	if(*motoA < PWM_MIN)*motoA = PWM_MIN;
	
	if(*motoB > PWM_MAX)*motoB = PWM_MAX;
	if(*motoB < PWM_MIN)*motoB = PWM_MIN;
	
}
/*************************************************************************
*功能描述：  将PWM输出量转换为绝对值，好加载到电机上
*入口参数：  需要绝对值化的数据
*返回的值：  绝对值
***************************************************************************/
int GFP_abs(int p)
{
	int q;
	q = p>0?p:(-p);
	return q;
}
/*************************************************************************
*功能描述：  加载PWM输出量到电机上
*入口参数：  A电机的控制输出量，B电机的控制输出量
*返回的值：  无
***************************************************************************/
void Load(int moto1,int moto2)
{
	if(moto1>0) Ain1 = 1,Ain2 = 0;
	else        Ain1 = 0,Ain2 = 1;
	TIM_SetCompare1(TIM1, GFP_abs(moto1));
	
	
	if(moto2>0) Bin1 = 1,Bin2 = 0;
	else        Bin1 = 0,Bin2 = 1;
	TIM_SetCompare4(TIM1, GFP_abs(moto2));
	
}
