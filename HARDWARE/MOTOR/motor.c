#include "motor.h"

/*************************************************************************
*����������  ������ſ�������ת��ʼ��
*��ڲ�����  ��
*���ص�ֵ��  ��
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
*����������  PWM�޷�
*��ڲ�����  A����Ŀ����������B����Ŀ��������
*���ص�ֵ��  ��
***************************************************************************/

void Limit(int *motoA, int *motoB)
{
	if(*motoA > PWM_MAX)*motoA = PWM_MAX;
	if(*motoA < PWM_MIN)*motoA = PWM_MIN;
	
	if(*motoB > PWM_MAX)*motoB = PWM_MAX;
	if(*motoB < PWM_MIN)*motoB = PWM_MIN;
	
}
/*************************************************************************
*����������  ��PWM�����ת��Ϊ����ֵ���ü��ص������
*��ڲ�����  ��Ҫ����ֵ��������
*���ص�ֵ��  ����ֵ
***************************************************************************/
int GFP_abs(int p)
{
	int q;
	q = p>0?p:(-p);
	return q;
}
/*************************************************************************
*����������  ����PWM������������
*��ڲ�����  A����Ŀ����������B����Ŀ��������
*���ص�ֵ��  ��
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
