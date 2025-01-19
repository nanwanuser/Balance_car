#include "sys.h"
#include "oled.h"
#include "bmp.h"
#include <stdio.h>
#include "usart.h"
#include "ANO_Send.h"
#include "LED.h"
#include "kalman.h"
#include "mpukalman.h"
#include "control.h"
#include "24l01.h"

int PWM_MAX = 7200,PWM_MIN = -7200;
int MOTO1,MOTO2;
char string[20];
int check;
extern float Angle_X_Final;  		//�ɼ��ٶȼ����x��б�Ƕ�
extern float Angle_Y_Final;  		//�ɼ��ٶȼ����y��б�Ƕ�
extern int Encoder_left,Encoder_right;
extern int Vertical_out,Velocity_out,Turn_out;
extern int PWM_out;
extern float Vertical_Kp;
int i=0;
extern float Med_Angle;

//2G4����ת������
Get_24L01_DataType Get_24L01_Data;
//24L01�������ݱ���
nrf24l01TYPE nrf24l01;



int main(void)
{
	delay_init();									//��ʱ������ʼ��
	NVIC_Configuration();					//�жϷ����ʼ��
	uart_init(115200);						//���ڳ�ʼ��
	
	LED_Init();
	
	OLED_Init();									//OLED��ʼ��
	OLED_ColorTurn(0);						//0������ʾ��1 ��ɫ��ʾ
  OLED_DisplayTurn(0);					//0������ʾ 1 ��Ļ��ת��ʾ	

	MPU_Init();										//MPU6050��ʼ��
	mpu_dmp_init();								//DMP���ʼ��	
	MPU6050_EXTI_Init();					//MPU6050�ⲿ�жϳ�ʼ��
	
	
	Encoder_TIM2_Init();          //TIM2�������ӿڳ�ʼ��
	Encoder_TIM4_Init();          //TIM4�������ӿڳ�ʼ��
	Motor_Init();									//�������ת���ų�ʼ��
	PWM_Init_TIM1(0,7199);				//TIM1����PWM�����ʼ��
	
	NRF24L01_Init();							//NRF24L01��ʼ��
	NRF24L01_RX_Mode();					//����Ϊ����ģʽ
	
	
	while(1)
	{					
		sprintf(string,"Pitch:");
		OLED_ShowString(0,24,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Pitch);
		OLED_ShowString(80,24,(u8*)string,8,1);		//OLED��ʾ������
		sprintf(string,"Roll:");
		OLED_ShowString(0,32,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Roll);
		OLED_ShowString(80,32,(u8*)string,8,1);		//OLED��ʾ������
		sprintf(string,"Yaw:");
		OLED_ShowString(0,40,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Yaw);
		OLED_ShowString(80,40,(u8*)string,8,1);		//OLED��ʾƫ����
		sprintf(string,"Med_Angle:");
		OLED_ShowString(0,48,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Med_Angle);	//OLED��ʾ��е��ֵ
		OLED_ShowString(80,48,(u8*)string,8,1);   
		OLED_Refresh();			
		NRF24L01_RxPacket(nrf24l01.nrf24l01_RxBuff);
		Get_24L01_Data.Go_Forward = (nrf24l01.nrf24l01_RxBuff[2]<<8) + nrf24l01.nrf24l01_RxBuff[1];
		Get_24L01_Data.Go_Backward = (nrf24l01.nrf24l01_RxBuff[4]<<8) + nrf24l01.nrf24l01_RxBuff[3];
		Get_24L01_Data.Go_Turnleft = (nrf24l01.nrf24l01_RxBuff[6]<<8) + nrf24l01.nrf24l01_RxBuff[5];
		Get_24L01_Data.Go_Turnright = (nrf24l01.nrf24l01_RxBuff[8]<<8) + nrf24l01.nrf24l01_RxBuff[7];	
		
	}
}

