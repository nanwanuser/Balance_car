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
extern float Angle_X_Final;  		//由加速度计算的x倾斜角度
extern float Angle_Y_Final;  		//由加速度计算的y倾斜角度
extern int Encoder_left,Encoder_right;
extern int Vertical_out,Velocity_out,Turn_out;
extern int PWM_out;
extern float Vertical_Kp;
int i=0;
extern float Med_Angle;

//2G4数据转换变量
Get_24L01_DataType Get_24L01_Data;
//24L01缓存数据变量
nrf24l01TYPE nrf24l01;



int main(void)
{
	delay_init();									//延时函数初始化
	NVIC_Configuration();					//中断分组初始化
	uart_init(115200);						//串口初始化
	
	LED_Init();
	
	OLED_Init();									//OLED初始化
	OLED_ColorTurn(0);						//0正常显示，1 反色显示
  OLED_DisplayTurn(0);					//0正常显示 1 屏幕翻转显示	

	MPU_Init();										//MPU6050初始化
	mpu_dmp_init();								//DMP库初始化	
	MPU6050_EXTI_Init();					//MPU6050外部中断初始化
	
	
	Encoder_TIM2_Init();          //TIM2编码器接口初始化
	Encoder_TIM4_Init();          //TIM4编码器接口初始化
	Motor_Init();									//电机正反转引脚初始化
	PWM_Init_TIM1(0,7199);				//TIM1——PWM输出初始化
	
	NRF24L01_Init();							//NRF24L01初始化
	NRF24L01_RX_Mode();					//配置为接收模式
	
	
	while(1)
	{					
		sprintf(string,"Pitch:");
		OLED_ShowString(0,24,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Pitch);
		OLED_ShowString(80,24,(u8*)string,8,1);		//OLED显示俯仰角
		sprintf(string,"Roll:");
		OLED_ShowString(0,32,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Roll);
		OLED_ShowString(80,32,(u8*)string,8,1);		//OLED显示翻滚角
		sprintf(string,"Yaw:");
		OLED_ShowString(0,40,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Yaw);
		OLED_ShowString(80,40,(u8*)string,8,1);		//OLED显示偏航角
		sprintf(string,"Med_Angle:");
		OLED_ShowString(0,48,(u8*)string,8,1);
		sprintf(string,"%.2f",(float)Med_Angle);	//OLED显示机械中值
		OLED_ShowString(80,48,(u8*)string,8,1);   
		OLED_Refresh();			
		NRF24L01_RxPacket(nrf24l01.nrf24l01_RxBuff);
		Get_24L01_Data.Go_Forward = (nrf24l01.nrf24l01_RxBuff[2]<<8) + nrf24l01.nrf24l01_RxBuff[1];
		Get_24L01_Data.Go_Backward = (nrf24l01.nrf24l01_RxBuff[4]<<8) + nrf24l01.nrf24l01_RxBuff[3];
		Get_24L01_Data.Go_Turnleft = (nrf24l01.nrf24l01_RxBuff[6]<<8) + nrf24l01.nrf24l01_RxBuff[5];
		Get_24L01_Data.Go_Turnright = (nrf24l01.nrf24l01_RxBuff[8]<<8) + nrf24l01.nrf24l01_RxBuff[7];	
		
	}
}

