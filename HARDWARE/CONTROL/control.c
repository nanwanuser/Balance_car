#include "control.h"
#include "sys.h"
#include "usart.h"
#include "ANO_Send.h"
#include "kalman.h"
#include "mpukalman.h"
#include <math.h>
#include "24l01.h"
float Med_Angle = -4.45;// ��е��ֵ
float Pitch, Roll, Yaw; // �Ƕ�
extern short gyrox, gyroy, gyroz; // �����ǡ������ٶ�
extern short aacx, aacy, aacz; // ���ٶ�
int PWM_out;

int Encoder_left,Encoder_right;
int Vertical_out,Velocity_out,Turn_out=0;

float Vertical_Kp=630,Vertical_Kd=1.51;// 630 1.51 

float Velocity_Kp=0.261,Velocity_Ki=0.0011 ,a=0;//0.261 0.0011
extern float Angle_x_temp;  		//�ɼ��ٶȼ����x��б�Ƕ�
extern float Angle_y_temp;  		//�ɼ��ٶȼ����y��б�Ƕ�

extern float Accel_x;	     		//X����ٶ�ֵ�ݴ�
extern float Accel_y;	    		//Y����ٶ�ֵ�ݴ�
extern float Accel_z;	     		//Z����ٶ�ֵ�ݴ�
extern float Gyro_x;				//X�������������ݴ�
extern float Gyro_y;        		//Y�������������ݴ�
extern float Gyro_z;		 		//Z�������������ݴ�
float accx,accy,accz;//������Ǽ��ٶ�ֵ
float Angle_y_temp_last;

//2G4����ת������
extern Get_24L01_DataType Get_24L01_Data;
//24L01�������ݱ���
extern nrf24l01TYPE nrf24l01;
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{	
		
		if(PAin(2) == 0)
		{
			EXTI_ClearITPendingBit(EXTI_Line2);
			//1.�ɼ����������� && MPU6050�Ƕ���Ϣ
			Encoder_left = Read_Speed(2);// �������԰�װ���պ����108�ȣ�Ϊ�˱������������һ�£�����Ҫ������һ��ȡ����
			Encoder_right = Read_Speed(4);
			
			mpu_dmp_get_data(&Pitch, &Roll, &Yaw);				// �Ƕ�
			MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		// �����ǡ������ٶ�
			
			//2.�����ݴ���ջ������У�����������������
			Vertical_out = Vertical(Med_Angle,Pitch,gyroy);
			Velocity_out = Velocity(Encoder_left,-Encoder_right);
			Turn_out = Turn(gyroz);
			
			PWM_out = Vertical_out + Vertical_Kp * Velocity_out;
			//3.�ѿ�����������ص�����ϣ�������յĿ���
			MOTO1 = PWM_out - Turn_out; 
			MOTO2 = PWM_out + Turn_out;
			if(Pitch>45 || Pitch<-45){MOTO1=0;MOTO2=0;}
			Limit(&MOTO1, &MOTO2);
			
			Load(-MOTO1,MOTO2);
		}
	}
	
}

/******************************
ֱ������Kp * Ek +Kd * Ek_D
��ڲ����������Ƕȣ���ʵ�Ƕȣ���ʵ���ٶ�
���ڣ�ֱ�������
*******************************/
int Vertical(float Med, float Angle, float gyrox)
{
	int PWM_out;
	
	
	PWM_out = Vertical_Kp * (Angle-Med_Angle) + Vertical_Kd * (gyrox - 0);//��0�����������ٶ�
	
	return PWM_out;
}



/******************************
�ٶȻ�PI:Kp * Ek + Ki*Ek_S
*******************************/
int Velocity(int encoder_left, int encoder_right)
{
	
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;
	
	
	
	//1.�����ٶ�ƫ��
	Encoder_Err = (encoder_left+encoder_right) - 0;//ע�ⲻҪ����2������ȥ���
	
	//2.���ٶ�ƫ����е�ͨ�˲�
	//low_out = (1 - a) * Ek + a * low_out_last
	EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	
	//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S += EnC_Err_Lowout;  
	Encoder_S =  Encoder_S + Get_24L01_Data.Go_Forward/18;
	//4.�����޷�
	Encoder_S = Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S); 
	//5.���ٶȻ������������
	PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
	return PWM_out;
}




/******************************
ת�򻷣�ϵ�� * Z����ٶ�
*******************************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out = (-0.1) * gyro_Z-Get_24L01_Data.Go_Turnright;
	return PWM_out;
}







