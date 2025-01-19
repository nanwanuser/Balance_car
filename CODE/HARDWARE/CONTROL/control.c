#include "control.h"
#include "sys.h"
#include "usart.h"
#include "ANO_Send.h"
#include "kalman.h"
#include "mpukalman.h"
#include <math.h>
#include "24l01.h"
float Med_Angle = -4.45;// 机械中值
float Pitch, Roll, Yaw; // 角度
extern short gyrox, gyroy, gyroz; // 陀螺仪——角速度
extern short aacx, aacy, aacz; // 加速度
int PWM_out;

int Encoder_left,Encoder_right;
int Vertical_out,Velocity_out,Turn_out=0;

float Vertical_Kp=630,Vertical_Kd=1.51;// 630 1.51 

float Velocity_Kp=0.261,Velocity_Ki=0.0011 ,a=0;//0.261 0.0011
extern float Angle_x_temp;  		//由加速度计算的x倾斜角度
extern float Angle_y_temp;  		//由加速度计算的y倾斜角度

extern float Accel_x;	     		//X轴加速度值暂存
extern float Accel_y;	    		//Y轴加速度值暂存
extern float Accel_z;	     		//Z轴加速度值暂存
extern float Gyro_x;				//X轴陀螺仪数据暂存
extern float Gyro_y;        		//Y轴陀螺仪数据暂存
extern float Gyro_z;		 		//Z轴陀螺仪数据暂存
float accx,accy,accz;//三方向角加速度值
float Angle_y_temp_last;

//2G4数据转换变量
extern Get_24L01_DataType Get_24L01_Data;
//24L01缓存数据变量
extern nrf24l01TYPE nrf24l01;
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{	
		
		if(PAin(2) == 0)
		{
			EXTI_ClearITPendingBit(EXTI_Line2);
			//1.采集编码器数据 && MPU6050角度信息
			Encoder_left = Read_Speed(2);// 电机是相对安装，刚好相差108度，为了编码器输出极性一致，就需要对其中一个取反。
			Encoder_right = Read_Speed(4);
			
			mpu_dmp_get_data(&Pitch, &Roll, &Yaw);				// 角度
			MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		// 陀螺仪——角速度
			
			//2.将数据传入闭环控制中，计算出控制输出量。
			Vertical_out = Vertical(Med_Angle,Pitch,gyroy);
			Velocity_out = Velocity(Encoder_left,-Encoder_right);
			Turn_out = Turn(gyroz);
			
			PWM_out = Vertical_out + Vertical_Kp * Velocity_out;
			//3.把控制输出量加载到电机上，完成最终的控制
			MOTO1 = PWM_out - Turn_out; 
			MOTO2 = PWM_out + Turn_out;
			if(Pitch>45 || Pitch<-45){MOTO1=0;MOTO2=0;}
			Limit(&MOTO1, &MOTO2);
			
			Load(-MOTO1,MOTO2);
		}
	}
	
}

/******************************
直立环：Kp * Ek +Kd * Ek_D
入口参数：期望角度，真实角度，真实角速度
出口：直立环输出
*******************************/
int Vertical(float Med, float Angle, float gyrox)
{
	int PWM_out;
	
	
	PWM_out = Vertical_Kp * (Angle-Med_Angle) + Vertical_Kd * (gyrox - 0);//“0”是期望角速度
	
	return PWM_out;
}



/******************************
速度环PI:Kp * Ek + Ki*Ek_S
*******************************/
int Velocity(int encoder_left, int encoder_right)
{
	
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;
	
	
	
	//1.计算速度偏差
	Encoder_Err = (encoder_left+encoder_right) - 0;//注意不要除以2，有舍去误差
	
	//2.对速度偏差进行低通滤波
	//low_out = (1 - a) * Ek + a * low_out_last
	EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	
	//3.对速度偏差积分，积分出位移
	Encoder_S += EnC_Err_Lowout;  
	Encoder_S =  Encoder_S + Get_24L01_Data.Go_Forward/18;
	//4.积分限幅
	Encoder_S = Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S); 
	//5.对速度环控制输出计算
	PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
	return PWM_out;
}




/******************************
转向环：系数 * Z轴角速度
*******************************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out = (-0.1) * gyro_Z-Get_24L01_Data.Go_Turnright;
	return PWM_out;
}







