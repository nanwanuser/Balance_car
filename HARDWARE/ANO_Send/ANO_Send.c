#include "usart.h"
#include "ANO_Send.h"
#include "stm32f10x.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 Data_to_Send[100];


void AnoTc_SendUserTest(s16 A, s16 B, s32 C)
{
	
	// AA 05 AF F1 02 E4 01 36
	unsigned char _cnt = 0;
	unsigned char i;
	unsigned char sumcheck = 0;
	unsigned char addcheck = 0;
	
	Data_to_Send[_cnt++] = 0xAA;
	Data_to_Send[_cnt++] = 0xFF;
	Data_to_Send[_cnt++] = 0xF1;
	Data_to_Send[_cnt++] = 8;
	Data_to_Send[_cnt++] = BYTE0(A);
	Data_to_Send[_cnt++] = BYTE1(A);
	Data_to_Send[_cnt++] = BYTE0(B);
	Data_to_Send[_cnt++] = BYTE1(B);
	Data_to_Send[_cnt++] = BYTE0(C);
	Data_to_Send[_cnt++] = BYTE1(C);
	Data_to_Send[_cnt++] = BYTE2(C);
	Data_to_Send[_cnt++] = BYTE3(C);
	
	for(i = 0; i < _cnt; i++)
	{
		sumcheck += Data_to_Send[i];
		addcheck += sumcheck;
	}
	//if(sumcheck == Data_to_Send[Data_to_Send[3]+4] && addcheck == Data_to_Send[Data_to_Send[3]+5])
	Data_to_Send[_cnt++]=sumcheck;
	Data_to_Send[_cnt++]=addcheck;
	for(i = 0;i < _cnt;i ++)
	{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
		USART_SendData(USART1, Data_to_Send[i]);  
		delay_us(500);
	}
}
