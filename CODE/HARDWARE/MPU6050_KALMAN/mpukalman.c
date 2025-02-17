
/*
 * PB10 -> I2C2_SCL
 * PB11 -> I2C2_SDA
*/

#include "mpukalman.h"
#include "delay.h"
int Timeout = 10000;

//static void delay_us(uint32_t n)
//{
//	uint8_t j;
//	while(n--)
//	for(j=0;j<10;j++);
//}

//static void delay_ms(uint32_t n)
//{
//	while(n--)
//	delay_us(1000);
//}

void I2C2_Init(void)
{		
	
		I2C_InitTypeDef  I2C_InitStructure;
		GPIO_InitTypeDef  GPIO_InitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11; // here change the pins
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    
		I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    //I2C_InitStructure.I2C_OwnAddress1 = 0x30;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;//100K速度

    I2C_Cmd(I2C2, ENABLE);
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_AcknowledgeConfig(I2C2, ENABLE);
}

/*
 * 函数名：I2C_ByteWrite
 * 描述  ：写一个字节到I2C设备寄存器中
 * 输入  ：REG_Address 接收数据的IIC设备寄存器的地址 
 *         REG_data 待写入的数据
 * 输出  ：无
 * 返回  ：无
 * 调用  ：内部调用
 */	
void I2C_ByteWrite(uint8_t REG_Address, uint8_t REG_data)
{

				I2C_GenerateSTART(I2C2,ENABLE);

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

				I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Transmitter);

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

				I2C_SendData(I2C2,REG_Address);

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

				I2C_SendData(I2C2,REG_data);

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

				I2C_GenerateSTOP(I2C2,ENABLE);

}


/*
 * 函数名：I2C_ByteRead
 * 描述  ：从IIC设备寄存器中读取一个字节
 * 输入  ：REG_Address 读取数据的寄存器的地址 
 * 输出  ：无
 * 返回  ：无
 * 调用  ：内部调用 
*/
uint8_t I2C_ByteRead(uint8_t REG_Address)
{
				static uint8_t last = 0;	
				uint8_t REG_data = 0;
				Timeout = 10000;

				//while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));

				I2C_GenerateSTART(I2C2,ENABLE);//起始信号

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

				I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Transmitter);//发送设备地址+写信号

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
					Timeout --;
					if(Timeout == 0) return last;
							};//

				I2C_Cmd(I2C2,ENABLE);

				I2C_SendData(I2C2,REG_Address);//发送存储单元地址，从0开始

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

				I2C_GenerateSTART(I2C2,ENABLE);//起始信号

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

				I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Receiver);//发送设备地址+读信号

				while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

				I2C_AcknowledgeConfig(I2C2,DISABLE);

				I2C_GenerateSTOP(I2C2,ENABLE);

				while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));

				REG_data=I2C_ReceiveData(I2C2);//读出寄存器数据

				last = REG_data;

				return REG_data;

				}

/*
 * 函数名：void InitMPU6050(void)
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void MPU6050_Init(void)
{
			I2C2_Init();
			I2C_ByteWrite(PWR_MGMT_1,0x00);//解除休眠状态
			delay_ms(100);
			I2C_ByteWrite(SMPLRT_DIV,0x07);	 //IIC写入时的地址字节数据
			delay_ms(100);
			I2C_ByteWrite(CONFIG1,0x06);	 //低通滤波频率
			delay_ms(100);
			I2C_ByteWrite(GYRO_CONFIG,0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
			delay_ms(100);
			I2C_ByteWrite(ACCEL_CONFIG,0x01);//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
			delay_ms(100);
}


/*
 * 函数名：GetData
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 */

int16_t GetData(unsigned char REG_Address)
{
			char H,L;
			H=I2C_ByteRead(REG_Address);
			L=I2C_ByteRead(REG_Address+1);
			return (H<<8)+L;   //合成数据
}

int16_t MPU_get(unsigned char REG) {
			char H,L;
			H=I2C_ByteRead(REG);
			L=I2C_ByteRead(REG+1);
			return (H<<8)+L;   //合成数据
}

