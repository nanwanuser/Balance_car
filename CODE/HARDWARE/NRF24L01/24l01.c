#include "24l01.h"
#include "delay.h"
#include "led.h" 
#include "usart.h"

const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x20,0x97,0x07,0x28,0x00}; //发送地址
const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x20,0x97,0x07,0x28,0x00};


/*初始化24L01的IO口
使用的SPI通信协议
模拟SPI协议 分配的IO口：
NRF_SCK   ->  PA6
NRF_CS    ->  PA7
NRF_CE    ->  PB0
NRF_IRQ   ->  PA3			浮空输入
NRF_MISO  ->  PA4			浮空输入
NRF_MOSI  ->  PA5
*/


void NRF24L01_Init(void)
{ 	
			GPIO_InitTypeDef   GPIO_InitStructure;
	
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

			//NRF_SCK  引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          		//输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			//NRF_CS   引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;           	//输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			//NRF_CE   引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;              //输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			/////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////

			//NRF_IRQ  引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;           	//输入
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			//NRF_MISO  引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;           	//输入
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			//NRF_MOSI  引脚初始化
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;              //输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			/////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////

			GPIO_SetBits(GPIOB,GPIO_Pin_10);//NRF_CS引脚拉高

			NRF24L01_CE = 0; 		//CE引脚置低，方可配置NRF寄存器
			NRF24L01_CS = 1;    //禁止SPI传输	
	 		 	 
}
//检测24L01是否存在
//返回值:1，成功;0，失败	
unsigned char NRF24L01_Check(void)
{
	unsigned char buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	unsigned char i;
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)
	{
		if(buf[i] != 0XA5)
		{
			break;
		}
	}	 							   
	if(i!=5)
	{
		return 0;//检测24L01错误	
	}
	return 1;		 //检测到24L01
}	 	 
unsigned char NRF24L01_SPI_RW(unsigned char data)
{
	unsigned char bit;
	for(bit=0;bit<8;bit++)
	{
		NRF24L01_SCK = 0;
		if(data & 0x80)
		{
			NRF24L01_MOSI = 1;
		}
		else	
		{
			NRF24L01_MOSI = 0;
		}	
		delay_us(1);
		NRF24L01_SCK = 1;
		data = data<<1;
		data|= NRF24L01_MISO ;
		delay_us(1);
		NRF24L01_SCK = 0;
	}
	return data ;
}
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value)
{
	unsigned char status;	
	NRF24L01_CS = 0;	
	status =NRF24L01_SPI_RW(reg);//发送寄存器号 
	NRF24L01_SPI_RW(value);      //写入寄存器的值
	NRF24L01_CS = 1;	  //关闭SPI传输	       
	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
unsigned char NRF24L01_Read_Reg(unsigned char reg)
{
	unsigned char reg_val;	    
 	NRF24L01_CS = 0;	       //使能SPI传输		
	NRF24L01_SPI_RW(reg);             //发送寄存器号
	reg_val=NRF24L01_SPI_RW(0XFF);    //读取寄存器内容
	NRF24L01_CS = 1;	       //关闭SPI传输	    
	return(reg_val);         //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len)
{
	unsigned char status,u8_ctr;	       
	NRF24L01_CS = 0;	       //使能SPI传输
	status= NRF24L01_SPI_RW(reg);//发送寄存器值(位置),并读取状态值  
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]= NRF24L01_SPI_RW(0XFF);//读出数据
	NRF24L01_CS = 1;	       //关闭SPI传输	
	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
	unsigned char status,u8_ctr;	    
 	NRF24L01_CS = 0;	       //使能SPI传输
	status = NRF24L01_SPI_RW(reg);//发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)NRF24L01_SPI_RW(*pBuf++); //写入数据	 
	NRF24L01_CS = 1;	       //关闭SPI传输
	return status;          //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	unsigned char sta;
	NRF24L01_Write_Reg(FLUSH_TX,0xff); //清空发送FIFO指令	 /*   清除上一次发送的FIFO寄存器  */
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); //清除所有中断标识
	NRF24L01_CE=0;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	NRF24L01_CE=1;
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf)
{
	unsigned char sta;		
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_CE=0;			//SPI使能
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
		NRF24L01_CE=1; 
		return 0; 
	}	
	return 1;//没收到任何数据
}	

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;		

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA,0x01);    	//使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);	//使能通道0的接收地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR,0x1A); //自动重发15次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH,NRF_channel_RX);	     	//设置RF通信频率		  
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP,NRF_Speed+1);	//设置发射速率功率，(发射模式下)无线速率 1Mbps-0x06   2Mbps-0x0E  接收模式要+1把LNA给打开      
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG,0x0f);		//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);//写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
	NRF24L01_CE = 1; 									//CE为高,进入接收模式 
	delay_ms(1);
}		

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;		
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);//写TX节点地址
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA,0x01);    	//使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);	//使能通道0的接收地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1A); //自动重发15次
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH,NRF_channel_RX);	     	//设置RF通信频率		  
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP,NRF_Speed);	//设置发射速率功率，(发射模式下)无线速率 1Mbps-0x06   2Mbps-0x0E  接收模式要+1把LNA给打开  
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);		//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF24L01_CE = 1; 									//CE为高,进入接收模式 
	delay_ms(1);
}




