#include "24l01.h"
#include "delay.h"
#include "led.h" 
#include "usart.h"

const unsigned char TX_ADDRESS[TX_ADR_WIDTH]={0x20,0x97,0x07,0x28,0x00}; //���͵�ַ
const unsigned char RX_ADDRESS[RX_ADR_WIDTH]={0x20,0x97,0x07,0x28,0x00};


/*��ʼ��24L01��IO��
ʹ�õ�SPIͨ��Э��
ģ��SPIЭ�� �����IO�ڣ�
NRF_SCK   ->  PA6
NRF_CS    ->  PA7
NRF_CE    ->  PB0
NRF_IRQ   ->  PA3			��������
NRF_MISO  ->  PA4			��������
NRF_MOSI  ->  PA5
*/


void NRF24L01_Init(void)
{ 	
			GPIO_InitTypeDef   GPIO_InitStructure;
	
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

			//NRF_SCK  ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          		//���
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			//NRF_CS   ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;           	//���
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			//NRF_CE   ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;              //���
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			/////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////

			//NRF_IRQ  ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;           	//����
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			//NRF_MISO  ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;           	//����
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			//NRF_MOSI  ���ų�ʼ��
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;              //���
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			/////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////

			GPIO_SetBits(GPIOB,GPIO_Pin_10);//NRF_CS��������

			NRF24L01_CE = 0; 		//CE�����õͣ���������NRF�Ĵ���
			NRF24L01_CS = 1;    //��ֹSPI����	
	 		 	 
}
//���24L01�Ƿ����
//����ֵ:1���ɹ�;0��ʧ��	
unsigned char NRF24L01_Check(void)
{
	unsigned char buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	unsigned char i;
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)
	{
		if(buf[i] != 0XA5)
		{
			break;
		}
	}	 							   
	if(i!=5)
	{
		return 0;//���24L01����	
	}
	return 1;		 //��⵽24L01
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
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value)
{
	unsigned char status;	
	NRF24L01_CS = 0;	
	status =NRF24L01_SPI_RW(reg);//���ͼĴ����� 
	NRF24L01_SPI_RW(value);      //д��Ĵ�����ֵ
	NRF24L01_CS = 1;	  //�ر�SPI����	       
	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
unsigned char NRF24L01_Read_Reg(unsigned char reg)
{
	unsigned char reg_val;	    
 	NRF24L01_CS = 0;	       //ʹ��SPI����		
	NRF24L01_SPI_RW(reg);             //���ͼĴ�����
	reg_val=NRF24L01_SPI_RW(0XFF);    //��ȡ�Ĵ�������
	NRF24L01_CS = 1;	       //�ر�SPI����	    
	return(reg_val);         //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len)
{
	unsigned char status,u8_ctr;	       
	NRF24L01_CS = 0;	       //ʹ��SPI����
	status= NRF24L01_SPI_RW(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬  
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]= NRF24L01_SPI_RW(0XFF);//��������
	NRF24L01_CS = 1;	       //�ر�SPI����	
	return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
	unsigned char status,u8_ctr;	    
 	NRF24L01_CS = 0;	       //ʹ��SPI����
	status = NRF24L01_SPI_RW(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)NRF24L01_SPI_RW(*pBuf++); //д������	 
	NRF24L01_CS = 1;	       //�ر�SPI����
	return status;          //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	unsigned char sta;
	NRF24L01_Write_Reg(FLUSH_TX,0xff); //��շ���FIFOָ��	 /*   �����һ�η��͵�FIFO�Ĵ���  */
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); //��������жϱ�ʶ
	NRF24L01_CE=0;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
	NRF24L01_CE=1;
	while(NRF24L01_IRQ!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf)
{
	unsigned char sta;		
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_CE=0;			//SPIʹ��
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
		NRF24L01_CE=1; 
		return 0; 
	}	
	return 1;//û�յ��κ�����
}	

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;		

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA,0x01);    	//ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);	//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR,0x1A); //�Զ��ط�15��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH,NRF_channel_RX);	     	//����RFͨ��Ƶ��		  
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP,NRF_Speed+1);	//���÷������ʹ��ʣ�(����ģʽ��)�������� 1Mbps-0x06   2Mbps-0x0E  ����ģʽҪ+1��LNA����      
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG,0x0f);		//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);//дTX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ
	NRF24L01_CE = 1; 									//CEΪ��,�������ģʽ 
	delay_ms(1);
}		

//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;		
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);//дTX�ڵ��ַ
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA,0x01);    	//ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);	//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1A); //�Զ��ط�15��
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH,NRF_channel_RX);	     	//����RFͨ��Ƶ��		  
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP,NRF_Speed);	//���÷������ʹ��ʣ�(����ģʽ��)�������� 1Mbps-0x06   2Mbps-0x0E  ����ģʽҪ+1��LNA����  
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG,0x0E);		//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	NRF24L01_CE = 1; 									//CEΪ��,�������ģʽ 
	delay_ms(1);
}




