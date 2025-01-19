#ifndef __24L01_H
#define __24L01_H	 		  
#include "sys.h"   
#include <stdio.h>
#include <string.h>

//NRF24L01�Ĵ�����������
#define NRF_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
////////////////////////////////////////////////////////////////////////////////////////////////////////
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

/////////////////////  ���Խ������豸������ͬһ��SPI��������////////////////////////////
/////////////////////  ֱ������SCK MISO MOSI�������ż���

#define	NRF24L01_CE        PBout(0)
#define NRF24L01_IRQ 	   	 PAin(3)
#define	NRF24L01_CS        PAout(7)
#define	NRF24L01_SCK       PAout(6)
#define	NRF24L01_MISO      PAin(4)
#define	NRF24L01_MOSI      PAout(5)

//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��	 
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��   
									   	   
#define NRF_channel_RX 0x14     //�����ŵ�ȡֵ0-127
#define NRF_channel_TX 0x14  	//�����ŵ�ȡֵ0-127
#define NRF_Speed      0x06    //(����ģʽ��)�������� 1Mbps-0x06   2Mbps-0x0E  ����ģʽҪ+1��LNA����


unsigned char NRF24L01_SPI_RW(unsigned char data);
void NRF24L01_Init(void);						//��ʼ��
void NRF24L01_RX_Mode(void);					//����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);					//����Ϊ����ģʽ

unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char u8s);//д������
unsigned char NRF24L01_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char u8s);	//��������		  
unsigned char NRF24L01_Read_Reg(unsigned char reg);					//���Ĵ���
unsigned char NRF24L01_Write_Reg(unsigned char reg, unsigned char value);		//д�Ĵ���
unsigned char NRF24L01_Check(void);						//���24L01�Ƿ����
unsigned char NRF24L01_TxPacket(unsigned char *txbuf);				//����һ����������
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf);				//����һ����������


typedef struct
{
	unsigned char nrf24l01_TxBuff[32];
	unsigned char nrf24l01_RxBuff[32];
}nrf24l01TYPE;


typedef struct
{
	//����ң������������ֵ
	short Go_Forward;
	short Go_Backward;
	short Go_Turnleft;
	short Go_Turnright;

	
}Get_24L01_DataType;


extern nrf24l01TYPE nrf24l01;
extern Get_24L01_DataType Get_24L01_Data;
#endif











