#include <stdint.h>


#include <rtthread.h>
#include "LDChip.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_spi.h"
#include "AsrItem.h"
#include "dfs_posix.h"
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include "dfs_posix.h"
#include "string.h"

/*************�˿���Ϣ********************
 * ����˵��
 * RST      PB6
 * CS  		 	PB8
 * WR/SPIS  PB9
 * P2/SDCK  PB3
 * P1/SDO   PB4
 * P0/SDI   PB5
 * IRQ      PC1
 * A0				PB7
 * RD       PA0
*****************************************/

#define CS_H()	GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define CS_L()	GPIO_ResetBits(GPIOB, GPIO_Pin_8)

#define SPIS_H()  GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define SPIS_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_9)

#define RST_H() GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define RST_L() GPIO_ResetBits(GPIOB, GPIO_Pin_6)


uint32 nMp3Size=0;				   //mp3�ļ��Ĵ�С
uint32 nMp3Pos=0;				   //mp3�ļ���ƫ��(����ƫ�ƾ����ļ���С)
uint8  nLD_Mode = LD_MODE_IDLE;	   //������¼��ǰ���ڽ���ASRʶ�����ڲ���MP3
uint8  bMp3Play=0;				   //������¼����MP3��״̬


void delay_ms(unsigned long uldata)
{
	rt_thread_delay(uldata);
}


uint8 nAsrStatus = 0;

static uint8 spi_send_byte(uint8 byte)
{
	/* ѭ����ⷢ�ͻ������Ƿ��ǿ� */
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

	/*ͨ��SPI3���跢������*/
	SPI_I2S_SendData(SPI3,byte);

	/* �ȴ��������ݣ�ѭ�����������ݻ����� */
	while (SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET);

	/* ���ض��������� */
	return SPI_I2S_ReceiveData(SPI3);
}

//дld3320�Ĵ���
void ld3320_write_reg(uint8 data1,uint8 data2)
{
	CS_L();

	SPIS_L();

	spi_send_byte(0x04);

	spi_send_byte(data1);

	spi_send_byte(data2);

	CS_H();

}

//��ld3320�Ĵ���
uint8 ld3320_read_reg(uint8 reg_add)
{
	uint8 i;

	CS_L();
	SPIS_L();

	spi_send_byte(0x05);

	spi_send_byte(reg_add);

	i=spi_send_byte(0);

	CS_H();

	return(i);
}

void ld3320_nvic_cfg(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  //�ж���������
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//�ⲿ�ж�������
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_GenerateSWInterrupt(EXTI_Line1);
		
	GPIO_SetBits(GPIOC,GPIO_Pin_1);	 //Ĭ�������ж�����

	EXTI_ClearFlag(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line1);
	//�ж�Ƕ������
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



void ld3320_spi_init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  //spi�˿�����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3 | RCC_APB2Periph_GPIOB,ENABLE);	   //ʹ��SPI3����ʱ��
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 

	//P0/P1/P2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//spis Ƭѡ	WR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	CS_H();
	
	//spi��������
	SPI_Cmd(SPI3, DISABLE);
	/* SPI3 ���� */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //ʱ�Ӽ��� ����״̬ʱ��SCK���ֵ͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //ʱ����λ ���ݲ����ӵ�һ��ʱ�ӱ��ؿ�ʼ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;							   //�������NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;   //�����ʿ��� SYSCLK/128
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //���ݸ�λ��ǰ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC����ʽ�Ĵ�����ʼֵΪ7
	SPI_Init(SPI3, &SPI_InitStructure);
	/* ʹ��SPI3 */
	SPI_Cmd(SPI3, ENABLE);
}


uint8 spi_read_byte(void)
{
	return (spi_send_byte(0));
}


void  ld3320_sclk_out(void)
	{	/* PA8 ���	 8M ���� */
	    GPIO_InitTypeDef GPIO_InitStructure;
	
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	    /*    MCO    configure */
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	    RCC_MCOConfig( RCC_MCO_PLLCLK_Div2);		//36M
	}
void ld3320_gpio_configuration(void)
{
    

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
	                        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD ,ENABLE);
		//LD_CS/A0/RSET
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6;//;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
	 	GPIO_SetBits(GPIOB,GPIO_Pin_7);	/*A0Ĭ������*/
				ld3320_sclk_out();
}


void LD_reset(void)	  //оƬ��λ
{
	RST_H();
	delay_ms(2);
	RST_L();
	delay_ms(2);
  RST_H();
	delay_ms(2);
	CS_L();
	delay_ms(2);
	CS_H();
	delay_ms(2);
}

/************************************************************************************************************/

void LD_Init_Common(void)	//ͨ�ó�ʼ��
{

	LD_reset();
	ld3320_read_reg(0x06);
	ld3320_write_reg(0x17, 0x35); //��λLD3320
	delay_ms(5);
	ld3320_read_reg(0x06);

	ld3320_write_reg(0x89, 0x03);  //ģ���·���� ��ʼ��ʱд03H
	delay_ms(5);
	ld3320_write_reg(0xCF, 0x43);  //�ڲ�ʡ��ģʽ���� ��ʼ��ʱд��43H
	delay_ms(5);
	ld3320_write_reg(0xCB, 0x02);

	/*PLL setting*/
	ld3320_write_reg(0x11, LD_PLL_11);   //ʱ��Ƶ������1
	if (nLD_Mode == LD_MODE_MP3)   //MP3����
	{
		ld3320_write_reg(0x1E, 0x00); 	//ADCר�ÿ��ƣ�Ӧ��ʼ��Ϊ00H
		ld3320_write_reg(0x19, LD_PLL_MP3_19);   //ʱ��Ƶ������2
		ld3320_write_reg(0x1B, LD_PLL_MP3_1B);   //ʱ��Ƶ������3
		ld3320_write_reg(0x1D, LD_PLL_MP3_1D);	//ʱ��Ƶ������4
	}
	else						   //����ʶ��
	{
		ld3320_write_reg(0x1E,0x00);	   //ADCר�ÿ��ƣ�Ӧ��ʼ��Ϊ00H
		ld3320_write_reg(0x19, LD_PLL_ASR_19); 	 //ʱ��Ƶ������2
		ld3320_write_reg(0x1B, LD_PLL_ASR_1B);	 //ʱ��Ƶ������3
		ld3320_write_reg(0x1D, LD_PLL_ASR_1D);	 //ʱ��Ƶ������4
	}
	delay_ms(5);

	ld3320_write_reg(0xCD, 0x04);	 //DSP�������� ��ʼ��ʱд��04H ����DSP����
	ld3320_write_reg(0x17, 0x4C); 	 //д4CH����ʹDSP���ߣ��Ƚ�ʡ��
	delay_ms(5);
	ld3320_write_reg(0xB9, 0x00);	 //ASR����ǰ���ʶ�����ַ������ȣ�ƴ���ַ�������ʼ��ʱд��00H	 ÿ���һ��ʶ����Ҫ�趨һ��
	ld3320_write_reg(0xCF, 0x4F); 	 //�ڲ�ʡ��ģʽ���� MP3��ʼ����ASR��ʼ��ʱд�� 4FH
	ld3320_write_reg(0x6F, 0xFF); 	 //��оƬ���г�ʼ��ʱ����Ϊ0xFF
}

uint8 RunASR(char cmd)
{
	uint8 i=0;
	uint8 j;
	uint8 asrflag=0;
	for (i=0; i<5; i++)			//	��ֹ����Ӳ��ԭ����LD3320оƬ����������������һ������5������ASRʶ������
	{
		LD_AsrStart();
		delay_ms(10);
		if (LD_AsrAddFixed(cmd)==0)
		{
			LD_reset();			//	LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			delay_ms(10);			//	���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}
		LD_TEST();
		delay_ms(10);
		j= LD_AsrRun();
		if (j == 0)
		{
			LD_reset();			//	LD3320оƬ�ڲ����ֲ���������������LD3320оƬ
			delay_ms(10);			//	���ӳ�ʼ����ʼ����ASRʶ������
			continue;
		}

		asrflag=1;
		break;					//	ASR���������ɹ����˳���ǰforѭ������ʼ�ȴ�LD3320�ͳ����ж��ź�
	}

	return asrflag;
}


void LD_Init_MP3(void)	//���ų�ʼ��
{
	nLD_Mode = LD_MODE_MP3;	   //��ǰ����MP3����
	LD_Init_Common();		   //ͨ�ó�ʼ��

	ld3320_write_reg(0xBD,0x02);	   //�ڲ�������� ��ʼ��ʱд��FFH
	ld3320_write_reg(0x17, 0x48);	//д48H���Լ���DSP
	delay_ms(10);

	ld3320_write_reg(0x85, 0x52); 	//�ڲ��������� ��ʼ��ʱд��52H
	ld3320_write_reg(0x8F, 0x00);  	//LineOut(��·���)ѡ�� ��ʼ��ʱд��00H
	ld3320_write_reg(0x81, 0x00);	//���������� ����Ϊ00HΪ�������
	ld3320_write_reg(0x83, 0x00);	//���������� ����Ϊ00HΪ�������
	ld3320_write_reg(0x8E, 0xff);	//�����������  ���Ĵ�������Ϊ00HΪ�������	�˴������ر�
	ld3320_write_reg(0x8D, 0xff);	//�ڲ�������� ��ʼ��ʱд��FFH
	delay_ms(10);
	ld3320_write_reg(0x87, 0xff);	//ģ���·���� MP3���ų�ʼ��ʱд FFH
	ld3320_write_reg(0x89, 0xff);   //ģ���·���� MP3����ʱд FFH
	delay_ms(10);
	ld3320_write_reg(0x22, 0x00);   //FIFO_DATA���޵�8λ
	ld3320_write_reg(0x23, 0x00);	//FIFO_DATA���޸�8λ
	ld3320_write_reg(0x20, 0xef);   //FIFO_DATA���޵�8λ
	ld3320_write_reg(0x21, 0x07);	//FIFO_DATA���޸�8λ
	ld3320_write_reg(0x24, 0x77);    
	ld3320_write_reg(0x25, 0x03);	
	ld3320_write_reg(0x26, 0xbb);    
	ld3320_write_reg(0x27, 0x01); 	
}

/* ����ʶ���ʼ�� */
void LD_Init_ASR(void)	   
{
	nLD_Mode=LD_MODE_ASR_RUN;	     //��ǰ��������ʶ��
	LD_Init_Common();	   			 //ͨ�ó�ʼ��

	ld3320_write_reg(0xBD, 0x00);	 //
	ld3320_write_reg(0x17, 0x48);	 //д48H���Լ���DSP
	delay_ms( 10 );

	ld3320_write_reg(0x3C, 0x80);
	ld3320_write_reg(0x3E, 0x07);
	ld3320_write_reg(0x38, 0xff);
	ld3320_write_reg(0x3A, 0x07);
 	delay_ms( 10 );

	ld3320_write_reg(0x40, 0);
	ld3320_write_reg(0x42, 8);
	ld3320_write_reg(0x44, 0);
	ld3320_write_reg(0x46, 8);
	delay_ms( 10 );
}


//����mp3׼��
void LD_play()	 
{
	nMp3Pos=0;
	bMp3Play=1;

	if (nMp3Pos >=  nMp3Size)
		return ; 

	fill_the_fifo();
//	LD_ReloadMp3Data();

    ld3320_write_reg(0xBA, 0x00);
	ld3320_write_reg(0x17, 0x48);
	ld3320_write_reg(0x33, 0x01);
	ld3320_write_reg(0x29, 0x04);
	
	ld3320_write_reg(0x02, 0x01); 
	ld3320_write_reg(0x85, 0x5A);

}

//��������
void LD_AdjustMIX2SPVolume(uint8 val)	  
{
	val = ((15-val)&0x0f) << 2;		  //
	ld3320_write_reg(0x8E, val | 0xc3);
	ld3320_write_reg(0x87, 0x78);
}


//����fifo�Ӷ�����fifo�ж�,Ϊ����mp3��׼��
void fill_the_fifo(void)
{
    uint8 ucStatus;
	int i = 0;
	ucStatus = ld3320_read_reg(0x06);
	//fifo�Ƿ�����
	while ( !(ucStatus&MASK_FIFO_STATUS_AFULL))
	{

		ld3320_write_reg(0x01,0xff);
		i++;
		ucStatus = ld3320_read_reg(0x06);
	}
	
}

//���оƬ�ڲ����޳���
// Return 1: success.
uint8 LD_Check_ASRBusyFlag_b2()	   
{
	uint8 j;
	uint8 flag = 0;
	for (j=0; j<10; j++)
	{
		if (ld3320_read_reg(0xb2) == 0x21)
		{
			flag = 1;
			break;
		}
		delay_ms(10);
	}
	return flag;
}

void LD_AsrStart()		
{
	LD_Init_ASR();
}

// Return 1: success.
uint8 LD_AsrRun()		//��ʼʶ��
{

	ld3320_write_reg(0x35, MIC_VOL);//��˷�����
	ld3320_write_reg(0xB3, 0xF);    //������


	ld3320_write_reg(0x1C, 0x09);	//ADC���ؿ��� д09H Reserve����������
	ld3320_write_reg(0xBD, 0x20);	//��ʼ�����ƼĴ��� д��20H��Reserve����������
	ld3320_write_reg(0x08, 0x01);	//���FIFO���ݵ�0λ��д��1�����FIFO_DATA ��2λ��д��1�����FIFO_EXT
	delay_ms( 1 );
	ld3320_write_reg(0x08, 0x00);	//���FIFO���ݵ�0λ�����ָ��FIFO����д��һ��00H��
	delay_ms( 1 );

	if (LD_Check_ASRBusyFlag_b2() == 0)	  //оƬ�ڲ�����
	{
		return 0;
	}

	ld3320_write_reg(0xB2, 0xff);	  //ASR��DSPæ��״̬ 0x21 ��ʾ�У���ѯ��Ϊ��״̬���Խ�����һ�� ??? Ϊʲô����read??
	ld3320_write_reg(0x37, 0x06);	  //ʶ����������·��Ĵ��� д06H ֪ͨDSP��ʼʶ������ �·�ǰ����Ҫ���B2�Ĵ���
	delay_ms( 5 );
	LD_TEST();
	ld3320_write_reg(0x1C, 0x0b);	  // ADC���ؿ���  д0BH ��˷�����ADCͨ������
	ld3320_write_reg(0x29, 0x10);	  //�ж����� ��2λ��FIFO �ж����� ��4λ��ͬ���ж����� 1����0������

	ld3320_write_reg(0xBD, 0x00);	  //��ʼ�����ƼĴ��� д��00 Ȼ������ ΪASRģ��


	return 1;
}

void LD_AsrAddFixed_ByString(char * pRecogString, uint8 k)
{
	uint8 nAsrAddLength;

	if (*pRecogString==0)
		return;

	ld3320_write_reg(0xc1, k );	   //ASR��ʶ����Index��00H��FFH��
	ld3320_write_reg(0xc3, 0 );	   //ASR��ʶ�������ʱд��00
	ld3320_write_reg(0x08, 0x04);	 //���FIFO���ݵ�0λ��д��1�����FIFO_DATA ��2λ��д��1�����FIFO_EXT
	delay_ms(1);
	ld3320_write_reg(0x08, 0x00);	 //���FIFO���ݵ�0λ�����ָ��FIFO����д��һ��00H��
	delay_ms(1);

	for (nAsrAddLength=0; nAsrAddLength<50; nAsrAddLength++)
	{
		if (pRecogString[nAsrAddLength] == 0)
			break;
		ld3320_write_reg(0x5, pRecogString[nAsrAddLength]);	  //дFIFO_EXT���ݿ�
	}

	ld3320_write_reg(0xb9, nAsrAddLength);	  //��ǰ���ʶ�����ַ������ȳ�ʼ��ʱд��00H ÿ���һ��ʶ����Ҫ�趨һ��
	ld3320_write_reg(0xb2, 0xff);	  //DSPæ��״̬ 0x21 ��ʾ�� ���Խ�����һ��	   ??
	ld3320_write_reg(0x37, 0x04);	  //����ʶ����������·��Ĵ��� д04H��֪ͨDSPҪ���һ��ʶ���
}


uint8 LD_GetResult()
{
	uint8 res;

	res = ld3320_read_reg(0xc5);

	return res;
}


void LD_TEST(void)
{
// ld3320_write_reg(0x35, 0x22);
// ld3320_write_reg(0x1b, 0x44);
// ld3320_write_reg(0xb3, 0x66);
// ld3320_read_reg(0xBF);
// ld3320_read_reg(0x06);
// ld3320_read_reg(0x35);
// ld3320_read_reg(0xb3);
//
}


static	int fd1,wu_fd;
uint8  LD_struct_index;
static	char line_data[100];
void LD_ReloadMp3Data_Again(void);
LD_struct ld_struct[LD_num];

int wugege_read_line(int fd_num)
{
		char *ptr;
		double result;
		double	read_num;
		read_num=read(fd_num, line_data, sizeof(line_data));
		if(read_num==-1||read_num==0)
			return -1;
		ptr=strchr(line_data, 0x0d);
		if(!ptr) 
			return -1;
		else 
		result=(ptr-line_data);
		*ptr='\0';
		if(read_num==(result+2))
			return 0;
		 lseek(fd_num,((result-read_num+2)), SEEK_CUR);
			return 1 ;

}
char wugege_read_ini(char *path)
{

	char *ptr=NULL;
	char *outer_ptr=NULL;
	char *inner_ptr=NULL;
	int cmp_result;
	char pecode[2];
	rt_kprintf("\r\n���翪ʼ��ȡ�����ļ�%s \r\n",path);
	ptr = strchr(path, '.');
	if(!ptr) return 0;
	cmp_result =strcmp(ptr, ".ini");	
	if(cmp_result)
		return 0;	
	wu_fd = open(path,0,0);
	if(wu_fd == -1)
	{
	   close(wu_fd);
	   return wu_fd;
	}
	 LD_struct_index=0;
	 memset(ld_struct,'\0',sizeof(LD_struct)*LD_num);
	while(wugege_read_line(wu_fd))
	{
		inner_ptr=line_data;
		outer_ptr=strchr(inner_ptr,':');
			strncpy(pecode,inner_ptr,(outer_ptr-inner_ptr));
	  if(pecode[1]<0x30)
		ld_struct[LD_struct_index].pCode= pecode[0]-0x30;
		else
			ld_struct[LD_struct_index].pCode=(pecode[0]-0x30)*10+(pecode[1]-0x30);
			outer_ptr++;
		inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
			strncpy(ld_struct[LD_struct_index].sRecog,inner_ptr,(outer_ptr-inner_ptr));
			outer_ptr++;
			inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
		strncpy(ld_struct[LD_struct_index].mp3_name,inner_ptr,(outer_ptr-inner_ptr));
		outer_ptr++;
			inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
		strncpy(ld_struct[LD_struct_index].next_ini,inner_ptr,(outer_ptr-inner_ptr));
			outer_ptr++;
			strcpy(ld_struct[LD_struct_index].action,outer_ptr);
			LD_struct_index++;
	};
				inner_ptr=line_data;
		outer_ptr=strchr(inner_ptr,':');
			strncpy(pecode,inner_ptr,(outer_ptr-inner_ptr));
	  if(pecode[1]<0x30)
		ld_struct[LD_struct_index].pCode= pecode[0]-0x30;
		else
			ld_struct[LD_struct_index].pCode=(pecode[0]-0x30)*10+(pecode[1]-0x30);
			outer_ptr++;
		inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
			strncpy(ld_struct[LD_struct_index].sRecog,inner_ptr,(outer_ptr-inner_ptr));
			outer_ptr++;
			inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
		strncpy(ld_struct[LD_struct_index].mp3_name,inner_ptr,(outer_ptr-inner_ptr));
		outer_ptr++;
			inner_ptr=outer_ptr;
		outer_ptr=strchr(inner_ptr,':');
		strncpy(ld_struct[LD_struct_index].next_ini,inner_ptr,(outer_ptr-inner_ptr));
			outer_ptr++;
			strcpy(ld_struct[LD_struct_index].action,outer_ptr);
	close(wu_fd);
	rt_kprintf("����: ����ȡ%d������ָ��\r\n",LD_struct_index);
	return 1;
}
/*******************************************************************************************************************/


// Return 1: success.
//	���ʶ��ؼ���������߿���ѧϰ"����ʶ��оƬLD3320�߽��ؼ�.pdf"�й��������������մ�����÷�
uint8 LD_AsrAddFixed(char cmd)	  //��ӹؼ����ﵽLD3320оƬ��
{
	//�ȼ�鴫����ļ����Ƿ���Ϲ淶
	char i;
	for(i=0;i<=LD_struct_index;i++)
		{
		rt_kprintf("����û���:%d ָ��: %s  ��������:%s �����ļ�:%s ִ�ж���:%s  \r\n",ld_struct[i].pCode,ld_struct[i].sRecog,ld_struct[i].mp3_name,ld_struct[i].next_ini,ld_struct[i].action);
	LD_AsrAddFixed_ByString(ld_struct[i].sRecog,ld_struct[i].pCode);
		}
			wu_fd = open(Name_ini,0,0);
		i = get_file_size(wu_fd);
	read(wu_fd, line_data, sizeof(line_data));
		line_data[i]='\0';
		close(wu_fd);
		rt_kprintf("����û���: 49 �û���: %s  \r\n",line_data);
		LD_AsrAddFixed_ByString(line_data,49);  //����û���
	return 1;

}
char get_mps_ini_file(char *mp3_name,char *ini_name,char *action,char index)
{
		if(index>LD_struct_index)  return 0;
		strcpy(mp3_name,ld_struct[index].mp3_name);
		strcpy(ini_name,ld_struct[index].next_ini);
		strcpy(action,ld_struct[index].action);
		return 1;
}
uint8 ucRegVal;	  //�Ĵ������ݱ���
uint8 ucHighInt;  //�Ĵ������ݱ���
uint8 ucLowInt;	  //�Ĵ������ݱ���




void ProcessInt0(void)	  //���� ����ʶ���ж�
{
	uint8 nAsrResCount=0;

	ucRegVal = ld3320_read_reg(0x2B);

	if (nLD_Mode == LD_MODE_ASR_RUN)		//��ǰ��������ʶ��
	{
		// ����ʶ��������ж�
		// �����������룬����ʶ��ɹ���ʧ�ܶ����жϣ�
		ld3320_write_reg(0x29,0) ;				     //�ж����� FIFO �ж����� 0��ʾ������
		ld3320_write_reg(0x02,0) ;		             // FIFO�ж�����	 FIFO_DATA FIFO_EXT�ж�   ������
		if ((ucRegVal & 0x10) &&		             //2b����λΪ1 оƬ�ڲ�FIFO�жϷ��� MP3����ʱ������жϱ�־�����ⲿMCU��FIFO_DATA��Reload��
		     ld3320_read_reg(0xb2)==0x21 && 	     //��b2�õ�0x21��ʾ�п��Խ�����һ��ASR����
		     ld3320_read_reg(0xbf)==0x35)		     //������ֵΪ0x35������ȷ����һ������ʶ��������������
		{
			nAsrResCount = ld3320_read_reg(0xba);    //ASR���ж�ʱ���ж�����ʶ���м���ʶ���ѡ
			if (nAsrResCount==1)
			{
				ld3320_read_reg(0xc5);
			}
			if (nAsrResCount>0 && nAsrResCount<=4) 	 //1 �C 4: N��ʶ���ѡ 0���ߴ���4��û��ʶ���ѡ
			{
				nAsrStatus=LD_ASR_FOUNDOK;		     //��ʾһ��ʶ�����̽�������һ��ʶ����
			}
			else
			{
				nAsrStatus=LD_ASR_FOUNDZERO;	     //��ʾһ��ʶ�����̽�����û��ʶ����
			}
		}
		else
		{
			nAsrStatus=LD_ASR_FOUNDZERO;	         //��ʾһ��ʶ�����̽�����û��ʶ����
		}

		ld3320_write_reg(0x2b, 0);
		ld3320_write_reg(0x1C,0);	  				 //ADC���ؿ��� д00H ADC������
		return;
	}

	// �������Ų������жϣ������֣�
	// A. ����������ȫ�������ꡣ
	// B. ���������ѷ�����ϡ�
	// C. ����������ʱ��Ҫ���꣬��Ҫ�����µ����ݡ�	
	ucHighInt = ld3320_read_reg(0x29); 
	ucLowInt=ld3320_read_reg(0x02); 
	ld3320_write_reg(0x29,0) ;
	ld3320_write_reg(0x02,0) ;
    if(ld3320_read_reg(0xBA)&CAUSE_MP3_SONG_END)
    {
	// A. ����������ȫ�������ꡣ

		ld3320_write_reg(0x2B,  0);
      	ld3320_write_reg(0xBA, 0);	
		ld3320_write_reg(0xBC,0x0);	
		bMp3Play=0;					// ��������ȫ����������޸�bMp3Play�ı���
		ld3320_write_reg(0x08,1);

      	ld3320_write_reg(0x08,0);
		ld3320_write_reg(0x33, 0);
		close(fd1);
		return ;
     }

	if(nMp3Pos>=nMp3Size)
	{
	// B. ���������ѷ�����ϡ�

		ld3320_write_reg(0xBC, 0x01);
		ld3320_write_reg(0x29, 0x10);
		if(fd1!=0)
		close(fd1);
		return;	
	}

	// C. ����������ʱ��Ҫ���꣬��Ҫ�����µ����ݡ�	

	LD_ReloadMp3Data_Again();
			
	ld3320_write_reg(0x29,ucHighInt); 
	ld3320_write_reg(0x02,ucLowInt) ;

}

struct rt_event event;
void EXTI1_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line1)!= RESET ) 
	{
			//ProcessInt0(); 
		EXTI_ClearFlag(EXTI_Line1);
		EXTI_ClearITPendingBit(EXTI_Line1);
		rt_event_send(&event,(1 << 0));
	}
}
   
/*  ������ȡmp3�ļ����ݵ�fifo,ֱ��fifo��
 *	��дmp3�ļ����ݵ�fifoʱ,LD3320�����벥��
 *	��Ȼдmp3�ļ����ݵ�fifo��ʱ���̹�������ʱ��
 *	�������첥����ϵ�ʱ������ProcessInt0����
 *	ProcessInt0�����ֻ���ô˺���,����������������
 */
void LD_ReloadMp3Data_Again(void)
{
	uint8 val;
    uint8 ucStatus;
//	int result,i,j;
	//rt_kprintf("���ڼ���MP3�ļ�\n");
	ucStatus = ld3320_read_reg(0x06);
	//fifo�Ƿ�����
	while (!(ucStatus&MASK_FIFO_STATUS_AFULL) && nMp3Pos<=nMp3Size)
	{

		nMp3Pos++;
		read(fd1,&val,1);
	//	val++;
		ld3320_write_reg(0x01,val);
		ucStatus = ld3320_read_reg(0x06);
	
	}
//	rt_kprintf("����MP3�ļ���� !\n");

	if(nMp3Pos>=nMp3Size)
	{
	//rt_kprintf("�����ļ��ļ���� !\n");
	    ld3320_write_reg(0xBC, 0x01);
		ld3320_write_reg(0x29, 0x10);

		//�ȴ�MP3�������
		while(!(ld3320_read_reg(0xBA)&CAUSE_MP3_SONG_END));

		ld3320_write_reg(0x2B,  0);
      	ld3320_write_reg(0xBA, 0);	
		ld3320_write_reg(0xBC,0x0);	
		bMp3Play=0;					// ��������ȫ����������޸�bMp3Play�ı���
		ld3320_write_reg(0x08,1);

      	ld3320_write_reg(0x08,0);
		ld3320_write_reg(0x33, 0);
		//rt_kprintf("����mp3��ϣ��ر������ļ�!\n");
		close(fd1);
		//rt_kprintf("������ϣ��ر������ļ�ϵͳ�ɹ�!\n");

	}
		
}

/*
 * ����mp3ʱ���ô˺�������
 */
int PlayDemoSound_mp3(char *path)
{
	rt_kprintf("PlayDemoSound_mp3 path is %s\r\n",path);
	fd1 = open(path,0,0);
	if(fd1 == -1)
	{
	   close(fd1);
	   return fd1;
	}

	bMp3Play = 1;

	nMp3Size = get_file_size(fd1)-1;

	//��LD3320��ʼ��Ϊ����MP3ģʽ
	LD_Init_MP3();
	//���ö�������
	LD_AdjustMIX2SPVolume(7);
	//��ʼ����
	LD_play();

    return 0;
}


