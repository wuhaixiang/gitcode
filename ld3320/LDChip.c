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

/*************端口信息********************
 * 接线说明
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


uint32 nMp3Size=0;				   //mp3文件的大小
uint32 nMp3Pos=0;				   //mp3文件的偏移(最后的偏移就是文件大小)
uint8  nLD_Mode = LD_MODE_IDLE;	   //用来记录当前是在进行ASR识别还是在播放MP3
uint8  bMp3Play=0;				   //用来记录播放MP3的状态


void delay_ms(unsigned long uldata)
{
	rt_thread_delay(uldata);
}


uint8 nAsrStatus = 0;

static uint8 spi_send_byte(uint8 byte)
{
	/* 循环检测发送缓冲区是否是空 */
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

	/*通过SPI3外设发出数据*/
	SPI_I2S_SendData(SPI3,byte);

	/* 等待接收数据，循环检查接收数据缓冲区 */
	while (SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET);

	/* 返回读出的数据 */
	return SPI_I2S_ReceiveData(SPI3);
}

//写ld3320寄存器
void ld3320_write_reg(uint8 data1,uint8 data2)
{
	CS_L();

	SPIS_L();

	spi_send_byte(0x04);

	spi_send_byte(data1);

	spi_send_byte(data2);

	CS_H();

}

//读ld3320寄存器
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

  //中断引脚配置
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//外部中断线配置
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_GenerateSWInterrupt(EXTI_Line1);
		
	GPIO_SetBits(GPIOC,GPIO_Pin_1);	 //默认拉高中断引脚

	EXTI_ClearFlag(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line1);
	//中断嵌套配置
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
  //spi端口配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3 | RCC_APB2Periph_GPIOB,ENABLE);	   //使能SPI3外设时钟
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 

	//P0/P1/P2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//spis 片选	WR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	CS_H();
	
	//spi功能配置
	SPI_Cmd(SPI3, DISABLE);
	/* SPI3 配置 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   //全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //时钟极性 空闲状态时，SCK保持低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //时钟相位 数据采样从第一个时钟边沿开始
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;							   //软件产生NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;   //波特率控制 SYSCLK/128
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //数据高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC多项式寄存器初始值为7
	SPI_Init(SPI3, &SPI_InitStructure);
	/* 使能SPI3 */
	SPI_Cmd(SPI3, ENABLE);
}


uint8 spi_read_byte(void)
{
	return (spi_send_byte(0));
}


void  ld3320_sclk_out(void)
	{	/* PA8 输出	 8M 波形 */
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
	 	GPIO_SetBits(GPIOB,GPIO_Pin_7);	/*A0默认拉高*/
				ld3320_sclk_out();
}


void LD_reset(void)	  //芯片复位
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

void LD_Init_Common(void)	//通用初始化
{

	LD_reset();
	ld3320_read_reg(0x06);
	ld3320_write_reg(0x17, 0x35); //软复位LD3320
	delay_ms(5);
	ld3320_read_reg(0x06);

	ld3320_write_reg(0x89, 0x03);  //模拟电路控制 初始化时写03H
	delay_ms(5);
	ld3320_write_reg(0xCF, 0x43);  //内部省电模式设置 初始化时写入43H
	delay_ms(5);
	ld3320_write_reg(0xCB, 0x02);

	/*PLL setting*/
	ld3320_write_reg(0x11, LD_PLL_11);   //时钟频率设置1
	if (nLD_Mode == LD_MODE_MP3)   //MP3播放
	{
		ld3320_write_reg(0x1E, 0x00); 	//ADC专用控制，应初始化为00H
		ld3320_write_reg(0x19, LD_PLL_MP3_19);   //时钟频率设置2
		ld3320_write_reg(0x1B, LD_PLL_MP3_1B);   //时钟频率设置3
		ld3320_write_reg(0x1D, LD_PLL_MP3_1D);	//时钟频率设置4
	}
	else						   //语音识别
	{
		ld3320_write_reg(0x1E,0x00);	   //ADC专用控制，应初始化为00H
		ld3320_write_reg(0x19, LD_PLL_ASR_19); 	 //时钟频率设置2
		ld3320_write_reg(0x1B, LD_PLL_ASR_1B);	 //时钟频率设置3
		ld3320_write_reg(0x1D, LD_PLL_ASR_1D);	 //时钟频率设置4
	}
	delay_ms(5);

	ld3320_write_reg(0xCD, 0x04);	 //DSP休眠设置 初始化时写入04H 允许DSP休眠
	ld3320_write_reg(0x17, 0x4C); 	 //写4CH可以使DSP休眠，比较省电
	delay_ms(5);
	ld3320_write_reg(0xB9, 0x00);	 //ASR：当前添加识别句的字符串长度（拼音字符串）初始化时写入00H	 每添加一条识别句后要设定一次
	ld3320_write_reg(0xCF, 0x4F); 	 //内部省电模式设置 MP3初始化和ASR初始化时写入 4FH
	ld3320_write_reg(0x6F, 0xFF); 	 //对芯片进行初始化时设置为0xFF
}

uint8 RunASR(char cmd)
{
	uint8 i=0;
	uint8 j;
	uint8 asrflag=0;
	for (i=0; i<5; i++)			//	防止由于硬件原因导致LD3320芯片工作不正常，所以一共尝试5次启动ASR识别流程
	{
		LD_AsrStart();
		delay_ms(10);
		if (LD_AsrAddFixed(cmd)==0)
		{
			LD_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			delay_ms(10);			//	并从初始化开始重新ASR识别流程
			continue;
		}
		LD_TEST();
		delay_ms(10);
		j= LD_AsrRun();
		if (j == 0)
		{
			LD_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			delay_ms(10);			//	并从初始化开始重新ASR识别流程
			continue;
		}

		asrflag=1;
		break;					//	ASR流程启动成功，退出当前for循环。开始等待LD3320送出的中断信号
	}

	return asrflag;
}


void LD_Init_MP3(void)	//播放初始化
{
	nLD_Mode = LD_MODE_MP3;	   //当前进行MP3播放
	LD_Init_Common();		   //通用初始化

	ld3320_write_reg(0xBD,0x02);	   //内部增益控制 初始化时写入FFH
	ld3320_write_reg(0x17, 0x48);	//写48H可以激活DSP
	delay_ms(10);

	ld3320_write_reg(0x85, 0x52); 	//内部反馈设置 初始化时写入52H
	ld3320_write_reg(0x8F, 0x00);  	//LineOut(线路输出)选择 初始化时写入00H
	ld3320_write_reg(0x81, 0x00);	//耳机左音量 设置为00H为最大音量
	ld3320_write_reg(0x83, 0x00);	//耳机右音量 设置为00H为最大音量
	ld3320_write_reg(0x8E, 0xff);	//喇叭输出音量  本寄存器设置为00H为最大音量	此处声音关闭
	ld3320_write_reg(0x8D, 0xff);	//内部增益控制 初始化时写入FFH
	delay_ms(10);
	ld3320_write_reg(0x87, 0xff);	//模拟电路控制 MP3播放初始化时写 FFH
	ld3320_write_reg(0x89, 0xff);   //模拟电路控制 MP3播放时写 FFH
	delay_ms(10);
	ld3320_write_reg(0x22, 0x00);   //FIFO_DATA下限低8位
	ld3320_write_reg(0x23, 0x00);	//FIFO_DATA下限高8位
	ld3320_write_reg(0x20, 0xef);   //FIFO_DATA上限低8位
	ld3320_write_reg(0x21, 0x07);	//FIFO_DATA上限高8位
	ld3320_write_reg(0x24, 0x77);    
	ld3320_write_reg(0x25, 0x03);	
	ld3320_write_reg(0x26, 0xbb);    
	ld3320_write_reg(0x27, 0x01); 	
}

/* 语音识别初始化 */
void LD_Init_ASR(void)	   
{
	nLD_Mode=LD_MODE_ASR_RUN;	     //当前进行语音识别
	LD_Init_Common();	   			 //通用初始化

	ld3320_write_reg(0xBD, 0x00);	 //
	ld3320_write_reg(0x17, 0x48);	 //写48H可以激活DSP
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


//播放mp3准备
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

//音量调整
void LD_AdjustMIX2SPVolume(uint8 val)	  
{
	val = ((15-val)&0x0f) << 2;		  //
	ld3320_write_reg(0x8E, val | 0xc3);
	ld3320_write_reg(0x87, 0x78);
}


//填满fifo从而触发fifo中断,为播放mp3做准备
void fill_the_fifo(void)
{
    uint8 ucStatus;
	int i = 0;
	ucStatus = ld3320_read_reg(0x06);
	//fifo是否满了
	while ( !(ucStatus&MASK_FIFO_STATUS_AFULL))
	{

		ld3320_write_reg(0x01,0xff);
		i++;
		ucStatus = ld3320_read_reg(0x06);
	}
	
}

//检测芯片内部有无出错
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
uint8 LD_AsrRun()		//开始识别
{

	ld3320_write_reg(0x35, MIC_VOL);//麦克风音量
	ld3320_write_reg(0xB3, 0xF);    //灵敏度


	ld3320_write_reg(0x1C, 0x09);	//ADC开关控制 写09H Reserve保留命令字
	ld3320_write_reg(0xBD, 0x20);	//初始化控制寄存器 写入20H；Reserve保留命令字
	ld3320_write_reg(0x08, 0x01);	//清除FIFO内容第0位：写入1→清除FIFO_DATA 第2位：写入1→清除FIFO_EXT
	delay_ms( 1 );
	ld3320_write_reg(0x08, 0x00);	//清除FIFO内容第0位（清除指定FIFO后再写入一次00H）
	delay_ms( 1 );

	if (LD_Check_ASRBusyFlag_b2() == 0)	  //芯片内部出错
	{
		return 0;
	}

	ld3320_write_reg(0xB2, 0xff);	  //ASR：DSP忙闲状态 0x21 表示闲，查询到为闲状态可以进行下一步 ??? 为什么不是read??
	ld3320_write_reg(0x37, 0x06);	  //识别控制命令下发寄存器 写06H 通知DSP开始识别语音 下发前，需要检查B2寄存器
	delay_ms( 5 );
	LD_TEST();
	ld3320_write_reg(0x1C, 0x0b);	  // ADC开关控制  写0BH 麦克风输入ADC通道可用
	ld3320_write_reg(0x29, 0x10);	  //中断允许 第2位：FIFO 中断允许 第4位：同步中断允许 1允许；0不允许

	ld3320_write_reg(0xBD, 0x00);	  //初始化控制寄存器 写入00 然后启动 为ASR模块


	return 1;
}

void LD_AsrAddFixed_ByString(char * pRecogString, uint8 k)
{
	uint8 nAsrAddLength;

	if (*pRecogString==0)
		return;

	ld3320_write_reg(0xc1, k );	   //ASR：识别字Index（00H—FFH）
	ld3320_write_reg(0xc3, 0 );	   //ASR：识别字添加时写入00
	ld3320_write_reg(0x08, 0x04);	 //清除FIFO内容第0位：写入1→清除FIFO_DATA 第2位：写入1→清除FIFO_EXT
	delay_ms(1);
	ld3320_write_reg(0x08, 0x00);	 //清除FIFO内容第0位（清除指定FIFO后再写入一次00H）
	delay_ms(1);

	for (nAsrAddLength=0; nAsrAddLength<50; nAsrAddLength++)
	{
		if (pRecogString[nAsrAddLength] == 0)
			break;
		ld3320_write_reg(0x5, pRecogString[nAsrAddLength]);	  //写FIFO_EXT数据口
	}

	ld3320_write_reg(0xb9, nAsrAddLength);	  //当前添加识别句的字符串长度初始化时写入00H 每添加一条识别句后要设定一次
	ld3320_write_reg(0xb2, 0xff);	  //DSP忙闲状态 0x21 表示闲 可以进行下一步	   ??
	ld3320_write_reg(0x37, 0x04);	  //语音识别控制命令下发寄存器 写04H：通知DSP要添加一项识别句
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
	rt_kprintf("\r\n吴哥哥开始读取配置文件%s \r\n",path);
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
	rt_kprintf("吴哥哥: 共读取%d个配置指令\r\n",LD_struct_index);
	return 1;
}
/*******************************************************************************************************************/


// Return 1: success.
//	添加识别关键词语，开发者可以学习"语音识别芯片LD3320高阶秘籍.pdf"中关于垃圾词语吸收错误的用法
uint8 LD_AsrAddFixed(char cmd)	  //添加关键词语到LD3320芯片中
{
	//先检查传入的文件名是否符合规范
	char i;
	for(i=0;i<=LD_struct_index;i++)
		{
		rt_kprintf("添加用户码:%d 指令: %s  播放音乐:%s 配置文件:%s 执行动作:%s  \r\n",ld_struct[i].pCode,ld_struct[i].sRecog,ld_struct[i].mp3_name,ld_struct[i].next_ini,ld_struct[i].action);
	LD_AsrAddFixed_ByString(ld_struct[i].sRecog,ld_struct[i].pCode);
		}
			wu_fd = open(Name_ini,0,0);
		i = get_file_size(wu_fd);
	read(wu_fd, line_data, sizeof(line_data));
		line_data[i]='\0';
		close(wu_fd);
		rt_kprintf("添加用户码: 49 用户名: %s  \r\n",line_data);
		LD_AsrAddFixed_ByString(line_data,49);  //添加用户名
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
uint8 ucRegVal;	  //寄存器备份变量
uint8 ucHighInt;  //寄存器备份变量
uint8 ucLowInt;	  //寄存器备份变量




void ProcessInt0(void)	  //播放 语音识别中断
{
	uint8 nAsrResCount=0;

	ucRegVal = ld3320_read_reg(0x2B);

	if (nLD_Mode == LD_MODE_ASR_RUN)		//当前进行语音识别
	{
		// 语音识别产生的中断
		// （有声音输入，不论识别成功或失败都有中断）
		ld3320_write_reg(0x29,0) ;				     //中断允许 FIFO 中断允许 0表示不允许
		ld3320_write_reg(0x02,0) ;		             // FIFO中断允许	 FIFO_DATA FIFO_EXT中断   不允许
		if ((ucRegVal & 0x10) &&		             //2b第四位为1 芯片内部FIFO中断发生 MP3播放时会产生中断标志请求外部MCU向FIFO_DATA中Reload数
		     ld3320_read_reg(0xb2)==0x21 && 	     //读b2得到0x21表示闲可以进行下一步ASR动作
		     ld3320_read_reg(0xbf)==0x35)		     //读到数值为0x35，可以确定是一次语音识别流程正常结束
		{
			nAsrResCount = ld3320_read_reg(0xba);    //ASR：中断时，判断语音识别有几个识别候选
			if (nAsrResCount==1)
			{
				ld3320_read_reg(0xc5);
			}
			if (nAsrResCount>0 && nAsrResCount<=4) 	 //1 – 4: N个识别候选 0或者大于4：没有识别候选
			{
				nAsrStatus=LD_ASR_FOUNDOK;		     //表示一次识别流程结束后，有一个识别结果
			}
			else
			{
				nAsrStatus=LD_ASR_FOUNDZERO;	     //表示一次识别流程结束后，没有识别结果
			}
		}
		else
		{
			nAsrStatus=LD_ASR_FOUNDZERO;	         //表示一次识别流程结束后，没有识别结果
		}

		ld3320_write_reg(0x2b, 0);
		ld3320_write_reg(0x1C,0);	  				 //ADC开关控制 写00H ADC不可用
		return;
	}

	// 声音播放产生的中断，有三种：
	// A. 声音数据已全部播放完。
	// B. 声音数据已发送完毕。
	// C. 声音数据暂时将要用完，需要放入新的数据。	
	ucHighInt = ld3320_read_reg(0x29); 
	ucLowInt=ld3320_read_reg(0x02); 
	ld3320_write_reg(0x29,0) ;
	ld3320_write_reg(0x02,0) ;
    if(ld3320_read_reg(0xBA)&CAUSE_MP3_SONG_END)
    {
	// A. 声音数据已全部播放完。

		ld3320_write_reg(0x2B,  0);
      	ld3320_write_reg(0xBA, 0);	
		ld3320_write_reg(0xBC,0x0);	
		bMp3Play=0;					// 声音数据全部播放完后，修改bMp3Play的变量
		ld3320_write_reg(0x08,1);

      	ld3320_write_reg(0x08,0);
		ld3320_write_reg(0x33, 0);
		close(fd1);
		return ;
     }

	if(nMp3Pos>=nMp3Size)
	{
	// B. 声音数据已发送完毕。

		ld3320_write_reg(0xBC, 0x01);
		ld3320_write_reg(0x29, 0x10);
		if(fd1!=0)
		close(fd1);
		return;	
	}

	// C. 声音数据暂时将要用完，需要放入新的数据。	

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
   
/*  继续读取mp3文件数据到fifo,直到fifo满
 *	边写mp3文件数据到fifo时,LD3320会变解码播放
 *	当然写mp3文件数据到fifo的时间会短过声音的时间
 *	当声音快播放完毕的时候会进入ProcessInt0函数
 *	ProcessInt0函数又会调用此函数,所以声音得以连续
 */
void LD_ReloadMp3Data_Again(void)
{
	uint8 val;
    uint8 ucStatus;
//	int result,i,j;
	//rt_kprintf("正在加载MP3文件\n");
	ucStatus = ld3320_read_reg(0x06);
	//fifo是否满了
	while (!(ucStatus&MASK_FIFO_STATUS_AFULL) && nMp3Pos<=nMp3Size)
	{

		nMp3Pos++;
		read(fd1,&val,1);
	//	val++;
		ld3320_write_reg(0x01,val);
		ucStatus = ld3320_read_reg(0x06);
	
	}
//	rt_kprintf("加载MP3文件完成 !\n");

	if(nMp3Pos>=nMp3Size)
	{
	//rt_kprintf("加载文件文件完成 !\n");
	    ld3320_write_reg(0xBC, 0x01);
		ld3320_write_reg(0x29, 0x10);

		//等待MP3播放完毕
		while(!(ld3320_read_reg(0xBA)&CAUSE_MP3_SONG_END));

		ld3320_write_reg(0x2B,  0);
      	ld3320_write_reg(0xBA, 0);	
		ld3320_write_reg(0xBC,0x0);	
		bMp3Play=0;					// 声音数据全部播放完后，修改bMp3Play的变量
		ld3320_write_reg(0x08,1);

      	ld3320_write_reg(0x08,0);
		ld3320_write_reg(0x33, 0);
		//rt_kprintf("播放mp3完毕，关闭音乐文件!\n");
		close(fd1);
		//rt_kprintf("播放完毕，关闭音乐文件系统成功!\n");

	}
		
}

/*
 * 播放mp3时调用此函数即可
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

	//将LD3320初始化为播放MP3模式
	LD_Init_MP3();
	//设置耳机音量
	LD_AdjustMIX2SPVolume(7);
	//开始播放
	LD_play();

    return 0;
}


