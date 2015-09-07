/************************************************************************************/
//	版权所有：Copyright (c) 2005 - 2010 ICRoute INC.
/************************************************************************************/


#ifndef LD_CHIP_H
#define LD_CHIP_H

#define uint8 unsigned char
#define uint16 unsigned int
#define uint32 unsigned long

//	以下三个状态定义用来记录程序是在运行ASR识别还是在运行MP3播放
#define LD_MODE_IDLE		0x00
#define LD_MODE_ASR_RUN		0x08
#define LD_MODE_MP3		 	0x40


//	以下五个状态定义用来记录程序是在运行ASR识别过程中的哪个状态
#define LD_ASR_NONE				0x00	//	表示没有在作ASR识别
#define LD_ASR_RUNING			0x01	//	表示LD3320正在作ASR识别中
#define LD_ASR_FOUNDOK			0x10	//	表示一次识别流程结束后，有一个识别结果
#define LD_ASR_FOUNDZERO 		0x11	//	表示一次识别流程结束后，没有识别结果
#define LD_ASR_ERROR	 		0x31	//	表示一次识别流程中LD3320芯片内部出现不正确的状态


#define CLK_IN   		 	36//22.1184	/* user need modify this value according to clock in */
#define LD_PLL_11			(uint8)((CLK_IN/2.0)-1)
#define LD_PLL_MP3_19		0x0f
#define LD_PLL_MP3_1B		0x18
#define LD_PLL_MP3_1D   	(uint8)(((90.0*((LD_PLL_11)+1))/(CLK_IN))-1)

#define LD_PLL_ASR_19 		(uint8)(CLK_IN*32.0/(LD_PLL_11+1) - 0.51)
#define LD_PLL_ASR_1B 		0x48
#define LD_PLL_ASR_1D 		0x1f

// LD chip fixed values.
#define        RESUM_OF_MUSIC               0x01
#define        CAUSE_MP3_SONG_END           0x20

#define        MASK_INT_SYNC				0x10
#define        MASK_INT_FIFO				0x04
#define    	   MASK_AFIFO_INT				0x01
#define        MASK_FIFO_STATUS_AFULL		0x08

#include "rtthread.h"

#define LD_num  49
#define Name_ini "/sys/name.ini"
typedef struct 
{
	char pCode;
	char sRecog[35];
	char mp3_name[25];
	char next_ini[20];
	char  action[15];
}LD_struct;

void LD_reset(void);

void LD_Init_Common(void);
void LD_Init_MP3(void);
void LD_Init_ASR(void);

void LD_play(void);
void LD_AdjustMIX2SPVolume(uint8 value);
void LD_ReloadMp3Data(void);
void LD_ReloadMp3Data_2(void);

uint8 LD_ProcessAsr(uint32 RecogAddr);
void LD_AsrStart(void);
uint8 LD_AsrRun(void);
uint8 LD_AsrAddFixed(char cmd);
uint8 LD_GetResult(void);
uint8 RunASR(char cmd);
uint8 LD_Check_ASRBusyFlag_b2(void);
int wugege_read_line(int fd_num);
char wugege_read_ini(char *path);
char get_mps_ini_file(char *mp3_name,char *ini_name,char *atcion,char index);
void LD_AsrAddFixed_ByString(char * pRecogString, uint8 k);

void delay_ms(unsigned long uldata);
void LD_TEST(void);
void ld3320_gpio_configuration(void);
void ld3320_nvic_cfg(void);
void ld3320_spi_init(void);
void fill_the_fifo(void);
int PlayDemoSound_mp3(char *path);
void ProcessInt0(void); 


extern uint32 nMp3StartPos;
extern uint32 nMp3Size;
extern uint32 nMp3Pos;
extern uint8 bMp3Play;
extern uint8 nLD_Mode;
extern struct rt_event event;
#endif
