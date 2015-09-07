
/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include <rthw.h>
#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include "dfs_posix.h"
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#endif

#include "led.h"
#include "LDchip.h"

#include <stdio.h>
#include "irmp.h"
#include "irsnd.h"
#include "uart2.h"
#define sys_default_count 200
/**********************信号量、邮箱初始化**************************/
struct rt_semaphore sem;//利用静态信号量，定义信号量
struct rt_mailbox mb;//利用静态邮箱，定义邮箱
static char mailpool[128];//定义邮箱池，其决定了，邮箱可以接受多少封邮件
static char dfs_mailpool[64];//定义文件系统的邮箱池
struct rt_mailbox dfs_mb;//静态方式定义文件系统邮箱
const char message[]="开始读取文件！\n";
/******************************************************************/
ALIGN(RT_ALIGN_SIZE)
/*LED1线程控制模块和线程入口函数*/

static uint8 sys_wake=0;
static uint8 sys_wake_count=0;

 char sys_down_mp3[]="/sys/sleep.mp3";
 char sys_shi_mp3[]="/sys/wakeup.mp3";
 char get_remot1_mp3[]="/sys/getremote1.mp3";
 char get_remot2_mp3[]="/sys/getremote2.mp3";
  char check_fail_mp3[]="/sys/checktfail.mp3";
 char sen_remot_mp3[]="/sys/sendremote.mp3";
 char act1[]="getremote";
 char act2[]="sendremote";
char sys_ini_name[25]="/sys/sys.ini";
/*LED1线程入口函数*/
static rt_uint8_t led1_stack[ 512 ];//LED1线程栈
static struct rt_thread led1_thread;//声明LED1线程控制块
static void led1_thread_entry(void* parameter)
{
  /*在LED1线程中初始化LED的GPIO配置*/

  LED_GPIO_Config();
  while(1)
  {
		//rt_kprintf ( "\r\n	LED1 is going!!!\r\n");
   LED1(ON);//点亮LED1
   rt_thread_delay(100);//调用RTT的API将当前线程挂起200ticks，也就是1sec
   LED1(OFF);//关闭LED1
		  rt_thread_delay(100);
		if(bMp3Play == 0)
   sys_wake_count++;
   if(sys_wake_count==sys_default_count)
   	{
		sys_wake_count=0;
		if(sys_wake)
			{
				sys_wake=0;
				rt_kprintf("系统进入休眠状态,如需使用请唤醒!\r\n");
			 rt_mb_send(&mb,(rt_uint32_t)sys_down_mp3);//发送邮件
			}
   }
  }
}
//static rt_uint8_t LD_stack[ 512 ];//LED1线程栈
//static struct rt_thread LD_thread;//声明LED1线程控制块

/*LED3线程控制块与线程入口函数*/
static rt_uint8_t led3_stack[ 512 ];//LED3线程栈
static struct rt_thread led3_thread;//LED3线程控制块

uint32_t SysCtlClockGet(void)
{
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    return RCC_ClocksStatus.SYSCLK_Frequency;
}
void timer2_init (void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 7;
    TIM_TimeBaseStructure.TIM_Prescaler = ((F_CPU / F_INTERRUPTS)/8) - 1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)                                                  // Timer2 Interrupt Handler
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	(void) irsnd_ISR(); 	// call irmp ISR
  (void) irmp_ISR();    
  // call other timer interrupt routines...
}

/*LED3线程入口函数*/
IRMP_DATA irmp_data;
IRMP_DATA irmp_data1;
 char hot_remote_state=0;
 char hot_remote_send=0;
static void led3_thread_entry(void* parameter)
{
	
	rt_kprintf ( "\r\n led3_thread_entry!!\r\n");
    irmp_init();                                                            // initialize irmp
	  irsnd_init();        
    timer2_init();                                                          // initialize timer2
	rt_kprintf ( "\r\n 红外收发系统初始化完成!\r\n");
    for (;;)
    {
			char *str;
		//	rt_kprintf ( "IRMP  is going!!!\r\n");
			 rt_thread_delay(1);
			if(RT_EOK== (rt_mb_recv(&dfs_mb,(rt_uint32_t*)&str,RT_WAITING_NO)))
			{rt_kprintf("接收到控制邮件，数据为 %s",str);
				//   TIM_Cmd(TIM2, ENABLE);
							if(strcmp(str,"getremote")==0)
								hot_remote_state=1;
								if(strcmp(str,"sendremote")==0)	
									hot_remote_send=1;
			}
        if (irmp_get_data (&irmp_data))
        {
          rt_kprintf("接收红外信号编码方式: %s",irmp_protocol_names[irmp_data.protocol]);
          rt_kprintf(" 编码地址: 0x%2X",irmp_data.address);
          rt_kprintf(" 命令: 0x%2X",irmp_data.command);
          rt_kprintf(" 标志位: 0x%2X\r\n",irmp_data.flags );
			if(1==hot_remote_state)	
			{
			hot_remote_state=2;
			irmp_data1=irmp_data;
			rt_mb_send(&mb,(rt_uint32_t)get_remot1_mp3);//发送邮件
				 rt_kprintf("红外接收完成第一次\n" );
			}
		else if(2==hot_remote_state)
			{
			if(irmp_data1.command==irmp_data.command)
				{
					 rt_kprintf("红外接收完成第二次,验证通过\n" );
				rt_mb_send(&mb,(rt_uint32_t)get_remot2_mp3);//发送邮件
				hot_remote_state=0;
					//  TIM_Cmd(TIM2, DISABLE);
				}
			else {
				 rt_kprintf("红外接收完成第二次,验证不通过，再次接收红外数据\n" );
				rt_mb_send(&mb,(rt_uint32_t)check_fail_mp3);//发送邮件
				hot_remote_state=1;
				}
			}
        }
				if(hot_remote_send)	
		{   
		hot_remote_send=0;
		//	 TIM_Cmd(TIM2, ENABLE);
			/*	irmp_data.protocol = IRMP_SAMSUNG32_PROTOCOL;                             // use NEC protocol
        irmp_data.address  = 0x0e0e;                                        // set address to 0x00FF
        irmp_data.command  = 0xf30c;                                        // set command to 0x0001
        irmp_data.flags    = 0;                                             // don't repeat frame*/
        irsnd_send_data (&irmp_data, TRUE);                                 // send frame, wait for completion			// don't repeat frame
			rt_mb_send(&mb,(rt_uint32_t)sen_remot_mp3);
		  rt_kprintf("发送红外数据完成\n");
		  rt_kprintf("发射红外信号编码方式: %s",irmp_protocol_names[irmp_data.protocol]);
          rt_kprintf(" 编码地址: 0x%2X",irmp_data.address);
          rt_kprintf(" 命令: 0x%2X",irmp_data.command);
          rt_kprintf(" 标志位: 0x%2X\r\n",irmp_data.flags );
		//  TIM_Cmd(TIM2, DISABLE);
		}
    }
}

/***************LED2..邮件发送调试函数**********************/
#ifdef RT_USING_FINSH
#include <finsh.h>
void LED2_control(char *cmd,char state)
{
 if(state)
    LED2(ON);//非0表示LED2开启
    else
	LED2(OFF);
	rt_kprintf("LD2控制线程发送邮件:%s",cmd);
		 rt_mb_send(&dfs_mb,(rt_uint32_t)cmd);//发送邮件
 
}

FINSH_FUNCTION_EXPORT(LED2_control, control LED2 state);//将该函数添加进入finsh，方便以后调用
#endif



/*************命令处理函数***********************/
void deal_action(char *cmd)
{

	rt_kprintf("处理动作指令:%s\n",cmd);
	if(strcmp(cmd,"remot")==0)
		 {
			rt_kprintf("发送接收红外邮件\n");
		 rt_mb_send(&dfs_mb,(rt_uint32_t)act1);//发送邮件
		}
	else	if(strcmp(cmd,"serem")==0) 
		{
			
		rt_kprintf("发送发射红外邮件\n");
		rt_mb_send(&dfs_mb,(rt_uint32_t)act2); 
		}
		else 	if(strcmp(cmd,"wakup")==0)
		{
			rt_kprintf("接收到命令，系统进入唤醒\n");
			sys_wake=1;
			sys_wake_count=0;
		}
		else if(strcmp(cmd,"sleep")==0)
		{
		sys_wake=0;
			sys_wake_count=0;
				rt_kprintf("接收到命令，系统进入睡眠\n");
		}
	  else return ;
			
}


/*********************LD播放线程 优先级最高************************/
struct rt_thread LD_thread;
rt_uint8_t LD_stack[512];
static void LD_thread_entry(void* parameter)
{
	rt_uint32_t evt;
//	rt_base_t level;
	/* 关闭中断*/
	rt_kprintf("进入LD专用线程!\r\n");
	while(1)
	{
		if(rt_event_recv(&event,(1 << 0), RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,50,&evt)== RT_EOK)
		{	
		//level = rt_hw_interrupt_disable();
		rt_enter_critical();
		//	rt_kprintf("mp3 playing..\r\n");	
			ProcessInt0(); 
		//	rt_hw_interrupt_enable(level);
			rt_exit_critical();
		}
	}
}
/*********************DFS 线程************************/
rt_uint8_t dfs_stack[2048];
struct rt_thread dfs_thread;
extern uint8 nAsrStatus;
void dfs_thread_entry(void* parameter)
{

   char *rev_meail;//定义指针，用来保存邮件内容	
  	/******************启动文件系统***************/
		/* init the device filesystem */
		dfs_init();
		elm_init();
		/* mount sd card fat partition 1 as root directory */
		if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
		{
			rt_kprintf("文件系统初始化成功!\n");
		}
		else
			rt_kprintf("文件系统初始化失败!\n");
	

	/*************************初始化LD3320***********************/
			ld3320_gpio_configuration();
			ld3320_nvic_cfg();
			ld3320_spi_init();
			LD_reset();
			nAsrStatus = LD_ASR_NONE;
			bMp3Play=0;
	
		if(wugege_read_ini(sys_ini_name)!=1)
				rt_kprintf("读取系统配置文件失败,语音识别将受到严重影响\n");
				rt_kprintf("读取系统配置文件成功\n");
		 if(PlayDemoSound_mp3(sys_shi_mp3)<0)
	   			rt_kprintf(" 播放初始化音乐失败，请确认是否能使用LD3320!\n");
		  		 rt_kprintf("播放初始化音乐成功！\n");
	 		rt_thread_delay(10);//延迟该线程10ms
			sys_wake=1;
			sys_wake_count=0;
  while(1)
  {
		  	if(RT_EOK==(rt_mb_recv(&mb,(rt_uint32_t*)&rev_meail,RT_WAITING_NO)))
		{
			
			rt_kprintf("接收到邮件,播放音乐%s\r\n",rev_meail);
					 if(PlayDemoSound_mp3(rev_meail)<0)
							rt_kprintf("播放音乐失败%s \r\n",rev_meail);
			rt_kprintf("接收到邮件,播放音乐完毕\r\n");
				//	 *rev_meail=NULL;
			nAsrStatus = LD_ASR_NONE;
		}
			rt_thread_delay(5);//延迟该线程5ms
			while(bMp3Play != 0)
				rt_thread_delay(5);
			switch (nAsrStatus)
			{
			case LD_ASR_RUNING:    //  LD3320芯片正在语音识别中
				break;
		
			case LD_ASR_ERROR:	   //  LD3320芯片发生错误
				break;
		
			case LD_ASR_NONE:	   //  LD3320芯片无动作
			{
				nAsrStatus=LD_ASR_RUNING;
				if (RunASR(sys_wake)==0)	//	启动一次ASR识别流程：ASR初始化，ASR添加关键词语，启动ASR运算
				{
					nAsrStatus = LD_ASR_ERROR;
				}
				break;
			}
			case LD_ASR_FOUNDOK:	//	LD3320芯片识别到语音
			{
	char mp3_name[25];	//mp3文件名缓冲区
	char action[15];
	char ini_name[20];
				int nAsrRes = LD_GetResult();	//	一次ASR识别流程结束，去取ASR识别结果
							if(nAsrRes==49)
							{
							sys_wake=1;
							sys_wake_count=0;
							PlayDemoSound_mp3(sys_shi_mp3);
							rt_kprintf("系统唤醒成功 \n"); 		
							}
							else if(sys_wake)
							{
							get_mps_ini_file(mp3_name,ini_name,action,nAsrRes);
							rt_kprintf("识别用户的mp3名字 %s 配置文件%s 命令%s 原始码%d\r\n",mp3_name,ini_name,action,nAsrRes);
							sys_wake_count=0;
							sys_wake=1;
							deal_action(action);
							 if(PlayDemoSound_mp3(mp3_name)<0)
							rt_kprintf("播放音乐失败%s \r\n",mp3_name);
							if(0!=strcmp(ini_name,sys_ini_name))
								{
									strcpy(sys_ini_name,ini_name);
									rt_kprintf("重新载入配置文件%s \r\n",sys_ini_name);
										if(wugege_read_ini(sys_ini_name)!=1)
									rt_kprintf("读取系统配置文件失败,语音识别将受到严重影响\r\n");
									}
							}
				rt_kprintf("\r\n识别到用户关键字为:%d \r\n",nAsrRes); 	
				nAsrStatus = LD_ASR_NONE;
				break;
			}
			case LD_ASR_FOUNDZERO: //无识别结果
				nAsrStatus = LD_ASR_NONE;
				break;
			default:
				nAsrStatus = LD_ASR_NONE;
				break;
			}
			}
  }

int rt_application_init()
{
  rt_err_t result;
	/******************初始化usart2  留出uart2作为WIFI接口并*******************/
  uart2_config(115200);   
  /******************usart2初始化结束***************/

  /******************初始化信号量和邮箱**************/
  rt_mb_init(&mb,"mb",&mailpool[0],sizeof(mailpool)/4,RT_IPC_FLAG_FIFO);//按照先入先出的原则，申请了一个可以存放32封邮件的邮箱
  rt_mb_init(&dfs_mb,"dfs_mb",&dfs_mailpool[0],sizeof(dfs_mailpool)/4,RT_IPC_FLAG_FIFO);
	result = rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);  //初始化事件机制 用于mp3播放
	if(result != RT_EOK)
	{
	rt_kprintf("init event failed.\r\n");
		return-1;
		}
  /***************************************************/

  /*初始化LED1  时间统计线程*/
	result = rt_thread_init(&led1_thread,"led1",led1_thread_entry,RT_NULL,&led1_stack[0],sizeof(led1_stack),7,3);
	if (result == RT_EOK)
	{	
        rt_thread_startup(&led1_thread);
	}
	/*初始化LED3  红外收发线程*/
	result = rt_thread_init(&led3_thread, "led3",led3_thread_entry,RT_NULL,&led3_stack[0],sizeof(led3_stack),	5,3	);
	/*如果初始化LED1线程成功*/
	if (result == RT_EOK)
	{	/*启动LED1线程*/
        rt_thread_startup(&led3_thread);
	}

                     
/***************初始化DFS线程***************************/
result=rt_thread_init(&dfs_thread, "dfs",dfs_thread_entry, RT_NULL,&dfs_stack[0], sizeof(dfs_stack),6,20);
if (result==RT_EOK)
{
  rt_thread_startup(&dfs_thread);
}
/***************初始化LD3320播放线程***************************/
result=rt_thread_init(&LD_thread,"ld_t",LD_thread_entry,RT_NULL,&LD_stack[0], sizeof(LD_stack), 4,50);
if (result==RT_EOK)
{
  rt_thread_startup(&LD_thread);
}
	return 0;
}

/*@}*/
