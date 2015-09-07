
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
/**********************�ź����������ʼ��**************************/
struct rt_semaphore sem;//���þ�̬�ź����������ź���
struct rt_mailbox mb;//���þ�̬���䣬��������
static char mailpool[128];//��������أ�������ˣ�������Խ��ܶ��ٷ��ʼ�
static char dfs_mailpool[64];//�����ļ�ϵͳ�������
struct rt_mailbox dfs_mb;//��̬��ʽ�����ļ�ϵͳ����
const char message[]="��ʼ��ȡ�ļ���\n";
/******************************************************************/
ALIGN(RT_ALIGN_SIZE)
/*LED1�߳̿���ģ����߳���ں���*/

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
/*LED1�߳���ں���*/
static rt_uint8_t led1_stack[ 512 ];//LED1�߳�ջ
static struct rt_thread led1_thread;//����LED1�߳̿��ƿ�
static void led1_thread_entry(void* parameter)
{
  /*��LED1�߳��г�ʼ��LED��GPIO����*/

  LED_GPIO_Config();
  while(1)
  {
		//rt_kprintf ( "\r\n	LED1 is going!!!\r\n");
   LED1(ON);//����LED1
   rt_thread_delay(100);//����RTT��API����ǰ�̹߳���200ticks��Ҳ����1sec
   LED1(OFF);//�ر�LED1
		  rt_thread_delay(100);
		if(bMp3Play == 0)
   sys_wake_count++;
   if(sys_wake_count==sys_default_count)
   	{
		sys_wake_count=0;
		if(sys_wake)
			{
				sys_wake=0;
				rt_kprintf("ϵͳ��������״̬,����ʹ���뻽��!\r\n");
			 rt_mb_send(&mb,(rt_uint32_t)sys_down_mp3);//�����ʼ�
			}
   }
  }
}
//static rt_uint8_t LD_stack[ 512 ];//LED1�߳�ջ
//static struct rt_thread LD_thread;//����LED1�߳̿��ƿ�

/*LED3�߳̿��ƿ����߳���ں���*/
static rt_uint8_t led3_stack[ 512 ];//LED3�߳�ջ
static struct rt_thread led3_thread;//LED3�߳̿��ƿ�

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

/*LED3�߳���ں���*/
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
	rt_kprintf ( "\r\n �����շ�ϵͳ��ʼ�����!\r\n");
    for (;;)
    {
			char *str;
		//	rt_kprintf ( "IRMP  is going!!!\r\n");
			 rt_thread_delay(1);
			if(RT_EOK== (rt_mb_recv(&dfs_mb,(rt_uint32_t*)&str,RT_WAITING_NO)))
			{rt_kprintf("���յ������ʼ�������Ϊ %s",str);
				//   TIM_Cmd(TIM2, ENABLE);
							if(strcmp(str,"getremote")==0)
								hot_remote_state=1;
								if(strcmp(str,"sendremote")==0)	
									hot_remote_send=1;
			}
        if (irmp_get_data (&irmp_data))
        {
          rt_kprintf("���պ����źű��뷽ʽ: %s",irmp_protocol_names[irmp_data.protocol]);
          rt_kprintf(" �����ַ: 0x%2X",irmp_data.address);
          rt_kprintf(" ����: 0x%2X",irmp_data.command);
          rt_kprintf(" ��־λ: 0x%2X\r\n",irmp_data.flags );
			if(1==hot_remote_state)	
			{
			hot_remote_state=2;
			irmp_data1=irmp_data;
			rt_mb_send(&mb,(rt_uint32_t)get_remot1_mp3);//�����ʼ�
				 rt_kprintf("���������ɵ�һ��\n" );
			}
		else if(2==hot_remote_state)
			{
			if(irmp_data1.command==irmp_data.command)
				{
					 rt_kprintf("���������ɵڶ���,��֤ͨ��\n" );
				rt_mb_send(&mb,(rt_uint32_t)get_remot2_mp3);//�����ʼ�
				hot_remote_state=0;
					//  TIM_Cmd(TIM2, DISABLE);
				}
			else {
				 rt_kprintf("���������ɵڶ���,��֤��ͨ�����ٴν��պ�������\n" );
				rt_mb_send(&mb,(rt_uint32_t)check_fail_mp3);//�����ʼ�
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
		  rt_kprintf("���ͺ����������\n");
		  rt_kprintf("��������źű��뷽ʽ: %s",irmp_protocol_names[irmp_data.protocol]);
          rt_kprintf(" �����ַ: 0x%2X",irmp_data.address);
          rt_kprintf(" ����: 0x%2X",irmp_data.command);
          rt_kprintf(" ��־λ: 0x%2X\r\n",irmp_data.flags );
		//  TIM_Cmd(TIM2, DISABLE);
		}
    }
}

/***************LED2..�ʼ����͵��Ժ���**********************/
#ifdef RT_USING_FINSH
#include <finsh.h>
void LED2_control(char *cmd,char state)
{
 if(state)
    LED2(ON);//��0��ʾLED2����
    else
	LED2(OFF);
	rt_kprintf("LD2�����̷߳����ʼ�:%s",cmd);
		 rt_mb_send(&dfs_mb,(rt_uint32_t)cmd);//�����ʼ�
 
}

FINSH_FUNCTION_EXPORT(LED2_control, control LED2 state);//���ú�����ӽ���finsh�������Ժ����
#endif



/*************�������***********************/
void deal_action(char *cmd)
{

	rt_kprintf("������ָ��:%s\n",cmd);
	if(strcmp(cmd,"remot")==0)
		 {
			rt_kprintf("���ͽ��պ����ʼ�\n");
		 rt_mb_send(&dfs_mb,(rt_uint32_t)act1);//�����ʼ�
		}
	else	if(strcmp(cmd,"serem")==0) 
		{
			
		rt_kprintf("���ͷ�������ʼ�\n");
		rt_mb_send(&dfs_mb,(rt_uint32_t)act2); 
		}
		else 	if(strcmp(cmd,"wakup")==0)
		{
			rt_kprintf("���յ����ϵͳ���뻽��\n");
			sys_wake=1;
			sys_wake_count=0;
		}
		else if(strcmp(cmd,"sleep")==0)
		{
		sys_wake=0;
			sys_wake_count=0;
				rt_kprintf("���յ����ϵͳ����˯��\n");
		}
	  else return ;
			
}


/*********************LD�����߳� ���ȼ����************************/
struct rt_thread LD_thread;
rt_uint8_t LD_stack[512];
static void LD_thread_entry(void* parameter)
{
	rt_uint32_t evt;
//	rt_base_t level;
	/* �ر��ж�*/
	rt_kprintf("����LDר���߳�!\r\n");
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
/*********************DFS �߳�************************/
rt_uint8_t dfs_stack[2048];
struct rt_thread dfs_thread;
extern uint8 nAsrStatus;
void dfs_thread_entry(void* parameter)
{

   char *rev_meail;//����ָ�룬���������ʼ�����	
  	/******************�����ļ�ϵͳ***************/
		/* init the device filesystem */
		dfs_init();
		elm_init();
		/* mount sd card fat partition 1 as root directory */
		if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
		{
			rt_kprintf("�ļ�ϵͳ��ʼ���ɹ�!\n");
		}
		else
			rt_kprintf("�ļ�ϵͳ��ʼ��ʧ��!\n");
	

	/*************************��ʼ��LD3320***********************/
			ld3320_gpio_configuration();
			ld3320_nvic_cfg();
			ld3320_spi_init();
			LD_reset();
			nAsrStatus = LD_ASR_NONE;
			bMp3Play=0;
	
		if(wugege_read_ini(sys_ini_name)!=1)
				rt_kprintf("��ȡϵͳ�����ļ�ʧ��,����ʶ���ܵ�����Ӱ��\n");
				rt_kprintf("��ȡϵͳ�����ļ��ɹ�\n");
		 if(PlayDemoSound_mp3(sys_shi_mp3)<0)
	   			rt_kprintf(" ���ų�ʼ������ʧ�ܣ���ȷ���Ƿ���ʹ��LD3320!\n");
		  		 rt_kprintf("���ų�ʼ�����ֳɹ���\n");
	 		rt_thread_delay(10);//�ӳٸ��߳�10ms
			sys_wake=1;
			sys_wake_count=0;
  while(1)
  {
		  	if(RT_EOK==(rt_mb_recv(&mb,(rt_uint32_t*)&rev_meail,RT_WAITING_NO)))
		{
			
			rt_kprintf("���յ��ʼ�,��������%s\r\n",rev_meail);
					 if(PlayDemoSound_mp3(rev_meail)<0)
							rt_kprintf("��������ʧ��%s \r\n",rev_meail);
			rt_kprintf("���յ��ʼ�,�����������\r\n");
				//	 *rev_meail=NULL;
			nAsrStatus = LD_ASR_NONE;
		}
			rt_thread_delay(5);//�ӳٸ��߳�5ms
			while(bMp3Play != 0)
				rt_thread_delay(5);
			switch (nAsrStatus)
			{
			case LD_ASR_RUNING:    //  LD3320оƬ��������ʶ����
				break;
		
			case LD_ASR_ERROR:	   //  LD3320оƬ��������
				break;
		
			case LD_ASR_NONE:	   //  LD3320оƬ�޶���
			{
				nAsrStatus=LD_ASR_RUNING;
				if (RunASR(sys_wake)==0)	//	����һ��ASRʶ�����̣�ASR��ʼ����ASR��ӹؼ��������ASR����
				{
					nAsrStatus = LD_ASR_ERROR;
				}
				break;
			}
			case LD_ASR_FOUNDOK:	//	LD3320оƬʶ������
			{
	char mp3_name[25];	//mp3�ļ���������
	char action[15];
	char ini_name[20];
				int nAsrRes = LD_GetResult();	//	һ��ASRʶ�����̽�����ȥȡASRʶ����
							if(nAsrRes==49)
							{
							sys_wake=1;
							sys_wake_count=0;
							PlayDemoSound_mp3(sys_shi_mp3);
							rt_kprintf("ϵͳ���ѳɹ� \n"); 		
							}
							else if(sys_wake)
							{
							get_mps_ini_file(mp3_name,ini_name,action,nAsrRes);
							rt_kprintf("ʶ���û���mp3���� %s �����ļ�%s ����%s ԭʼ��%d\r\n",mp3_name,ini_name,action,nAsrRes);
							sys_wake_count=0;
							sys_wake=1;
							deal_action(action);
							 if(PlayDemoSound_mp3(mp3_name)<0)
							rt_kprintf("��������ʧ��%s \r\n",mp3_name);
							if(0!=strcmp(ini_name,sys_ini_name))
								{
									strcpy(sys_ini_name,ini_name);
									rt_kprintf("�������������ļ�%s \r\n",sys_ini_name);
										if(wugege_read_ini(sys_ini_name)!=1)
									rt_kprintf("��ȡϵͳ�����ļ�ʧ��,����ʶ���ܵ�����Ӱ��\r\n");
									}
							}
				rt_kprintf("\r\nʶ���û��ؼ���Ϊ:%d \r\n",nAsrRes); 	
				nAsrStatus = LD_ASR_NONE;
				break;
			}
			case LD_ASR_FOUNDZERO: //��ʶ����
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
	/******************��ʼ��usart2  ����uart2��ΪWIFI�ӿڲ�*******************/
  uart2_config(115200);   
  /******************usart2��ʼ������***************/

  /******************��ʼ���ź���������**************/
  rt_mb_init(&mb,"mb",&mailpool[0],sizeof(mailpool)/4,RT_IPC_FLAG_FIFO);//���������ȳ���ԭ��������һ�����Դ��32���ʼ�������
  rt_mb_init(&dfs_mb,"dfs_mb",&dfs_mailpool[0],sizeof(dfs_mailpool)/4,RT_IPC_FLAG_FIFO);
	result = rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);  //��ʼ���¼����� ����mp3����
	if(result != RT_EOK)
	{
	rt_kprintf("init event failed.\r\n");
		return-1;
		}
  /***************************************************/

  /*��ʼ��LED1  ʱ��ͳ���߳�*/
	result = rt_thread_init(&led1_thread,"led1",led1_thread_entry,RT_NULL,&led1_stack[0],sizeof(led1_stack),7,3);
	if (result == RT_EOK)
	{	
        rt_thread_startup(&led1_thread);
	}
	/*��ʼ��LED3  �����շ��߳�*/
	result = rt_thread_init(&led3_thread, "led3",led3_thread_entry,RT_NULL,&led3_stack[0],sizeof(led3_stack),	5,3	);
	/*�����ʼ��LED1�̳߳ɹ�*/
	if (result == RT_EOK)
	{	/*����LED1�߳�*/
        rt_thread_startup(&led3_thread);
	}

                     
/***************��ʼ��DFS�߳�***************************/
result=rt_thread_init(&dfs_thread, "dfs",dfs_thread_entry, RT_NULL,&dfs_stack[0], sizeof(dfs_stack),6,20);
if (result==RT_EOK)
{
  rt_thread_startup(&dfs_thread);
}
/***************��ʼ��LD3320�����߳�***************************/
result=rt_thread_init(&LD_thread,"ld_t",LD_thread_entry,RT_NULL,&LD_stack[0], sizeof(LD_stack), 4,50);
if (result==RT_EOK)
{
  rt_thread_startup(&LD_thread);
}
	return 0;
}

/*@}*/
