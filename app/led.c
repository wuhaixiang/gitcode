/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名  ：led.c
 * 描述    ：led 应用函数库
 *          
 * 实验平台：野火STM32开发板
 * 硬件连接：-----------------
 *          |   PC2 - LED1    |
 *          |   PC3- LED2    |
 *          |				|
 *           ----------------- 
 * 库版本  ：ST3.0.0
 *
 * 作者    ：fire  QQ: 313303034
 * 博客    ：firestm32.blog.chinaunix.net 
**********************************************************************************/

#include "led.h"

void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 );	 // turn off all led
}



/******************* (C) COPYRIGHT 2011 野火嵌入式开发工作室 *****END OF FILE****/
