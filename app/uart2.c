#include "uart2.h"
/*************************初始化usart2************************/
/*串口初始化*/
void uart2_config(int boud)
{
	
 USART_InitTypeDef USART_InitStructure;
 /*启动uart2时钟*/
	uart2_gpio_config(); //初始化IO口
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 /* USARTy and USARTz configuration -------------------------------------------*/
  /* USARTy and USARTz configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */

  USART_InitStructure.USART_BaudRate = boud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure USARTy */
  USART_Init(USART2, &USART_InitStructure);
  /* Enable the USARTz Receive Interrupt */
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  /* Enable USARTy */
  USART_Cmd(USART2, ENABLE);

}
/*GPIO 初始化*/
void uart2_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//初始化RX引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	//初始化TX引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*初始化uart2中断*/
void uar2_NVIC_config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the USARTz Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
