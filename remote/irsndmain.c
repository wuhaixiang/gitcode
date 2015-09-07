/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * irsndmain.c - demo main module to test IRSND encoder on AVRs
 *
 * Copyright (c) 2010-2015 Frank Meyer - frank(at)fli4l.de
 *
 * ATMEGA88 @ 8 MHz internal RC      Osc with BODLEVEL 4.3V: lfuse: 0xE2 hfuse: 0xDC efuse: 0xF9
 * ATMEGA88 @ 8 MHz external Crystal Osc with BODLEVEL 4.3V: lfuse: 0xFF hfuse: 0xDC efuse: 0xF9
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#include "irsnd.h"
#include "irmp.h"
#include "usart.h"	

#ifndef F_CPU
#  error F_CPU unknown
#endif
uint32_t SysCtlClockGet(void)
{
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    return RCC_ClocksStatus.SYSCLK_Frequency;
}

void
timer2_init (void)
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * timer 1 compare handler, called every 1/10000 sec
 *---------------------------------------------------------------------------------------------------------------------------------------------------
*/
void TIM2_IRQHandler(COMPA_VECT)                                                             // Timer1 output compare A interrupt service routine, called every 1/15000 sec
{
 TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                              
    (void) irsnd_ISR(); 
			(void) irmp_ISR();       
	// call irsnd ISR
    // call other timer interrupt routines here...
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * MAIN: main routine
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
int
ismain (void)
{
    IRMP_DATA irmp_data;
		IRMP_DATA irmp_data1;
    irsnd_init();                                                           // initialize irsnd
    timer2_init();                                                          // initialize timer
  //  sei ();                                                                 // enable interrupts

    for (;;)
    {
			 if (irmp_get_data (&irmp_data))
        {
          /*
			printf ( "Protocol: %d, Address: %d, Command: %d, Flags: %d\n",
					( uint16_t ) irmp_data.protocol,
					irmp_data.command,
					irmp_data.flags,
					( uint16_t ) irmp_data.protocol );
          */
          //printf("R: Code: %s",Proto[irmp_data.protocol-1]);
          printf("R: Code: %s",irmp_protocol_names[irmp_data.protocol]);
          
          printf(" Address: 0x%.2X",irmp_data.address);
          printf(" Command: 0x%.2X",irmp_data.command);
          printf(" Flags: 0x%.2X\r\n",irmp_data.flags );
            // ir signal decoded, do something here...
            // irmp_data.protocol is the protocol, see irmp.h
            // irmp_data.address is the address/manufacturer code of ir sender
            // irmp_data.command is the command code
            // irmp_protocol_names[irmp_data.protocol] is the protocol name (if enabled, see irmpconfig.h)
        }
        irmp_data1.protocol = IRMP_NEC_PROTOCOL;                             // use NEC protocol
        irmp_data1.address  = 0xABF0;                                        // set address to 0x00FF
        irmp_data1.command  = 0x5A;                                        // set command to 0x0001
        irmp_data1.flags    = 0;                                             // don't repeat frame

        irsnd_send_data (&irmp_data1, TRUE);                                 // send frame, wait for completion
      //  _delay_ms (1000);
    }
}
