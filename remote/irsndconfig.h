/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * irsndconfig.h
 *
 * DO NOT INCLUDE THIS FILE, WILL BE INCLUDED BY IRSND.H!
 *
 * Copyright (c) 2010-2015 Frank Meyer - frank(at)fli4l.de
 *
 * $Id: irsndconfig.h,v 1.75 2015/06/15 11:40:55 fm Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef _IRSNDCONFIG_H_
#define _IRSNDCONFIG_H_

#if !defined(_IRSND_H_)
#  error please include only irsnd.h, not irsndconfig.h
#endif

//~ #define IRSND_DEBUG 1                                   // activate debugging

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * F_INTERRUPTS: number of interrupts per second, should be in the range from 10000 to 20000, typically 15000
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#ifndef F_INTERRUPTS
#  define F_INTERRUPTS                          15000   // interrupts per second
#endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Change settings from 1 to 0 if you want to disable one or more encoders.
 * This saves program space.
 * 1 enable  decoder
 * 0 disable decoder
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

// typical protocols, disable here!             Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRSND_SUPPORT_SIRCS_PROTOCOL            1       // Sony SIRCS           >= 10000                 ~200 bytes
#define IRSND_SUPPORT_NEC_PROTOCOL              1       // NEC + APPLE          >= 10000                 ~100 bytes
#define IRSND_SUPPORT_SAMSUNG_PROTOCOL          1       // Samsung + Samsung32  >= 10000                 ~300 bytes
#define IRSND_SUPPORT_MATSUSHITA_PROTOCOL       1       // Matsushita           >= 10000                 ~200 bytes
#define IRSND_SUPPORT_KASEIKYO_PROTOCOL         1       // Kaseikyo             >= 10000                 ~300 bytes

// more protocols, enable here!                 Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRSND_SUPPORT_DENON_PROTOCOL            1       // DENON, Sharp         >= 10000                 ~200 bytes
#define IRSND_SUPPORT_RC5_PROTOCOL              1       // RC5                  >= 10000                 ~150 bytes
#define IRSND_SUPPORT_RC6_PROTOCOL              1       // RC6                  >= 10000                 ~250 bytes
#define IRSND_SUPPORT_RC6A_PROTOCOL             1       // RC6A                 >= 10000                 ~250 bytes
#define IRSND_SUPPORT_JVC_PROTOCOL              1       // JVC                  >= 10000                 ~150 bytes
#define IRSND_SUPPORT_NEC16_PROTOCOL            1       // NEC16                >= 10000                 ~150 bytes
#define IRSND_SUPPORT_NEC42_PROTOCOL            1      // NEC42                >= 10000                 ~150 bytes
#define IRSND_SUPPORT_IR60_PROTOCOL             1       // IR60 (SDA2008)       >= 10000                 ~250 bytes
#define IRSND_SUPPORT_GRUNDIG_PROTOCOL          1       // Grundig              >= 10000                 ~300 bytes
#define IRSND_SUPPORT_SIEMENS_PROTOCOL          1       // Siemens, Gigaset     >= 15000                 ~150 bytes
#define IRSND_SUPPORT_NOKIA_PROTOCOL            1       // Nokia                >= 10000                 ~400 bytes

// exotic protocols, enable here!               Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRSND_SUPPORT_KATHREIN_PROTOCOL         1       // Kathrein             >= 10000                 DON'T CHANGE, NOT SUPPORTED YET!
#define IRSND_SUPPORT_NUBERT_PROTOCOL           1       // NUBERT               >= 10000                 ~100 bytes
#define IRSND_SUPPORT_FAN_PROTOCOL              1       // FAN (ventilator)     >= 10000                 ~100 bytes
#define IRSND_SUPPORT_SPEAKER_PROTOCOL          1       // SPEAKER              >= 10000                 ~100 bytes
#define IRSND_SUPPORT_BANG_OLUFSEN_PROTOCOL     1       // Bang&Olufsen         >= 10000                 ~250 bytes
#define IRSND_SUPPORT_RECS80_PROTOCOL           1       // RECS80               >= 15000                 ~100 bytes
#define IRSND_SUPPORT_RECS80EXT_PROTOCOL        1       // RECS80EXT            >= 15000                 ~100 bytes
#define IRSND_SUPPORT_THOMSON_PROTOCOL          1       // Thomson              >= 10000                 ~250 bytes
#define IRSND_SUPPORT_NIKON_PROTOCOL            1       // NIKON                >= 10000                 ~150 bytes
#define IRSND_SUPPORT_NETBOX_PROTOCOL           1       // Netbox keyboard      >= 10000                 DON'T CHANGE, NOT SUPPORTED YET!
#define IRSND_SUPPORT_ORTEK_PROTOCOL            1       // ORTEK (Hama)         >= 10000                 DON'T CHANGE, NOT SUPPORTED YET!
#define IRSND_SUPPORT_TELEFUNKEN_PROTOCOL       1       // Telefunken 1560      >= 10000                 ~150 bytes
#define IRSND_SUPPORT_FDC_PROTOCOL              1       // FDC IR keyboard      >= 10000 (better 15000)  ~150 bytes
#define IRSND_SUPPORT_RCCAR_PROTOCOL            1       // RC CAR               >= 10000 (better 15000)  ~150 bytes
#define IRSND_SUPPORT_ROOMBA_PROTOCOL           1       // iRobot Roomba        >= 10000                 ~150 bytes
#define IRSND_SUPPORT_RUWIDO_PROTOCOL           1       // RUWIDO, T-Home       >= 15000                 ~250 bytes
#define IRSND_SUPPORT_A1TVBOX_PROTOCOL          1       // A1 TV BOX            >= 15000 (better 20000)  ~200 bytes
#define IRSND_SUPPORT_LEGO_PROTOCOL             1       // LEGO Power RC        >= 20000                 ~150 bytes
#define IRSND_SUPPORT_RCMM_PROTOCOL             1       // RCMM 12,24, or 32    >= 20000                 DON'T CHANGE, NOT SUPPORTED YET!
#define IRSND_SUPPORT_LGAIR_PROTOCOL            1       // LG Air Condition     >= 10000                 ~150 bytes.
#define IRSND_SUPPORT_SAMSUNG48_PROTOCOL        1       // Samsung48            >= 10000                 ~100 bytes
#define IRSND_SUPPORT_PENTAX_PROTOCOL           1       // Pentax               >= 10000                 ~150 bytes
#define IRSND_SUPPORT_S100_PROTOCOL             1       // S100                 >= 10000                 ~150 bytes
#define IRSND_SUPPORT_ACP24_PROTOCOL            1       // ACP24                >= 10000                 ~150 bytes


/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * AVR XMega section:
 *
 * Change hardware pin here:                    IRSND_XMEGA_OC0A = OC0A on ATxmegas  supporting OC0A, e.g. ATxmega128A1U
 *                                              IRSND_XMEGA_OC0B = OC0B on ATxmegas  supporting OC0B, e.g. ATxmega128A1U
 *                                              IRSND_XMEGA_OC0C = OC0C on ATxmegas  supporting OC0C, e.g. ATxmega128A1U
 *                                              IRSND_XMEGA_OC0D = OC0D on ATxmegas  supporting OC0D, e.g. ATxmega128A1U
 *                                              IRSND_XMEGA_OC1A = OC1A on ATxmegas  supporting OC1A, e.g. ATxmega128A1U
 *                                              IRSND_XMEGA_OC1B = OC1B on ATxmegas  supporting OC1B, e.g. ATxmega128A1U
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#if defined(__AVR_XMEGA__)                                              // XMEGA
#  define IRSND_PORT_PRE                        PORTD                   
#  define XMEGA_Timer                           TCD0
#  define IRSND_OCx                             IRSND_XMEGA_OC0B        // use OC0B

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * AVR ATMega/ATTiny section:
 *
 * Change hardware pin here:                    IRSND_OC2  = OC2  on ATmegas         supporting OC2,  e.g. ATmega8
 *                                              IRSND_OC2A = OC2A on ATmegas         supporting OC2A, e.g. ATmega88
 *                                              IRSND_OC2B = OC2B on ATmegas         supporting OC2B, e.g. ATmega88
 *                                              IRSND_OC0  = OC0  on ATmegas         supporting OC0,  e.g. ATmega162
 *                                              IRSND_OC0A = OC0A on ATmegas/ATtinys supporting OC0A, e.g. ATtiny84, ATtiny85, ATtiny87/167
 *                                              IRSND_OC0B = OC0B on ATmegas/ATtinys supporting OC0B, e.g. ATtiny84, ATtiny85
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#elif defined(ATMEL_AVR)
#  define IRSND_OCx                             IRSND_OC2B              // use OC2B

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * PIC C18 or XC8 section:
 *
 * Change hardware pin here:                    IRSND_PIC_CCP1 = RC2 on PIC 18F2550/18F4550, ...
 *                                              IRSND_PIC_CCP2 = RC1 on PIC 18F2550/18F4550, ...
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#elif defined(PIC_C18)                                                  // C18 or XC8 compiler
#  if defined(__12F1840)                                                // XC8 compiler
#    define Pre_Scaler                          1                       // define prescaler for timer2 e.g. 1,4,16
#    define F_CPU                               32000000UL              // PIC frequency: set your freq here
#    define PIC_Scaler                          2                       // PIC needs /2 extra in IRSND_FREQ_32_KHZ calculation for right value

#  else                                                                 // C18 compiler
#    define IRSND_OCx                           IRSND_PIC_CCP2          // Use PWMx for PIC
                                                                        // change other PIC C18 specific settings:
#    define F_CPU                               48000000UL              // PIC frequency: set your freq here
#    define Pre_Scaler                          4                       // define prescaler for timer2 e.g. 1,4,16
#    define PIC_Scaler                          2                       // PIC needs /2 extra in IRSND_FREQ_32_KHZ calculation for right value
#  endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * ARM STM32 section:
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#elif defined (ARM_STM32)                                               // use B6 as IR output on STM32
#  define IRSND_PORT_LETTER                     A
#  define IRSND_BIT_NUMBER                      6
#  define IRSND_TIMER_NUMBER                    3
#  define IRSND_TIMER_CHANNEL_NUMBER            1                       // only channel 1 can be used at the moment, others won't work

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Other target systems
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#elif !defined (UNIX_OR_WINDOWS)
#  error target system not defined.
#endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Use Callbacks to indicate output signal or something else
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#ifndef IRSND_USE_CALLBACK
#  define IRSND_USE_CALLBACK                    0                       // flag: 0 = don't use callbacks, 1 = use callbacks, default is 0
#endif

#endif // _IRSNDCONFIG_H_