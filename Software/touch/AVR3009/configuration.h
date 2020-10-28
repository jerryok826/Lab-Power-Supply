/*******************************************************************************
*   $FILE:  configuration.h
*   Brief : Select QT device and interface pins for CHANGE and RESET pin 
*   Atmel Corporation:  http://www.atmel.com
*   Support email:  touch@atmel.com
******************************************************************************/
/*  License
*   Copyright (c) 2012, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/*============================================================================
Include files
============================================================================*/
#if defined (__ICCAVR__)
#include <intrinsics.h>
#include <ioavr.h>
#include <inavr.h>
#else
#include <util/delay.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#endif

/*============================================================================
Macro Definitions
============================================================================*/
#if defined (__ICCAVR__)
// 8 MHz CPU speed
#define CPU_SPEED 8 
// delay for 8 MHz CPU Frequency 
#define _delay_us(x) __delay_cycles(CPU_SPEED * (x))
// maximum 4 ms delay possible with below macro
#define _delay_ms(y) __delay_cycles((y) * 1000 * CPU_SPEED)
#endif

#define JOIN( x, y )		x ## y	
#define CONCAT(REG, PIN)	JOIN(REG, PIN)

// Select the device to used

// QT Device list
#define QT1060	1
#define QT1070	2
#define QT2120	3
#define QT2160	4
#define QT60160	5
#define QT60240	6

// Select one of the QT device from above list
#define QT_DEVICE QT1070

// Device Interface Settings

// RESET port and pin configuration
#define RESET_PORT  D	// PORT
#define RESET_PIN   3	// Pin Number

// CHANGE Status port and pin configuration
#define CHANGE_STATUS_PORT  D 	// PORT
#define CHANGE_STATUS_PIN   2	// Pin Number

/*============================================================================
Type definitions
============================================================================*/

#if defined (__ICCAVR__)
// An signed 8-bit value
typedef signed char int8_t;
// An unsigned 8-bit value
typedef unsigned char uint8_t;
// An unsigned 16-bit value
typedef unsigned short uint16_t;
// A signed 16-bit value
typedef short int16_t;
// An unsigned 32-bit value
typedef unsigned long uint32_t;
#endif

#endif /* CONFIGURATION_H_ */
/*============================================================================
  END OF FILE 
============================================================================*/		