/*******************************************************************************
*   $FILE:  LED.c
*   Brief:  Handling LED update based on touch key status
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

/*============================================================================
Include files
============================================================================*/
#include "LED.h"

/*============================================================================
Function Definitions
============================================================================*/
/*============================================================================
Name    :   InitTouchStatusPorts
------------------------------------------------------------------------------
Purpose :	Configure the PORTC pins 0 to 3 for displaying touch status
Input   :   none
Output  :   none
Return	:	none
Notes   :
============================================================================*/
void InitTouchStatusPorts(void)
{
	/* Configure the PORTB(0 - 7 bits) for displaying touch status
	 * for the key0 to key7
	 */
	DDRB |= 0xFF; //set as output
		
	// Turn off all 8 LEDs by driving high
	PORTB |= 0xFF;
}

/*============================================================================
Name    :   UpdateLedStatus
------------------------------------------------------------------------------
Purpose :   update touch key status through LED indications
Input   :   QtStatusPtr :	Pointer to byte array for Qtouch-data
Output  :   none
Return	:   none
Notes   :
============================================================================*/
void UpdateLedStatus(uint8_t *QtStatusPtr)
{	
#if (QT_DEVICE == QT1070) || (QT_DEVICE == QT1121) || (QT_DEVICE == QT2120)\
		|| (QT_DEVICE == QT2160)
	// Turn on the corresponding LED for the detected key0 to key7
	PORTB = ~QtStatusPtr[1];
#elif (QT_DEVICE == QT1060) || (QT_DEVICE == QT60160) || (QT_DEVICE == QT60240)
	// Turn on the corresponding LED for the detected key0 to key7
	PORTB = ~QtStatusPtr[0];
#endif
}

/*============================================================================
  END OF FILE 
============================================================================*/	