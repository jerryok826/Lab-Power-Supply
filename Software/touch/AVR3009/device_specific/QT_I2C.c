/******************************************************************************* 
*   $FILE:  QT_I2C.c
*   Brief: Application interfaces to drive the QT device 
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
#include "QT_API.h"

#if (QT_DEVICE == QT1121) || (QT_DEVICE == QT1060) || (QT_DEVICE == QT1070)\
 || (QT_DEVICE == QT2120) || (QT_DEVICE == QT2160) || (QT_DEVICE == QT60160)\
 || (QT_DEVICE == QT60240)
/*============================================================================
Macros
============================================================================*/

#define TWI_BIT_RATE	32
#define TWI_PRE_SCALAR	0

#if (QT_DEVICE == QT60240)
// write unlock data
#define WRITE_UNLOCK 0x55
#endif

/*============================================================================
Global variable definitions
============================================================================*/

// Setup block structure
SetupBlock setup_block;

// application storage for QT device-status
uint8_t QtStatus[QT_STATUS_SIZE];

/*============================================================================
Function Definitions
============================================================================*/
/*============================================================================
Name    :   InitQtInterface
------------------------------------------------------------------------------
Purpose :	Initialize communication interface and change notification pin
			and reset pin
Input   :   none
Output  :   none
Return	:	none
Notes   :
============================================================================*/
void InitQtInterface(void)
{
	// Initialize TWI peripheral for 100 KHz speed	
	twi_master_initialise(TWI_BIT_RATE, TWI_PRE_SCALAR);	
	
	//Configure the CHANGE pin as input
	CONCAT(DDR, CHANGE_STATUS_PORT) &= ~(1u << CHANGE_STATUS_PIN);
  
	//Configure the RESET pin as output
	CONCAT(DDR, RESET_PORT) |= (1u << RESET_PIN);
	
	// Enable global interrupt for TWI interrupt handler  
#if defined (__ICCAVR__)
	__enable_interrupt();
#else
	sei();	
#endif
}

/*============================================================================
Name    :   ReadSetupBlock
------------------------------------------------------------------------------
Purpose :	Read entire setup block from QT-device
Input   :   ReadLength	:	Number of bytes to read
			ReadPtr		:	Pointer to byte array for read-data
Output  :   none
Return	:	TRUE if read successful, else FALSE
Notes   :
============================================================================*/
uint8_t ReadSetupBlock(uint8_t ReadLength, uint8_t *ReadPtr)
{
	// Read setup block
	return (twi_read ( I2C_ADDRESS, QT_SETUPS_BLOCK_ADDR, ReadLength, ReadPtr));
}		

/*============================================================================
Name    :   WriteSetupBlock
------------------------------------------------------------------------------
Purpose :	write entire setup block to QT-device
Input   :   WriteLength	:	Number of bytes to write
			WritePtr	:	Pointer to byte array containing write-data
Output  :   none
Return	:	TRUE if write successful, else FALSE
Notes   :
============================================================================*/
uint8_t WriteSetupBlock(uint8_t WriteLength, uint8_t *WritePtr)
{	
#if (QT_DEVICE == QT60240)	
	uint8_t write_unlock_data = WRITE_UNLOCK;
	
	// Write 0x55 to the address 130 to allow write access to setups
	if(twi_write (I2C_ADDRESS, QT_SETUPS_WRITE_UNLOCK, 
		1, &write_unlock_data) == FALSE)
		{ 
			return FALSE; //return, write operation failed
		}			

#endif
	
	// write setup block
	return(twi_write(I2C_ADDRESS, QT_SETUPS_BLOCK_ADDR, WriteLength, WritePtr));
}

/*============================================================================
Name    :   ReadKeyStatus
------------------------------------------------------------------------------
Purpose :	Read detection status of all keys
Input   :   ReadLength	:	Number of bytes to read
			ReadPtr		:	Pointer to byte array for read-data
Output  :   none
Return	:	TRUE if read successful, else FALSE
Notes   :
============================================================================*/
uint8_t ReadKeyStatus(uint8_t ReadLength, uint8_t *ReadPtr)
{
	// Read detection status of all keys
	return (twi_read ( I2C_ADDRESS, QT_STATUS_ADDR, ReadLength, ReadPtr));
}

/*============================================================================
Name    :   ResetQT
------------------------------------------------------------------------------
Purpose :   Performs a hardware reset of the QT device
Input   :   none
Output  :   none
Return	:   none
Notes   :
============================================================================*/
void ResetQT(void)
{
	// Pull the reset pin LOW
	CONCAT(PORT, RESET_PORT) &= ~(1u << RESET_PIN);
  
	// The reset pin must be low at least 10us to cause a reset
	_delay_us(15);
  
	// Pull line back to high to resume normal operation 
	CONCAT(PORT, RESET_PORT) |= (1u << RESET_PIN); 
}

/*============================================================================
Name    :   GetCommsReady
------------------------------------------------------------------------------
Purpose :   Check communication is ready and able to read Chip ID
Input   :   none
Output  :   none
Return	:   none
Notes   :
============================================================================*/
void GetCommsReady(void)
{
	uint8_t chip_id;	
	
	/* Establish communication with the QT device
	 * wait for successful transfer at the device's address - Read Chip-ID 
	 */
	while (!twi_read(I2C_ADDRESS, QT_CHIP_ID, 1, &chip_id));	

	/* Check that the responding device is a intended device! 
	 * If the read chip ID is wrong, the program goes no further.*/
	while (chip_id != QT_DEVICE_ID);
}

#endif /* (QT_DEVICE == QT1121) || (QT_DEVICE == QT1060) || 
(QT_DEVICE == QT1070) || (QT_DEVICE == QT2120) || (QT_DEVICE == QT2160) || 
(QT_DEVICE == QT60240) */
/*============================================================================
  END OF FILE 
============================================================================*/	