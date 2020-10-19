/******************************************************************************* 
*   $FILE:  main.c
*   Brief: main function for this application
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
#include "LED.h"

/*============================================================================
Function Definitions
============================================================================*/
/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :	main function for QT device interface example 
Input   :   none			
Output  :   none
Return	:	none
Notes   :
============================================================================*/
int main(void)
{	
	/* Initialize communication interface and 
	 * change notification pin and reset pin */
	InitQtInterface();
  
	/* Initialize the IO port for LED indication
	* for touch status */	
	InitTouchStatusPorts();
	
	// Reset QT device
	ResetQT();
	
	// Check communication is ready and able to read Chip ID
	GetCommsReady();
	
    // Read setup block
    if (ReadSetupBlock(sizeof(setup_block), (uint8_t *)&setup_block) == TRUE)
    {	    
        /* TO DO : modify setup block parameters here
        * from default valus if required 
        * For example: To set NTHR for Key 0 to 20
        * setup_block.key0_NTHR = 20; 
        */

        // Write setup block
        if (WriteSetupBlock(sizeof(setup_block),(uint8_t *)&setup_block) == TRUE)
        {	
            while(1)
            {	
                /* Read all status-bytes 
                * if CHANGE pin is asserted */						
                if (CHANGE_PIN)
                {
                    // read all status-bytes
                    ReadKeyStatus(sizeof(QtStatus), QtStatus);
                    
                    /* TO DO : process the received data here...
                    * In this example updating the touch status through LEDs */			
                    UpdateLedStatus(QtStatus);
                }	
            }
        }	
    }
	
	// read or write setup block failed!!!
	while(1);		
}

/*============================================================================
  END OF FILE 
============================================================================*/	
