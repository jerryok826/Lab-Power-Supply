/******************************************************************************* 
*   $FILE:  TWI_Master.c
*   Brief: TWI driver for master configuration
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
#include "configuration.h"
#include "TWI_Master.h"

/*============================================================================
Local Macro Definitions
============================================================================*/
#define ADDR_SET	((uint8_t)0x01)

/*============================================================================
Static variable definitions
============================================================================*/
// Transceiver buffer
static uint8_t twi_buf[TWI_BUFFER_SIZE];

// Number of bytes to be transmitted.
static uint8_t twi_msg_size;	

// State byte. Default set to TWI_NO_STATE.
static uint8_t twi_state = TWI_NO_STATE;	

// Transceiver buffer
static uint8_t twi_addr;		

// TWI_statusReg is defined in TWI_Master.h	
union twi_status_reg twi_status = {0};		
	
/*============================================================================
Function Definitions
============================================================================*/
/*============================================================================
Name    :   twi_master_initialise
------------------------------------------------------------------------------
Purpose :	Initializes the TWI peripheral as Master 
			Remember to enable interrupts from the application 
			after initializing the TWI.
Input   :   bit_rate: division factor for the bit rate generator (0 - 255)
			pre_scalar_value - pre-scalar for the Bit Rate Generator
			
				pre_scalar_value	Actual Pre-scaler
					0					1
					1					4
					2					16
					3					64
			
Output  :   none
Return	:	none
Notes   :
============================================================================*/

void twi_master_initialise(uint8_t bit_rate, uint8_t pre_scalar_value)
{
    TWBR = bit_rate;				// Set bit rate register 
    TWSR = pre_scalar_value;		// set pre-scalar
    TWDR = 0xFF;					// Default content = SDA released.
    TWCR = (1 << TWEN) |			// Enable TWI-interface and release TWI pins.
            (0 << TWIE) | (0 << TWINT) |				// Disable Interupt.
            (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) |	// No Signal requests.
            (0 << TWWC);	                                 
} 

/*============================================================================
Name    :   twi_transceiver_busy
------------------------------------------------------------------------------
Purpose :	This function to test if the TWI_ISR is busy in write\read.
Input   :   none			
Output  :   none
Return	:	none
Notes   :
============================================================================*/  

uint8_t twi_transceiver_busy(void)
{
    // If TWI Interrupt is enabled then the Transceiver is busy
    return (TWCR & (1<<TWIE));	
}

/*============================================================================
Name    :   twi_get_state_info
------------------------------------------------------------------------------
Purpose :This function to fetch the state information of the previous operation.
        The function will hold execution (loop) until the TWI_ISR has completed
        with the previous operation. If there was an error, then the function 
        will return the TWI State code.
Input   :   none			
Output  :   none
Return	:	none
Notes   :
============================================================================*/

uint8_t twi_get_state_info(void)
{
    // Wait until TWI has completed the transmission.
    while (twi_transceiver_busy());
    // Return error state.
    return (twi_state);             
}

/*============================================================================
Name    :   twi_write
------------------------------------------------------------------------------
Purpose :	Executes multi-byte write to QT-device
			The function will hold execution (loop) until the TWI_ISR has 
			completed with the previous operation, then initialize the 
			next operation , wait till write operation completion and return.
Input   :   SlaveAddress:	Device address on the TWI bus
			WriteAddress:	Register address
			WriteLength	:	Number of bytes to write
			WritePtr	:	Pointer to byte array containing write-data
Output  :   none
Return	:	TRUE if transfer completes, else FALSE
Notes   :
============================================================================*/

uint8_t twi_write(uint8_t slave_address,uint8_t write_address, 
                  uint8_t write_length, uint8_t *write_ptr)
{
    uint8_t temp;
    
    // Wait until TWI is ready for next transmission.
    while (twi_transceiver_busy());		
    
    // Number of data to write including address location
    twi_msg_size = write_length + 1;
    // Store slave address with write setting (Reset the address bit0 for write)
    twi_addr  = (slave_address << 1);
    // Fill the buffer 1st location with wrire address
    twi_buf[0]  = write_address;
    
    // Copy data to the global TWI buffer which is used in the interrupt  
    if (twi_msg_size > 1)				
    {
        for ( temp = 1; temp <= write_length; temp++ )
            twi_buf[ temp ] = write_ptr[ temp - 1 ];
    }
    
    twi_status.all = 0;				// reset status byte holding flags.
    twi_state = TWI_NO_STATE;
    
    TWCR =	(1 << TWEN) |             // TWI Interface enabled.
            (1 << TWIE) | (1 << TWINT) |    // Enable TWI Interupt 
                                            // and clear the flag.
            (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) |	// Initiate a START 
                                                        // condition.
            (0 << TWWC);  
    
    while (twi_transceiver_busy());	// Wait until TWI write operation is done
    
   
    return (twi_status.last_trans_ok);	// Return TWI operation status
}

/*============================================================================
Name    :   twi_rewrite
------------------------------------------------------------------------------
Purpose :	This function to resend the last message. The driver will reuse 
			the data previously put in the transceiver buffers. The function
			will hold execution (loop) until the TWI_ISR has completed with 
			the previous operation, then initialize the next operation, wait 
			till write operation completion and return.
Input   :   none
Output  :   none
Return	:	TRUE if transfer completes, else FALSE
Notes   :
============================================================================*/

uint8_t twi_rewrite(void)
{
    // Wait until TWI is ready for next transmission. 
    while (twi_transceiver_busy()); 
    
    twi_status.all = 0;
    
    twi_state = TWI_NO_STATE ;
    
    TWCR =	(1 << TWEN) |                   // TWI Interface enabled.
            (1 << TWIE) | (1 << TWINT) |    // Enable TWI Interupt and 
            // clear the flag.
            (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a START 
            // condition.
            (0 << TWWC); 
    
    // Wait until TWI write operation is done			   
    while (twi_transceiver_busy());
    
    // Return TWI operation status     		 
    return (twi_status.last_trans_ok);                      
}

/*============================================================================
Name    :   twi_read
------------------------------------------------------------------------------
Purpose :	Executes multi-byte read from QT-device.
			The function will hold execution (loop) until the TWI_ISR has 
			completed with the previous operation, before reading out the data 
			and returning. If there was an error in the previous transmission 
			the function will return the TWI error code.
Input   :   slave_address :	Device address on the I2C bus
			read_address :	Register address
			read_length :	Number of bytes to read
			read_ptr :		Pointer to byte array for read-data
Output  :   none
Return	:	TRUE if transfer completes, else FALSE
Notes   :
============================================================================*/

uint8_t twi_read(uint8_t slave_address,uint8_t read_address, 
                 uint8_t read_length, uint8_t *read_ptr)
{
    uint8_t i;
    
    // Wait until TWI is ready for next transmission. 
    while (twi_transceiver_busy());	
    
    /* write address-pointer to device */
    twi_write(slave_address, read_address, 0, 0);
    
    /* 50 us delay between each write or read cycle
    * between TWI Stop and TWI Start
    */	
    _delay_us(50);	

    //Store slave address with write setting (Set the address bit0 for read) 
    twi_addr  = (slave_address << 1) | ADDR_SET;	
    // reset status byte holding flags.
    twi_status.all = 0;	
    
    twi_msg_size = read_length;	//Number of bytes to read 
    
    TWCR = 	(1 << TWEN) |			// TWI Interface enabled.
    (1 << TWIE) | (1 << TWINT) |    // Enable TWI Interupt and 
                                    // clear the flag.
    (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a START
    (0 << TWWC); 
    
    //Wait until TWI completes the read operation 
    while (twi_transceiver_busy());		
    
    if (twi_status.last_trans_ok)		//Copy data from Transceiver buffer.              
    {                                             
        for (i = 0; i < read_length; i++)	//Copy data from Transceiver buffer.
        {
            read_ptr[i] = twi_buf[i];
        }
    }
    
    return (twi_status.last_trans_ok);                                   
}

/*============================================================================
// ********** TWI Interrupt Handler ********** //
------------------------------------------------------------------------------
Purpose :	This function is the Interrupt Service Routine (ISR), and called 
			when the TWI interrupt is triggered; that is whenever a TWI event 
			has occurred.
Input   :   none
Output  :   none
Return	:	none
Notes   :
============================================================================*/

#if defined (__ICCAVR__)
#pragma vector=TWI_vect
__interrupt void TWI_ISR(void)
#else
ISR(TWI_vect)
#endif
{
    static uint8_t twi_buf_ptr;
    
    switch (TWSR)
    {
        case TWI_START:			// START has been transmitted  
        case TWI_REP_START:		// Repeated START has been transmitted
            twi_buf_ptr = 0;    // Set buffer pointer to TWI Address location
            TWDR = twi_addr;
            TWCR = (1 << TWEN) |
            (1 << TWIE) | (1 << TWINT) | 
            (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) |
            (0 << TWWC);
        break;
    
    case TWI_MTX_ADR_ACK:	// SLA+W has been tramsmitted and ACK received
    case TWI_MTX_DATA_ACK:	// Data byte has been tramsmitted and 
        // ACK received
        if (twi_buf_ptr < twi_msg_size)
        {
            TWDR = twi_buf[twi_buf_ptr++];
            TWCR = (1 << TWEN)|				// TWI Interface enabled
            (1 << TWIE) | (1 << TWINT) |	// Enable TWI Interupt 
                                            // and clear the flag to send byte
            (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) |		
            (0 << TWWC);									
        }
        else								// Send STOP after last byte
        {
            // Set status bits to completed successfully. 
            twi_status.last_trans_ok = TRUE;		
            TWCR = (1 << TWEN) |        // TWI Interface enabled
            (0<<TWIE) | (1<<TWINT) |    // Disable TWI Interrupt and 
                                        // clear the flag
            (0 << TWEA) | (0 << TWSTA) | (1 << TWSTO) | // Initiate a STOP 
            (0 << TWWC);                                 
        }
        break;
    
    case TWI_MRX_DATA_ACK:  // Data byte has been received and ACK tramsmitted
        twi_buf[twi_buf_ptr++] = TWDR;
    case TWI_MRX_ADR_ACK:		// SLA+R has been tramsmitted and ACK received
        if (twi_buf_ptr < (twi_msg_size - 1))// Detect the last byte to NACK it.
        {
            TWCR = (1 << TWEN) |					    // TWI Interface enabled
            (1 << TWIE) | (1 << TWINT) |  // Enable TWI Interupt and clear the 
            // flag to read next byte
            (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // Send ACK
            // after reception
            (0 << TWWC);   
        }
        else                                // Send NACK after next reception
        {
            TWCR = (1 << TWEN) |            // TWI Interface enabled
            (1 << TWIE) | (1 << TWINT) |    // Enable TWI Interupt and clear the 
                                            // flag to read next byte
            (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // Send NACK 
                                                        // after reception
            (0 << TWWC);                                  
        }    
        break; 
    
    // Data byte has been received and NACK tramsmitted		
    case TWI_MRX_DATA_NACK:							
        twi_buf[twi_buf_ptr] = TWDR;
        // Set status bits to completed successfully. 
        twi_status.last_trans_ok = TRUE;			
        TWCR = (1 << TWEN) |			// TWI Interface enabled
        (0 << TWIE) | (1 << TWINT) |	// Disable TWI Interrupt and 
                                        // clear the flag
        (0 << TWEA) | (0 << TWSTA) | (1 << TWSTO) | // Initiate a STOP
        (0 << TWWC);                                 
        break;  
    
    case TWI_ARB_LOST:					// Arbitration lost
        TWCR = (1 << TWEN) |			// TWI Interface enabled
        (1 << TWIE) | (1 << TWINT) |	// Enable TWI Interupt and 
                                        // clear the flag
        (0 << TWEA) | (1 << TWSTA) | (0 << TWSTO) | // Initiate a (RE)START
        (0 << TWWC);                                 
        break;
    
    case TWI_MTX_ADR_NACK:   // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:   // SLA+R has been tramsmitted and NACK received    
    case TWI_MTX_DATA_NACK:  // Data byte has been tramsmitted and NACK received
    case TWI_BUS_ERROR:		// Bus error due to an illegal START or STOP
    default:   
        // Store TWSR and automatically sets clears noErrors bit.
        twi_state = TWSR;			
        /* Reset TWI Interface */
        TWCR = (1 << TWEN) |	// Enable TWI-interface and release TWI pins
        (0 << TWIE) | (0 << TWINT) |			// Disable Interupt
        (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | // No Signal requests
        (0 << TWWC);
        break;                                 
    }
}

/*============================================================================
  END OF FILE 
============================================================================*/
