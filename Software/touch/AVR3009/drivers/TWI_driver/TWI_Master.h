
/******************************************************************************* 
*   $FILE:  TWI_Master.h
*   Brief: Header file for the TWI driver
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

#ifndef __TWI_MASTER_H
#define __TWI_MASTER_H

/*============================================================================
Macros
============================================================================*/

// TWI Status/Control register definitions

#define TWI_BUFFER_SIZE		125		// Set this to the largest message size that 
									// will be sent including address byte.
										
#define TRUE          1
#define FALSE         0


// TWI State codes

// ** General TWI Master staus codes **

// START has been transmitted  
#define TWI_START                  0x08 
// Repeated START has been transmitted
#define TWI_REP_START              0x10
// Arbitration lost
#define TWI_ARB_LOST               0x38  

// **TWI Master Transmitter staus codes**

// SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_ACK            0x18
// SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_ADR_NACK           0x20
// Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_ACK           0x28
// Data byte has been tramsmitted and NACK received
#define TWI_MTX_DATA_NACK          0x30

// ** TWI Master Receiver staus codes **

// SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_ACK            0x40
// SLA+R has been tramsmitted and NACK received
#define TWI_MRX_ADR_NACK           0x48
// Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_ACK           0x50
// Data byte has been received and NACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58

// ** TWI Slave Transmitter staus codes **

// Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK            0xA8
/* Arbitration lost in SLA+R/W as Master; own SLA+R has been received; 
 * ACK has been returned */
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0
// Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_ACK           0xB8
// Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_NACK          0xC0 
/* Last data byte in TWDR has been transmitted (TWEA = “0”); 
 * ACK has been received */
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  

// ** TWI Slave Receiver staus codes **

// Own SLA+W has been received ACK has been returne
#define TWI_SRX_ADR_ACK            0x60
/* Arbitration lost in SLA+R/W as Master; own SLA+W has been received; 
 * ACK has been returned */
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68
// General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70
/* Arbitration lost in SLA+R/W as Master; 
 * General call address has been received; ACK has been returned */
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78
/* Previously addressed with own SLA+W; data has been received; 
 * ACK has been returned */
#define TWI_SRX_ADR_DATA_ACK       0x80
/* Previously addressed with own SLA+W; data has been received; 
 * NOT ACK has been returned */
#define TWI_SRX_ADR_DATA_NACK      0x88
/* Previously addressed with general call; data has been received; 
 * ACK has been returned */
#define TWI_SRX_GEN_DATA_ACK       0x90
/* Previously addressed with general call; data has been received; 
 * NOT ACK has been returned */
#define TWI_SRX_GEN_DATA_NACK      0x98
/* A STOP condition or repeated START condition has been received 
 * while still addressed as Slave */
#define TWI_SRX_STOP_RESTART       0xA0  

// ** TWI Miscellaneous status codes **

// No relevant state information available; TWINT = “0”
#define TWI_NO_STATE               0xF8 
// Bus error due to an illegal START or STOP condition
#define TWI_BUS_ERROR              0x00  

/*============================================================================
  Global definitions
============================================================================*/

union twi_status_reg                       // Status byte holding flags.
{
    unsigned char all;
    struct
    {
        unsigned char last_trans_ok :1;      
        unsigned char unused_bits :7;
    };
};

/*============================================================================
External variables
============================================================================*/
extern union twi_status_reg twi_status;

/*============================================================================
Function definitions
============================================================================*/
void twi_master_initialise(uint8_t bit_rate, uint8_t pre_scalar_value);
uint8_t twi_transceiver_busy(void);
uint8_t twi_get_state_info(void);
uint8_t twi_write(uint8_t slave_address,uint8_t write_address, 
                  uint8_t write_length, uint8_t *write_ptr);
uint8_t twi_rewrite(void);
uint8_t twi_read(uint8_t slave_address,uint8_t read_address, 
                 uint8_t read_length, uint8_t *read_ptr);


#endif /*__TWI_MASTER_H*/
/*============================================================================
  END OF FILE 
============================================================================*/	