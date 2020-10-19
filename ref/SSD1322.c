/*****************************************************************************
 *  Module for Microchip Graphics Library
 *  Solomon Systech SSD1322 OLED display controller driver
  *****************************************************************************
 * FileName:        SSD1322.c
 * Dependencies:    Graphics.h
 * Processor:       PIC32
 * Compiler:       	MPLAB C32
 * Linker:          MPLAB LINK32
 *
 * Software License Agreement
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rowan Taubitz			1/06/2011			Initial driver
 *****************************************************************************/
#include "Graphics/Graphics.h"

// Color
WORD		    _color;

// Clipping region control
SHORT       _clipRgn;

// Clipping region borders
SHORT       _clipLeft;
SHORT       _clipTop;
SHORT       _clipRight;
SHORT       _clipBottom;

/////////////////////// LOCAL FUNCTIONS PROTOTYPES ////////////////////////////
/*void        PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch);
void        PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch);
void        PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch);
void        PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch);

void        PutImage1BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch);
void        PutImage4BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch);
void        PutImage8BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch);
void        PutImage16BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch);
*/

/*********************************************************************
* Macros:  SetAddress(x, y)
*
* PreCondition: none
*
* Input: x,y - start x,y position
*
* Output: none
*
* Side Effects: none
*
* Overview: sets pointer to to write/read operations
*
* Note: none
*
********************************************************************/
#define SetAddress(x, y) \
    DisplaySetCommand(); \
    DeviceWrite(CMD_COL); \
		DisplaySetData(); \
    DeviceWrite(x + 0x1C); \
    DisplaySetCommand(); \
    DeviceWrite(CMD_ROW); \
		DisplaySetData(); \
    DeviceWrite(y);
      
/*********************************************************************
* Function:  void ResetDevice()
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: resets LCD, initializes PMP
*
* Note: none
*
********************************************************************/
void ResetDevice(void)
{
	// Initialize device
	DeviceInit();

	DisplayEnable();	// Enable LCD
	DisplaySetCommand();
	DeviceWrite(CMD_COMLOCK); // Send lock Command
	DisplaySetData();
	DeviceWrite(0x12);	// Unlock device 
	DisplaySetCommand();
	DeviceWrite(CMD_DISPOFF);	// Turn the display off
	DeviceWrite(CMD_SETCLKFREQ);	// Set Front Clock Divider / Oscillator Frequency
	DisplaySetData();
	DeviceWrite(0x91); // 0b1001.0001
	DisplaySetCommand();
	DeviceWrite(CMD_MUXRATIO);	// Set MUX Ratio
	DisplaySetData();
	DeviceWrite(0x3F);	// Duty = 1/64
	DisplaySetCommand();
	DeviceWrite(CMD_DISPOFFSET);	// Set Display Offset
	DisplaySetData();
	DeviceWrite(0x00);
	DisplaySetCommand();
	DeviceWrite(CMD_SETSTART);	// Set Display Start Line 
	DisplaySetData();
	DeviceWrite(0x00);
	DisplaySetCommand(); 
	DeviceWrite(CMD_COL); // Set Column Address
	DisplaySetData(); 
	DeviceWrite(0x1C); // Start column (offset of 28)
	DeviceWrite(0x5B); // End column = 256, 4 per column = 64-1+offset(28) = 91
	DisplaySetCommand(); 
	DeviceWrite(CMD_ROW); // Set Row Address
	DisplaySetData(); 
	DeviceWrite(0x00); // Start row
	DeviceWrite(0x3F); // 64-1 rows
	DisplaySetCommand();
	DeviceWrite(CMD_MODE);	// Set Re-map and Dual COM Line mode
	DisplaySetData();
	DeviceWrite(0x14); // Horizontal address increment, Disable Column Address Re-map, Enable Nibble Re-map, Scan from COM0 to COM[N–1] (where N is the Multiplex ratio), Disable COM Split Odd Even
	DeviceWrite(0x11); // Enable Dual COM mode (MUX = 63)
	DisplaySetCommand();
	//Write_Command(CMD_SETGPIO); //GPIO
	//Write_Command(0x00); 
	DeviceWrite(CMD_VDDSEL);	// Function Selection
	DisplaySetData();
	DeviceWrite(0x01);	// Enable internal VDD  regulator
	DisplaySetCommand();
	DeviceWrite(CMD_DISPENHA);	// Display Enhancement A
	DisplaySetData();
	DeviceWrite(0xA0);
	DeviceWrite(0xfd);
	DisplaySetCommand();
	DeviceWrite(CMD_CONTRSTCUR);	// Set Contrast Current
	DisplaySetData();
	DeviceWrite(0x9f);
	DisplaySetCommand();
	DeviceWrite(CMD_MSTCONTRST);	// Master Contrast Current Control
	DisplaySetData();
	DeviceWrite(0x0f);
	DisplaySetCommand();
	DeviceWrite(CMD_DEFGRYTABLE);	// Select Default Linear Gray Scale table
	DeviceWrite(CMD_ENGREYSCALE);	// Enable Gray Scale table 
	DeviceWrite(CMD_PHASELEN);	// Set Phase Length
	DisplaySetData();
	DeviceWrite(0xE2);
	DisplaySetCommand();
	DeviceWrite(CMD_DISPENHB);	// Display Enhancement  B
	DisplaySetData();
	DeviceWrite(0x82);	// User is recommended to set A[5:4] to 00b.
	DeviceWrite(0x20);	// Default
	DisplaySetCommand();
	DeviceWrite(CMD_PRECHRGVOL);	// Set Pre-charge voltage
	DisplaySetData();
	DeviceWrite(0x1F);
	DisplaySetCommand();
	DeviceWrite(CMD_SECPRECHRG);	// Set Second Precharge Period
	DisplaySetData();
	DeviceWrite(0x08);
	DisplaySetCommand();
	DeviceWrite(CMD_SETVCOMH);	// Set VCOMH
	DisplaySetData();
	DeviceWrite(0x07);    
	DisplaySetCommand();
	DeviceWrite(CMD_DISPNORM);	// Normal Display
	DeviceWrite(CMD_DISPON);	// Sleep mode OFF
	
	// Disable LCD
	DisplayDisable();
}

/*********************************************************************
* Function: void PutPixel(SHORT x, SHORT y)
*
* PreCondition: none
*
* Input: x,y - pixel coordinates
*
* Output: none
*
* Side Effects: none
*
* Overview: puts pixel
*
* Note: none
*
********************************************************************/
void PutPixel(SHORT x, SHORT y)
{
	BYTE    columnPixel[2];         // 4 Pixels in each column, 2 Pixels per byte read
	
	if(_clipRgn)
	{
		if(x < _clipLeft)
			return;
		if(x > _clipRight)
			return;
		if(y < _clipTop)
			return;
		if(y > _clipBottom)
			return;
	}
	
	DisplayEnable();
   // Set Column and Row Address
	SetAddress(x / 4, y);

    // Read Column
	DisplaySetCommand();
	DeviceWrite(CMD_READ);
	DisplaySetData();
	columnPixel[0] = SingleDeviceRead();    // Dummy reading
	columnPixel[0] = DeviceRead();    // Start reading cycle for pixel 0, 1
	columnPixel[1] = DeviceRead();    // Start reading cycle for pixel 2, 3

	
	// Modify pixel
	switch (x % 4) {
		case 0:
			// needs to be lhs of column 1
			columnPixel[0] = (_color << 4) | (columnPixel[0] & 0x0F);
			break;
		case 1:
			// needs to be rhs of column 1
			columnPixel[0] = (_color) | (columnPixel[0] & 0xF0);
			break;
		
		case 2:
			// needs to be lhs of column 2
			columnPixel[1] = (_color << 4) | (columnPixel[1] & 0x0F);
			break;
		
		case 3:
			// needs to be rhs of column 2
			columnPixel[1] = (_color) | (columnPixel[1] & 0xF0);
	}		
	
	SetAddress(x / 4, y);
	DisplaySetCommand();
	DeviceWrite(CMD_WRITE);
	DisplaySetData();
	// Write Column back
	DeviceWrite(columnPixel[0]);
	DeviceWrite(columnPixel[1]);

	DisplayDisable();
}

/*********************************************************************
* Function: WORD GetPixel(SHORT x, SHORT y)
*
* PreCondition: none
*
* Input: x,y - pixel coordinates 
*
* Output: pixel color
*
* Side Effects: none
*
* Overview: return pixel color at x,y position
*
* Note: none
*
********************************************************************/
WORD GetPixel(SHORT x, SHORT y)
{
	BYTE    columnPixel[2];         // 4 Pixels in each column, 2 Pixels per byte read
	
  DisplayEnable();
   // Set Column and Row Address
	SetAddress(x / 4, y);

    // Read Column
	DisplaySetCommand();
	DeviceWrite(CMD_READ);
	DisplaySetData();
	columnPixel[0] = SingleDeviceRead();    // Dummy reading
	columnPixel[0] = DeviceRead();    // Start reading cycle for pixel 0, 1
	columnPixel[1] = DeviceRead();    // Start reading cycle for pixel 2, 3
	DisplayDisable();
	// Return pixel
	switch (x % 4) {
		case 0:
			// needs to be lhs of column 1
			return (columnPixel[0] & 0xF0) >> 4;
			break;
		case 1:
			// needs to be rhs of column 1
			return columnPixel[0] & 0x0F;
			break;
		
		case 2:
			// needs to be lhs of column 2
			return (columnPixel[1] & 0xF0) >> 4;
			break;
		
		case 3:
			// needs to be rhs of column 2
			return columnPixel[1] & 0x0F;
	}	
  return 0;
}

/*********************************************************************
* Function: WORD Bar(SHORT left, SHORT top, SHORT right, SHORT bottom)
*
* PreCondition: none
*
* Input: left,top - top left corner coordinates,
*        right,bottom - bottom right corner coordinates
*
* Output: For NON-Blocking configuration:
*         - Returns 0 when device is busy and the shape is not yet completely drawn.
*         - Returns 1 when the shape is completely drawn.
*         For Blocking configuration:
*         - Always return 1.
*
* Side Effects: none
*
* Overview: draws rectangle filled with current color
*
* Note: none
*
********************************************************************/

WORD Bar(SHORT left, SHORT top, SHORT right, SHORT bottom)
{
	SHORT   x, y;
	BYTE    frontColumnPixel[2];	// 4 Pixels in each column, 2 Pixels per byte read
	BYTE    endColumnPixel[2]; // Column data that the bar ends in

	#ifndef USE_NONBLOCKING_CONFIG
	while(IsDeviceBusy() != 0);
	
	#else
	if(IsDeviceBusy() != 0)
	    return (0);
	#endif
	
	if (right < left) return 1;
	
	if(_clipRgn)
	{
		if(left < _clipLeft)
	  	left = _clipLeft;
		if(right > _clipRight)
			right = _clipRight;
		if(top < _clipTop)
			top = _clipTop;
		if(bottom > _clipBottom)
			bottom = _clipBottom;
	}
	if (left < 0)
		left = 0;
	if (right > 255)
		right = 255;
		
	SHORT leftCol = left / 4;
	SHORT rightCol = right / 4;
	BYTE leftModulo = left % 4;
	BYTE rightModulo = right % 4;
	BYTE colWidth = rightCol - leftCol;

	DisplayEnable();

	// If both left and right are in the same column (left %4) <= (right % 4)
	if ((right - left < 4) && (leftModulo <= rightModulo))
	{
		// read the column data, work out which pixels need to be modified, mod them and write column
		for(y = top; y < bottom + 1; y++)
		{
		SetAddress(leftCol, y);
		DisplaySetCommand();
		DeviceWrite(CMD_READ);
		DisplaySetData();
		frontColumnPixel[0] = SingleDeviceRead();    // Dummy reading
		frontColumnPixel[0] = endColumnPixel[0] = DeviceRead();    // Start reading cycle for pixel 0, 1
		frontColumnPixel[1] = endColumnPixel[1] = DeviceRead();    // Start reading cycle for pixel 2, 3
		
		// Write out changes
		SetAddress(leftCol, y);
		DisplaySetCommand();
		DeviceWrite(CMD_WRITE);
		DisplaySetData();
		
		switch (leftModulo) {
			case 0:
				// modify from pixel 1 by number of right - left pixels
				frontColumnPixel[0] = (_color << 4) | _color;
				frontColumnPixel[1] = (_color << 4) | _color;
				break;
			case 1:
				// modify from pixel 2 onwards (maintain pixel 1)
				frontColumnPixel[0] = (_color) | (frontColumnPixel[0] & 0xF0);
				frontColumnPixel[1] = (_color << 4) | _color;
				break;
			
			case 2:
				// modify from pixel 3 onwards (maintain pixel 1 & 2)
				frontColumnPixel[1] = (_color << 4) | _color;
				break;
			
			case 3:
				// modify from pixel 4 onwards (maintain pixel 1, 2 & 3)
				frontColumnPixel[1] = (_color) | (frontColumnPixel[1] & 0xF0);
				break;
		}
		switch (rightModulo) {
			case 0:
				// maintain pixel 1, reset the rest back to the previous color
				frontColumnPixel[0] = (frontColumnPixel[0] & 0xF0) | (endColumnPixel[0] & 0x0F);
				frontColumnPixel[1] = endColumnPixel[1];
				break;
			case 1:
				// maintain up to pixel 2, reset the rest back to the previous color
				//frontColumnPixel[0] = frontColumnPixel[0];
				frontColumnPixel[1] = endColumnPixel[1];
				break;
			
			case 2:
				// maintain up to pixel 3, reset the rest back to the previous color
				//frontColumnPixel[0] = frontColumnPixel[0];
				frontColumnPixel[1] = (frontColumnPixel[1] & 0xF0) | (endColumnPixel[1] & 0x0F);
				break;
			
			case 3:
				break;
				// maintain exactly what was set, do nothing
		}
		
		// Write Column back
		DeviceWrite(frontColumnPixel[0]);
		DeviceWrite(frontColumnPixel[1]);
		
		}
	} else {
		for(y = top; y < bottom + 1; y++)
		{
			// Read front column
			SetAddress(leftCol, y);
			DisplaySetCommand();
			DeviceWrite(CMD_READ);
			DisplaySetData();
			frontColumnPixel[0] = SingleDeviceRead();    // Dummy reading
			frontColumnPixel[0] = DeviceRead();    // Start reading cycle for pixel 0, 1
			frontColumnPixel[1] = DeviceRead();    // Start reading cycle for pixel 2, 3
			
			// Read end column
			if (colWidth > 1)
			{
				SetAddress(rightCol, y);
				DisplaySetCommand();
				DeviceWrite(CMD_READ);
				DisplaySetData();
				endColumnPixel[0] = SingleDeviceRead();    // Dummy reading
			}	
			endColumnPixel[0] = DeviceRead();    // Start reading cycle for pixel 0, 1
			endColumnPixel[1] = DeviceRead();    // Start reading cycle for pixel 2, 3
			
			// Write any pixels we need to in the front column, then do the middle, then the end column outside of the 'for' loop
			switch (leftModulo) {
				case 0:
					// modify from pixel 1 onwards
					frontColumnPixel[0] = (_color << 4) | _color;
					frontColumnPixel[1] = (_color << 4) | _color;
					break;
				case 1:
					// modify from pixel 2 onwards (maintain pixel 1)
					frontColumnPixel[0] = (_color) | (frontColumnPixel[0] & 0xF0);
					frontColumnPixel[1] = (_color << 4) | _color;
					break;
				
				case 2:
					// modify from pixel 3 onwards (maintain pixel 1 & 2)
					frontColumnPixel[1] = (_color << 4) | _color;
					break;
				
				case 3:
					// modify from pixel 4 onwards (maintain pixel 1, 2 & 3)
					frontColumnPixel[1] = (_color) | (frontColumnPixel[1] & 0xF0);
			}
			
			SetAddress(leftCol, y);
			DisplaySetCommand();
			DeviceWrite(CMD_WRITE);
			DisplaySetData();
			// Write column data back
			DeviceWrite(frontColumnPixel[0]);
			DeviceWrite(frontColumnPixel[1]);
			
			for(x = 0; x < (rightCol - leftCol)-1; x++)
			{
				// Write out all the pixels in the middle
				DeviceWrite((_color << 4) | _color);
				DeviceWrite((_color << 4) | _color);
			}
			
			// Write end column data
			switch (rightModulo) {
				case 0:
					// modify only pixel 1 (maintain pixel 2, 3 & 4)
					endColumnPixel[0] = (_color << 4) | (endColumnPixel[0] & 0x0F);
					break;
				case 1:
					// modify up to pixel 2 (maintain pixel 3 & 4)
					endColumnPixel[0] = (_color << 4) | _color;
					break;
				
				case 2:
					// modify up to pixel 3 (maintain pixel 4)
					endColumnPixel[0] = (_color << 4) | _color;
					endColumnPixel[1] = (_color << 4) | (endColumnPixel[1] & 0x0F);
					break;
				
				case 3:
					// modify all pixels
					endColumnPixel[0] = (_color << 4) | _color;
					endColumnPixel[1] = (_color << 4) | _color;
			}
			// Write column data back
			DeviceWrite(endColumnPixel[0]);
			DeviceWrite(endColumnPixel[1]);
		}	
	}
	DisplayDisable();
  return (1);
}

/*********************************************************************
* Function: void ClearDevice(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: clears screen with current color 
*
* Note: none
*
********************************************************************/
void ClearDevice(void)
{
	DWORD x, y;
	DisplayEnable();
	SetAddress(0,0);
	DisplaySetCommand();
	DeviceWrite(CMD_WRITE);
	DisplaySetData();
	// Initialize array as _color
	for (y = 0; y < (DWORD) (GetMaxY() + 1); y++)
	{
		for (x = 0; x < (DWORD) ((GetMaxX() + 1) / 2); x++)
		{
			DeviceWrite((_color << 4) | (_color));
		}
	}
	DisplayDisable();
}
/*********************************************************************
* Function: WORD PutImage(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: For NON-Blocking configuration:
*         - Returns 0 when device is busy and the image is not yet completely drawn.
*         - Returns 1 when the image is completely drawn.
*         For Blocking configuration:
*         - Always return 1.
*
* Side Effects: none
*
* Overview: outputs image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
/*
WORD PutImage(SHORT left, SHORT top, void *bitmap, BYTE stretch)
{
    FLASH_BYTE  *flashAddress;
    BYTE        colorDepth;
    WORD        colorTemp;

    #ifndef USE_NONBLOCKING_CONFIG
    while(IsDeviceBusy() != 0);

    #else
    if(IsDeviceBusy() != 0)
        return (0);
    #endif

    // Save current color
    colorTemp = _color;

    switch(*((SHORT *)bitmap))
    {
            #ifdef USE_BITMAP_FLASH

        case FLASH:

            // Image address
            flashAddress = ((BITMAP_FLASH *)bitmap)->address;

            // Read color depth
            colorDepth = *(flashAddress + 1);

            // Draw picture
            switch(colorDepth)
            {
                case 1:     PutImage1BPP(left, top, flashAddress, stretch); break;
                case 4:     PutImage4BPP(left, top, flashAddress, stretch); break;
                case 8:     PutImage8BPP(left, top, flashAddress, stretch); break;
                case 16:    PutImage16BPP(left, top, flashAddress, stretch); break;
            }

            break;
            #endif
            #ifdef USE_BITMAP_EXTERNAL

        case EXTERNAL:

            // Get color depth
            ExternalMemoryCallback(bitmap, 1, 1, &colorDepth);

            // Draw picture
            switch(colorDepth)
            {
                case 1:     PutImage1BPPExt(left, top, bitmap, stretch); break;
                case 4:     PutImage4BPPExt(left, top, bitmap, stretch); break;
                case 8:     PutImage8BPPExt(left, top, bitmap, stretch); break;
                case 16:    PutImage16BPPExt(left, top, bitmap, stretch); break;
                default:    break;
            }

            break;
            #endif

        default:
            break;
    }

    // Restore current color
    _color = colorTemp;
    return (1);
}
*/
#ifdef USE_DRV_PUTIMAGE

/*********************************************************************
* Function: void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch)
{
    register FLASH_BYTE *flashAddress;
    register FLASH_BYTE *tempFlashAddress;
    BYTE                temp;
    WORD                sizeX, sizeY;
    WORD                x, y;
    BYTE                stretchX, stretchY;
    WORD                pallete[2];
    BYTE                mask;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Read image size
    sizeY = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;
    pallete[0] = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;
    pallete[1] = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;

    DisplayEnable();
    for(y = 0; y < sizeY; y++)
    {
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            flashAddress = tempFlashAddress;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            mask = 0;
            for(x = 0; x < sizeX; x++)
            {

                // Read 8 pixels from flash
                if(mask == 0)
                {
                    temp = *flashAddress;
                    flashAddress++;
                    mask = 0x80;
                }

                // Set color
                if(mask & temp)
                {
                    SetColor(pallete[1]);
                }
                else
                {
                    SetColor(pallete[0]);
                }

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    DeviceWrite(_color);
                    //DeviceWrite(_color.v[0]);
                }

                // Shift to the next pixel
                mask >>= 1;
            }
        }
    }

    DisplayDisable();
}

/*********************************************************************
* Function: void PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs 16 color image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch)
{
    register FLASH_BYTE *flashAddress;
    register FLASH_BYTE *tempFlashAddress;
    WORD                sizeX, sizeY;
    register WORD       x, y;
    BYTE                temp;
    register BYTE       stretchX, stretchY;
    WORD                pallete[16];
    WORD                counter;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Read image size
    sizeY = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;

    // Read pallete
    for(counter = 0; counter < 16; counter++)
    {
        pallete[counter] = *((FLASH_WORD *)flashAddress);
        flashAddress += 2;
    }

    DisplayEnable();
    for(y = 0; y < sizeY; y++)
    {
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            flashAddress = tempFlashAddress;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read 2 pixels from flash
                if((x & 0x0001) == 0)
                {

                    // Set color
                    SetColor(pallete[temp >> 4]);
                }
                else
                {
                    temp = *flashAddress;
                    flashAddress++;

                    // Set color
                    SetColor(pallete[temp & 0x0f]);
                }

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    DeviceWrite(_color);
                    //DeviceWrite(_color.v[0]);
                }

                // Shift to the next pixel
                temp >>= 4;
            }
        }
    }

    DisplayDisable();
}

/*********************************************************************
* Function: void PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs 256 color image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch)
{
    register FLASH_BYTE *flashAddress;
    register FLASH_BYTE *tempFlashAddress;
    WORD                sizeX, sizeY;
    WORD                x, y;
    BYTE                temp;
    BYTE                stretchX, stretchY;
    WORD                pallete[256];
    WORD                counter;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Read image size
    sizeY = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD *)flashAddress);
    flashAddress += 2;

    // Read pallete
    for(counter = 0; counter < 256; counter++)
    {
        pallete[counter] = *((FLASH_WORD *)flashAddress);
        flashAddress += 2;
    }

    DisplayEnable();
    for(y = 0; y < sizeY; y++)
    {
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            flashAddress = tempFlashAddress;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read pixels from flash
                temp = *flashAddress;
                flashAddress++;

                // Set color
                SetColor(pallete[temp]);

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    //DeviceWrite(_color.v[1]);
                    DeviceWrite(_color);
                }
            }
        }
    }

    DisplayDisable();
}

/*********************************************************************
* Function: void PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs hicolor image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE *bitmap, BYTE stretch)
{
    register FLASH_WORD *flashAddress;
    register FLASH_WORD *tempFlashAddress;
    WORD                sizeX, sizeY;
    register WORD       x, y;
    WORD                temp;
    register BYTE       stretchX, stretchY;

    // Move pointer to size information
    flashAddress = (FLASH_WORD *)bitmap + 1;

    // Read image size
    sizeY = *flashAddress;
    flashAddress++;
    sizeX = *flashAddress;
    flashAddress++;

    DisplayEnable();
    for(y = 0; y < sizeY; y++)
    {
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            flashAddress = tempFlashAddress;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read pixels from flash
                temp = *flashAddress;
                flashAddress++;

                // Set color
                SetColor(temp);

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    //DeviceWrite(_color.v[1]);
                    DeviceWrite(_color);
                }
            }
        }
    }

    DisplayDisable();
}

#endif
#ifdef USE_BITMAP_EXTERNAL

/*********************************************************************
* Function: void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage1BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch)
{
    register DWORD  memOffset;
    BITMAP_HEADER   bmp;
    WORD            pallete[2];
    BYTE            lineBuffer[((GetMaxX() + 1) / 8) + 1];
    BYTE            *pData;
    SHORT           byteWidth;

    BYTE            temp;
    BYTE            mask;
    WORD            sizeX, sizeY;
    WORD            x, y;
    BYTE            stretchX, stretchY;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (2 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 2 * sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 2 * sizeof(WORD);

    // Line width in bytes
    byteWidth = bmp.width >> 3;
    if(bmp.width & 0x0007)
        byteWidth++;

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y = 0; y < sizeY; y++)
    {

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;
        DisplayEnable();
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            pData = lineBuffer;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            mask = 0;
            for(x = 0; x < sizeX; x++)
            {

                // Read 8 pixels from flash
                if(mask == 0)
                {
                    temp = *pData++;
                    mask = 0x80;
                }

                // Set color
                if(mask & temp)
                {
                    SetColor(pallete[1]);
                }
                else
                {
                    SetColor(pallete[0]);
                }

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    //DeviceWrite(_color.v[1]);
                    DeviceWrite(_color);
                }

                // Shift to the next pixel
                mask >>= 1;
            }
        }

        DisplayDisable();
    }
}

/*********************************************************************
* Function: void PutImage4BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage4BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch)
{
    register DWORD  memOffset;
    BITMAP_HEADER   bmp;
    WORD            pallete[16];
    BYTE            lineBuffer[((GetMaxX() + 1) / 2) + 1];
    BYTE            *pData;
    SHORT           byteWidth;

    BYTE            temp;
    WORD            sizeX, sizeY;
    WORD            x, y;
    BYTE            stretchX, stretchY;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (16 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 16 * sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 16 * sizeof(WORD);

    // Line width in bytes
    byteWidth = bmp.width >> 1;
    if(bmp.width & 0x0001)
        byteWidth++;

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y = 0; y < sizeY; y++)
    {

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;
        DisplayEnable();
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            pData = lineBuffer;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read 2 pixels from flash
                if(x & 0x0001)
                {

                    // second pixel in byte
                    SetColor(pallete[temp >> 4]);
                }
                else
                {
                    temp = *pData++;

                    // first pixel in byte
                    SetColor(pallete[temp & 0x0f]);
                }

                // Set color
                SetColor(pallete[temp & 0x0f]);

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    //DeviceWrite(_color.v[1]);
                    DeviceWrite(_color);
                }

                // Shift to the next pixel
                temp >>= 4;
            }
        }

        DisplayDisable();
    }
}

/*********************************************************************
* Function: void PutImage8BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage8BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch)
{
    register DWORD  memOffset;
    BITMAP_HEADER   bmp;
    WORD            pallete[256];
    BYTE            lineBuffer[(GetMaxX() + 1)];
    BYTE            *pData;

    BYTE            temp;
    WORD            sizeX, sizeY;
    WORD            x, y;
    BYTE            stretchX, stretchY;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (256 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 256 * sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 256 * sizeof(WORD);

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y = 0; y < sizeY; y++)
    {

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, sizeX, lineBuffer);
        memOffset += sizeX;
        DisplayEnable();
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            pData = lineBuffer;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read pixels from flash
                temp = *pData++;

                // Set color
                SetColor(pallete[temp]);

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    DeviceWrite(_color);
                    //DeviceWrite(_color.v[0]);
                }
            }
        }

        DisplayDisable();
    }
}

/*********************************************************************
* Function: void PutImage16BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage16BPPExt(SHORT left, SHORT top, void *bitmap, BYTE stretch)
{
    register DWORD  memOffset;
    BITMAP_HEADER   bmp;
    WORD            lineBuffer[(GetMaxX() + 1)];
    WORD            *pData;
    WORD            byteWidth;

    WORD            temp;
    WORD            sizeX, sizeY;
    WORD            x, y;
    BYTE            stretchX, stretchY;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER);

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    byteWidth = sizeX << 1;

    for(y = 0; y < sizeY; y++)
    {

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;
        DisplayEnable();
        for(stretchY = 0; stretchY < stretch; stretchY++)
        {
            pData = lineBuffer;
            SetAddress(left, top + y);
			DisplaySetCommand();
            DeviceWrite(CMD_WRITE);
			DisplaySetData();
            for(x = 0; x < sizeX; x++)
            {

                // Read pixels from flash
                temp = *pData++;

                // Set color
                SetColor(temp);

                // Write pixel to screen
                for(stretchX = 0; stretchX < stretch; stretchX++)
                {
                    DeviceWrite(_color);
                    //DeviceWrite(_color.v[0]);
                }
            }
        }

        DisplayDisable();
    }
}

#endif
