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
#ifndef _SSD1322_H
    #define _SSD1322_H

    #include "GraphicsConfig.h"
    #include "GenericTypeDefs.h"

// Additional hardware-accelerated functions implemented in the driver.
// These definitions exclude the PutPixel()-based functions from compilation
// in the primitives layer (Primitive.c file).
//#define USE_DRV_FONT
//#define USE_DRV_LINE
//#define USE_DRV_CIRCLE
//#define USE_DRV_FILLCIRCLE
    #define USE_DRV_BAR
    #define USE_DRV_CLEARDEVICE
//   #define USE_DRV_PUTIMAGE

    #ifdef USE_16BIT_PMP
        #error This driver doesn't support 16-bit PMP (remove USE_16BIT_PMP option from HardwareProfile.h)
    #endif
    #ifndef DISP_HOR_RESOLUTION
        #error DISP_HOR_RESOLUTION must be defined in HardwareProfile.h
    #endif
    #ifndef DISP_VER_RESOLUTION
        #error DISP_VER_RESOLUTION must be defined in HardwareProfile.h
    #endif
    #ifndef COLOR_DEPTH
        #error COLOR_DEPTH must be defined in HardwareProfile.h
    #endif
    #ifndef DISP_ORIENTATION
        #error DISP_ORIENTATION must be defined in HardwareProfile.h
    #endif

/*********************************************************************
* Overview: Horizontal and vertical screen size.
*********************************************************************/
    #if (DISP_HOR_RESOLUTION != 256)
        #error This driver doesn't supports this resolution. Horisontal resolution must be 256 pixels.
    #endif
    #if (DISP_VER_RESOLUTION != 64)
        #error This driver doesn't supports this resolution. Vertical resolution must be 64 pixels.
    #endif

/*********************************************************************
* Overview: Display orientation.
*********************************************************************/
    #if (DISP_ORIENTATION != 0)
        #error This driver doesn't support this orientation.
    #endif

/*********************************************************************
* Overview: Color depth.
*********************************************************************/
    #if (COLOR_DEPTH != 16)
        #error This driver doesn't support this color depth. It should be 16.
    #endif

// Clipping region control codes
    #define CLIP_DISABLE    0
    #define CLIP_ENABLE     1

// Color codes
    #define BLACK           (WORD) 0x0
    #define WHITE           (WORD) 0xf

    #define GRAY0           (WORD) 0x0
    #define GRAY1           (WORD) 0x1
    #define GRAY2           (WORD) 0x2
    #define GRAY3           (WORD) 0x3
    #define GRAY4           (WORD) 0x4
    #define GRAY5           (WORD) 0x5
    #define GRAY6           (WORD) 0x6
    #define GRAY7           (WORD) 0x7
    #define GRAY8           (WORD) 0x8
    #define GRAY9           (WORD) 0x9
    #define GRAY10          (WORD) 0xa
    #define GRAY11          (WORD) 0xb
    #define GRAY12          (WORD) 0xc
    #define GRAY13          (WORD) 0xd
    #define GRAY14          (WORD) 0xe
    #define GRAY15          (WORD) 0xf

// Color
extern WORD _color;

// Clipping region control
extern SHORT    _clipRgn;

// Clipping region borders
extern SHORT    _clipLeft;
extern SHORT    _clipTop;
extern SHORT    _clipRight;
extern SHORT    _clipBottom;

// Display commands
	  #define CMD_COL         0x15
    #define CMD_ROW         0x75
    #define CMD_WRITE       0x5C
    #define CMD_READ        0x5D
    #define CMD_DISPON      0xAF
    #define CMD_DISPOFF     0xAE
		#define CMD_ENGREYSCALE	0x00
		#define CMD_MODE				0xA0
		#define CMD_SETSTART		0xA1
		#define CMD_DISPOFFSET	0xA2
		#define CMD_DISPNORM		0xA6
		#define CMD_DISPINVERT	0xA7
		#define CMD_DISPALLON		0xA5
		#define CMD_DISPALLOFF	0xA4
		#define CMD_VDDSEL			0xAB
		#define CMD_PHASELEN		0xB1
		#define CMD_SETCLKFREQ	0xB3
		#define CMD_DISPENHA		0xB4
		#define CMD_SETGPIO			0xB5
		#define CMD_SECPRECHRG	0xB6
		#define CMD_SETGRYTABLE	0xB8
		#define CMD_DEFGRYTABLE	0xB9
		#define CMD_PRECHRGVOL	0xBB
		#define CMD_SETVCOMH		0xBE
		#define CMD_CONTRSTCUR	0xC1
		#define CMD_MSTCONTRST	0xC7
		#define CMD_MUXRATIO		0xCA
		#define CMD_DISPENHB		0xD1
		#define CMD_COMLOCK			0xFD


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
* Overview: resets device, initialize PMP
*
* Note: none
*
********************************************************************/
void ResetDevice(void);

/*********************************************************************
* Macros:  GetMaxX()
*
* PreCondition: none
*
* Input: none
*
* Output: maximum horizontal coordinate
*
* Side Effects: none
*
* Overview: returns maximum horizontal coordinate
*
* Note: none
*
********************************************************************/
    #define GetMaxX()   (DISP_HOR_RESOLUTION - 1)

/*********************************************************************
* Macros:  GetMaxY()
*
* PreCondition: none
*
* Input: none
*
* Output: maximum vertical coordinate
*
* Side Effects: none
*
* Overview: returns maximum vertical coordinate
*
* Note: none
*
********************************************************************/
    #define GetMaxY()   (DISP_VER_RESOLUTION - 1)

/*********************************************************************
* Macros:  SetColor(color)
*
* PreCondition: none
*
* Input: color coded in format:
*           bits 03 02 01 00 
*          color  H  H  H  H
*
* Output: none
*
* Side Effects: none
*
* Overview: sets current color
*
* Note: none
*
********************************************************************/
    #define SetColor(color) _color = color;

/*********************************************************************
* Macros:  GetColor()
*
* PreCondition: none
*
* Input: none
*
* Output: color coded in format:
*           bits 03 02 01 00 
*          color  H  H  H  H
*
* Side Effects: none
*
* Overview: returns current color
*
* Note: none
*
********************************************************************/
    #define GetColor()  _color

/*********************************************************************
* Macros:  SetActivePage(page)
*
* PreCondition: none
*
* Input: graphic page number
*
* Output: none
*
* Side Effects: none
*
* Overview: sets active graphic page 
*
* Note: SSD1322 has only page
*
********************************************************************/
    #define SetActivePage(page)

/*********************************************************************
* Macros: SetVisualPage(page)
*
* PreCondition: none
*
* Input: graphic page number
*
* Output: none
*
* Side Effects: none
*
* Overview: sets graphic page to display
*
* Note: SSD1322 has only page
*
********************************************************************/
    #define SetVisualPage(page)

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
void    PutPixel(SHORT x, SHORT y);

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
* Overview: returns pixel color at x,y position
*
* Note: none
*
********************************************************************/
WORD    GetPixel(SHORT x, SHORT y);

/*********************************************************************
* Macros: SetClipRgn(left, top, right, bottom)
*
* PreCondition: none
*
* Input: left,top,right,bottom - clipping region borders
*
* Output: none
*
* Side Effects: none
*
* Overview: sets clipping region 
*
* Note: none
*
********************************************************************/
    #define SetClipRgn(left, top, right, bottom) \
    _clipLeft = left;                            \
    _clipTop = top;                              \
    _clipRight = right;                          \
    _clipBottom = bottom;

/*********************************************************************
* Macros: GetClipLeft()
*
* PreCondition: none
*
* Input: none
*
* Output: left clipping border
*
* Side Effects: none
*
* Overview: returns left clipping border
*
* Note: none
*
********************************************************************/
    #define GetClipLeft()   _clipLeft

/*********************************************************************
* Macros: GetClipRight()
*
* PreCondition: none
*
* Input: none
*
* Output: right clipping border
*
* Side Effects: none
*
* Overview: returns right clipping border
*
* Note: none
*
********************************************************************/
    #define GetClipRight()  _clipRight

/*********************************************************************
* Macros: GetClipTop()
*
* PreCondition: none
*
* Input: none
*
* Output: top clipping border
*
* Side Effects: none
*
* Overview: returns top clipping border
*
* Note: none
*
********************************************************************/
    #define GetClipTop()    _clipTop

/*********************************************************************
* Macros: GetClipBottom()
*
* PreCondition: none
*
* Input: none
*
* Output: bottom clipping border
*
* Side Effects: none
*
* Overview: returns bottom clipping border
*
* Note: none
*
********************************************************************/
    #define GetClipBottom() _clipBottom

/*********************************************************************
* Macros: SetClip(control)
*
* PreCondition: none
*
* Input: control - 0 disable/ 1 enable
*
* Output: none
*
* Side Effects: none
*
* Overview: enables/disables clipping 
*
* Note: none
*
********************************************************************/
    #define SetClip(control)    _clipRgn = control;

/*********************************************************************
* Macros: IsDeviceBusy()
*
* PreCondition: none
*
* Input: none
*
* Output: busy status
*
* Side Effects: none
*
* Overview:  returns non-zero if LCD controller is busy 
* (previous drawing operation is not complete)
*
* Note: SSD1322 is always accessible
*
********************************************************************/
    #define IsDeviceBusy()  0

/*********************************************************************
* Macros: SetPalette(colorNum, color)
*
* PreCondition: none
*
* Input: colorNum - register number, color - color
*
* Output: none
*
* Side Effects: none
*
* Overview:  sets palette register
*
* Note: SSD1322 has no palette
*
********************************************************************/
    #define SetPalette(colorNum, color)


#endif // _SSD1322_H
