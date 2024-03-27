/* 
 * oled_drv.c 3/4/24
 */

#include "main.h"

#include <string.h>
#include <stdint.h>

#include "ugui.h"
#include "oled_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 oled_drv.c
#if -0
https://github.com/wjklimek1/SSD1322_OLED_library/tree/master

Examples with DMA
In a folder with examples for STM32F411 also two projects utilizing DMA data strasfers were included. 
First one uses DMA in blocking mode, so CPU has to wait for transmission end to leave a function. This 
still gives some preformance boost, especially for frame buffer transfers.

In a second example DMA works in non blocking mode, so CPU only commissions DMA transfer and then 
leaves function. That takes much less CPU time, but library in this example was changed slightly 
to handle CS pin in a different way.

Following changes were made to enable non blocking mode DMA:

CS pin is set high again in SPI_TX_Completed() callback
CS pin is not set high again in API functions
API functions using SPI directly wait with transmission untill previous one was finished by 
polling SPI_transmission_finished flag.
This gives considerable preformance boost when huge data blocks, like frame buffer, are transfered. 
When DMA sends data through SPI, CPU can go on and do other useful task.
#endif

extern SPI_HandleTypeDef hspi1;

// #define PIX_COL_SIZE (128)
#define PIX_COL_SIZE (256)
#define PIX_ROW_SIZE (64)

static UG_GUI gui;
static uint8_t pixmap[PIX_COL_SIZE * PIX_ROW_SIZE / 8];

#define SSD1322_SETCOMMANDLOCK 0xFD
#define SSD1322_DISPLAYOFF 0xAE
#define SSD1322_DISPLAYON 0xAF
#define SSD1322_SETCLOCKDIVIDER 0xB3
#define SSD1322_SETDISPLAYOFFSET 0xA2
#define SSD1322_SETSTARTLINE 0xA1
#define SSD1322_SETREMAP 0xA0
#define SSD1322_FUNCTIONSEL 0xAB
#define SSD1322_DISPLAYENHANCE 0xB4
#define SSD1322_SETCONTRASTCURRENT 0xC1
#define SSD1322_MASTERCURRENTCONTROL 0xC7
#define SSD1322_SETPHASELENGTH 0xB1
#define SSD1322_DISPLAYENHANCEB 0xD1
#define SSD1322_SETPRECHARGEVOLTAGE 0xBB
#define SSD1322_SETSECONDPRECHARGEPERIOD 0xB6
#define SSD1322_SETVCOMH 0xBE
#define SSD1322_NORMALDISPLAY 0xA6
#define SSD1322_INVERSEDISPLAY 0xA7
#define SSD1322_SETMUXRATIO 0xCA
#define SSD1322_SETCOLUMNADDR 0x15
#define SSD1322_SETROWADDR 0x75
#define SSD1322_WRITERAM 0x5C
#define SSD1322_ENTIREDISPLAYON 0xA5
#define SSD1322_ENTIREDISPLAYOFF 0xA4
#define SSD1322_SETGPIO 0xB5
#define SSD1322_EXITPARTIALDISPLAY 0xA9
#define SSD1322_SELECTDEFAULTGRAYSCALE 0xB9

static void
draw_point (short x, short y, short pen)
{
  if ( x < 0 || x >= PIX_COL_SIZE || y < 0 || y >= PIX_ROW_SIZE ) {
     // Add error message or return an error
     return;
  }
  
  // find pixels byte pointer
  register uint8_t *pBuf = &pixmap[(x >> 3) + (y * (PIX_COL_SIZE / 8))];
  
  switch (pen) {
    case 0:
      *pBuf |= (0x80 >> (x % 8));
      break;
    case 1:
      *pBuf &= ~(0x80 >> (x % 8));
      break;
    case 2: // INVERSE:     
      *pBuf ^=  (0x80 >> (x%8)); 
      break;
  }
}

static int
ug_to_pen (UG_COLOR c)
{
  switch (c) {
    case C_BLACK:
      return 0;
    case C_RED:
      return 2;
    default:
      return 1;
  }
}

static UG_COLOR
pen_to_ug (int pen)
{
  switch (pen) {
    case 0:
      return C_BLACK;
    case 2:
      return C_RED;
    default:
      return C_WHITE;
  }
}

static void
local_draw_point (UG_S16 x, UG_S16 y, UG_COLOR c)
{
  draw_point (x, y, ug_to_pen (c));
}

void
oled_init_real (void)
{
  clear_pixmap ();

  // init the ugui driver.
  UG_Init (&gui, local_draw_point, PIX_COL_SIZE, PIX_ROW_SIZE);
  UG_SetBackcolor (pen_to_ug (1));
  UG_SetForecolor (pen_to_ug (0));
}

void
clear_pixmap (void)
{
  memset (pixmap, 0x00, sizeof (pixmap));
}

void
oled_update_ssd1322 (void)
{
  oled_command (SSD1322_SETCOLUMNADDR);  // 0x15
  oled_data (0x1c);
  oled_data (0x5b);

  oled_command (SSD1322_SETROWADDR); // 0x75
  oled_data (0x00);
  oled_data (0x63);

  // ssd1322_command(SSD1322_WRITERAM); // 0x5C
  oled_command (SSD1322_WRITERAM); // 0x5C

  uint16_t srcIndex = 0;
  uint16_t bufSize = sizeof (pixmap);
  uint8_t *pBuf = pixmap;

  while (srcIndex < bufSize) {

    uint8_t destIndex = 0;
    uint8_t destArray[64] = { 0 };
    memset (destArray, 0x00, sizeof (destArray));

    while (destIndex < 64)      // 128 pixels, two nibles per byte
    {
      uint8_t mask = 0x80;

      while (mask > 0) {
        // upper nibble
        destArray[destIndex] |= (pBuf[srcIndex] & mask) ? 0xf0 : 0x00;
        //shift mask to next bit, but this goes into lower nibble.
        mask >>= 1;
        destArray[destIndex] |= (pBuf[srcIndex] & mask) ? 0x0f : 0x00;

        destIndex++;
        mask >>= 1;
      }
      srcIndex++;
    }
    // Send to display here.
    oled_data2 (destArray, 64);  // send 64 bytes to oled
  }
  return;
}

void
oled_ssd1322_fill (uint8_t fill_byte)
{
  int i = 0;

  oled_command (SSD1322_SETCOLUMNADDR);  // 0x15      
  oled_data (0x1C); // #define MIN_SEG  0x1C
  oled_data (0x5B); // #define MAX_SEG  0x5B

  oled_command (SSD1322_SETROWADDR); // 0x75
  oled_data (0x00);
  oled_data (0x3f);

  oled_command (SSD1322_WRITERAM); // 0x5C

  for (i = 0; i < 256; i++) {
    oled_data3 (fill_byte, 64);
    oled_data3 (fill_byte, 64);
  }
}

void
oled_ssd1322_clear (void)
{
  oled_ssd1322_fill (0x00);
}

void
oled_spi_cs_set()
{
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void
oled_spi_cs_clear()
{
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void
oled_command (uint8_t byte)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  oled_spi_cs_clear();
  HAL_SPI_Transmit(&hspi1 ,&byte, 1, 100);
  oled_spi_cs_set();
}

void
oled_data (uint8_t byte)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  oled_spi_cs_clear();
  HAL_SPI_Transmit(&hspi1 , &byte, 1, 100);
  oled_spi_cs_set();
}

void
oled_data2 (uint8_t * pByte, uint8_t len)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

//  oled_spi_cs_clear();
  oled_spi_cs_clear();
  HAL_SPI_Transmit(&hspi1 ,pByte, len, 100); 
  oled_spi_cs_set();
//  oled_spi_cs_set();
}

// Fill transfer
uint8_t big_buf[100];
void
oled_data3 (uint8_t byte, uint8_t len)
{
  int i = 0;
  memset(big_buf, byte, len);
//  gpio_set (GPIOB, GPIO10);  // D/C pin
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);	// 3/3/24
//  spi_enable (my_spi);
//  oled_spi_cs_clear();
 for (i = 0; i < len; i++) {
//    spi_xfer (my_spi, byte);
    oled_spi_cs_clear();
    HAL_SPI_Transmit(&hspi1 ,&byte, 1, 100); // 3/3/24
//    HAL_SPI_Transmit(&hspi1 ,big_buf, len, 100); // 3/3/24
    oled_spi_cs_set();
  } 
//  oled_spi_cs_set();
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);	// 3/3/24 CS line
}

void
oled_reset(int reset)
{ 
  if (reset) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
  } else {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
  }
}

static void
oled_reset_pulse(void) 
{
  oled_reset(0);
  HAL_Delay(2);
  oled_reset(1);
  HAL_Delay(2);
  oled_reset(0);  
  HAL_Delay(2);
}

void
oled_display_off(void)
{
  oled_command (SSD1322_DISPLAYOFF); // was 0xAE
}

void
oled_init_ssd1322 (void)   // SSD1322 256X64
{
  oled_spi_cs_set();

  oled_reset_pulse ();
  // wait minimum 200millisecs before sending commands
//  millisecs_sleep(300); 
  HAL_Delay(300);

  oled_command (SSD1322_SETCOMMANDLOCK); // 0xFD
  oled_data (0x12);        // Unlock OLED driver IC

  oled_command (SSD1322_DISPLAYOFF); // was 0xAE

  oled_command (0x00);          // enable gray scale     

  oled_command (SSD1322_SETCLOCKDIVIDER); // 0xB3
  oled_data (0x91); // 0xB3 ??

  oled_command (SSD1322_SETMUXRATIO); // 0xCA
  oled_data (0x3F); // duty = 1/64

  oled_command (SSD1322_SETDISPLAYOFFSET); // 0xA2
  oled_data (0x00);

  oled_command (SSD1322_SETSTARTLINE); // 0xA1
  oled_data (0x00);

  oled_command (SSD1322_SETREMAP); // 0xA0 
  oled_data (0x14); // Horizontal address increment,Disable Column Address Re-map,
  //      // Enable Nibble Re-map,Scan from COM[N-1] to COM0,Disable COM Split Odd Even  
  oled_data (0x11); //Enable Dual COM mode

  oled_command (SSD1322_SETGPIO); // 0xB5
  oled_data (0x00);  // Disable GPIO Pins Input

  oled_command (SSD1322_FUNCTIONSEL); // 0xAB
  oled_data (0x01); // selection external vdd

  oled_command (SSD1322_DISPLAYENHANCE); // 0xB4
  oled_data (0xa0); // enables the external VSL
  oled_data (0xFD); // 0xfFD,Enhanced low GS display quality; default is 0xb5(normal),

  oled_command (SSD1322_SETCONTRASTCURRENT); // 0xC1
  oled_data (0xFF); // 0xFF - default is 0x7f

  oled_command (SSD1322_MASTERCURRENTCONTROL); // 0xC7
  oled_data (0x0F); // default is 0x0F

  // Set grayscale
  oled_command (SSD1322_SELECTDEFAULTGRAYSCALE); // 0xB9

  oled_command (SSD1322_SETPHASELENGTH); // 0xB1
  oled_data (0xE2); // default is 0x74

  oled_command (SSD1322_DISPLAYENHANCEB);  // 0xD1
  oled_data (0x82); // Reserved; default is 0xa2(normal)
  oled_data (0x20);

  oled_command (SSD1322_SETSECONDPRECHARGEPERIOD); // 0xB6
  oled_data (0x08); // default

  oled_command (SSD1322_SETVCOMH); // 0xBE
  oled_data (0x07); // 0.86xVcc;default is 0x04

  // ssd1322_command(SSD1322_NORMALDISPLAY);// 0xA6
  oled_command (SSD1322_NORMALDISPLAY); // 0xA6

  oled_command (SSD1322_EXITPARTIALDISPLAY); // 0xA9

  //Clear down image ram before opening display
  oled_ssd1322_fill (0x00);  // C_WHITE
  oled_ssd1322_fill (C_WHITE);
  oled_command (SSD1322_DISPLAYON); // 0xAF
}

int
spi_1_init (void)
{
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);	// Set NNS line high // 3/3/24 CS line
  oled_spi_cs_set();

  // SPI 1
  // rcc_periph_clock_enable (RCC_SPI1); // 3/3/24/
  // rcc_periph_clock_enable (RCC_GPIOA);

  // PB10 -> D/C, PB11 -> RES
//  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10); // 3/3/24/

  // PA7=MOSI, PA5=SCK
//  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO5); // 3/3/24/
  // NSS=PA15
//  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4); // 3/3/24/

//  spi_reset (SPI1); // 3/3/24/
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
#if 0
  spi_init_master (SPI1, // 3/3/24/
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_256,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_128,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_64,       // 4 worked, 2 was nosiy 
                   SPI_CR1_BAUDRATE_FPCLK_DIV_16,       // 4 worked, 2 was nosiy 
                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
//                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
//                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
#endif
//  spi_disable_software_slave_management (SPI1); // 3/3/24/
  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
//  spi_enable_ss_output (SPI1); // 3/3/24/
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);	// Set NNS line high // 3/3/24 CS line
   oled_spi_cs_set();
//  my_spi = SPI1;
  return 0;
}

void
splash_scr (void)
{
  char buf[50];

  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);
  UG_FontSelect (&FONT_12X16);

//  UG_FontSelect (&FONT_8X12);

//  snprintf(buf,sizeof buf, "Built:%s %s\r\n",__DATE__,__TIME__);
  snprintf (buf, sizeof buf, "  %s", "Lab Power Supply");
  UG_PutString (0, 0, buf);

  UG_FontSelect (&FONT_8X12);
  snprintf (buf, sizeof buf, "  %s", "   0 to 15 V, 1.5A");
  UG_PutString (0, 17, buf);

#if 1
  snprintf (buf, sizeof buf, "   %s", "By: F.O. Design Works");
//  snprintf (buf, sizeof buf, "   %s", "By: Jerry Okeefe");
  UG_PutString (0, 34, buf);
#else
//  snprintf (buf, sizeof buf, "Built: %s, %s", __DATE__, __TIME__);
  UG_PutString (0, 31,"  LPS ");

//  snprintf (buf, sizeof buf, "Program: %d, Heap: %d", program_bytes, xPortGetFreeHeapSize ());
//    snprintf (buf, sizeof buf, "Jerry OKeefe");

//  UG_PutString (0, 45, "Jerry OKeefe");
#endif
 // oled_ssd1322_fill (C_WHITE);
//  oled_ssd1322_fill (C_BLACK);

  oled_update_ssd1322 (); // 3/3/24
}

