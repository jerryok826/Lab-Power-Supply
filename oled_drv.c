///////////////////////////////////////////////////////////////////////
// meter.c -- Implementation for Analog Meter
// Date: Wed Dec  6 22:51:32 2017   (C) ve3wwg@gmail.com
///////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdint.h>

#include "ugui.h"
#include "oled_drv.h"
#include "miniprintf.h"
#include "main.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>

// indent -gnu -br -cli2 -lp -nut -l100 oled_drv.c

int my_spi = SPI1;

// #define PIX_COL_SIZE (128)
#define PIX_COL_SIZE (256)

#define PIX_ROW_SIZE (64)
// #define PIX_ROW_SIZE (128)

static UG_GUI gui;
static uint8_t pixmap[PIX_COL_SIZE * PIX_ROW_SIZE / 8];
// static uint8_t dummy;

#if 0
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const uint8_t logo16_glcd_bmp[] = { 
  0B00000000, 0B11000000,
  0B00000001, 0B11000000,
  0B00000001, 0B11000000,
  0B00000011, 0B11100000,
  0B11110011, 0B11100000,
  0B11111110, 0B11111000,
  0B01111110, 0B11111111,
  0B00110011, 0B10011111,
  0B00011111, 0B11111100,
  0B00001101, 0B01110000,
  0B00011011, 0B10100000,
  0B00111111, 0B11100000,
  0B00111111, 0B11110000,
  0B01111100, 0B11110000,
  0B01110000, 0B01110000,
  0B00000000, 0B00110000
};
#endif

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

#if 0
static uint8_t *
to_pixel (short x, short y, unsigned *bitno)
{
  *bitno = 7 - y % 8;           // Inverted

  if (x < 0 || x >= PIX_COL_SIZE || y < 0 || y >= PIX_ROW_SIZE) {
    return &dummy;
  }
  unsigned inv_y = (PIX_ROW_SIZE - 1) - y;
  unsigned pageno = inv_y / 8;
  unsigned colno = x % PIX_COL_SIZE;

  return &pixmap[pageno * PIX_COL_SIZE + colno];
}
#endif

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

#if 0
void
load_stars_pixmap (void)
{
  uint8_t *pp = pixmap;
  uint8_t x, y;
  uint8_t i = 0;

  memset (pixmap, 0x00, sizeof (pixmap));

  for (i = 0; i < (64 / 16); i++) {     // number of stars rows in vertical
    for (y = 0; y < 16; y++) {  // number of vertical bits in stars
      for (x = 0; x < (256 / 16); x++) {
        *pp++ = logo16_glcd_bmp[y * 2 + 0];
        *pp++ = logo16_glcd_bmp[y * 2 + 1];
      }
    }
  }
}
#endif

void
clear_pixmap (void)
{
  memset (pixmap, 0x00, sizeof (pixmap));
}

void
oled_update_ssd1322 (void)      // SSD13ee OLED chip
{
  // ssd1322_command(SSD1322_SETCOLUMNADDR);  // 0x15
  // ssd1322_data(MIN_SEG); // #define MIN_SEG        0x1C
  // ssd1322_data(MAX_SEG); // #define MAX_SEG        0x5B
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
    oled_data2 (destArray, 64);
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
oled_command (uint8_t byte)
{
  gpio_clear (GPIOB, GPIO10);   // D/C pin  
  spi_enable (my_spi);
  spi_xfer (my_spi, byte);
  spi_disable (my_spi);
}

void
oled_data (uint8_t byte)
{
  gpio_set (GPIOB, GPIO10);     // D/C pin
  spi_enable (my_spi);
  spi_xfer (my_spi, byte);
  spi_disable (my_spi);
}

void
oled_data2 (uint8_t * pByte, uint8_t len)
{
  int i = 0;
  gpio_set (GPIOB, GPIO10);     // D/C pin
  spi_enable (my_spi);
  for (i = 0; i < len; i++) {
    spi_xfer (my_spi, *pByte++);
  }
  spi_disable (my_spi);
}

// Fill transfer
void
oled_data3 (uint8_t byte, uint8_t len)
{
  int i = 0;
  gpio_set (GPIOB, GPIO10);  // D/C pin
  spi_enable (my_spi);
  for (i = 0; i < len; i++) {
    spi_xfer (my_spi, byte);
  }
  spi_disable (my_spi);
}

void
oled_reset(int reset)
{ 
  if (reset) {
     gpio_clear (GPIOB, GPIO11);  // Reset pin
  } else {
     gpio_set (GPIOB, GPIO11);  // Reset pin
  }
}

static void
oled_reset_pulse(void) 
{
//  gpio_set (GPIOB, GPIO11);  // Reset pin
  oled_reset(0);
  millisecs_sleep(2);
//  gpio_clear (GPIOB, GPIO11); // Reset pin
  oled_reset(1);
  millisecs_sleep(2);
//  gpio_set (GPIOB, GPIO11);  // Reset pin
  oled_reset(0);  
  millisecs_sleep(2);
}

#if 0
void
oled_init (void)          // SSD1306 128X64
{
  static uint8_t cmds[] = {
    0xAE, 0x00, 0x10, 0x40, 0x81, 0xCF, 0xA1, 0xA6,
    0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
    0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF, 0xFF
  };

  gpio_clear (GPIOC, GPIO12);
  oled_reset_pulse ();
  for (unsigned ux = 0; cmds[ux] != 0xFF; ++ux) {
    oled_command (cmds[ux]);
  }
  gpio_set (GPIOC, GPIO12);
}
#endif

void
oled_display_off(void)
{
  oled_command (SSD1322_DISPLAYOFF); // was 0xAE
}

void
oled_init_ssd1322 (void)   // SSD1322 256X64
{
  oled_reset_pulse ();
  // wait minimum 200millisecs before sending commands
  millisecs_sleep(300); 

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
  oled_ssd1322_fill (0x00);

  oled_command (SSD1322_DISPLAYON); // 0xAF
}

int
spi_1_init (void)
{
  // SPI 1
  rcc_periph_clock_enable (RCC_SPI1);
  rcc_periph_clock_enable (RCC_GPIOA);

  // PB10 -> D/C, PB11 -> RES
//  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO11);
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);

  // PA7=MOSI, PA5=SCK
  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO5);
  // NSS=PA15
  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);

  spi_reset (SPI1);
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master (SPI1,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_256,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_128,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_64,       // 4 worked, 2 was nosiy 
                   SPI_CR1_BAUDRATE_FPCLK_DIV_16,       // 4 worked, 2 was nosiy 
                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
//                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
//                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_disable_software_slave_management (SPI1);
  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_ss_output (SPI1);
  my_spi = SPI1;
  return 0;
}

int
spi_2_init (void)
{
  // SPI 2
  rcc_periph_clock_enable (RCC_SPI2);
  rcc_periph_clock_enable (RCC_GPIOB);

  // PB10 -> D/C, PB11 -> RES
//  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO11);
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);

  // PB15=MOSI, PB13=SCK, NSS=PB12
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                 GPIO13 | GPIO15 | GPIO12);
  // MISO=PB14
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);

  spi_reset (SPI2);
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master (SPI2,
                   SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_disable_software_slave_management (SPI2);
  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_ss_output (SPI2);
  my_spi = SPI2;
  return 0;
}

// End oled_drv.c
