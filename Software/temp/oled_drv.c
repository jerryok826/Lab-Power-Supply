///////////////////////////////////////////////////////////////////////
// meter.c -- Implementation for Analog Meter
// Date: Wed Dec  6 22:51:32 2017   (C) ve3wwg@gmail.com
///////////////////////////////////////////////////////////////////////

#include <string.h>

#include "ugui.h"
#include "oled_drv.h"
#include "miniprintf.h"
#include "oled.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>

int my_spi = SPI1;

#define PIX_COL_SIZE (128)
// #define PIX_ROW_SIZE (64)
#define PIX_ROW_SIZE (128)

static UG_GUI gui;
static uint8_t pixmap[PIX_COL_SIZE*PIX_ROW_SIZE/8];
static uint8_t dummy;

static uint8_t *
to_pixel(short x,short y,unsigned *bitno) {
	*bitno = 7 - y % 8;	// Inverted

	if ( x < 0 || x >= PIX_COL_SIZE
	  || y < 0 || y >= PIX_ROW_SIZE )
	  	return &dummy;

	unsigned inv_y = (PIX_ROW_SIZE-1) - y;
	unsigned pageno = inv_y / 8;
	unsigned colno = x % PIX_COL_SIZE;

	return &pixmap[pageno * PIX_COL_SIZE + colno];
}

static void
draw_point(short x,short y,short pen) {

	if ( x < 0 || x >= PIX_COL_SIZE || y < 0 || y >= PIX_ROW_SIZE )
		return;

	unsigned bitno;
	uint8_t *byte = to_pixel(x,y,&bitno);
	uint8_t mask = 1 << bitno;
	
	switch ( pen ) {
	case 0:
		*byte &= ~mask;
		break;
	case 1:
		*byte |= mask;
		break;
	default:
		*byte ^= mask;
	}
}

static int
ug_to_pen(UG_COLOR c) {

	switch ( c ) {
	case C_BLACK:
		return 0;
	case C_RED:
		return 2;
	default:
		return 1;
	}
}

static UG_COLOR
pen_to_ug(int pen) {

	switch ( pen ) {
	case 0:
		return C_BLACK;
	case 2:
		return C_RED;
	default:
		return C_WHITE;
	}	
}

static void
local_draw_point(UG_S16 x,UG_S16 y,UG_COLOR c) {
	draw_point(x,y,ug_to_pen(c));
}

void
oled_init_real(void) 
{
	memset(pixmap,0,PIX_COL_SIZE*PIX_ROW_SIZE/8);

	UG_Init(&gui,local_draw_point,PIX_COL_SIZE,PIX_ROW_SIZE);
	UG_SetBackcolor(pen_to_ug(1));
	UG_SetForecolor(pen_to_ug(0));
}

void
oled_update(void) 
{
	uint8_t *pp = pixmap;

	oled_command2(0x20,0x02);// Page mode
	oled_command(0x40);
	oled_command2(0xD3,0x00);
	for ( uint8_t px=0; px<8; ++px ) {
		oled_command(0xB0|px);
		oled_command(0x00); // Lo col
		oled_command(0x10); // Hi col
		for ( unsigned bx=0; bx<PIX_COL_SIZE; ++bx )
			oled_data(*pp++);
	}
}

static void
sleep (int ms)
{
  int i, k;
  for (k = 0; k < ms; k++) {
    for (i = 0; i < 1000; i++);
  }
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
oled_command2 (uint8_t byte, uint8_t byte2)
{
  gpio_clear (GPIOB, GPIO10);
  spi_enable (my_spi);
  spi_xfer (my_spi, byte);
  spi_xfer (my_spi, byte2);
  spi_disable (my_spi);
}

void
oled_data (uint8_t byte)
{
  gpio_set (GPIOB, GPIO10);
  spi_enable (my_spi);
  spi_xfer (my_spi, byte);
  spi_disable (my_spi);
}

static void
oled_reset (void)
{
  gpio_clear (GPIOB, GPIO11);   // Reset pin
  sleep (10);
  gpio_set (GPIOB, GPIO11);
}

void
oled_init (void)
{
  static uint8_t cmds[] = {
    0xAE, 0x00, 0x10, 0x40, 0x81, 0xCF, 0xA1, 0xA6,
    0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
    0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF, 0xFF
  };

  gpio_clear (GPIOC, GPIO12);
  oled_reset ();
  for (unsigned ux = 0; cmds[ux] != 0xFF; ++ux) {
    oled_command (cmds[ux]);
  }
  gpio_set (GPIOC, GPIO12);
}

int spi_1_init(void)
{
  // SPI 1
  rcc_periph_clock_enable (RCC_SPI1);
  rcc_periph_clock_enable (RCC_GPIOA);
 
  // PB10 -> D/C, PB11 -> RES
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO11);

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
                   SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
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

int spi_2_init(void)
{
  // SPI 2
  rcc_periph_clock_enable (RCC_SPI2);
  rcc_periph_clock_enable (RCC_GPIOB);
 
  // PB10 -> D/C, PB11 -> RES
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10 | GPIO11);

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
