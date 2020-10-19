/* main.c : Lab Power Supply 
 */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/iwdg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mcuio.h"
#include "led_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 led_drv.c 

typedef struct _led_db
{
  uint32_t port;
  uint32_t bit;
  uint32_t invert;
} led_db_t;

#define OUTPUT_LED 0
#define MEM1_LED   1
#define MEM2_LED   2
#define DEBUG_LED  3
#define OC_LED     4

led_db_t led[] = {
  {GPIOB, GPIO1, 1},            // OUTPUT_LED
  {GPIOB, GPIO14, 1},           // MEM1_LED
  {GPIOB, GPIO15, 1},           // MEM2_LED
  {GPIOC, GPIO12, 0},           // DEBUG_LED
  {GPIOC, GPIO11, 0},           // OC_LED
};

void
led_setup (int led_nub)
{
  if (led_nub < LED_MAX) {
    gpio_set_mode (led[led_nub].port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                   led[led_nub].bit);
  }
  led_off (led_nub);
}

void
led_all_setup (void)
{
  int idx = 0;
  for (idx = 0; (idx < LED_MAX); idx++) {
    led_setup (idx);
  }
}

void
led_on (int led_nub)
{
  if (led_nub < LED_MAX) {
    if (led[led_nub].invert) {
      gpio_set (led[led_nub].port, led[led_nub].bit);
    }
    else {
      gpio_clear (led[led_nub].port, led[led_nub].bit);
    }
  }
}

void
led_all_on (void)
{
  int idx = 0;
  for (idx = 0; (idx < LED_MAX); idx++) {
    led_on (idx);
  }
}

void
led_off (int led_nub)
{
  if (led_nub < LED_MAX) {
    if (led[led_nub].invert) {
      gpio_clear (led[led_nub].port, led[led_nub].bit);
    }
    else {
      gpio_set (led[led_nub].port, led[led_nub].bit);
    }
  }
}

void
led_all_off (void)
{
  int idx = 0;
  for (idx = 0; (idx < LED_MAX); idx++) {
    led_off (idx);
  }
}

void
led_toggle (int led_nub)
{
  if (led_nub < LED_MAX) {
    gpio_toggle (led[led_nub].port, led[led_nub].bit);
  }
}

void
led_all_toggle (void)
{
  int idx = 0;
  for (idx = 0; (idx < LED_MAX); idx++) {
    led_toggle (idx);
  }
}
