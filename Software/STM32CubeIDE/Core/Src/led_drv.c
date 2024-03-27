/* 
 * led_drv.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

#include "led_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 led_drv.c 

typedef struct _led_db
{
  GPIO_TypeDef *port;
  uint32_t bit;
  uint32_t invert;
} led_db_t;

led_db_t led[] = {
  {GPIOB, GPIO_PIN_1, 1},            // OUTPUT_LED
  {GPIOB, GPIO_PIN_14, 1},           // MEM1_LED
  {GPIOB, GPIO_PIN_15, 1},           // MEM2_LED
  {GPIOC, GPIO_PIN_12, 0},           // DEBUG_LED
  {GPIOC, GPIO_PIN_11, 0},           // OC_LED
};

void gpio_set (GPIO_TypeDef *port, int bit)
{
  //	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // MEM2_LED
    HAL_GPIO_WritePin(port, bit, 1);	
}

void gpio_clear (GPIO_TypeDef *port, int bit)
{
  //	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // MEM2_LED
    HAL_GPIO_WritePin(port, bit, 0);	
}

#if 0
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
#endif

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
    HAL_GPIO_TogglePin (led[led_nub].port, led[led_nub].bit);
  }
}

void
led_all_toggle (void)
{
  int idx = 0;
  for (idx = 0; (idx < LED_MAX); idx++) {
     HAL_GPIO_TogglePin (led[idx].port, led[idx].bit);
  }
}
