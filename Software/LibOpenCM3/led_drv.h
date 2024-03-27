#ifndef __LED_DRV_H
#define __LED_DRV_H

#define OUTPUT_LED 0
#define MEM1_LED   1
#define MEM2_LED   2
#define DEBUG_LED  3
#define OC_LED     4
#define LED_MAX    5

void led_setup(int led_nub);
void led_all_setup (void);

void led_on(int led_nub);
void led_all_on (void);
void led_off(int led_nub);
void led_all_off (void);
void led_toggle(int led_nub);
void led_all_toggle (void);

#endif  /* __LED_DRV_H */
