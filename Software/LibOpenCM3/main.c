/* main.c : Lab Power Supply
 */

#include <stdio.h>
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
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mcuio.h"
#include "miniprintf.h"

#include "ugui.h"
#include "oled_drv.h"
#include "i2c.h"
#include "mcp23017.h"
#include "main.h"
#include "led_drv.h"
#include "gcvt_2.h"
#include "at42qt1070.h"
#include "mcp9808.h"

// #define USE_USB 1
// #define TASK_TRACE 1
#define ENABLE_WDT 1

// TO DO
// 1. Add watch dog timer support. Done
// 2. flash mem buttons. Done
// 3. check stack space. Done
// 4. check program size. Done.
// 5. use struture instend of globals
// 6. over-sample and filter ADC results, Done but not sure if its useful
// 7. optimize OLED, run faster, update only delta lines
// 8. use the voltmeter to auto adjust the PS output setting
// 9. Test zero setting. Looks ok but some time jumps to 1.7 volts. Done
// 10. Add encoder range support. Done
// 11. Add start up splash screen. Done
// 12. make presets adjustable, 5V and 3.3 V. Done
// 13. boot up diag. Done
// 14. Fix OLED start up problem. Done
// 15. Add remote calibrate function. Done
// 16. Add program and heap size dump. Done
// 17. How to measure noise??
// 18. Add memory preset adjustment support.
// 19. When output is off, set V out to zero so out show zero volts. Higher drive voltage?
// 20. Use 1 precent resitors for Vin divider.
// 21. remove old switch code. Done
// 22. Limit top output so its gracefull. Done
// 23. Build diff amp to measure switcher ripple
// 24. Add boot up diag, i2c checks, etc

// indent -gnu -br -cli2 -lp -nut -l100 main.c
// sudo gtkterm --port /dev/ttyACM0

#define GPIO_PORT_OUT_ENB	GPIOB
#define GPIO_OUT_ENB		GPIO12

#define GPIO_PORT_CURRENT_LIMIT_IN   GPIOB
#define GPIO_PIN_CURRENT_LIMIT_IN    GPIO13  // #define GPIO13 (1 << 13)

parameters_t param = {
  0.0, // v_out;
  0.0, // i_out;
  0.0, // v_set;
  0.0, // i_set;
  0.0, // p_out;
  0.0, // v_in;
  0.0, // heatsink_temp;
  0.0, // cpu_temp;  
  0.0, // mem1_v_set;
  0.0, // mem2_v_set;

  0, // mode;
  0, // out_on;
  0, // lock;  
  0, // ticks;  
};

I2C_Control i2c_handle;         // I2C Control struct
TaskHandle_t sw_debouce_taskHandle = NULL;
TaskHandle_t oled_task_Handle = NULL;
static SemaphoreHandle_t i2c_mutex;

static volatile bool show_rx = true;    // false;
float mcp9808_float_temp = 0.0;
char mcp9808_str[20] = "x.x";
char Vin_str[30] = "0.0"; 
char Vtop_str[30] = "12.0";
#define REGULATOR_DROP (3.0)
float top_voltage_limit = 15.00;  // This should be calculated from input voltage

int v_dac_set = 0;
int i_dac_set = 0;

#define MEM1_VOLTAGE_PRESET 3.3 // was 3.3
#define MEM1_CURRENT_PRESET 1.2

#define MEM2_VOLTAGE_PRESET 5.0
#define MEM2_CURRENT_PRESET 1.5

float mem1_voltage_preset = MEM1_VOLTAGE_PRESET;        // normally 3.3 volts
float mem1_current_preset = MEM1_CURRENT_PRESET;        // normally 3.3 volts

float mem2_voltage_preset = MEM2_VOLTAGE_PRESET;
float mem2_current_preset = MEM2_CURRENT_PRESET;

float last_voltage_preset = 0.0;
volatile TickType_t mem1_press_time = 0;        // for xTaskGetTickCount ();
volatile TickType_t mem2_press_time = 0;

uint32_t output_state = 0;
uint32_t mem1_state = 0;
uint32_t mem2_state = 0;

typedef struct _cursor
{
  int cursor_on;
  int position;
  int multi;
} cursor_t;

cursor_t cursor_voltage = {
  0,                            // cursor cursor_on
  0,                            // cursor position
  1                             // cursor multi
};

cursor_t cursor_current = {
  0,                            // cursor cursor_on
  0,                            // cursor position
  1                             // cursor multi
};

int voltage_cursor_on = 0;
int voltage_position = 0;
int voltage_multi = 1;

int current_cursor_on = 0;
int current_position = 0;
int current_multi = 1;

mov_avg_t volts_mov_avg;
mov_avg_t current_mov_avg;

int show_status_flag = 0;
int log_interval = 10;
int debug_flag = 0;

int program_bytes = 0;
static const char pOFF[] = { "OFF" };
static const char pCV[] = { "CV" };
static const char pCC[] = { "CC" };

const char *mode_str = pOFF;

uint8_t V_RotaryCurrentState = 0x00;
uint8_t V_RotaryTransition = 0;
int V_RotaryPosition = 0;
int V_RotaryPosition_last = 0;
uint8_t I_RotaryCurrentState = 0x00;
uint8_t I_RotaryTransition = 0;
int I_RotaryPosition = 0;
int I_RotaryPosition_last = 0;

int flash_stack_size = 0;
int console_stack_size = 0;
int bounce_stack_size = 0;
int oled_stack_size = 0;

void
millisecs_sleep (uint64_t ms)
{
  volatile TickType_t start = xTaskGetTickCount ();

  while (ms < (xTaskGetTickCount () - start));
}

void
vApplicationTickHook (void)     // 1 millsecond tick
{
//  gpio_toggle (GPIOC, GPIO12);
}

void
wait_til_usb_ready (void)
{
  while (!usb_ready ()) {
    vTaskDelay (pdMS_TO_TICKS (100));
  }
}

/*********************************************************************
 * Lock mutex:
 *********************************************************************/
void
mutex_lock (void)
{
  xSemaphoreTake (i2c_mutex, (TickType_t) 10);
}

/*********************************************************************
 * Unlock mutex:
 *********************************************************************/
void
mutex_unlock (void)
{
  xSemaphoreGive (i2c_mutex);
}

 // https://electronics.stackexchange.com/questions/99915/stm32-rotary-encoder-with-hardware-interrupts
static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// level irq ?? https://electronics.stackexchange.com/questions/158749/how-to-use-level-triggered-interrupts-with-stm32f1xx
// Does not appear to be support for level interrupts in stm32f1
static void
exti_setup (void)
{
  /* Enable GPIOB clock. */
  rcc_periph_clock_enable (RCC_GPIOB);

  /* Enable AFIO clock. */
  rcc_periph_clock_enable (RCC_AFIO);

  /* Enable EXTI0 interrupt. */
  nvic_enable_irq (NVIC_EXTI0_IRQ);

  /* Set GPIO1 (in GPIO port B) to 'input float'. */
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

  /* Configure the EXTI 0 subsystem. */
  exti_select_source (EXTI0, GPIOB);

//  exti_set_trigger (EXTI0, EXTI_TRIGGER_RISING);  // does not trigger
//  exti_set_trigger (EXTI0, EXTI_TRIGGER_FALLING); // shoudl be best choice
  exti_set_trigger (EXTI0, EXTI_TRIGGER_BOTH);  // was used a lot
  exti_enable_request (EXTI0);
}

//  https://www.freertos.org/taskresumefromisr.html
//  https://www.freertos.org/RTOS_Task_Notification_As_Counting_Semaphore.html

// Wait for GPIO B 0 than wake up MCP23017 I2C access
void
exti0_isr (void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Clear the interrupt. */
  exti_reset_request (EXTI0);

  vTaskNotifyGiveFromISR (sw_debouce_taskHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
}

/*********************************************************************
 * Read ADC Channel
 *********************************************************************/
static uint16_t
read_adc (uint8_t channel)
{
// See AN2834 "How to get the best ADC accuracy in STM32 microcontrollers"

  adc_set_sample_time (ADC1, channel, ADC_SMPR_SMP_239DOT5CYC);
  adc_set_regular_sequence (ADC1, 1, &channel);
  adc_start_conversion_direct (ADC1);
  while (!adc_eoc (ADC1)) {
    taskYIELD ();
  }
  return adc_read_regular (ADC1);
}

/*********************************************************************
 * Return temperature in C * 100
 *********************************************************************/
int
degrees_C100 (void)
{
  int vtemp;
  static const int v25 = 1365;  // see https://github.com/ve3wwg/stm32f103c8t6/issues/11

  vtemp = (int) read_adc (ADC_CHANNEL_TEMP) * 3300 / 4095;

  return (v25 - vtemp) * 1000 / 45 + 2500;      // temp = (1.43 - Vtemp) / 4.5 + 25.00
}

static void
rd_env (void)
{
  int temp100, vref;
  int adc0;

  temp100 = degrees_C100 ();
  vref = read_adc (ADC_CHANNEL_VREF) * 330 / 4095;
  adc0 = (read_adc (0) * 330 / 4095) * 11.875;  // VIN_CHK
//  adc1 = read_adc (1) * 330 / 4095;
  std_printf ("Temperature %d.%02d C, Vref %d.%02d Volts, ch0 %d.%02d V\n",
              temp100 / 100, temp100 % 100, vref / 100, vref % 100, adc0 / 100, adc0 % 100);

  std_printf ("Vin: %sV, Vtop: %sV\n", Vin_str, Vtop_str);
}

static void
adc_init (void)
{
  rcc_periph_clock_enable (RCC_GPIOA);  // Enable GPIOA for ADC
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1); // PA0 & PA1

  // Initialize ADC:
  rcc_peripheral_enable_clock (&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
  adc_power_off (ADC1);
  rcc_peripheral_reset (&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
  rcc_peripheral_clear_reset (&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
  rcc_set_adcpre (RCC_CFGR_ADCPRE_PCLK2_DIV6);  // Set. 12MHz, Max. 14MHz
  adc_set_dual_mode (ADC_CR1_DUALMOD_IND);      // Independent mode
  adc_disable_scan_mode (ADC1);
  adc_set_right_aligned (ADC1);
  adc_set_single_conversion_mode (ADC1);
  adc_set_sample_time (ADC1, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
  adc_set_sample_time (ADC1, ADC_CHANNEL_VREF, ADC_SMPR_SMP_239DOT5CYC);
  adc_enable_temperature_sensor ();
  adc_power_on (ADC1);
  adc_reset_calibration (ADC1);
  adc_calibrate_async (ADC1);
  while (adc_is_calibrating (ADC1));
}

/*********************************************************************
 * Background Tasks
 *********************************************************************/
static void
back_ground_tasks (void *arg __attribute__ ((unused)))
{
  uint8_t seconds = 0;
  int step = 0;
  TickType_t xLastWakeTime;
  uint16_t gpio_port_b_in = 0;
  uint16_t mcp23017_port;

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  // Init the hardware
  led_all_on ();
  ina219_reset ();
  mcp4728_init ();
  ina219_init ();
  read_Vin ();

//  mov_avg_filter_init (&volts_mov_avg);
//  mov_avg_filter_init (&current_mov_avg);

  iwdg_reset ();
  vTaskDelay (pdMS_TO_TICKS (2000));    // seems to set how long spash screen is on
  iwdg_reset ();

  // Wait for init task to finish before the OLED runs
  vTaskResume (oled_task_Handle);

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount ();
  for (;;) {
    vTaskDelayUntil (&xLastWakeTime, 100);      // 1 millisecond per tick
    iwdg_reset ();

    // check preset update needed
    mem1_check_preset_timeout ();
    mem2_check_preset_timeout ();

    flash_stack_size = uxTaskGetStackHighWaterMark (NULL);

    // update slower tasks
    if (++step < 10) {
      continue;
    }
    // One second support items
    step = 0;

    led_toggle (DEBUG_LED);     // One second tick

    read_Vin ();                // Update input voltage reading
    mcp9808_read_float ();      // Update heat sink temperture reading

#if 1
    // check for stuck interrupt
    gpio_port_b_in = gpio_port_read (GPIOB);
//    std_printf ("port_b: %d\n", gpio_port_b_in & 1);
    if (!(gpio_port_b_in & 1)) {
      mcp23017_port = mcp23017_read16 (GPIO_REG);       // clear any  pending interrupts
#if 1                           // def TASK_TRACE
      std_printf ("port_b: %d\n", gpio_port_b_in & 1);
      std_printf ("Clear mcp23017 IRQ, 0x%02X\n", mcp23017_port);
#endif

      // Setup interrupt registers
      mcp23017_write16 (GPINTEN_REG, 0x007f);
      mcp23017_write16 (DEFVAL_REG, 0x0000);
      mcp23017_write16 (INTCON_REG, 0x0000);    // interrupt on change
    }
#endif

    // Show output logging every ten seconds
    if ((!((++seconds) % log_interval)) && (show_status_flag == 1)) {
      // Show status on console
      show_status ();
    }
  }
}

void
output_enb (int enb)            // to enable the output
{
  if (enb) {  
    led_on (OUTPUT_LED);
    param.out_on = 1;
    gpio_set (GPIO_PORT_OUT_ENB, GPIO_OUT_ENB); // output is active high
  }
  else {  
    led_off (OUTPUT_LED);
    param.out_on = 0;    
    gpio_clear (GPIO_PORT_OUT_ENB, GPIO_OUT_ENB);
  }
}

int
get_program_size (void)
{
  return program_bytes;
}

static void
splash_scr (void)
{
  char buf[50];

  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);
  UG_FontSelect (&FONT_12X16);

  mini_snprintf (buf, sizeof buf, "  %s", "Lab Power Supply");
  UG_PutString (0, 0, buf);

  UG_FontSelect (&FONT_8X12);
  mini_snprintf (buf, sizeof buf, "  %s", "   0 to 15 V, 1.5A");
  UG_PutString (0, 17, buf);

#if 0
//  mini_snprintf (buf, sizeof buf, "   %s", "By: F.O. Design Works");
  mini_snprintf (buf, sizeof buf, "   %s", "By: Jerry Okeefe");
  UG_PutString (0, 34, buf);
#else
  mini_snprintf (buf, sizeof buf, "Built: %s, %s", __DATE__, __TIME__);
  UG_PutString (0, 31, buf);

  mini_snprintf (buf, sizeof buf, "Program: %d, Heap: %d", program_bytes, xPortGetFreeHeapSize ());
  UG_PutString (0, 45, buf);
#endif

  oled_update_ssd1322 ();
}

void
diag_system (void)
{
  int temp = 0;

  std_printf ("\n\n\nRun Diagnostic Tests");

  // on board I2C items
  std_printf ("\nmcp23017_access_test: ");
  mcp23017_access_test ();
  std_printf ("Passed\n");

  std_printf ("at42qt1070_access_test: ");
  at42qt1070_key_status ();
  std_printf ("Passed\n");

  // remote baord I2C items
  std_printf ("mcp9808_access_test: ");
  temp = mcp9808_read_tens_deg () / 16; // read and convert to Deg C
  std_printf ("Passed %d C\n", temp);

  std_printf ("ina219_access_test: ");
  ina219_access_test ();
  std_printf ("Passed\n");

  std_printf ("mcp4728_access_test: ");
  mcp4728_write (1, 1, 4096);
  std_printf ("Passed\n");
}

float
read_Vin (void)
{

  param.v_in = ((float) read_adc (0)) / 112.5; // Need 1% resistors here!
  gcvt_2 (param.v_in, 1, Vin_str);
  top_voltage_limit = param.v_in - REGULATOR_DROP;
  gcvt_2 (top_voltage_limit, 1, Vtop_str);
  
  // Add error meaages to serial port
  return param.v_in;
}

//float vout_volts=0.0;
// float vout_amps=0.0;

static void
oled_task (void *arg __attribute__ ((unused)))
{
  char buf[60];
  int i;
  char volts_str[40];
  char current_str[40];
  int line_len = 0;
  uint32_t gpio_port_b_in = 0;
  uint8_t flash_cntr = 0;
  float volts = 0.0;
  float amps = 0.0;

#ifdef USE_USB
//  wait_til_usb_ready ();  // Does not appear to be needed for USB
#endif

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  init_cursors ();
  led_all_on ();

//  oled_display_off();
  oled_init_real ();            // clears pix map and init th ugui driver.
  oled_init_ssd1322 ();

  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  diag_system ();
  splash_scr ();

#ifdef TASK_TRACE
  std_printf ("splash_scr()\n");
#endif

  // Suspend ourselves.
  vTaskSuspend (NULL);

  led_all_off ();

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

//  oled_init_all ();

  for (;;) {

    clear_pixmap ();            // Without this we get overlap chars
    iwdg_reset ();

    volts = voltage_read ();
//    vout_volts = volts;
    param.v_out = volts;
    amps = current_read ();
    param.i_out = amps;
//    vout_amps = amps;
    
    gcvt_2 (volts, 3, volts_str);
    gcvt_2 (amps, 3, current_str);
    mini_snprintf (buf, sizeof buf, "Out: %sV, %sA", volts_str, current_str);
    UG_FontSelect (&FONT_12X16);
    UG_PutString (0, 0, buf);

    // Draw analog line matching percentage of load current limit.
    for (i = 0; i < 8; i++) {   // draw four lines
      // Full scale set to current limit
      line_len = (int) ((amps * 256.0) / param.i_set);
      UG_DrawLine (0, BAR_ROW + i, line_len, BAR_ROW + i, C_BLACK);
    }

//    UG_SetBackcolor (C_BLACK);
//    UG_SetForecolor (C_WHITE);

//    UG_FontSelect (&FONT_8X12); // Change to lower font
//    UG_PutString (0, 17, "%A");
    UG_SetBackcolor (C_WHITE);
    UG_SetForecolor (C_BLACK);
    UG_FontSelect (&FONT_8X12); // Change to lower font

    volts_str[0] = ' ';
    // Do we need "last_voltage_preset" for current
    gcvt_2 (last_voltage_preset, 3, volts_str + 1);
    if (voltage_cursor_on && (flash_cntr & 1)) {        // add cursor
      volts_str[strlen (volts_str) - 1 - voltage_position] = '_';
    }

    current_str[0] = ' ';
    gcvt_2 (param.i_set, 3, current_str + 1);
    if (current_cursor_on && (flash_cntr & 1)) {        // add cursor
      current_str[strlen (current_str) - 1 - current_position] = '_';
    }

    mini_snprintf (buf, sizeof buf, "Sets: %sV, %sA", volts_str, current_str);
    UG_PutString (0, 26, buf);

    // Update output and OC state
    if (!output_state) {
      mode_str = pOFF;
    }
    else {
      // check if in CC mode
      // check current limit state, check PB13
      gpio_port_b_in = gpio_port_read (GPIO_PORT_CURRENT_LIMIT_IN);
      if ((gpio_port_b_in & GPIO_PIN_CURRENT_LIMIT_IN) != 0) {
        mode_str = pCV;
        led_off (OC_LED);
      }
      else {
        mode_str = pCC;
        led_on (OC_LED);
      }
    }

    mini_snprintf (buf, sizeof buf, "Mode: %s, Temp: %sC", mode_str, mcp9808_str);
    UG_PutString (0, 37, buf);

    if (top_voltage_limit < 8.0) {
       strcpy(Vtop_str, "input voltage to low!!");
       mini_snprintf (buf, sizeof buf, "Vin:%s, ERR: V INPUT LOW!!", Vin_str);
    } else {
       mini_snprintf (buf, sizeof buf, "Vin:%sV, Vtop:%sV, %lu",
                   Vin_str, Vtop_str, xTaskGetTickCount () / 1000);
    }
    UG_PutString (0, 49, buf);

    oled_update_ssd1322 ();

    iwdg_reset ();              // refresh the WD

    vTaskDelay (pdMS_TO_TICKS (50));
    flash_cntr++;
    oled_stack_size = uxTaskGetStackHighWaterMark (NULL);
  }
}

#define DAC_LSB 16              // 16 millivolt per LSB
/*****************
  Calibrate output
 *****************/
void
calibrate (void)
{
  int current_voltage = 0;
  int set_voltage_int = 0;
  int diff = 0;
  int error = 0;

  // Do everything in ints for speed

  //  Output needs to be enabled
  if ((gpio_port_read (GPIO_PORT_OUT_ENB) & GPIO_OUT_ENB) != 0) {
    current_voltage = voltage_read_int ();      // in millivolts
    set_voltage_int = (uint32_t) (param.v_set * 1000.0);        // convert to milli volts
    diff = current_voltage - set_voltage_int;

    error = (diff / DAC_LSB);
    if (error) {
      v_dac_set -= error;
      mcp4728_write (0x00, 1, v_dac_set);       // 16 millivolt per LSB
    }
    std_printf ("%s(%d) diff: %d %d %d %d\n", __func__, __LINE__, diff, current_voltage,
                set_voltage_int, error);
  }
  else {
    std_printf ("%s(%d) Output not enabled!!\n", __func__, __LINE__);
  }
}

/*****************
  I2C drivers
 *****************/
int
mcp4728_access_test (void)
{
  mcp4728_init ();
  return 0;
}

void
mcp4728_init (void) 
{
  mcp4728_write (0x00, 1, 2048 / 6);    // for 0.517 volts, value of 341
  mcp4728_write (0x01, 1, 546); // for 0.829 volts for 0.400 ma value of 546
  mcp4728_write (0x02, 0, 0x0000);      // set to zero
  mcp4728_write (0x03, 0, 2900);        // Voltage output zero offset

  voltage_wrt (0.0);
  current_wrt (1.500);
}

void
mcp4728_write (uint8_t dac, uint8_t gain, uint16_t dac_val)
{
  uint8_t addr = MCP4728_ADDR (0);      // I2C Address
  uint8_t ref_sel = 1;
  uint8_t buf[5];

  buf[0] = MULTI_WRT_CMD | ((dac & 0x03) << 1); // Table 5-1, pg 34 and FIGURE 5-10:
  buf[1] = ((dac_val >> 8) & 0x0F) | ((ref_sel & 1) << 7) | ((gain & 1) << 4);
  buf[2] = dac_val & 0xFF;

  mutex_lock ();
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, buf[0]);
  i2c_write (&i2c_handle, buf[1]);
  i2c_write (&i2c_handle, buf[2]);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
}

int
mcp4728_rd_bsy (void)
{
  uint8_t addr = MCP4728_ADDR (0);      // I2C Address
  uint8_t buf[5];
  uint8_t bsy = 0;

  mutex_lock ();
  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 3);
  i2c_stop (&i2c_handle);
  mutex_unlock ();

  bsy = 1;
  if (buf[0] & 0x80) {
    bsy = 0;
  }
//  std_printf ("%s(%d) buf[0]: 0x%02X 0x%02X\n", __func__, __LINE__, buf[0], bsy);
  return bsy;
}

void
ina219_init (void)
{
  uint16_t config = 0;

  // load config reg See: 8.6.2.1 Configuration Register (
  config = 0x07;                // Set Mode bits, continuous shunt and bus
  config |= 0x0F << 3;          // Set SADC bits, max filtering 68 millsec
  config |= 0x0F << 7;          // Set BADC bits, max filtering 68 millsec
//  config |= 0x01 << 11;         // Set PG bits. // �80 mV, Saturates at 0.79 Amps.
  config |= 0x10 << 11;         // Set PG bits.   // �160 mV, Works to 1.5 Amps
//  config |= 0x11 << 11;         // Set PG bits. // �320 mV
  config |= 0x01 << 13;         // Set BRNF bit, 32 volt FSR
  ina219_reg_write (0x00, config);

// Config reg: 0x199F
//  std_printf ("%s(%d) Config reg: 0x%04X\n", __func__, __LINE__, config);

  // load calibration reg
  ina219_reg_write (0x05, 4096);
}

void
ina219_reset (void)
{
  ina219_reg_write (0, 0xB99F);
}

int
ina219_access_test (void)
{
  ina219_reg_read (2, 0);
  return 0;
}

int
ina219_reg_read (uint8_t reg, uint8_t local_debug)
{
  uint8_t addr = INA219_ADDR (0);       // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;

  // Set register pointer
  mutex_lock ();
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg);
  i2c_stop (&i2c_handle);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 2);
  i2c_stop (&i2c_handle);
  mutex_unlock ();

  val16 = (buf[0] << 8) | buf[1];
  if (local_debug) {
    std_printf ("%s(%d) %d, 0x%02X 0x%02X\n", __func__, __LINE__, val16, buf[0], buf[1]);
  }

  return (val16);
}

void
ina219_reg_write (uint8_t reg, uint16_t val16)
{
  uint8_t addr = INA219_ADDR (0);       // I2C Address

  // Set register pointer
  mutex_lock ();
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg);
  i2c_write (&i2c_handle, val16 >> 8);
  i2c_write (&i2c_handle, val16 & 0xFF);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
}

void
init_cursors (void)
{
  cursor_voltage.cursor_on = 0;
  cursor_voltage.position = 0;
  cursor_voltage.multi = 0;

  cursor_current.cursor_on = 0;
  cursor_current.position = 0;
  cursor_current.multi = 0;

  voltage_cursor_on = 0;
  voltage_position = 2;
  voltage_multi = 100;

  current_cursor_on = 0;
  current_position = 2;
  current_multi = 100;
}

void
voltage_button (int button_state)
{
  // Move to next up scale
  voltage_cursor_on = 1;
  voltage_position++;
  voltage_multi *= 10;

  if (voltage_position > 2) {
    voltage_cursor_on = 1;
    voltage_position = 0;
    voltage_multi = 1;
  }
  if (debug_flag) {
     std_printf ("%s Button state %d, %d\n", __func__, button_state, voltage_multi);
  }
}

void
current_button (int button_state)
{
  // Move to next up scale
  current_cursor_on = 1;
  current_position++;
  current_multi *= 10;

  if (current_position > 2) {
    current_cursor_on = 1;
    current_position = 0;
    current_multi = 1;
  }

  if (debug_flag) {
     std_printf ("%s Button state %d, %d\n", __func__, button_state, current_multi);
  }
}

#define FALLING_EDGE 0
#define RISING_EDGE  1
void
output_button_CB (int edge_dir)
{
  std_printf ("Output Button edge %d\n", edge_dir);
  if (edge_dir == FALLING_EDGE) {
    // toggle output state
    output_state = !output_state;
  }

  if (output_state & 1) {
    voltage_wrt (param.v_set);
    output_enb (1);
    led_on (OUTPUT_LED);
  }
  else {
//    voltage_zero_wrt (); Is this needed ??
    output_enb (0);
    led_off (OUTPUT_LED);
  }
}

void
mem1_button_CB (void)
{
  param.v_set = mem1_voltage_preset;
  last_voltage_preset = mem1_voltage_preset;
  param.i_set = mem1_current_preset;
  // don't need to check output state because output control block output
  voltage_wrt (param.v_set);
  current_wrt (param.i_set);

  led_on (MEM1_LED);
  led_off (MEM2_LED);
  init_cursors ();

  if (debug_flag) {
     std_printf ("Mem1 Button press\n");
  }
  return;
}

void
remote_voltage_preset(float preset_voltage)
{   
  led_off (MEM1_LED);
  led_off (MEM2_LED);
  param.v_set = preset_voltage;
  last_voltage_preset = param.v_set;
  voltage_wrt (param.v_set);
}

void
remote_current_preset(float preset_current)
{   
  led_off (MEM1_LED);
  led_off (MEM2_LED);
  param.i_set = preset_current;
  current_wrt (param.i_set);
}

void
mem2_button_CB (void)
{
  param.v_set = mem2_voltage_preset;
  last_voltage_preset = mem2_voltage_preset;
  param.i_set = mem2_current_preset;
  // don't need to check output state because output control block output
  voltage_wrt (param.v_set);
  current_wrt (param.i_set);

  led_on (MEM2_LED);
  led_off (MEM1_LED);
  init_cursors ();

  if (debug_flag) {
      std_printf ("Mem2 Button press\n");
  }
  return;
}

void
voltage_wrt (float voltage)
{
  param.v_set = voltage;
//  v_dac_set = (int) (voltage / 0.01525);        // for 5.000V
  v_dac_set = (int) (voltage / 0.0060); // for 5.000V
  mcp4728_write (0x00, 1, v_dac_set);   // 1 millvolt per bit ??
//  calibrate (); // seems to cause out V to duble
}

void
voltage_zero_wrt (void)
{
  float voltage = 0.0;
  v_dac_set = (int) (voltage / 0.01525);
  mcp4728_write (0x00, 1, v_dac_set);   // 1 millvolt per bit ??
}

void
current_wrt (float current)
{
//  char i_str[20];
//  i_dac_set = (int) (current / 0.00050); // for 0.266 at DAC output
  i_dac_set = (int) (current * 2000);   // 2 millvolt dac limit equal 1 milliamp measurement

#if 0
  gcvt_2 (current, 3, i_str);
  std_printf ("i:%s, i_dac_set: %d\n", i_str, i_dac_set);
#endif

  mcp4728_write (0x01, 1, i_dac_set);
  param.i_set = current;
}

void
mem1_check_preset_timeout (void)
{
  volatile TickType_t current_time = 0;

  if (mem1_press_time == 0) {
    return;
  }
  current_time = xTaskGetTickCount ();
  if ((current_time - mem1_press_time) > 1000) {
    mem1_voltage_preset = param.v_set;
    last_voltage_preset = mem1_voltage_preset;
    mem1_press_time = 0;
    std_printf ("MEM1 preset Update\n");
  }
}

void
mem2_check_preset_timeout (void)
{
  volatile TickType_t current_time = 0;

  if (mem2_press_time == 0) {
    return;
  }
  current_time = xTaskGetTickCount ();
  if ((current_time - mem2_press_time) > 1000) {
    mem2_voltage_preset = param.v_set;
    last_voltage_preset = mem2_voltage_preset;
    mem2_press_time = 0;
    std_printf ("MEM2 preset Update\n");
  }
}

static void
sw_debouce_task (void *arg __attribute__ ((unused)))
{
  uint16_t mcp23017_inputs = 0;
  uint16_t mcp23017_inputs_last = 0;
  uint8_t key_status = 0;
  uint8_t key_value = 0;
  int indx = 0;
  const TickType_t xBlockTime = 2000;
  uint32_t __attribute__ ((unused)) ulNotifiedValue;
  int state = 0;
  float millivolts = 0.0;
  float milliamps = 0.0;
  uint16_t mcp23017_io_dir = 0;

  static volatile TickType_t press_time = 0;
  static volatile TickType_t release_time = 0;
  static volatile TickType_t press_time_diff = 0;
  static volatile TickType_t press_long = 0;
  
 // Setup IRQ on PORT B pin 0, GPIO0
 exti_setup ();

// Clear "Change" interrupt line
  // AT42QT1070 Change line
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);

  at42qt1070_init ();           // 100millisec delay
//  at42qt1070_det_status (); // clear any stale status 
//  at42qt1070_key_status (); // clear any stale status

  // Setup MCP23017 Outputs
  mcp23017_reset ();
  mcp23017_io_dir = 0xffff;     // default io_dir as inputs
  mcp23017_init (mcp23017_io_dir);

  // Setup interrupt registers
  mcp23017_write16 (GPINTEN_REG, 0x007f);
  mcp23017_write16 (DEFVAL_REG, 0x0000);
  mcp23017_write16 (INTCON_REG, 0x0000);        // interrupt on rising or falling logic

//  mcp23017_write16 (DEFVAL_REG, 0x0000);
//  mcp23017_write16 (INTCON_REG, 0xffff);  // interrupt on rising or falling logic

  // Failed without line below.
//  mcp23017_inputs_last = mcp23017_read16 (GPIO_REG);    // clear any  pending interrupts

  bounce_stack_size = uxTaskGetStackHighWaterMark (NULL);

 // Setup IRQ on PORT B pin 0, GPIO0
 // exti_setup ();

  for (;;) {
    iwdg_reset ();

//    ulNotifiedValue = ulTaskNotifyTake (pdFALSE, xBlockTime); // normal configUSE_PREEMPTION 1
    ulNotifiedValue = ulTaskNotifyTake (pdTRUE, xBlockTime); // normal configUSE_PREEMPTION 0
    if (ulNotifiedValue == 0) {
      /* Did not receive a notification within the expected time. */
      //     vCheckForErrorConditions();
//      std_printf ("Event notification timeout\n");
      mcp23017_inputs = mcp23017_read16 (GPIO_REG);
    }
    else {
      // Check for interrupt
      if ((gpio_port_read (GPIOB) & 1) == 0) {
        mcp23017_inputs = mcp23017_read16 (GPIO_REG);

        if (debug_flag) {
          std_printf ("%d, inputs: 0x%04X, 0x%04X, 0x%04X\n", indx++, mcp23017_inputs,
                      mcp23017_inputs_last, mcp23017_inputs ^ mcp23017_inputs_last);
        }

        // Touch key support
        if (!(mcp23017_inputs & (1 << 6))) {
          // Need to read both status bytes to clear change int line
          at42qt1070_det_status ();
          key_status = at42qt1070_key_status ();
          if (key_status == 0) { // 0 if key press is released
            release_time = xTaskGetTickCount ();
            press_time_diff = release_time - press_time;
            press_long = 0;
            if (press_time_diff > 1000) {       // one second hold time
              press_long = 1;
            }
            switch (key_value) {
              case 1:
                if (press_long) {
                  //     mem1_voltage_preset = param.v_set;
                }
                //      last_voltage_preset =  mem1_voltage_preset;
                mem1_button_CB ();
                break;
              case 2:
                if (press_long) {
                  //        mem2_voltage_preset = param.v_set;
                }
                //      last_voltage_preset =  mem2_voltage_preset;
                mem2_button_CB ();
                break;
              case 4:
                output_button_CB (0);
                break;
            }
            std_printf ("Key release: 0x%02X, %lu, d_time: %lu, type: %lu\n", key_status,
                        release_time, press_time_diff, press_long);
          }
          else {
            key_value = key_status;
            switch (key_value) {
              case 1:
                mem1_press_time = xTaskGetTickCount ();
                break;
              case 2:
                mem2_press_time = xTaskGetTickCount ();
                break;
            }
            press_time = xTaskGetTickCount ();
            std_printf ("Key press:   0x%02X, %lu\n", key_value, xTaskGetTickCount ());
          }
        }

        V_RotaryCurrentState = mcp23017_inputs & 3;
        V_RotaryTransition = (V_RotaryTransition << 2) | V_RotaryCurrentState;

        V_RotaryPosition = V_RotaryPosition + enc_states[V_RotaryTransition & 0x0F];
        if (V_RotaryPosition != V_RotaryPosition_last) {
//          std_printf ("V encoder: %d, %d\n", mcp23017_inputs & 3, V_RotaryPosition);
          V_RotaryPosition_last = V_RotaryPosition;
          millivolts = (float) ((enc_states[V_RotaryTransition & 0x0F] * voltage_multi) * 0.001);
          param.v_set -= millivolts;    // why neg

          // Check for legal voltage range
          if (param.v_set > top_voltage_limit) {
            param.v_set = top_voltage_limit;
          }
          if (param.v_set < 0.0) {
            param.v_set = 0.0;
          }
          last_voltage_preset = param.v_set;
          led_off (MEM1_LED);
          led_off (MEM2_LED);
          voltage_wrt (param.v_set);
//        calibrate (); // not sure if this will work??
//          std_printf ("dac set: %d\n", v_dac_set);
        }

        I_RotaryCurrentState = (mcp23017_inputs >> 3) & 3;
        I_RotaryTransition = (I_RotaryTransition << 2) | I_RotaryCurrentState;
        I_RotaryPosition = I_RotaryPosition + enc_states[I_RotaryTransition & 0x0F];
        if (I_RotaryPosition != I_RotaryPosition_last) {
//          std_printf ("I encoder: %d, %d\n", (mcp23017_inputs >> 3) & 3, I_RotaryPosition);
          I_RotaryPosition_last = I_RotaryPosition;
          milliamps = (float) ((enc_states[I_RotaryTransition & 0x0F] * current_multi) * 0.001);
          param.i_set -= milliamps;
          if (param.i_set > 1.5) {
            param.i_set = 1.5;
          }
          if (param.i_set < 0.0) {
            param.i_set = 0.0;
          }
          led_off (MEM1_LED);
          led_off (MEM2_LED);
          current_wrt (param.i_set);
        }

        // Check all Buttons
        if (!(state = (mcp23017_inputs >> 2) & 1)) {
          voltage_button (state);
        }

        if (!(state = (mcp23017_inputs >> 5) & 1)) {
          current_button (state);
        }

        // Save present state for next button press
        mcp23017_inputs_last = mcp23017_inputs;
      }
    }
    bounce_stack_size = uxTaskGetStackHighWaterMark (NULL);
  }
}

#if 0
void
mov_avg_filter_init (mov_avg_t * p)
{
  memset (p->mov_avg_data, 0, FILTER_SIZE);
  p->array_size = FILTER_SIZE;
  p->indx = 0;
  p->init_fill = 0;
}

// void put_fifo2(mov_avg_t *p, uint32_t value);

void
put_fifo2 (mov_avg_t * p, uint32_t value)
{
  int i = 0;
  if (p->init_fill == 0) {
    for (i = 0; i < p->array_size; ++i) {
      p->mov_avg_data[i] = value;
    }
    p->init_fill = 1;
  }
  p->mov_avg_data[p->indx++] = value;
  if (p->indx >= p->array_size) {
    p->indx = 0;
  }
}

// uint32_t get_avg2(mov_avg_t *p);

uint32_t
get_avg2 (mov_avg_t * p)
{
  int i = 0;
  uint32_t avg = 0;

  for (i = 0; i < p->array_size; i++) {
    avg += p->mov_avg_data[i];
  }
  return avg;
}

uint32_t mov_avg_data[FILTER_SIZE];
static int indx;

void
put_fifo (uint32_t value)
{
//   static int indx=0;

  mov_avg_data[indx++] = value;
  if (indx >= FILTER_SIZE) {
    indx = 0;
  }
}

uint32_t
get_avg (void)
{
  int i = 0;
  uint32_t avg = 0;

  for (i = 0; i < FILTER_SIZE; i++) {
    avg += mov_avg_data[i];
  }
  return avg;
}
#endif

uint16_t
voltage_read_int (void)
{
  uint16_t val16 = 0;

  val16 = ina219_reg_read (2, 0);       // read bus voltage
  val16 >>= 3;                  // shift out status bits.
  return val16 * 4;             // convert to millivolts
}

float
voltage_read (void)
{
  uint32_t val32 = 0;
  float v = 0.0;

  val32 = voltage_read_int ();
  v = (float) val32;
  v = v / 1000.0;      // Convert to volts
  return v;
}

uint32_t
current_read_int (void)
{
  uint16_t val16 = 0;

  val16 = ina219_reg_read (1, debug_flag);  // read shut voltage, 0.0315 amps for 100 ohms
  if (val16 && 0xC000) {        // Check for nagative values
  }
  else {
    val16 = 0;     // -= 392;  // remove constant current offset.
  }
  if (debug_flag) {
    std_printf ("%s(%d) %d, 0x%02X\n\n", __func__, __LINE__, val16, val16);
  }
  return val16;
}

float
current_read (void)
{
  uint32_t val32 = 0;
  float amps = 0.0;

  val32 = current_read_int ();
  amps = (float) val32 *0.000010;  // 10 uAmps LSB
  amps /= 0.1;                  // divided by 0.1 ohm sense resistor.
  return amps;
}

int
on_off_contrl (int on)
{
  if (on) {
    output_enb (1);
    led_on (OUTPUT_LED);
  } else {
    output_enb (0);
    led_off (OUTPUT_LED);
  }
  return 0;
}

/*********************************************************************
 * Console:
 *********************************************************************/
static void
console_task (void *arg __attribute__ ((unused)))
{

#ifdef USE_USB
//  wait_til_usb_ready ();  // Does not appear to be needed for USB
#endif

#ifdef TASK_TRACE
  std_printf ("STM32F103 Console Ready:\n");
#endif

  cmd_line_process ();
}

void
show_status (void)
{
  char str[30];
  float volts = 0.0;
  float amps = 0.0;

  volts = voltage_read ();
  amps = current_read ();

  memset (str, 0, sizeof (str));
  gcvt_2 (volts, 3, str);
  std_printf ("\nOutput: %s V", str);

  memset (str, 0, sizeof (str));
  gcvt_2 (param.v_set, 3, str);
  std_printf (" Set: %s V", str);

  memset (str, 0, sizeof (str));
  gcvt_2 (amps, 3, str);
  std_printf (", %s A", str);

  std_printf (", mode: %s", mode_str);
  std_printf (", temp: %sC\n", mcp9808_str);

  // Read stm32f103 temp, ref v, ch0, and ch1. PA0 and PA1
  rd_env ();
}

void
reboot (void)
{
  scb_reset_system ();          /* reset the stm32 */
}

/*********************************************************************
 * Main program: Device initialization etc.
 *********************************************************************/
int
main (void)
{
  rcc_clock_setup_in_hse_8mhz_out_72mhz ();

  rcc_periph_clock_enable (RCC_GPIOA);
  rcc_periph_clock_enable (RCC_GPIOB);
  rcc_periph_clock_enable (RCC_GPIOC);
  rcc_periph_clock_enable (RCC_AFIO);

//   RCC_PLL,
//   RCC_HSE, low speed external, crystal
//   RCC_HSI, high speed external, R/C
//   RCC_LSE, low speed external, crystal
//   RCC_LSI  low speed internal, R/C

  // Setup watchdog timer
  rcc_osc_on (RCC_LSI);
  rcc_wait_for_osc_ready (RCC_LSI);

  /* watchdog setup */
#ifdef ENABLE_WDT
  iwdg_start ();
  iwdg_set_period_ms (7000);    // trigger below 3000
#endif

  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);  // IRQ mcp23017 Int
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);  // AT42QT1070 Change line

  // OLED and mcp23017 reset line setup
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11 | GPIO5);

  // Hold OLED reset in reset
  oled_reset (1);               // GPIOB GPIO11

  // https://github.com/BuFran/hal/blob/master/include/hal/delay.h
  //  systick_get_value();

  // OUTPUT enable
  gpio_set_mode (GPIO_PORT_OUT_ENB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                 GPIO_OUT_ENB);

  // LED port setup
  led_all_setup ();

#ifndef USE_USB
  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
  std_set_device (mcu_uart2);   // Use UART2 for std I/O
//  open_uart (2, 115200, "8N1", "rw", 0, 0);
  open_uart (2, 9600, "8N1", "rw", 0, 0);
#endif

  /* I2C setup */
  rcc_periph_clock_enable (RCC_I2C1);   // I2C
  rcc_periph_clock_enable (RCC_AFIO);
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
  gpio_set (GPIOB, GPIO6 | GPIO7);      // Idle high

  // AFIO_MAPR_I2C1_REMAP=0, PB6+PB7
  gpio_primary_remap (0, 0);

  i2c_mutex = xSemaphoreCreateMutex (); // Add check

  // Configure I2C1
  i2c_configure (&i2c_handle, I2C1, 1000);

  // Setup ADC
  adc_init ();

#ifdef USE_USB
  // void usb_start(bool gpio_init,unsigned priority);

//  usb_start (1, 1);
  usb_start (1, configMAX_PRIORITIES - 1);
  std_set_device (mcu_usb);     // Use USB for std I/O
#endif

  // Disable output
  output_enb (0);               // output off

  /* setup the OLED SPI PORT */
  spi_1_init ();
//  spi_2_init ();

 // Setup IRQ on PORT B pin 0, GPIO0
 // exti_setup ();

#define STACK_SIZE 600          // was 800

  // Higher the priority number than the task has higher priority
  // Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY).

  xTaskCreate (back_ground_tasks, "back_ground", STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

  xTaskCreate (console_task, "console", STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

  xTaskCreate (oled_task, "oled", STACK_SIZE, NULL, configMAX_PRIORITIES - 3, &oled_task_Handle);

  // This is the interruot drivern switch denbounce task. Need to the highest priority task!
  xTaskCreate (sw_debouce_task, "sw_debouce", STACK_SIZE, NULL, configMAX_PRIORITIES - 1,
               &sw_debouce_taskHandle);

  program_bytes = flash_bytes_used ();

  vTaskStartScheduler ();

  for (;;);
}

// End main.c
