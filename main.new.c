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

//#define USE_USB 1
#define USE_CAP_TOUCH 1  // AT42QT1 #ifdef USE_CAP_TOUCH
// #define TASK_TRACE 1
#define ENABLE_WDT 1


// TO DO
// 1. Add watch dog timer support. Done
// 2. flash mem buttons
// 3. check stack space. Done
// 4. check program size. Done.
// 5. use struture instend of globals
// 6. over-sample and filter ADC results
// 7. optimize OLED, run faster, update only delta lines
// 8. use the voltmeter to auto adjust the PS output setting
// 9. Test zero setting. Looks ok but some time jumps to 1.7 volts. Done
// 10. Add encoder range support. Done
// 11. Add start up splash screen. Done
// 12. make presets adjustable, 5V and 3.3 V
// 13. boot up diag. Done
// 14. Fix OLED start up problem
// 15. Add remote calibrate function
// 16. Add program and heap size dump
// 17. How to measure noise??
// 18. Add memory preset adjustment support.
// 19. When output is off, set V out to zero so out show zer volts.
// 20. Use 1 precent resitors for Vin divider.

// indent -gnu -br -cli2 -lp -nut -l100 main.c
// sudo gtkterm --port /dev/ttyACM0

void sleep (int ms)
{
  int i, k;
  for (k = 0; k < ms; k++) {
    for (i = 0; i < 1000; i++);
  }
}

void
wait_til_usb_ready (void)
{
  while (!usb_ready ()) {
    vTaskDelay (pdMS_TO_TICKS (100));
  }
}

#define FLASH_BASE_ADDRESS ((uint32_t)0x08000000)
#define FLASH_OPERATION_ADDRESS ((uint32_t)0x0800f000)

int
flash_bytes_used (void)
{
  uint32_t *ptr = (uint32_t *) FLASH_OPERATION_ADDRESS;
  // FLASH_BASE_ADDRESS

  while (*ptr == 0xFFFFFFFF) {
    ptr -= 4;
  }

  return (ptr - ((uint32_t *) FLASH_BASE_ADDRESS));
}

#define FLASH_MS		400     // Signal flash time in ms

#define GPIO_PORT_OUT_ENB	GPIOB
#define GPIO_OUT_ENB		GPIO12

#define GPIO_PORT_CURRENT_LIMIT_IN   GPIOB
#define GPIO_PIN_CURRENT_LIMIT_IN    GPIO13     // #define GPIO13 (1 << 13)

typedef struct _parameters
{
  float mcp9808_float_temp;
  uint8_t blkId;
  char mcp9808_str[50];
} parameters_t;

parameters_t param = {
  0.0,                          // mcp9808_float_temp
  1,
  "x.x"                         // mcp9808_str[50]
};

static volatile bool show_rx = true;    // false;
I2C_Control i2c;                // I2C Control struct
TaskHandle_t sw_debouce_taskHandle = NULL;
float mcp9808_float_temp = 0.0;
char mcp9808_str[20] = "x.x";
char Vin_str[20] = "x.x V";
float measured_voltage = 0.0;
float measured_current = 0.0;
int v_dac_set = 0;
int i_dac_set = 0;
float voltage_set = 0.000;
float current_set = 1.500;
uint32_t output_state = 0;
uint32_t mem1_state = 0;
uint32_t mem2_state = 0;

int voltage_cursor_on = 0;
int voltage_position = 0;
int voltage_multi = 1;

int current_cursor_on = 0;
int current_position = -1;
int current_multi = 1;

int ps_ready = 0;

int show_status_flag = 0;

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
//  exti_set_trigger (EXTI0, EXTI_TRIGGER_RISING);
  exti_set_trigger (EXTI0, EXTI_TRIGGER_FALLING);
//  exti_set_trigger (EXTI0, EXTI_TRIGGER_BOTH);  // was used a lot
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
static int
degrees_C100 (void)
{
  int vtemp;
  static const int v25 = 1365;  // see https://github.com/ve3wwg/stm32f103c8t6/issues/11

  vtemp = (int) read_adc (ADC_CHANNEL_TEMP) * 3300 / 4095;

  return (v25 - vtemp) * 1000 / 45 + 2500;      // temp = (1.43 - Vtemp) / 4.5 + 25.00
}

static void
rd_temp (void)
{
  int temp100, vref;
  int adc0, adc1;

  temp100 = degrees_C100 ();
  vref = read_adc (ADC_CHANNEL_VREF) * 330 / 4095;
  adc0 = (read_adc (0) * 330 / 4095) * 11.875;  // VIN_CHK
  adc1 = read_adc (1) * 330 / 4095;

  std_printf ("Temperature %d.%02d C, Vref %d.%02d Volts, ch0 %d.%02d V, ch1 %d.%02d V\n",
              temp100 / 100, temp100 % 100,
              vref / 100, vref % 100, adc0 / 100, adc0 % 100, adc1 / 100, adc1 % 100);
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
 * Signal Flash task
 *********************************************************************/
static void
flash_task (void *arg __attribute__ ((unused)))
{
  int gpio_port_b_in = 0;
  int i = 0;
  uint8_t cntr = 0;

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  // Init the hardware
  ina219_reset ();
  mcp4728_init ();
  ina219_init ();

#ifdef USE_CAP_TOUCH
  at42qt1070_init ();
  at42qt1070_show_status ();
#endif

  led_all_on ();
  ps_ready = 0;                 // tell everyone the PS is ready

  for (i = 0; i < 3; i++) {
    vTaskDelay (pdMS_TO_TICKS (1000));
    iwdg_reset ();
  }
  led_all_off ();

  ps_ready = 1;                 // tell everyone the PS is ready

  for (;;) {
    iwdg_reset ();

    led_toggle (DEBUG_LED);

    // to do
    // 1. Read Vin
    read_Vin ();                // Update input voltage reading

    // 2. read temperatrure.    
    mcp9808_read ();            // Update heat sink temperture reading

    // 3. check 3.3 v
    // 4. check ref Voltages   
    // check for problems

    // check for stuck interrupt
    gpio_port_b_in = gpio_port_read (GPIOB);
//    std_printf ("port_b: %d\n", gpio_port_b_in & 1);
    if (!(gpio_port_b_in & 1)) {
      mcp23017_read16 (GPIO_REG);       // clear any  pending interrupts
#ifdef TASK_TRACE
      std_printf ("port_b: %d\n", gpio_port_b_in & 1);
      std_printf ("Clear mcp23017 IRQ, 0x%02X\n", mcp23017_port);
#endif

      // Setup interrupt registers
      mcp23017_write16 (GPINTEN_REG, 0xffff);
      mcp23017_write16 (DEFVAL_REG, 0x0000);
      mcp23017_write16 (INTCON_REG, 0x0000);    // interrupt on change
    }

    if ((!(++cntr & 7)) && (show_status_flag == 1)) {
      show_status ();
    }

    vTaskDelay (pdMS_TO_TICKS (500));
  }
}

void
output_enb (int enb)            // to enable the output
{
  if (enb) {
    // Enable regulator output
    gpio_set (GPIO_PORT_OUT_ENB, GPIO_OUT_ENB); // output is active high
  }
  else {
    // Disable regulator output
    gpio_clear (GPIO_PORT_OUT_ENB, GPIO_OUT_ENB);
  }
}

static void
splash_scr (void)
{
  char buf[50];

  clear_pixmap ();
  oled_ssd1322_clear ();

  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);
  UG_FontSelect (&FONT_12X16);

  mini_snprintf (buf, sizeof buf, "  %s", "Lab Power Supply");
  UG_PutString (0, 0, buf);

  UG_FontSelect (&FONT_8X12);
  mini_snprintf (buf, sizeof buf, "  %s", "   0 to 15 V, 1.5A");
  UG_PutString (0, 17, buf);

#if 1
  mini_snprintf (buf, sizeof buf, "   %s", "By: F.O. Design Works");
  UG_PutString (0, 34, buf);
  
//  mini_snprintf (buf, sizeof buf, "Built: %s, %s", __DATE__, __TIME__);
//  UG_PutString (0, 45, buf);
#else
  mini_snprintf (buf, sizeof buf, "Built: %s, %s", __DATE__, __TIME__);
  UG_PutString (0, 31, buf);

  mini_snprintf (buf, sizeof buf, "Program: %d, Heap: %d", 
                 program_bytes, xPortGetFreeHeapSize ());
  UG_PutString (0, 45, buf);
#endif

  oled_update_ssd1322 ();
}

void
diag_system (void)
{
  int temp = 0;

  // on baord I2C items
  std_printf ("\nmcp23017_access_test: ");
  mcp23017_access_test ();
  std_printf ("Passed\n");
  
#ifdef USE_CAP_TOUCH
  std_printf ("\nat42qt1070_access_test: ");
  at42qt1070_key_status ();
  std_printf ("Passed\n");
#endif

  // remote baord I2C items
  std_printf ("mcp9808_access_test: ");
  temp = mcp9808_read_int ();
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
  float Vin = 0.0;

//  Vin = (read_adc (0) * 330 / 4095) * 11.875;    
//  Vin = ((float) read_adc (0)) / 105.0;
  Vin = ((float) read_adc (0)) / 112.5; // Need 1 % resistors here!
  gcvt_2 (Vin, 1, Vin_str);
  return Vin;
}

void
oled_init_all (void)
{
#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  clear_pixmap ();
  oled_init_real ();
  oled_init_ssd1322 ();
  oled_ssd1322_clear ();
}

static void
oled_task (void *arg __attribute__ ((unused)))
{
  char buf[100];
  static int cntr = 0;
  int i;
  char volts_str[40];
  char current_str[40];
  int line_len = 0;
  uint32_t gpio_port_b_in = 0;
  uint8_t flash_cntr = 0;

#ifdef USE_USB
//  wait_til_usb_ready ();  // Does not appear to be needed for USB
#endif

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  init_cursors ();

  gpio_set (GPIOB, GPIO11);     // Reset pin

  clear_pixmap ();
  oled_init_real ();
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

  while (ps_ready == 0) {
    vTaskDelay (pdMS_TO_TICKS (100));
    iwdg_reset ();
  }

#if 0
  oled_init_real ();            // test code
  oled_init_ssd1322 ();
#endif

  oled_ssd1322_clear ();

#ifdef TASK_TRACE
  std_printf ("%s(%d)\n", __func__, __LINE__);
#endif

  oled_init_all ();

  for (;;) {

    clear_pixmap ();            // does not appear to be needed

    voltage_read ();
    current_read ();

    gcvt_2 (measured_voltage, 3, volts_str);
    gcvt_2 (measured_current, 3, current_str);
    mini_snprintf (buf, sizeof buf, "Out: %sV, %sA", volts_str, current_str);
    UG_FontSelect (&FONT_12X16);
    UG_PutString (0, 0, buf);

    // Draw analog line matching percentage of max load current
    for (i = 0; i < 4; i++) {   // draw four lines
      // Full scale set to current limit 
      line_len = (int) ((measured_current * 256.0) / current_set);
      UG_DrawLine (0, BAR_ROW + i, line_len, BAR_ROW + i, C_BLACK);
    }

    UG_FontSelect (&FONT_8X12); // Change to lower font

    volts_str[0] = ' ';
    gcvt_2 (voltage_set, 3, volts_str + 1);
    if (voltage_cursor_on && (flash_cntr & 1)) {
      volts_str[strlen (volts_str) - 1 - voltage_position] = '_';
    }

    current_str[0] = ' ';
    gcvt_2 (current_set, 3, current_str + 1);
    if (current_cursor_on && (flash_cntr & 1)) {
      current_str[strlen (current_str) - 1 - current_position] = '_';
    }

    mini_snprintf (buf, sizeof buf, "Sets: %sV, %sA", volts_str, current_str);
    UG_PutString (0, 23, buf);

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

    mini_snprintf (buf, sizeof buf, "Vin: %sV, Ticks: %d", Vin_str, cntr++);
    UG_PutString (0, 49, buf);

    oled_update_ssd1322 ();

    vTaskDelay (pdMS_TO_TICKS (200));   // was 100
    flash_cntr++;
    oled_stack_size = uxTaskGetStackHighWaterMark (NULL);
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
  mcp4728_write (0x01, 1, 2048 / 6);    // for 0.517 volts, value of 341
  mcp4728_write (0x01, 1, 546); // for 0.829 volts for 0.400 ma value of 546
  mcp4728_write (0x02, 0, 0x0000);      // set to zero
  mcp4728_write (0x03, 0, 2900);        // Voltage output zero offset

  voltage_wrt (0.0);
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

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, buf[0]);
  i2c_write (&i2c, buf[1]);
  i2c_write (&i2c, buf[2]);
  i2c_stop (&i2c);
}

int
mcp4728_rd_bsy (void)
{
  uint8_t addr = MCP4728_ADDR (0);      // I2C Address
  uint8_t buf[5];
  uint8_t bsy = 0;

  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 3);
  i2c_stop (&i2c);

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
  config |= 0x0F << 3;          // Set SADC bits
  config |= 0x0F << 7;          // Set BADC bits
  config |= 0x01 << 11;         // Set PG bits. // ±160 mV
  config |= 0x01 << 13;         // Set BRNF bit, 32 volt FSR
  ina219_reg_write (0x00, config);

//  std_printf ("%s(%d) Config reg: 0x%04X\n", __func__, __LINE__, config);       // Config reg: 0x199F

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
  ina219_reg_read (2);
  return 0;
}

int
ina219_reg_read (uint8_t reg)
{
  uint8_t addr = INA219_ADDR (0);       // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;

  // Set register pointer 
  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, reg);
  i2c_stop (&i2c);

  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

//  std_printf ("%s(%d) 0x%02X 0x%02X\n", __func__, __LINE__, buf[0], buf[1]);
  val16 = (buf[0] << 8) | buf[1];

  return (val16);
}

void
ina219_reg_write (uint8_t reg, uint16_t val16)
{
  uint8_t addr = INA219_ADDR (0);       // I2C Address

  // Set register pointer 
  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, reg);
  i2c_write (&i2c, val16 >> 8);
  i2c_write (&i2c, val16 & 0xFF);
  i2c_stop (&i2c);
}

void
mcp9808_init (void)
{
}

float
mcp9808_read (void)
{
  uint8_t addr = MCP9808_ADDR (0);      // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;
  float temp = 0.0;

  // Set register pointer 
  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x05);       // AMBIENT TEMPERATURE REGISTER
  i2c_stop (&i2c);

  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

  buf[0] &= 0x1F;
  val16 = (buf[0] << 8) | buf[1];

  temp = val16 / (16.0);        // save float value
  gcvt_2 (temp, 1, mcp9808_str);

  return temp;
}

int
mcp9808_read_int (void)
{
  uint8_t addr = MCP9808_ADDR (0);      // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;
  int temp;

  // Set register pointer 
  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x05);       // AMBIENT TEMPERATURE REGISTER
  i2c_stop (&i2c);

  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

  buf[0] &= 0x1F;
  val16 = (buf[0] << 8) | buf[1];

  temp = val16 / 16;            // save float value

  return temp;
}

int
mcp9808_access_test (void)
{
  uint8_t addr = MCP9808_ADDR (0);      // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;

  // Set register pointer 
  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x05);       // AMBIENT TEMPERATURE REGISTER
  i2c_stop (&i2c);

  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

  val16 = (buf[0] << 8) | buf[1];
  return val16;
}

void
init_cursors (void)
{
  voltage_cursor_on = 0;
  voltage_position = 2;
  voltage_multi = 100;

  current_cursor_on = 0;
  current_position = 1;
  current_multi = 10;
}

void
voltage_button (int button_state)
{
  // Move to next up scale
  voltage_cursor_on = 1;
  voltage_position++;
  voltage_multi *= 10;

#if 0
  if (voltage_position == 3) {
    voltage_position++;         // skip over decimal point
  }
#endif
//  if (voltage_position > 4) {
  if (voltage_position > 2) {
    voltage_cursor_on = 1;
    voltage_position = 0;
    voltage_multi = 1;
  }
  std_printf ("%s Button state %d, %d\n", __func__, button_state, voltage_multi);
}

void
current_button (int button_state)
{
  // Move to next up scale  
  current_cursor_on = 1;
  current_position++;
  current_multi *= 10;

#if 0
  if (current_position == 3) {
    current_position++;         // skip over decimal point
  }
#endif
//  if (current_position > 4) {
  if (current_position > 2) {
    current_cursor_on = 1;
    current_position = 0;
    current_multi = 1;
  }

  std_printf ("%s Button state %d, %d\n", __func__, button_state, current_multi);
}

void
output_enble (int state)
{
  if (state & 1) {
    output_enb (1);
  }
  else {
    output_enb (0);
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
    output_enb (1);
    voltage_wrt (voltage_set);
    led_on (OUTPUT_LED);
  }
  else {
    output_enb (0);
    voltage_zero_wrt(); // turn off voltgae out so out is zero
    led_off (OUTPUT_LED);
  }
}

void
mem1_button_CB (void)
{
  voltage_set = 3.3;  // preset the supply to 3.3 Volts
  if (output_state) {
     voltage_wrt (voltage_set);
  }
  current_wrt (0.250);
  init_cursors ();

  std_printf ("Mem1 Button press\n");
  return;
}

void
mem2_button_CB (void)
{
  voltage_set = 5.0;
  if (output_state) {
     voltage_wrt (voltage_set);
  }
//  voltage_wrt (5.0);            // preset the supply to 5.0 Volts
  current_wrt (0.300);
//  current_wrt (0.100);
  init_cursors ();

  std_printf ("Mem2 Button press\n");
  return;
}

void
voltage_wrt (float voltage)
{
  voltage_set = voltage;
//v_dac_set = (int) (voltage / 0.01525);        // for 5.000V
  v_dac_set = (int) (voltage / 0.01549);        // for 5.000V
  mcp4728_write (0x00, 1, v_dac_set);   // 1 millvolt per bit ??
}

void
voltage_zero_wrt (void)
{
  float voltage = 0.0;
  v_dac_set = (int) (voltage / 0.01525);	// for 5.000V
  mcp4728_write (0x00, 1, v_dac_set);	// 1 millvolt per bit ??
}

void
current_wrt (float current)
{
//  char i_str[20];
//  i_dac_set = (int) (current / 0.00050);        // for 0.266 at DAC output
  i_dac_set = (int) (current * 2000); // 2 millvolt dac limit equal 1 milliamp measurement

#if 0
  gcvt_2 (current, 3, i_str);
  std_printf ("i:%s, i_dac_set: %d\n", i_str, i_dac_set);
#endif
  
  mcp4728_write (0x01, 1, i_dac_set);
  current_set = current;
}

static void
sw_debouce_task (void *arg __attribute__ ((unused)))
{
  uint16_t mcp23017_inputs = 0;
  uint16_t mcp23017_inputs_last = 0;
  uint8_t key_status =0;
  int indx = 0;
  const TickType_t xBlockTime = 2000;
  uint32_t __attribute__ ((unused)) ulNotifiedValue;
  int state = 0;
//  int last_state = 0;
  float millivolts = 0.0;
  float milliamps = 0.0;
  uint16_t mcp23017_io_dir = 0;

//  std_printf ("\n===============\n  %s() task started\n", __func__);

  // Setup MCP23017 Outputs
  mcp23017_reset ();
  mcp23017_io_dir = 0xffff;     // default io_dir as inputs
  mcp23017_init (mcp23017_io_dir);

  // Setup interrupt registers
  mcp23017_write16 (GPINTEN_REG, 0xffff);
  mcp23017_write16 (DEFVAL_REG, 0x0000);
  mcp23017_write16 (INTCON_REG, 0x0000);        // interrupt on rsing or falling logic

  // Failed without line below.
  mcp23017_inputs_last = mcp23017_read16 (GPIO_REG);    // clear any  pending interrupts

#ifdef USE_CAP_TOUCH
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);  // AT42QT1070 Change line

// Clear "Change" interrupt line
  at42qt1070_init ();
  at42qt1070_show_status ();
  
  at42qt1070_det_status ();
  at42qt1070_key_status (); 
#endif

  for (;;) {

//    mcp23017_read16 (GPIO_REG); // clear any  pending interrupts

    ulNotifiedValue = ulTaskNotifyTake (pdFALSE, xBlockTime);   // normal
//    ulNotifiedValue = ulTaskNotifyTake (pdTRUE, xBlockTime);
    if (ulNotifiedValue == 0) {
      /* Did not receive a notification within the expected time. */
      //     vCheckForErrorConditions();
//      std_printf ("Event notification timeout\n");

      // Not sure if the line below is needed
      //   mcp23017_read16 (GPIO_REG); // clear any  pending interrupts
    }
    else {
      // Wait of interrupt
      if ((gpio_port_read (GPIOB) & 1) == 0) {
        //      mcp23017_int_flag = mcp23017_read16 (INTF_REG);
        //      mcp23017_int_flag = mcp23017_read16 (INTCAP_REG); // INTF_REG);
        mcp23017_inputs = mcp23017_read16 (GPIO_REG) & 0xFFFF;  // mcp23017_inputs_mask;
//      mcp23017_inputs_delta = mcp23017_inputs_last ^ mcp23017_inputs;
        std_printf ("%d, inputs: 0x%04X, 0x%04X, 0x%04X\n", indx++, mcp23017_inputs,
                    mcp23017_inputs_last, mcp23017_inputs ^ mcp23017_inputs_last);
//      mcp23017_inputs_last = mcp23017_inputs;
        
#ifdef USE_CAP_TOUCH	
        if (!(mcp23017_inputs & 1<<6)) { 
	   // Need to read both status bytes to clear change int line 
           at42qt1070_det_status ();
	   key_status = at42qt1070_key_status();
//           std_printf ("key_status: %d\n", key_status);
	   if (key_status == 1) {
               mem1_button_CB ();
	   }
	   if (key_status == 2) {
               mem2_button_CB ();
	   }
	   if (key_status == 4) {
             output_button_CB (0);
	   }
	}
#endif
	
        V_RotaryCurrentState = mcp23017_inputs & 3;
        V_RotaryTransition = (V_RotaryTransition << 2) | V_RotaryCurrentState;

        V_RotaryPosition = V_RotaryPosition + enc_states[V_RotaryTransition & 0x0F];
        if (V_RotaryPosition != V_RotaryPosition_last) {
//          std_printf ("V encoder: %d, %d\n", mcp23017_inputs & 3, V_RotaryPosition);
          V_RotaryPosition_last = V_RotaryPosition;
          millivolts = (float) ((enc_states[V_RotaryTransition & 0x0F] * voltage_multi) * 0.001);
          voltage_set -= millivolts;    // why neg
          if (voltage_set > 15.0) {
            voltage_set = 15.0;
          }
          if (voltage_set < 0.0) {
            voltage_set = 0.0;
          }
          voltage_wrt (voltage_set);
//          std_printf ("dac set: %d\n", v_dac_set);
        }

        I_RotaryCurrentState = (mcp23017_inputs >> 3) & 3;
        I_RotaryTransition = (I_RotaryTransition << 2) | I_RotaryCurrentState;
        I_RotaryPosition = I_RotaryPosition + enc_states[I_RotaryTransition & 0x0F];
        if (I_RotaryPosition != I_RotaryPosition_last) {
//          std_printf ("I encoder: %d, %d\n", (mcp23017_inputs >> 3) & 3, I_RotaryPosition);
          I_RotaryPosition_last = I_RotaryPosition;
          milliamps = (float) ((enc_states[I_RotaryTransition & 0x0F] * current_multi) * 0.001);
          current_set -= milliamps;
          if (current_set > 1.5) {
            current_set = 1.5;
          }
          if (current_set < 0.0) {
            current_set = 0.0;
          }
          current_wrt (current_set);
        }

        // Check all Buttons
        if (!(state = (mcp23017_inputs >> 2) & 1)) {
          voltage_button (state);
        }

        if (!(state = (mcp23017_inputs >> 5) & 1)) {
          current_button (state);
        }

#ifndef USE_CAP_TOUCH	
        state = ((mcp23017_inputs >> 8) & 1);
        last_state = ((mcp23017_inputs_last >> 8) & 1);
        if (state ^ last_state) {       // if there is any state change
          output_button_CB (state);
        }

        // Check MEM2 Buttons
        if (!(state = (mcp23017_inputs >> 7) & 1)) {
          mem2_button_CB ();
        }

        // Check MEM1 Buttons
        if (!(state = (mcp23017_inputs >> 6) & 1)) {
          mem1_button_CB ();
        }
#endif

        // Save present state for next button press
        mcp23017_inputs_last = mcp23017_inputs;
      }
    }
    bounce_stack_size = uxTaskGetStackHighWaterMark (NULL);

    //  mcp23017_read16 (GPIO_REG); // clear any  pending interrupts   
  }
}

void
voltage_read (void)
{
  uint16_t val16 = 0;

  val16 = ina219_reg_read (2);  // read bus voltage
  val16 >>= 3;                  // shift out status bits.
  measured_voltage = (float) val16 *4.0;        // 4 millivolts LSB 
  measured_voltage = measured_voltage / 1000.0; // Convert to volts
//  measured_voltage -= 0.030; // offset adjust
  measured_voltage -= 0.020; // offset adjust
}

void
current_read (void)
{
  uint16_t val16 = 0;

  val16 = ina219_reg_read (1);  // read shut voltage, 0.0315 amps for 100 ohms
  if (val16 && 0xC000) {        // Check for nagative values
  }
  else {
    val16 = 0;                  // -= 392;  // remove constant current offset.
  }
  measured_current = (float) val16 *0.000010;   // 10 uAmps LSB
  measured_current /= 0.1;      // divided by 0.1 ohm sense resistor.
  measured_current -= 0.002;     // offset adjust
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
  char str[50];

  memset (str, 0, sizeof (str));
  gcvt_2 (measured_voltage, 3, str);
  std_printf ("\n Output: %s V", str);

  memset (str, 0, sizeof (str));
  gcvt_2 (measured_current, 3, str);
  std_printf (", %s A", str);

  std_printf (", mode: %s", mode_str);

  std_printf (", temp: %sC\n", mcp9808_str);

  // Read stm32f103 temp, ref v, ch0, and ch1. PA0 and PA1
  rd_temp ();

#ifdef USE_CAP_TOUCH
  at42qt1070_show_status ();
#endif

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
  iwdg_set_period_ms (5000);    // trigger below 3000
#endif

  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);  // IRQ mcp23017 Int
#ifdef USE_CAP_TOUCH  
  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8);  // AT42QT1070 Change line
#endif

  // OLED and mcp23017 reset line setup
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11 | GPIO5);

  // OLED reset line toggle
  gpio_clear (GPIOB, GPIO11);
  sleep (10);
  gpio_set (GPIOB, GPIO11);
 
 // https://github.com/BuFran/hal/blob/master/include/hal/delay.h 
 //  systick_get_value();
  
#if 0
  // test sleep() timming
  while(1) {
  gpio_clear (GPIOB, GPIO11);
  sleep (10);
  gpio_set (GPIOB, GPIO11);
  sleep (10);
  }
#endif

  // OUTPUT enable
  gpio_set_mode (GPIO_PORT_OUT_ENB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                 GPIO_OUT_ENB);

  // LED port setup
  led_all_setup ();
  ps_ready = 0;

#ifndef USE_USB
  gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
  std_set_device (mcu_uart2);   // Use UART2 for std I/O
  open_uart (2, 115200, "8N1", "rw", 0, 0);
#endif

  /* I2C setup */
  rcc_periph_clock_enable (RCC_I2C1);   // I2C
  rcc_periph_clock_enable (RCC_AFIO);
  gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
  gpio_set (GPIOB, GPIO6 | GPIO7);      // Idle high

  // AFIO_MAPR_I2C1_REMAP=0, PB6+PB7
  gpio_primary_remap (0, 0);

  // Configure I2C1
  i2c_configure (&i2c, I2C1, 1000);

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
  exti_setup ();

#define STACK_SIZE 500          // was 800

// Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY).
  xTaskCreate (flash_task, "flash", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

  xTaskCreate (console_task, "console", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

  xTaskCreate (oled_task, "oled", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

  // This is the interruot drivern switch denbounce task. Need to the highest priority task!
  xTaskCreate (sw_debouce_task, "sw_debouce", STACK_SIZE, NULL, configMAX_PRIORITIES,
               &sw_debouce_taskHandle);

  program_bytes = flash_bytes_used ();

  vTaskStartScheduler ();

  for (;;);
}

// End main.c
