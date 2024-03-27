/*
 * main_loop.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
//#include <stdlib.h>
#include <stdbool.h>
//#include <ctype.h>
//#include <stdint.h>

#include "main_loop.h"
#include "led_drv.h"
#include "mcp9808.h"
#include "mcp23017.h"
#include "at42qt1070.h"
#include "ina219_drv.h"
#include "ugui.h"
#include "oled_drv.h"
#include "i2c_scan.h"

// indent -gnu -br -cli2 -lp -nut -l100 main_loop.c

parameters_t param = {
  0.0,                          // v_out;
  0.0,                          // i_out;
  0.0,                          // v_set;
  0.0,                          // i_set;
  0.0,                          // p_out;
  0.0,                          // v_in;
  0.0,                          // heatsink_temp;
  0.0,                          // cpu_temp;  
  0.0,                          // mem1_v_set;
  0.0,                          // mem2_v_set;

  0,                            // mode;
  0,                            // out_on;
  0,                            // lock;  
  0,                            // ticks;  // param.ticks
};

enum LPS_MODE {
  CV,
  CC
};

/// @brief 
typedef struct _oled_values
{
  float volts;
  float amps;
  int bar_len;
  float v_set;
  float i_set;
  int lps_mode;
  float temp_c;
  float top_voltage_limit;
  float Vin;
  int output_state;
  int ticks;
} oled_values_t;

oled_values_t oled2;

void oled_lps (void);
void cap_touch_update (void);
void rotaryVoltageProcess (int encoder_inputs);
void rotaryCurrentProcess (int encoder_inputs);
void encoders_check_status(void);
void oled_update (oled_values_t * p);
int lps_loop (void);

void
oled_default_load (void)
{
  oled2.volts = 3.3;
  oled2.amps = 0.41;
  oled2.v_set = 5.0;
  oled2.i_set = 1.5;
  oled2.bar_len = 125;
//  oled2.last_voltage_preset = 5.0;
  oled2.lps_mode = CV;
  oled2.temp_c = 27.4;
  oled2.top_voltage_limit = 8.0;
  oled2.Vin = 12.2;
  // oled2.Vtop = 9.1;
  oled2.output_state = 0;
  oled2.ticks = param.ticks;
}

extern ADC_HandleTypeDef hadc1;

static volatile bool show_rx = true;    // false;
float mcp9808_float_temp = 0.0;
char mcp9808_str[20] = "26.3";  // "x.x";
char Vin_str[30] = "0.0";
char Vtop_str[30] = "12.0";
#define REGULATOR_DROP (3.0)
float top_voltage_limit = 15.00;        // This should be calculated from input voltage

float millivolts = 0.0;
float milliamps = 0.0;

#define MEM1_VOLTAGE_PRESET 3.3 // was 3.3
#define MEM1_CURRENT_PRESET 0.5 // 1.2

#define MEM2_VOLTAGE_PRESET 5.0
#define MEM2_CURRENT_PRESET 0.5 // 1.5

float mem1_voltage_preset = MEM1_VOLTAGE_PRESET;        // normally 3.3 volts
float mem1_current_preset = MEM1_CURRENT_PRESET;        // normally 3.3 volts

float mem2_voltage_preset = MEM2_VOLTAGE_PRESET;
float mem2_current_preset = MEM2_CURRENT_PRESET;

float last_voltage_preset = 0.0;
//volatile TickType_t mem1_press_time = 0;        // for xTaskGetTickCount ();
//volatile TickType_t mem2_press_time = 0;

uint32_t output_state = 0;
uint32_t mem1_state = 0;
uint32_t mem2_state = 0;

volatile int mem1_press_time = 0;       // for xTaskGetTickCount ();
volatile int mem2_press_time = 0;

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
int debug_flag = 1;

int program_bytes = 0;
static const char pOFF[] = { "OFF" };

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

  /* USER CODE BEGIN 1 */
HAL_StatusTypeDef ret;
uint8_t buf[50];
int16_t val;
float temp_c;
int i = 0;
int err = 0;
int16_t gpio_reg = 0;

void
output_enb (int enb)            // to enable the output
{
  if (enb) {
    led_on (OUTPUT_LED);
    param.out_on = 1;
    HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, 1);
  }
  else {
    led_off (OUTPUT_LED);
    param.out_on = 0;
    HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, 0);
  }
}

#define FALLING_EDGE 0
#define RISING_EDGE  1
void
output_button_CB (int edge_dir)
{
  printf ("Output Button edge %d\r\n", edge_dir);
  if (edge_dir == FALLING_EDGE) {
    // toggle output state
    output_state = !output_state;
  }

  if (output_state & 1) {
    voltage_wrt (param.v_set);
    led_on (OUTPUT_LED);
    output_enb (1);
  }
  else {
//    voltage_zero_wrt (); // This is not needed. OUTPUT_ENB will disable output
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
  init_cursors ();              // init encoder change resolution

  if (debug_flag) {
    printf ("Mem1 Button press\r\n");
  }
  return;
}

void
remote_voltage_preset (float preset_voltage)
{
  led_off (MEM1_LED);
  led_off (MEM2_LED);
  param.v_set = preset_voltage;
  last_voltage_preset = param.v_set;
  voltage_wrt (param.v_set);
}

void
remote_current_preset (float preset_current)
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
  init_cursors ();        // init encoder change resolution

  if (debug_flag) {
    printf ("Mem2 Button press\r\n");
  }
  return;
}

int
diag_system (void)
{
  float temp_c = 0;
  int i2c_device = 0;

  printf ("\r\n\r\n\r\nRun Diagnostic Tests\r\n");

  i2c_device = i2c_scan ();
  printf ("i2c devices found: %d\r\n", i2c_device);

  // on board I2C items
  printf ("mcp23017_access_test: ");
  mcp23017_access_test ();
  printf ("Passed\r\n");

  printf ("at42qt1070_access_test: ");
  at42qt1070_key_status ();
  printf ("Passed\r\n");

  // remote baord I2C items
  printf ("mcp9808_access_test: ");

  temp_c = mcp9808_read ();     // read and convert to Deg C
  printf ("Passed %3.2f C\r\n", temp_c);

  printf ("ina219_access_test: ");
  ina219_access_test ();
  printf ("Passed\r\n");

  printf ("mcp4728_access_test: ");
  mcp4728_access_test();
  printf ("Passed\r\n");

  return 0;
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
  voltage_multi = 50;

  current_cursor_on = 0;
  current_position = 2;
  current_multi = 50;
}

/*********************************************************************
 * Read ADC Channel
 *********************************************************************/

int
adc_read (int ch)
{
  int adc_val = 0;
  HAL_ADC_Start (&hadc1);       // start the adc 

  HAL_ADC_PollForConversion (&hadc1, 100);      // poll for conversion 

  adc_val = HAL_ADC_GetValue (&hadc1);  // get the adc value 

  HAL_ADC_Stop (&hadc1);        // stop adc 
  return adc_val;
}

float
read_Vin (void)
{
  param.v_in = ((float) adc_read (ADC_CHANNEL_0) / 104.5);      // Need 1% resistors here!
  return param.v_in;
}

void
read_environment(void)
{
  int temp100, vref;
  int adc0;

  temp100 = degrees_C100 ();
  vref = adc_read (ADC_CHANNEL_VREFINT) * 330 / 4095;   // 
  adc0 = (adc_read (ADC_CHANNEL_0) * 330 / 4095) * 11.875; // VIN_CHK
//  adc1 = read_adc (1) * 330 / 4095;
  printf ("Temperature %d.%02d C, Vref %d.%02d Volts, ch0 %d.%02d V\r\n",
          temp100 / 100, temp100 % 100, vref / 100, vref % 100, adc0 / 100, adc0 % 100);

  printf ("Vin: %3.3fV, Vtop: %sV\r\n", read_Vin(), Vtop_str);
}

int
degrees_C100 (void)
{
  int vtemp;
  static const int v25 = 1365;  // see https://github.com/ve3wwg/stm32f103c8t6/issues/11

  vtemp = adc_read (ADC_CHANNEL_TEMPSENSOR);
//  vtemp = (int)adc_read(ADC_CHANNEL_TEMPSENSOR) * 3300 / 4095;
  return (v25 - vtemp) * 1000 / 45 + 2500;      // temp = (1.43 - Vtemp) / 4.5 + 25.00
}

int
lps_setup (void)
{

  printf ("=====================\r\n");
  printf ("=====================\r\n");
  printf ("%s Built on: %s @ %s\r\n", __FILE__, __DATE__, __TIME__);
  printf ("%d, %s %s(%d)\r\n", i++, __FILE__, __func__, __LINE__);

  HAL_Delay (100);

//  i2c_scan ();
  diag_system ();

  oled_init_real ();
  oled_init_ssd1322 ();
  splash_scr ();

  mcp23017_reset ();
  mcp23017_init (0xFFFF);       // 0xffff;  // default io_dir as inputs
  // Init the hardware
  led_all_on ();
  ina219_reset ();
  mcp4728_init ();
  ina219_init ();

  output_enb (0);   // Disable output
  param.v_set = 0.0;
  param.i_set = 1.5;

  voltage_wrt (param.v_set);
  current_wrt (param.i_set);
  printf ("v_set: %3.2fV, iset: %3.2fA\r\n", param.v_set, param.i_set);

  at42qt1070_show_status ();

  read_environment();

  return 0;
}


 // https://electronics.stackexchange.com/questions/99915/stm32-rotary-encoder-with-hardware-interrupts
static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

int
lps_loop (void)
{
  int line_len = 0;
  float amps = 0.0;
  param.v_set = 3.3;
  uint16_t mcp23017_io_dir = 0;

  init_cursors ();
  led_all_on ();

//  oled_display_off();
  oled_init_real ();     // clears pix map and init th ugui driver.
  oled_init_ssd1322 ();

  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);

  splash_scr ();

  led_all_off ();

// Clear at42qt1070 "Change" interrupt line
//  gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8); 

  at42qt1070_init ();           // 100millisec delay
  at42qt1070_det_status ();     // clear any stale status 
  at42qt1070_key_status ();     // clear any stale status

  // Setup MCP23017 Outputs
  mcp23017_reset ();
  mcp23017_io_dir = 0xffff;     // default io_dir as inputs
  mcp23017_init (mcp23017_io_dir);

  // Setup interrupt registers
  mcp23017_write16 (GPINTEN_REG, 0x007f);
  mcp23017_write16 (DEFVAL_REG, 0x0000);
  mcp23017_write16 (INTCON_REG, 0x0000);        // interrupt on rising or falling logic

//  oled_init_all ();
  int start_time = HAL_GetTick ();
  int last_tick = start_time;
  int new_tick = start_time;
//  int sub_64_tick = 0;
  int last_display_update_tick = start_time;
//  int last_oled_update_tick = start_time;

  param.v_set = 0.0;
  param.i_set = 1.5;
  voltage_wrt (param.v_set);
  current_wrt (param.i_set);

  oled_default_load ();

  for (;;) {
    new_tick = HAL_GetTick ();
//    printf("enc %d\r\n",new_tick);
    if (last_tick != new_tick) {
      last_tick = new_tick;
      led_toggle (DEBUG_LED);   // (OUTPUT_LED);
      // Check mcp23017 Int line
//      if ((sub_64_tick & 0x07) == 0){
//      cap_touch_update ();      // Should use touch GPIO pin 
//      }

      // Any change on the voltage/Current encoders
      encoders_check_status();

      if (new_tick > (last_display_update_tick + 1000)) {
        last_display_update_tick = new_tick;
//      printf ("oled %d\r\n", new_tick);
        encoders_check_status();

        oled2.volts = voltage_read ();
        oled2.amps = current_read ();
        oled2.v_set = param.v_set;
        oled2.i_set = param.i_set;
        oled2.bar_len = line_len = (int) ((amps * 256.0) / param.i_set);
        oled2.lps_mode = CV;
        oled2.temp_c = mcp9808_read ();
        oled2.top_voltage_limit = top_voltage_limit;
        oled2.Vin = read_Vin ();

        encoders_check_status();

//      oled2.Vtop = 9.1;
//      oled2.ticks = param.ticks & 0xff;
        oled2.output_state = param.out_on;      // output_state;
        oled2.ticks = HAL_GetTick () % 100;     // TaskGetTickCount () / 1000);
        oled_update (&oled2);  // build screen picture

        encoders_check_status();

        oled_update_ssd1322 (); // Send to OLED
      }
    }
  }
}

void
cap_touch_update (void)
{
  uint8_t key_status = 0;
//  static volatile int press_time = 0;
//  static volatile int release_time = 0;
//  static volatile int press_time_diff = 0;
//  static volatile int press_long = 0;

  if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_8) == 0) {

    at42qt1070_det_status ();
    key_status = at42qt1070_key_status () & 0x07;
//    switch (key_value) {
    switch (key_status) {
      case 1:
        mem1_button_CB ();
        break;
      case 2:
        mem2_button_CB ();
        break;
      case 4:
        output_button_CB (0);
        break;
    }
  }
}

void encoders_check_status(void) 
{
  uint16_t mcp23017_inputs = 0;

  if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0) == 0) {
       mcp23017_inputs = mcp23017_read16 (GPIO_REG);
      rotaryVoltageProcess (mcp23017_inputs & 0x07);  // Only these encoders bits

      rotaryCurrentProcess ((mcp23017_inputs >> 3) & 0x7);    // Only these encoders bits
  }
  cap_touch_update ();      // Should use touch GPIO pin 
}

void
rotaryVoltageProcess (int encoder_inputs)
{
  float millivolts = 0.0;
  static int last_encoder_inputs;

  encoder_inputs &= 0x7;
  /* Check for V encoder bit change */
  if (last_encoder_inputs == encoder_inputs) {
    // Just return if no change
    return;
  }
  V_RotaryCurrentState = encoder_inputs & 3;
  V_RotaryTransition = (V_RotaryTransition << 2) | V_RotaryCurrentState;
  V_RotaryPosition = V_RotaryPosition + enc_states[V_RotaryTransition & 0x0F];
  if (V_RotaryPosition != V_RotaryPosition_last) {
    printf ("V encoder: %d, %d\r\n", encoder_inputs & 3, V_RotaryPosition);
    V_RotaryPosition_last = V_RotaryPosition;
    millivolts = (float) ((enc_states[V_RotaryTransition & 0x0F] * voltage_multi) * 0.001);
    param.v_set -= millivolts;  // why neg
    printf ("V encoder: %d, %d, %2.2f %3.2f\r\n", encoder_inputs & 3, V_RotaryPosition, millivolts,
            param.v_set);
    // Check for legal voltage range
    if (param.v_set > top_voltage_limit) {
      param.v_set = top_voltage_limit;
    }
    if (param.v_set < 0.0) {
      param.v_set = 0.0;
    }
    last_voltage_preset = param.v_set; // what does last_voltage_preset do?
    voltage_wrt (param.v_set);
  }

  // Get the SW buttons edge state
  if ((last_encoder_inputs & 1 << 2) != (encoder_inputs & 1 << 2)) {
    if ((encoder_inputs & 1 << 2) == 0) {
//          voltage_button (sw_state);
      printf ("V encoder button pressed!\r\n");
    }
  }
  last_encoder_inputs = encoder_inputs;
}

void
rotaryCurrentProcess (int encoder_inputs)
{
  float milliamps = 0.0;
  static int last_encoder_inputs;

  encoder_inputs = encoder_inputs & 0x7;;
  /* Check for I encoders bit change */
  if (last_encoder_inputs == (encoder_inputs)) {
    return; // Just return if no change
  }

  // Process current encoder changes
  I_RotaryCurrentState = encoder_inputs & 3;
  I_RotaryTransition = (I_RotaryTransition << 2) | I_RotaryCurrentState;
  I_RotaryPosition = I_RotaryPosition + enc_states[I_RotaryTransition & 0x0F];
  if (I_RotaryPosition != I_RotaryPosition_last) {
    printf ("I encoder: %d, %d\r\n", (encoder_inputs >> 3) & 3, I_RotaryPosition);
    I_RotaryPosition_last = I_RotaryPosition;
    printf ("I encoder: %d, %d, %2.2f %3.2f\r\n", (encoder_inputs >> 3) & 3,
            I_RotaryPosition, milliamps, param.i_set);
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

  // Get the SW buttons edge state
  if ((last_encoder_inputs & 1 << 2) != (encoder_inputs & 1 << 2)) {
    if ((encoder_inputs & 1 << 2) == 0) {
//          current_button (sw_state);
      printf ("I encoder button pressed!\r\n");
    }
  }

  last_encoder_inputs = encoder_inputs;;
}

void
oled_update (oled_values_t * p)
{
  char buf[60];
  char str[20];

  clear_pixmap ();        // Without this we get overlap chars
  UG_SetBackcolor (C_WHITE);
  UG_SetForecolor (C_BLACK);
  UG_FontSelect (&FONT_12X16);

  snprintf (buf, sizeof buf, "Out: %3.3fV, %3.3fA", p->volts, p->amps);
  UG_PutString (0, 0, buf);

  // Draw analog line matching percentage of load current limit.
  for (i = 0; i < 8; i++) {     // draw four lines
    // Full scale set to current limit
    UG_DrawLine (0, BAR_ROW + i, p->bar_len, BAR_ROW + i, C_BLACK);
  }
  UG_FontSelect (&FONT_8X12);   // Change to lower font

  snprintf (buf, sizeof buf, "Sets: %3.3fV, %3.3fA", p->v_set, p->i_set);
  UG_PutString (0, 26, buf);

  strcpy(str,"CV");
  snprintf (buf, sizeof buf, "Mode: %s, Temp: %2.1fC", str, p->temp_c);
  UG_PutString (0, 37, buf);

  if (top_voltage_limit < 8.0) {
    snprintf (buf, sizeof buf, "Vin:%3.2f, ERR: V INPUT LOW!!", p->Vin);
  }
  else {
    snprintf (buf, sizeof buf, "Vin:%3.1fV, Output: %s %d", p->Vin,
              (p->output_state) ? "On," : "Off,", p->ticks);
  }
  UG_PutString (0, 49, buf);
//  oled_update_ssd1322 ();
}

