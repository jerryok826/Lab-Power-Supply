#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

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

#include "main.h"
#include "cmdproc.h"
#include "led_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 cmd_line.c

#define UNUSED __attribute((unused))

// static char line_buf[128];

void EditInit (char *buffer, int bufsize);

bool EditLine (char cin, char *cout);

#define BELL    0x07
#define BS      0x08
#define LF      0x0A
#define CR      0x0D

static char *line_b = (char *) NULL;    // line_buf;
static int size = 0;
static int pos = 0;

/**
 * Initializes the edit buffer.
 *
 * @param buffer the edit buffer
 * @param bufsize the size of the edit buffer
 */
void
EditInit (char *buffer, int bufsize)
{
  line_b = buffer;
  size = bufsize;
}

/** 
 * Processes a character into an edit buffer, returns true
 * if a full line has been received
 * 
 * @param cin the character to process
 * @param cout the output character
 * @return true if a full line was entered, false otherwise
 */
bool
EditLine (char cin, char *cout)
{
  *cout = cin;                  // echo by default
  switch (cin) {                // carriage return is ignored
    case '\n':
      break;
    case '\r':                 // end-of-line
      line_b[pos] = '\0';
      pos = 0;
      return true;
    case 0x7F:
    case 0x08:                 // backspace
      if (pos > 0) {
        pos--;
      }
      break;
    default:
      if (pos < (size - 1)) {   // store char as long as there is space to do so
        line_b[pos++] = cin;
      }
      else {
        *cout = 0x07;           // bell
      }
      break;
  }
  return false;
}

int my_atoi (const char *str);

int
my_atoi (const char *str)
{
  const char *p = str;
  int nub = 0;

  if (str == NULL) {
    return 0;
  }
  while (*p == ' ') {           // isspace() fails
    p++;
  }
  nub = 0;
  while (((*p) >= '0') && ((*p) <= '9')) {      // isdigit() fails
    nub *= 10;
    nub += (*p++) - '0';
  }
  return nub;
}

/*********************************************************************
 * Display a menu:
 *********************************************************************/
void show_help (const cmd_t * cmds);

void
show_help (const cmd_t * cmds)
{
  if (cmds == NULL) {
    return;
  }

  for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
    if ((cmd->name == NULL) || (cmd->help == NULL)) {
      std_printf ("%s(%d) error\n", __func__, __LINE__);
      return;
    }
    std_printf ("%10s: %s\n", cmd->name, cmd->help);
  }
}

static int
do_cc (int argc, char *argv[]UNUSED)
{
  if (argc != 2) {
    std_printf ("Please specify a current!\n");
    return -1;
  }
  return 0;
}

extern int show_status_flag;

static int
do_status (int argc UNUSED, char *argv[]UNUSED)
{
  if (show_status_flag) {
    show_status_flag = 0;
  }
  else {
    show_status_flag = 1;
  }
  return 0;
}

static int
do_reboot (int argc UNUSED, char *argv[]UNUSED)
{
  std_printf ("Rebooting in one second\n");
  vTaskDelay (pdMS_TO_TICKS (1000));

  reboot ();
  return 0;
}

static int
do_log (int argc, char *argv[])
{
  if (argc != 2) {
    std_printf ("Please specify a logging interval (0 to stop)\n");
    return -1;
  }
  int interval = my_atoi (argv[1]);
  if (interval > 0) {
    //       LoggingOn(interval);
  }
  else {
    //      LoggingOff();
  }
  return 0;
}

static int
do_set_voltage (int argc, char *argv[]UNUSED)
{
  if (argc != 2) {
    std_printf ("Please specify a output voltage\n");
    return -1;
  }
  int millvolts = my_atoi (argv[1]);
  float voltage = (float) millvolts / 1000.0;
  voltage_wrt (voltage);

  return 0;
}

static int
do_set_current (int argc, char *argv[]UNUSED)
{
  if (argc != 2) {
    std_printf ("Please specify a output current\n");
    return -1;
  }
  int millamps = my_atoi (argv[1]);
  float current = (float) millamps / 1000.0;
  current_wrt (current);

  return 0;
}

static int
do_on (int argc UNUSED, char *argv[]UNUSED)
{
  output_enble (1);
  led_on (OUTPUT_LED);
  return 0;
}

static int
do_off (int argc UNUSED, char *argv[]UNUSED)
{
  output_enble (0);
  led_off (OUTPUT_LED);
  return 0;
}

static int
set_dac_cmd (int argc, char *argv[]UNUSED)
{
  if (argc != 4) {
    std_printf ("Please specify a DAC value (0 to 4096)\n");
    return -1;
  }   
  int channel = my_atoi (argv[1]);
  int range = my_atoi (argv[2]);
  int bits = my_atoi (argv[3]);
  
  std_printf ("%s(%d, %d, %d)\n",__func__, channel, range, bits);
  
  mcp4728_write (channel, range, bits);   // 1 millvolt per bit ??
  return 0;
}

static int
rd_adc (void)
{
  int adc_value = 1234;
  std_printf ("adc_value: %d, 0x%04X\n", adc_value, adc_value);
  return adc_value;
}

static int
rd_adc_cmd (int argc UNUSED, char *argv[]UNUSED)
{
  rd_adc ();
  return 0;
}

static int
rd_temp_cmd (int argc UNUSED, char *argv[]UNUSED)
{
  // rd_temp ();
  return 0;
}

static int
do_diag (int argc UNUSED, char *argv[]UNUSED)
{
  diag_system ();
  return 0;
}

static int
do_mem1 (int argc UNUSED, char *argv[]UNUSED)
{
  mem1_button_CB ();
  return 0;
}

static int
do_mem2 (int argc UNUSED, char *argv[]UNUSED)
{
  mem2_button_CB ();
  return 0;
}

#if 0
static int
do_code (int argc UNUSED, char *argv[]UNUSED)
{
  std_printf ("%s(%d) Built on: %s %s\n",__func__, __LINE__,__DATE__,__TIME__);
#if 0
    mini_snprintf (buf, sizeof buf, "Program: %d, Heap: %d", 
                 program_bytes, xPortGetFreeHeapSize ());
#endif

  return 0;
}
#endif

static const cmd_t commands_tab[] = {
  {"help", do_help, "Show help"},
  {"diag", do_diag, "do system diag"},
  {"volts", do_set_voltage, "set voltage"},
  {"current", do_set_current, "set current"},
  {"on", do_on, "Turn output on"},
  {"off", do_off, "Turn output off"},
  {"mem1", do_mem1, "press mem1"},
  {"mem2", do_mem2, "press mem2"},
  {"dac", set_dac_cmd, "<lsb>, set dac directly"},
  {"adc", rd_adc_cmd, "Read ADC input"},
  {"temp", rd_temp_cmd, "Read temperature"},
  {"cc", do_cc, "<mA>, set constant current mode"},
  {"s", do_status, "Show status"},
  {"reboot", do_reboot, "Reboot STM32"},
  {"log", do_log, "<interval> Starts logging with <interval> ms"},
//  {"code", do_code, "Show program code stats"},
  {NULL, NULL, NULL}
};

int
do_help (int argc UNUSED, char *argv[]UNUSED)
{
  show_help (commands_tab);
  return 0;
}

void
cmd_line_process (void)
{
  char ch;
  bool haveLine = false;
  char line_buf[128];

  EditInit (line_buf, sizeof (line_buf));

  vTaskDelay (pdMS_TO_TICKS (1000));

  std_printf ("\n          Lab Power Supply Console:\n");
  show_help (commands_tab);

  std_printf ("> ");

  for (;;) {
    haveLine = EditLine (std_getc (), &ch);
    std_printf ("%c", ch);
    if (haveLine == true) {
      std_printf ("\r\n");
//      std_printf ("Line: %s\r\n", line_buf);
      std_printf ("> ");
      if (haveLine) {
        int result = cmd_process (commands_tab, line_buf);
        switch (result) {
          case CMD_OK:
            std_printf ("OK\n");
            break;
          case CMD_NO_CMD:
            break;
          case CMD_UNKNOWN:
            std_printf ("Unknown command, available commands:\n");
//            show_help (commands_tab);
            break;
          default:
            std_printf ("%d\n", result);
            break;
        }
//        std_printf ("%s>", "Fix Me");   // ControlGetModeString());
      }
    }
  }
}