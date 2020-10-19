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
#include "modbus_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 cmd_line.c

#define UNUSED __attribute((unused))

// static char line_buf[128];

void EditInit (char *buffer, int bufsize);

extern int flash_stack_size;
extern int console_stack_size;
extern int bounce_stack_size;
extern int oled_stack_size;

static void get_line_input(void);

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
do_reboot (int argc UNUSED, char *argv[]UNUSED)
{
  std_printf ("Rebooting in one second\n");
  vTaskDelay (pdMS_TO_TICKS (1000));

  reboot ();
  return 0;
}

extern int show_status_flag;
extern int log_interval;

static int
do_log (int argc, char *argv[])
{
  
  if (argc != 2) {
    log_interval = 10; 
    show_status_flag = 0;
  } else {
    log_interval = my_atoi (argv[1]);
    show_status_flag =1;
  }
  
  if ((log_interval > 0) && show_status_flag) {
    std_printf ("logging interval: %d seconds\n", log_interval);
    show_status_flag = 1;
  }
  else {
    std_printf ("logging off\n");
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
  on_off_contrl (1);
  return 0;
}

static int
do_off (int argc UNUSED, char *argv[]UNUSED)
{
  on_off_contrl (0);
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

extern char mcp9808_str[];
static int
rd_temp_cmd (int argc UNUSED, char *argv[]UNUSED)
{
  int cpu_temp100 = degrees_C100 ();
  
//  std_printf ("Cpu %d.%02d C, Output: %s C\n", cpu_temp100 / 100, cpu_temp100 % 100, mcp9808_str);
  std_printf ("Output: %s C, Cpu %d.%02d C\n", mcp9808_str, cpu_temp100 / 100, cpu_temp100 % 100);

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

static int
do_code (int argc UNUSED, char *argv[]UNUSED)
{
  int stack_size = 600;
  console_stack_size = uxTaskGetStackHighWaterMark (NULL);
  
  std_printf ("Built on: %s, %s\n", __DATE__,__TIME__);
  
  std_printf ("  Program: %d bytes, Heap: %d bytes, %d bytes\n", 
             get_program_size(), xPortGetFreeHeapSize (), xPortGetMinimumEverFreeHeapSize());
  //	uxTaskGetStackHighWaterMark (NULL);
  std_printf ("  Stack size used: %d, %d, %d, %d\n", 
            stack_size - flash_stack_size, stack_size - console_stack_size, 
	    stack_size - bounce_stack_size, stack_size - oled_stack_size);
//            stack_size - flash_stack_size, stack_size - console_stack_size, 
//	    stack_size - bounce_stack_size, stack_size - oled_stack_size);
	     
  return 0;
}

static int
do_cal (int argc UNUSED, char *argv[]UNUSED)
{
  std_printf ("%s(%d) Built on: %s %s\n",__func__, __LINE__,__DATE__,__TIME__);
  calibrate();
  return 0;
}

extern int debug_flag;

static int
do_debug (int argc UNUSED, char *argv[]UNUSED)
{
  debug_flag = !debug_flag;
  return 0;
}

uint16_t crc16(char *ptr, int count)
{
   int  crc;
   char i;
   crc = 0;
   while (--count >= 0)
   {
      crc = crc ^ (int) *ptr++ << 8;
      i = 8;
      do
      {
         if (crc & 0x8000)
            crc = crc << 1 ^ 0x1021;
         else
            crc = crc << 1;
      } while(--i);
   }
   return (crc);
}

#if 0
const char local_buf[256]={"012345667890 hello, %s, %s"__DATE__ __TIME__};
char buf[25];
#endif

static int
do_blk_rd (int argc UNUSED, char *argv[]UNUSED)
{
#if 0
   uint16_t crc=0;
   
   flash_block_rd ((uint8_t *)buf);

//  hex_dump ((uint8_t *)local_buf, sizeof(local_buf));
  hex_dump ((uint8_t *)buf, sizeof(buf));
  
  crc = crc16(buf, 256);
  std_printf ("CRC: %04X\n", crc);
#endif

  return 0;
}

static int
do_blk_wrt (int argc UNUSED, char *argv[]UNUSED)
{
#if 0
   uint16_t crc=0;
   flash_block_wrt ((uint8_t *)local_buf);
   crc = crc16(local_buf, 256);
   std_printf ("CRC: %04X\n", crc);
#endif
   return 0;
}

static const cmd_t commands_tab[] = {
  {"help", do_help, "Show help"},
  {"diag", do_diag, "do system diag"},
  {"v", do_set_voltage, "set voltage"},
  {"c", do_set_current, "set current limit"},
  {"on", do_on, "Turn output on"},
  {"off", do_off, "Turn output off"},
  {"m1", do_mem1, "mem1 output preset"},
  {"m2", do_mem2, "mem1 output preset"},
  {"dac", set_dac_cmd, "<lsb>, set dac directly"},
  {"adc", rd_adc_cmd, "Read ADC input"},
  {"temp", rd_temp_cmd, "Read temperature"},
  {"reboot", do_reboot, "Reboot STM32"},
  {"log", do_log, "<interval> Starts logging with <interval> seconds"},
  {"code", do_code, "Show program code stats"},
  {"cal", do_cal, "Calibrate unit"},
  {"debug", do_debug, "Toggle debug flag"},
  {"blk_rd", do_blk_rd, "block read"},
  {"blk_wrt", do_blk_wrt, "block write"},
  
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
//  char line_buf[128];
  char line_buf[50];
  
#if 1 
  /* process modbus commands */
  get_line_input();
#endif

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
      std_printf ("> ");
      if (haveLine) {
        int result = cmd_process (commands_tab, line_buf);
        switch (result) {
          case CMD_OK:
              std_printf ("> ");
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

static void get_line_input(void)
{
    char rx_buf[100];
    char tx_buf[100];
    char *ptr;
    int mosbua_pkt_len = 0;
    int i=0;
    
  init_registers();

  /* simple noncanonical input */
  do {
    
    mosbua_pkt_len = 0;
    ptr = rx_buf;
    memset (rx_buf, 0, sizeof (rx_buf));

    // Read serial until we have a complete line
    do {
      *ptr++ = std_getc();
      mosbua_pkt_len++;
    } while (strchr (rx_buf, '\n') == NULL);

    if (mosbua_pkt_len > 0) {
      modbus_process (rx_buf, mosbua_pkt_len, tx_buf);
//      response_len = write (fd, tx_buf, response_len);
//      std_printf("%s\n",tx_buf);
      i=0;
      while(tx_buf[i]) {  
//         std_putc(tx_buf[i++]);  
	 putc_uart(2,tx_buf[i++]);
      }
//      std_printf ("TX %d: %s\n", response_len, tx_buf);
    }
    else {                      /* rdlen == 0 */
//      std_printf ("Timeout from read\n");
    }
    /* repeat read to get full message */
  }
  while (1);
}
