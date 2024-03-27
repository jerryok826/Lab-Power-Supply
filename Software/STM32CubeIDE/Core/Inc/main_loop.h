/*
 * main_loop.h, Jerry OKeefe, 3/6/24
 */

#ifndef __MAIN_LOOP_H
#define __MAIN_LOOP_H

#include <stdint.h>

#include "at42qt1070.h"
#include "i2c_scan.h"
#include "ina219_drv.h"
#include "led_drv.h"
#include "main_loop.h"
#include "mcp23017.h"
#include "mcp9808.h"
#include "oled_drv.h"
#include "ugui_config.h"
#include "ugui.h"

#define BAR_ROW (17) // was 18

#define FILTER_SIZE 10
typedef struct _mov_avg
{
  uint32_t mov_avg_data[FILTER_SIZE];
  int array_size;
  int indx;
  int init_fill;
} mov_avg_t;

typedef struct _parameters
{
  float v_out;
  float i_out;
  float v_set;
  float i_set;
  float p_out;
  float v_in;
  float heatsink_temp;
  float cpu_temp;  
  float mem1_v_set;
  float mem2_v_set;
  
  int mode;
  int out_on;
  int lock;  
  int ticks;  
  
} parameters_t;

void mov_avg_filter_init(mov_avg_t *p);
void put_fifo2(mov_avg_t *p, uint32_t value);
uint32_t get_avg2(mov_avg_t *p);

void show_console (void);
void reboot(void);
int get_program_size(void);

#define MULTI_WRT_CMD  (0x40) // Fig 5-8, pg39
#define SINGLE_WRT_CMD (0x58) // Fig 5-10, pg41
void mcp4728_init (void);
void mcp4728_write (uint8_t dac, uint8_t gain, uint16_t dac_val);
int mcp4728_rd_bsy (void);
int mcp4728_access_test (void);
void voltage_wrt(float voltage);
void current_wrt(float current);
uint16_t voltage_read_int(void);

uint32_t current_read_int (void);

void put_fifo(uint32_t value);
uint32_t get_avg(void);

void mov_avg_filters_init(void);

float voltage_read(void);
float current_read(void);
void voltage_zero_wrt (void);
int degrees_C100 (void);

int ads1115_init (void);
int ads1115_read (uint8_t ch, uint16_t * data);
// void output_button_CB (int button_state);
// void output_button_CB (int button_state, int button_last_state);
void output_button_CB (int edge_dir);

void mem1_button_CB (void);
void mem2_button_CB (void);
void voltage_button (int button_state);
void current_button (int button_state);
void show_status(void);
int flash_bytes_used(void);
void cmd_line_process (void);
void init_cursors (void);

void wait_til_usb_ready (void);
float read_Vin(void);
int diag_system(void);
void oled_init_all(void);
void output_enb (int enb);    // to enable the output
void mem1_check_preset_timeout(void);
void mem2_check_preset_timeout(void);
void vApplicationTickHook (void);
void millisecs_sleep(uint64_t ms);
void calibrate(void);
void remote_voltage_preset(float preset_voltage);
void remote_current_preset(float preset_current);

void hex_dump (uint8_t *buf, int len);
void flash_block_rd ( uint8_t * output_data);
int flash_block_wrt ( uint8_t * input_data);
uint16_t crc16(char *ptr, int count);

void mutex_lock(void);
void mutex_unlock(void);

int on_off_contrl (int on);
int lps_setup(void);
int lps_loop(void);

#endif  /* __MAIN_LOOP_H */
