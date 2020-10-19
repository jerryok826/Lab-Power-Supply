/* main.c : Lab Power Supply 
 */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include "i2c.h"
#include "gcvt_2.h"
#include "mcp9808.h"

#include "main.h"
// indent -gnu -br -cli2 -lp -nut -l100 mcp9808.c

extern I2C_Control i2c_handle;
extern char mcp9808_str[];

void
mcp9808_init (void)
{
}

#define MCP9808_TEMP_REG 0x05

int
mcp9808_read_tens_deg (void)
{
  uint8_t addr = MCP9808_ADDR (0);  // I2C Address
  uint8_t buf[5];
  uint16_t val16 = 0;

  // Set register pointer 
  mutex_lock();  
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, MCP9808_TEMP_REG);   // AMBIENT TEMPERATURE REGISTER
  i2c_stop (&i2c_handle);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 2);
  i2c_stop (&i2c_handle);
  mutex_unlock();

  buf[0] &= 0x1F;               // Clear flag bits
  val16 = (buf[0] << 8) | buf[1];       // temperature deg C * 16

  // Needs to be divided  by 16 to convert to degs C.
  return val16;
}

float
mcp9808_read_float (void)
{
  int temp_mills = 0;
  float temp;

  temp_mills = mcp9808_read_tens_deg ();
  temp = temp_mills / (16.0);   // save float value
  gcvt_2 (temp, 1, mcp9808_str);

  return temp;
}
