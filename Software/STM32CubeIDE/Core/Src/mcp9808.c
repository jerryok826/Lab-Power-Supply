/* 
 * mcp9808.c : Lab Power Supply 
 */

#include "main.h"

#include <stdlib.h>
#include <string.h>
//#include <stdbool.h>
//#include <ctype.h>
//#include <stdint.h>

#include "mcp9808.h"

// indent -gnu -br -cli2 -lp -nut -l100 mcp9808.c

void
mcp9808_init (void)
{
}

#define MCP9808_ADDR (0x18 <<1) /// (n)   (0x18|((n)&7))
#define MCP9808_TEMP_REG 0x05
extern I2C_HandleTypeDef hi2c1;

uint16_t
mcp9808_reg_read (int reg)
{
  uint8_t buf[12];
  uint16_t val16 = 0;

  // Tell MCP9808 that we want to read from the temperature register
  //  mutex_lock();  // How is this done in cube??
  buf[0] = reg;
  if (HAL_I2C_Master_Transmit (&hi2c1, MCP9808_ADDR, buf, 1, 100) == HAL_OK) {
    // Read 2 bytes from the temperature register
    if (HAL_I2C_Master_Receive (&hi2c1, MCP9808_ADDR, buf, 2, 10) == HAL_OK) {
      //Combine the bytes
      val16 = (buf[0] << 8) | buf[1];   // temperature deg C * 16
      return val16;
    }
  }
  return 0;
}

float 
mcp9808_read (void)
{
  uint16_t val16 = 0;
  float temp_c = 0.0;

  val16 = mcp9808_reg_read (MCP9808_TEMP_REG);
  val16 &= 0x1FFF;
  temp_c = val16 / 16.0;
  return temp_c;
}
