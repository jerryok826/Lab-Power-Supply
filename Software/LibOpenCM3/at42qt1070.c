/*********************************************************************
 * at42qt1070.c : 
 *
 *
 *********************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "FreeRTOS.h"
#include "i2c.h"
#include "task.h"
#include "mcuio.h"

#include "at42qt1070.h"
#include "main.h"

// indent -gnu -br -cli2 -lp -nut -l100 at42qt1070.c

extern I2C_Control i2c_handle;  // I2C Control struct
#define AT42QT1070_ADDR(n) (0x1B|((n)&7))

void
at42qt1070_init (void)
{
  // reset device by writing non-zero value to register 0x39
  at42qt1070_write8 (RESET_REG, 0xff);
  vTaskDelay (pdMS_TO_TICKS (100));     // 100millisec delay 
  at42qt1070_det_status ();     // clear any stale status 
  at42qt1070_key_status ();     // clear any stale status
}


static int
at42qt1070_reg_sel (int reg_addr)
{
  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address

  // Let higher layer lock this I2C access 
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg_addr);
  i2c_stop (&i2c_handle);

  return 0;
}

uint8_t
at42qt1070_read8 (int reg_addr)
{
  uint8_t buf[10];

  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address
  mutex_lock ();
  at42qt1070_reg_sel (reg_addr);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 1);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
  return buf[0];
}

int
at42qt1070_write8 (uint8_t reg, uint8_t val8)
{
  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address

  mutex_lock ();
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg);
  i2c_write (&i2c_handle, val8);
  i2c_stop (&i2c_handle);
  mutex_unlock ();

  return 0;
}

uint8_t
at42qt1070_chip_id (void)
{
  uint8_t buf[5];

  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address
  mutex_lock ();
  at42qt1070_reg_sel (CHIP_ID_REG);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 1);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
  return buf[0];
}

uint8_t
at42qt1070_det_status (void)
{
  uint8_t buf[5];

  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address
  mutex_lock ();
  at42qt1070_reg_sel (DET_STATUS_REG);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 1);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
  return buf[0];
}

uint8_t
at42qt1070_key_status (void)
{
  uint8_t buf[5];

  uint8_t addr = AT42QT1070_ADDR (0);   // I2C Address
  mutex_lock ();
  at42qt1070_reg_sel (KEY_STATUS_REG);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 1);
  i2c_stop (&i2c_handle);
  mutex_unlock ();
  return buf[0];
}

void
at42qt1070_show_status (void)
{
  std_printf ("chip_id: 0x%02X, Det status: 0x%02X, Key status: 0x%02X\n",
              at42qt1070_chip_id (), at42qt1070_det_status (), at42qt1070_key_status ());
}
