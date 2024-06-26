/*
 * ina219.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

#include "ina219_drv.h"

// indent -gnu -br -cli2 -lp -nut -l100 ina219_drv.c

//#define MULTI_WRT_CMD  (0x40) // Fig 5-8, pg39
extern I2C_HandleTypeDef hi2c1;
extern int debug_flag;

#define INA219_ADDR  (0x40<<1)

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
  v = v / 1000.0;               // Convert to volts
  return v;
}

uint32_t
current_read_int (void)
{
  uint16_t val16 = 0;

  val16 = ina219_reg_read (1, debug_flag);      // read shut voltage, 0.0315 amps for 100 ohms
  if (val16 && 0xC000) {        // Check for nagative values
  }
  else {
    val16 = 0;                  // -= 392;  // remove constant current offset.
  }
  if (debug_flag) {
//    printf ("%s(%d) %d, 0x%02X\r\n\r\n", __func__, __LINE__, val16, val16);
  }
  return val16;
}

float
current_read (void)
{
  uint32_t val32 = 0;
  float amps = 0.0;

  val32 = current_read_int ();
  amps = (float) val32 *0.000010;       // 10 uAmps LSB
  amps /= 0.1;                  // divided by 0.1 ohm sense resistor.
  return amps;
}

void
ina219_init (void)
{
  uint16_t config = 0;
  uint16_t val16 = 0;

  val16 = ina219_reg_read (0x00, 0);
  printf ("%s(%d) Reset Config reg: 0x%04X\r\n", __func__, __LINE__, val16);

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
  printf ("%s(%d) Config reg: 0x%04X\r\n", __func__, __LINE__, config);

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
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[5];
  uint16_t val16 = 0;

  ret = HAL_I2C_Master_Transmit (&hi2c1, INA219_ADDR, &reg, 1, 10);     // HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    printf ("%s %s(%d) err: %d\r\n", __FILE__, __func__, __LINE__, ret);
    return 1;
  }
  else {
    // Read 2 bytes from the temperature register
    ret = HAL_I2C_Master_Receive (&hi2c1, INA219_ADDR, buf, 2, 10);     // HAL_MAX_DELAY);
    if (ret != HAL_OK) {
      printf ("ERR: %s %s(%d)\r\n", __FILE__, __func__, __LINE__);
      return 1;
    }
    else {
      val16 = (buf[0] << 8) | buf[1];
//       printf ("%s(%d) %d, 0x%02X 0x%02X\r\n", __func__, __LINE__, val16, buf[0], buf[1]);
      return (val16);
    }
  }
  return val16;
}

int
ina219_reg_write (uint8_t reg, uint16_t val16)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[7];

  buf[0] = reg;
  buf[1] = val16 & 0xff;
  buf[2] = val16 >> 8;

  ret = HAL_I2C_Master_Transmit (&hi2c1, INA219_ADDR, buf, 3, 10);      // HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    printf ("%s %s(%d) err: %d\r\n", __FILE__, __func__, __LINE__, ret);
    return -1;
  }
  return 0;
}
