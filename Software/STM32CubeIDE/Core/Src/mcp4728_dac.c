/*
 * mcp4728_dac.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

#include "i2c_scan.h"

// indent -gnu -br -cli2 -lp -nut -l100 mcp4728_dac.c

#define MULTI_WRT_CMD  (0x40)   // Fig 5-8, pg39
int v_dac_set = 0;
int i_dac_set = 0;

/*****************
  I2C drivers
 *****************/
#define MCP4728_ADDR (0x60 <<1)
extern I2C_HandleTypeDef hi2c1;

void mcp4728_init (void);
int mcp4728_access_test (void);
void mcp4728_write (uint8_t dac, uint8_t gain, uint16_t dac_val);
int mcp4728_rd_bsy (void);
void voltage_wrt (float voltage);
void voltage_zero_wrt (void);
void current_wrt (float current);

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

int
mcp4728_access_test (void)
{
  int status = 0;

  status = i2c_device_access_test (MCP4728_ADDR);
//  printf ("%s %s(%d) status: %d\r\n", __FILE__, __func__, __LINE__, status);
  return status;
}

void
mcp4728_write (uint8_t dac, uint8_t gain, uint16_t dac_val)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t ref_sel = 1;
  uint8_t buf[5];

  buf[0] = MULTI_WRT_CMD | ((dac & 0x03) << 1); // Table 5-1, pg 34 and FIGURE 5-10:
  buf[1] = ((dac_val >> 8) & 0x0F) | ((ref_sel & 1) << 7) | ((gain & 1) << 4);
  buf[2] = dac_val & 0xFF;
  ret = HAL_I2C_Master_Transmit (&hi2c1, MCP4728_ADDR, buf, 3, 1000);   // HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    printf ("%s %s(%d) err: %d\r\n", __FILE__, __func__, __LINE__, ret);
  }
  else {
//      printf("%s %s(%d) PASS: %d\r\n",__FILE__,__func__,__LINE__, ret);
  }
}

int
mcp4728_rd_bsy (void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[5];
  uint8_t rdy = 0;
  uint16_t val16 = 0;
  uint8_t reg_addr = 0;

  ret = HAL_I2C_Master_Transmit (&hi2c1, MCP4728_ADDR, &reg_addr, 1, 1000);     // HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    printf ("%s %s(%d) err: %d\r\n", __FILE__, __func__, __LINE__, ret);
    return -1;
  }
  else {
    // Read 2 bytes from the temperature register
    ret = HAL_I2C_Master_Receive (&hi2c1, MCP4728_ADDR, buf, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
      printf ("ERR: %s %s(%d)\r\n", __FILE__, __func__, __LINE__);
      return -1;
    }
    else {
      //Combine the bytes
      val16 = (buf[1] << 8) | buf[0];
    }
  }
//  i2c_start_addr(&i2c_handle, addr, Read);
//  i2c_read_bytes(&i2c_handle, buf, 3); // why three??
//  i2c_stop(&i2c_handle);

  rdy = (val16 >> 15) & 1;
  // printf ("%s(%d) reg[0]: 0x%02X, rdy: %d\n", __func__, __LINE__, val16, rdy);
  return rdy;
}

void
voltage_wrt (float voltage)
{
  v_dac_set = (int) (voltage / 0.0060); // for 5.000V
  mcp4728_write (0x00, 1, v_dac_set);   // 1 millvolt per bit ??
//  calibrate (); // seems to cause out V to duble
//  printf("%s %s(%d) voltage: %3.3f, v_dac_set: %d\r\n",__FILE__,__func__,__LINE__, voltage, v_dac_set);
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
  i_dac_set = (int) (current * 2000);   // 2 millvolt dac limit equal 1 milliamp measurement

  mcp4728_write (0x01, 1, i_dac_set);
  //  printf("%s %s(%d) currentt: %3.3f, i_dac_set: %d\r\n",__FILE__,__func__,__LINE__, current, i_dac_set);
}
