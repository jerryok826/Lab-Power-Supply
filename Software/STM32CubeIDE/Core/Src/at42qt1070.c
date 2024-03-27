/*
 * at42qt1070.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <string.h>
#include <stdio.h>

#include "at42qt1070.h"

// indent -gnu -br -cli2 -lp -nut -l100 at42qt1070.c

// I2C Control struct
extern I2C_HandleTypeDef hi2c1;

#define AT42QT1070_ADDR (0x1B<<1)

void
at42qt1070_init (void)
{
  // reset device by writing non-zero value to register 0x39
  at42qt1070_write8 (RESET_REG, 0xff);
  HAL_Delay(300);  // Shorter than this I see errors

  at42qt1070_AKS_clear(3);
  at42qt1070_AKS_clear(4);
  at42qt1070_AKS_clear(5);
  at42qt1070_AKS_clear(6);
  at42qt1070_AKS_clear(7);

  at42qt1070_write8 (CALIBRATE_REG, 0xff);
  HAL_Delay(100);  // 100millisec delay 
  at42qt1070_det_status ();     // clear any stale status 
  at42qt1070_key_status ();     // clear any stale status
}

int
at42qt1070_reg_sel (uint8_t reg_addr)
{
  HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Master_Transmit(&hi2c1, AT42QT1070_ADDR, &reg_addr, 1, 100); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
      printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
      return -1;
	} 

  printf("%s %s(%d) Pass: %d\r\n",__FILE__,__func__,__LINE__, ret);

  return 0;
}

// Used to disable unused inputs
int
at42qt1070_AKS_clear(uint8_t aks_reg)
{
  int ret=0;

  ret = at42qt1070_write8 (AKS_BASE_REG+aks_reg, 0);
	if ( ret != 0 ) {
      printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
      return -1;
	} 
  return 0;
}

uint8_t
at42qt1070_read8 (uint8_t reg_addr)
{
  HAL_StatusTypeDef ret = HAL_OK; 
  uint8_t buf[10];

  buf[0] = reg_addr;
	ret = HAL_I2C_Master_Transmit(&hi2c1, AT42QT1070_ADDR, buf, 1, 100); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {  
      printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
  } else {
	      ret = HAL_I2C_Master_Receive(&hi2c1, AT42QT1070_ADDR, buf, 1, 100); //HAL_MAX_DELAY);
	      if ( ret != HAL_OK ) {
          printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
	      } else {
//          printf("%s %s(%d) pass 0x%02X\r\n",__FILE__,__func__,__LINE__, buf[0]);
          return buf[0];
        }
      }
  return 0;
}

int
at42qt1070_write8 (uint8_t reg, uint8_t val8)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[10];

  buf[0] = reg;
  buf[1] = val8;

	ret = HAL_I2C_Master_Transmit(&hi2c1, AT42QT1070_ADDR, buf, 2, 100); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
      printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
      return -1;
	} else {
    //  printf("%s %s(%d) Pass: %d\r\n",__FILE__,__func__,__LINE__, ret);
  }
  return 0;
} 

uint8_t
at42qt1070_chip_id (void)
{
  // should return 0x2E
  return at42qt1070_read8 (CHIP_ID_REG);   
}

// TOUCH: This bit is set if any keys are in detect.
uint8_t
at42qt1070_det_status (void)
{
  // DET_STATUS_REG	0x02
  return at42qt1070_read8 (DET_STATUS_REG);
}

// 
uint8_t
at42qt1070_key_status (void)
{
  // KEY_STATUS_REG	0x03
  return at42qt1070_read8 (KEY_STATUS_REG);   
}

void
at42qt1070_show_status (void)
{
  printf ("chip_id: 0x%02X, Det status: 0x%02X, Key status: 0x%02X\r\n",
              at42qt1070_chip_id (), at42qt1070_det_status (), at42qt1070_key_status ());
}