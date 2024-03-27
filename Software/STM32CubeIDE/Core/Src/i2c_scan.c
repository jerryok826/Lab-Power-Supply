/* 
 * i2c_scan.c 3/4/24
 */
#include "main.h"

#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

int i2c_device_access_test(uint8_t i2c_adr)
{
  HAL_StatusTypeDef error = HAL_OK;

  error = HAL_I2C_IsDeviceReady(&hi2c1, i2c_adr<<1,1,HAL_MAX_DELAY);
  if (error == HAL_OK) {
    return 0;
  } else {
    return -1;
  }
}

void
i2c_scan2 (void)
{
  int rc=0;
  uint8_t address;
  int nDevices;

  printf ("Scanning...\r\n");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    rc= i2c_device_access_test(address);
    if (rc == 0) { // error == HAL_OK) {
      printf ("I2C device @ 0x%02x\r\n", address);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    printf ("No I2C devices found\r\n");
  } else {
    printf ("%d I2C devices found\r\n",nDevices);
  }
}

int 
i2c_scan (void)
{
  int rc=0;
  uint8_t address;
  int nDevices;
  uint8_t i2c_hits[128];
  int idx=0;

  memset(i2c_hits,0,sizeof(i2c_hits));

  nDevices = 0;
  for (address = 1; address < 128; address++) {
    rc= i2c_device_access_test(address);
    if (rc == 0) { 
      i2c_hits[idx++] = address;
      nDevices++;
    }
  }
  if (nDevices == 0) {
    printf ("No I2C devices found\r\n");
  } else {
    printf ("%d I2C devices found: ",nDevices);
    for(idx=0;idx<127;idx++) {
       if (i2c_hits[idx]) {
          if (idx != 0) {
              printf(",");
          }
          printf("0x%02X",i2c_hits[idx]);
       }
    }
    printf("\r\n");
  }
return  nDevices;
}
