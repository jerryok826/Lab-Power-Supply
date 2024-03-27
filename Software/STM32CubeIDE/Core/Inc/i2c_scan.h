/* 
 * i2c_scan.h 3/4/24
 */

#ifndef __I2C_SCAN_H
#define __I2C_SCAN_H

#include <stdint.h>

int i2c_device_access_test(uint8_t i2c_adr);
void i2c_scan2(void);
int  i2c_scan (void);

#endif  /* __I2C_SCAN_H */
