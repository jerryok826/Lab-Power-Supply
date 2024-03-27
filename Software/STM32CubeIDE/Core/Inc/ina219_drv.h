

#ifndef __INA219_DRV_H
#define __INA219_DRV_H
#include <stdint.h>

void ina219_init (void);
void ina219_reset (void);
int ina219_access_test (void);
int ina219_reg_read (uint8_t reg, uint8_t local_debug);
int ina219_reg_write (uint8_t reg, uint16_t val16);
float voltage_read (void);
float current_read (void);

#endif // __INA219_DRV
