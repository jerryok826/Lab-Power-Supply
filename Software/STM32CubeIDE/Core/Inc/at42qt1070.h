

#ifndef __AT42QT1070_H
#define __AT42QT1070_H

#include <stdint.h>
#define CHIP_ID 0x2E

// registers
#define CHIP_ID_REG	0x00
#define CHIP_FRIM_VER_REG 0x01
#define DET_STATUS_REG	0x02
#define KEY_STATUS_REG	0x03
#define CALIBRATE_REG   0x38
#define RESET_REG	0x39
#define AKS_BASE_REG 39 // 0x27

void at42qt1070_init (void);
int at42qt1070_reg_sel (uint8_t reg_addr);
uint8_t at42qt1070_read8 (uint8_t reg_addr);
int at42qt1070_write8 (uint8_t reg, uint8_t val8);
uint8_t at42qt1070_chip_id (void);
uint8_t at42qt1070_det_status (void);
uint8_t at42qt1070_key_status (void);
void at42qt1070_show_status (void);
int at42qt1070_AKS_clear (uint8_t offset);

#endif // __AT42QT1070_H
