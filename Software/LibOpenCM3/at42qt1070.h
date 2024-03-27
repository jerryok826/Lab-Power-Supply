

#ifndef __AT42QT1070_H
#define __AT42QT1070_H

// registers
#define CHIP_ID_REG	0x00
#define DET_STATUS_REG	0x02
#define KEY_STATUS_REG	0x03
#define RESET_REG	0x39

void at42qt1070_init (void);
uint8_t at42qt1070_read8 (int reg_addr);
int at42qt1070_write8 (uint8_t reg, uint8_t val8);
uint8_t at42qt1070_chip_id (void);
uint8_t at42qt1070_det_status (void);
uint8_t at42qt1070_key_status (void);
void at42qt1070_show_status (void);

#endif // __AT42QT1070_H
