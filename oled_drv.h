
#ifndef __OLED_DRV_H
#define __OLED_DRV_H

void oled_init_real(void); 
void oled_update(void);
void oled_init (void);
void oled_init_ssd1322 (void); // SSD1322 256X64
void oled_ssd1322_fill(uint8_t fill_byte);
void oled_ssd1322_clear(void);
void oled_display_off(void);
void oled_reset(int reset);

void clear_pixmap (void);

void oled_update_ssd1322(void);
void load_stars_pixmap(void);
void oled_command(uint8_t byte);

void oled_data(uint8_t byte);
void oled_data2(uint8_t *pByte, uint8_t len);
void oled_data3(uint8_t byte, uint8_t len);
void oled_data4 (uint16_t word, uint8_t len);

int spi_1_init(void);
int spi_2_init(void);

#endif // __OLED_DRV_H

