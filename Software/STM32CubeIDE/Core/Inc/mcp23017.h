

#ifndef __MCP23017_H
#define __MCP23017_H

#include <stdint.h>

// registers
// IOCON.BANK = 0
#define IODIR_REG	0x00
#define IPOL_REG	0x02
#define GPINTEN_REG	0x04
#define DEFVAL_REG	0x06
#define INTCON_REG	0x08
#define IOCON_REG	0x0A
#define GPPU_REG	0x0C
#define INTF_REG	0x0E
#define INTCAP_REG	0x10
#define GPIO_REG	0x12
#define OLAT_REG	0x15

int mcp23017_write(int reg,int value);
int mcp23017_reg_sel(uint8_t reg_addr); 

// void mcp23017_write_both(int reg,int value); 
unsigned mcp23017_inputs(void); 
void mcp23017_outputs(int value); 
unsigned mcp23017_captured(void); 
unsigned mcp23017_interrupts(void);
void mcp23017_init(int io_dir);
// void post_outputs(void); 
int mcp23017_write16 (uint8_t reg, uint16_t reg_val);
uint16_t mcp23017_read16(uint8_t reg_addr); 
int mcp23017_access_test (void);
void mcp23017_reset (void);  // Port b pin 5 is mcp23017 reset line

#endif // __MCP23017_H

