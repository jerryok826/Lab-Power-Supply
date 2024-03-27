#ifndef __MCP9808_H
#define __MCP9808_H
#include <stdint.h>

void mcp9808_init (void);
uint16_t mcp9808_reg_read (int reg);
float  mcp9808_read (void);

#endif  /* __MCP9808_H */
