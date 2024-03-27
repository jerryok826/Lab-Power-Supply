#ifndef __MCP9808_H
#define __MCP9808_H

#define MCP9808_ADDR(n)	 (0x18|((n)&7))

void mcp9808_init (void);
int mcp9808_read_tens_deg (void);
float mcp9808_read_float (void);

#endif  /* __MCP9808_H */
