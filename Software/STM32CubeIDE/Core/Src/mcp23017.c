/* 
 * mcp23017.c, Jerry OKeefe, 3/6/24
 */

#include "main.h"
#include <stdio.h>
#include "mcp23017.h"

// indent -gnu -br -cli2 -lp -nut -l100 mcp23017.c

// Good exmaple
// https://github.com/ruda/mcp23017/blob/master/src/mcp23017.c

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

#define MCP23017_ADDR  (0x20 <<1)      // (n) (0x20|((n)&7))

extern I2C_HandleTypeDef hi2c1; // I2C Control struct

#define DEBUG_LINE printf("ERR: %s %s(%d)\r\n",__FILE__,__func__,__LINE__);

int
mcp23017_write16 (uint8_t reg, uint16_t val16)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[7];

  buf[0] = reg;
  buf[1] = val16 & 0xff;
  buf[2] = val16 >> 8;

  ret = HAL_I2C_Master_Transmit(&hi2c1, MCP23017_ADDR, buf, 3, 1000); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
        printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
        return -1;
  } else {
      return 0;
  }
  return 0;
}

/*
 * Write to MCP23017 A or B register set:
 */
int
mcp23017_write8 (int reg_addr, uint8_t value)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[7];

  buf[0] = reg_addr;
  buf[1] = value;

  ret = HAL_I2C_Master_Transmit(&hi2c1, MCP23017_ADDR, buf, 2, 1000); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
        printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
        return -1;
  } else {
      return 0;
  }
  return 0;
}

int
mcp23017_reg_sel (uint8_t reg_addr)
{
  HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Master_Transmit(&hi2c1, MCP23017_ADDR, &reg_addr, 1, 1000); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
        printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__, ret);
        return -1;
  } else {
    return 0;// IS mcp23017 reset line on B-5 or B-9 ??
// From ohm check it should be  B-9
  }

  return 0;
}

int mcp23017_access_test (void)
{
 int val16=0;
 
 val16 = mcp23017_read16 (GPIO_REG);
 return val16;
}

uint16_t
mcp23017_read16 (uint8_t reg_addr)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[12];
  uint16_t val16 = 0x1234;

	ret = HAL_I2C_Master_Transmit(&hi2c1, MCP23017_ADDR, &reg_addr, 1, 1000); // HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
     printf("%s %s(%d) err: %d\r\n",__FILE__,__func__,__LINE__,ret);
     return 1;
	} else {
	  ret = HAL_I2C_Master_Receive(&hi2c1, MCP23017_ADDR, buf, 2, HAL_MAX_DELAY);
	  if ( ret != HAL_OK ) {
          printf("ERR: %s %s(%d)\r\n",__FILE__,__func__,__LINE__);
          return 1;
	  } else {
	    	  //Combine the bytes
          val16 = (buf[1] << 8) | buf[0];
        }
      }
 return val16;
}

/*
 * Read the MCP23017 input pins (excluding outputs,
 * 16-bits) :
 */
unsigned
mcp23017_inputs (void)
{
  return mcp23017_read16 (GPIO_REG);
}

/*
 * Write 16-bits to outputs :
 */
void
mcp23017_outputs (int value)
{
  mcp23017_write16 (GPIO_REG, value);
}

/*
 * Read MCP23017 captured values (16-bits):
 */
unsigned
mcp23017_captured (void)
{
  return mcp23017_read16 (INTCAP_REG);
}

/*
 * Read interrupting input flags (16-bits):
 */
unsigned
mcp23017_interrupts (void)
{
  return mcp23017_read16 (INTF_REG);
}

void
mcp23017_reset (void)  // Port B pin 5 is mcp23017 reset line
{
//  gpio_set (GPIOB, GPIO5);  // Reset pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  // millisecs_sleep(2);
  HAL_Delay(2);
//  gpio_clear (GPIOB, GPIO5); // Reset pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  // millisecs_sleep(2);
  HAL_Delay(2);
//  gpio_set (GPIOB, GPIO5);  // Reset pin
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(2);  // time for device to recover
}

/*
 * Configure the MCP23017 GPIO Extender :
 */
void
mcp23017_init (int io_dir)
{
  mcp23017_write16 (IOCON_REG,   0b01000100); /* MIRROR=1,ODR=1 */
  mcp23017_write16 (GPINTEN_REG, 0x0000);   /* No interrupts enabled */
  mcp23017_write16 (DEFVAL_REG,  0x0000);   /* Clear default value */
  mcp23017_write16 (OLAT_REG,    0x0000);   /* OLATx=0 */
  mcp23017_write16 (GPPU_REG,    0xFFFF);   // 0b11110000);      /* 4-7 are pullup */
  mcp23017_write16 (IPOL_REG,    0x0000);   /* No inverted polarity */
  mcp23017_write16 (IODIR_REG,   io_dir);   // 0x00ff); // 0x2000);  // 0b11110000); 
  mcp23017_write16 (INTCON_REG,  0x0000);   /* Cmp inputs to previous */
  mcp23017_write16 (GPINTEN_REG, 0xffff);   /* Interrupt on changes */
}
