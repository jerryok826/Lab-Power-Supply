/*********************************************************************
 * mcp23017.c : Interface with MCP23017 I/O Extender Chip
 *
 *
 *********************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "i2c.h"
#include "mcp23017.h"
#include "main.h"

// indent -gnu -br -cli2 -lp -nut -l100 mcp23017.c

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

// #define MCP_REGISTER(r,g) (((r)<<1)|(g))        /* For I2C routines */
#define MCP23017_ADDR(n) (0x20|((n)&7))

// static unsigned gpio_addr = 0x20;    /* MCP23017 I2C Address */
// static const int gpio_inta = 17;     /* GPIO pin for INTA connection */
// static int is_signaled = 0;     /* Exit program if signaled */

extern I2C_Control i2c_handle;    // I2C Control struct

void
mcp23017_write16 (uint8_t reg, uint16_t reg_val)
{
  uint8_t addr = MCP23017_ADDR (0);     // I2C Address

  mutex_lock();  
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg);
//  i2c_write (&i2c_handle, reg_val >> 8);
  i2c_write (&i2c_handle, reg_val & 0xff);
  i2c_write (&i2c_handle, reg_val >> 8);
  i2c_stop (&i2c_handle);
  mutex_unlock();
}

/*
 * Write to MCP23017 A or B register set:
 */
int
mcp23017_write (int reg_addr, int value)
{
  int rc = 0;
//  unsigned reg_addr = MCP_REGISTER (reg, AB);
  uint8_t addr = MCP23017_ADDR (0);     // I2C Address

  mutex_lock();  
  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg_addr);
  i2c_write (&i2c_handle, value);
  i2c_stop (&i2c_handle);
  mutex_unlock();

  return rc;
}

int
mcp23017_reg_sel (int reg_addr)
{
  uint8_t addr = MCP23017_ADDR (0);     // I2C Address

  i2c_start_addr (&i2c_handle, addr, Write);
  i2c_write (&i2c_handle, reg_addr);
  i2c_stop (&i2c_handle);

  return 0;
}

int mcp23017_access_test (void)
{
 int val16=0;
 
 val16 = mcp23017_read16 (GPIO_REG);
 return val16;
}


uint16_t
mcp23017_read16 (int reg_addr)
{
//  return i2c_read16(gpio_addr,reg_addr) & 0xF0F0;
  uint8_t buf[10];

  uint8_t addr = MCP23017_ADDR (0);     // I2C Address
  mutex_lock();  
  mcp23017_reg_sel (reg_addr);

  i2c_start_addr (&i2c_handle, addr, Read);
  i2c_read_bytes (&i2c_handle, buf, 2);
  i2c_stop (&i2c_handle);
  mutex_unlock();

//  return (buf[0] << 8) | buf[1];
  return (buf[1] << 8) | buf[0];
}

/*
 * Read the MCP23017 input pins (excluding outputs,
 * 16-bits) :
 */
unsigned
mcp23017_inputs (void)
{
//  return i2c_read16(gpio_addr,reg_addr) & 0xF0F0;

//  unsigned reg_addr = GPIO_REG; // MCP_REGISTER (GPIO_REG, GPIOA_BANK);
//  return mcp23017_read16 (GPIO_REG);
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
//      return i2c_read16(gpio_addr,reg_addr) & 0xF0F0;

//  unsigned reg_addr = INTCAP_REG; // MCP_REGISTER (INTCAP_REG, GPIOA_BANK);
//  return mcp23017_read16 (INTCAP_REG) & 0xF0F0;
  return mcp23017_read16 (INTCAP_REG);
}

/*
 * Read interrupting input flags (16-bits):
 */
unsigned
mcp23017_interrupts (void)
{
//      return i2c_read16(gpio_addr,reg_addr) & 0xF0F0;
//  unsigned reg_addr = INTF_REG; // MCP_REGISTER (INTF_REG, GPIOA_BANK);
//  return mcp23017_read16 (INTF_REG) & 0xF0F0;
  return mcp23017_read16 (INTF_REG);
}

void
mcp23017_reset (void)  // Port b pin 5 is mcp23017 reset line
{
  gpio_set (GPIOB, GPIO5);  // Reset pin
  millisecs_sleep(2);
  gpio_clear (GPIOB, GPIO5); // Reset pin
  millisecs_sleep(2);
  gpio_set (GPIOB, GPIO5);  // Reset pin
}

/*
 * Configure the MCP23017 GPIO Extender :
 */
void
mcp23017_init (int io_dir)
{
//  int v, int_flags;

  mcp23017_write16 (IOCON_REG, 0b01000100); /* MIRROR=1,ODR=1 */
  mcp23017_write16 (GPINTEN_REG, 0x0000);     /* No interrupts enabled */
  mcp23017_write16 (DEFVAL_REG,  0x0000);      /* Clear default value */
  mcp23017_write16 (OLAT_REG,    0x0000);        /* OLATx=0 */
  mcp23017_write16 (GPPU_REG,    0xFFFF);      // 0b11110000);      /* 4-7 are pullup */
  mcp23017_write16 (IPOL_REG,    0x00000); /* No inverted polarity */
  mcp23017_write16 (IODIR_REG,   io_dir); // 0x00ff); // 0x2000);  // 0b11110000); 
  mcp23017_write16 (INTCON_REG,  0x0000);   /* Cmp inputs to previous */
  mcp23017_write16 (GPINTEN_REG, 0xffff);  /* Interrupt on changes */

  /*
   * Loop until all interrupts are cleared:
   */
#if 0
  do {
    int_flags = mcp23017_interrupts ();
    if (int_flags != 0) {
      v = mcp23017_captured ();
//                      printf("  Got change %04X values %04X\n",int_flags,v);
    }
  } while (int_flags != 0x0000 && !is_signaled);
#endif
}

/*
 * Copy input bit settings to outputs :
 */
#if 0
void
post_outputs (void)
{
  int inbits = mcp23017_inputs ();      /* Read inputs */
  int outbits = inbits >> 4;    /* Shift to output bits */
  mcp23017_outputs (outbits);   /* Copy inputs to outputs */
//      printf("  Outputs:      %04X\n",outbits);
}
#endif

/*
 * Main program :
 */
#if 0
int
main (int argc, char **argv)
{
  int int_flags, v;
  int fd;

  signal (SIGINT, sigint_handler);      /* Trap on SIGINT */

  i2c_init (node);              /* Initialize for I2C */
  mcp23017_init ();             /* Configure MCP23017 @ 20 */

  fd = gpio_open_edge (gpio_inta);      /* Configure INTA pin */

  puts ("Monitoring for MCP23017 input changes:\n");
  post_outputs ();              /* Copy inputs to outputs */

  do {
    gpio_poll (fd);             /* Pause until an interrupt */

    int_flags = mcp23017_interrupts ();
    if (int_flags) {
      v = mcp23017_captured ();
//  printf("  Input change: flags %04X values %04X\n", int_flags,v);
      post_outputs ();
    }
  } while (!is_signaled);       /* Quit if ^C'd */

  fputc ('\n', stdout);

  i2c_close ();                 /* Close I2C driver */
  close (fd);                   /* Close gpio17/value */
  gpio_close (gpio_inta);       /* Unexport gpio17 */
  return 0;
}
#endif

/*********************************************************************
 * End mcp23017.c - Warren Gay
 * Mastering the Raspberry Pi - ISBN13: 978-1-484201-82-4
 * This source code is placed into the public domain.
 *********************************************************************/
