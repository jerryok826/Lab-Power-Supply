#define LIGHT_SENSOR_ADDR(n)	(0x10|((n)&7))
#define MCP9808_ADDR(n)	 (0x18|((n)&7))
#define MCP4728_ADDR(n)	 (0x60|((n)&7))
#define MCP23017_ADDR(n) (0x20|((n)&7))
#define INA219_ADDR(n)   (0x40|((n)&7))


void
mcp4728_write (uint8_t dac, uint16_t dac_val)
{
  uint8_t addr = MCP4728_ADDR (0);      // I2C Address
  uint8_t buf[5];

  buf[0] = 0x58 | ((dac & 0x03) << 1);
  buf[1] = ((dac_val >> 8) & 0x0F) | 0x80;      // ref bit enabled
  buf[2] = dac_val & 0xFF;

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, buf[0]);
  i2c_write (&i2c, buf[1]);
  i2c_write (&i2c, buf[2]);
  i2c_stop (&i2c);
}

// #define ADS1115_ADDR(n) (0x48|((n)&7)) // For addr pin GND
#define ADS1115_ADDR(n) (0x49|((n)&7))  // For addr pin to VDD

int
ads1115_init (void)
{
  uint8_t addr = ADS1115_ADDR (0);      // I2C Address
  int ch = 0;
  int range = 1;                // +-4.096V

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x01);       // config reg
//  i2c_write (&i2c, 0x84); // to set the ADS111x to continuous-conversion mode 
//  i2c_write (&i2c, 0x83);

//  i2c_write (&i2c, 0xc4 | ((ch & 0x3)<<4));
//  i2c_write (&i2c, 0xc2); //  | ((ch & 0x3) | 0x4) << 4);
  i2c_write (&i2c, 0x80 | (((ch & 0x3) | 0x4) << 4) | (range << 1));
  i2c_write (&i2c, 0x83);
  i2c_stop (&i2c);

  return 0;
}

int
ads1115_read (uint8_t ch, uint16_t * data)
{
  uint8_t addr = ADS1115_ADDR (0);      // I2C Address
  uint8_t buf[10];
  int val32;

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x00);       // point to conversion reg
  i2c_stop (&i2c);

  buf[0] = 0x01;
  buf[1] = 0x23;
  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

//  std_printf ("%s(%d) 0x%02X 0x%02X\n", __func__, __LINE__, buf[0], buf[1]);
  *data = (buf[0] << 8) | buf[1];
//  std_printf ("%s(%d) %d, 0x%04X\n", __func__, __LINE__, *data, *data);
//  std_printf ("%s(%d) %f\n", __func__, __LINE__, *data);

  val32 = *data;
  float reading = ((float) val32 / (float) 0x7fff) * 4.096;

//  float_str (reading_str, sizeof (reading_str), reading);
  gcvt(reading, 6, reading_str); 

  return 0;
}

// #define ADS1115_ADDR(n) (0x48|((n)&7)) // For addr pin GND
#define ADS1115_ADDR(n) (0x49|((n)&7))  // For addr pin to VDD

int
ads1115_init (void)
{
  uint8_t addr = ADS1115_ADDR (0);      // I2C Address
  int ch = 0;
  int range = 1;                // +-4.096V

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x01);       // config reg
//  i2c_write (&i2c, 0xc4 | ((ch & 0x3)<<4));
//  i2c_write (&i2c, 0xc2); //  | ((ch & 0x3) | 0x4) << 4);
  i2c_write (&i2c, 0x80 | (((ch & 0x3) | 0x4) << 4) | (range << 1));
  i2c_write (&i2c, 0x83);
  i2c_stop (&i2c);

  return 0;
}

int
ads1115_read (uint8_t ch, uint16_t * data)
{
  uint8_t addr = ADS1115_ADDR (0);      // I2C Address
  uint8_t buf[10];
  int val32;

  i2c_start_addr (&i2c, addr, Write);
  i2c_write (&i2c, 0x00);       // point to conversion reg
  i2c_stop (&i2c);

  buf[0] = 0x01;
  buf[1] = 0x23;
  i2c_start_addr (&i2c, addr, Read);
  i2c_read_bytes (&i2c, buf, 2);
  i2c_stop (&i2c);

  *data = (buf[0] << 8) | buf[1];
  val32 = *data;
  float reading = ((float) val32 / (float) 0x7fff) * 4.096;

  gcvt(reading, 6, reading_str); 

  return 0;
}
