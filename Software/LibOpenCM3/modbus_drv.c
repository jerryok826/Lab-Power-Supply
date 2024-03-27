
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include "modbus_drv.h"

#include "main.h"
extern parameters_t param;

// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
// http://unixwiz.net/techtips/termios-vmin-vtime.html
// https://github.com/lambcutlet/DPS5005_pyGUI
// https://www.ls-homeprojects.co.uk/dps3005-psu-module-and-modbus-rtu-python-arduino/

// indent -gnu -br -cli2 -lp -nut -l100 modbus_drv.c
// gcc -O2 -Wall serial.c modbus.c -o serial

int debug = 0;                  // set to one to enable trace logs

#define REG_CNT 40
uint16_t ps_register[REG_CNT];

void
init_registers (void)
{
  memset (ps_register, 0, sizeof (ps_register));

  // Typical value of registers
  ps_register[0] = 723;         // vset, 7.23 volts, ok
  ps_register[1] = 3678;        // iset, 3.678 amps, ok

  ps_register[2] = 528;         // vout, 5.28 v // ok
  ps_register[3] = 1899;        // iout, 1.899 amps, ok

  ps_register[4] = 523;         // power out, show as 1.23 watts, ok
  ps_register[5] = 1278;        // vin, iset show as 12.78, ok

  ps_register[6] = 1;           // lock button,
  ps_register[7] = 0;           // protection, 0=ok, 1=OVP, 2=OCP, 3=OPP

  ps_register[8] = 0;           // mode, CC == 1 or CV == 0 // ok
  ps_register[9] = 1;           // on/off, 1 = on, 0 = off; // ok

  ps_register[11] = 15;         // model, show as 15 // ok
  ps_register[12] = 13;         // version, show as 1.3 // ok
}

static int
get_register (int reg)
{
  switch (reg) {
    case 0:
      return (param.v_set * 100);       // reg 0
    case 1:
      return (param.i_set * 1000);      // reg 1
    case 2:
      return (param.v_out * 100);       // reg 2
    case 3:
      return (param.i_out * 1000);      // reg 3
    case 5:
      return (param.v_in * 100);        // reg 5
    case 9:
      return (param.out_on);    // reg 9
    default:
      return -1;
  }
  return -1;
}

#if 0
static int
put_register (int reg, uint16_t val16)
{
  switch (reg) {
    case 0:
      param.v_set = val16 / 100.0;      // reg 0
      return 0;
    case 1:
      param.i_set = val16 / 1000.0;     // reg 1
      return 0;
    case 2:
      param.v_out = val16 / 100.0;      // reg 2
      return 0;
    case 3:
      param.i_out = val16 / 1000.0;     // reg 3
      return 0;
    case 5:
      param.v_in = val16 / 100.0;       // reg 5
      return 0;
    case 9:
      param.out_on = val16;     // reg 9
      return 0;
    default:
      return -1;
  }
  return -1;
}
#endif

typedef struct _modbus_frame
{
  uint8_t slave_adr;
  uint8_t function;
  uint16_t reg_nub;
  uint16_t nub_reg;
  uint8_t byte_cnt;             // number bytes in register list

  uint8_t checksum;
} modbus_frame_t;

// Convert string packet into raw bytes and do verify checksum
static int
modbus_rx_deframe (char *buf_in, uint8_t * buf_out)
{
  int i = 0;
  int pkt_len = 0;
  uint8_t sum = 0;
  int val8;
  int len = 0;
  char str[3];

  // check start of frame
  if (buf_in[0] != ':') {
    printf ("No start of frame error: 0x%02X\n", buf_in[0]);
    return -1;
  }

  len = strlen (buf_in + 1);
  for (i = 1; i < (len);) {
    if (iscntrl ((int)buf_in[i])) {
      break;                    // check for \r \n 
    }
    str[0] = buf_in[i]; 
    str[1] = buf_in[i + 1];
    str[2] = 0;
    // Convert char to bytes
    val8 = strtol (str, NULL, 16);
    sum += val8;
    buf_out[i / 2] = val8;
    i += 2;
    pkt_len++;
  }
  // check sum should be zero
  if (sum != 0) {
    return -2;
  }
  return pkt_len;
}

// MODBUS supported types
#define WRT_BLK_REGS 0x10
#define WRT_REG  0x06
#define RD_BLK_REGS  0x03

// decode byte array to packet struct
static int
modbus_rx_pkt (uint8_t * buf_in, modbus_frame_t * rx_pkt)
{
  int rx_pkt_len = 0;
  uint16_t val16 = 0;
  int k = 0;
  uint16_t reg_nub = 0;

  memset (rx_pkt, 0, sizeof (modbus_frame_t));
  rx_pkt->slave_adr = buf_in[0];
  rx_pkt->function = buf_in[1];
  switch (rx_pkt->function) {
    case WRT_BLK_REGS:             // write registers, 0x10
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      rx_pkt->nub_reg = buf_in[4] << 8 | buf_in[5];
      rx_pkt->byte_cnt = buf_in[6];
      rx_pkt_len = rx_pkt->byte_cnt + 7 + 1;    // & is head bytes, 1 is checksum byte

      /* Write set data to registers */
      reg_nub = rx_pkt->reg_nub;
      for (k = 0; k < rx_pkt->byte_cnt;) {
        ps_register[reg_nub] = buf_in[7 + k] << 8 | buf_in[8 + k];
        val16 = ps_register[reg_nub];
        if (reg_nub == 0) {
          /* Process VSET command */
          float voltage = (float) val16 / 100.0;
          remote_voltage_preset(voltage);
        }
        if (reg_nub == 1) {
          /* Process VSET command */
          float current = (float) val16 / 100.0;
          remote_current_preset(current); 
        }
        if (reg_nub == 9) {
          /* Process OnOff command */
          on_off_contrl (val16 & 1);    // This works!
        }
        k += 2;
      }
      break;
    case WRT_REG:              // single write register, 0x06
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      // Add pkt->reg_nub range check
      rx_pkt_len = rx_pkt->byte_cnt + 7 + 1;    // & is head bytes, 1 is checksum byte

      /* Write set data to registers */
      ps_register[rx_pkt->reg_nub] = buf_in[7] << 8 | buf_in[8];

      /* Process OnOff command */
      val16 = ps_register[rx_pkt->reg_nub];
      if (rx_pkt->reg_nub == 9) {
        //        on_off_contrl(val16 & 1);
      }
#if 0
      if (rx_pkt->reg_nub == 0) {
        /* Process VSET command */
        float voltage = (float) val16 / 200.0; // why 200
        remote_voltage_preset(voltage);
        remote_current_preset(1.3);	 // need to get this value
      }
#endif
      break;
    case RD_BLK_REGS:              // read registers, 0x03
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      rx_pkt->nub_reg = buf_in[4] << 8 | buf_in[5];
      rx_pkt->checksum = buf_in[6];
      rx_pkt_len = 7;
      break;
  }
  return rx_pkt_len;
}

// Deframe and decode MODBUS packet request.
static int
modbus_rx_pkt_decode (char *str, modbus_frame_t * rx_pkt)
{
  uint8_t rx_buf[100];
  int pkt_len = 0;

  pkt_len = modbus_rx_deframe (str, rx_buf);
  if (pkt_len > 0) {
    // decode byte array to packet struct
    pkt_len = modbus_rx_pkt (rx_buf, rx_pkt);
  }
  return pkt_len;
}

  // Encode MODBUS response packet into a ASCII string
static int
modbus_tx_frame_pkt (uint8_t * buf_in, int buf_in_len, char *buf_out)
{
  int i = 0;
  int k = 1;
  uint8_t sum = 0;

  buf_out[0] = ':';             // mark start of frame
  for (i = 0; i < buf_in_len; i++) {
    snprintf (&buf_out[k], buf_in_len, "%02X", buf_in[i]);
    sum += buf_in[i];
    k += 2;
  }
  snprintf (&buf_out[k], buf_in_len, "%02X\r\n%c", 256 - sum, 0);       // CRC
  return strlen (buf_out);
}


// Build response packet from request packet
static int
modbus_tx_response_pkt (modbus_frame_t * pkt, char *buf_in, int buf_len)
{
  int i = 0;
  int pkt_len = 0;
  int reg_nub = pkt->reg_nub;
  int reg_cnt = pkt->nub_reg;   // use reg_cnt Fix me
  int16_t val16 = 0;
  uint8_t tx_buf[100];

  memset (buf_in, 0, buf_len);
  tx_buf[0] = pkt->slave_adr;   // Slave Address
  tx_buf[1] = pkt->function;    // Function
  switch (pkt->function) {
    case WRT_BLK_REGS:             // block write register, 0x10
      tx_buf[2] = ((pkt->reg_nub & 0xFF) >> 8) & 0xFF;
      tx_buf[3] = pkt->reg_nub & 0xFF;
      tx_buf[4] = ((pkt->nub_reg & 0xFF) >> 8) & 0xFF;
      tx_buf[5] = pkt->nub_reg & 0xFF;
      pkt_len = 6;
      break;
    case WRT_REG:              // single write register, 0x06
      tx_buf[2] = ((pkt->reg_nub & 0xFF) >> 8) & 0xFF;
      tx_buf[3] = pkt->reg_nub & 0xFF;
      tx_buf[4] = ((pkt->nub_reg & 0xFF) >> 8) & 0xFF;
      tx_buf[5] = pkt->nub_reg & 0xFF;
      pkt_len = 6;
      break;
    case RD_BLK_REGS:              // read register, 0x03
      // reload registers
      ps_register[0] = param.v_set * 100;
      ps_register[1] = param.i_set * 1000;
      ps_register[2] = param.v_out * 100;
      ps_register[3] = param.i_out * 1000;
      ps_register[5] = param.v_in * 100;
      ps_register[9] = param.out_on;
      
      // form response packet
      tx_buf[2] = reg_cnt * 2;  // 12          // byte_cnt 
      for (i = 0; i < reg_cnt; i++) {
        val16 = get_register (reg_nub + i);
        tx_buf[3 + i * 2] = (val16 >> 8) & 0Xff;
        tx_buf[4 + i * 2] = val16 & 0xFF;

//        tx_buf[3 + i * 2] = (ps_register[i + reg_nub] >> 8) & 0Xff;
//        tx_buf[4 + i * 2] = ps_register[i + reg_nub] & 0xFF;
      }
      pkt_len = i * 2 + 3;      // 5; // (pkt->nub_reg*2) + 3 +1; // was 5
      break;
    default:
      return -1;
      break;
  }

  // encode MODBUS packet
  pkt_len = modbus_tx_frame_pkt (tx_buf, pkt_len, buf_in);
  return pkt_len;
}

int
modbus_process (char *rx_buf, int mosbua_pkt_len, char *tx_buf)
{
  modbus_frame_t rx_pkt;
  int response_len = 0;
  int rc = 0;
  int tx_len = 0;

  if (mosbua_pkt_len > 0) {
    printf ("RX %d: %s", mosbua_pkt_len, rx_buf);

    rc = modbus_rx_pkt_decode (rx_buf, &rx_pkt);
    if (rc > 0) {
      // Build response packet from request packet
      tx_len = modbus_tx_response_pkt (&rx_pkt, tx_buf, sizeof (tx_buf));
      printf ("TX %d: %s\n", response_len, tx_buf);
    }
  }
  return tx_len;
}
