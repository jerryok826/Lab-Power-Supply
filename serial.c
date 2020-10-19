#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>

// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
// http://unixwiz.net/techtips/termios-vmin-vtime.html
// https://github.com/lambcutlet/DPS5005_pyGUI
// https://www.ls-homeprojects.co.uk/dps3005-psu-module-and-modbus-rtu-python-arduino/

// indent -gnu -br -cli2 -lp -nut -l100 serial.c
// gcc -O2 serial.c -o serial

char modbus_buf[100];
int debug = 0;


typedef struct
{
  int rate;
  int flag;
} speed_spec;

int
get_baudrate_flag (int baudrate)
{
  int flag = 0;
  speed_spec speeds[] = {
    {1200, B1200},
    {2400, B2400},
    {4800, B4800},
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
    {57600, B57600},
    {115200, B115200},
    {230400, B230400},
    {0, 0}
  };

  speed_spec *s;
  for (s = speeds; s->rate; s++) {
    if (baudrate == s->rate) {
      flag = s->flag;
      break;
    }
  }
  return flag;
}

int
set_interface_attribs (int fd, int speed)
{
  struct termios tty;

  if (tcgetattr (fd, &tty) < 0) {
    printf ("Error from tcgetattr: %s\n", strerror (errno));
    return -1;
  }

  cfsetospeed (&tty, (speed_t) speed);
  cfsetispeed (&tty, (speed_t) speed);

  tty.c_cflag |= (CLOCAL | CREAD);      /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;           /* 8-bit characters */
  tty.c_cflag &= ~PARENB;       /* no parity bit */
  tty.c_cflag &= ~CSTOPB;       /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;      /* no hardware flowcontrol */


  /* setup for non-canonical mode */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  /* fetch bytes as they become available */
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;          // 1;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf ("Error from tcsetattr: %s\n", strerror (errno));
    return -1;
  }
  return 0;
}

void
set_mincount (int fd, int mcount)
{
  struct termios tty;

  if (tcgetattr (fd, &tty) < 0) {
    printf ("Error tcgetattr: %s\n", strerror (errno));
    return;
  }

  tty.c_cc[VMIN] = mcount ? 1 : 0;
  tty.c_cc[VTIME] = 5;          // 5;          /* half second timer */

  if (tcsetattr (fd, TCSANOW, &tty) < 0)
    printf ("Error tcsetattr: %s\n", strerror (errno));
}

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

  ps_register[4] = 523;         // pot, show as 1.23 watts, ok
  ps_register[5] = 1278;        // vin, iset show as 12.78, ok

  ps_register[6] = 1;           // lock button,
  ps_register[7] = 0;           // protection, 0=ok, 1=OVP, 2=OCP, 3=OPP

  ps_register[8] = 0;           // mode, CC == 1 or CV == 0 // ok
  ps_register[9] = 1;           // on/off, 1 = on, 0 = off; // ok

  ps_register[11] = 15;         // model, show as 15 // ok
  ps_register[12] = 13;         // version, show as 1.3 // ok
}

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
int
modbus_rx_deframe (char *buf_in, uint8_t * buf_out, int buf_out_len)
{
  int i = 0;
  int pkt_len = 0;
  uint8_t sum = 0;
  uint8_t a = 0;
  uint8_t b;
  int val8;
  int len = 0;
  char str[3];

  // check start of frame
  if (buf_in[0] != ':') {
    printf ("Start of frame error\n");
    return -1;
  }

  len = strlen (buf_in + 1);
  for (i = 1; i < (len);) {
    if (iscntrl (buf_in[i])) {
      break;                    // check for \r \n 
    }
    str[0] = buf_in[i];
    str[1] = buf_in[i + 1];
    str[2] = 0;
    val8 = strtol (str, NULL, 16);
    sum += val8;
    buf_out[i / 2] = val8;
    i += 2;
    pkt_len++;
  }
  if (sum != 0) {
    return -2;
  }
  return pkt_len;
}

#define WRT_REGS 0x10
#define WRT_REG  0x06
#define RD_REGS  0x03

// decode byte array to packet struct
int
modbus_rx_pkt (char *buf_in, int buf_len, modbus_frame_t * rx_pkt)
{
  int rx_pkt_len = 0;

  memset(rx_pkt, 0, sizeof(modbus_frame_t));
  rx_pkt->slave_adr = buf_in[0];
  rx_pkt->function = buf_in[1];
  switch (rx_pkt->function) {
    case WRT_REGS:             // write registers, 0x10
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      rx_pkt->nub_reg = buf_in[4] << 8 | buf_in[5];
      rx_pkt->byte_cnt = buf_in[6] << 8;
      rx_pkt_len = rx_pkt->byte_cnt + 7 + 1;  // & is head bytes, 1 is checksum byte
      break;
    case WRT_REG:              // write register, 0x06
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      // Add pkt->reg_nub range check
      ps_register[rx_pkt->reg_nub] = buf_in[4] << 8 | buf_in[5];
      rx_pkt_len = rx_pkt->byte_cnt + 7 + 1;  // & is head bytes, 1 is checksum byte
      break;
    case RD_REGS:              // read registers, 0x03
      rx_pkt->reg_nub = buf_in[2] << 8 | buf_in[3];
      rx_pkt->nub_reg = buf_in[4] << 8 | buf_in[5];
      rx_pkt->checksum = buf_in[6];
      rx_pkt_len = 7;
      break;
  }
  return rx_pkt_len;
}

#if 0
      pkt_len = modbus_rx_deframe (buf, buf_out, sizeof (buf_out));
      if (pkt_len > 0) {
        // decode byte array to packet struct
        modbus_rx_pkt (buf_out, sizeof (buf_out), &rx_pkt);

int modbus_rx_pkt_decode(char *str, modbus_frame_t * rx_pkt) 
{
   uint8_t rx_buf[100];
   
      byte_len = modbus_rx_deframe (str, rx_buf, sizeof (rx_buf));
      if (byte_len > 0) {
        // decode byte array to packet struct
        modbus_rx_pkt (rx_buf, sizeof (rx_buf), &rx_pkt);
      }
}

 ////////////////////////////////////////
        // Build response packet from request packet
        response_len = modbus_tx_response_pkt (&rx_pkt, buf1, sizeof (buf1));

        response_len = modbus_tx_frame_pkt (buf1, response_len, buf3, sizeof (buf3));
        wrt_cnt = write (fd, buf3, response_len);

        // Build response packet from request packet
        response_len = modbus_build_tx_frame(&tx_pkt, char str);
        wrt_cnt = write (fd, buf3, response_len);
        printf ("TX %d: %s\n", response_len, buf3);
     

int modbus_rx_pkt_decode(char *str) 
        wrt_cnt = write (fd, buf3, response_len);
#endif

// Build response packet from request packet
int
modbus_tx_response_pkt (modbus_frame_t * pkt, char *buf_in, int buf_len)
{
  uint8_t ck_sum = 0;
  int i = 0;
  int pkt_len = 0;
  int reg_cnt = pkt->nub_reg;

  memset (buf_in, 0, buf_len);
  buf_in[0] = pkt->slave_adr;   // Slave Address
  buf_in[1] = pkt->function;    // Function
  switch (pkt->function) {
    case WRT_REGS:             // write register, 0x10
      buf_in[2] = ((pkt->reg_nub & 0xFF) >> 8) & 0xFF;
      buf_in[3] = pkt->reg_nub & 0xFF;
      buf_in[4] = ((pkt->nub_reg & 0xFF) >> 8) & 0xFF;
      buf_in[5] = pkt->nub_reg & 0xFF;
      pkt_len = 6;
      break;
    case WRT_REG:              // write register, 0x06
      buf_in[2] = ((pkt->reg_nub & 0xFF) >> 8) & 0xFF;
      buf_in[3] = pkt->reg_nub & 0xFF;
      buf_in[4] = ((pkt->nub_reg & 0xFF) >> 8) & 0xFF;
      buf_in[5] = pkt->nub_reg & 0xFF;
      pkt_len = 6;
      break;
    case RD_REGS:              // read register, 0x03
      buf_in[2] = reg_cnt * 2;  // 12          // byte_cnt 
      for (i = 0; i < reg_cnt; i++) {
        buf_in[3 + i * 2] = (ps_register[i] >> 8) & 0Xff;
        buf_in[4 + i * 2] = ps_register[i] & 0xFF;
      }
      pkt_len = i * 2 + 3;      // 5; // (pkt->nub_reg*2) + 3 +1; // was 5
      break;
  }
  return pkt_len;
}

 // Encode response packet ino ASCII string
int
modbus_tx_frame_pkt (uint8_t * buf_in, int buf_in_len, char *buf_out, int buf_out_len)
{
  static char hex_tab[] =
    { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

  int i = 0;
  int k = 1;
  uint8_t sum = 0;
  uint8_t a;

  memset (buf_out, 0, buf_out_len);
  buf_out[0] = ':';             // mark start of frame
  for (i = 0; i < buf_in_len; i++) {
    a = (buf_in[i] >> 4) & 0xF;
    buf_out[k++] = hex_tab[a];
    a = buf_in[i] & 0xf;
    buf_out[k++] = hex_tab[a];
    sum += buf_in[i];
  }
  snprintf (&buf_out[k], buf_in_len, "%02X\r\n", 256 - sum);    // CRC
  return strlen (buf_out);
}

int
main (int argc, char *argv[])
{
  char portname[100] = { "/dev/ttyUSB0" };
  int baudrate_flag = B9600;    // B115200;
  int baudrate = 9600;
  int fd;
  int wlen;
  int rc = 0;
  int pkt_len = 0;
  int i;

  init_registers ();

  if (argc < 2) {
    fprintf (stderr, "example: %s /dev/ttyS0 [115200]\n", argv[0]);
    exit (1);
  }

  for (i = 0; i < argc; i++) {
    printf ("argv[%i]: %s\n", i, argv[i]);
  }

  if (argv[1][0] == '/') {
    strcpy (portname, argv[1]);
  }

  if ((argc > 2) && isdigit (argv[2][0])) {
    baudrate = atoi (&argv[2][0]);
  }

  baudrate_flag = get_baudrate_flag (baudrate);
  if (!baudrate_flag) {
    printf ("baudrate error: %d\n", baudrate);
    exit (0);
  }

  fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf ("Error opening %s: %s\n", portname, strerror (errno));
    return -1;
  }
  printf ("%s opened to: %s, Baud: %d\n", argv[0], portname, baudrate); // see_speed (baudrate));

  /*baudrate 115200, 8 bits, no parity, 1 stop bit */
  set_interface_attribs (fd, baudrate_flag);
  //set_mincount(fd, 0);    /* set to pure timed read */

  /* simple noncanonical input */
  do {
    unsigned char buf[200];     // 80];
    char *ptr;
    int mosbua_pkt_len = 0;
    int rd_cnt = 0;

    mosbua_pkt_len = 0;
    ptr = buf;
    memset (buf, 0, sizeof (buf));

    // Read until we have a complete line
    do {
      rd_cnt = read (fd, ptr, sizeof (buf) - 1);
      ptr += rd_cnt;
      mosbua_pkt_len += rd_cnt;
    } while (strchr (buf, '\n') == NULL);

    if (mosbua_pkt_len > 0) {

      printf ("RX %d: %s", mosbua_pkt_len, buf);

      modbus_frame_t rx_pkt;
      char buf1[100];
      char buf3[100];
      char buf_out[100];
      int response_len;
      int wrt_cnt = 0;

      // Convert string packet into raw bytes and verify checksum
      pkt_len = modbus_rx_deframe (buf, buf_out, sizeof (buf_out));
      if (pkt_len > 0) {
        // decode byte array to packet struct
        modbus_rx_pkt (buf_out, sizeof (buf_out), &rx_pkt);

        // Build response packet from request packet
        response_len = modbus_tx_response_pkt (&rx_pkt, buf1, sizeof (buf1));

        response_len = modbus_tx_frame_pkt (buf1, response_len, buf3, sizeof (buf3));
        wrt_cnt = write (fd, buf3, response_len);
        printf ("TX %d: %s\n", response_len, buf3);
      }
    }
    else if (mosbua_pkt_len < 0) {
      printf ("Error from read: %d: %s\n", mosbua_pkt_len, strerror (errno));
    }
    else {                      /* rdlen == 0 */
      printf ("Timeout from read\n");
    }
    /* repeat read to get full message */
  }
  while (1);
}
