// http://e2e.ti.com/support/microcontrollers/c2000/f/171/t/787720?CCS-LAUNCHXL-F28379D-Convert-a-Float-to-String-

// Also see:
// https://gist.github.com/Belgarion/80b6a178ac9b000d1d85210f9e08accb

// CCS/LAUNCHXL-F28379D: Convert a Float to String.

#include <stdio.h>
#include <stdint.h>
#include <string.h>

// indent -gnu -br -cli2 -lp -nut -l100 gcvt.c 

// defines for gcvt
#define PSH(X) (*(buf++)=(X))
#define PSH1(X) (*(buf--)=(X))
#define PEEK() buf[-1]
#define POP() *(--buf) = '\0'
#define PLUS 1
#define SPACE 2

// function to convert from float to string
char *
gcvt (float f, uint16_t ndigit, char *buf)
{
  int i;
  uint32_t z, k;
  //int exp = 0;
  char *c = buf;
  float f2, t, scal;
  int sign = 0;

  if ((int) ndigit == -1)
    ndigit = 5;

  /* Unsigned long long only allows for 20 digits of precision
   * which is already more than double supports, so we limit the
   * digits to this.  long double might require an increase if it is ever
   * implemented.
   */
  if (ndigit > 20)
    ndigit = 20;

  if (f < 0.0) {
    sign = 1;
    f = -f;
    buf++;
  }

  scal = 1;
  for (i = ndigit; i > 0; i--)
    scal *= 10;
  k = f + 0.1 / scal;
  f2 = f - k;
  if (!f) {
    PSH ('0');
    if (ndigit > 0)
      PSH ('.');
    for (i = 0; i < ndigit; i++)
      PSH ('0');
    PSH (0);
    return c;
  }

  i = 1;
  while (f >= 10.0) {
    f /= 10.0;
    i++;
  }

  buf += i + ndigit + 1;

  PSH1 (0);

  if (ndigit > 0) {
    t = f2 * scal;
    z = t + 0.5;
    for (i = 0; i < ndigit; i++) {
      PSH1 ('0' + (z % 10));
      z /= 10;
    }
    PSH1 ('.');
  }
  else
    PSH1 (0);

  do {
    PSH1 ('0' + (k % 10));
    k /= 10;
  } while (k);

  if (sign)
    PSH1 ('-');
  return c;
}

void
terminate_str (char *buf, int len)
{
  int i;
  char *str1 = NULL;

  strcat (buf, ".000");         // gcvt() drops trailing zeros

  str1 = strchr (buf, '.');
  if (str1 == NULL) {
    return;
  }

  for (i = 0; i < (len + 1); i++) {
    str1++;
  }
  // terminate string
  *str1 = 0;
}


// ====================================================

#if 0
#define precision 2             // precision for decimal digits

// function to convert float to string
char *
ftos (float num, char *buff)
{
  float f = num;
  char *str = buff;
  int a, b, c, k, l = 0, m, i = 0;

  // check for negative float
  if (f < 0.0) {

    str[i++] = '-';
    f *= -1;
  }

  a = f;                        // extracting whole number
  f -= a;                       // extracting decimal part
  k = precision;

  // number of digits in whole number
  while (k > -1) {
    l = pow (10, k);
    m = a / l;
    if (m > 0) {
      break;
    }
    k--;
  }

  // number of digits in whole number are k+1

  /*
     extracting most significant digit i.e. right most digit , and concatenating to string
     obtained as quotient by dividing number by 10^k where k = (number of digit -1)
   */

  for (l = k + 1; l > 0; l--) {
    b = pow (10, l - 1);
    c = a / b;
    str[i++] = c + 48;
    a %= b;
  }
  str[i++] = '.';

  /* extracting decimal digits till precision */

  for (l = 0; l < precision; l++) {
    f *= 10.0;
    b = f;
    str[i++] = b + 48;
    f -= b;
  }

  str[i] = '\0';

  return str;
}
#endif

#if 0
== == == == == == == == == == == == == == ==
  These are the numbers I am passing to the different functions
  float xa_mag = 1004.0, xa_phase = 45.0, wa_mag = 2546.0, wa_phase = 55.0;
float xb_mag = 324.34, xb_phase = 15.2, wb_mag = 547.0, wb_phase = 10.0;
float xc_mag = 220.22, xc_phase = 30.0, wc_mag = 65.4, wc_phase = 60.0;
float xn_mag = 745.99, xn_phase = 20.0, wn_mag = 66.66, wn_phase = 180.0;

#endif

// gcc -O2 -Wall gcvt.c -o test

int
main ()
{
  char buf[100];
  float voltage = 3.000;

  printf (" Hello\n");
  gcvt (voltage, 3, buf);
  printf (" Hello: %s\n", buf);
  terminate_str (buf, 3);
  printf (" Hello: %s\n", buf);
  return 0;
}
