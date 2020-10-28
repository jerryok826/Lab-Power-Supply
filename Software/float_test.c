#include <stdio.h>

// gcc -O2 -Wall float_test.c -o float_test

int main()
{
  int data = 18010;
  float reading = 0.0;
  
  reading = (((float)data)/0x7fff) * 4.096;
  
  printf("data: %d, reading: %f\n", data, reading);
  
  reading = 2.251319;
  
  data = 2.251319;
  
  reading = reading - (float)data;
  
  printf("reading: %d.%d\n", data, (int)(reading * 1000000));
  
  return 0;
}
