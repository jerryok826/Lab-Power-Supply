#include <stdio.h>
#include <stdlib.h>
#include <string.h>
  
// char *gcvt(double x, int ndigit, char *buf);

// gcc -Wall -O2 test.c -o test

void terminate_str(char *buf, int len)
{
  int i;
  char *str1 = NULL;
  
  str1 = strchr(buf, '.');
  for (i=0;i<(len+1); i++) {
     str1++;
  }
  // terminate string
  *str1 = 0; 
}

int main()
{
  float value = 1.123456;
  char buf[100];
  
  gcvt(value, 6, buf);
  
  printf(" %s\n",buf);
  
  terminate_str(buf, 3);
 
  printf(" %s\n",buf);
  
  return 0;
}
