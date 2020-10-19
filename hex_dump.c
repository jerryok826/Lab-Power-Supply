/* hex_dump.c : Lab Power Supply 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>

#include "main.h"
//#include "miniprintf.h"
#include "mcuio.h"  // for std_printf
//#include "FreeRTOS.h"

/*************************************************************/
void
hex_dump (uint8_t *buf, int len)
{
    char lineBuf[100];
    uint16_t offset=0;
    uint16_t hexOffset=0;
    uint16_t startOffset=0;
    char *ln = "\n";

    std_printf("Offset   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F        ASCII%s",ln);
    std_printf("======  == == == == == == == == == == == == == == == ==  *================*%s",ln);
    for(offset=0; offset < len; offset++){
        /* clear line buffer on start of line */
        hexOffset = offset%16;
        if(hexOffset == 0){
            memset(lineBuf,' ',sizeof(lineBuf));
            sprintf(lineBuf,"0x%04X: ",offset & 0xfff0);
            startOffset = strlen(lineBuf);
            lineBuf[startOffset+(16*3)+1]= '*';
            lineBuf[startOffset+(16*3)+16+2]= 0;
        }
        
        /* Fill line buffer with hex values */
        sprintf(lineBuf+startOffset+hexOffset*3,"%02X ",buf[offset]);
        
        /*  Fill line buffer with ASCII chars */
        if(isprint(buf[offset])){
            lineBuf[hexOffset+startOffset+(16*3)+2]= buf[offset];
        } else {
            lineBuf[hexOffset+startOffset+(16*3)+2]= '.';
        }
        /* print line */
        if((hexOffset == 15)||(offset ==(len-1))){
           // Line buffer fix up
           lineBuf[strlen(lineBuf)]= ' ';
           strcat(lineBuf, "*");
           std_printf("%s%s",lineBuf,ln);
        }
    }
}

