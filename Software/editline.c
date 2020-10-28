#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "editline.h"

#define BELL    0x07
#define BS      0x08
#define LF      0x0A
#define CR      0x0D

static char *line = NULL;
static int size = 0;
static int pos = 0;

/**
 * Initializes the edit buffer.
 *
 * @param buffer the edit buffer
 * @param bufsize the size of the edit buffer
 */
void EditInit(char *buffer, int bufsize)
{
    line = buffer;
    size = bufsize;
}

/** 
 * Processes a character into an edit buffer, returns true
 * if a full line has been received
 * 
 * @param cin the character to process
 * @param cout the output character
 * @return true if a full line was entered, false otherwise
 */
bool EditLine(char cin, char *cout)
{
    *cout = cin;                // echo by default
    switch (cin) {              // carriage return is ignored
    case '\n':
        break;
    case '\r':                 // end-of-line
        line[pos] = '\0';
        pos = 0;
        return true;
    case 0x7F:
    case 0x08:                 // backspace
        if (pos > 0) {
            pos--;
        }
        break;
    default:
        if (pos < (size - 1)) { // store char as long as there is space to do so
            line[pos++] = cin;
        } else {
            *cout = 0x07;       // bell
        }
        break;
    }
    return false;
}

