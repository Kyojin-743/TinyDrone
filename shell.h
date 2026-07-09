#ifndef SHELL_H_
#define SHELL_H_


#include <stdint.h>
#include <stdbool.h>
#include "kernel.h"
#include "mm.h"

//-----------------------------------------------------------------------------
// Pre-Processor Macros and Defines
//-----------------------------------------------------------------------------
#define MAX_CHARS       30
#define MAX_FIELDS      5
#define MAX_DIG_U32     10
#define MAX_QUEUE       15

#define DELIMETER       0
#define ALPHA           1
#define NUMERIC         2

//ANSII full commands
#define GOTO_HOME       "\x1B[H"
#define GOTO_BEGINNING  "\x1B[G"
#define PREV_LINE       "\x1B[F"
#define SAVE_POS        "\x1B[s"
#define RETURN_2_POS    "\x1B[u"

#define CLEAR_SCREEN    "\x1B[2J"
#define CLEAR_LINE      "\x1B[2K"
#define CLEAR_COLOR     "\x1b[0m"

#define SET_FG_BL       "\x1b[30m"
#define SET_FG_W        "\x1b[97m"
#define SET_FG_R        "\x1b[31m"
#define SET_FG_G        "\x1b[32m"
#define SET_FG_Y        "\x1b[33m"
#define SET_FG_B        "\x1b[34m"

#define SET_BG_BL       "\x1b[40m"
#define SET_BG_W        "\x1b[107m"
#define SET_BG_R        "\x1b[41m"
#define SET_BG_G        "\x1b[42m"
#define SET_BG_Y        "\x1b[43m"
#define SET_BG_B        "\x1b[44m"

#define HIDE_CURSOR     "\x1B[?25l"
#define SHOW_CURSOR     "\x1B[?25h"

//-----------------------------------------------------------------------------
//Types
//-----------------------------------------------------------------------------
typedef enum _color{
    White = 0,
    Black,
    Red,
    Green,
    Blue,
    Yellow
}Color;

typedef struct _uartArgs
{
    Color bg;
    Color fg;
    char end;
}UartArgs;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
#define Print(s, ...) (_printArgs(s, (UartArgs){\
    .bg = Black,\
    .fg = White,\
    .end = '\n',\
    __VA_ARGS__\
}))

void _printArgs(const char str[], UartArgs uArgs);
void strcpy(char* to, const char* from);
void strncpy(char* to, const char* from, uint8_t n);
void strcpyFill(char* to, const char* from, uint8_t len, char letter);
void bytecpy(void* dst, void* src, uint32_t size);
uint32_t strlen(const char *str);
void strnappnd(char* to, const char* from, uint8_t n);
bool strcmp(const char* s1, const char* s2);
int32_t atoi32(const char* str);
//uint16_t pow(int base, int power);
void itoa32(uint32_t num, char str[MAX_DIG_U32]);
void htoa(uint32_t num, char str[MAX_DIG_U32]);
inline int min(int l, int r);
int usprintf(char *buffer, const char *format, ...);

#endif /* SHELL_H_ */
