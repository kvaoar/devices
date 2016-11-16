#ifndef __TERMINAL_H
#define __TERMINAL_H
#include "stm32f1xx_hal.h"

typedef struct{
 char* p;
 int l;
} TString;

typedef void(*COMMAND)( TString param  ); 

typedef struct{
 TString cmd;
 COMMAND exe;
} TDictonary;

void help (TString param );
void fset (TString param );

void term_init(void);
void term_reset(void);
void term_print(const char* str);
void term_addstr(char* str, uint32_t len);
void term_exe(void);

#endif
