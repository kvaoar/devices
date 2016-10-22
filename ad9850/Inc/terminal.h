#ifndef __TERMINAL_H
#define __TERMINAL_H
#include "stm32f1xx_hal.h"


typedef void(*COMMAND)( int len, char* param ); 
void help (int len, char* param);

typedef struct{
 char cmd [10];
 COMMAND exe;
} TDictonary;

typedef struct{
 char* p;
 int l;
} TParam;

void init_term(void);
void t_print(const char* str);
void t_addstr(char* str, uint32_t len);
void t_exe(void);

#endif
