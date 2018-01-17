/*
 * term.h
 *
 *  Created on: 16 авг. 2017 г.
 *      Author: kvaoar
 */

#ifndef TERM_H_
#define TERM_H_

#include "stm32f1xx.h"
void TermInit(UART_HandleTypeDef* huart);

void TermClr();
void TermRet();
void TermPut(char c);

#endif /* TERM_H_ */
