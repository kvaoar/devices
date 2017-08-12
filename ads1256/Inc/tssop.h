#ifndef TSSOP_H
#define TSSOP_H
#include "stm32f1xx.h"
void tssop_init(UART_HandleTypeDef* huart);
void tssop_transmit(uint32_t data);
#endif
