#ifndef TOSLINK_H
#define TOSLINK_H
#include "stm32f1xx.h"

void toslink_init(IRDA_HandleTypeDef* hirda);
void toslink_transmit(uint32_t data);
#endif
