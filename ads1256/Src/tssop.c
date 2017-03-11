#include "tssop.h"

UART_HandleTypeDef* huart;

void tssop_init(UART_HandleTypeDef* uart){
huart = uart;
};

uint32_t tssop_start(){

}