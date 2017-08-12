#include "tssop.h"
#include "string.h"

UART_HandleTypeDef* huart; //uart used for tssop
uint8_t enabled = 0;	//write enable flag
#define TSSOP_LEN (4) // data buffer length
uint8_t tssop_buf[TSSOP_LEN];	//data buffer
uint8_t ptr = 0;	// write position in buffer
uint8_t rdy = 0; //buffer full flag

void tssop_init(UART_HandleTypeDef* uart){
huart = uart;
	memset(tssop_buf,0, TSSOP_LEN );

}

void tssop_start(){
enabled = 1;
}
void tssop_stop(){
enabled = 0;
}


void tssop_recieve_irq(uint8_t byte){
if(enabled == 1)
		{
		tssop_buf[ptr++] = byte;
		if(ptr >= TSSOP_LEN) 
				{
					ptr = 0;
					rdy = 1;
				}
		}
}

void tssop_check_buf(){

}
