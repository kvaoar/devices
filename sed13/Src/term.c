/*
 * term.c
 *
 *  Created on: 16 авг. 2017 г.
 *      Author: kvaoar
 */
#include "term.h"
#include "sed1335.h"
#include "string.h"
#include "stm32f1xx.h"

#include "ringbufferdma.h"

uint8_t is_dma_en = 0;
uint8_t line = 0;
uint8_t pos = 0;

UART_HandleTypeDef* uart;

#define STRLEN (256)

#define LINEMAX	(30)
#define POSMAX	(40)

uint8_t buf[STRLEN];
RingBufferDmaU8 rbuf;

void TermInit(UART_HandleTypeDef* huart){

	uart = huart;
	GLCD_Initialize();
	TermClr();
	RingBufferDmaU8_initUSARTRx(&rbuf,uart,buf,STRLEN-1);

}



void TermClr(){
	line = 0;
	pos = 0;
	GLCD_ClearGraphic();
	GLCD_ClearText();
	GLCD_TextGoTo(0,0);
}

void TermRet(){
	pos = 0;
	line++;
	if(line >= LINEMAX)TermClr();

};

void TermPut(char c){
	GLCD_TextGoTo(pos,line);
	GLCD_WriteChar(c);
	pos++;

	if(pos >= POSMAX)
		TermRet();
};


void TermProc(){
char cbuf[STRLEN];
memset(cbuf,0,POSMAX);

int len = RingBufferDmaU8_available(&rbuf);

if(len > 0){


	if(RingBufferDmaU8_readLine(&rbuf,cbuf,STRLEN-1)) {
		int slen = strlen(cbuf);
		if(slen >1){
		for(int i = 0; i < slen; i++)
			TermPut(cbuf[i]);
		TermRet();
		}
	}
}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

}
