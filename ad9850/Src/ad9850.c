
#include "ad9850.h"

#include "mxconstants.h"

void delay(){
		for(int i = 0; i < 3; i++) i = 1*i;
};

void clk() {
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
delay();
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
	delay();
}

void freq_upd(){
HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	delay();
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	delay();
}

static void send(uint8_t byte) {
	uint8_t i = 0;
	for (i = 0; i < 8; i++) 
	{
		if (byte & 1)
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
			delay();
		clk();
		byte >>= 1;
	}
	delay();
	HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
}

void ad9850_freq(uint32_t f) {

	float freq = 0;
	uint32_t word = 0;
	uint8_t i = 0;
	
	freq = (float)4294967295 / 100000000;
	freq *= f;
	word = (uint32_t)freq;

	for (i = 0; i < 4; i++) {
		send((uint8_t) (word & 0xff));
		word >>= 8;
	}
	send(0x00);
	delay();
	freq_upd();
}

void ad9850_init(void) {

//	clk();
//	freq_upd();
}
