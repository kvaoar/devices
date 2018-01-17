#include "stm32f1xx_hal.h"

#define SED1335_PORT GPIOA

#define SED1335_A0	GPIO_PIN_10
#define SED1335_WR	GPIO_PIN_8
#define SED1335_RD	GPIO_PIN_9
#define SED1335_CS	GPIO_PIN_11
#define SED1335_RES	GPIO_PIN_12

#define SED1335_D0   0

  GPIO_InitTypeDef GPIO_InitStruct;

  void delay(){__NOP();__NOP();__NOP();__NOP();__NOP();}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_InitPorts(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin   =  GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SED1335_PORT, &GPIO_InitStruct);
	
   HAL_GPIO_WritePin(SED1335_PORT, SED1335_A0 | SED1335_WR | SED1335_RD | SED1335_CS, GPIO_PIN_SET);
   delay();
HAL_GPIO_WritePin(SED1335_PORT, SED1335_RES, GPIO_PIN_SET);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
SED1335_PORT->ODR = (SED1335_PORT->ODR&0xFF00)|(dataToWrite << SED1335_D0);
/*SED1335_PORT->BSRR = (dataToWrite � SED1335_D0);
dataToWrite ^= 0xFF;

SED1335_PORT->BRR = (dataToWrite � SED1335_D0);*/


   SED1335_PORT->BRR = (SED1335_CS | SED1335_A0 | SED1335_WR);
   delay();
	SED1335_PORT->BSRR = (SED1335_CS | SED1335_A0 | SED1335_WR);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite)
{
SED1335_PORT->ODR = (SED1335_PORT->ODR&0xFF00)|(commandToWrite << SED1335_D0);
/*SED1335_PORT->BSRR = (dataToWrite � SED1335_D0);
dataToWrite ^= 0xFF;

SED1335_PORT->BRR = (dataToWrite � SED1335_D0);*/
SED1335_PORT->BRR =(SED1335_CS | SED1335_WR);	
delay();
SED1335_PORT->BSRR =(SED1335_CS | SED1335_WR);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
unsigned char tmp;

	
 GPIO_InitStruct.Pin = 0xFF << SED1335_D0;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(SED1335_PORT, &GPIO_InitStruct);
	
SED1335_PORT->BRR =(SED1335_CS | SED1335_RD);
delay();
tmp = (( (uint16_t)SED1335_PORT->IDR >> SED1335_D0) & 0xFF);

	
SED1335_PORT->BSRR =  (SED1335_CS | SED1335_RD);



GPIO_InitStruct.Pin   =  (0xFF << SED1335_D0);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(SED1335_PORT, &GPIO_InitStruct);

return tmp;
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
char GLCD_ReadByteFromROMMemory(char * ptr)
{
return *(ptr);
}
//









