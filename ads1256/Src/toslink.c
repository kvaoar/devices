#include "toslink.h"
IRDA_HandleTypeDef* hirda;

void toslink_init(IRDA_HandleTypeDef* irda){
hirda = irda;
};

void toslink_transmit(uint32_t data){
	HAL_IRDA_Transmit( hirda,(uint8_t*)&data,4,100);
};
