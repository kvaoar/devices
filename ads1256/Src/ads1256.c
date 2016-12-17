#include "ads1256.h"


SPI_HandleTypeDef* AD_SpiHandle;

void ads1256_init(SPI_HandleTypeDef* spi){
AD_SpiHandle = spi;
}

void ads1256_delay_t6(){
	for(int i = 0; i < 10; i++){i = 1*i;}
}

uint8_t ads1256_read_reg(uint8_t reg){
	uint8_t buf[2] = {(ADS1256_RREG | reg),0};
	uint8_t result = 0;
	HAL_SPI_Transmit(AD_SpiHandle,buf,2,1000);
	ads1256_delay_t6();
	HAL_SPI_Receive(AD_SpiHandle, &result,1,1000);
	return result;
}

void ads1256_write_reg(uint8_t reg, uint8_t val){
	uint8_t buf[3] = {(ADS1256_WREG | reg),0,val};
	HAL_SPI_Transmit(AD_SpiHandle,buf,3,1000);
}

void read_regs(uint8_t from_reg, uint8_t* to_buf, uint8_t bytes){
	if(bytes == 0) bytes = 1;
	uint8_t buf[2] = {(ADS1256_RREG | from_reg),bytes-1};
	HAL_SPI_Transmit(AD_SpiHandle,buf,2,1000);
	ads1256_delay_t6();
	HAL_SPI_Receive(AD_SpiHandle, to_buf,bytes,1000);
}

void ads1256_write_regs(uint8_t to_reg, uint8_t* from_buf, uint8_t bytes){
	if(bytes == 0) bytes = 1;
	uint8_t buf[2] = {(ADS1256_WREG | to_reg),bytes-1};
	HAL_SPI_Transmit(AD_SpiHandle,buf,2,1000);
	HAL_SPI_Transmit(AD_SpiHandle,from_buf,bytes,1000);
}

void ads1256_cmd(uint8_t c){
	HAL_SPI_Transmit(AD_SpiHandle,&c,1,1000);
}

void ads1256_read_data_continue_start(){
	ads1256_cmd(ADS1256_RDATAC);
	ads1256_delay_t6();
}

void ads1256_read_data_continue_stop(){
	ads1256_cmd(ADS1256_SDATAC);
	ads1256_delay_t6();
}

void ads1256_standby(){
	ads1256_cmd(ADS1256_STANDBY);
	ads1256_delay_t6();
}

void ads1256_wake_up(){
	ads1256_cmd(ADS1256_WAKEUP);
	ads1256_delay_t6();
}

void ads1256_self_calibration(){
	ads1256_cmd(ADS1256_WAKEUP);
	ads1256_delay_t6();
}

void ads1256_sync(){
	ads1256_cmd(ADS1256_SYNC);
	ads1256_delay_t6();
}







