#include "ads1256.h"
#include "mxconstants.h"


SPI_HandleTypeDef* AD_SpiHandle;

void nss_hi(){HAL_GPIO_WritePin(nss_GPIO_Port,nss_Pin,GPIO_PIN_SET);}
void nss_low(){HAL_GPIO_WritePin(nss_GPIO_Port,nss_Pin,GPIO_PIN_RESET);}

void rst_hi(){HAL_GPIO_WritePin(rst_GPIO_Port,rst_Pin,GPIO_PIN_SET);}
void rst_low(){HAL_GPIO_WritePin(rst_GPIO_Port,rst_Pin,GPIO_PIN_RESET);}

GPIO_PinState drdy_tst(){return HAL_GPIO_ReadPin(drdy_GPIO_Port,drdy_Pin);}


void ads1256_delay_t6(){
	for(int i = 0; i < 100; i++){i = 1*i;}
}

void ads1256_wait_drdy(){
while(HAL_GPIO_ReadPin(drdy_GPIO_Port,drdy_Pin) == GPIO_PIN_SET){	HAL_Delay(1);};
}

void ads1256_hwreset(){
	rst_low();
HAL_Delay(10);
	rst_hi();
	HAL_Delay(10);
	ads1256_wait_drdy();
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
	//if(bytes == 0) bytes = 1;
	uint8_t cmd = ADS1256_RREG | from_reg;
	uint8_t length = bytes - 1;
	uint8_t buf[2] = {(cmd),(length)};
	
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
uint32_t ads1256_read_data(){
	
	uint8_t buf[3] = {0,0,0};
	ads1256_delay_t6();
	HAL_SPI_Receive(AD_SpiHandle, buf,3,1000);
	
	return((((uint32_t)buf[0]) <<16)+(((uint32_t)buf[1])<<8)+(((uint32_t)buf[2])  ));
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

void ads1256_init(SPI_HandleTypeDef* spi){
AD_SpiHandle = spi;
ads1256_hwreset();
	nss_low();
	
	ads1256_write_reg(ADS1256_STATUS, ADS1256_STATUS_BUFEN);//ADS1256_STATUS_BUFEN
	//HAL_Delay(100);
	ads1256_wait_drdy();
	ads1256_write_reg(ADS1256_MUX, ADS1256_MUX_AIN4P|ADS1256_MUX_AIN5N);
	//HAL_Delay(100);
	ads1256_wait_drdy();
	ads1256_write_reg(ADS1256_ADCON, ADS1256_ADCON_CLKOUT_OFF|ADS1256_ADCON_ISENSOR_OFF|ADS1256_ADCON_PGA1 );//(1<<3)|(1<<4)
	//HAL_Delay(100);
	ads1256_wait_drdy();
	ads1256_write_reg(ADS1256_DRATE, ADS1256_DRATE_ADS1256_500SPS);
//	HAL_Delay(100);
	ads1256_wait_drdy();
		//HAL_Delay(2000);
	//ads1256_wait_drdy();
	//ads1256_cmd( ADS1256_SELFCAL);

	//ads1256_wait_drdy();
}

uint8_t ads1256_getstatus(){
	return ads1256_read_reg(ADS1256_STATUS);
}

