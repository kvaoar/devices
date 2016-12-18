
#ifndef __ADS1256_H
#define __ADS1256_H
#include "stm32f1xx_hal.h"

/* ADS1256 Commands */
#define ADS1256_WAKEUP 0
#define ADS1256_RDATA 1
#define ADS1256_RDATAC 3
#define ADS1256_SDATAC 0x0F
#define ADS1256_RREG 0x10		//Or with starting register address and add second command byte
#define ADS1256_WREG 0x50		//Or with starting register address and add second command byte
#define ADS1256_SELFCAL 0xF0
#define ADS1256_SELFOCAL 0xF1
#define ADS1256_SELFGCAL 0xF2
#define ADS1256_SYSOCAL 0xF3
#define ADS1256_SYSGCAL 0xF4
#define ADS1256_SYNC 0xFC
#define ADS1256_STANDBY 0xFD
#define ADS1256_RESET 0xFE

/* Register Addresses */
#define ADS1256_STATUS 0
#define ADS1256_MUX 1
#define ADS1256_ADCON 2
#define ADS1256_DRATE 3
#define ADS1256_IO 4
#define ADS1256_OFC0 5
#define ADS1256_OFC1 6
#define ADS1256_OFC2 7
#define ADS1256_FSC0 8
#define ADS1256_FSC1 9
#define ADS1256_FSC2 10

/* Status Register Bits */
#define ADS1256_DRDY 1
#define ADS1256_BUFEN 2
#define ADS1256_ACAL 4
#define ADS1256_ORDER 8



void ads1256_init(SPI_HandleTypeDef* spi);
uint8_t ads1256_getstatus(void);
void ads1256_read_data_continue_start();
uint32_t ads1256_read_data(void);
GPIO_PinState drdy_tst();
	void ads1256_sync();
void	ads1256_wake_up();
void ads1256_cmd(uint8_t c);


void nss_hi();
void nss_low();

#endif
