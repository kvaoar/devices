
#ifndef __ADS1256_H
#define __ADS1256_H
#include "stm32f1xx_hal.h"

/* ADS1256 Commands */
#define ADS1256_WAKEUP 		(0x00)
#define ADS1256_RDATA 		(0x01)
#define ADS1256_RDATAC 		(0x03)
#define ADS1256_SDATAC 		(0x0F)
#define ADS1256_RREG 		(0x10)		//Or with starting register address and add second command byte
#define ADS1256_WREG 		(0x50)		//Or with starting register address and add second command byte
#define ADS1256_SELFCAL 	(0xF0)
#define ADS1256_SELFOCAL 	(0xF1)
#define ADS1256_SELFGCAL 	(0xF2)
#define ADS1256_SYSOCAL 	(0xF3)
#define ADS1256_SYSGCAL 	(0xF4)
#define ADS1256_SYNC 		(0xFC)
#define ADS1256_STANDBY 	(0xFD)
#define ADS1256_RESET 		(0xFE)

/* Register Addresses */
#define ADS1256_STATUS 	(0)
#define ADS1256_MUX 	(1)
#define ADS1256_ADCON 	(2)
#define ADS1256_DRATE 	(3)
#define ADS1256_IO 		(4)
#define ADS1256_OFC0 	(5)
#define ADS1256_OFC1 	(6)
#define ADS1256_OFC2 	(7)
#define ADS1256_FSC0 	(8)
#define ADS1256_FSC1 	(9)
#define ADS1256_FSC2 	(10)

/* Status Register Bits */
#define ADS1256_STATUS_DRDY 	(1)
#define ADS1256_STATUS_BUFEN 	(2)
#define ADS1256_STATUS_ACAL 	(4)
#define ADS1256_STATUS_ORDER 	(8)

#define ADS1256_DRATE_ADS1256_30000SPS 	(0xF0)	/*reset the default values  */
#define ADS1256_DRATE_ADS1256_15000SPS	(0xE0)
#define ADS1256_DRATE_ADS1256_7500SPS	(0xD0)
#define ADS1256_DRATE_ADS1256_3750SPS	(0xC0)
#define ADS1256_DRATE_ADS1256_2000SPS	(0xB0)
#define ADS1256_DRATE_ADS1256_1000SPS	(0xA1)
#define ADS1256_DRATE_ADS1256_500SPS	(0x92)
#define ADS1256_DRATE_ADS1256_100SPS	(0x82)
#define ADS1256_DRATE_ADS1256_60SPS		(0x72)
#define ADS1256_DRATE_ADS1256_50SPS		(0x63)
#define ADS1256_DRATE_ADS1256_30SPS		(0x53)
#define ADS1256_DRATE_ADS1256_25SPS		(0x43)
#define ADS1256_DRATE_ADS1256_15SPS		(0x33)
#define ADS1256_DRATE_ADS1256_10SPS		(0x20)
#define ADS1256_DRATE_ADS1256_5SPS		(0x13)
#define ADS1256_DRATE_ADS1256_2d5SPS	(0x03)

#define ADS1256_MUX_AIN0P				(0x00)
#define ADS1256_MUX_AIN1P				(0x10)
#define ADS1256_MUX_AIN2P				(0x20)
#define ADS1256_MUX_AIN3P				(0x30)
#define ADS1256_MUX_AIN4P				(0x40)
#define ADS1256_MUX_AIN5P				(0x50)
#define ADS1256_MUX_AIN6P				(0x60)
#define ADS1256_MUX_AIN7P               (0x70)
#define ADS1256_MUX_AINGNDP				(0x80)

#define ADS1256_MUX_AIN0N				(0x00)
#define ADS1256_MUX_AIN1N				(0x01)
#define ADS1256_MUX_AIN2N				(0x02)
#define ADS1256_MUX_AIN3N				(0x03)
#define ADS1256_MUX_AIN4N				(0x04)
#define ADS1256_MUX_AIN5N				(0x05)
#define ADS1256_MUX_AIN6N				(0x06)
#define ADS1256_MUX_AIN7N               (0x07)
#define ADS1256_MUX_AINGNDN				(0x08)


#define ADS1256_ADCON_CLKOUT_OFF  (0<<5)//00 = Clock Out OFF
#define ADS1256_ADCON_CLKOUT_DIV1 (1<<5)//01 = Clock Out Frequency = fCLKIN(default)
#define ADS1256_ADCON_CLKOUT_DIV2 (2<<5)//10 = Clock Out Frequency = fCLKIN/2
#define ADS1256_ADCON_CLKOUT_DIV4 (3<<5)//11 = Clock Out Frequency = fCLKIN/4

#define ADS1256_ADCON_ISENSOR_OFF 	(0<<3)//00 = Sensor Detect OFF (default)
#define ADS1256_ADCON_ISENSOR_500NA (1<<3)//01 = Sensor Detect Current = 0.5µA
#define ADS1256_ADCON_ISENSOR_2UA 	(2<<3)//10 = Sensor Detect Current = 2µA
#define ADS1256_ADCON_ISENSOR_10UA 	(3<<3)//11 = Sensor Detect Current = 10µA

#define ADS1256_ADCON_PGA1 	(0)
#define ADS1256_ADCON_PGA2 	(1)
#define ADS1256_ADCON_PGA4 	(2)
#define ADS1256_ADCON_PGA8 	(3)
#define ADS1256_ADCON_PGA16 (4)
#define ADS1256_ADCON_PGA32 (5)
#define ADS1256_ADCON_PGA64 (6) 

















void ads1256_init(SPI_HandleTypeDef* spi);
uint8_t ads1256_getstatus(void);
void ads1256_read_data_continue_start();
uint32_t ads1256_read_data(void);
GPIO_PinState drdy_tst();
	void ads1256_sync();
void	ads1256_wake_up();
void ads1256_cmd(uint8_t c);
void read_regs(uint8_t from_reg, uint8_t* to_buf, uint8_t bytes);

void nss_hi();
void nss_low();

#endif
