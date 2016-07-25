/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"
#include "TouchPanel.h"
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
DMA_HandleTypeDef hdma_sdio;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const int TSTWORD = 201;
static RTC_TimeTypeDef sTime;
static RTC_DateTypeDef		sDate;

// FSMC
#define FSMC_LCD_INSTANCE &hsram1
#define FSMC_LCD_DATA 		(uint32_t*)0x60020000
#define FSMC_LCD_COMMAND  (uint32_t*)0x60000000

void FSMC_LCD_Write_Command(uint16_t command) {
  *(__IO uint16_t *)FSMC_LCD_COMMAND = command; 
	//HAL_SRAM_Write_16b(FSMC_LCD_INSTANCE, FSMC_LCD_COMMAND, &command, 1);
}

void FSMC_LCD_Write_Data(uint16_t data) {
  *(__IO uint16_t *)FSMC_LCD_DATA = data; 
	//HAL_SRAM_Write_16b(FSMC_LCD_INSTANCE, FSMC_LCD_DATA, &data, 1);
}

uint16_t FSMC_LCD_Read_Data(void) {
	uint16_t data = 0;
	data = *(__IO uint16_t *)FSMC_LCD_DATA;
  //HAL_SRAM_Read_16b(FSMC_LCD_INSTANCE, FSMC_LCD_DATA, &data, 1);
	return data;
}

void FSMC_LCD_Write_Register(uint16_t reg, uint16_t val)
{
  FSMC_LCD_Write_Command(reg);
  FSMC_LCD_Write_Data(val);
}

// SSD1289 Initialize Sequence
// R01h Display Control Register
#define LCD_R01_RL 0
#define LCD_R01_REV 1
#define LCD_R01_CAD 0
#define LCD_R01_BGR 0
#define LCD_R01_SM 0
#define LCD_R01_TB 1
#define LCD_R01_MUX 319

// PB11-LED_A
void LCD_BACKLIGHT_ON() { HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);}
void LCD_BACKLIGHT_OFF() { HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); }

void SSD1289_Init() {

  LCD_BACKLIGHT_ON();

  // Display ON Sequence
  // Power supply setting

  // Set R07h at 0021h
  FSMC_LCD_Write_Register(0x07, 0x0021);
  HAL_Delay(1);  // s
  FSMC_LCD_Write_Register(0x00, 0x0001);
  HAL_Delay(1);  // s

  // Set R07h at 0023h
  FSMC_LCD_Write_Register(0x07, 0x0023);
  HAL_Delay(1);  // s

  // Set R10h at 0000h
  FSMC_LCD_Write_Register(0x10, 0x0000);
  HAL_Delay(1);  // s

  // Wait 30ms
  HAL_Delay(30);  // s

  // Set R07h at 0033h
  FSMC_LCD_Write_Register(0x07, 0x0033);
  HAL_Delay(1);  // s

  // Entry Mode setting (R11h)
  FSMC_LCD_Write_Register(0x0011, 0x6070);
  HAL_Delay(1);  // s

  // LCD driver AC setting (R02h)
  FSMC_LCD_Write_Register(0x0002, 0x0600);
  HAL_Delay(1);  // s

  // Driver Output Control (R01h)
  FSMC_LCD_Write_Register(0x0001,
      (LCD_R01_RL << 14) | (LCD_R01_REV << 13) | (LCD_R01_CAD << 12) | (LCD_R01_BGR << 11) | (LCD_R01_SM << 10) | (LCD_R01_TB << 9) | LCD_R01_MUX);
  HAL_Delay(1);
	
	FSMC_LCD_Write_Register(0x000b, 0x0000);  // Frame Cycle Control
  FSMC_LCD_Write_Register(0x000f, 0x0000);  // Gate Scan Start Position
	FSMC_LCD_Write_Register(0x0044, 0xef00);  // Horizontal RAM start and end address
  FSMC_LCD_Write_Register(0x0045, 0x0000);  // Vertical RAM start address
  FSMC_LCD_Write_Register(0x0046, 0x013f);  // Vertical RAM end address

  // RAM data write (R22h)
  FSMC_LCD_Write_Command(0x0022);  // s

  // Display ON and start to write RAM
  HAL_Delay(100);
}

// LCD Graphic Interface
// Colors
#define yellow    0x07FF
#define magneta   0xF81F
#define cyan      0xFFE0
#define red       0x001F
#define green     0x07E0
#define blue      0xF800
#define white     0xFFFF
#define black     0x3185
#define glass      0x0000

const unsigned char font_5x7[][5] = { { 0x00, 0x00, 0x00, 0x00, 0x00 }, // „?„‚„?„q„u„|
    { 0x00, 0x00, 0x4f, 0x00, 0x00 },     // !
    { 0x00, 0x07, 0x00, 0x07, 0x00 },     // "
    { 0x14, 0x7f, 0x14, 0x7f, 0x14 },     // #
    { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },     // $
    { 0x23, 0x13, 0x08, 0x64, 0x62 },     // %
    { 0x36, 0x49, 0x55, 0x22, 0x40 },     // &
    { 0x00, 0x05, 0x03, 0x00, 0x00 },     // ,
    { 0x00, 0x1c, 0x22, 0x41, 0x00 },     // (
    { 0x00, 0x41, 0x22, 0x1c, 0x00 },     // )
    { 0x14, 0x08, 0x3E, 0x08, 0x14 },     // *
    { 0x08, 0x08, 0x3E, 0x08, 0x08 },     // +
    { 0x00, 0x50, 0x30, 0x00, 0x00 },     // ,
    { 0x08, 0x08, 0x08, 0x08, 0x08 },     // -
    { 0x00, 0x00, 0x40, 0x00, 0x00 },     // .
    { 0x20, 0x10, 0x08, 0x04, 0x02 },     // /
    { 0x3e, 0x51, 0x49, 0x45, 0x3e },     // 0
    { 0x00, 0x42, 0x7f, 0x40, 0x00 },     // 1
    { 0x42, 0x61, 0x51, 0x49, 0x46 },     // 2
    { 0x21, 0x41, 0x45, 0x4b, 0x31 },     // 3
    { 0x18, 0x14, 0x12, 0x7f, 0x10 },     // 4
    { 0x27, 0x45, 0x45, 0x45, 0x39 },     // 5
    { 0x3c, 0x4a, 0x49, 0x49, 0x30 },     // 6
    { 0x01, 0x71, 0x09, 0x05, 0x03 },     // 7
    { 0x36, 0x49, 0x49, 0x49, 0x36 },     // 8
    { 0x06, 0x49, 0x49, 0x29, 0x1e },     // 9
    { 0x00, 0x00, 0x24, 0x00, 0x00 },     // :
    { 0x00, 0x56, 0x36, 0x00, 0x00 },     // ;
    { 0x08, 0x1C, 0x3E, 0x7F, 0x00 },     // <
    { 0x14, 0x14, 0x14, 0x14, 0x14 },     // =
    { 0x00, 0x7F, 0x3E, 0x1C, 0x08 },     // >
    { 0x02, 0x01, 0x51, 0x09, 0x06 },     // ?
    { 0x32, 0x49, 0x71, 0x41, 0x3e },     // @
    { 0x7e, 0x11, 0x11, 0x11, 0x7e },     // A
    { 0x7f, 0x49, 0x49, 0x49, 0x36 },     // B
    { 0x3e, 0x41, 0x41, 0x41, 0x22 },     // C
    { 0x7f, 0x41, 0x41, 0x22, 0x1c },     // D
    { 0x7f, 0x49, 0x49, 0x49, 0x41 },     // E
    { 0x7f, 0x09, 0x09, 0x09, 0x01 },     // F
    { 0x3e, 0x41, 0x49, 0x49, 0x3a },     // G
    { 0x7f, 0x08, 0x08, 0x08, 0x7f },     // H
    { 0x00, 0x41, 0x7f, 0x41, 0x00 },     // I
    { 0x20, 0x40, 0x41, 0x3f, 0x01 },     // J
    { 0x7f, 0x08, 0x14, 0x22, 0x41 },     // K
    { 0x7f, 0x40, 0x40, 0x40, 0x40 },     // L
    { 0x7f, 0x02, 0x0c, 0x02, 0x7f },     // M
    { 0x7f, 0x04, 0x08, 0x10, 0x7f },     // N
    { 0x3e, 0x41, 0x41, 0x41, 0x3e },     // O
    { 0x7f, 0x09, 0x09, 0x09, 0x06 },     // P
    { 0x3e, 0x41, 0x51, 0x21, 0x5e },     // Q
    { 0x7f, 0x09, 0x19, 0x29, 0x46 },     // R
    { 0x46, 0x49, 0x49, 0x49, 0x31 },     // S
    { 0x01, 0x01, 0x7f, 0x01, 0x01 },     // T
    { 0x3f, 0x40, 0x40, 0x40, 0x3f },     // U
    { 0x1f, 0x20, 0x40, 0x20, 0x1f },     // V
    { 0x3f, 0x40, 0x30, 0x40, 0x3f },     // W
    { 0x63, 0x14, 0x08, 0x14, 0x63 },     // X
    { 0x07, 0x08, 0x70, 0x08, 0x07 },     // Y
    { 0x61, 0x51, 0x49, 0x45, 0x43 },     // Z
    { 0x00, 0x7F, 0x41, 0x41, 0x00 },     // [
    { 0x02, 0x04, 0x08, 0x10, 0x20 },     //
    { 0x02, 0x04, 0x08, 0x10, 0x20 },     //
    { 0x00, 0x41, 0x41, 0x7F, 0x00 },     // ]
    { 0x04, 0x02, 0x01, 0x02, 0x04 },     // ^
    { 0x40, 0x40, 0x40, 0x40, 0x40 },     // _
    { 0x20, 0x54, 0x54, 0x54, 0x78 },     // a
    { 0x7F, 0x48, 0x44, 0x44, 0x38 },     // b
    { 0x38, 0x44, 0x44, 0x44, 0x20 },     // c
    { 0x38, 0x44, 0x44, 0x48, 0x7F },     // d
    { 0x38, 0x54, 0x54, 0x54, 0x18 },     // e
    { 0x08, 0x7E, 0x09, 0x01, 0x02 },     // f
    { 0x0C, 0x52, 0x52, 0x52, 0x3E },     // g
    { 0x7F, 0x08, 0x04, 0x04, 0x78 },     // h
    { 0x00, 0x44, 0x7D, 0x40, 0x00 },     // i
    { 0x20, 0x40, 0x44, 0x3D, 0x00 },     // j
    { 0x7F, 0x10, 0x28, 0x44, 0x00 },     // k
    { 0x00, 0x41, 0x7F, 0x40, 0x00 },     // l
    { 0x7C, 0x04, 0x18, 0x04, 0x78 },     // m
    { 0x7C, 0x08, 0x04, 0x04, 0x78 },     // n
    { 0x38, 0x44, 0x44, 0x44, 0x38 },     // o
    { 0x7C, 0x14, 0x14, 0x14, 0x08 },     // p
    { 0x08, 0x14, 0x14, 0x18, 0x7C },     // q
    { 0x7C, 0x08, 0x04, 0x04, 0x08 },     // r
    { 0x48, 0x54, 0x54, 0x54, 0x20 },     // s
    { 0x04, 0x3F, 0x44, 0x40, 0x20 },     // t
    { 0x3C, 0x40, 0x40, 0x20, 0x7C },     // u
    { 0x1C, 0x20, 0x40, 0x20, 0x1C },     // v
    { 0x3C, 0x40, 0x30, 0x40, 0x3C },     // w
    { 0x44, 0x28, 0x10, 0x28, 0x44 },     // x
    { 0x0C, 0x50, 0x50, 0x50, 0x3C },     // y
    { 0x44, 0x64, 0x54, 0x4C, 0x44 },     // z
    { 0x00, 0x08, 0x36, 0x41, 0x00 },     // {
    { 0x00, 0x00, 0x7f, 0x00, 0x00 },     // |
    { 0x00, 0x41, 0x36, 0x08, 0x00 },     // }
    { 0x02, 0x01, 0x02, 0x02, 0x01 }     // ~
};

void LCD_SetCursor(uint16_t x, uint16_t y) {

  FSMC_LCD_Write_Register(0x004E,x);
  FSMC_LCD_Write_Register(0x004F,y);
  FSMC_LCD_Write_Command(0x22);
}

void LCD_Clear(uint16_t color) {
  uint32_t index = 0;
  LCD_SetCursor(0, 0);
  FSMC_LCD_Write_Command(0x22);
  for (index = 0; index < 240 * 320; index++) {
    FSMC_LCD_Write_Data(color);
  }
	FSMC_LCD_Write_Command(0x0022); 
}

void LCD_SetPoint(uint16_t x, uint16_t y, uint16_t Color) {
  LCD_SetCursor(y, x);
  FSMC_LCD_Write_Register(0x22, Color);
}

void LCD_SetArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  FSMC_LCD_Write_Command(0x44);
  FSMC_LCD_Write_Data((x2 << 8) | x1);    // Source RAM address window
  FSMC_LCD_Write_Command(0x45);
  FSMC_LCD_Write_Data(y1);    // Gate RAM address window
  FSMC_LCD_Write_Command(0x46);
  FSMC_LCD_Write_Data(y2);    // Gate RAM address window
  LCD_SetCursor(x1, y1);
}

void LCD_WriteChar5x7(uint16_t x, uint16_t y, char c, uint16_t t_color,
    uint16_t b_color, uint8_t rot, uint8_t zoom) {
  unsigned char h, ch, p, mask, z, z1;

  if (rot != 0)
    LCD_SetArea(x, y, x + (6 * zoom) - 1, y + (8 * zoom) - 1);
  else
    LCD_SetArea(y, x, y + (8 * zoom) - 1, x + (6 * zoom) - 1);

  for (h = 0; h < 6; h++) {
    if (h < 5) {
      if ((uint8_t)c < 129)
        ch = font_5x7[c - 32][h];
      else
        ch = font_5x7[c - 32 - 63][h];

      if (rot != 0) {
        FSMC_LCD_Write_Register(0x0011,0x6078);
        FSMC_LCD_Write_Command(0x22);
      }
    } else
      ch = 0;

    z1 = zoom;
    while (z1 != 0) {
      if (rot != 0)
        mask = 0x01;
      else
        mask = 0x80;

      for (p = 0; p < 8; p++) {
        z = zoom;
				
				if(b_color == glass)
        while (z != 0) {
          if (ch & mask)
            FSMC_LCD_Write_Data(t_color);
          else
            FSMC_LCD_Read_Data();
          z--;
        }
				else
					while (z != 0) {
          if (ch & mask)
            FSMC_LCD_Write_Data(t_color);
          else
						FSMC_LCD_Write_Data(b_color);
          z--;
        }
				
        if (rot != 0)
          mask = mask << 1;
        else
          mask = mask >> 1;
      }
      z1--;
    }
  }
	
	LCD_SetArea(0,0,239,316);
}

void LCD_WriteString_5x7(uint16_t x, uint16_t y, char *text, uint16_t charColor,
    uint16_t b_color, uint8_t rot, uint8_t zoom) {
  uint8_t i;
  for (i = 0; *text; i++)
    LCD_WriteChar5x7(x + (i * 6 * zoom), y, *text++, charColor, b_color, rot,
        zoom);
}

void LCD_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
    uint16_t color) {
  uint16_t x, y, dx, dy;
  if (y1 == y2) {
    if (x1 <= x2)
      x = x1;
    else {
      x = x2;
      x2 = x1;
    }

    while (x <= x2) {
      LCD_SetPoint(x, y1, color);
      x++;
    }
    return;
  }

  else if (y1 > y2)
    dy = y1 - y2;
  else
    dy = y2 - y1;

  if (x1 == x2) {
    if (y1 <= y2)
      y = y1;
    else {
      y = y2;
      y2 = y1;
    }

    while (y <= y2) {
      LCD_SetPoint(x1, y, color);
      y++;
    }
    return;
  }

  else if (x1 > x2) {
    dx = x1 - x2;
    x = x2;
    x2 = x1;
    y = y2;
    y2 = y1;
  } else {
    dx = x2 - x1;
    x = x1;
    y = y1;
  }
  if (dx == dy) {
    while (x <= x2) {
      x++;
      if (y > y2)
        y--;

      else
        y++;
      LCD_SetPoint(x, y, color);
    }
  } else {
    LCD_SetPoint(x, y, color);
    if (y < y2) {
      if (dx > dy) {
        int16_t p = dy * 2 - dx;
        int16_t twoDy = 2 * dy;
        int16_t twoDyMinusDx = 2 * (dy - dx);
        while (x < x2) {
          x++;
          if (p < 0)
            p += twoDy;
          else {
            y++;
            p += twoDyMinusDx;
          }
          LCD_SetPoint(x, y, color);
        }
      } else {
        int16_t p = dx * 2 - dy;
        int16_t twoDx = 2 * dx;
        int16_t twoDxMinusDy = 2 * (dx - dy);
        while (y < y2) {
          y++;
          if (p < 0)
            p += twoDx;
          else {
            x++;
            p += twoDxMinusDy;
          }
          LCD_SetPoint(x, y, color);
        }
      }
    } else {
      if (dx > dy) {
        int16_t p = dy * 2 - dx;
        int16_t twoDy = 2 * dy;
        int16_t twoDyMinusDx = 2 * (dy - dx);
        while (x < x2) {
          x++;
          if (p < 0)
            p += twoDy;
          else {
            y--;
            p += twoDyMinusDx;
          }
          LCD_SetPoint(x, y, color);
        }
      } else {
        int16_t p = dx * 2 - dy;
        int16_t twoDx = 2 * dx;
        int16_t twoDxMinusDy = 2 * (dx - dy);
        while (y2 < y) {
          y--;
          if (p < 0)
            p += twoDx;
          else {
            x++;
            p += twoDxMinusDy;
          }
          LCD_SetPoint(x, y, color);
        }
      }
    }
  }
}

void LCD_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
    uint16_t color, uint8_t fill) {
  if (fill) {
    uint16_t i;
    if (x1 > x2) {
      i = x2;
      x2 = x1;
    } else
      i = x1;
    for (; i <= x2; i++)
      LCD_Draw_Line(i, y1, i, y2, color);
    return;
  }
  LCD_Draw_Line(x1, y1, x1, y2, color);
  LCD_Draw_Line(x1, y2, x2, y2, color);
  LCD_Draw_Line(x2, y2, x2, y1, color);
  LCD_Draw_Line(x2, y1, x1, y1, color);
}


void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc){
	static int day = 0;
HAL_RTC_GetDate(hrtc, &sDate, FORMAT_BIN);
HAL_RTC_GetTime(hrtc, &sTime, FORMAT_BIN);
	if(day != sDate.Date){
		day = sDate.Date;
		HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR2,sDate.Month);
		HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR3,sDate.Date);
		HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR4,sDate.Year);
	}

};

void HAL_RTCEx_RTCEventErrorCallback(RTC_HandleTypeDef *hrtc){

while(1){};

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
 FATFS fileSystem;
 FIL testFile;
  UINT testBytes;
  FRESULT res;
	char buf[100];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
	

HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
HAL_Delay(100);

  SSD1289_Init();
LCD_Clear(black);
		
TP_Init();	
 TouchPanel_Calibrate(0);
//HAL_UART_Transmit(&huart1,"start\n",strlen("start\n"),1);
//HAL_UART_Transmit(&huart1,"star1\n",strlen("star1\n"),1);
//HAL_UART_Transmit(&huart1,"star2\n",strlen("star2\n"),1);

	//MX_USB_DEVICE_Init();
	//HAL_Delay(1000);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
 
 f_mount(&fileSystem, SD_Path, 1);
 
		
		uint8_t path[13] = "testfile.txt";
    path[12] = '\0';

		
		
 	LCD_Clear(yellow);/*
  LCD_WriteString_5x7(20, 240 - 30, "Hello world.", red, yellow, 0, 1);
	LCD_WriteString_5x7(20, 240 - 30 - 20, "STM32F103VET6", green, yellow, 0, 1);
*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
/*	LCD_Clear(yellow);*/

  //LCD_Draw_Rectangle(50, 160, 150, 175, red, 1);
  //LCD_Draw_Rectangle(10, 10, 40, 40, blue, 1);
  //LCD_Draw_Rectangle(20, 20, 60, 60, white, 1);
		/*
		memset(buf,0,100);
		sprintf(buf, "Date: %02d/%02d/%02d", sDate.Date, sDate.Month, sDate.Year);
LCD_WriteString_5x7(50,100, buf, magneta, yellow,0, 2);
		memset(buf,0,100);
sprintf(buf, "Time: %02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
LCD_WriteString_5x7(50, 160, buf, magneta, yellow,0, 2);
//HAL_UART_Transmit(&huart1,(uint8_t*)&buf, strlen(buf), 10);
*/
		//Read_Ads7846();
		//LCD_TouchRead(&display);
	/*	int x,y;
		TP_GetAdXY(&x,&y); 
				sprintf(buf, "x %05d y %05d", x,y);
LCD_WriteString_5x7(50, 50, buf, magneta, yellow,0, 2);
		*/
		HAL_Delay(10);
		
		if(Read_Ads7846() != 0){
		getDisplayPoint(&display, Read_Ads7846(), &matrix ) ; // ???????? ?????
 					//	 TP_DrawPoint(display.x,display.y);	// ?????????? ?????

		sprintf(buf, "xS %03d yS %03d", display.x,display.y);
LCD_WriteString_5x7(50, 75, buf, magneta, yellow,0, 2);
			LCD_Draw_Rectangle(display.y,display.x,display.y+3,display.x+3,red,1);
		}else{
		//sprintf(buf, "xS NON yS NON");
		}
		
	/*	if(sTime.Seconds %10 ==0){
		     res = f_open(&testFile, (char*)path, FA_READ |FA_WRITE );
		uint16_t tmp = f_size(&testFile);
		res = f_lseek(&testFile, tmp);
			
		memset(buf,0,100);
		sprintf(buf, "Date: %02d/%02d/%02d Time: %02d:%02d:%02d\r\n\0", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);

    res = f_write(&testFile, buf, strlen(buf), &testBytes);
		f_sync(&testFile);
    res = f_close(&testFile);
*/
	//MX_USB_DEVICE_Init();
	/*HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);*/

		
	//	}
		
		/*for(int i = 1; i < 10; i++){
			char buf[255];
			memset(buf,0,255);
			sprintf(buf,"Tcvetkov Evgeniy %d",i);
			LCD_WriteString_5x7(50, 240 - 30 - 20 - 20*i, buf, magneta, glass,0, 1);
		}*/
		

/*
  LCD_WriteString_5x7(10, 240 - 30 - 20 - 20 - 20, "abcdefghijklmnopqrstuvwxyz",
      yellow, black, 0, 1);
  LCD_WriteString_5x7(10, 240 - 30 - 20 - 20 - 20 - 10,
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ", green, black, 0, 1);
  LCD_WriteString_5x7(10, 240 - 30 - 20 - 20 - 20 - 10 - 10,
      "!@#$%^&*()+-={[}]|;:'\"<,.>`~", blue, black, 0, 1);
	  LCD_WriteString_5x7(10, 240 - 30 - 20 - 20 - 20 - 10 - 10 -10,
      "!@#$%^&*()+-={[}]|;:'\"<,.>`~", blue, black, 1, 1);*/
	//	LCD_WriteString_5x7(10, 240 - 30 - 20 - 20 - 20 - 10 - 10 -10 - 10,
    //  "", blue, black, 1, 1);
			//FSMC_LCD_Write_Command(0x0022); 
		//HAL_Delay(1000);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

	
if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1) != TSTWORD){
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1, TSTWORD);

  sTime.Hours = 22;
  sTime.Minutes = 02;
  sTime.Seconds = 00;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  DateToUpdate.WeekDay = RTC_WEEKDAY_FRIDAY;
  DateToUpdate.Month = RTC_MONTH_APRIL;
  DateToUpdate.Date = 23;
  DateToUpdate.Year = 16;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,sDate.Month);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,sDate.Date);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4,sDate.Year);
} else {
		sDate.Month = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
		sDate.Date = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
		sDate.Year = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

}

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  HAL_SRAM_Init(&hsram1, &Timing, NULL);

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
