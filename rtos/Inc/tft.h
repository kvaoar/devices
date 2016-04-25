#include "stm32f1xx_hal.h"

// FSMC
#define FSMC_LCD_INSTANCE &hsram1
#define FSMC_LCD_DATA 		(uint32_t*)0x60020000
#define FSMC_LCD_COMMAND  (uint32_t*)0x60000000

// SSD1289 Initialize Sequence
// R01h Display Control Register
#define LCD_R01_RL 0
#define LCD_R01_REV 1
#define LCD_R01_CAD 0
#define LCD_R01_BGR 0
#define LCD_R01_SM 0
#define LCD_R01_TB 1
#define LCD_R01_MUX 319

void FSMC_LCD_Write_Command(uint16_t command);
void FSMC_LCD_Write_Data(uint16_t data);
uint16_t FSMC_LCD_Read_Data(void);
void FSMC_LCD_Write_Register(uint16_t reg, uint16_t val);

void LCD_BACKLIGHT_ON(void);
void LCD_BACKLIGHT_OFF(void);

void SSD1289_Init(void);

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
#define glass     0x0000

void LCD_SetCursor(uint16_t x, uint16_t y);
void LCD_Clear(uint16_t color);
void LCD_SetPoint(uint16_t x, uint16_t y, uint16_t Color);
void LCD_SetArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_WriteChar5x7(uint16_t x, uint16_t y, char c, uint16_t t_color, uint16_t b_color, uint8_t rot, uint8_t zoom);
void LCD_WriteString_5x7(uint16_t x, uint16_t y, char *text, uint16_t charColor, uint16_t b_color, uint8_t rot, uint8_t zoom);
void LCD_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill);
