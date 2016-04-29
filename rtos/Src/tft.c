#include "stm32f1xx_hal.h"
#include "tft.h"


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
