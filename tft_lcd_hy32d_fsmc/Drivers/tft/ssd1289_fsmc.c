
#include "ssd1289_fsmc.h"


void delay(uint32_t delayTime)
{	
    uint32_t i;
    for(i = 0; i < delayTime; i++);
}
 
void Init_Periph()
{
	
}


__inline void Lcd_Write_Index(uint16_t index)
{
	*(uint16_t *) (LCD_REG) = index;	
}
////////////////////////

__inline void Lcd_Write_Data(uint16_t data)
{   
    *(uint16_t *) (LCD_DATA)= data; 	
}
///////////////////
uint16_t Lcd_Read_Data()
{
		uint16_t data = * (uint16_t *)(LCD_DATA);
		return data;	
}
////////////////////////
//???????????????	
uint16_t Lcd_Read_Reg(uint16_t reg_addr)
{
	volatile uint16_t data = 0;
	
	Lcd_Write_Index(reg_addr);
	data = Lcd_Read_Data();
	
	return data;
}
///////////////////////
//?????????? ????
void Lcd_Write_Reg(uint16_t reg,uint16_t value)
{	
    *(uint16_t *) (LCD_REG) = reg;	
    *(uint16_t *) (LCD_DATA) = value;
}
//////////////////
//????????????U
void Set_Cursor(uint16_t x_kur, uint16_t y_kur)
{	
	Lcd_Write_Reg(0x004e,x_kur);
	Lcd_Write_Reg(0x004f,y_kur);
	Lcd_Write_Index(0x0022);

}
////////////////////////
//??????????????????
void Lcd_Clear(uint16_t color)
{
	uint32_t index = 0;
	
	Set_Cursor(0,0);	
	
	  for(index=0;index < 76800;index++)
	  { 			
		  Lcd_Write_Data(color);
	  }
}
/////////////////////////////////////
//??????id ????????????????, ????????
/*void Get_Lcd_Id(uint16_t x, uint16_t y,uint16_t color, uint16_t phone, uint8_t size)
{
	 uint16_t data = 0;
	 uint8_t id [4] = {0};
	
	data = Lcd_Read_Reg(0x00);
	itoa( data, (char*) id, 16); //??????? ???? ?? ????	for (uint8_t k = 0; k < 4; k++)
	{
		Draw_Simbol(x ,y + 8*k*size,color,phone, &simbols[(id[k]-0x20)*8],size);
	}
}*/
///////////////////////////////////

void Init_SSD1289()
{
   delay(300);
 
    Lcd_Write_Reg(0x0007,0x0021);
    Lcd_Write_Reg(0x0000,0x0001);
    Lcd_Write_Reg(0x0007,0x0023);
    Lcd_Write_Reg(0x0010,0x0000);

    delay(300);
    Lcd_Write_Reg(0x007,0x0033);
    Lcd_Write_Reg(0x0011,0x6838);
    Lcd_Write_Reg(0x0002,0x0600);
		
	  Lcd_Clear(blue);
	
}
/////////////////////////////////////////////////
//????????????????????? ?????????, ???? ??????? ???
void Draw_Simbol ( uint16_t x, uint16_t y, uint16_t color, uint16_t phone, const uint8_t *sim, uint8_t size)
{
	 uint8_t j,i,k,p = 0;//??????? ??????????????

	for( j=0; j<8; j++)//? 8-?????
	{
		for( k=0; k< size; k++)//?????????? ???
		{			  
			Set_Cursor(x+(j*size)+k, y);//????? ????
			
			for( i=0; i<8; i++)//? 8-?????
			{
				//?? ??? ??? 1 - ??????? ??? size ??
				if(0x01 & (sim[j] >> (7 - i))) 
				{
					for( p=0; p< size; p++)
					{
						Lcd_Write_Data(color);
					}
				}
				//?? ??? ??? 0 - ??????? ?? size ??
				else 
				{
					for(p=0; p< size; p++)
					{
						 Lcd_Write_Data(phone);
					}
				}
			}
		}
	}

}
///////////////////////////////////
//????????????? ???????, ??? ?????????, ??? ???
void Draw_String (uint16_t start_x, uint16_t start_y, uint16_t color, uint16_t phone, uint8_t size, const uint8_t *string)
{
	uint16_t max_x,max_y,x1,y1,k = 0;//???? ??? ?????????????? ???????? ???
	
	if(size > 5) size=5;//??????????????????
	max_y = 320/(size*8);//????????? ?x
	max_x = 240/(size*8);//????????? ?y
	
	for(x1 =start_x; x1 < max_x; x1++)//?X
	{
		for(y1 =start_y; y1 < max_y; y1++)//?Y
		{
			//?????????????????
			Draw_Simbol(x1*8*size,y1*8*size,color,phone,simbols+((string[k])-32)*8,size);
			//??????????????-????? ???- ???????
			if(k++ > (strlen((char*) string))-2) return;
			
		}
	}
}
/////////////////////////////
//???????????????
void Draw_Point (uint16_t x,uint16_t y,uint16_t color,uint8_t size)
{
	uint8_t j,i = 0;

	for(j=0;j < size; j++)
	{
		Set_Cursor(x+j, y);
		for(i=0;i < size;i++)
		{
			Lcd_Write_Data(color);
		}
	}
}
//////////////////////////////////////////
//?????????k?? °???
void Draw_Line (uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color, uint8_t size)
{
	int deltaX = abs(x2 - x1);
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	int error2 = error * 2;
	
	for (;;)
	{
		Draw_Point(x1,y1,color,size);
		
		if(x1 == x2 && y1 == y2)
		break;
		
		
		
		if(error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		
		if(error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
}
/////////////////////////////////
//???????????????
void Draw_Horizont_Line(uint16_t x1,uint16_t y1,uint16_t y2,uint16_t color, uint8_t size)
{
	Draw_Line(x1, y1, x1, y2, color, size);
}
///////////////////////////////
//???????????????
void Draw_Vertical_Line(uint16_t x1,uint16_t x2,uint16_t y1,uint16_t color, uint8_t size)
{
	Draw_Line(x1, y1, x2, y1, color, size);
}
///////////////////////////////
//???????????
void Draw_Reactangle(uint16_t left,uint16_t top,uint16_t right,uint16_t bottom,uint16_t color, uint8_t size)
{
	Draw_Horizont_Line(top, left, right, color, size);
	Draw_Horizont_Line(bottom, left, right, color, size);
	Draw_Vertical_Line(top, bottom, left, color, size);
	Draw_Vertical_Line(top, bottom, right, color, size);
		
}
///////////////////////////////
//????????????????°???
/*void Draw_Circle (uint16_t x0,uint16_t y0,uint16_t radius,uint16_t color,uint8_t size)
{
	int x = 0;
	int y = radius;
	int delta = 2 - 2 * radius;
	int error = 0;
	while(y >= 0)
	{
		Draw_Point(x0 + x, y0 + y,color,size);
		Draw_Point(x0 + x, y0 - y,color,size);
		Draw_Point(x0 - x, y0 + y,color,size);
		Draw_Point(x0 - x, y0 - y,color,size);
		error = 2 * (delta + y) - 1;
		if(delta < 0 && error <= 0)
		{
			++x;
			delta += 2 * x + 1;
			continue;
		}
		error = 2 * (delta - x) - 1;
		if(delta > 0 && error > 0)
		{
			--y;
			delta += 1 - 2 * y;
			continue;
		}
		++x;
		delta += 2 * (x - y);
		--y;
	}
}*/
/////////////////////////////////////
//
void Draw_Circle1(unsigned int x,unsigned int y,char radius,char fill, char size, unsigned int color)
{
	int a_,b_,P;
	a_ = 0;
	b_ = radius;
	P = 1 - radius;
	while (a_ <= b_)
	{
		if(fill == 1)
		{
			Draw_Area(x-a_,y-b_,x+a_,y+b_,color);
			Draw_Area(x-b_,y-a_,x+b_,y+a_,color);
		}
		else
		{
			Draw_Point(a_+x, b_+y, color, size);
			Draw_Point(b_+x, a_+y, color, size);
			Draw_Point(x-a_, b_+y, color, size);
			Draw_Point(x-b_, a_+y, color, size);
			Draw_Point(b_+x, y-a_, color, size);
			Draw_Point(a_+x, y-b_, color, size);
			Draw_Point(x-a_, y-b_, color, size);
			Draw_Point(x-b_, y-a_, color, size);
		}
		if (P < 0 )
		{
			P = (P + 3) + (2* a_);
			a_ ++;
		}
		else
		{
			P = (P + 5) + (2* (a_ - b_));
			a_ ++;
			b_ --;
		}
	}
}


void Set_Work_Area(uint16_t y1, uint16_t x1, uint16_t y2, uint16_t x2)
{
	
	Lcd_Write_Reg(0x0044,((x2 << 8) | x1));
	Lcd_Write_Reg(0x0045,y1);
	Lcd_Write_Reg(0x0046,y2);
	Set_Cursor(x1, y1);
}


void Draw_Area(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom, uint16_t color)
{
	register uint16_t x,y;
	
	Set_Work_Area(left, top, right, bottom);

	for(y=top; y<=bottom; y++)
	{
		for(x=left; x<=right; x++)
		{
			Lcd_Write_Data(color);
	    }
	}
		
	Set_Work_Area(0, 0, 319, 239 );
	
}


void Draw_Triangle( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t size, uint16_t color)
{
	
	Draw_Line( x1, y1, x2, y2, color, size);
	Draw_Line( x2, y2, x3, y3, color, size);
	Draw_Line( x3, y3, x1, y1, color, size);
}
//////////////////////////////////////////////////
void Draw_Picture_Hor(uint16_t start_x,uint16_t start_y,uint16_t end_x,uint16_t end_y,const uint16_t *pic)
{
	uint16_t  x,y;
	for (y=0;y<(end_y);y++)
	{
		Set_Cursor(start_x,start_y+y);
		Lcd_Write_Index(0x0022);
		for (x=0;x<(end_x);x++) Lcd_Write_Data(*pic++);
	}
}
