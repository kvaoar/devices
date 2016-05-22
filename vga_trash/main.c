/***************************************************************************
 * STM32 VGA demo
 * Copyright (C) 2012 Artekit Italy
 * http://www.artekit.eu
 * Written by Ruben H. Meleca
 
### main.c
 
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

***************************************************************************/

#include "stm32f10x.h"
#include "sys.h"
#include "video.h"
#include "gdi.h"
#include "space_invaders.h"
#include "math.h"

extern void demoInit(void);
static GDI_WINDOW test; 

void RCC_Configuration(void)
{
	/* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
							RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Configuration(void)
{

}

void drawCircle(int x0, int y0, int radius) 
{
	int x = 0;
	int y = radius;
	int delta = 1 - 2 * (radius);
	int error = 0;

	while(y >= 0) 
		{
		
		gdiPoint(0,x0 + x, y0 + y,0);
		gdiPoint(0,x0 + x, y0 - y,0);
		gdiPoint(0,x0 - x, y0 + y,0);
		gdiPoint(0,x0 - x, y0 - y,0);
		
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
}



void CRT_Draw_Ring(u16 r, u16 xpos, u16 ypos, u8 fill, float tt)
    {       
			u32 s = 0;
      static double phi = 0;
        if (1)
          {
            for(;r>0;r--)
            {
              float a = 0;
								u16 x =0, y = 0;  
								for(s = 0 ;s<((2*3.1415)/tt); s++, a+=tt)
									{     
										x=xpos+(r*sin(a+phi));
										y=ypos+(r*cos(a+phi));
										gdiPoint(0,x, y,fill); 
//phi +=0.000001;										
										
									}
            }
          } 
					
					//phi += 0.3;
//          else
//          {
//            float a = 0;
//      u16 x =0, y = 0;
//      
//      for(s = 0 ;s<(200*2*3.14); s++, a+=9)
//        {     
//          x=xpos+(r*sin(a));
//          y=ypos+(r*cos(a));
//          gdiPoint(0,x, y,1);     
//        }
//          }
   
         return;
     }

u32 ran(u32 p){
		u32 ran;
		while(p--)ran *=  rand();
return ran;
}


const double Pi = 3.14159;
double n,m;
int max =1;
int color;
 
void Draw(double x, double y, double L, u32 r, double a) {          // ??????? ????????? ????????
  if(L > max) 
		{
     L--;
		
//     moveto(x,y);
//     lineto((int)(x+L*cos(a)),(int)(y-L*sin(a)));
    
			gdiLine(0, x, y, (int)(x+r*cos(a)), (int)(y-r*sin(a)),1);
		
		x=x+r*cos(a);
    y=y-r*sin(a);
 
                                                // ??????????? ?????
    Draw(x,y,L,r,a+Pi/13);
		
    Draw(x,y,L,r,a-Pi/13);
  }
}




int main(void)
{
	
		u32 rx=0, ry=0, fx, fy, ff;
		
 	RCC_Configuration();
	GPIO_Configuration();

	test.caption= "WINDOWS 0.1";
	test.rc.x=20;
	test.rc.y=20;
   test.rc.h=40;
	 test.rc.w=80;
	test.style=GDI_WINCAPTION|GDI_WINCAPTION_LEFT;
	
	vidInit();
	sysInitSystemTimer();
	vidClearScreen();
//CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2,1, 2*3.1415/12);
		
//sysDelayMs(2000);
	//CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2,1, 2*3.1415/12);
//CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2, 2, 2*3.1415/12);
//	siInitialScreen();
//CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2, 1);
	//vidClearScreen();
	
//fx=1; fy = 1;
//rx=10;ry=10;
//ff = rand()%100;
//while(ff--){
//rx = rand()%(400);
//ry = rand()%(200);
//}

//	

				gdiDrawTextEx(30, 170, "KVAOAR",1);
gdiDrawTextEx(340, 170, "CASA",1);
gdiDrawTextEx(340, 30, "STM32F103",1);
gdiDrawTextEx(340, 50, "VGA OUT",1);
gdiDrawTextEx(340, 100, "PYTHAGORAS",1);
gdiDrawTextEx(340, 120, "TREE",1);
gdiDrawTextEx(30, 30, "22.05.16",1);

Draw( VID_PIXELS_X/2,VID_PIXELS_Y/2+50,18,7,1.57);
		
while(1)
	{



//		if((ff++%(rand()%200) )==0){
//			
//			
//		fx = rand()%2 ? 0:(rand()%2 ? -1:1);
//		fy =  rand()%2 ? 0:(rand()%2 ? -1:1);
//		}
//		
		
	

		
		
		
		
//}
//		ry++;
//		rx++;
//		fx=ran(1)%(400/10);
//		fy=ran(1)%(200/10);
//		
//		ff=rand(3)%2;
	//	CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2, 2, 2*3.1415/12);
		//drawCircle(VID_PIXELS_X/2, VID_PIXELS_Y/2, 70);
	//	CRT_Draw_Ring(100, VID_PIXELS_X/2, VID_PIXELS_Y/2,0, 1);
		
		//400*200
		
		
		
//#define	GDI_ROP_COPY			0
//#define	GDI_ROP_XOR				1
//#define	GDI_ROP_AND				2

		
		
		
		
			
  //  gdiDrawTextEx(VID_PIXELS_X/2, VID_PIXELS_Y/2, "STM32", 0);
	//	gdiCircle(VID_PIXELS_X/2, VID_PIXELS_Y/2, 50, 0);
	//	gdiLine(0, (VID_PIXELS_X/2), (VID_PIXELS_Y/2)-50, VID_PIXELS_X/2, VID_PIXELS_Y/2, 0);
//	  gdiPoint(0, rx, ry, 0);
//		
//		rx += fx;
//		ry += fy;
//		
//		
//	if ((rx >= 400 )||(gdiTestPoint( rx, ry)==1 )) {
//		//vidClearScreen();
//	//rx = rand()%(400);
//	//ry = rand()%(200);
//		fx = -fx;
//				fx += rand()%2 ? 0:(rand()%2 ? -1:1);
//		fy +=  rand()%2 ? 0:(rand()%2 ? -1:1);
//	}
//	
//	
//		if ((ry >= 200)||(gdiTestPoint( rx, ry)==1 )) {
//		//vidClearScreen();
//	//rx = rand()%(400);
//	//ry = rand()%(200);
//			fy = -fy;
//					fx += rand()%2 ? 0:(rand()%2 ? -1:1);
//		fy +=  rand()%2 ? 0:(rand()%2 ? -1:1);
//	}
		//sysDelayMs(1);
		//vidClearScreen();
	//	vidClearScreen();
	//	gdiRectangle((VID_PIXELS_X/2)-50, (VID_PIXELS_Y/2)-50, (VID_PIXELS_X/2)+50, (VID_PIXELS_Y/2)+50, 0);	
		
			//gdiPoint(0, VID_PIXELS_X/2, VID_PIXELS_Y/2, 0);
			//gdiLine(0, (VID_PIXELS_X/2),0, VID_PIXELS_X/2, (VID_PIXELS_Y), 0);
			
		
		
	}
	//demoInit();
	//siInit();

	return 0;
}
