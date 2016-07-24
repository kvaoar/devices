/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.c
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "TouchPanel.h"
#include <stdio.h>

SPI_HandleTypeDef  hspi1;

/* Private variables ---------------------------------------------------------*/
Matrix matrix ;
Coordinate  display ;
uint8_t PMode2;
Example example;
My_color my_color;

void TP_CS(uint8_t i){if(i == 1)
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					else		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
uint8_t TP_INT_IN(){ if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET)return 1; else return 0;}

/* DisplaySample LCD×ø±êÉÏ¶ÔÓ¦µÄads7843²ÉÑùADÖµ Èç£ºLCD ×ø±ê45,45 Ó¦¸ÃµÄX Y²ÉÑùADC·Ö±ðÎª3388,920 */	
Coordinate ScreenSample[3];
/* LCDÉÏµÄ×ø±ê */
Coordinate DisplaySample[3] = {{45,45},{45,270},{190,190}};	 // 

//Coordinate DisplaySample[3] = {{20,20},{20,300},{220,160}};

/* Private define ------------------------------------------------------------*/
#define THRESHOLD 2   /* ²îÖµÃÅÏÞ */


void HexToBCD(uint16_t Num);

 //--------------------------------------
// Convet HEX 2 byte to 7-Segment code
//--------------------------------------
void HexToBCD(uint16_t Num)
{
uint16_t temp = Num;
uint16_t temp2;
temp2= temp%10; 
example.S0=(unsigned char)temp2; // ñàìàÿ ìëàäøàÿ öèôðà ïóñòü áóäåò5
temp =  temp/10; temp2=temp;// ïîëó÷àåì ïåðâûå 4 ÷èñëà
example.S1=(unsigned char)temp2%10; // ïîëó÷àåì ÷åòâåðòóþ öèôðó
temp =  temp/10; temp2=temp;	 // ïåðâûå 3 ÷èñëà
example.S2=(unsigned char)temp%10; // ïîëó÷àåì òðåòüþ öèôðó
temp =  temp/10; temp2=temp;	 // ïåðâûå 2 ÷èñëà
example.S3=(unsigned char)temp2%10; // ïîëó÷àåì âòîðóþ öèôðó
temp = (uint16_t)temp/10;	 // ïåðâûå 1 ÷èñëà
example.S4=(unsigned char)temp; // ïîëó÷àåì ïåðâóþ ñàìóþ ñòàðøóþ öèôðó
}	





/*******************************************************************************
* Function Name  : ADS7843_SPI_Init
* Description    : ADS7843 SPI ³õÊ¼»¯
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void ADS7843_SPI_Init(void) 
{ 
  
 GPIO_InitTypeDef GPIO_InitStruct;

  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  


  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  
	
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);
	
} 

/*******************************************************************************
* Function Name  : TP_Init
* Description    : ADS7843¶Ë¿Ú³õÊ¼»¯
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TP_Init(void) 
{ 
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* TP_CS */

	  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /* TP_IRQ */

	  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  TP_CS(1); 
	
  ADS7843_SPI_Init(); 
} 

/*******************************************************************************
* Function Name  : DelayUS
* Description    : ÑÓÊ±1us
* Input          : - cnt: ÑÓÊ±Öµ
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void DelayUS(uint32_t cnt)
{
  uint16_t i;
  for(i = 0;i<cnt;i++)
  {
     uint8_t us = 12; /* ÉèÖÃÖµÎª12£¬´óÔ¼ÑÓ1Î¢Ãë */    
     while (us--)     /* ÑÓ1Î¢Ãë	*/
     {
       ;   
     }
  }
}


/*******************************************************************************
* Function Name  : WR_CMD
* Description    : Ïò ADS7843Ð´Êý¾Ý
* Input          : - cmd: ´«ÊäµÄÊý¾Ý
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void WR_CMD (uint8_t cmd)  
{ 
  /* Wait for SPI1 Tx buffer empty */ 
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI1 data */ 
 // SPI_I2S_SendData(SPI1,cmd); 
  /* Wait for SPI1 data reception */ 
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI1 received data */ 
 // SPI_I2S_ReceiveData(SPI1); 
	
	HAL_SPI_Transmit(&hspi1,&cmd,1,100);
} 

/*******************************************************************************
* Function Name  : RD_AD
* Description    : ¶ÁÈ¡ADCÖµ
* Input          : None
* Output         : None
* Return         : ADS7843·µ»Ø¶þ×Ö½ÚÊý¾Ý
* Attention		 : None
*******************************************************************************/
static int RD_AD(void)  
{ 
  unsigned short buf,temp; 
  /* Wait for SPI1 Tx buffer empty */ 
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI1 data */ 
 // SPI_I2S_SendData(SPI1,0x0000); 
	uint16_t tmp = 0x0000;
	//HAL_SPI_Transmit(&hspi1,(uint8_t*)&tmp,1,100);
  /* Wait for SPI1 data reception */ 
  //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI1 received data */ 
  //temp=SPI_I2S_ReceiveData(SPI1); 
	HAL_SPI_Receive(&hspi1,(uint8_t*)&temp,1,100);
  buf=temp<<8; 
  DelayUS(1); 
  //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
  /* Send SPI1 data */ 
  //SPI_I2S_SendData(SPI1,0x0000); 
	//HAL_SPI_Transmit(&hspi1,(uint8_t*)&tmp,1,100);
  /* Wait for SPI1 data reception */ 
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI1 received data */ 
 // temp=SPI_I2S_ReceiveData(SPI1); 
 HAL_SPI_Receive(&hspi1,(uint8_t*)&temp,1,100);
  buf |= temp; 
  buf>>=3; 
  buf&=0xfff; 
  return buf; 
} 

/*******************************************************************************
* Function Name  : Read_X
* Description    : ¶ÁÈ¡ADS7843Í¨µÀX+µÄADCÖµ 
* Input          : None
* Output         : None
* Return         : ADS7843·µ»ØÍ¨µÀX+µÄADCÖµ
* Attention		 : None
*******************************************************************************/
int Read_X(void)  
{  
  int i; 
  TP_CS(0); 
  DelayUS(1); 
  WR_CMD(CHX); 
  DelayUS(1); 
  i=RD_AD(); 
  TP_CS(1); 
  return i;    
} 

/*******************************************************************************
* Function Name  : Read_Y
* Description    : ¶ÁÈ¡ADS7843Í¨µÀY+µÄADCÖµ
* Input          : None
* Output         : None
* Return         : ADS7843·µ»ØÍ¨µÀY+µÄADCÖµ
* Attention		 : None
*******************************************************************************/
int Read_Y(void)  
{  
  int i; 
  TP_CS(0); 
  DelayUS(1); 
  WR_CMD(CHY); 
  DelayUS(1); 
  i=RD_AD(); 
  TP_CS(1); 
  return i;     
} 

/*******************************************************************************
* Function Name  : TP_GetAdXY
* Description    : ¶ÁÈ¡ADS7843 Í¨µÀX+ Í¨µÀY+µÄADCÖµ
* Input          : None
* Output         : None
* Return         : ADS7843·µ»Ø Í¨µÀX+ Í¨µÀY+µÄADCÖµ 
* Attention		 : None
*******************************************************************************/
void TP_GetAdXY(int *x,int *y)  
{ 
  int adx,ady; 
  adx=Read_X(); 
  DelayUS(1); 
  ady=Read_Y(); 
  *x=adx; 
  *y=ady; 
} 

/*******************************************************************************
* Function Name  : TP_DrawPoint
* Description    : ÔÚÖ¸¶¨×ù±ê»­µã
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TP_DrawPoint(uint16_t Xpos,uint16_t Ypos)
{
 /* LCD_SetPoint(Xpos,Ypos,my_color);    
  LCD_SetPoint(Xpos+1,Ypos,my_color);
  LCD_SetPoint(Xpos,Ypos+1,my_color);
  LCD_SetPoint(Xpos+1,Ypos+1,my_color);	*/
}	

/*******************************************************************************
* Function Name  : DrawCross
* Description    : ÔÚÖ¸¶¨×ù±ê»­Ê®×Ö×¼ÐÇ
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void DrawCross(uint16_t Xpos,uint16_t Ypos)
{
 /* LCD_DrawLine(Xpos-15,Ypos,Xpos-2,Ypos,0xffff);
  LCD_DrawLine(Xpos+2,Ypos,Xpos+15,Ypos,0xffff);
  LCD_DrawLine(Xpos,Ypos-15,Xpos,Ypos-2,0xffff);
  LCD_DrawLine(Xpos,Ypos+2,Xpos,Ypos+15,0xffff);
  
  LCD_DrawLine(Xpos-15,Ypos+15,Xpos-7,Ypos+15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos-15,Ypos+7,Xpos-15,Ypos+15,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos-15,Ypos-15,Xpos-7,Ypos-15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos-15,Ypos-7,Xpos-15,Ypos-15,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos+7,Ypos+15,Xpos+15,Ypos+15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos+15,Ypos+7,Xpos+15,Ypos+15,RGB565CONVERT(184,158,131));

  LCD_DrawLine(Xpos+7,Ypos-15,Xpos+15,Ypos-15,RGB565CONVERT(184,158,131));
  LCD_DrawLine(Xpos+15,Ypos-15,Xpos+15,Ypos-7,RGB565CONVERT(184,158,131));	  	*/
}	
	
/*******************************************************************************
* Function Name  : Read_Ads7846
* Description    : µÃµ½ÂË²¨Ö®ºóµÄX Y
* Input          : None
* Output         : None
* Return         : Coordinate½á¹¹ÌåµØÖ·
* Attention		 : None
*******************************************************************************/
Coordinate *Read_Ads7846(void)
{
  static Coordinate  screen;
  int m0,m1,m2,TP_X[1],TP_Y[1],temp[3];
  uint8_t count=0;
  int buffer[2][9]={{0},{0}};  /* ×ø±êXºÍY½øÐÐ¶à´Î²ÉÑù */
  ADS7843_SPI_Init(); 		
  do					       /* Ñ­»·²ÉÑù9´Î */
  {		   
    TP_GetAdXY(TP_X,TP_Y);  
	buffer[0][count]=TP_X[0];  
	buffer[1][count]=TP_Y[0];
	count++;  
  }
  while((TP_INT_IN()==0)&& count<9);  /* TP_INT_INÎª´¥ÃþÆÁÖÐ¶ÏÒý½Å,µ±ÓÃ»§µã»÷´¥ÃþÆÁÊ±TP_INT_IN»á±»ÖÃµÍ */
  if(count==9)   /* ³É¹¦²ÉÑù9´Î,½øÐÐÂË²¨ */ 
  {
while(1);		
    /* Îª¼õÉÙÔËËãÁ¿,·Ö±ð·Ö3×éÈ¡Æ½¾ùÖµ */
    temp[0]=(buffer[0][0]+buffer[0][1]+buffer[0][2])/3;
	temp[1]=(buffer[0][3]+buffer[0][4]+buffer[0][5])/3;
	temp[2]=(buffer[0][6]+buffer[0][7]+buffer[0][8])/3;
	/* ¼ÆËã3×éÊý¾ÝµÄ²îÖµ */
	m0=temp[0]-temp[1];
	m1=temp[1]-temp[2];
	m2=temp[2]-temp[0];
	/* ¶ÔÉÏÊö²îÖµÈ¡¾ø¶ÔÖµ */
	m0=m0>0?m0:(-m0);
    m1=m1>0?m1:(-m1);
	m2=m2>0?m2:(-m2);
	/* ÅÐ¶Ï¾ø¶Ô²îÖµÊÇ·ñ¶¼³¬¹ý²îÖµÃÅÏÞ£¬Èç¹ûÕâ3¸ö¾ø¶Ô²îÖµ¶¼³¬¹ýÃÅÏÞÖµ£¬ÔòÅÐ¶¨Õâ´Î²ÉÑùµãÎªÒ°µã,Å×Æú²ÉÑùµã£¬²îÖµÃÅÏÞÈ¡Îª2 */
	if( m0>THRESHOLD  &&  m1>THRESHOLD  &&  m2>THRESHOLD ) return 0;
	/* ¼ÆËãËüÃÇµÄÆ½¾ùÖµ£¬Í¬Ê±¸³Öµ¸øscreen */ 
	if(m0<m1)
	{
	  if(m2<m0) 
	    screen.x=(temp[0]+temp[2])/2;
	  else 
	    screen.x=(temp[0]+temp[1])/2;	
	}
	else if(m2<m1) 
	  screen.x=(temp[0]+temp[2])/2;
	else 
	  screen.x=(temp[1]+temp[2])/2;

	/* Í¬ÉÏ ¼ÆËãYµÄÆ½¾ùÖµ */
    temp[0]=(buffer[1][0]+buffer[1][1]+buffer[1][2])/3;
	temp[1]=(buffer[1][3]+buffer[1][4]+buffer[1][5])/3;
	temp[2]=(buffer[1][6]+buffer[1][7]+buffer[1][8])/3;
	m0=temp[0]-temp[1];
	m1=temp[1]-temp[2];
	m2=temp[2]-temp[0];
	m0=m0>0?m0:(-m0);
	m1=m1>0?m1:(-m1);
	m2=m2>0?m2:(-m2);
	if(m0>THRESHOLD&&m1>THRESHOLD&&m2>THRESHOLD) return 0;

	if(m0<m1)
	{
	  if(m2<m0) 
	    screen.y=(temp[0]+temp[2])/2;
	  else 
	    screen.y=(temp[0]+temp[1])/2;	
    }
	else if(m2<m1) 
	   screen.y=(temp[0]+temp[2])/2;
	else
	   screen.y=(temp[1]+temp[2])/2;

	return &screen;
  }  
  return 0; 
}
	 
/* ÏÂÃæÊÇ´¥ÃþÆÁµ½Òº¾§ÆÁ×ø±ê±ä»»µÄ×ª»»º¯Êý */
/* Ö»ÓÐÔÚLCDºÍ´¥ÃþÆÁ¼äµÄÎó²î½Ç¶È·Ç³£Ð¡Ê±,²ÅÄÜÔËÓÃÏÂÃæ¹«Ê½ */


/*******************************************************************************
* Function Name  : setCalibrationMatrix
* Description    : ¼ÆËã³ö K A B C D E F
* Input          : None
* Output         : None
* Return         : ·µ»Ø1±íÊ¾³É¹¦ 0Ê§°Ü
* Attention		 : None
*******************************************************************************/
FunctionalState setCalibrationMatrix( Coordinate * displayPtr,
                          Coordinate * screenPtr,
                          Matrix * matrixPtr)
{

  FunctionalState retTHRESHOLD = ENABLE ;
  /* K£½(X0£­X2) (Y1£­Y2)£­(X1£­X2) (Y0£­Y2) */
  matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                       ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
  if( matrixPtr->Divider == 0 )
  {
    retTHRESHOLD = DISABLE;
  }
  else
  {
    /* A£½((XD0£­XD2) (Y1£­Y2)£­(XD1£­XD2) (Y0£­Y2))£¯K	*/
    matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
	/* B£½((X0£­X2) (XD1£­XD2)£­(XD0£­XD2) (X1£­X2))£¯K	*/
    matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                    ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;
    /* C£½(Y0(X2XD1£­X1XD2)+Y1(X0XD2£­X2XD0)+Y2(X1XD0£­X0XD1))£¯K */
    matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
    /* D£½((YD0£­YD2) (Y1£­Y2)£­(YD1£­YD2) (Y0£­Y2))£¯K	*/
    matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    /* E£½((X0£­X2) (YD1£­YD2)£­(YD0£­YD2) (X1£­X2))£¯K	*/
    matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                    ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
    /* F£½(Y0(X2YD1£­X1YD2)+Y1(X0YD2£­X2YD0)+Y2(X1YD0£­X0YD1))£¯K */
    matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
  }
  return( retTHRESHOLD ) ;
}

/*******************************************************************************
* Function Name  : getDisplayPoint
* Description    : Í¨¹ý K A B C D E F °ÑÍ¨µÀX YµÄÖµ×ª»»ÎªÒº¾§ÆÁ×ø±ê
* Input          : None
* Output         : None
* Return         : ·µ»Ø1±íÊ¾³É¹¦ 0Ê§°Ü
* Attention		 : None
*******************************************************************************/
FunctionalState getDisplayPoint(Coordinate * displayPtr,
                     Coordinate * screenPtr,
                     Matrix * matrixPtr )
{
  FunctionalState retTHRESHOLD =ENABLE ;

  if( matrixPtr->Divider != 0 )
  {
    /* XD = AX+BY+C */        
    displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                      (matrixPtr->Bn * screenPtr->y) + 
                       matrixPtr->Cn 
                    ) / matrixPtr->Divider ;
	/* YD = DX+EY+F */        
    displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                      (matrixPtr->En * screenPtr->y) + 
                       matrixPtr->Fn 
                    ) / matrixPtr->Divider ;
  }
  else
  {
    retTHRESHOLD = DISABLE;
  }
  return(retTHRESHOLD);
} 

/*******************************************************************************
* Function Name  : TouchPanel_Calibrate
* Description    : Ð£×¼´¥ÃþÆÁ
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TouchPanel_Calibrate(uint8_t CalSiNo)
{
  uint8_t i;
  Coordinate * Ptr;

	GetFormat();

  for(i=0;i<3;i++)
  {
  // LCD_Clear(Black);
  if(!CalSiNo){	// êàëèáðîâêà ïðîïèñàíà çàðàííåå
//   	ScreenSample[0].x = 400; ScreenSample[1].x = 3535; ScreenSample[2].x = 1957;
//	ScreenSample[0].y = 560; ScreenSample[1].y = 604; ScreenSample[2].y	=3528;

//	äëÿ ìîåãî äèñïëåÿ óñòàíîâèòü ñëåäóþùèå ïàðàìåòðû êàëèáðîâêè ýêðàíà ïî-óìîë÷àíèþ
 	ScreenSample[0].x = 0xD10; ScreenSample[1].x = 0x355; ScreenSample[2].x = 0x06D2;
	ScreenSample[0].y = 0x0369; ScreenSample[1].y = 0x35B; ScreenSample[2].y	= 0x0C36;


   }
   else {  // êàëèáðîâêó äåëàåì çàíîãî

   //GUI_Text(10,10,"Touch crosshair to calibrate",0xffff,Black);
   HAL_Delay(500);
   DrawCross(DisplaySample[i].x,DisplaySample[i].y);
   do
   {
   Ptr=Read_Ads7846();
   }
   while( Ptr == (void*)0 );
   ScreenSample[i].x= Ptr->x; ScreenSample[i].y= Ptr->y;

  }	}
  setCalibrationMatrix( &DisplaySample[0],&ScreenSample[0],&matrix ) ;  /* ËÍÈëÖµµÃµ½²ÎÊý */	   
  //LCD_Clear(my_color);
} 

//***********************************************
uint8_t LCD_TouchRead(Coordinate * displayLCD){
	uint16_t px,py,x,y;
	int p,f;


	GetFormat();							 /*obtiene valor de formato de pantalla activo*/
	p=0;
	f=0;

	while(f<30){
		getDisplayPoint(&display, Read_Ads7846(), &matrix ) ;	 /*lee la posición del puntero sobre la pantalla*/
		px=display.x;
		py=display.y;
		if((PMode2==0)&&(py<400)){x=px,y=py;p=1;break;}
		if((PMode2==1)&&(px<400)){x=px,y=py;p=1;break;}
		f++;
	}

	if(f>=30){p=0;x=400;y=400;}
	displayLCD->x = x;
  displayLCD->y =	y;

  return(p);
	}
//*********************************
void GetFormat(void)				  /*obtiene valor PMode2 de Formato de pantalla*/
	{
	PMode2=LCD_FormatRead();
	}
//*********************************
uint8_t LCD_FormatRead(void){
	uint8_t value;
	value=1;  /*Pmode=1 Landscape vertical, Pmode=0 Landscape Apaisada*/	//PMode;
	return(value);
}//********************************




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
