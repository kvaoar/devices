/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

IRDA_HandleTypeDef hirda1;
IRDA_HandleTypeDef hirda2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DBUFSZ	(32)
uint8_t usb_to_toslink[DBUFSZ+1];
uint16_t UtoT_toslink_pos = 0;
uint16_t UtoT_usb_pos = 0;
uint8_t toslink_to_usb[DBUFSZ+1];
uint8_t irda_rx_char = 0;
uint16_t TtoU_usb_pos = 0;
uint16_t TtoU_toslink_pos = 0;

volatile uint8_t half = 0;
volatile uint8_t full = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_IRDA_Init(void);
static void MX_USART2_IRDA_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//void HAL_IRDA_RxHalfCpltCallback(IRDA_HandleType111Def *hirda){half=1;}
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda){
	T_rx(irda_rx_char);
	HAL_IRDA_Receive_IT(&hirda1,&irda_rx_char,1);

}
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda){ }
/* USER CODE END 0 */


void dbg(char* str, uint16_t val){
	char buf[10];
	memset(buf,0,10);
	sprintf(buf,"%s = %d\r\n",str,val);
	CDC_Transmit_FS(buf,strlen(buf));
	HAL_Delay(1);
}
void s_dbg(char*str,uint16_t len){
	char buf[255];
	memset(buf,'*',255);
	buf[0]='[';
	memcpy(&buf[1],str,len);
	buf[len+1]=']';
	buf[len+2]='\r';
	buf[len+3]='\n';
	for(int i = 1; i <= len; i++) if(!isalnum(buf[i]))buf[i]='*';
	CDC_Transmit_FS(buf,len+4);
	HAL_Delay(1);
}

void TtoU(){
	uint16_t toslink_now = TtoU_toslink_pos;
	//uint16_t b = TtoU_usb_pos;

	if(toslink_now == TtoU_usb_pos) return;
	if(toslink_now >TtoU_usb_pos){
		uint16_t d_siz = toslink_now-TtoU_usb_pos;
		CDC_Transmit_FS(&toslink_to_usb[TtoU_usb_pos],d_siz);
		TtoU_usb_pos = toslink_now;
	}
	if(toslink_now < TtoU_usb_pos){
		uint16_t first_len = DBUFSZ-TtoU_usb_pos;
		CDC_Transmit_FS(&toslink_to_usb[TtoU_usb_pos],first_len);
		uint16_t second_len = toslink_now;
		CDC_Transmit_FS(&toslink_to_usb[0],second_len);
		TtoU_usb_pos = toslink_now;
	}

}



void UtoT(){
	uint16_t usb_now = UtoT_usb_pos;
	if(usb_now  == UtoT_toslink_pos) return;
	if(usb_now  > UtoT_toslink_pos){
		uint16_t data_size = usb_now -UtoT_toslink_pos;
		HAL_IRDA_Transmit(&hirda2,&usb_to_toslink[UtoT_toslink_pos],data_size,20);
		UtoT_toslink_pos = usb_now ;
	}
	if(usb_now  < UtoT_toslink_pos){
		uint16_t first_len = DBUFSZ-UtoT_toslink_pos;
		HAL_IRDA_Transmit(&hirda2,&usb_to_toslink[UtoT_toslink_pos],first_len,20);
		uint16_t second_len = usb_now ;
		HAL_IRDA_Transmit(&hirda2,&usb_to_toslink[0],second_len,20);
		UtoT_toslink_pos = usb_now ;
	}

}

void U_rx(uint8_t* Buf, uint16_t Len){
	uint32_t new_usb_pos = Len + UtoT_usb_pos;
	if(new_usb_pos <  DBUFSZ){
	  memcpy(&usb_to_toslink[UtoT_usb_pos],Buf,Len);
	  UtoT_usb_pos = new_usb_pos;
	} else {
		uint16_t first_len = DBUFSZ - UtoT_usb_pos;
	  memcpy(&usb_to_toslink[UtoT_usb_pos],Buf,first_len);
	  uint16_t second_len = Len - first_len;
	  memcpy(&usb_to_toslink[0],&Buf[first_len],second_len);
	  UtoT_usb_pos = second_len;

	}

	//s_dbg(usb_to_toslink,DBUFSZ);


}

void T_rx(char c){

	toslink_to_usb[TtoU_toslink_pos] = irda_rx_char;
	TtoU_toslink_pos++;

	if(TtoU_toslink_pos >= DBUFSZ) TtoU_toslink_pos = 0;


}

int main(void)
{

  /* USER CODE BEGIN 1 */
uint32_t old_transmit_time = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
   MX_DMA_Init();
  // MX_TIM2_Init();
   MX_USART1_IRDA_Init();
   MX_USART2_IRDA_Init();
   MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
   memset(usb_to_toslink,0,DBUFSZ);
   memset(toslink_to_usb,0,DBUFSZ);
   HAL_Delay(100);
  	HAL_GPIO_WritePin(USBEN_GPIO_Port, USBEN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

  	HAL_IRDA_Receive_IT(&hirda1,&irda_rx_char,1);
  	old_transmit_time = HAL_GetTick();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    	uint32_t now = HAL_GetTick();
    	if(now - old_transmit_time > 10){
    		old_transmit_time = now;
    		UtoT();
    		TtoU();

    	}

    /* USER CODE END WHILE */

    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1263;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/* USART1 init function */
static void MX_USART1_IRDA_Init(void)
{

  hirda1.Instance = USART1;
  hirda1.Init.BaudRate = 9600;
  hirda1.Init.WordLength = IRDA_WORDLENGTH_8B;
  hirda1.Init.Parity = IRDA_PARITY_NONE;
  hirda1.Init.Mode = IRDA_MODE_RX;
  hirda1.Init.Prescaler = 1;
  hirda1.Init.IrDAMode = IRDA_POWERMODE_NORMAL;
  if (HAL_IRDA_Init(&hirda1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_IRDA_Init(void)
{

	hirda2.Instance = USART2;
	hirda2.Init.BaudRate = 9600;
	hirda2.Init.WordLength = IRDA_WORDLENGTH_8B;
	hirda2.Init.Parity = IRDA_PARITY_NONE;
	hirda2.Init.Mode = IRDA_MODE_TX;
	hirda2.Init.Prescaler = 1;
	hirda2.Init.IrDAMode = IRDA_POWERMODE_NORMAL;
  if (HAL_IRDA_Init(&hirda2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USBEN_GPIO_Port, USBEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USBEN_Pin */
  GPIO_InitStruct.Pin = USBEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USBEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
