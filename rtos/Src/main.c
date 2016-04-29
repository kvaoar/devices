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
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "tft.h"

const int TSTWORD = 201;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

UART_HandleTypeDef huart1;
SRAM_HandleTypeDef hsram1;
osThreadId defaultTaskHandle;
osThreadId TaskTFTHandle;
osThreadId TaskFATFSHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static char gbuf[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_FSMC_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskTFT(void const * argument);
void StartTaskFATFS(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc){
	
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
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

void MX_RTC_BACKUP_Init(void){
	
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
	
	if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1) != TSTWORD){
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1, TSTWORD);

  sTime.Hours = 22;
  sTime.Minutes = 02;
  sTime.Seconds = 00;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 23;
  sDate.Year = 16;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,sDate.Month);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,sDate.Date);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4,sDate.Year);
} else {
		sDate.Month = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
		sDate.Date = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
		sDate.Year = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FSMC_Init();

  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskTFT */
  osThreadDef(TaskTFT, StartTaskTFT, osPriorityHigh, 0, 256);
  TaskTFTHandle = osThreadCreate(osThread(TaskTFT), NULL);

  /* definition and creation of TaskFATFS */
  osThreadDef(TaskFATFS, StartTaskFATFS, osPriorityNormal, 0, 1024);
  TaskFATFSHandle = osThreadCreate(osThread(TaskFATFS), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
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

  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
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

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
uint8_t sec = 0;
RTC_TimeTypeDef	sTime;
RTC_DateTypeDef	sDate;
char buf[50];
	sprintf(buf, "idle run\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
  /* Infinite loop */
  for(;;)
  {	
		HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
		//memset(buf,0,255);
		if(sec != sTime.Seconds){
			sec = sTime.Seconds;
		memset(buf,0,50);
		sprintf(buf, "Date: %02d/%02d/%02d Time: %02d:%02d:%02d\r\n\0", sDate.Date, sDate.Month, 
		sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
		}
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* StartTaskTFT function */
void StartTaskTFT(void const * argument)
{
  /* USER CODE BEGIN StartTaskTFT */
RTC_TimeTypeDef	sTime;
RTC_DateTypeDef	sDate;
char buf[100];
	
	sprintf(buf, "tft run\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
SSD1289_Init();
LCD_Clear(yellow);
	memset(buf,0,100);
  LCD_WriteString_5x7(20, 240 - 30, "Hello world.", red, yellow, 0, 1);
	LCD_WriteString_5x7(20, 240 - 30 - 20, "STM32F103VET6", green, yellow, 0, 1);
  /* Infinite loop */
  for(;;)
  {
		
		HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
		memset(buf,0,100);
		sprintf(buf, "Date: %02d/%02d/%02d", sDate.Date, sDate.Month, sDate.Year);
		LCD_WriteString_5x7(50,100, buf, magneta, yellow,0, 2);
		memset(buf,0,100);
		sprintf(buf, "Time: %02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
		LCD_WriteString_5x7(50, 160, buf, magneta, yellow,0, 2);
		LCD_WriteString_5x7(50, 50, gbuf, green, yellow,0, 2);
		
    osDelay(500);
  }
  /* USER CODE END StartTaskTFT */
}

/* StartTaskFATFS function */
void StartTaskFATFS(void const * argument)
{
  /* USER CODE BEGIN StartTaskFATFS */
  /* Infinite loop */
	UINT count = 0;
uint32_t i = 1;
static	FATFS fileSystem;
static	FIL testFile;
	FRESULT res = FR_OK;
	char buf[100];
	sprintf(gbuf, "fat run");
			//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
	
	do{
	osDelay(1000);
//	sprintf(SD_Path,"0:/\0");
	res = f_mount(&fileSystem, SD_Path, 1);
		sprintf(gbuf, "fat mnt %i",res);
		
    osDelay(1000);
	//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);

	/*if(res == FR_NO_FILESYSTEM){ 
		res = f_mkfs("", 0, 0);
		sprintf(gbuf, "fat mkfs %i",res);
		
    osDelay(1000);
		//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
		
		res = f_mount(&fileSystem, SD_Path, 1);
		sprintf(gbuf, "fat mnt %i",res);
		
    osDelay(1000);
	//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
	
}*/
	}while (res!= FR_OK);
	
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
  res = f_open(&testFile, "testfile.txt", FA_OPEN_ALWAYS | FA_READ |FA_WRITE );
			sprintf(gbuf, "fat open %i",res);
	
    osDelay(1000);
			//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
	/*uint16_t tmp = f_size(&testFile);
	f_lseek(&testFile, tmp);
			*/
	for(;;)
  {
		//if(i > 100000) vTaskDelete(TaskFATFSHandle);
		if(i%100 == 0){
			sprintf(&gbuf[9], "fat wr %i", i);
			
    osDelay(1000);
			//HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
		}
		memset(buf,0,100);
		sprintf(buf, "%lu\r\n", i++);
    res = f_write(&testFile, buf, strlen(buf), &count);
		if( res != FR_OK) break;
		f_sync(&testFile);
    //f_close(&testFile);
    osDelay(10);
  }
  /* USER CODE END StartTaskFATFS */
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
