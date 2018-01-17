/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



volatile  u8 Counter;

volatile  u8 rled_blink = 0;
volatile  u8 gled_blink = 0;
u8 led_period = 128;
u8 led_ontime = 10;

volatile uint16_t Conversion_Value;


/**
  * @brief  Configure TIM2 peripheral in PWM mode
  * @param  None
  * @retval None
  */

void TIM2_Config(void)
{
  
  uint16_t AFRval=FLASH_ReadOptionByte(0x4803);
if(!(AFRval&0x0001))
{
FLASH_ProgramOptionByte(0x4803, (uint8_t)(AFRval&0x0001));
}

  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_1, 80-1);
  TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,25, TIM2_OCPOLARITY_LOW);
  TIM2_OC1PreloadConfig(ENABLE);
  TIM2_ARRPreloadConfig(ENABLE);
  //TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  TIM2_Cmd(ENABLE);
}

void TIMER4_Configuration(void)
{
  TIM4_DeInit();
  TIM4_TimeBaseInit(TIM4_PRESCALER_32, 0xFA );
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
}

/**
  * @brief  Configure ADC2 Continuous Conversion with End Of Conversion interrupt 
  *         enabled .
  * @param  None
  * @retval None
  */
void ADC_Config()
{
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);
  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_5, ADC1_PRESSEL_FCPU_D18, \
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL5,\
            DISABLE);
  ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
  ADC1_StartConversion();
}

void main(void)
{
  GPIO_Init(PWM_PORT, PWM_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GLED_PORT, GLED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(RLED_PORT, RLED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  TIM2_Config(); 
  TIMER4_Configuration();
  
  ADC_Config();
  enableInterrupts();

  while (1)
  {
    
    
    for(u32 i = 0; i < 0xFFF; i++) {};
    if(Conversion_Value < 350){
      rled_blink = 1;
      gled_blink = 0;
      led_ontime = 35;
      TIM2_SetCompare1(35);
    }
    else {
      rled_blink = 0;
      gled_blink = 1;
      
      if(Conversion_Value/10 < 80){
        led_ontime = Conversion_Value/8;
        TIM2_SetCompare1(Conversion_Value/10);
      }
      else{
        TIM2_SetCompare1(80);
        led_ontime = led_period;
      }
    }
  }
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
