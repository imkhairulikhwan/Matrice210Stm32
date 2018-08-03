
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "math.h"

/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery.h"
#include <list>

#include "crossSearch.h"
#include "measTreatement.h"
#include <Log.h>
#include <UartSender.h>
#include <UartFrame.h>

//#include "mbed.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*Defines------------------*/
#define ADC_charNb 4

/* Variables---------------*/
/*freq controle*/
uint32_t period;
char actualFreq [15];
uint8_t freqSelector=0;


/*button*/
bool antiRebondFlag = 0;

/*adc*/
char displayBuffer [ADC_charNb];//for displaying
uint8_t title_xPos = 10;
uint8_t title_yPos = 120;
uint8_t loading_xPos = 10;
uint8_t loading_yPos = 152;
int32_t adcValue;

#define BUFFER_SIZE 1
uint8_t buffer[BUFFER_SIZE];

uint8_t uart_rx_buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void change_ADCFreq();


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
crossSearch searchAlgo;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  //* Timer1 init
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  period = htim1.Instance->ARR;

  //* LCD init
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

  //* LED init for LCD measure
  BSP_LED_Init(DISCO_LED3);

  // Uart config
  UartSender::getInstance()->config(&huart1, &huart5);

  //*Algo init
  //searchAlgo.init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	LOG("Code launched");
	HAL_ADC_Start_IT(&hadc1);	// Start ADC isr
	HAL_UART_Receive_IT(&huart5, &uart_rx_buffer, 1);	// start UART isr
	BSP_LCD_DisplayStringAt(title_xPos, title_yPos, (uint8_t*) "Nivitec", CENTER_MODE);
	char loading[] = {'-', '\\', '|', '/' };
	int loadingIndex = 0;
  	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  		// Send continuously ADC value
  		UartFrame frame;
		frame.pushInt(0);
		frame.pushInt(adcValue);	//
		frame.send();
		Log::debug("Send value");
		char line[2];
		line[0] = loading[loadingIndex];
		line[1] = '\0';
		loadingIndex = (++loadingIndex)%4;
		BSP_LCD_DisplayStringAt(loading_xPos, loading_yPos, reinterpret_cast<uint8_t*>(line), CENTER_MODE);
		HAL_Delay(1000);
		//*/

  		/*/ Search algo
		BSP_LCD_DisplayStringAt(freq_xPos, freq_yPos, (uint8_t*) "20Hz", CENTER_MODE);
		//Measures treatment
		BSP_LCD_ClearStringLine(5);
		//BSP_LCD_ClearStringLine(6);
		sprintf(displayBuffer, "%lu", new_ADCMeas);
		BSP_LCD_DisplayStringAt(mes_xPos, mes_yPos, (uint8_t*) displayBuffer, CENTER_MODE);

		searchAlgo.process();
		//*/
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 432;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
	{
		Log::debug("UART_RxCpltCallback : %c", uart_rx_buffer);
		HAL_UART_Receive_IT(&huart5, &uart_rx_buffer, 1);
	}
	/**
	  * @brief new value for TIM1 counter register to change ADC measure freq
	  * @param None
	  * @retval None
	  */
	void change_ADCFreq()
	{
		period /= 10;
		freqSelector += 1;
		if(period <= 1 )
		{
			period = 2250;
			freqSelector = 0;
		}

		__HAL_TIM_SET_AUTORELOAD(&htim1, period);
		//htim1.Instance->ARR = period;
	}

	/**
	  * @brief EXTI line detection callbacks
	  * @param GPIO_Pin: Specifies the pins connected EXTI line
	  * @retval None
	  */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		/*/ Send measure test
		UartFrame frame;
		frame.pushInt(0);
		frame.pushInt((uint32_t)(rand()/100000.0));
		frame.send();
		Log::debug("Send measure");
		while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
			;
		//*/

		/*/ Search algo managment
		if(antiRebondFlag == 0)
		{
			//Start ADC
			// ADC init
			HAL_ADC_Start_IT(&hadc1);

			//Start SM for measure
			searchAlgo.startMeasure();

			//clear to display
			BSP_LCD_ClearStringLine(5);
			BSP_LCD_ClearStringLine(6);

			// starting timer for debouncer
			HAL_TIM_Base_Start_IT(&htim14);
			antiRebondFlag = 1;
		}
	//*/
	}

	/**
	  * @brief ADC line detection callbacks
	  * @param hadc: Specifies the measuring ADC
	  * @retval None
	  */
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		__disable_irq();

		// ADC reading
		adcValue = HAL_ADC_GetValue(hadc);
		//*/

		/*/ Search algo
		// To print ADC value on LCD
		new_ADCMeas = HAL_ADC_GetValue(hadc);

		// adding the ADC measure to the active list
		searchAlgo.add(new_ADCMeas);
		//*/

		// toggle LED for freq measure
		BSP_LED_Toggle(DISCO_LED3);

		__enable_irq();

	}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM14)
	{
		antiRebondFlag = 0;
		HAL_TIM_Base_Stop(&htim14);
		__HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
		//BSP_LCD_DisplayStringAtLine(7, (uint8_t*) "TIM14");
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
