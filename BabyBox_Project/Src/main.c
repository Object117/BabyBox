
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
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "device_led.h"
#include "device_relay.h"
#include "device_photoInterrupter.h"
#include "device_PIRsensor.h"
#include "stateMachine.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t TxBuffer[] = "Interrupt!\n";
UART_HandleTypeDef huart2;

USER_ACTION* tCurrent_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void testLEDinit(void);
//static void EXTI0_1_IRQHandler_Config(void);
static void EXTI4_IRQHandler_Config(void);
static void EXTI9_5_IRQHandler_Config(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  //  EXTI0_1_IRQHandler_Config();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  EXTI4_IRQHandler_Config();
  EXTI9_5_IRQHandler_Config();
#if 1
  LED_Init(RED_LED, GREEN_LED, BLUE_LED);
//  PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  PIS_Init(PIS1);
  PIS_Init(PIS2);
  PIR_Sensor_Init(PIR_SENSOR1);
  Relay_Init(RELAY1);
  Relay_Init(RELAY2);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  extdoor_status = EXT_DOOR_CLOSE;
  innerdoor_state = INNER_DOOR_CLOSE;
  baby_state = BABY_NONE;

  changeingState = initialize_state();

  uint8_t Welcome[] = "Welcome";
  HAL_UART_Transmit(&huart1, (uint8_t*)Welcome, sizeof(Welcome), 10);		// TESTING

#if 1
  testLEDinit();
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
#endif

  while(1)
  {
  /* USER CODE END WHILE */\
  /* USER CODE BEGIN 3 */
	  tCurrent_state = change_state();
	  tCurrent_state->extdoor_open();
	  tCurrent_state->extdoor_close();
	  tCurrent_state->inner_door_open();
	  tCurrent_state->inner_door_close();
	  tCurrent_state->baby_in();
	  tCurrent_state->baby_none();
  }
  /* USER CODE END 3 */

}


void testLEDinit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure the GPIO_LED Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
#if 0
static void EXTI0_1_IRQHandler_Config(void) {		// User Button
	/*Enable and set EXTI lines 0 to 1 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}
#endif

static void EXTI4_IRQHandler_Config(void) {		//exteranl Photo Interrupter
	/*Enable and set EXTI lines 2 to 3 Interrupt*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

static void EXTI9_5_IRQHandler_Config(void) {		// Inner Photo Interrupter // PIR Motion Sensor
	/*Enable and set EXTI lines 4 to 15 Interrupt*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t Action) {

	uint8_t EXT_DOOR_close[] = "EXT_DOOR_CLOSE";
	uint8_t EXT_DOOR_open[] = "EXT_DOOR_OPEN";
	uint8_t INNER_DOOR_close[] = "INNER_DOOR_CLOSE";
	uint8_t INNER_DOOR_open[] = "INNER_DOOR_OPEN";
	uint8_t MOTION_detect[] = "MOTION_DETECT";


	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);

	switch(HAL_GPIO_ReadPin(EXT_DOOR_PORT, EXT_DOOR_PIN)) {
		case GPIO_PIN_SET	:
			HAL_UART_Transmit(&huart1, (uint8_t*)EXT_DOOR_close, sizeof(EXT_DOOR_close), 10);		// TESTING
			extdoor_status = EXT_DOOR_CLOSE;
			break;

		case GPIO_PIN_RESET	:
			HAL_UART_Transmit(&huart1, (uint8_t*)EXT_DOOR_open, sizeof(EXT_DOOR_open), 10);		// TESTING
			extdoor_status = EXT_DOOR_OPEN;
			break;

		default		:	// exception
			extdoor_status = EXT_DOOR_CLOSE;
			break;
	}

	switch(HAL_GPIO_ReadPin(INNER_DOOR_PORT, INNER_DOOR_PIN)) {
		case GPIO_PIN_SET	:
			HAL_UART_Transmit(&huart1, (uint8_t*)INNER_DOOR_close, sizeof(INNER_DOOR_close), 10);		// TESTING
			innerdoor_state = INNER_DOOR_CLOSE;
			break;

		case GPIO_PIN_RESET	:
			HAL_UART_Transmit(&huart1, (uint8_t*)INNER_DOOR_open, sizeof(INNER_DOOR_open), 10);		// TESTING
			innerdoor_state = INNER_DOOR_OPEN;
			break;

		default		:	// exception
			innerdoor_state = INNER_DOOR_CLOSE;
			break;
	}

	if(Action == MOTION_DETECTING) {
		HAL_UART_Transmit(&huart1, (uint8_t*)MOTION_detect, sizeof(MOTION_detect), 10);		// TESTING
		baby_state = BABY_IN;
	}
}

/* USER CODE END 4 */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
