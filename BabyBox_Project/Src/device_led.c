/*
 * led.c
 *
 *  Created on: 2018. 9. 3.
 *      Author: leejh
 */

#include "device_led.h"

/** @ Private_Variables
  * @{
  */

GPIO_TypeDef* LED_PORT[LEDn] = {LED1_GPIO_PORT	,
								LED2_GPIO_PORT	,
								LED3_GPIO_PORT	};

const uint16_t LED_PIN[LEDn] = {LED1_PIN	,
								LED2_PIN	,
								LED3_PIN	};

//GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
//const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
//const uint16_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

/**
  * @brief  Configures LED GPIO.
  * @param  Led Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED3
  *     @arg LED4
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */

void LED_Init(Led_TypeDef Led1, Led_TypeDef Led2, Led_TypeDef Led3) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the GPIO_LED_Clock */
	LEDx_GPIO_CLK_ENABLE(Led1);
	LEDx_GPIO_CLK_ENABLE(Led2);
	LEDx_GPIO_CLK_ENABLE(Led3);

	/* Configure the GPIO_LED Pin */
	GPIO_InitStruct.Pin = LED_PIN[Led1] | LED_PIN[Led2] | LED_PIN[Led3];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(LED_PORT[Led1], &GPIO_InitStruct);
	HAL_GPIO_Init(LED_PORT[Led2], &GPIO_InitStruct);
	HAL_GPIO_Init(LED_PORT[Led3], &GPIO_InitStruct);

	HAL_GPIO_WritePin(LED_PORT[Led1], LED_PIN[Led1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT[Led2], LED_PIN[Led2], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT[Led3], LED_PIN[Led3], GPIO_PIN_RESET);
}

void LED_ON(Led_TypeDef Led) {
	HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

void LED_OFF(Led_TypeDef Led) {
	HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

void LED_Toggle(Led_TypeDef Led) {
	HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

void LED_ALL_BLINK(Led_TypeDef Led1, Led_TypeDef Led2, Led_TypeDef Led3) {
	HAL_GPIO_WritePin(LED_PORT[Led1], LED_PIN[Led1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT[Led2], LED_PIN[Led2], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT[Led3], LED_PIN[Led3], GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_PORT[Led1], LED_PIN[Led1], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT[Led2], LED_PIN[Led2], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT[Led3], LED_PIN[Led3], GPIO_PIN_SET);
	HAL_Delay(200);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Mode Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */

#if 0
void PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the Button Clock */
	BUTTONx_GPIO_CLK_ENABLE(Button);
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	GPIO_InitStruct.Pin = BUTTON_PIN[Button];
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	if(Mode == BUTTON_MODE_GPIO) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
	}

	if(Mode == BUTTON_MODE_EXTI) {
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
	}
}

uint32_t PB_GetState(Button_TypeDef Button) {
	return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
#endif
