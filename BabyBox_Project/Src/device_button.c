/*
 * device_button.c
 *
 *  Created on: 2018. 10. 1.
 *      Author: user
 */

#include "device_button.h"

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {BUTTON_1_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {BUTTON_1_PIN};

void Button_Init(BUTTON_TypeDef button) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the GPIO_BUTTON_Clock */
	BUTTONx_GPIO_CLK_ENABLE(button);

	/* Configure the GPIO_BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_PIN[button];
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(BUTTON_PORT[button], &GPIO_InitStruct);
	HAL_GPIO_WritePin(BUTTON_PORT[button], BUTTON_PIN[button], GPIO_PIN_RESET);
}
