/*
 * device_buzzer.c
 *
 *  Created on: 2018. 10. 1.
 *      Author: user
 */

#include "device_buzzer.h"

GPIO_TypeDef* BUZZER_PORT[BUZZERn] = {BUZZER_1_PORT	 ,
									  BUZZER_2_PORT  };

const uint16_t BUZZER_PIN[BUZZERn] = {BUZZER_1_PIN	,
									  BUZZER_2_PIN   };

void Buzzer_Init(BUZZER_TypeDef buzzer) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the GPIO_BUZZER_Clock */
	BUZZERx_GPIO_CLK_ENABLE(buzzer);

	/* Configure the GPIO_BUZZER Pin */
	GPIO_InitStruct.Pin = BUZZER_PIN[buzzer];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(BUZZER_PORT[buzzer], &GPIO_InitStruct);
	HAL_GPIO_WritePin(BUZZER_PORT[buzzer], BUZZER_PIN[buzzer], GPIO_PIN_SET);
}

void Buzzer_On(BUZZER_TypeDef buzzer) {
	HAL_GPIO_WritePin(BUZZER_PORT[buzzer], BUZZER_PIN[buzzer], GPIO_PIN_RESET);
}

void Buzzer_Off(BUZZER_TypeDef buzzer) {
	HAL_GPIO_WritePin(BUZZER_PORT[buzzer], BUZZER_PIN[buzzer], GPIO_PIN_SET);
}
