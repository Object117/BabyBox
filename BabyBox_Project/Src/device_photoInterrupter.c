/*
 * device_photoInterrupter.c
 *
 *  Created on: 2018. 9. 6.
 *      Author: leejh
 */
#include"device_photoInterrupter.h"

GPIO_TypeDef* PIS_PORT[PISn] = { PIS1_GPIO_PORT, PIS2_GPIO_PORT};
uint16_t PIS_PIN[PISn] = { PIS1_PIN, PIS2_PIN };

void PIS_Init(PIS_TypeDef pis) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Enable the GPIO_PIS_Clock */
	PISx_GPIO_CLK_ENABLE(pis);

	/*Configure the GPIO_PIS_Clock */
	GPIO_InitStruct.Pin = PIS_PIN[pis];
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;		//ORIG	GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;					//ORIG	GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(PIS_PORT[pis], &GPIO_InitStruct);

}
