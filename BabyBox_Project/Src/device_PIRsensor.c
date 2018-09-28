/*
 * device_PIRsensor.c
 *
 *  Created on: 2018. 9. 6.
 *      Author: leejh
 */
#include"device_PIRsensor.h"

GPIO_TypeDef* PIR_PORT[PIR_SENSORn] = { PIR1_SENSOR_PORT };
uint16_t PIR_PIN[PIR_SENSORn] = { PIR1_SENSOR_PIN };

void PIR_Sensor_Init(PIRsensor_TypeDef pirsensor) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Enable the GPIO_PIS_Clock */
	PIRx_GPIO_CLK_ENABLE(pirsensor);

	/*Configure the GPIO_PIR_Clock */
	GPIO_InitStruct.Pin = PIR1_SENSOR_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(PIR_PORT[pirsensor], &GPIO_InitStruct);
}
