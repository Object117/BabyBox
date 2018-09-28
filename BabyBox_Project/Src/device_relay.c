/*
 * device_delay.c
 *
 *  Created on: 2018. 9. 5.
 *      Author: leejh
 */
#include "device_relay.h"

GPIO_TypeDef* RELAY_PORT[RELAYn] = {RELAY1_GPIO_PORT	,
									RELAY2_GPIO_PORT  	};

const uint16_t RELAY_PIN[RELAYn] = {RELAY1_PIN		,
									RELAY2_PIN		};

void Relay_Init(Relay_TypeDef relay) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Enable the GPIO_Relay_Clock*/
	RELAYx_GPIO_CLK_ENABLE(relay);

	/*Configure the GPIO_Realy pin */
	GPIO_InitStruct.Pin = RELAY_PIN[relay];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(RELAY_PORT[relay], &GPIO_InitStruct);

	HAL_GPIO_WritePin(RELAY_PORT[relay], RELAY_PIN[relay], GPIO_PIN_RESET);
}

void lock_door(void) {
	HAL_GPIO_WritePin(RELAY_PORT[RELAY1], RELAY_PIN[RELAY1], GPIO_PIN_SET);
}

void unlock_door(void) {
	HAL_GPIO_WritePin(RELAY_PORT[RELAY1], RELAY_PIN[RELAY1], GPIO_PIN_RESET);
}

void light_on(void) {
	HAL_GPIO_WritePin(RELAY_PORT[RELAY2], RELAY_PIN[RELAY2], GPIO_PIN_SET);
}

void light_off(void) {
	HAL_GPIO_WritePin(RELAY_PORT[RELAY2], RELAY_PIN[RELAY2], GPIO_PIN_RESET);
}

void excuteRelayTest(void) {

#if 1
	HAL_GPIO_TogglePin(RELAY_PORT[RELAY1], RELAY_PIN[RELAY1]);
#else
	HAL_GPIO_WritePin(RELAY_PORT[RELAY1], RELAY_PIN[RELAY1], GPIO_PIN_RESET);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(RELAY_PORT[RELAY1], RELAY_PIN[RELAY1], GPIO_PIN_SET);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(RELAY_PORT[RELAY2], RELAY_PIN[RELAY2], GPIO_PIN_RESET);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(RELAY_PORT[RELAY2], RELAY_PIN[RELAY2], GPIO_PIN_SET);
	HAL_Delay(2000);
#endif
}
