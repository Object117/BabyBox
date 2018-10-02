/*
 * device_button.h
 *
 *  Created on: 2018. 10. 1.
 *      Author: user
 */

#ifndef DEVICE_BUTTON_H_
#define DEVICE_BUTTON_H_

#include"stm32f1xx_hal.h"

typedef enum {
	EMERGENCY_BUTTON = 0
}BUTTON_TypeDef;

#define BUTTONn 					1

// BUTTON 1 -> Emergency Button
#define BUTTON_1_PIN							GPIO_PIN_1
#define BUTTON_1_PORT							GPIOB
#define BUTTON_1_GPIO_CLK_ENABLE()				__HAL_RCC_GPIOB_CLK_ENABLE()
#define BUTTON_1_GPIO_CLK_DISABLE()				__HAL_RCC_GPIOB_CLK_DISABLE()
#define BUTTON_EMERGENCY						BUTTON_1_PIN		// Re-define

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)		do { if((__BUTTON__) == EMERGENCY_BUTTON) BUTTON_1_GPIO_CLK_ENABLE();} while(0);
#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)	(((__BUTTON__) == EMERGENCY_BUTTON) ? BUTTON_1_GPIO_CLK_DISABLE : 0)

void Button_Init(BUTTON_TypeDef button);

#endif /* DEVICE_BUTTON_H_ */
