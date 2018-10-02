/*
 * device_buzzer.h
 *
 *  Created on: 2018. 10. 1.
 *      Author: user
 */

#ifndef DEVICE_BUZZER_H_
#define DEVICE_BUZZER_H_

#include"stm32f1xx_hal.h"

typedef enum {
	BUZZER_INTERNAL = 0,
	BUZZER_EXTERNAL
}BUZZER_TypeDef;		// PIR : Passive Infrated Sensor : for Motion detection

#define BUZZERn							2

// BUZZER_1 -> INTERNAL BUZZER
#define BUZZER_1_PIN					GPIO_PIN_12
#define BUZZER_1_PORT					GPIOB
#define BUZZER_1_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define BUZZER_1_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOB_CLK_DISABLE()
#define INTERNAL_BUZZER					BUZZER_1_PIN		// Redefine.

// BUZZER_2 -> EXTERNAL BUZZER
#define BUZZER_2_PIN					GPIO_PIN_5
#define BUZZER_2_PORT					GPIOB
#define BUZZER_2_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define BUZZER_2_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOB_CLK_DISABLE()
#define EXTERNAL_BUZZER					BUZZER_2_PIN		// Redefine.

#define BUZZERx_GPIO_CLK_ENABLE(__BUZZER__) do { if((__BUZZER__) == BUZZER_INTERNAL) BUZZER_1_GPIO_CLK_ENABLE(); else \
												 if((__BUZZER__) == BUZZER_EXTERNAL) BUZZER_2_GPIO_CLK_ENABLE();} while(0)
#define BUZZERx_GPIO_CLK_DISABLE(__BUZZER__) (((__BUZZER__) == BUZZER_INTERNAL) ? BUZZER_1_GPIO_CLK_DISABLE() : \
											  ((__BUZZER__) == BUZZER_EXTERNAL) ? BUZZER_2_GPIO_CLK_DISABLE() : 0)

void Buzzer_Init(BUZZER_TypeDef buzzer);
void Buzzer_On(BUZZER_TypeDef buzzer);
void Buzzer_Off(BUZZER_TypeDef buzzer);
#endif /* DEVICE_BUZZER_H_ */
