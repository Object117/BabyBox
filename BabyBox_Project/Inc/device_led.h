/*
 * led.h
 *
 *  Created on: 2018. 9. 3.
 *      Author: leejh
 */

#ifndef DEVICE_LED_H_
#define DEVICE_LED_H_

/**
 * @brief LED Types Definition
 */
#include "stm32f1xx_hal.h"

typedef enum {
	LED1 = 0	,
	LED2		,
	LED3		,
	/* Color led aliases */
	RED_LED = LED1		,
	GREEN_LED = LED2	,
	BLUE_LED = LED3

}Led_TypeDef;

/**
 * @brief BUTTON Definition
 */

typedef enum {
	BUTTON_USER = 0
}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;


/** @defgroup STM32F072B_DISCOVERY_LED STM32F072B_DISCOVERY LED
  * @{
  */
#define LEDn                             4

/*
 * Red colar
 */
#define LED1_PIN                         GPIO_PIN_15
#define LED1_GPIO_PORT                   GPIOA
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

/*
 * Green colar
 */
#define LED2_PIN                         GPIO_PIN_3
#define LED2_GPIO_PORT                   GPIOB
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

/*
 * Blue colar
 */
#define LED3_PIN                         GPIO_PIN_4
#define LED3_GPIO_PORT                   GPIOB
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__) do { if((__LED__) == LED1) LED1_GPIO_CLK_ENABLE(); else \
                                           if((__LED__) == LED2) LED2_GPIO_CLK_ENABLE(); else \
                                           if((__LED__) == LED3) LED3_GPIO_CLK_ENABLE();} while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)  (((__LED__) == LED1) ? LED1_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED2) ? LED2_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() : 0 )
/**
  * @}
  */

/** @defgroup STM32F072B_DISCOVERY_BUTTON STM32F072B_DISCOVERY BUTTON
  * @{
  */
#if 0
#define BUTTONn                          1

/**
 * @brief USER push-button
 */
#define USER_BUTTON_PIN                GPIO_PIN_0                       /* PA0 */
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn          EXTI0_1_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)    do { if((__BUTTON__) == BUTTON_USER) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)   (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0 )
#endif
/**
  * @}
  */

/** @defgroup STM32F072B_LED_Related_Function
  * @{
  */
void LED_Init(Led_TypeDef Led1, Led_TypeDef Led2, Led_TypeDef Led3);
void LED_ON(Led_TypeDef Led);
void LED_OFF(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);
void LED_ALL_BLINK(Led_TypeDef Led1, Led_TypeDef Led2, Led_TypeDef Led3);

/** @defgroup STM32F072B_LED_Related_Function
  * @{
  */
void PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t PB_GetState(Button_TypeDef Button);

#endif /* DEVICE_LED_H_ */
