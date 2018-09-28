/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PWM_CH1_Pin GPIO_PIN_0
#define PWM_CH1_GPIO_Port GPIOA
#define PWM_CH2_Pin GPIO_PIN_1
#define PWM_CH2_GPIO_Port GPIOA
#define PHOTO_1_EXTI4_Pin GPIO_PIN_4
#define PHOTO_1_EXTI4_GPIO_Port GPIOA
#define PHOTO_2_EXTI5_Pin GPIO_PIN_5
#define PHOTO_2_EXTI5_GPIO_Port GPIOA
#define PIR_EXTI6_Pin GPIO_PIN_6
#define PIR_EXTI6_GPIO_Port GPIOA
#define LoadCell_DT_Pin GPIO_PIN_7
#define LoadCell_DT_GPIO_Port GPIOA
#define LoadCell_CLK_Pin GPIO_PIN_0
#define LoadCell_CLK_GPIO_Port GPIOB
#define Emergence_BTN_EXTI1_Pin GPIO_PIN_1
#define Emergence_BTN_EXTI1_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define Buzzer_CTRL_Pin GPIO_PIN_12
#define Buzzer_CTRL_GPIO_Port GPIOB
#define ELECTROMAGNET_CTRL_1_Pin GPIO_PIN_13
#define ELECTROMAGNET_CTRL_1_GPIO_Port GPIOB
#define ELECTROMAGNET_CTRL_2_Pin GPIO_PIN_14
#define ELECTROMAGNET_CTRL_2_GPIO_Port GPIOB
#define BAR_LED_CTRL_1_Pin GPIO_PIN_15
#define BAR_LED_CTRL_1_GPIO_Port GPIOB
#define BAR_LED_CTRL_2_Pin GPIO_PIN_8
#define BAR_LED_CTRL_2_GPIO_Port GPIOA
#define USER_LED_STATUS_Pin GPIO_PIN_11
#define USER_LED_STATUS_GPIO_Port GPIOA
#define USER_KEY_1_EXTI_Pin GPIO_PIN_12
#define USER_KEY_1_EXTI_GPIO_Port GPIOA
#define LED_R_EXTERNAL_Pin GPIO_PIN_15
#define LED_R_EXTERNAL_GPIO_Port GPIOA
#define LED_G_EXTERNAL_Pin GPIO_PIN_3
#define LED_G_EXTERNAL_GPIO_Port GPIOB
#define LED_B_EXTERNAL_Pin GPIO_PIN_4
#define LED_B_EXTERNAL_GPIO_Port GPIOB
#define Buzzer_CTRL_EXTRNAL_Pin GPIO_PIN_5
#define Buzzer_CTRL_EXTRNAL_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
