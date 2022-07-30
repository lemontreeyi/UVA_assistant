/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void TIM10_Set(uint8_t sta);
void TIM11_Set(uint8_t sta);
void TIM13_Set(uint8_t sta);
void TIM14_Set(uint8_t sta);
extern char UWB_data[4][8];
extern int data_flag[4];
extern float yaw;
extern uint16_t Task1_Point1_x;
extern uint16_t Task1_Point1_y;
extern uint16_t Task1_Point2_x;
extern uint16_t Task1_Point2_y;
extern uint16_t Task1_Type1;
extern uint16_t Task1_Type2;
extern uint16_t Task2_Type;
extern uint16_t Task1_index_x1;
extern uint16_t Task1_index_y1;
extern uint16_t Task1_index_x2;
extern uint16_t Task1_index_y2;
extern uint16_t tim10_1ms;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOE
#define Moto_down_Pin GPIO_PIN_0
#define Moto_down_GPIO_Port GPIOC
#define Moto_up_Pin GPIO_PIN_1
#define Moto_up_GPIO_Port GPIOC
#define Moto_Pin_Pin GPIO_PIN_2
#define Moto_Pin_GPIO_Port GPIOC
#define Yaw_Out_Pin GPIO_PIN_0
#define Yaw_Out_GPIO_Port GPIOA
#define Roll_Out_Pin GPIO_PIN_1
#define Roll_Out_GPIO_Port GPIOA
#define Pitch_Out_Pin GPIO_PIN_2
#define Pitch_Out_GPIO_Port GPIOA
#define Thr_Out_Pin GPIO_PIN_3
#define Thr_Out_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define AUX2_In_Pin GPIO_PIN_5
#define AUX2_In_GPIO_Port GPIOA
#define Yaw_In_Pin GPIO_PIN_6
#define Yaw_In_GPIO_Port GPIOA
#define Roll_In_Pin GPIO_PIN_7
#define Roll_In_GPIO_Port GPIOA
#define Pitch_In_Pin GPIO_PIN_0
#define Pitch_In_GPIO_Port GPIOB
#define Thr_In_Pin GPIO_PIN_1
#define Thr_In_GPIO_Port GPIOB
#define Ctrl_In_Pin GPIO_PIN_10
#define Ctrl_In_GPIO_Port GPIOB
#define Mode_In_Pin GPIO_PIN_11
#define Mode_In_GPIO_Port GPIOB
#define Mode_Out_Pin GPIO_PIN_12
#define Mode_Out_GPIO_Port GPIOD
#define Ctrl_Out_Pin GPIO_PIN_13
#define Ctrl_Out_GPIO_Port GPIOD
#define AUX1_Out_Pin GPIO_PIN_14
#define AUX1_Out_GPIO_Port GPIOD
#define AUX2_Out_Pin GPIO_PIN_15
#define AUX2_Out_GPIO_Port GPIOD
#define AUX1_In_Pin GPIO_PIN_3
#define AUX1_In_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_5
#define KEY1_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOB
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void   USART_RxCallback(USART_TypeDef *huart);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
