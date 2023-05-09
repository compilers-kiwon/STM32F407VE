/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	GPIO_TypeDef*	gpio_type;
	uint16_t		gpio_pin;
} GPIO_Info;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define	LEFT		0
#define	RIGHT		1
#define	NUM_OF_LEDs	3

#define	LED_OFF	GPIO_PIN_SET
#define	LED_ON	GPIO_PIN_RESET

#define	SWITCH_ON	GPIO_PIN_SET
#define	SWITCH_OFF	GPIO_PIN_RESET

#define	SW1_PIN	GPIO_PIN_3

const static GPIO_Info	led[][NUM_OF_LEDs] = {
		{{GPIOD,GPIO_PIN_12},{GPIOD,GPIO_PIN_13},{GPIOD,GPIO_PIN_14}},
		{{GPIOC,GPIO_PIN_6},{GPIOB,GPIO_PIN_5},{GPIOB,GPIO_PIN_0}}
};

const static GPIO_Info	swtch[] = {
		{GPIOE,SW1_PIN}
};

enum{
	SW1 = 0,
	SW2 = 1,
	SW3 = 2,
	SW4 = 3
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
