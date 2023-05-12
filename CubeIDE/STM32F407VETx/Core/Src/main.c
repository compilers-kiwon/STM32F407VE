/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "CLCD.h"
#include "7SEG.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define	get_random_number(MAX_NUM)	(HAL_RNG_GetRandomNumber(&hrng)%(MAX_NUM))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t	uart3_rx_data;
static uint8_t	random_number_sig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int	change_LED_state(int led_pos,int pin_state)
{
	for(int i=0;i<NUM_OF_LEDs;i++)
	{
		HAL_GPIO_WritePin(led[led_pos][i].gpio_type, led[led_pos][i].gpio_pin, pin_state);
	}

	return	0;
}

static int	set_LED_state_by_switch(int switch_idx)
{
	if( HAL_GPIO_ReadPin(swtch[switch_idx].gpio_type, swtch[switch_idx].gpio_pin) == SWITCH_ON )
	{
		change_LED_state(LEFT, LED_ON);
		change_LED_state(RIGHT, LED_ON);
	}
	else
	{
		change_LED_state(LEFT, LED_OFF);
		change_LED_state(RIGHT, LED_OFF);
	}

	return	0;
}

int	_write(int file,char* p,int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)p, len, 10);
	return	0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_RNG_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //change_LED_state(LEFT, LED_OFF);
  //change_LED_state(RIGHT, LED_OFF);
  set_LED_state_by_switch(SW1);

  HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
  HAL_TIM_Base_Start_IT(&htim7);

  CLCD_GPIO_Init();
  CLCD_Init();

  _7SEG_GPIO_Init();

  CLCD_Puts(CLCD_COL0, CLCD_ROW0, (uint8_t*)"Welcome to");
  CLCD_Puts(CLCD_COL0, CLCD_ROW1, (uint8_t*)"STM32F407VETx");
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if( random_number_sig == TRUE )
	  {
		  clear_sig(random_number_sig);
		  CLCD_Printf("Random Number:\n%d\n",
		  				(int)get_random_number(MAX_RANDOM_NUMBER));
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t	led_idx;

	switch(GPIO_Pin)
	{
		case GPIO_PIN_3:led_idx=0;break;
		case GPIO_PIN_15:led_idx=1;break;
		case GPIO_PIN_4:led_idx=2;break;
		case GPIO_PIN_10:set_sig(random_number_sig);break;
		default:/*do nothing*/;break;
	}

	HAL_GPIO_TogglePin(led[LEFT][led_idx].gpio_type, led[LEFT][led_idx].gpio_pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == USART3 )
	{
		HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
		HAL_UART_Transmit(&huart3, &uart3_rx_data, sizeof(uart3_rx_data), 10);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int	left_led_state = LED_OFF;
	static int	right_led_state = LED_ON;
	static int	cnt = 0;

	if( htim->Instance == TIM7 )
	{
		//set_sig(random_number_sig);

		display_7SEG_number(cnt);
		cnt = (cnt+1)%100;

		//change_LED_state(LEFT, left_led_state);
		change_LED_state(RIGHT, right_led_state);

		left_led_state = (left_led_state==LED_OFF)?LED_ON:LED_OFF;
		right_led_state = (right_led_state==LED_ON)?LED_OFF:LED_ON;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
