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
#include "adc.h"
#include "dma.h"
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
#define	RIGHT_SG90_CCR	176
#define	LEFT_SG90_CCR	1319

#define	MAX_OCTAVE	8
#define	MAX_SCALE	12

#define	BUZZER_SRC_TIMER		(TIM2)
#define	BUZZER_SRC_CHANNEL		(TIM_CHANNEL_1)
#define	BUZZER_SRC_TIMER_FREQ	84000000
#define	BUZZER_SRC_TIMER_ARR	168

#define	NUM_OF_ADC_CONVERSION	4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define	get_random_number(MAX_NUM)	(HAL_RNG_GetRandomNumber(&hrng)%(MAX_NUM))
#define	update_SG90_dir(cur)		((cur)=((cur)==LEFT_SG90_CCR)?RIGHT_SG90_CCR:LEFT_SG90_CCR)

#define	next_octave(o)	(((o)+1)%MAX_OCTAVE)
#define	next_scale(s)	(((s)+1)%MAX_SCALE)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t	uart3_rx_data;
static uint8_t	clcd_out_sig;
#if 0
static uint16_t	sg90_ccr = LEFT_SG90_CCR;

static uint32_t	freq[MAX_OCTAVE][MAX_SCALE] = {
		{33,35,37,39,41,44,46,49,52,55,58,62},
		{65,69,73,78,82,87,92,98,104,110,117,123},
		{131,139,147,156,165,175,185,196,208,220,233,247},
		{262,277,294,311,330,349,370,392,415,440,466,494},
		{523,554,587,622,659,698,740,784,831,880,932,988},
		{1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,1865,1976},
		{2093,2217,2349,2489,2637,2794,2960,3136,3322,3520,3729,3951},
		{4186,4435,4699,4978,5274,5588,5920,6272,6645,7040,7459,7902}
};

static char* scale[MAX_SCALE] = {
		"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

static int32_t	cur_octave = MAX_OCTAVE-1;
static int32_t	cur_scale = MAX_SCALE-1;
#endif
static uint8_t	src_idx_of_adc;
volatile static uint16_t	adc_val[NUM_OF_ADC_CONVERSION];
const static char*	adc_name[NUM_OF_ADC_CONVERSION] = {"VR1","VR2","VR3","CDS"};
const static struct{uint32_t min_val,max_val;}
	resolution[NUM_OF_ADC_CONVERSION] = {{0,4095},{0,4095},{0,4095},{0,4095}};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t	get_7SEG_value(uint8_t adc_ptr)
{
	return	100*(uint32_t)adc_val[adc_ptr]/resolution[adc_ptr].max_val;
}

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

#if 0
int	set_buzzer(void)
{
	/*
	if( ++cur_scale == MAX_SCALE )
	{
		cur_scale = 0;

		if( ++cur_octave == MAX_OCTAVE )
		{
			cur_octave = 0;
		}
	}
	*/
	BUZZER_SRC_TIMER->PSC =
			BUZZER_SRC_TIMER_FREQ/(/*freq[cur_octave][cur_scale]*/adc_val[0]*BUZZER_SRC_TIMER_ARR);

	return	0;
}
#endif
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_RNG_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  change_LED_state(LEFT, LED_OFF);
  change_LED_state(RIGHT, LED_OFF);
  set_LED_state_by_switch(SW1);

  HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
  HAL_TIM_Base_Start_IT(&htim7);

  CLCD_GPIO_Init();
  CLCD_Init();

  _7SEG_GPIO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if 0
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#endif
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_val,4);

  display_7SEG_number(get_7SEG_value(src_idx_of_adc));
  set_sig(clcd_out_sig);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if( clcd_out_sig == TRUE )
	  {
		  clear_sig(clcd_out_sig);
		  CLCD_Printf("RNG:%03d\n7SEG:%s",
					  (int)get_random_number(MAX_RANDOM_NUMBER),
					  adc_name[src_idx_of_adc]);
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
		case GPIO_PIN_10:
			src_idx_of_adc = (src_idx_of_adc+1)%NUM_OF_ADC_CONVERSION;
			set_sig(clcd_out_sig);
			break;
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

	if( htim->Instance == TIM7 )
	{
		display_7SEG_number(get_7SEG_value(src_idx_of_adc));

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
