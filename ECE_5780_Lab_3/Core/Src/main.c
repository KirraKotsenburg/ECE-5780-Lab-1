/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

volatile int c1 = 0;

void EXTI0_1_IRQHandler(){
	
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	
	while(c1 <= 1500000){
		c1++;
	}
	
// Toggle LEDs for green and orange
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	EXTI->PR |= (1 << 0);
	c1 = 0;
	
/* Part 1
	
// Toggle LEDs for green and orange
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
// Reset Pending Register
	EXTI->PR |= (1 << 0);
	*/
}

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
  //HAL_Init();



  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable the GPIOA clock in the RCC
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER |= (1 << 16);

	GPIOA->MODER &= ~(1 << 1);
	GPIOA->MODER &= ~(1 << 0);

	GPIOC->OTYPER &= ~(1 << 6);
	GPIOC->OTYPER &= ~(1 << 9);
	GPIOC->OTYPER &= ~(1 << 7);
	GPIOC->OTYPER &= ~(1 << 8);

	GPIOC->OSPEEDR &= ~(1 << 12);
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->OSPEEDR &= ~(1 << 14);
	GPIOC->OSPEEDR &= ~(1 << 16);
	
	GPIOA->OSPEEDR &= ~(1 << 0);

	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13));
	GPIOC->PUPDR &= ~((1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17));
	GPIOC->PUPDR &= ~((1 << 14) | (1 << 15));
	
	
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~(1 << 0);

	GPIOC->ODR |= (1 << 9);
	GPIOC->ODR &= ~(1 << 6);
	
// Connect PA0 to EXTI0
	SYSCFG->CFGR1 &= ~((1 << 2) | (1 << 1) | (1 << 0));

// Unmask
	EXTI->IMR |= (1 << 0);
	
// Rising edge trigger
	EXTI->RTSR |= (1 << 0);
	
	SYSCFG->EXTICR[0] &= ~((1 << 2) | (1 << 1) | (1 << 0));
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	// Set the priority high
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	
	NVIC_SetPriority(SysTick_IRQn, 2);

  /* Infinite loop */
  while (1)
  {
		HAL_Delay(500); // Delay 500ms
		
		GPIOC->ODR ^= GPIO_ODR_6;
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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