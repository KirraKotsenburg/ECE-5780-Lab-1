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

/*
void EXTI0_1_IRQHandler(){
	
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	
	while(c1 <= 1500000){
		c1++;
	}
	
// Toggle LEDs for green and orange
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	EXTI->PR |= (1 << 0);
	c1 = 0;
	
 Part 1
	
// Toggle LEDs for green and orange
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
// Reset Pending Register
	EXTI->PR |= (1 << 0);
	
}
*/


void TIM2_IRQHandler(){
	//toggle LEDs
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	// Reset pending flag
	TIM2->SR &= ~TIM_SR_UIF;
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
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCONFIG clock
	
	//Enable Timer 2 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//Enable Timer 3 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Set the PSC and ARR
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	// Generate an interrupt on the UEV event (4Hz)
	TIM2->EGR |= TIM_EGR_TG;
	TIM2->DIER |= TIM_DIER_UIE;
	
	// Enabling Timer 2 control register to start
	//TIM2->CR1 |= TIM_CR1_CEN; Might not need
	
	// Enabling the interrupt handler for TIM2
	NVIC_EnableIRQ(TIM2_IRQn);
	
		// Set the PSC and ARR (PWM of TIM3)
	TIM3->PSC = 79;
	TIM3->ARR = 125;
	
	// Set the bits [6:4] for Capture/Compare Mode Register (...110...) MIGHT NOT NEED
	//TIM3->CCMR1 |= (1 << 6);
	//TIM3->CCMR1 |= (1 << 5);
	//TIM3->CCMR1 &= ~(1 << 4);
	/*
	//CCMR1 CC1S [1:0] bits set to output
	TIM3->CCMR1 &= ~((1 << 1) | (1 << 0));
	
	//CCMR1 CC2S [9:8] bits set to output
	TIM3->CCMR1 &= ~((1 << 9) | (1 << 8));
	*/
	// Clear the bits of OC1M
	//TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	
	// Set output channel 1 to PWM mode 2
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

	
	// Set output channel 2 to PWM mode 1
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

	
	
	// Set the output compare preload for both channels
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
	
	// Set the output enable for both channels in CCER
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	
	// Set CCRx for both channels to 20% of ARR
	TIM3->CCR1 = 25;
	TIM3->CCR2 = 25;
	
//	TIM3->CCR1 = 50;
//	TIM3->CCR2 = 25;
	
// Configuring Pin Alternate functions section
	// Set PC6/7 to Alternate MODER
	GPIOC->MODER |= GPIO_MODER_MODER6_1;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	
	// Setting PC8/9 to generic output MODER
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	
	GPIOC->ODR |= GPIO_ODR_9;
	
	// Start TIM2
	TIM2->CR1 |= TIM_CR1_CEN;
	// Start TIM3
	TIM3->CR1 |= TIM_CR1_CEN;

  /* Infinite loop */
  while (1)
  {
	//	HAL_Delay(500); // Delay 500ms
		
//		GPIOC->ODR ^= GPIO_ODR_6;
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