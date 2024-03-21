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

void init_LEDS();

void init_GPIO_Analog();

void config_ADC();

void config_DAC();

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	
	// Initialize PC0 for analog
	init_GPIO_Analog();
	
	//Initailize LEDs
	init_LEDS();
	
	//Configure and start the ADC
	config_ADC();

	//Configure DAC
	config_DAC();
	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	
		
		
	uint8_t index = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(1); // delay for 1ms between samples
		
	/*	Part 1
	if(ADC1->DR > 20){
		GPIOC->ODR |= GPIO_ODR_6;
	} else{ // For if a voltage drops below threshold
		GPIOC->ODR &= ~GPIO_ODR_6; // red
	}
	
		if(ADC1->DR > 100){
			GPIOC->ODR ^= GPIO_ODR_7;
	} else{ // For if a voltage drops below threshold
			GPIOC->ODR &= ~GPIO_ODR_7; //
	}
	
		if(ADC1->DR > 170){
			GPIOC->ODR |= GPIO_ODR_8;
	} else{ // For if a voltage drops below threshold
			GPIOC->ODR &= ~GPIO_ODR_8;
	}
	
		if(ADC1->DR > 230){
			GPIOC->ODR |= GPIO_ODR_9;
	} else{ // For if a voltage drops below threshold
			GPIOC->ODR &= ~GPIO_ODR_9;
	}
	*/
	// Set up DAC data into register
	DAC->DHR8R1 = sine_table[index];
	
	if (index == 31){
		index = 0;
	}else{
			// Increment the index of table
		index = index + 1;
	}

	
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Initializes the LEDs to output mode */
void init_LEDS(){
	
	GPIO_InitTypeDef initc6789 = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc6789);

}
/* Sets PC0 to analog mode for ADC_IN */
void init_GPIO_Analog(){
	  // Enable GPIOC clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
		// Set PC0 to analog mode
	GPIOC->MODER |= (1 << 1) | (1 << 0);
	// No pull up or down
	GPIOC->PUPDR &= ~(1 << 0);
	
}
/* Configure the ADC */
void config_ADC(){
	
	  // Enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// Configure to continuous conversion (1 at 13)
	ADC1->CFGR1 |= (1 << 13);
	
	// Configure resolution to 8-bits (10 at 4:3)
	ADC1->CFGR1 |= (1 << 4);
	ADC1->CFGR1 &= ~(1 << 3);
	
	// Disable hardware triggers (00 at 11:10)
	ADC1->CFGR1 &= ~((1 << 11) |(1 << 10));
	
	// Select channel 10 (PC0)
	ADC1->CHSELR |= (1 << 10);
	
	// Self calibrate for ADC
	ADC1->CR |= (1 << 31);
	
	//Need to wait for calibration to finish (0 is all clear)
	while(ADC1->CR & ADC_CR_ADCAL){	
	}
	
	// ADC enable
	ADC1->CR |= (1 << 0);
	
	// Wait till the ADC ready flag is set (if 1 then all clear)
	while(!(ADC1->ISR & ADC_ISR_ADRDY)){
	}
	
	//ADC start 
	ADC1->CR |= (1 << 2);
	
}

/* Configure the DAC*/
void config_DAC(){
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Enable DAC clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// Set to analog mode
	GPIOA->MODER |= (1 << 9) | (1 << 8);
	
	// No pull up or down
	GPIOA->PUPDR &= ~((1 << 9) |(1 << 8));
	
	
	// Set channel 1 to software trigger (111 for bits 5:3)
	DAC->CR |= (1 << 5) | (1 << 4) | (1 << 3);
	
	// Enable channel 1 of DAC
	DAC->CR |= (1 << 0);
	
	
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
