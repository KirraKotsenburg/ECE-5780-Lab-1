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



	
	// Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Set I2C in RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	  /* Configure the system clock */
  SystemClock_Config();
	
	// Set up LED
	GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc89);
	// Set up LED
	GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc67);
	
	
	// Set PB11 and PB13 to alternate function mode
//	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13))
//																		| GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1;
	
		// Set PB14 PB11 and PB13 to output mode
//	GPIOB->MODER |= (1 << 28); // for 14
		GPIOB->MODER |= (1<<23) | (1<<27) | (1<<28);
    GPIOB->MODER &= ~((1<<22)| (1<<26) | (1<<29));
	
	// Set PC0 to output mode
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER &= ~(1 << 1);

	
	//Set PB11 as open drain
	GPIOB->OTYPER |= (1 << 11);
	// Set PB13 as open drain
	GPIOB->OTYPER |= (1 << 13);
	
	// Select alternate function mode I2C2_SDA and I2C2_SCL for PB11 and PB13
	GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos);
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos);
	
	
	// Set PB14 to push pull output
	GPIOB->OTYPER &= ~(1 << 14);
	// Set PC0 to push pull output
	GPIOC->OTYPER &= ~(1 << 0);
	
	// Set PB14 high
	GPIOB->ODR |= (1 << 14);
	
	// Set PC0 to high
	GPIOC->ODR |= (1 << 0);


	// Set TIMINGR register to correct values
	I2C2->TIMINGR |= (1 << 28); // Prescaler
	I2C2->TIMINGR |= (13 << 0); // SCLL
	I2C2->TIMINGR |= (0xF << 8); // SCLH
	I2C2->TIMINGR |= (2 << 16); // SDADEL
	I2C2->TIMINGR |= (4 << 20); // SCLDEL
	
	// Enable using PE in CR1 reg
	I2C2->CR1 |= (1 << 0);
	
	// Clear slave address
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	// Slave address
	I2C2->CR2 |= (0x69 << 1);
	
	// Set num bites to 1
	I2C2->CR2 |= (1 << 16);
	
	// Set RD_WR to write operation
	I2C2->CR2 &= ~(1 << 10);
	
	// Set start bit
	I2C2->CR2 |= (1 << 13);
	
	// Wait till flags are set
	while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))){
		GPIOC->ODR |= GPIO_ODR_8;
	}
	
	
	
	// Check if NACKF is set
	if((I2C2->CR2 & I2C_ISR_NACKF)){
		GPIOC->ODR |= GPIO_ODR_6;
	}
	// Set the TXDR reg to WHO_AM_I address
	I2C2->TXDR |= 0xF;
	
	// Wait for transfer complete flag
	while(!(I2C2->ISR & I2C_ISR_TC)){
		GPIOC->ODR |= GPIO_ODR_9;
	}
	
// Reset for a read instead of write
	// Slave address
	I2C2->CR2 |= (0x69 << 1);
	
	// Set num bites to 1
	I2C2->CR2 |= (1 << 16);
	
	// Set RD_WR to read operation
	I2C2->CR2 |= (1 << 10);
	
	// Set start bit
	I2C2->CR2 |= (1 << 13);
	
	// Wait till flags are set
	while(!(I2C2->CR2 & I2C_ISR_NACKF) || !(I2C2->CR2 & I2C_ISR_RXNE)){
		
	}
	
	// Check if NACKF is set
	if((I2C2->CR2 & I2C_ISR_NACKF)){
		GPIOC->ODR |= GPIO_ODR_6;
	}
	
	// Check RXDR
	if((I2C2->RXDR & 0xFF) == 0xD3){
		GPIOC->ODR |= GPIO_ODR_7;
	} else{
		GPIOC->ODR |= GPIO_ODR_6;
	}
	
	// Set the stop bit
	I2C2->CR2 |= (1 << 14);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
