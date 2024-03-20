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

#define L3GD20 0x69

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void I2C_Config();

void I2C_WriteReg(uint16_t devAddr, uint8_t regAddr, uint8_t data);

int8_t I2C_ReadReg(uint16_t devAddr);

void I2C_SetRegAddr(uint16_t devAddr, uint8_t regAddr);



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

	/* Configure the system clock */
  SystemClock_Config();
	
	// Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Set I2C in RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	
	// Set up all LEDs
	GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc89);
	GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initc67);
	
	
	// Configure the I2C
	I2C_Config();

	int8_t normal_mode = 0xB;
	int8_t ctrl_reg1_addr = 0x20;
	I2C_WriteReg(L3GD20, ctrl_reg1_addr, normal_mode); 
	

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//Read and save value of X and Y axes
				I2C_SetRegAddr(L3GD20, 0x28); 
		int8_t x_low = I2C_ReadReg(L3GD20); 

		I2C_SetRegAddr(L3GD20, 0x29);
		int8_t x_high = I2C_ReadReg(L3GD20); 
		
		// 16 bit measured value for X
		int16_t x_data = ((int16_t)x_high << 8) | (uint8_t)x_low;

		
		I2C_SetRegAddr(L3GD20, 0x2A); 
		int8_t y_low = I2C_ReadReg(L3GD20); 

		
		I2C_SetRegAddr(L3GD20, 0x2B);
		int8_t y_high = I2C_ReadReg(L3GD20);
	
		// 16 bit measured value for Y
		int16_t y_data = ((int16_t)y_high << 8) | (uint8_t)y_low;
	
		int32_t threshold = 1000;
		
		GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 | GPIO_ODR_9); // Reset the LEDs
		// Set the LEDs for thilting the gyro
		if (y_data > threshold) {
				GPIOC->ODR |= GPIO_ODR_6; // Red LED for positive Y 
		} else if (y_data < -threshold) {
				GPIOC->ODR |= GPIO_ODR_7; // Blue LED for negative Y 
		}

		if (x_data > threshold) {
				GPIOC->ODR |= GPIO_ODR_9; // Green LED for positive X
		} else if (x_data < -threshold) {
				GPIOC->ODR |= GPIO_ODR_8; // Orange LED for negative X
		}
		
		HAL_Delay(1000);
		

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Function for setting up I2C. Sets correct modes, alternate functions, and enabling bit*/
void I2C_Config() 
{
	// Set PB14 PB11 and PB13 to output mode
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
	
}

/* I2C write register function*/
void I2C_WriteReg(uint16_t devAddr, uint8_t regAddr, uint8_t data) 
{
		// Clear slave address
	I2C2->CR2 = 0;
	
	// Slave address
	I2C2->CR2 |= (devAddr << 1);
	
	// Set num bytes to 1
	I2C2->CR2 |= (0x2 << 16); // maybe do 0x2
	
	// Set RD_WR to write operation
	I2C2->CR2 &= ~(1 << 10);
	
	// Set start bit
	I2C2->CR2 |= I2C_CR2_START;
	
	// Wait till flags are set
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))){
	}
	
	// Check if NACKF is set
	if(I2C2->ISR & I2C_ISR_NACKF){
		//GPIOC->ODR ^= GPIO_ODR_6; // red if error
	}
	// Set the TXDR reg to WHO_AM_I address
	I2C2->TXDR |= regAddr;
	
	// Wait for transfer complete flag
	while (!(I2C2->ISR & I2C_ISR_TXIS)) {
		//GPIOC->ODR ^= GPIO_ODR_9; // green
		}
		
	// Write data into the TXDR 	
	I2C2->TXDR = data;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {
	}
		
}

/* I2C read regicter function*/
int8_t I2C_ReadReg(uint16_t devAddr) 
{
	// Reset for a read instead of write
	// Slave address
	I2C2->CR2 |= 0;
	int8_t data = 0;
	
	// Use SADD[7:1] bit field in CR2 register to set slave address to L3GD20
	I2C2->CR2 |= (devAddr << 1);
	
	// Set num bytes to 1
	I2C2->CR2 |= (1 << 16);
	
	// Set RD_WR to read operation
	I2C2->CR2 |= (1 << 10);
	
	// Set start bit
	I2C2->CR2 |= I2C_CR2_START;
	
	// Wait till flags are set
	while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))){}
	
	// Check if NACKF is set
	if((I2C2->ISR & I2C_ISR_NACKF)){
		//GPIOC->ODR ^= GPIO_ODR_6; // red if error
	}	
		
	// Wait for TC flag set
	while (!(I2C2->ISR & I2C_ISR_TC)) {
		//GPIOC->ODR ^= GPIO_ODR_9; // green
	}
		
	// Read contents of RXDR register and return data - remember it is 1 byte at a time
	data = I2C2->RXDR;
	
	return data;
}

/* Function for setting the register address of I2C*/
void I2C_SetRegAddr(uint16_t devAddr, uint8_t regAddr)
{
	// Clear slave address
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (devAddr << 1);
	
	// Num bytes
	I2C2->CR2 |= (1 << 16);
	
	// Set RD_WR to 0 for write
	I2C2->CR2 &= ~(1 << 10);
	
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
	}
		
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF){
		//GPIOC->ODR |= GPIO_ODR_6; // red if error
	}
	
	// Write data into the TXDR 
	I2C2->TXDR = regAddr;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {
		//GPIOC->ODR ^= GPIO_ODR_9; // green
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
