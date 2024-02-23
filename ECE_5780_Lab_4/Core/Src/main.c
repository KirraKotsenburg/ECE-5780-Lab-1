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



char USART3_ReadChar(void) {
    while(!(USART3->ISR & USART_ISR_RXNE));  // Wait for RXNE (Receive data register not empty)
    return USART3->RDR;  // Read the received data
}
void USART3_WriteChar(char ch) {
    while ((USART3->ISR &= USART_ISR_TXE_Msk) == 0);  // Wait for TXE (Transmit data register empty)
    USART3->TDR = ch;  // Transmit the character
}
// Function to send a string via USART3
void USART3_SendString(const char *str) {
    while (*str) {
        USART3_WriteChar(*str++);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

   HAL_Init();
	/* Configure the system clock */
  SystemClock_Config();
	
  // Enable GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	
	// Enable USART3 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// alternate function mode
	GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5)) 
								| GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; 
								
	// Select appropriate function number in alternate function registers 
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL4_Pos;
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL5_Pos;
	
	// Set the Baudrate
	//uint32_t clk = HAL_RCC_GetHCLKFreq();
	USART3->BRR = 69;
	
	// Enable the Transmitter and Receiver hardware
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable the USART
	USART3->CR1 |= USART_CR1_UE;

	// Enable the Receive not empty interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;
	
	//Set the priority
	NVIC_SetPriority(USART3_4_IRQn, 1);
	
	//Enabling pins 8 and 9 (green and orange)
	GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc89);
	
		//Enabling pins 6 and 7 (green and orange)
	GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc67);
	
  while (1)
  {
		// Part 2
		char color = USART3_ReadChar();
		
		char num = USART3_ReadChar();
		
		if((color == 'r') | (num == 'R')){
				if(num == '0'){
					GPIOC->ODR &= ~GPIO_ODR_6;
				}else if(num == '1'){
					GPIOC->ODR |= GPIO_ODR_6;
				}else if(num == '2'){
					GPIOC->ODR ^= GPIO_ODR_6;
				}else{
					USART3_SendString("Error: Unrecognized input\n\n");
				}
				
		}else if((color == 'g') | (num == 'G')){
				if(num == '0'){
					GPIOC->ODR &= ~GPIO_ODR_9;
				}else if(num == '1'){
					GPIOC->ODR |= GPIO_ODR_9;
				}else if(num == '2'){
					GPIOC->ODR ^= GPIO_ODR_9;
				}else{
					USART3_SendString("Error: Unrecognized input\n\n");
				}
			}else if((color == 'b') | (num == 'B')){
				if(num == '0'){
					GPIOC->ODR &= ~GPIO_ODR_7;
				}else if(num == '1'){
					GPIOC->ODR |= GPIO_ODR_7;
				}else if(num == '2'){
					GPIOC->ODR ^= GPIO_ODR_7;
				}else{
					USART3_SendString("Error: Unrecognized input\n\n");
				}
			}else if((color == 'o') | (num == 'O')){
				if(num == '0'){
					GPIOC->ODR &= ~GPIO_ODR_8;
				}else if(num == '1'){
					GPIOC->ODR |= GPIO_ODR_8;
				}else if(num == '2'){
					GPIOC->ODR ^= GPIO_ODR_8;
				}else{
					USART3_SendString("Error: Unrecognized input\n\n");
				}
				
		}else{
			USART3_SendString("Error: Unrecognized command.\n\n");
		  USART3_SendString("Error: Unrecognised action of LED.");
			
			}
				
		
		
		/* // Part 1
    char ch = USART3_ReadChar();  // Read a character from USART3
    // Process the received character
		
		if(ch == 'o' || ch == 'O')
			GPIOC->ODR ^= GPIO_ODR_8;   // Through If condition if the character entered is 'o' or 'O' the orange LED glows
		else if(ch == 'g' || ch == 'G')
			GPIOC->ODR ^= GPIO_ODR_9;   // Through elseIf condition if the character entered is 'g' or 'G' the green LED glows
		else if(ch == 'r' || ch == 'R')
			GPIOC->ODR ^= GPIO_ODR_6;   // Through If condition if the character entered is 'o' or 'O' the orange LED glows
		else if(ch == 'b' || ch == 'B')
			GPIOC->ODR ^= GPIO_ODR_7;
		else
			USART3_SendString("Error: Unrecognized command\n\n"); 
		*/
  }

  /* USER CODE END 3 */
}

/*
void USART3_4_IRQnHandler(){
	
	receiveRegVal = USART3->RDR;
	
	USART3->CR1 |= USART_CR1_RE;
	
	newData = 1;
}
*/
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
