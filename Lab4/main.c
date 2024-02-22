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
	uint32_t newData;
	uint32_t dataAquired;
	uint32_t nextData;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	//enable leds
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->OTYPER &= ~(1<<6 | 1<<7 | 1<<8 | 1<<9);
	GPIOC->OSPEEDR &= ~(1<<12 | 1<<14 | 1<<16 |1 <<18);
	GPIOC->PUPDR &= ~(1<<12 | 1<<14 | 1<<13 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19);	
	
	//Section 4.1
	//Set the pins into alternate function mode
	GPIOC->AFR[0] |= (1<<16 | 1<<20);
	GPIOC->MODER |= (1<<9 | 1<<11);
	
	//enable the usart rcc peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	//set the baud rate
	USART3->BRR = (HAL_RCC_GetHCLKFreq() / 115200) + 1;
	
	//enable transmit and recieve
	USART3->CR1 |= (1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<5);
		
	//enable the interrupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	
  while (1)
  {
		//Call the transmitChar function
		//transmitChar('L');
		//arrayLoop("Iamdying\n");
		HAL_Delay(1000);
		//toggleLEDs();
		
		if(dataAquired){
			twoInputToggle(newData, nextData);
			dataAquired = 0;		
		}
				
  }
  /* USER CODE END 3 */
}

//Set up the handler
void USART3_4_IRQHandler(void){
	if(USART3->ISR & USART_ISR_RXNE){
		newData = USART3->RDR;
		//HAL_Delay(1000);
		nextData = USART3->TDR;
		dataAquired = 1;
	}
}

//toggling the LED's function
void twoInputToggle(uint32_t data, uint32_t nextData){
	while((USART3->ISR) & 1<<5){;}
	char c = data;
	char n = nextData;
	switch(c){
		case 'r':
			if(nextData == 0){
				GPIOC->ODR ^= (1<<6);
			}
			else if(nextData == 1){
				GPIOC->ODR ^= (1<<6);
			}
			else if(nextData == 2){
				while(1){
					GPIOC->ODR ^= (1<<6);
				}
			}
			else
				transmitArray("Error");
			
			transmitChar('r');
		break;
		case 'g':
						if(nextData == 0){
				GPIOC->ODR ^= (1<<9);
			}
			else if(nextData == 1){
				GPIOC->ODR ^= (1<<9);
			}
			else if(nextData == 2){
				while(1){
					GPIOC->ODR ^= (1<<9);
				}
			}
			else
				transmitArray("Error");
			transmitChar('g');
		break;
		case 'b':
						if(nextData == 0){
				GPIOC->ODR ^= (1<<7);
			}
			else if(nextData == 1){
				GPIOC->ODR ^= (1<<7);
			}
			else if(nextData == 2){
				while(1){
					GPIOC->ODR ^= (1<<7);
				}
			}
			else
				transmitArray("Error");
			transmitChar('b');
		break;
		case 'o':
						if(nextData == 0){
				GPIOC->ODR ^= (1<<8);
			}
			else if(nextData == 1){
				GPIOC->ODR ^= (1<<8);
			}
			else if(nextData == 2){
				while(1){
					GPIOC->ODR ^= (1<<8);
				}
			}
			else
				transmitArray("Error");
			transmitChar('o');
		break;
		default: transmitArray("Error");				
		}
}

//toggling the LED's function
void toggleLEDs(void){
	while(!((USART3->ISR) & 1<<5)){;}
	char c = USART3->RDR;
	switch(c){
		case 'r':
			GPIOC->ODR ^= (1<<6);
			transmitChar('r');
		break;
		case 'g':
			GPIOC->ODR ^= (1<<9);
			transmitChar('g');
		break;
		case 'b':
			GPIOC->ODR ^= (1<<7);
			transmitChar('b');
		break;
		case 'o':
			GPIOC->ODR ^= (1<<8);
			transmitChar('o');
		break;
		default: transmitArray("Error");				
		}
}
//create the looping function
void transmitArray(char* string){
	uint32_t i = 0;
	while(string[i] != '\0'){
		transmitChar(string[i]);
		i++;
	}
	
}
void transmitChar(char letter){
	while(!((USART3->ISR) & (1<<7))){;}
	USART3->TDR = letter;
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
