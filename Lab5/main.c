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
  HAL_Init();
  SystemClock_Config();
	
	//5.1.1 Enable GPIOB & GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Enable the LED's
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	GPIOC->OTYPER &= ~(1<<6 | 1<<7 | 1<<8 | 1<<9);
	GPIOC->OSPEEDR &= ~(1<<12 | 1<<14 | 1<<16 |1 <<18);
	GPIOC->PUPDR &= ~(1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 1<<17 | 1<<18 | 1<<19);	
	
	//5.2.2 set PB11
	GPIOB->MODER |= (1<<23 || 0<<22);
	GPIOB->OTYPER |= (1<<11);
	GPIOB->AFR[1] |= (1<<12);
		
	//5.2.3 set PB13
	GPIOB->MODER |= (1<<27);
	GPIOB->OTYPER |= (1<<13);
	GPIOB->AFR[1] |= (1<<20 | 1<<22);

	//5.2.4 set PB14
	GPIOB->MODER |= (1<<28);
	GPIOB->ODR |= (1<<14);
	
	//5.2.5 set PC0
	GPIOC->MODER |= (1<<0);
	GPIOC->ODR |= (1<<0);
	
	//5.3.1 set the I2C
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//5.3.2 set up the TIMINGR
	I2C2->TIMINGR = 
		(1<<28)     //PreScaler
	| (0x4 << 20) //SCLDEL
	| (0x2<<16)   //SDADEL
	| (0xF<<8)    //SCLH
	| (0x13<<0)   //SCLL
	| (I2C2->TIMINGR & (0xF<<24));

	//5.3.2 enable the I2C Peripheral
	I2C2->CR1 |= (1<<0);
	
	//5.4.1
	I2C2->CR2 |= (1<<16);
	
	//set wrn bit
	I2C2->CR2 |= (0<<10);
	
	//set start bit
	I2C2->CR2 |= (1<<13);
	
	/* Clear the NBYTES and SADD bit fields
	* The NBYTES field begins at bit 16, the SADD at bit 0
	*/
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	/* Set NBYTES = 42 and SADD = 0x14
	* Can use hex or decimal values directly as bitmasks.
	* Remember that for 7-bit addresses, the lowest SADD bit
	* is not used and the mask must be shifted by one.
	*/
	I2C2->CR2 |= (42 << 16) | (0x14 << 1);
	
	

  while (1)
  {
		
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
