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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "liquidcrystal_i2c.h"

void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void display_spot(uint16_t spot1, uint16_t spot2){
	//funkcija za ispis podataka na displayu
	  HD44780_Clear();
	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr("Slobodna mjesta");

	  if(spot1 == 1){
		  HD44780_SetCursor(4,1);
		  HD44780_PrintStr("P1");
	  }
	  else{}
	  if(spot2 == 1){
		  HD44780_SetCursor(10,1);
		  HD44780_PrintStr("P2");
	  }
	  else{}
	  if(spot1 == 0 && spot2 == 0){
		  HD44780_Clear();
		  HD44780_SetCursor(1,0);
		  HD44780_PrintStr("Nema slobodnih");
		  HD44780_SetCursor(5,1);
 		  HD44780_PrintStr("mjesta");
	  }
	  HAL_Delay(1000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HD44780_Init(2);


  	//inicijalizacija pomoćnih varijabli
  	uint16_t switchState_P1;
    uint16_t switchState_P2;

    uint16_t spot1;
    uint16_t spot2;


    /* USER CODE BEGIN WHILE */
    while (1)
    {
      /* USER CODE END WHILE */
      //očitavanje stanja senzora
  	  switchState_P1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
  	  switchState_P2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

  	  //provjera stanja sezora i upravljanje signalizacijom
  	  if(switchState_P1 == 0){
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  		  spot1=0;

  	  }
  	  if(switchState_P1 == 1){
  	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  	  		  spot1 = 1;
  	  	  }
  	  if(switchState_P2 == 0){
  	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  	  		  spot2=0;
  	  	  }
  	  if(switchState_P2 == 1){
  	  	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  	  	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  	  	  	  spot2 = 1;
  	  	  }
  	  if(switchState_P1 == 0 && switchState_P2 == 0){

  		  spot1=0;
  		  spot2=0;

  	  }
  	  //pozivanje funkcije za ispis slobodnih mjesta
  	  display_spot(spot1, spot2);

  	/* USER CODE END WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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
