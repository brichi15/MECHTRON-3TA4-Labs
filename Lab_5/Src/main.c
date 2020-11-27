/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum state{
	CW,
	CW_HALF,
	CCW,
	CCW_HALF
}state ;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
char lcd_buffer[6];
state STATE;						//state
uint16_t phase_count;						//counts cycles

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void cw_or_ccw_state(void);
void cw_half_or_ccw_half_state(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	BSP_LCD_GLASS_Init();
	
	STATE = CW;
	phase_count = 3; 							//start at 3 because increment will make it 0
	
	HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				
	HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8541;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		 //Starting Error 
		Error_Handler();
	}
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|OUT_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOY_CENTER_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_CENTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_A_Pin OUT_B_Pin OUT_C_Pin OUT_D_Pin */
  GPIO_InitStruct.Pin = OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|OUT_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if ((*htim).Instance==TIM3)	{										//tim3 clock
		
		switch(STATE){
			case CW:
				phase_count = (phase_count+1)%4;							//count forwards
				cw_or_ccw_state();
				break;
			
			case CW_HALF:
				phase_count = (phase_count+1)%8;
				cw_half_or_ccw_half_state();
				break;
			
			case CCW:
				phase_count--;							//count backwards
				if(phase_count>3) phase_count = 3;
				cw_or_ccw_state();
				break;
			
			case CCW_HALF:
				phase_count--;							//count backwards
				if(phase_count>7) phase_count = 7;
				cw_half_or_ccw_half_state();
				break;

		}
		
		
		sprintf(lcd_buffer,"%u",phase_count+1);
		BSP_LCD_GLASS_Clear();					
		BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);					//display phase
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {							//buffer button pressed
	switch(GPIO_Pin) {
		
		case GPIO_PIN_0:					//SELECT
			if(STATE==CW) 						STATE = CCW;
			else if(STATE==CCW) 			STATE = CW;
		  else if(STATE==CW_HALF)		STATE = CCW_HALF;
			else if(STATE==CCW_HALF)	STATE = CW_HALF;
				
			break;
		
		case GPIO_PIN_1:					//LEFT
			if(STATE==CW) 						STATE = CW_HALF;
			else if(STATE==CW_HALF) 	STATE = CW;
		  else if(STATE==CCW)				STATE = CCW_HALF;
			else if(STATE==CCW_HALF)	STATE = CCW;
		
			if(STATE == CW || STATE == CCW)  htim3.Init.Period *= 2;									//state changed to CW or CCW, double the period						
			else htim3.Init.Period /= 2;																							//state changed to CW_HALF or CCW_HALF, half the period
			if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
			
		 break;
		
		case GPIO_PIN_2:					//RIGHT
			break;
		
		case GPIO_PIN_3:					//UP	
			if(htim3.Init.Period > 1000) htim3.Init.Period -= 1000;										//speed up, decrease period
			if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
			break;
		
		
		case GPIO_PIN_5:					//DOWN
			htim3.Init.Period += 1000;																							//decrease speed, increase period
			if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
			break;
	} 
}

void cw_or_ccw_state(void){																//pin order for CW or CCW 
	
	switch(phase_count){
		case 0:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_SET);					//1
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_SET);				//2
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//3
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//4
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
	}
}

void cw_half_or_ccw_half_state(void){																//pin order for CW_HALF or CCW_HALF 
	
	switch(phase_count){
		case 0:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_SET);					//1
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_SET);				//2
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_SET);				//3
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//4
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//5
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//6
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//7
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOE,OUT_A_Pin,GPIO_PIN_RESET);				//8
			HAL_GPIO_WritePin(GPIOE,OUT_B_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,OUT_C_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,OUT_D_Pin,GPIO_PIN_RESET);
			break;
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
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"error");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
