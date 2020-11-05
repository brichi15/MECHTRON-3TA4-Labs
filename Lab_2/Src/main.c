/* USER CODE BEGIN Header */
/*******************************************************************************
	*MAC ID:			chiuh1
	*STUDENT NUM: 400054774
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;


/* USER CODE BEGIN PV */
enum states {									//	create states for finite state machine
	START,										//led flashing at 1HZ waiting for user input
	RANDOM_WAIT,							//generate random time between 1-4 seconds, wait this time
	WAIT_FOR_REACTION,				//turn on LED and start timer, wait for reaction to stop timer
	END_DISPLAY,							//display reaction time
	CHEATER,									//display Cheater!
};

enum states STATE;

uint32_t wait_time;

uint16_t best_time = 0;
uint16_t best_time_toggle = 0;

int m_sec_elapsed;
char m_sec_string[6] = "000000";

uint16_t VirtAddVarTab = 0x5555; // EEPROM saves 1 variable at this address

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */
void select_button_FSM(void);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
	BSP_LED_Init(LED4);																				//LED INIT
	BSP_LED_Init(LED5);
	
	HAL_TIM_Base_Stop_IT(&htim4);															//stop 1000hz timer
	
	BSP_LCD_GLASS_Init();																			//LCD INIT
	
	BSP_JOY_Init(JOY_MODE_EXTI);														//Joystick init
	
	EE_Init();
	
	STATE = START;
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Start");
	
	/*
	//testing RNG
	HAL_RNG_GenerateRandomNumber(&hrng, &wait_time);		//generate random number
	wait_time = (wait_time%3000) + 1000;								// scale to 1000-4000 milliseconds
	sprintf(m_sec_string,"%u",wait_time);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);
	*/
	
	/*
	//testing EEPROM
	EE_WriteVariable(VirtAddVarTab,16);
	EE_ReadVariable(VirtAddVarTab, &best_time);	
	sprintf(m_sec_string,"%u",best_time);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);
	*/
	
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim3.Init.Period = 9999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		 //Starting Error 
		Error_Handler();
	}
  /* USER CODE END TIM4_Init 2 */

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//see stm321xx_hal_tim.c for different callback function names
	
	if ((*htim).Instance==TIM3 && STATE==START )	{													
		BSP_LED_Toggle(LED4);	
		BSP_LED_Toggle(LED5);	
	}
	
	else if((*htim).Instance==TIM4){
		
		if(STATE==RANDOM_WAIT){
			m_sec_elapsed++;
			if(m_sec_elapsed >= wait_time){									//after random wait time turn on LED and switch state
				
				BSP_LCD_GLASS_Clear();
				
				BSP_LED_On(LED4);															//turn on both LEDs
				BSP_LED_On(LED5);
				
				m_sec_elapsed = 0;															//clear counter
				STATE = WAIT_FOR_REACTION;
			}				
			
		}
		
		else if(STATE==WAIT_FOR_REACTION){								//waiting for reaction state
			m_sec_elapsed++;
			
			if(m_sec_elapsed%50 == 0){														//write on LCD only once every 50 ms, bc cannot write to LCD too frequently 
				sprintf(m_sec_string,"%u",m_sec_elapsed);
				BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
		
		case GPIO_PIN_0:	//___________________________select button	(action button)________________________//
			select_button_FSM();
			break;
		
		
		case GPIO_PIN_3:	//___________________________up button (reset button)______________________________//
			
			HAL_TIM_Base_Stop_IT(&htim4);															//stop 1000hz timer
			HAL_TIM_Base_Start_IT(&htim3);														//start 1hz timer
		
			best_time_toggle = 0;																		//display own score toggle
			
			STATE = START;																						//go to start state
		
			BSP_LCD_GLASS_Clear();																		//display "start"
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Start");
			break;
		
		case GPIO_PIN_5:	//___________________________down button (show score button)_______________________//
			
			if(STATE == START || STATE == END_DISPLAY){
				best_time_toggle = (best_time_toggle+1)%2;
				
				if(best_time_toggle == 1){															//display best time	
					EE_ReadVariable(VirtAddVarTab, &best_time);
					if(best_time == 0){
						BSP_LCD_GLASS_Clear();															//NO TIME	
						BSP_LCD_GLASS_DisplayString((uint8_t*)"NO TIM");
					}
					else{
						sprintf(m_sec_string,"%u",best_time);
						BSP_LCD_GLASS_Clear();																	
						BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);
					}
				}		
				else{
					if(STATE == START){																										//display start
						BSP_LCD_GLASS_Clear();																	
						BSP_LCD_GLASS_DisplayString((uint8_t*)"start");
					}					
					else if(STATE == END_DISPLAY){																				//display recent time
						sprintf(m_sec_string,"%u",m_sec_elapsed);
						BSP_LCD_GLASS_Clear();																	
						BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);
					}					
				}
				
			}
			break;
			
			case GPIO_PIN_1:	//___________________________left button (reset top score)________________________//
				
				if(STATE == START){																	//can only reset score on the start state
					best_time = 0;																		//make best time zero
					EE_WriteVariable(VirtAddVarTab,best_time);				//update eeprom
				}
				break;
	} 
}

	//This function changes the states of the program on selection button press
void select_button_FSM(void){
	switch(STATE) {
		case START:								/*_____________________________START CASE_____________________________*/
		
			m_sec_elapsed = 0;																	// clear counter
		  BSP_LCD_GLASS_Clear();															//clear screen
			
			HAL_TIM_Base_Stop_IT(&htim3);												//stop 1hz timer
		
		  STATE = RANDOM_WAIT;																// go to random wait state
			
			HAL_RNG_GenerateRandomNumber(&hrng, &wait_time);		//generate random number
			wait_time = (wait_time%3000) + 1000;								// scale to 1000-4000 milliseconds
			
			HAL_TIM_Base_Start_IT(&htim4);											//start 1000hz timer
		
		BSP_LCD_GLASS_DisplayString((uint8_t*)"Wait");				//display "wait"
		
			BSP_LED_Off(LED4);																	//turm off LEDs
			BSP_LED_Off(LED5);
			break;
		
		case RANDOM_WAIT:								/*_____________________________WAIT CASE_____________________________*/
			
			BSP_LCD_GLASS_Clear();
			HAL_TIM_Base_Stop_IT(&htim4);																					//stop 1000hz timer
			
			STATE = CHEATER;																											//go to cheater state
		
			BSP_LCD_GLASS_ScrollSentence((uint8_t*)"      YOURE A CHEATER",1,150);		//display "Youre a cheater"
			break;
		
		case WAIT_FOR_REACTION:					/*________________________WAIT FOR REACTION CASE____________________*/
			
			HAL_TIM_Base_Stop_IT(&htim4);														//stop 1000hz timer
			
			STATE = END_DISPLAY;																		// go to end
		
			sprintf(m_sec_string,"%u",m_sec_elapsed);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)m_sec_string);		//display result 
		
			EE_ReadVariable(VirtAddVarTab, &best_time);							//check and compare with eeprom saved time, if less, replace the time
			
			if(m_sec_elapsed < best_time || best_time == 0){				//0 == no time, 0 is not possible anyways so its ok
				best_time = m_sec_elapsed;
				EE_WriteVariable(VirtAddVarTab,best_time);
			}
			
			best_time_toggle = 0;																		//display own score toggle
			
			break;
		
		case END_DISPLAY:							/*_____________________END DISPLAY CASE________________________________*/
			
			STATE = START;																				//go to start
			
			HAL_TIM_Base_Start_IT(&htim3);												//start 1hz timer
		
			BSP_LCD_GLASS_Clear();																//display "start"
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Start");				
		
			best_time_toggle = 0;																		//display own score toggle
		
			break;	
		
		case CHEATER:									/*____________________________CHEATER CASE___________________________*/
			
			STATE = START;																									//go to start
			
			HAL_TIM_Base_Start_IT(&htim3);																	//start 1hz timer
		
			BSP_LCD_GLASS_Clear();																					//display "start"
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Start");									//start 1hz timer
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
