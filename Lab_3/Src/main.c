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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum fsm_state {
	SHOW_TIME,
	SHOW_DATE,
	STORE_TIME_EE,
	SHOW_STORED,
	SET_TIME
} fsm_state;


typedef enum set_state {
	DAY,
	DATE,
	MONTH,
	YEAR,
	HOUR,
	MINUTE,
	SECOND,
} set_state;

typedef enum button {
	NONE,
	SEL,
	LEFT,
	RIGHT,
	UP,
	DOWN
}	button;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
RTC_DateTypeDef date_struct;
RTC_TimeTypeDef time_struct;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;


/* USER CODE BEGIN PV */
#define EEPROM_ADDRESS  0xA0
__IO uint16_t memLocation = 0x0000;
uint16_t EE_status;
char lcd_buffer[6];    // LCD display buffe

fsm_state STATE = SHOW_TIME;
button BUTTON = NONE;

set_state SET_STATE;

uint16_t sel_tick = 0;

uint16_t date, year, sec, min, hour;
char day[6], month[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void show_time();
void store_time();
void show_stored();
void show_date();
void set_time();

void get_day();
void get_month();

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
	HAL_InitTick(0x0000);
	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED4);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	


	/*---------------------TEST I2C and EEPROM----------------------
	uint8_t data1 = 0x80,  data2=0x81;
	uint8_t readData=0x00;
	
	EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation, data1);

  if(EE_status != HAL_OK)
  {
    I2C_Error(&hi2c1);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+1 , data2);////////////////////////////////
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&hi2c1);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation); 

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+1); 

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	
	BSP_LCD_GLASS_Clear();
	sprintf(lcd_buffer,"%d",readData);
	BSP_LCD_GLASS_DisplayString((uint8_t*) lcd_buffer);
	*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(STATE) {
			case SHOW_TIME:
				show_time();
				break;
		
			case SHOW_DATE:
				show_date();
				STATE = SHOW_TIME;
				break;
		
			case STORE_TIME_EE:
				store_time();
				STATE = SHOW_TIME;
				break;
		
			case SHOW_STORED:
				show_stored();
				STATE = SHOW_TIME;
				break;
		
			case SET_TIME:
				set_time();
				break;
		
			default:
				break;
			
			BUTTON = NONE;
	}
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 9;
  sTime.Minutes = 30;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 5;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"AA ERR");
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);
  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

	HAL_NVIC_SetPriority((IRQn_Type)EXTI15_10_IRQn, 3, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{	
	//BSP_LED_Toggle(LED4);					//test RTC ALARM
	
	switch(STATE) {
		case SHOW_TIME:
			HAL_RTC_GetTime(hrtc, &time_struct, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(hrtc, &date_struct, RTC_FORMAT_BIN);	// get date unlocks

			sec =  time_struct.Seconds;														//write to vars
			min =  time_struct.Minutes;
			hour = time_struct.Hours;
		

			sprintf(lcd_buffer,"%02u%02u%02u",hour,min,sec); 			//populate string

			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);		//display
		
			break;
		
		default:
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {							//buffer button pressed
	switch(GPIO_Pin) {
		
		case GPIO_PIN_0:
			BUTTON = SEL;
			break;
		
		case GPIO_PIN_1:
			BUTTON = LEFT;
			break;
		
		case GPIO_PIN_2:
			BUTTON = RIGHT;
			break;
		
		case GPIO_PIN_3:
			BUTTON = UP;
			break;
		
		case GPIO_PIN_5:
			BUTTON = DOWN;																					
			break;

	} 
}


void show_time(){
	
	if(BSP_JOY_GetState() == JOY_SEL){
		uwTick = 0;																				//initial tick time
		while(BSP_JOY_GetState() == JOY_SEL){															
			if (uwTick > 1000) {										//check if the difference in current time is greater than 1 second, if so show date
				STATE = SHOW_DATE;																	
				BUTTON = NONE;
				return;
			}
		}
		STATE = STORE_TIME_EE;
	}
	
	switch(BUTTON) {
		
		case LEFT:																																// show stored times
			STATE = SHOW_STORED;
		
			break;
		
		case DOWN:																																// clear EEPROM
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation, 0);
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+1, 0);
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+2, 0);
			
			memLocation = (memLocation+3)%6;
		
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation, 0);
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+1, 0);
			I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+2, 0);	
		
			break;
		
		case RIGHT:
			BSP_LCD_GLASS_Clear();
		
			STATE = SET_TIME;	
			SET_STATE = DAY;
			break;
		
		default:
			break;
	}
	BUTTON = NONE;																						// clear button incase it was pressed on accident in this state
	
}

void store_time(){
	
	memLocation = (memLocation+3)%6;													//cycle memory locations 0x00 to 0x05 (6 spots)
	
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation, sec);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+1, min);
  I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, memLocation+2, hour);
	
	BUTTON = NONE;																						// clear button incase it was pressed on accident in this state
		
}

void show_stored(){


	sec = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation); 							//display first stored time
	min = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+1); 
	hour = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+2);
	
	sprintf(lcd_buffer,"%02u%02u%02u",hour,min,sec); 			//populate string

	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);		//display
	
	HAL_Delay(1000);
	
	memLocation = (memLocation+3)%6;																					//increment to next 3 locations
	
	sec = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation); 							//display second stored time
	min = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+1); 
	hour = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+2);
	
	sprintf(lcd_buffer,"%02u%02u%02u",hour,min,sec); 			//populate string

	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);		//display
	
	HAL_Delay(1000);
	
	memLocation = (memLocation+3)%6;																					//increment again (cycle back)
	
	BUTTON = NONE;												// clear button incase it was pressed on accident in this state

}

void show_date(){								// show date state		
	
	date = date_struct.Date;						//get date
	year = 2020 + date_struct.Year;			//getc year
	
	get_day();
	get_month();
	
	BSP_LCD_GLASS_Clear();																					//print day
	BSP_LCD_GLASS_DisplayString((uint8_t*)day);			
	HAL_Delay(1000);
	
	sprintf(lcd_buffer,"%02u",date);																//print date
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
	HAL_Delay(1000);


	BSP_LCD_GLASS_Clear();																					//print month
	BSP_LCD_GLASS_DisplayString((uint8_t*)month);
	HAL_Delay(1000);
	
	sprintf(lcd_buffer,"%02u",year);																//print year
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
	HAL_Delay(1000);
	
	
	BUTTON = NONE;// clear button incase it was pressed on accident in this state

}

void set_time(){
	
	
	switch(SET_STATE){
		case DAY:
			get_day();																	//print day
			BSP_LCD_GLASS_DisplayString((uint8_t*)day);
			if(BUTTON == UP){
				date_struct.WeekDay = date_struct.WeekDay%7 + 1;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(date_struct.WeekDay == 1) date_struct.WeekDay = 7;
				else date_struct.WeekDay--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case DATE:	
			date = date_struct.Date;						//get date
			sprintf(lcd_buffer,"%02u",date);																//print date
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			if(BUTTON == UP){
				date_struct.Date = date_struct.Date%31 + 1;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(date_struct.Date == 1) date_struct.Date = 31;
				else date_struct.Date--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case MONTH:
			get_month();																				//print month
			BSP_LCD_GLASS_DisplayString((uint8_t*)month);
			if(BUTTON == UP){
				date_struct.Month = date_struct.Month%12 + 1;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(date_struct.Month == 1) date_struct.Month = 12;
				else date_struct.Month--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case YEAR:
			year = 2020 + date_struct.Year;														//getc year
			sprintf(lcd_buffer,"%02u",year);																//print year
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			if(BUTTON == UP){
				if(date_struct.Year < 99) date_struct.Year++;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(date_struct.Year > 0) date_struct.Year--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case HOUR:
			hour = time_struct.Hours;
			sprintf(lcd_buffer,"%02uxxxx",hour);																//print hour
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			if(BUTTON == UP){
				time_struct.Hours = (time_struct.Hours+1)%24 ;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(time_struct.Hours == 0) time_struct.Hours = 23;
				else time_struct.Hours--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case MINUTE:
			min = time_struct.Minutes;
			sprintf(lcd_buffer,"xx%02uxx",min);																//print min
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			if(BUTTON == UP){
				time_struct.Minutes = (time_struct.Minutes+1)%60;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(time_struct.Minutes == 0) time_struct.Minutes = 59;
				else time_struct.Minutes--;
				BSP_LCD_GLASS_Clear();
			}
			break;
		
		case SECOND:
			sec = time_struct.Seconds;
			sprintf(lcd_buffer,"xxxx%02u",sec);																//print sec
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			if(BUTTON == UP){
				time_struct.Seconds = (time_struct.Seconds+1)%60;
				BSP_LCD_GLASS_Clear();
			}
			else if(BUTTON == DOWN){
				if(time_struct.Seconds == 0) time_struct.Seconds = 59;
				else time_struct.Seconds--;
				BSP_LCD_GLASS_Clear();
			}
			break;
	}
	
	switch(BUTTON){
		case LEFT:
			if(SET_STATE == 0) SET_STATE = 6;
			else SET_STATE--;
			BSP_LCD_GLASS_Clear();
			break;
		case RIGHT:
			SET_STATE = (SET_STATE+1)%7;
			BSP_LCD_GLASS_Clear();
			break;
		
		case SEL:
			if (HAL_RTC_SetTime(&hrtc, &time_struct, RTC_FORMAT_BIN) != HAL_OK)	{
				Error_Handler();
			}
			if (HAL_RTC_SetDate(&hrtc, &date_struct, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			
			STATE = SHOW_TIME;
			HAL_Delay(500);
			break;
		default:
			break;
	}
	
	BUTTON = NONE;
}


void get_day(){
	switch(date_struct.WeekDay){				// assign weekday from structure
		case 1: 
			strcpy(day, "MON");
			break;
		case 2:
			strcpy(day, "TUE");
			break;
		case 3:
			strcpy(day, "WED");
			break;
		case 4:
			strcpy(day, "THU");
			break;
		case 5:
			strcpy(day, "FRI");
			break;
		case 6:
			strcpy(day, "SAT");
			break;
		case 7:
			strcpy(day, "SUN");
			break;
	} 
}

void get_month(){
	switch(date_struct.Month){				//Assign month from structure
		case 1:
			strcpy(month, "JAN");
			break;
		case 2:
			strcpy(month, "FEB");
			break;
		case 3:
			strcpy(month, "MAR");
			break;
		case 4:
			strcpy(month, "APR");
			break;
		case 5:
			strcpy(month, "MAY");
			break;
		case 6:
			strcpy(month, "JUN");
			break;
		case 7:
			strcpy(month, "JUL");
			break;
		case 8:
			strcpy(month, "AUG");
			break;
		case 9:
			strcpy(month, "SEP");
			break;
		case 10:
			strcpy(month, "OCT");
			break;
		case 11:
			strcpy(month, "NOV");
			break;
		case 12:
			strcpy(month, "DEC");
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
