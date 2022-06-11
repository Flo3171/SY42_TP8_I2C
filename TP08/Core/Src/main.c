/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Made by Ruff Guillaume
  * 		Humbert Celian
  * 		Cloarec Florian
  *
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
uint8_t data;
uint8_t buf[12];
float temp_c;
int16_t val;
static const uint8_t REG_TEMP = 0b00000101;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
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
	HAL_StatusTypeDef ret;

	//HAL_I2C_Mem_Read (, 0b011000, uint16_t MemAddress, I2C_MEMADD_SIZE_16BIT,&dato,1,5);
	//HAL_I2C_Master_Receive (&hi2c1, 0b011000, &buffer, strlen(buffer), 900);
	//HAL_I2C_Master_Transmit (&hi2c1, 0b011000, &buffer,strlen(buffer),  900);



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /*i2c_start(); // send START command
  i2c_write (0b011000 & 0xFE); //WRITE Command (see Section 4.1.4 “Address Byte�?)
  //also, make sure bit 0 is cleared ‘0’
  i2c_write(0x05); // Write TA Register Address
  i2c_start(); //Repeat START
  i2c_write(0b011000 | 0x01); // READ Command (see Section 4.1.4 “Address Byte�?)
  //also, make sure bit 0 is Set ‘1’
  UpperByte = i2c_read(ACK); // READ 8 bits
  //and Send ACK bit
  LowerByte = i2c_read(NAK); // READ 8 bits
  //and Send NAK bit
  i2c_stop();*/

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */



  uint8_t config_adress = 1;
  uint8_t HYSTERESIS_1_5C[2];
  HYSTERESIS_1_5C[0] = 1 << 9;

  uint8_t alert[2];
  uint8_t alert1[2];
  uint8_t alert2[2];

/*configuration des alertes*/
  alert[0]=1<<3; //activation du Alert Output Control bit
  alert1[0]=1<<0; //Alert Output Polarity bit mis a 0 car resistance de rapelle
  alert2[0]=0<<1; //Alert Output Mode bit mis en mode Interrupt output

  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, config_adress, 1, &HYSTERESIS_1_5C, 1, HAL_MAX_DELAY);

  HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b1000, 1, 0b01 , 1, HAL_MAX_DELAY);   //écriture sur le registre de résolution (0b1000) afin de régler la résolution a 0.25°C (0b01)

   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, config_adress, 1, &HYSTERESIS_1_5C, 1, HAL_MAX_DELAY);	//écriture dans le registre de configuration pour régler l'hystéresis a +1.5°C (placement d'un 1 sur le bit 9 du registre)


   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000010, 1, 0b0000000111100000, 1, HAL_MAX_DELAY);	//définition de la borne température "haute" à 30 degrés

   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000011, 1, 0b0000000110000000, 1, HAL_MAX_DELAY); //défition de la borne de température basse à 24 degrés

   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000100, 1, 0b0000001010000000, 1, HAL_MAX_DELAY);  //définition de la bone de température critique à 40 degrés

   //écritures sur le registre de configuration pour configurer les paramettres des alertes

   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000001, 1, &alert, 1, HAL_MAX_DELAY);

   HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000001, 1, &alert1, 1, HAL_MAX_DELAY);

  HAL_I2C_Mem_Write(&hi2c1, 0b011000<<1, 0b00000001, 1, &alert2, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*Deep sleep mode*/
	  /* Clear the WU FLAG */
	  //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	   /* clear the RTC Wake UP (WU) flag */
	 // __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	   /* Enable the WAKEUP PIN */
	//  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	 // HAL_PWR_EnterSTANDBYMode();


	  __WFI(); //endormissement du coeur qui sera reveiller a la prochaine interruption (le timer)
	  HAL_Delay (10000); //fonction de la hal utilisant Systick pour attendre n milliseconde, les mesures et envoies de températures se font donc toutes les 10 secondes

	  uint8_t data[2];

	  HAL_I2C_Mem_Read(&hi2c1, 0b011000<<1, 0b00000101, 1, data, 2, HAL_MAX_DELAY);  //Lecture de la temperature

	  			//conversion et formatage de la donnée récupérée
	  uint8_t UpperByte = data[0];
	  uint8_t LowerByte = data[1];

	  			//basé sur l'exemple fournis dans la documentation du capteur
	  float temp = (UpperByte & 0b00001111);
	  temp = (UpperByte*16+LowerByte/16);

	  		if ((UpperByte & 0x80) == 0x80){ //TA ³ TCRIT
	  				}
	  				if ((UpperByte & 0x40) == 0x40){ //TA > TUPPER
	  				}
	  				if ((UpperByte & 0x20) == 0x20){ //TA < TLOWER
	  				}
	  				UpperByte = UpperByte & 0x1F; //Clear flag bits
	  				if ((UpperByte & 0x10) == 0x10){ //TA < 0°C
	  				UpperByte = UpperByte & 0x0F; //Clear SIGN
	  				temp = 256 - (UpperByte * 16 + LowerByte / 16);
	  				}else //TA ³ 0°C
	  					temp = (UpperByte * 16 + (float)LowerByte / 16);

	  	uint8_t buff[30];

	  	//affichage de la température
	  	uint16_t size = sprintf(&buff,"temperature = %f \n\r", temp);
	  	HAL_UART_Transmit(&huart2, &buff, size, HAL_MAX_DELAY);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 20430, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


/*traitement de l'interruption générer par les alertes*/
/*Lors des tests je ne suis malheureusement pas parvenu a entrer des l'interruption en faisant*/
/* varier la temperature, mais en manipulant les branchement du montage, l'interruption fonctionnait bien*/
/*il s'agit surement d'un problème de configuration des alertes, que je n'ai pas reussi a résoudre*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
	uint8_t buff[50];
	uint16_t size = sprintf(&buff,"limite de temperature depassee\n\r");
	HAL_UART_Transmit(&huart2, &buff, size, HAL_MAX_DELAY);
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
