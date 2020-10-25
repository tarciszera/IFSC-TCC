/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UART comunication with I2C sensor control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "lib_aux.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

typedef enum dieBruter {
	WAIT_FOR_COMMAND = 0,
	SET_INNER_COIL_POLARITY,
	RESET_INNER_COIL_POLARITY,
	SET_MIDDLE_COIL_POLARITY,
	RESET_MIDDLE_COIL_POLARITY,
	SET_EXTERNAL_COIL_POLARITY,
	RESET_EXTERNAL_COIL_POLARITY,
	DO_MEASUREMENT,
	START_UP_POLARITY,
	CONTINUOUS_MEASUREMENT_STATE
}stateType;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(int count);

volatile unsigned int flagTIM2 = FALSE;

int askValue(const char* message);
int measurementReadyToRead();
void resetRxBuffer();
stateType verifyCommandRxBuffer();
int measurementProcedure();
void resetSSRBridge();
void resetExternalPolarity();
void setExternalPolarity();
void resetInnerPolarity();
void setInnerPolarity();
void resetMiddlePolarity();
void setMiddlePolarity();


int dataMeanSize;
int nTimerCount;

stateType state = WAIT_FOR_COMMAND;

unsigned int i = 0;
float xfValue = 0;
float yfValue = 0;
float zfValue = 0;
int middleFlag = 0;

float data[3];

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  // Sensor configuration
  MMC3416verifyConection();
  configureMMC3416();

  resetMiddlePolarity();
  resetInnerPolarity();
  resetExternalPolarity();

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))
  {
	  dataMeanSize = askValue("DataMeanSize?\n");
	  nTimerCount = askValue("nTimerCount?\n");
  }
  else
  {
	  dataMeanSize = 30;
  	  nTimerCount = 300;
  }

  MX_TIM2_Init(nTimerCount);
  HAL_TIM_Base_Start_IT(&htim2);


  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) // D2 pin, it's in pull up
  {
	  state = WAIT_FOR_COMMAND;
  }

  resetRxBuffer();

  while (1)
  {
	  if (flagTIM2)
	  {
		  switch(state) {

			 case WAIT_FOR_COMMAND  :
				state = verifyCommandRxBuffer();
				break;

			 case SET_INNER_COIL_POLARITY  :
				setInnerPolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "I Done\n");
				break;

			 case RESET_INNER_COIL_POLARITY  :
				resetInnerPolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "i Done\n");
				break;

			 case SET_MIDDLE_COIL_POLARITY  :
				setMiddlePolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "M Done\n");
				break;

			 case RESET_MIDDLE_COIL_POLARITY  :
				resetMiddlePolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "m Done\n");
				break;

			 case SET_EXTERNAL_COIL_POLARITY  :
				setExternalPolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "E Done\n");
				break;

			 case RESET_EXTERNAL_COIL_POLARITY  :
				resetExternalPolarity();
				state = WAIT_FOR_COMMAND;

				UART_transmit_message(aTxBuffer, "e Done\n");
				break;

			 case DO_MEASUREMENT  :
				if(!measurementProcedure())
				{
					state = WAIT_FOR_COMMAND;
				}
				break;

			 case START_UP_POLARITY  :

				resetExternalPolarity();
				resetInnerPolarity();
				resetMiddlePolarity();

				state = WAIT_FOR_COMMAND;
				UART_transmit_message(aTxBuffer, "S Done\n");

				break;

			 case CONTINUOUS_MEASUREMENT_STATE  :
			    measurementProcedure();
				break;
			 default :
			 state = WAIT_FOR_COMMAND;
		  }

	  }
  }
}


// User functions  ----------------------------------------------------------------------------------------

void resetSSRBridge()
{
	HAL_GPIO_WritePin(SSRPA_GPIO_Port, SSRPA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSRPB_GPIO_Port, SSRPB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSRNA_GPIO_Port, SSRNA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSRNB_GPIO_Port, SSRNB_Pin, GPIO_PIN_RESET);
}

void resetExternalPolarity()
{
	HAL_GPIO_WritePin(PHEx_GPIO_Port, PHEx_Pin, GPIO_PIN_RESET);
}

void setExternalPolarity()
{
	HAL_GPIO_WritePin(PHEx_GPIO_Port, PHEx_Pin, GPIO_PIN_SET);
}

void resetInnerPolarity()
{
	HAL_GPIO_WritePin(PHIn_GPIO_Port, PHIn_Pin, GPIO_PIN_RESET);
}

void setInnerPolarity()
{
	HAL_GPIO_WritePin(PHIn_GPIO_Port, PHIn_Pin, GPIO_PIN_SET);
}

void resetMiddlePolarity()
{
	resetSSRBridge();
	HAL_GPIO_WritePin(SSRPA_GPIO_Port, SSRPA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SSRNB_GPIO_Port, SSRNB_Pin, GPIO_PIN_SET);
}

void setMiddlePolarity()
{
	resetSSRBridge();
	HAL_GPIO_WritePin(SSRPB_GPIO_Port, SSRPB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SSRNA_GPIO_Port, SSRNA_Pin, GPIO_PIN_SET);
}

int measurementProcedure()
{
	int ret = 1;

	MMC3416_SET_RESET_offsetCancel_Measurement(data);

	xfValue += data[0];
	yfValue += data[1];
	zfValue += data[2];

	i++;
	if (i >= dataMeanSize)
	{


		xfValue = xfValue/dataMeanSize ;
		yfValue = yfValue/dataMeanSize ;
		zfValue = zfValue/dataMeanSize ;
		sendSerialData(xfValue, yfValue, zfValue);

		i = 0;

		xfValue = 0;
		yfValue = 0;
		zfValue = 0;

		flagTIM2 = FALSE; 		// gets TRUE in the TIM2 interrupt handler
		ret = 0;
	}

	return ret;
}

stateType verifyCommandRxBuffer()
{
	stateType state = WAIT_FOR_COMMAND;
	if (aRxBuffer[0] != '\0')
	{
	  if (aRxBuffer[0] == 'I')
	  {
		  state = SET_INNER_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'i')
	  {
		  state = RESET_INNER_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'M')
	  {
		  state = SET_MIDDLE_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'm')
	  {
		  state = RESET_MIDDLE_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'E')
	  {
		  state = SET_EXTERNAL_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'e')
	  {
		  state = RESET_EXTERNAL_COIL_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'D')
	  {
		  state = DO_MEASUREMENT;
	  }
	  else if (aRxBuffer[0] == 'S')
	  {
		  state = START_UP_POLARITY;
	  }
	  else if (aRxBuffer[0] == 'C')
	  {
		  state = CONTINUOUS_MEASUREMENT_STATE;
	  }
	  else
	  {
		  UART_transmit_message(aTxBuffer, "Invalid Command :(\n");
	  }
	}

	resetRxBuffer();
	return state;
}


void resetRxBuffer()
{
	aRxBuffer[0] = '\0';
	HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, sizeof(aRxBuffer));
	huart2.pRxBuffPtr = (uint8_t*)aRxBuffer;
}

int askValue(const char* message)
{
	unsigned int i = 0;

	UART_transmit_message(aTxBuffer, message);

	aRxBuffer[0] = '\0';

	HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, sizeof(aRxBuffer));

	while (1)
	{
	  HAL_Delay(100);
	  if (aRxBuffer[0] != '\0')
		  break;
	  else
	  {
		  resetRxBuffer();
	  }
	}

	int dataMeanSize = toString(aRxBuffer);

	if (dataMeanSize < 0)
	  dataMeanSize = -dataMeanSize;

	for (i = 0; i < 1000; i++)
	{
	if(aRxBuffer[i] == '\0')
	{
		aTxBuffer[i] = '\n';
		i++;
		break;
	}
	else
		aTxBuffer[i] = aRxBuffer[i];
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, i, 500);
	return dataMeanSize;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	huart2.pRxBuffPtr = (uint8_t*)aRxBuffer;
}

// --------------------------------------------------------------------------------------------------------------


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(int count)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = count;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {

    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(DATAREADY_GPIO_Port, DATAREADY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATAREADY_Pin
  GPIO_InitStruct.Pin = DATAREADY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DATAREADY_GPIO_Port, &GPIO_InitStruct);
   */

  /*Configure GPIO pin : PA12 */ //D2
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */ //D3
  GPIO_InitStruct.Pin = SSRPA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSRPA_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SSRPA_GPIO_Port, SSRPA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */ //D4
  GPIO_InitStruct.Pin = SSRPB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSRPB_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SSRPB_GPIO_Port, SSRPB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */ //D5
  GPIO_InitStruct.Pin = SSRNA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSRNA_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SSRNA_GPIO_Port, SSRNA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */ //D6
  GPIO_InitStruct.Pin = SSRNB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSRNB_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SSRNB_GPIO_Port, SSRNB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB4 */ //D12
  GPIO_InitStruct.Pin = PHEx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHEx_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(PHEx_GPIO_Port, PHEx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */ //D11
  GPIO_InitStruct.Pin = PHIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHIn_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(PHIn_GPIO_Port, PHIn_Pin, GPIO_PIN_RESET);


  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
