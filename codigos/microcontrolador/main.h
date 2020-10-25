/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_it.h"
#include "lib_aux.h"
#include "MCC3416_def.h"
#include "stm32l4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define DATAREADY_Pin GPIO_PIN_8
#define DATAREADY_GPIO_Port GPIOA
#define DATAREADY_EXTI_IRQn EXTI9_5_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

#define PHEx_Pin GPIO_PIN_4
#define PHEx_GPIO_Port GPIOB
#define PHIn_Pin GPIO_PIN_5
#define PHIn_GPIO_Port GPIOB
#define SSRPA_Pin GPIO_PIN_0
#define SSRPA_GPIO_Port GPIOB
#define SSRPB_Pin GPIO_PIN_7
#define SSRPB_GPIO_Port GPIOB
#define SSRNA_Pin GPIO_PIN_6
#define SSRNA_GPIO_Port GPIOB
#define SSRNB_Pin GPIO_PIN_1
#define SSRNB_GPIO_Port GPIOB

#define TRUE 	1
#define FALSE 	0

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
