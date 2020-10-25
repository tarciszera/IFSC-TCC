

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_AUX_H
#define __LIB_AUX_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include <string.h>
#include "stm32l4xx_hal_conf.h"
#include "MCC3416_def.h"

// Usart 2 CODE --------------------------------------------------------------------------

//#define USART_CR1_FIFOEN
#define BUFSIZE 	50

char aTxBuffer[BUFSIZE];
char aRxBuffer[BUFSIZE];
//  --------------------------------------------------------------------------------------

// I2C CODE ----------------------------
unsigned char i2cData[10]; 		// i2c data buffer
#define I2C_WRITE 				0
#define I2C_READ 				1
//  ------------------------------------

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

void UART_transmit_message(char* txBuffer, const char* message);
void sendSerialData(float x, float y, float z);
int toString(char* a);

void MMC3416verifyConection();
void configureMMC3416();

void MMC3416_Ask_Measurement();
unsigned int MMC3416_MeasurementIsDone(unsigned int nTries);
void MMC3416_Measurement_Operation(int* dataVector);

void MMC3416_Write_Register(unsigned char* i2cData, unsigned int nRegToWrite);
void MMC3416_Read_Register(unsigned char* i2cData, unsigned int nRegToRead);

void MMC3416_Recap_Operation();
void MMC3416_SET_Operation();
void MMC3416_RESET_Operation();
void MMC3416_SET_RESET_offsetCancel_Measurement(float* dataVector);

#ifdef __cplusplus
}
#endif

#endif /* __LIB_AUX_H */
