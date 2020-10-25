
/* Includes ------------------------------------------------------------------*/
#include "lib_aux.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_hal.h"

int toString(char* a) {
  int c, sign, offset, n;

  if (a[0] == '-') {  // Handle negative integers
    sign = -1;
  }

  if (sign == -1) {  // Set starting position to convert
    offset = 1;
  }
  else {
    offset = 0;
  }

  n = 0;

  for (c = offset; a[c] != '\0'; c++) {

	if (a[c] < '0' || a[c] > '9')
		return 0;

    n = n * 10 + a[c] - '0';
  }

  if (sign == -1) {
    n = -n;
  }

  return n;
}


void sendSerialData(float x, float y, float z)
{

	unsigned int i = 0;
	int aux;

	char buffer0[15];
	char buffer1[12];
	char buffer2[40];
	buffer0[0] = '\0';
	buffer1[0] = '\0';
	buffer2[0] = '\0';

	if ((x > -1) && (x < 0))
	{
		strncat(buffer2, "-\0", 2);
	}

	itoa((int)x, buffer0, 10);


	if(x < 0)
	  x = -x;

	aux = (int)(((int)(x*100))%100);

	itoa(aux, buffer1, 10);


	strncat(buffer2, buffer0, 5);
	strncat(buffer2, ".", 1);
	if (aux < 10)
		strncat(buffer2, "0", 1);
	strncat(buffer2, buffer1, 2);
	strncat(buffer2, ";", 1);

	if ((y > -1) && (y < 0))
	{
		strncat(buffer2, "-\0", 2);
	}

	itoa((int)y, buffer0, 10);


	if(y < 0)
	  y = -y;

	aux = (int)(((int)(y*100))%100);

	itoa(aux, buffer1, 10);

	strncat(buffer2, buffer0, 5);
	strncat(buffer2, ".", 1);
	if (aux < 10)
		strncat(buffer2, "0", 1);
	strncat(buffer2, buffer1, 2);
	strncat(buffer2, ";", 1);

	if ((z > -1) && (z < 0))
	{
		strncat(buffer2, "-\0", 2);
	}

	itoa((int)z, buffer0, 10);

	if(z < 0)
	  z = -z;

	aux = (int)(((int)(z*100))%100);

	itoa(aux, buffer1, 10);

	strncat(buffer2, buffer0, 5);
	strncat(buffer2, ".", 1);
	if (aux < 10)
		strncat(buffer2, "0", 1);
	strncat(buffer2, buffer1, 2);
	strncat(buffer2, ";\n", 2);

	for (i = 0; i < 1000; i++)
	{
	  if(buffer2[i] == '\0')
		  break;
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)buffer2, i, 500);

}

void UART_transmit_message(char* txBuffer, const char* message)
{
	int i;
	strcpy(aTxBuffer, message);
	for (i = 0; i < 70; i++){
		if (aTxBuffer[i] == '\0')
			break;
	}
    HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, i, 500);
}

void MMC3416_Write_Register(unsigned char* i2cData, unsigned int nRegToWrite)
{

	if (HAL_I2C_Master_Transmit(&hi2c1, (I2C_WRITE)|(MMC3416_SLAVE_ADDR<<1),i2cData,nRegToWrite+1,10) != HAL_OK)
		UART_transmit_message(aTxBuffer, "I2C write\n");

}

void MMC3416_Read_Register(unsigned char* i2cData, unsigned int nRegToRead)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, (I2C_WRITE)|(MMC3416_SLAVE_ADDR<<1),i2cData,1,10) != HAL_OK)
	  UART_transmit_message(aTxBuffer, "I2C read1\n");

	if (HAL_I2C_Master_Receive(&hi2c1, (I2C_READ)|(MMC3416_SLAVE_ADDR<<1),&i2cData[1],nRegToRead,10) != HAL_OK)
	  UART_transmit_message(aTxBuffer, "I2C read2\n");
}

void configureMMC3416()
{
	  // Writing the new configuration

	  i2cData[0] = MMC3416REG_CTRL0;
	  i2cData[1] = 0x00;
	  MMC3416_Write_Register(i2cData,1);

	  i2cData[0] = MMC3416REG_CTRL1; 	// config register address
	  i2cData[1] = 0x00;
	  MMC3416_Write_Register(i2cData,1);

}

void MMC3416verifyConection()
{
	if (HAL_I2C_IsDeviceReady(&hi2c1, (1)|(MMC3416_SLAVE_ADDR<<1),2,10) == HAL_OK)
	{
		UART_transmit_message(aTxBuffer, "\nMMC3416 detected!\n");
	} else
	{
		UART_transmit_message(aTxBuffer, "\nMMC3416 not detected :(\n");
	}
}

void MMC3416_Recap_Operation()
{

	i2cData[0] = MMC3416REG_CTRL0;
	i2cData[1] = 0x80;
	MMC3416_Write_Register(i2cData,1); 		// recap op

	while (1)
	{
		i2cData[0] = MMC3416REG_STATUS;
		MMC3416_Read_Register(i2cData, 1);
		if (!((i2cData[1]>>1)&(1)))
			break;
	}

}

void MMC3416_SET_Operation()
{
	i2cData[0] = MMC3416REG_CTRL0;
	i2cData[1] = 0x20;
	MMC3416_Write_Register(i2cData,1); 		// SET op
}

void MMC3416_RESET_Operation()
{
	i2cData[0] = MMC3416REG_CTRL0;
	i2cData[1] = 0x40;
	MMC3416_Write_Register(i2cData,1); 		// RESET op
}

void MMC3416_Ask_Measurement()
{
	i2cData[0] = MMC3416REG_CTRL0;
	i2cData[1] = 0x01;
	MMC3416_Write_Register(i2cData,1); 		// Ask 1 measure
}

unsigned int MMC3416_MeasurementIsDone(unsigned int nTries)
{
	int aux = 0, index = 0;

	while (aux == 0)
	{
		i2cData[0] = MMC3416REG_STATUS; 	// Verify if the measurement is done
		MMC3416_Read_Register(i2cData,1);
		aux = (1)&(i2cData[1]);
		index++;

		if (index >= nTries)
		{
			return 0;
		}
	}

	return 1;
}

void MMC3416_Measurement_Operation(int* dataVector)
{
	MMC3416_Ask_Measurement();
	while(!MMC3416_MeasurementIsDone(0xFF));

	i2cData[0] = MMC3416REG_X_L; 	// config register address
	MMC3416_Read_Register(i2cData,6);

	dataVector[0] = (int)(i2cData[1] + (i2cData[2]<<8)); 		// H + offset
	dataVector[1] = (int)(i2cData[3] + (i2cData[4]<<8)); 		// H + offset
	dataVector[2] = (int)(i2cData[5] + (i2cData[6]<<8)); 		// H + offset
}

void MMC3416_SET_RESET_offsetCancel_Measurement(float* dataVector)
{
	int bufferSET[3], bufferRESET[3];

	MMC3416_Recap_Operation();
	MMC3416_SET_Operation();
	MMC3416_Measurement_Operation(bufferSET);

	MMC3416_Recap_Operation();
	MMC3416_RESET_Operation();
	MMC3416_Measurement_Operation(bufferRESET);

	dataVector[0] = (float)((bufferSET[0] - bufferRESET[0])/(float)(2*MMC3416_Scale_Adjust_16Bits))*100;
	dataVector[1] = (float)((bufferSET[1] - bufferRESET[1])/(float)(2*MMC3416_Scale_Adjust_16Bits))*100;
	dataVector[2] = (float)((bufferSET[2] - bufferRESET[2])/(float)(2*MMC3416_Scale_Adjust_16Bits))*100;

}
