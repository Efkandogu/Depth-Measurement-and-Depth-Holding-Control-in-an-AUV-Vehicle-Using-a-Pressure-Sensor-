/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <math.h>
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;


const float MS5837_Pa = 100.0f;
const float MS5837_bar = 0.001f;
const float MS5837_mbar = 1.0f;



uint16_t C[8];
uint32_t D1, D2;
int32_t TEMP;
int32_t P;
uint8_t _model;

float fluidDensity = 1029;

void MX_I2C1_Init(void)
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

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t MS5837_init(I2C_HandleTypeDef *hi2c) {

	HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, MS5837_RESET, 1, 500);

	HAL_Delay(10);

	uint8_t temp;

	for ( uint8_t i = 0 ; i < 8 ; i++ )
	{
		temp = MS5837_PROM_READ+i*2;
		HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, &temp, 1, 500);

		uint8_t ret[2];
		HAL_I2C_Master_Receive(hi2c, MS5837_ADDR, ret, 2, 500);
		C[i]=(ret[0]<<8)|ret[1];

	}

	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = MS5837_crc4(C);

	if ( crcCalculated == crcRead ) {
		return TRUE;
	}

	return FALSE;
}

void MS5837_setModel(uint8_t model) {
	_model = model;
}

void MS5837_setFluidDensity(float density) {
	fluidDensity = density;
}

uint8_t MS5837_read(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[3];
	if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, MS5837_CONVERT_D1_8192, 1, HAL_MAX_DELAY) != HAL_OK)
	        return FALSE;

	HAL_Delay(20);

	if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, MS5837_ADC_READ, 1, HAL_MAX_DELAY) != HAL_OK)
	        return FALSE;

	if (HAL_I2C_Master_Receive(hi2c, MS5837_ADDR, data, 3, HAL_MAX_DELAY) != HAL_OK)
	        return FALSE;

	D1=(data[0]<<16)|(data[1]<<8)|(data[2]);

	if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, MS5837_CONVERT_D2_8192, 1, HAL_MAX_DELAY) != HAL_OK)
	        return FALSE;

	HAL_Delay(20);

	if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, MS5837_ADC_READ, 1, HAL_MAX_DELAY) != HAL_OK)
	        return 0;

	if (HAL_I2C_Master_Receive(hi2c, MS5837_ADDR, data, 3, HAL_MAX_DELAY) != HAL_OK)
	        return 0;

	D2=(data[0]<<16)|(data[1]<<8)|(data[2]);


	MS5837_calculate();

	return TRUE;
}

void MS5837_calculate()
{

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	dT = D2-(uint32_t)(C[5])*256l;
	if ( _model == MS5837_02BA )
	{
		SENS = (int64_t)(C[1])*65536l+((int64_t)(C[3])*dT)/128l;
		OFF = (int64_t)(C[2])*131072l+((int64_t)(C[4])*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	}
	else
	{
		SENS = (int64_t)(C[1])*32768l+((int64_t)(C[3])*dT)/256l;
		OFF = (int64_t)(C[2])*65536l+((int64_t)(C[4])*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}

	TEMP = 2000l+(int64_t)(dT)*C[6]/8388608LL;

	if ( _model == MS5837_02BA )
	{
		if((TEMP/100)<20)
		{
			Ti = (11*(int64_t)(dT)*(int64_t)(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	}
	else
	{
		if((TEMP/100)<20)
		{
			Ti = (3*(int64_t)(dT)*(int64_t)(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15)
			{
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20)
		{
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);

	if ( _model == MS5837_02BA )
	{
		P = (((D1*SENS2)/2097152l-OFF2)/32768l);
	}
	else
	{
		P = (((D1*SENS2)/2097152l-OFF2)/8192l);
	}
}

float MS5837_pressure(float conversion) {
    if ( _model == MS5837_02BA ) {
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

float MS5837_temperature() {
	return TEMP/100.0f;
}

float MS5837_depth() {
	return ( MS5837_pressure(MS5837_Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837_altitude() {
	return (1-pow(( MS5837_pressure(MS5837_mbar)/1013.25),.190284))*145366.45*.3048;
}

uint8_t MS5837_crc4(uint16_t *n_prom) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}

void read_depth()
{
//	char buffer[50];
//	if (MS5837_read(&hi2c1))
//	{
//
//		float depth = MS5837_depth();
//
//		sprintf(buffer, "Depth: %.2f meters\n", depth);
//
//		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//	}
//
//	else
//	{
//
//		char error_message[] = "Failed to read from MS5837!\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
//	}
}
/* USER CODE END 1 */
