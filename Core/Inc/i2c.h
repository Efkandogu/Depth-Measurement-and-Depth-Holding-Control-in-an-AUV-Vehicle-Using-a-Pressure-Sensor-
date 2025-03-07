/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define MS5837_ADDR              0x76<<1
#define MS5837_RESET             0x1E
#define MS5837_ADC_READ          0x00
#define MS5837_PROM_READ         0xA0
#define MS5837_CONVERT_D1_8192   0x4A
#define MS5837_CONVERT_D2_8192   0x5A

#define MS5837_30BA 			 2
#define MS5837_02BA 			 0

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */




enum {
	FALSE,
	TRUE
};



uint8_t MS5837_init(I2C_HandleTypeDef *hi2c);

void MS5837_setModel(uint8_t model);

void MS5837_setFluidDensity(float density);

uint8_t MS5837_read(I2C_HandleTypeDef *hi2c);

float MS5837_pressure(float conversion);

float MS5837_temperature();

float MS5837_depth();

float MS5837_altitude();

void MS5837_calculate();

uint8_t MS5837_crc4(uint16_t n_prom[]);

void read_depth();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

