#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#define MOTOR_CHANNEL 0
#define PWM_MIN 4000
#define PWM_MAX 8000

float target_depth = 66.0;

void SystemClock_Config(void);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2)
    {
//        char buffer[50];
  //      HAL_UART_Receive_IT(&huart2, (uint8_t *)buffer, sizeof(buffer));
    //    target_depth = atof(buffer);

    }
}


int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  esc_initialize(&huart1, MOTOR_CHANNEL);

  if (!MS5837_init(&hi2c1)) {
         Error_Handler();
     }
     MS5837_setModel(MS5837_30BA);
  //char buffer[50];
  //HAL_UART_Receive_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
  while (1)
  {
	  float current_depth = MS5837_depth();

	  float depth_error = target_depth - current_depth;

	  if (depth_error > 0.1) {
		  uint16_t pwm_value = PWM_MIN + (uint16_t)(depth_error * 200);
		  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
		  pololu_set_target(&huart1, MOTOR_CHANNEL, pwm_value);
	  } else if (depth_error < -0.1) {
		  uint16_t pwm_value = PWM_MIN + (uint16_t)(-depth_error * 200);
		  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
		  pololu_set_target(&huart1, MOTOR_CHANNEL, pwm_value);
	  } else {
		  pololu_set_target(&huart1, MOTOR_CHANNEL, PWM_MIN);
	  }

	 // sprintf(buffer, "Current Depth: %.2f m\n", current_depth);
	  //HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

/* USER CODE BEGIN 4 */

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
