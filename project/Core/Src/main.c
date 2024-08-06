/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint8_t Iz_pressed = 0;// Variable para rastrear si el botón izquierdo fue presionado
uint8_t De_pressed = 0;// Variable para rastrear si el botón derecho fue presionado
uint32_t last_Iz_press = 0;  // Tiempo de la última presión del botón izquierdo
uint32_t last_De_press = 0;  // Tiempo de la última presión del botón derecho
uint8_t Iz_blink = 0;  // Flag para indicar si el LED izquierdo debe parpadear
uint8_t De_blink = 0;  // Flag para indicar si el LED derecho debe parpadear
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
 void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();  // Obtiene el tiempo actual en milisegundos

    if(GPIO_Pin == Giro_Iz_Pin){  // Si se presionó el botón izquierdo
        if(current_time - last_Iz_press < 500){  // Si han pasado menos de 500ms desde la última presión
            Iz_pressed++;  // Incrementa el contador de presiones
            if(Iz_pressed >= 2){  // Si se ha presionado 2 o más veces
                Iz_blink = 1;  // Activa el parpadeo del LED izquierdo
                De_blink = 0;  // Desactiva el parpadeo del LED derecho
            }
        } else {
            Iz_pressed = 1;  // Reinicia el contador si han pasado más de 500ms
        }
        last_Iz_press = current_time;  // Actualiza el tiempo de la última presión

        // Si el LED derecho está encendido, lo apaga
        if(HAL_GPIO_ReadPin(Luz_De_GPIO_Port, Luz_De_Pin) == GPIO_PIN_SET){
            HAL_GPIO_WritePin(Luz_De_GPIO_Port, Luz_De_Pin, GPIO_PIN_RESET);
            De_blink = 0;
        }
    }

    if(GPIO_Pin == Giro_De_Pin){  // Si se presionó el botón derecho
        // El código es similar al del botón izquierdo, pero para el lado derecho
        if(current_time - last_De_press < 500){
        	De_pressed++;
            if(De_pressed >= 2){
            	De_blink = 1;
                Iz_blink = 0;
            }
        } else {
        	De_pressed = 1;
        }
        last_De_press = current_time;

        if(HAL_GPIO_ReadPin(Luz_Iz_GPIO_Port, Luz_Iz_Pin) == GPIO_PIN_SET){
            HAL_GPIO_WritePin(Luz_Iz_GPIO_Port, Luz_Iz_Pin, GPIO_PIN_RESET);
            Iz_blink = 0;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 HAL_Init();  // Inicializa la capa HAL (Hardware Abstraction Layer)
	  SystemClock_Config();  // Configura el reloj del sistema
	  MX_GPIO_Init();  // Inicializa los pines GPIO

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
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (Iz_blink) {  // Si el LED izquierdo debe parpadear
	  	        HAL_GPIO_TogglePin(Luz_Iz_GPIO_Port, Luz_Iz_Pin);  // Cambia el estado del LED izquierdo
	  	        HAL_Delay(500);  // Espera 500ms
	  	    }

	  	    if (De_blink) {  // Si el LED derecho debe parpadear
	  	        HAL_GPIO_TogglePin(Luz_De_GPIO_Port, Luz_De_Pin);  // Cambia el estado del LED derecho
	  	        HAL_Delay(500);  // Espera 500ms
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Luz_De_GPIO_Port, Luz_De_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Luz_Iz_GPIO_Port, Luz_Iz_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Giro_Iz_Pin Giro_De_Pin */
  GPIO_InitStruct.Pin = Giro_Iz_Pin|Giro_De_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Luz_De_Pin */
  GPIO_InitStruct.Pin = Luz_De_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Luz_De_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Luz_Iz_Pin */
  GPIO_InitStruct.Pin = Luz_Iz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Luz_Iz_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE END Error_Handler_Debug */
	}
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
