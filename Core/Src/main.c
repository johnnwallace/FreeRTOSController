/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "bno055_stm32.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for readIMU */
osThreadId_t readIMUHandle;
const osThreadAttr_t readIMU_attributes = {
  .name = "readIMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for evaluatePID */
osThreadId_t evaluatePIDHandle;
const osThreadAttr_t evaluatePID_attributes = {
  .name = "evaluatePID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for globalPos */
osMutexId_t globalPosHandle;
const osMutexAttr_t globalPos_attributes = {
  .name = "globalPos"
};
/* Definitions for motorCommand */
osMutexId_t motorCommandHandle;
const osMutexAttr_t motorCommand_attributes = {
  .name = "motorCommand"
};
/* Definitions for newOrientation */
osEventFlagsId_t newOrientationHandle;
const osEventFlagsAttr_t newOrientation_attributes = {
  .name = "newOrientation"
};
/* USER CODE BEGIN PV */
typedef struct {
  bno055_vector_t vec;
  uint32_t t;
} tVec_t;

const bno055_vector_t vPosSetpoint = {0.0, 0.0, 0.0, 0.0};
const double daGains[] = {1.0, 0.1, 0.1};

tVec_t tvPos;
tVec_t tvRate;
tVec_t tvAccel;

tVec_t tvMotorCommand;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void ReadIMU(void *argument);
void EvaluatePID(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void tvec_toString(char* out, tVec_t tv) {
	int xInt1 = floor(tv.vec.x);
	int xInt2 = trunc((tv.vec.x - xInt1) * 100);

	int yInt1 = floor(tv.vec.y);
	int yInt2 = trunc((tv.vec.y - yInt1) * 100);

	int zInt1 = floor(tv.vec.z);
	int zInt2 = trunc((tv.vec.z - zInt1) * 100);

	sprintf(out, "%lu: Yaw: %d.%02d, Roll: %d.%02d, Pitch: %d.%02d\r\n",
            tv.t, xInt1, xInt2, yInt1, yInt2, zInt1, zInt2);
}

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

double dSign(double x)
{
	if (x > 0) return 1.0;
	if (x < 0) return -1.0;
	return 0.0;
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of globalPos */
  globalPosHandle = osMutexNew(&globalPos_attributes);

  /* creation of motorCommand */
  motorCommandHandle = osMutexNew(&motorCommand_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readIMU */
  readIMUHandle = osThreadNew(ReadIMU, NULL, &readIMU_attributes);

  /* creation of evaluatePID */
  evaluatePIDHandle = osThreadNew(EvaluatePID, NULL, &evaluatePID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of newOrientation */
  newOrientationHandle = osEventFlagsNew(&newOrientation_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  char orientation[100];
  char motorCommand[100];
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    osMutexAcquire(globalPosHandle, 200);
    tvec_toString(orientation, tvPos);
    osMutexRelease(globalPosHandle);

    osMutexAcquire(motorCommandHandle, 200);
    tvec_toString(motorCommand, tvMotorCommand);
    osMutexRelease(motorCommandHandle);
    
    HAL_UART_Transmit(&huart2, (uint8_t*)motorCommand, sizeof(uint8_t)*strlen(motorCommand), HAL_MAX_DELAY);
    
    osDelay(200);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReadIMU */
/**
* @brief Function implementing the readIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadIMU */
void ReadIMU(void *argument)
{
  /* USER CODE BEGIN ReadIMU */

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(globalPosHandle, 0);
    tvPos.vec = bno055_getVectorEuler();

    /* set yaw to between -180 and 180 */
    if (tvPos.vec.x > 180) tvPos.vec.x = tvPos.vec.x - 360;

    tvPos.t = HAL_GetTick();
    osMutexRelease(globalPosHandle);

    osEventFlagsSet(newOrientationHandle, 1);

    osDelay(20);
  }
  /* USER CODE END ReadIMU */
}

/* USER CODE BEGIN Header_EvaluatePID */
/**
* @brief Function implementing the evaluatePID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EvaluatePID */
void EvaluatePID(void *argument)
{
  /* USER CODE BEGIN EvaluatePID */
  bno055_vector_t vCumulative = {0.0, 0.0, 0.0, 0.0};
  bno055_vector_t vDerivative = {0.0, 0.0, 0.0, 0.0};

  tVec_t tvError, tvLastError;
  bno055_vector_t vError, vLastError;

  double dt;

  /* Infinite loop */
  for(;;)
  {
    /* wait for a new orientation reading*/
    osEventFlagsWait(newOrientationHandle, 1, osFlagsWaitAll, osWaitForever);
    osEventFlagsClear(newOrientationHandle, 1);

    tvLastError = tvError;

    /* calculate error vector from current orientation */
    osMutexAcquire(globalPosHandle, 0);
    tvError.t = tvPos.t;
    tvError.vec.x = vPosSetpoint.x - tvPos.vec.x;

    tvError.vec.y = vPosSetpoint.y - tvPos.vec.y;
    tvError.vec.z = vPosSetpoint.z - tvPos.vec.z;
    osMutexRelease(globalPosHandle);

    vError = tvError.vec;
    vLastError = tvLastError.vec;
    dt = (tvError.t - tvLastError.t) / 1000.0;

    /* set cumulative error vector */
    vCumulative.x +=  vError.x;
    vCumulative.y +=  vError.y;
    vCumulative.z +=  vError.z;

    /* set derivative error vector */
    vDerivative.x = dSign(fmod(fabs(vError.x - vLastError.x),180)) / dt;
    vDerivative.y = dSign(fmod(fabs(vError.y - vLastError.y),180)) / dt;
    vDerivative.z = dSign(fmod(fabs(vError.z - vLastError.z),180)) / dt;

    /* set desired rate vector */
    osMutexAcquire(motorCommandHandle, 0);

    tvMotorCommand.t = HAL_GetTick() - tvError.t;

    tvMotorCommand.vec.x = daGains[0] * vError.x
                         + daGains[1] * vCumulative.x
                         + daGains[2] * vDerivative.x;

    tvMotorCommand.vec.y = daGains[0] * vError.y
                         + daGains[1] * vCumulative.y
                         + daGains[2] * vDerivative.y;

    tvMotorCommand.vec.z = daGains[0] * vError.z
                         + daGains[1] * vCumulative.z
                         + daGains[2] * vDerivative.z;

    osMutexRelease(motorCommandHandle);

    osDelay(20);
  }
  /* USER CODE END EvaluatePID */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
