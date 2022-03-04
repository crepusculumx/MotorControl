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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

#include "PwmDriver.h"
#include "GpioDriver.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "ComProtocol.h"
#include "As5048aDriver.h"
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

PwmDriver pwmDriver;
GpioDriver gpioDriverEn;
GpioDriver gpioDriverDir;
GpioDriver gpioDriverSpiCs;

MotorDriver motorDriver;
As5048aDriver as5048aDriver;

MotorController motorController;

ComProtocolParser comProtocolParser;

// 中断 flag
bool motor_control_flag = false;
bool tx_send_flag = false;
bool interrupt_flag_10Hz = false;

uint8_t rx_buffer[1];
uint8_t spi_buffer[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void init_all(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_all(void) {
  PwmDriver_init(&pwmDriver, 64, &htim1, TIM_CHANNEL_1, 64, 65535);
  GpioDriver_init(&gpioDriverDir, MOTOR_0_DIR_GPIO_Port, MOTOR_0_DIR_Pin);
  GpioDriver_init(&gpioDriverEn, MOTOR_0_EN_GPIO_Port, MOTOR_0_EN_Pin);
  GpioDriver_init(&gpioDriverSpiCs, SPI_CS_GPIO_Port, SPI_CS_Pin);

  MotorDriver_init(&motorDriver, &pwmDriver, &gpioDriverEn, &gpioDriverDir);

  As5048aDriver_init(&as5048aDriver, &hspi1, gpioDriverSpiCs);

  MotorController_init(&motorController, &motorDriver, 10, 2);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //开启usart1中断
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

//  HAL_SPI_Receive_IT(&hspi1, spi_buffer, 1);
  //开启定时器2中断
  HAL_TIM_Base_Start_IT(&htim2);
  init_all();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 通信Rx
    if (ComProtocolParser_have_receive(&comProtocolParser)) {
      ComProtocolParser_parse(&comProtocolParser);
      if (comProtocolParser.motor_cmd_ready_flag == true) {
        comProtocolParser.motor_cmd_ready_flag = false;
        MotorCmdMsg motor_cmd_msg = comProtocolParser.motor_cmd_msg;
        MotorCmd motor_cmd;
        motor_cmd.mode = motor_cmd_msg.mode;
        motor_cmd.value = (double) motor_cmd_msg.value / MOTOR_CMD_MSG_FLOAT_TO_INT;
        MotorController_set_cmd(&motorController, motor_cmd);
      }
    }

    if (interrupt_flag_10Hz) {
      interrupt_flag_10Hz = false;

      double pos = As5048aDriver_read_data(&as5048aDriver);
      MotorController_set_position(&motorController, pos);
    }

    // 周期发送
    if (tx_send_flag == true) {
      tx_send_flag = false;

      //printf("cur_pos:%lf \n", As5048aDriver_read_data(&as5048aDriver));
//      uint8_t buffer[MAX_PACKET_LENGTH + 7];
//      size_t it = 0;
//      protocol_dump(0, motorController.cur_motor_state, buffer, MAX_PACKET_LENGTH + 7, &it);
////      HAL_UART_Transmit_IT(&huart1, buffer, it);
//
//      printf("%lf \n", As5048aDriver_read_data(&as5048aDriver));
    }


    // 周期控制
    if (motor_control_flag == true) {
      motor_control_flag = false;
      MotorController_control(&motorController);
    }
  }
#pragma clang diagnostic pop
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief timer中断回调
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // 10Hz
  if (htim == (&htim2)) {
    interrupt_flag_10Hz = true;
    motor_control_flag = true;
    tx_send_flag = true;
  }
}

/**
 * @brief uart Rx中断回调
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    ComProtocolParser_input(&comProtocolParser, rx_buffer[0]);
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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

