/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "usart_printf.h"
#include "delay.h"

#include "bmi270_bmm350.h"  // BMI270+BMM350库
#include "bmp5_port.h"      // BMP580库
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
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t tx_buf[TX_BUF_SIZE];
volatile uint8_t tx_busy = 0; // 发送忙标志
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern int IMU_INIT_GETDATA(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	#pragma import(__use_no_semihosting)
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// 启动串口DMA接收
  HAL_UART_Receive_DMA(&huart1, rx_buf, RX_BUF_SIZE);

//  // 启动定时器中断
//  HAL_TIM_Base_Start_IT(&htim1);
  //启动串口中断	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	DWT_Delay_Init(); // 初始化DWT延时
	usart_printf_init(&huart1);
	 
//	  float temperature, pressure;
//	  // 初始化BMP580
//    int8_t bmp_status = BMP580_Init();
//    if (bmp_status != BMP5_OK) {
//        printf("BMP580 initialization failed!\n");
//        // 错误处理
//    }

 printf("========================================\n");
    printf("Dual Sensor Test Program\n");
    printf("I2C1 (PA15/PB7): BMI270 + BMM350\n");
    printf("I2C2 (PC4/PA8): BMP580\n");
    printf("========================================\n\n");
    
    /* 初始化BMI270 */
    printf("Initializing BMI270 (I2C1)...\n");
    int8_t imu_status = IMU_Init(1);
    if (imu_status != BMI2_OK) {
        printf("BMI270 initialization failed!\n");
    } else {
        printf("BMI270 initialization successful!\n");
    }
    
    /* 初始化BMP580 */
    printf("\nInitializing BMP580 (I2C2)...\n");
    int8_t bmp_status = BMP580_Init();
    if (bmp_status != BMP5_OK) {
        printf("BMP580 initialization failed!\n");
    }
    
    printf("\n--- Starting Data Reading ---\n");
    
    /* 数据结构体 */
    imu_data_t imu_data;
    float temperature, pressure;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
//    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
//    delay_ms(50);
//    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//    delay_ms(50);
//		IMU_INIT_GETDATA();
		 // 读取BMP580数据
//        if (BMP580_ReadData(&temperature, &pressure) == BMP5_OK) {
//            printf("BMP580 -> Temp: %.2f C, Press: %.2f hPa\n", temperature, pressure / 100.0f);
//        } else {
//            printf("BMP580 -> Read failed\n");
//        }
		 /* 读取BMI270数据 */
        if (IMU_ReadData(&imu_data) == BMI2_OK) {
            printf("BMI270 -> Acc(m/s²): [%.2f, %.2f, %.2f], Gyr(°/s): [%.2f, %.2f, %.2f], Mag(uT): [%.2f, %.2f, %.2f]\n",
                   imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                   imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z,
                   imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
        } else {
            printf("BMI270 -> Read failed\n");
        }
        
        /* 读取BMP580数据 */
        if (BMP580_ReadData(&temperature, &pressure) == BMP5_OK) {
            printf("BMP580 -> Temp: %.2f C, Press: %.2f hPa\n", temperature, pressure / 100.0f);
        } else {
            printf("BMP580 -> Read failed\n");
        }
        
        printf("----------------------------------------\n");
        
        HAL_Delay(500); // 500ms延时

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UART1_IDLE_Callback(uint16_t len)
{
    if(len > 0)
    {
        HAL_UART_Transmit(&huart1, rx_buf, len, 100);
				memset(rx_buf, 0, RX_BUF_SIZE); // 清空缓存
    }
}

/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        tx_busy = 0;
    }
}
*/
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {		
				
        if(tx_busy == 0)
        {
            tx_busy = 1;
					  strcpy((char*)tx_buf, "Timer DMA Send!\r\n");
            HAL_UART_Transmit_DMA(&huart1, tx_buf, strlen((char*)tx_buf));
        }
    }
}
*/

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
#ifdef USE_FULL_ASSERT
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
