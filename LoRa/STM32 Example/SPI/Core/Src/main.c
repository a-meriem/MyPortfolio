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
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lora.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	Lora_Idle_en,
	Lora_Tx_en,
	Lora_TxtoRx_en,
	Lora_Rx_en,
	Lora_RxtoTx_en
}Lora_Mode_ten;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
uint8_t Lora_status_u8 = 0 ;
uint8_t Lora_TxDoneHw_u8 = 0 ;
uint8_t Lora_TxDoneSw_u8 = 0 ;
uint8_t Lora_RxDoneHw_u8 = 0 ;
uint8_t Lora_RxDoneSw_u8 = 0 ;
*/
uint8_t Lora_payloadLength_u8 = 0;

uint8_t Lora_RxBuffer_au8[5] = {0.0} ;
uint8_t Lora_TxBuffer_au8[5]={0.0};

Lora_RxStates_ten Lora_RxCurrentState_en = Lora_RxIdle_en;
Lora_TxStates_ten Lora_TxCurrentState_en = Lora_TxIdle_en;
Lora_Mode_ten Lora_CurrentMode_en = Lora_Idle_en;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Lora_vInit();
  Lora_payloadLength_u8 = 2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Lora_TxBuffer_au8[0]=0x66;
  Lora_TxBuffer_au8[1]=0x01;
  Lora_RxBuffer_au8[0]=0;
  Lora_RxBuffer_au8[1]=0;
  Lora_vConfig();
  while (1)
  {
	  switch(Lora_CurrentMode_en){
	  case Lora_Idle_en :{
		  Lora_CurrentMode_en = Lora_RxtoTx_en;
	  }break;
	  case Lora_Tx_en:{
		  Lora_TxCurrentState_en = Lora_tenTxMode(Lora_TxBuffer_au8 , Lora_payloadLength_u8);
		  if (Lora_TxReady_en ==Lora_TxCurrentState_en ){
			  Lora_CurrentMode_en = Lora_TxtoRx_en ;
		  }
	  }break;
	  case Lora_TxtoRx_en:{
		  //HAL_Delay(1000);
		  Lora_RxBuffer_au8[0] = 0;
		  Lora_RxBuffer_au8[1] = 0;
		  Lora_RxCurrentState_en =Lora_RxConfig_en;
		  Lora_CurrentMode_en = Lora_Rx_en ;
	  }break;
	  case Lora_Rx_en:{
		  Lora_RxCurrentState_en = Lora_tenRxMode(Lora_RxBuffer_au8, Lora_payloadLength_u8);
		  if ( (Lora_RxReady_en ==Lora_RxCurrentState_en) /*||(Lora_RxTimeout_en ==Lora_RxCurrentState_en)*/ ){
			  Lora_CurrentMode_en = Lora_RxtoTx_en ;
		  }
	  }break;
	  case Lora_RxtoTx_en:{
		  HAL_Delay(1000);
		  Lora_TxBuffer_au8[1] =  Lora_RxBuffer_au8[1] +1;
		  Lora_TxCurrentState_en =Lora_TxConfig_en;
		  Lora_CurrentMode_en = Lora_Tx_en ;
		  }break;
	  }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback( SPI_HandleTypeDef *hspi )
{
	__HAL_DMA_DISABLE_IT ( &hdma_spi1_tx, DMA_IT_HT );
	Lora_Transmit_Complete = 1;
}

void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi )
{
	__HAL_DMA_DISABLE_IT ( &hdma_spi1_rx, DMA_IT_HT );
	Lora_Receive_Complete = 1;
}
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

