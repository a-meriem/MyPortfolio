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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ros.h"
#include "Ros_communication.h"
#include "common.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint8_t read_counter = 0 ;
double Nav_CurrentX_mm_d = 0 ;
double Nav_CurrentY_mm_d = 0 ;
double Nav_CurrentAngle_deg_d = 0 ;
extern float Nav_Orders_fromROS_arr [20];
float ROBOT_CurrentXYTheta_fromCM7_arr [4] = { 0.0 , 0.0 , 0.0 , 0.0 };
float ROBOT_INformationsRecievedFromCM7 [20] = {0.0} ;
extern float ROBOT_CurrentStatuts_i8 [1] ;
void * addr;
size_t len;
void * addr_1;
size_t len_1;



extern uint8_t Nav_FirstTimeRecievingTargets;

/* Ringbuffer variables */
volatile ringbuff_t* rb_cm4_to_cm7 = (ringbuff_t *) BUFF_CM4_TO_CM7_ADDR;
volatile ringbuff_t* rb_cm7_to_cm4 = (ringbuff_t *) BUFF_CM7_TO_CM4_ADDR;

/* ROS Variables */
extern ROS_Node_States_ten ROS_Node_CurrentState_en ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);
void vXYthetaGetter(void);
static void led_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if ( htim == &htim17)
	{

		switch (ROS_Node_CurrentState_en )
		{
		case ROS_Node_idle_en :
		{
		 //HAL_GPIO_WritePin(LD3_GPIO_PORT, LD3_GPIO_PIN,GPIO_PIN_SET);

		HAL_GPIO_TogglePin(LD3_GPIO_PORT, LD3_GPIO_PIN);

		ROS_SpinOnce();
		}break ;

		case ROS_Node_Communicating :
		{

			//HAL_GPIO_TogglePin(LD3_GPIO_PORT, LD3_GPIO_PIN);

			// Receive Current State , X , Y and Theta from CM7
			if ( (len = ringbuff_get_full(rb_cm7_to_cm4)) > 0 )
			{
				ringbuff_read(rb_cm7_to_cm4, ROBOT_INformationsRecievedFromCM7, len);

				read_counter = 0 ;

				while (read_counter < (len/4 - 1) )
				{
					if (ROBOT_INformationsRecievedFromCM7[read_counter] == 100)
					{
						ROBOT_CurrentXYTheta_fromCM7_arr[0] = ROBOT_INformationsRecievedFromCM7[read_counter + 1];
						ROBOT_CurrentXYTheta_fromCM7_arr[1] = ROBOT_INformationsRecievedFromCM7[read_counter + 2];
						ROBOT_CurrentXYTheta_fromCM7_arr[2] = ROBOT_INformationsRecievedFromCM7[read_counter + 3];
						ROBOT_CurrentXYTheta_fromCM7_arr[3] = ROBOT_INformationsRecievedFromCM7[read_counter + 4];
						read_counter += 4 ;
						vNav_CurrentXYTheta_Publisher();
					}
					else
					{
						ROBOT_CurrentStatuts_i8[0] = ROBOT_INformationsRecievedFromCM7[read_counter + 1];
						read_counter += 1 ;
						vNav_CurrentStatus_Publisher();
					}
				}
			}




		}break ;
		}

		// Transmission of Orders to CM7 will be placed in the subscriber callback ( See Ros_Communication.cpp )


	}
	ROS_SpinOnce();

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t i = 0, time, t1, t2;
	/* CPU2 goes to STOP mode and waits CPU1 to initialize all the steps first */
	/* CPU1 will wakeup CPU2 with semaphore take and release events */
	/* HW semaphore Clock enable */
	__HAL_RCC_HSEM_CLK_ENABLE();
	HAL_HSEM_ActivateNotification(HSEM_WAKEUP_CPU2_MASK);
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
	__HAL_HSEM_CLEAR_FLAG(HSEM_WAKEUP_CPU2_MASK);


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

	led_init();
	vROS_Setup();
	HAL_TIM_Base_Start_IT(&htim17);

  /* USER CODE END 2 */

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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void
led_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	LD3_GPIO_CLK_EN();

	GPIO_InitStruct.Pin = LD3_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_PORT, &GPIO_InitStruct);
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
