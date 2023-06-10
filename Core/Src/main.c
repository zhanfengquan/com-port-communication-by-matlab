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
#include "TLV56XX.h"
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
 UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//串口2接收	
#define Uart2RxBuffSIZE  32003     //最大接收字节数，500*64+3	

uint8_t Uart2RxBuffer[Uart2RxBuffSIZE]= {0};   //接收数据数组	
uint8_t aUart2RxBuff;			//接收中断缓冲	
uint16_t Uart2RxCnt = 0;//接收数据计数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t chip = 0;
uint8_t channel = 0;
uint16_t dacValue = 0;
//uint8_t RxBuff[RxBuff_SIZE];  //??????
//uint8_t RxBuff[64];
volatile uint8_t start_flag = 0;//需要用volatile，否则不会触发


volatile uint8_t RxLength = 0;  //????????
volatile uint8_t RxEndFlag = 0; //??????????
uint16_t count_loop;
/* USER CODE END 0 */
//初始化电压值
uint8_t dacValue_array[64]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//码表
uint8_t codebook[codebook_len][64];
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DAC_TLV5610_Init();
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aUart2RxBuff, 1);   //开启接收中断  
  /* USER CODE END 2 */
	for (chip = 1; chip <= 8; chip++)//chip从1到8
	{
			for (channel = 0; channel <= 7; channel++)//channel从0到7
			{
					TLV5610_WriteChannelValue(chip, channel, dacValue_array[(chip-1)*8+channel]);
			}
			
	}
	
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	start_flag = 0;
  while (1)
  {
		
    /* USER CODE END WHILE */
		if(start_flag == 1)
		{
			for (count_loop=0;count_loop<codebook_len;count_loop++)
			{
					for (chip = 1; chip <= 8; chip++)//chip从1到8
					{
							for (channel = 0; channel <= 7; channel++)//channel从0到7
							{
									TLV5610_WriteChannelValue(chip, channel, codebook[count_loop][(chip-1)*8+channel]);
							}
							
					}
//			HAL_Delay(1);
			}

//			HAL_UART_Transmit(&huart2, (uint8_t *)&start_flag, 1,0xFFFF);

			start_flag = 0;
		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TLV_FS_Pin|TLV_SCK_Pin|TLV_DATA_Pin|TLV_FRE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TLV_LDAC_GPIO_Port, TLV_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TLV_FS_Pin TLV_SCK_Pin TLV_DATA_Pin TLV_FRE_Pin */
  GPIO_InitStruct.Pin = TLV_FS_Pin|TLV_SCK_Pin|TLV_DATA_Pin|TLV_FRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TLV_LDAC_Pin */
  GPIO_InitStruct.Pin = TLV_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TLV_LDAC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){  
	UNUSED(huart); 	
//	uint8_t flag = 1;

	 Uart2RxBuffer[Uart2RxCnt++] = aUart2RxBuff;   //接收数据转存			
	 if((Uart2RxBuffer[Uart2RxCnt-1] == 0x0A)&&(Uart2RxBuffer[Uart2RxCnt-2] == 0x0D)&&(Uart2RxBuffer[Uart2RxCnt-3] == '#')) //更新码表			
	{	
		for (count_loop=0;count_loop<codebook_len;count_loop++)
			{
					for (chip = 1; chip <= 8; chip++)//chip从1到8
					{
							for (channel = 0; channel <= 7; channel++)//channel从0到7
							{
									codebook[count_loop][(chip-1)*8+channel] = Uart2RxBuffer[count_loop*64+(chip-1)*8+channel];
							}
							
					}
			}
//		HAL_UART_Transmit(&huart2, (uint8_t *)&Uart2RxBuffer, Uart2RxCnt,0xFFFF); //将收到的信息发送出去			
//			printf("hello\r\n");            
		while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束			
		Uart2RxCnt = 0;			
		memset(Uart2RxBuffer,0x00,sizeof(Uart2RxBuffer)); //清空数组		
		
	}	
	if((Uart2RxBuffer[Uart2RxCnt-1] == 0x0A)&&(Uart2RxBuffer[Uart2RxCnt-2] == 0x0D)&&(Uart2RxBuffer[Uart2RxCnt-3] == '@'))		//按码表进行相位切换
	{	
		start_flag = 1;
//		HAL_UART_Transmit(&huart2, (uint8_t *)&start_flag, 1,0xFFFF);
		while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
		Uart2RxCnt = 0;	
		memset(Uart2RxBuffer,0x00,sizeof(Uart2RxBuffer)); //清空数组
//		HAL_UART_Transmit(&huart2, (uint8_t *)&Uart2RxBuffer, sizeof(Uart2RxBuffer),0xFFFF);
	}	
	
HAL_UART_Receive_IT(&huart2, (uint8_t *)&aUart2RxBuff, 1);   //再开启接收中断}
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
