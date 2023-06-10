/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stdio.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//#define RxBuff_SIZE 256//?????? 
//extern uint8_t RxBuff[RxBuff_SIZE];  
//extern uint16_t RxBuff[64];   
#define codebook_len 500 //Âë±í³¤¶È
extern volatile uint8_t RxLength;   
extern volatile uint8_t RxEndFlag;
extern uint8_t chip;
extern uint8_t channel;
extern uint8_t dacValue_array[64];
extern uint8_t codebook[codebook_len][64];
extern volatile uint8_t start_flag;
//const char ADCMode1[8]="mode1#";//?????????	
//const char ADCMode2[8]="mode2#";//?????????	
//const char ADCStop[8]="stop#";//?????????	


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TLV_FS_Pin GPIO_PIN_5
#define TLV_FS_GPIO_Port GPIOC
#define TLV_SCK_Pin GPIO_PIN_6
#define TLV_SCK_GPIO_Port GPIOC
#define TLV_DATA_Pin GPIO_PIN_8
#define TLV_DATA_GPIO_Port GPIOC
#define TLV_FRE_Pin GPIO_PIN_9
#define TLV_FRE_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TLV_LDAC_Pin GPIO_PIN_8
#define TLV_LDAC_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
