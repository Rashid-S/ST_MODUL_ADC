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
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include <stdbool.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define i8 int8_t
#define i16 int16_t
#define i32 int32_t
#define MIN_LEN_FRAME 5
#define FRAME_HEADER 0x02  // адрес устройства
#define POS_CMD 0x01       // номер позиции



#define STATUS_CMD_GOOD            0    // Команда принята к исполнению
#define STATUS_BUSY                 1    // Команда не будет выполнена - занято
#define STATUS_CMD_UNKNOWN          2    // Неизвестная команда
#define STATUS_DATA_LEN_ERROR      3    // Ошибка в данных (переполнение или отсутствие данных)
#define STATUS_FMT_CMD_ERROR        4    // Ошибка в формате команды
#define STATUS_PARAM_VAL_ERROR      5    // Ошибка в значении параметра
#define STATUS_TIMEOUT             6    // Ошибка Ожидания ответа
#define STATUS_ERROR_ID            7    // Ошибка ID не существует
#define STATUS_ERROR_SLOT_ID_BUSY  8    // Ошибка ID занят
#define STATUS_ERROR_FLASH_DATA    9    // Ошибка Данные во флэш памяти записаны с ошибкой или отсутствуют

#define _PACKET_CMD_2            UART2_RX_BUF[POS_CMD]
#define _PACKET_ID_2             UART2_RX_BUF[POS_CMD]
#define CMD_10 0x10
#define CMD_03 0x03



#define LEN_CMD_SIMPLE_ANSWER (sizeof(_cmd_simple_answer) - LEN_CMD_CRC)
typedef struct
{
  u8 head;           // 0xAA
  u8 len_cmd;        // = 4
  u8 cmd;            // CMD
  u8 id;             // id event
  u8 status;         // Ответ на команду
  u8 crc;            // crc = cmd + result
  } _cmd_simple_answer;


#define SEND_PACKET_UART2(NAME) SendData_UART(MDR_UART2, (u8*)&NAME, sizeof(NAME))
	
#define	LEN_CMD_10_WRITE_REQ 11
#define	MIN_LEN_CMD_03_READ_REQ 8



	
	
	

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
