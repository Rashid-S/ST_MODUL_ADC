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
#include "mbcrc.h"
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
extern u8 New_Byte_UART_MCU;
extern u8 UART2_RX_Pos;
extern u8 UART2_RX_BUF[255];

_cmd_simple_answer Packet_cmd_simple_answer;    // Результат выполнения - простой ответ на команду

extern volatile u32 timerchar;

u16 crc = 0;

#pragma pack(1)
typedef struct
{
	u8 addrdev;
	u8 numfunc;
	
}_stranswer;

_stranswer ABC;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void SendData_UART(u8* data, u8 len);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void checkuart(void)
{
	if ((New_Byte_UART_MCU == true) && (timerchar >= 2) ) // Есть новые данные
    {
      New_Byte_UART_MCU = false;
      if (UART2_RX_Pos >= MIN_LEN_FRAME) // Ждем пока не накопится в буфере минимальная длина пакета
      {
        if (UART2_RX_BUF[0] != FRAME_HEADER)  // Если первый байт в буфере не равен 0x02 (начало пакета). Absend 0x02 in head
        {
          u8 i = 0;  // Позиция в буфере начала пакета
          for (;i < UART2_RX_Pos; i++) { if (UART2_RX_BUF[i] == FRAME_HEADER) { break; } } // Ищем заголовок пакета
          if (i == UART2_RX_Pos) { UART2_RX_Pos = 0; } // Не найден заголовок в буфере приёма. Absend 0x02 in buffer
          else // move finding 0x02 data in head
          {
						
            __disable_irq(); // Критическая секция
            UART2_RX_Pos -= i;
            memcpy(UART2_RX_BUF, &UART2_RX_BUF[i], UART2_RX_Pos); // Передвигаем данные в начало буфера
            __enable_irq();
          }
        }
        else
        {
					
          //if (UART2_RX_Pos >= _PACKET_ALL_LEN_2)  // Пакет получен полностью
          {  // Проверяем контрольную сумму. check crc. This is sum value data from pos_data to (tail - 1)
            
						
						crc = 0;
						crc = usMBCRC16((u8*) &UART2_RX_BUF, UART2_RX_Pos-2);
            u8 status = STATUS_CMD_GOOD;
						//for (u8 i = POS_CMD; i < POS_CMD + _PACKET_DATA_LEN_2 - 1; i++) { crc += UART2_RX_BUF[i]; } // Считаем сумму всех данных пакета (кроме заголовка и crc)
            //if (crc != _PACKET_CRC_2) { UART2_RX_BUF[0] = 0x00; } // Ошибочное CRC. Обнуляем заголовок. 
            //else 
             // Найден пакет с данными. Готовим пакет ответа
                                                                   // По умолчанию принимаем значение STATUS_CMD_GOOD
              Packet_cmd_simple_answer.cmd = _PACKET_CMD_2;                                    // Отвечаем на пришедшую команду _PACKET_CMD_2
              Packet_cmd_simple_answer.id =  _PACKET_ID_2;                                     // id event
              Packet_cmd_simple_answer.crc = _PACKET_CMD_2 + _PACKET_ID_2;                     // Заранее считаем crc (пока что без учета других данных)
              //if (_PACKET_ID_2 > EVENT_MAX) { status = STATUS_ERROR_ID; goto label_answer_2; } // Неверный ID
              //if ((Action_Base.event_mask & (1 << _PACKET_ID_2)) != 0) { status = STATUS_ERROR_SLOT_ID_BUSY; goto label_answer_2; } // Этот ID уже занят
              switch(_PACKET_CMD_2) // Пришедшая команда
              {
                /* Включение/отключение модуля ППУ (MODE_ENABLE / MODE_DISABLE) */
                case CMD_10:
								{
									if (UART2_RX_Pos == LEN_CMD_10_WRITE_REQ)
									{
										SendData_UART((u8*) &ABC, sizeof(ABC));
									}
									
									else
									{
										SendData_UART("poshel nahui", sizeof("poshel nahui")); //отправить ошибку
										
									}
									break;
								}
								case CMD_03:
								{
									if (UART2_RX_Pos == MIN_LEN_CMD_03_READ_REQ)
									{}
									else
									{
										// отправить ошибку
									}
									break;
								}
														
							default: { status = STATUS_CMD_UNKNOWN; }
						}
						}
					}
              //if (_PACKET_CMD_2 < MCU_START) { goto label_wo_answer_2; } // Если не надо отправлять ответ
//  //label_answer_2: // Переходим сюда если требуется отправлять ответ Packet_cmd_simple_answer
//              Packet_cmd_simple_answer.status = status;     // Вносим в пакет значение status
//              Packet_cmd_simple_answer.crc +=   status;     // Корректируем crc
//              SEND_PACKET_UART2(Packet_cmd_simple_answer);  // Отправка ответа
//  //label_wo_answer_2: // Переходим сюда если не требуется отправлять ответ Packet_cmd_simple_answer
//              // Удаляем обработанный пакет из буфера приёма UART2
//              NVIC_DisableIRQ(UART2_IRQn); // Критическая секция
//              {
//                UART2_RX_Pos -= (_PACKET_ALL_LEN_2);
//                if (UART2_RX_Pos!=0)
//                {
//                  memcpy(UART2_RX_BUF, &UART2_RX_BUF[_PACKET_ALL_LEN_2], UART2_RX_Pos); // Переносим остаток данных в буфере
//                }
//              }
//              NVIC_EnableIRQ(UART2_IRQn);
            //}
		}
	}
}

void SendData_UART(u8* data, u8 len)
{
  for (u8 i=0; i < len; i++)
  {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = data[i];
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

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
	
	LL_USART_EnableIT_RXNE(USART2);
	SysTick_Config(80000);
	LL_SYSTICK_EnableIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	memset (&ABC, 0, sizeof(ABC));
	ABC.addrdev = 0x02;
	ABC.numfunc = 0x10;
  while (1)
  {
    /* USER CODE END WHILE */
		checkuart();
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnablePLLMode();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(80000000);

  LL_SetSystemCoreClock(80000000);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
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

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

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
