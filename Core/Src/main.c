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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  bsIdle, bsWaitCommand2
} bl_state;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_COMMAND_ACK 0x79
#define BL_COMMAND_NACK 0x1F

#define BL_COMMAND_GET 0x00
#define BL_COMMAND_GET_VERSION_AND_READ_PROTECTION 0x01
#define BL_COMMAND_GET_ID 0x02
#define BL_COMMAND_READ_MEMORY 0x11
#define BL_COMMAND_GO 0x21
#define BL_COMMAND_WRITE_MEMORY 0x31
#define BL_COMMAND_ERASE 0x43
#define BL_COMMAND_EXTENDED_ERASE 0x44
#define BL_COMMAND_WRITE_PROTECT 0x63
#define BL_COMMAND_WRITE_UNPROTECT 0x73
#define BL_COMMAND_READOUT_PROTECT 0x82
#define BL_COMMAND_READOUT_UNPROTECT 0x92

#define BL_DEVICE_ID "URI-CRAP"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint32_t uwTick;
bl_state blState;
uint8_t rxChecksum;
uint8_t rxByte1;
volatile uint32_t nextTimeout;

uint8_t debugBuffer[512];
uint8_t debugBufferPos = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
static void consolePutString(char *sData);
static void consolePutChar(uint8_t ch);
static void bootloader_reset_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  LL_SYSCFG_DisableDBATT(LL_SYSCFG_UCPD1_STROBE | LL_SYSCFG_UCPD2_STROBE);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */

  consolePutString("\r\nSTM32-BOOTLOADER " __DATE__ " " __TIME__ "\r\n");
  bootloader_reset_state();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((nextTimeout) && (uwTick >= nextTimeout))
      bootloader_reset_state();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(16000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);
}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void) {

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART4 GPIO Configuration
   PA0   ------> USART4_TX
   PA1   ------> USART4_RX
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART4 interrupt Init */
  NVIC_SetPriority(USART3_4_LPUART1_IRQn, 0);
  NVIC_EnableIRQ(USART3_4_LPUART1_IRQn);

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART4, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART4);

  /* USER CODE BEGIN WKUPType USART4 */

  /* USER CODE END WKUPType USART4 */

  LL_USART_Enable(USART4);

  /* Polling USART4 initialisation */
  while ((!(LL_USART_IsActiveFlag_TEACK(USART4))) || (!(LL_USART_IsActiveFlag_REACK(USART4)))) {
  }
  /* USER CODE BEGIN USART4_Init 2 */
  LL_USART_EnableIT_RXNE(USART4);

  /* USER CODE END USART4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void bootloader_send_char(uint8_t ch) {
  consolePutChar(ch);
  rxChecksum ^= ch;
}

static void bootloader_start_buffer(uint8_t ch) {
  rxChecksum = 0xFF;
  bootloader_send_char(ch);
}

static void bootloader_end_buffer(uint8_t ch) {
  bootloader_send_char(ch);
  consolePutChar(rxChecksum);
  rxChecksum = 0xFF;
}

static void consolePutChar(uint8_t ch) {
  while (!LL_USART_IsActiveFlag_TXE(USART4))
    ;

  LL_USART_TransmitData8(USART4, ch);
}

static void consolePutString(char *sData) {
  while (*sData)
    consolePutChar((char) *sData++);

  while (!LL_USART_IsActiveFlag_TC(USART4))
    ;
}

static void bootloader_reset_state(void) {
  blState = bsIdle;
  rxChecksum = 0xFF;
  nextTimeout = 0;
}

static void bootloader_send_ack(void) {
  consolePutChar(BL_COMMAND_ACK);
}

static void bootloader_send_nack(void) {
  consolePutChar(BL_COMMAND_NACK);
}

static void bootloader_command_get(void) {
  bootloader_send_ack();
  bootloader_start_buffer(11); // Number of bytes to follow
  bootloader_send_char(0x10); // Bootloader version
  bootloader_send_char(BL_COMMAND_GET);
  bootloader_send_char(BL_COMMAND_GET_VERSION_AND_READ_PROTECTION);
  bootloader_send_char(BL_COMMAND_GET_ID);
  bootloader_send_char(BL_COMMAND_READ_MEMORY);
  bootloader_send_char(BL_COMMAND_GO);
  bootloader_send_char(BL_COMMAND_WRITE_MEMORY);
  bootloader_send_char(BL_COMMAND_ERASE);
  bootloader_send_char(BL_COMMAND_WRITE_PROTECT);
  bootloader_send_char(BL_COMMAND_WRITE_UNPROTECT);
  bootloader_send_char(BL_COMMAND_READOUT_PROTECT);
  bootloader_end_buffer(BL_COMMAND_READOUT_UNPROTECT);
  bootloader_send_ack();

  bootloader_reset_state();
}

static void bootloader_command_get_id(void) {

  uint32_t devID = LL_DBGMCU_GetDeviceID();
  bootloader_send_ack();
  bootloader_start_buffer(1); // Number of bytes to follow
  bootloader_send_char(devID >> 0x08);
  bootloader_end_buffer(devID & 0xFF);
  bootloader_send_ack();

  bootloader_reset_state();
}

static void bootloader_parse_command(uint8_t command) {
  if (rxChecksum != 0) {
    bootloader_send_nack();
    bootloader_reset_state();
    return;
  }

  switch (command) {
    case BL_COMMAND_GET:
      bootloader_command_get();
      break;

    case BL_COMMAND_GET_ID:
      bootloader_command_get_id();
      break;

    default:
      bootloader_send_nack();
      bootloader_reset_state();
  }
}

void USART_CharReception_Callback(uint8_t ch) {
  debugBuffer[debugBufferPos++] = ch;
  nextTimeout = uwTick + 10000;
  rxChecksum ^= ch;
  switch (blState) {
    case bsIdle:
      if (ch == 0x7F) {
        bootloader_send_ack();
        bootloader_reset_state();
      } else {
        blState = bsWaitCommand2;
        rxByte1 = ch;
      }
      break;
    case bsWaitCommand2:
      bootloader_parse_command(rxByte1);
      break;
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

