/**
 ******************************************************************************
 * @file           : bootloader.c
 * @brief          : STM32 USART Bootloader Device Code
 ******************************************************************************
 * @attention
 *
 * Don't forget to:
 * 1. Edit your system_stm32xxxx.c file and set VECT_TAB_OFFSET to the correct
 *    offset
 *
 * 2. Edit your STM32xxx_FLASH.ld file to change the offset and size
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  bsIdle, bsWaitCommand2
} bl_state;

/* Private define ------------------------------------------------------------*/
#define APP_ADDRESS 0x8002000
#define BOOTLOADER_TIMEOUT_SECONDS 30

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

#define BL_COMMAND_CUSTOM_JUMP 0x3F // '?'

static bl_state blState;
static uint8_t rxChecksum;
static uint8_t rxCmd;

static void bootloader_uart_send(uint8_t ch);
static void bootloader_reset_state(void);
static void bootloader_parse_command(uint8_t command);
static void USART_CharReception_Callback(uint8_t ch);

static USART_TypeDef *bootloader_uart;

static HAL_StatusTypeDef bootloader_erase_page(uint16_t pageNumber) {
  FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t pageError;

  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInitStruct.Page = pageNumber;
  eraseInitStruct.NbPages = 1;
  eraseInitStruct.Banks = 0;

  return HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
}

/* USER CODE BEGIN 4 */
static void bootloader_go(uint32_t address) {
  uint32_t stackAddress = *(__IO uint32_t*) address;
  uint32_t appAddress = address + 4U;

  typedef void (*Function_Pointer)(void);
  Function_Pointer jump_to_address;

  LL_USART_Disable(bootloader_uart);
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  __enable_irq();
  if (1) {

    jump_to_address = (Function_Pointer) (*(__IO uint32_t*) (appAddress));

    __set_MSP(stackAddress);

    jump_to_address();
  } else {

    asm("msr msp, %0; bx %1;" : : "r"(stackAddress), "r"(appAddress));
  }
}

static uint32_t bootloader_get_page(uint32_t address) {
  return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static void bootloader_reset_checksum(void) {
  rxChecksum = 0xFF;
}

static void bootloader_reset_state(void) {
  blState = bsIdle;
  bootloader_reset_checksum();
}

static uint8_t bootloader_get_uint8(void) {
  uint8_t ch = 0;
  uint32_t timeout = HAL_GetTick() + 1000;
  while (HAL_GetTick() < timeout) {
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(bootloader_uart)) {
      ch = LL_USART_ReceiveData8(bootloader_uart);
      break;
    }
  }
  rxChecksum ^= ch;
  return ch;
}

static uint16_t bootloader_get_uint16(void) {
  uint16_t retval;

  retval = ((uint16_t) bootloader_get_uint8() << 0x08);
  retval |= bootloader_get_uint8();
  return retval;
}

static uint32_t bootloader_get_uint32(void) {
  uint32_t retval;

  retval = ((uint16_t) bootloader_get_uint16() << 0x10);
  retval |= bootloader_get_uint16();
  return retval;
}

static void bootloader_uart_send(uint8_t ch) {
  while (!LL_USART_IsActiveFlag_TXE_TXFNF(bootloader_uart));
  LL_USART_TransmitData9(bootloader_uart, ch);
}

static void bootloader_send_char(uint8_t ch) {
  bootloader_uart_send(ch);
  rxChecksum ^= ch;
}

static void bootloader_start_buffer(void) {
  bootloader_reset_checksum();
}

static void bootloader_end_buffer(void) {
  bootloader_uart_send(rxChecksum);
  bootloader_reset_checksum();
}

static void bootloader_send_ack(void) {
  bootloader_uart_send(BL_COMMAND_ACK);
}

static void bootloader_send_nack(void) {
  bootloader_uart_send(BL_COMMAND_NACK);
}

static void bootloader_nack_reset(void) {
  bootloader_reset_state();
  bootloader_send_nack();
}

static void bootloader_command_get(void) {
  bootloader_send_ack();
  bootloader_start_buffer();
  bootloader_send_char(11); // Number of bytes to follow
  bootloader_send_char(0x31); // Bootloader version
  bootloader_send_char(BL_COMMAND_GET);
  bootloader_send_char(BL_COMMAND_GET_VERSION_AND_READ_PROTECTION);
  bootloader_send_char(BL_COMMAND_GET_ID);
  bootloader_send_char(BL_COMMAND_READ_MEMORY);
  bootloader_send_char(BL_COMMAND_GO);
  bootloader_send_char(BL_COMMAND_WRITE_MEMORY);
  bootloader_send_char(BL_COMMAND_EXTENDED_ERASE);
  bootloader_send_char(BL_COMMAND_WRITE_PROTECT);
  bootloader_send_char(BL_COMMAND_WRITE_UNPROTECT);
  bootloader_send_char(BL_COMMAND_READOUT_PROTECT);
  bootloader_send_char(BL_COMMAND_READOUT_UNPROTECT);
  bootloader_end_buffer();
  bootloader_send_ack();

  bootloader_reset_state();
}

static void bootloader_command_get_id(void) {
  uint32_t devID = HAL_GetDEVID();
  bootloader_send_ack();
  bootloader_start_buffer();
  bootloader_send_char(1); // Number of bytes to follow
  bootloader_send_char(devID >> 0x08);
  bootloader_send_char(devID & 0xFF);
  bootloader_send_ack();

  bootloader_reset_state();
}

static void bootloader_command_read_memory(void) {
// First I need to test for RDP. For now - I ignore it
  uint32_t addr;
  uint16_t readSize;

  bootloader_reset_checksum();
  bootloader_send_ack();
  addr = bootloader_get_uint32();

  bootloader_get_uint8();

  if (rxChecksum != 0xFF) { // Unclear why it's FF and not zero
    bootloader_nack_reset();
    return;
  }

  bootloader_send_ack();

  bootloader_reset_checksum();
  readSize = bootloader_get_uint8();
  bootloader_get_uint8();
  if (rxChecksum) {
    bootloader_nack_reset();
    return;
  }
  bootloader_send_ack();

  uint8_t *memAddr = (uint8_t*) addr;
  bootloader_start_buffer();
  for (uint16_t i = 0; i <= readSize; i++) {
    bootloader_send_char(*memAddr);

    memAddr++;
  }

  bootloader_reset_state();
}

static void bootloader_command_write_memory(void) {
  uint32_t start_addr;
  uint8_t number_of_bytes;
  uint8_t data[256] = { 0xFF };
  uint16_t i;
  HAL_StatusTypeDef status;

  bootloader_reset_checksum();
  bootloader_send_ack();

  // Get address
  start_addr = bootloader_get_uint32();
  bootloader_get_uint8();

  if (rxChecksum != 0xFF) { // Unclear why it's FF and not zero
    bootloader_nack_reset();
    return;
  }
  bootloader_send_ack();

  // Get number of bytes + data
  bootloader_reset_checksum();
  number_of_bytes = bootloader_get_uint8();
  for (i = 0; i <= number_of_bytes; i++)
    data[i] = bootloader_get_uint8();

  bootloader_get_uint8();

  if (rxChecksum != 0xFF) { // Unclear why it's FF and not zero
    bootloader_nack_reset();
    return;
  }

  // Program
  HAL_FLASH_Unlock();

  for (i = 0; i <= number_of_bytes; i += 8) {
    uint64_t value = (((uint64_t) data[i + 7]) << 0x38) | (((uint64_t) data[i + 6]) << 0x30) | (((uint64_t) data[i + 5]) << 0x28)
        | (((uint64_t) data[i + 4]) << 0x20) | (((uint64_t) data[i + 3]) << 0x18) | (((uint64_t) data[i + 2]) << 0x10)
        | (((uint64_t) data[i + 1]) << 0x08) | (((uint64_t) data[i + 0]) << 0x00);

    // Workaround for STM32CubeProgrammer which didn't erase the fucking page!
    if (bootloader_get_page(start_addr + i - 1) != bootloader_get_page(start_addr + i)) {
      status = bootloader_erase_page(bootloader_get_page(start_addr + i));
      if (status != HAL_OK)
        break;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr + i, value);
    if (status != HAL_OK)
      break;
  }

  HAL_FLASH_Lock();

  if (status == HAL_OK)
    bootloader_send_ack();
  else
    bootloader_send_nack();
  bootloader_reset_state();
}

static void bootloader_command_go(void) {
  uint32_t address;

  bootloader_reset_checksum();
  bootloader_send_ack();
  address = bootloader_get_uint32();

  bootloader_get_uint8();

  if (rxChecksum == 0xFF) {
    bootloader_send_ack();
    bootloader_go(address);
  } else {
    bootloader_send_nack();
  }

  bootloader_reset_state();
}

static void bootloader_command_extended_erase(void) {
  uint16_t number_of_pages;
  uint16_t i;
  uint16_t pages[128];

  bootloader_reset_checksum();
  bootloader_send_ack();
  number_of_pages = bootloader_get_uint16();

  if ((number_of_pages & 0xFFF0) == 0xFFF0) {
    // Special case - read the checksum and abort
    // I do not allow mass erase, since it'll erase this bootloader as well
    bootloader_get_uint8();
    bootloader_nack_reset();
    return;
  }

  if (number_of_pages > 127) {
    bootloader_nack_reset();
    return;
  }

  for (i = 0; i <= number_of_pages; i++)
    pages[i] = bootloader_get_uint16();

  bootloader_get_uint8();

  if (rxChecksum != 0xFF) { // Unclear why it's FF and not zero
    bootloader_nack_reset();
    return;
  }

  HAL_FLASH_Unlock();

  HAL_StatusTypeDef status;
  for (i = 0; i <= number_of_pages; i++) {
    status = bootloader_erase_page(pages[i]);
    if (status != HAL_OK)
      break;
  }

  HAL_FLASH_Lock();

  if (status == HAL_OK)
    bootloader_send_ack();
  else
    bootloader_send_nack();

  bootloader_reset_state();
}

static void bootloader_parse_command(uint8_t command) {
  rxCmd = 0;
  if (rxChecksum) {
    bootloader_send_nack();
    bootloader_reset_state();
  }

  switch (command) {
    case BL_COMMAND_GET:
      bootloader_command_get();
      break;

    case BL_COMMAND_GET_ID:
      bootloader_command_get_id();
      break;

    case BL_COMMAND_READ_MEMORY:
      bootloader_command_read_memory();
      break;

    case BL_COMMAND_EXTENDED_ERASE:
      bootloader_command_extended_erase();
      break;

    case BL_COMMAND_WRITE_MEMORY:
      bootloader_command_write_memory();
      break;

    case BL_COMMAND_GO:
      bootloader_command_go();
      break;

    default:
      bootloader_send_nack();
      bootloader_reset_state();
      break;
  }
}

void bootloader_start_user_program(void) {
  bootloader_go(APP_ADDRESS);
}

static void USART_CharReception_Callback(uint8_t ch) {
  rxChecksum ^= ch;
  switch (blState) {
    case bsIdle:
      if (ch == 0x7F) {
        bootloader_send_ack();
        bootloader_reset_state();
      } else if (ch == BL_COMMAND_CUSTOM_JUMP) {
        bootloader_start_user_program();
      } else {
        rxCmd = ch;
        blState = bsWaitCommand2;
      }
      break;

    case bsWaitCommand2:
      bootloader_parse_command(rxCmd);
      break;
  }
}

void bootloader_loop(USART_TypeDef *uart) {
  bootloader_uart = uart;
  bootloader_reset_state();

  const uint8_t *about = (uint8_t*) "STM32 BOOTLOADER-NG " __DATE__ " " __TIME__ "\r\n";
  for (uint8_t i = 0; about[i]; i++)
    bootloader_uart_send(about[i]);

  uint32_t lastAction = HAL_GetTick();
  while (HAL_GetTick() < lastAction + (BOOTLOADER_TIMEOUT_SECONDS * 1000)) {
    uint32_t targetTick = HAL_GetTick() + 5000;
    while (HAL_GetTick() < targetTick) {
      if (LL_USART_IsActiveFlag_RXNE_RXFNE(bootloader_uart)) {
        uint8_t ch = LL_USART_ReceiveData8(bootloader_uart);
        lastAction = HAL_GetTick();
        USART_CharReception_Callback(ch);
        targetTick = HAL_GetTick() + 5000;
      }
    }

    bootloader_reset_state();
  }
}
