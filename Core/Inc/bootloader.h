#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

extern void bootloader_loop(UART_HandleTypeDef *uart);
extern void bootloader_start_user_program(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
