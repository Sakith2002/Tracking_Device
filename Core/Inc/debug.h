#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f1xx_hal.h"

/**
 * Lightweight UART debug output.
 * Call DBG_Init() once during system init, then use DBG_Print() for
 * formatted output. Uses vsnprintf + HAL_UART_Transmit — no printf/stdio
 * stream infrastructure required in the calling modules.
 */

void DBG_Init(UART_HandleTypeDef *huart);
void DBG_Print(const char *fmt, ...);

#endif /* DEBUG_H */
