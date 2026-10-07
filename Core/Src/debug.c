#include "debug.h"
#include <stdarg.h>
#include <stdio.h>      /* vsnprintf only — no stream I/O used */

#define DBG_BUF 128

static UART_HandleTypeDef *dbg_uart = NULL;

void DBG_Init(UART_HandleTypeDef *huart) {
    dbg_uart = huart;
}

void DBG_Print(const char *fmt, ...) {
    if (!dbg_uart) return;
    char buf[DBG_BUF];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len > 0) {
        HAL_UART_Transmit(dbg_uart, (uint8_t *)buf,
                          (len < DBG_BUF) ? (uint16_t)len : DBG_BUF - 1, 50);
    }
}
