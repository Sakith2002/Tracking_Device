#ifndef __SIM7670_H
#define __SIM7670_H

#include "stm32f1xx_hal.h"

// Function Prototypes
void SIM7670_Init(UART_HandleTypeDef *huart);
uint8_t SIM7670_Wait_For_Network(UART_HandleTypeDef *huart, uint32_t timeout_ms);
void SIM7670_SendSMS(UART_HandleTypeDef *huart, char *phone_number, char *message);
void SIM7670_SendCommand(UART_HandleTypeDef *huart, char *command);

// Power Management Functions
void SIM7670_Sleep(UART_HandleTypeDef *huart);
void SIM7670_WakeUp(UART_HandleTypeDef *huart);

#endif