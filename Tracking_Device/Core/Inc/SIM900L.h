#ifndef __SIM900L_H
#define __SIM900L_H

#include "stm32f1xx_hal.h"

// Function Prototypes
void SIM900_Init(UART_HandleTypeDef *huart);
void SIM900_SendSMS(UART_HandleTypeDef *huart, char *phone_number, char *message);
void SIM900_SendCommand(UART_HandleTypeDef *huart, char *command);

#endif