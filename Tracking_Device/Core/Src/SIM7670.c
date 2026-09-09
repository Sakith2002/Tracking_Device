#include "SIM7670.h"
#include <string.h>
#include <stdio.h>

/**
  * @brief  Initializes the SIM7670 module and sets SMS to text mode
  */
void SIM7670_Init(UART_HandleTypeDef *huart) {
    HAL_Delay(3000); // Give A7670 time to boot up

    SIM7670_SendCommand(huart, "AT\r\n");
    HAL_Delay(500);

    SIM7670_SendCommand(huart, "ATE0\r\n"); // Disable echo
    HAL_Delay(500);

    SIM7670_SendCommand(huart, "AT+CMGF=1\r\n"); // Text mode for SMS
    HAL_Delay(500);
}

/**
  * @brief  Waits for the A7670 to register on the 4G/LTE network
  * @retval 1 if registered, 0 if timeout
  */
uint8_t SIM7670_Wait_For_Network(UART_HandleTypeDef *huart, uint32_t timeout_ms) {
    uint32_t startTick = HAL_GetTick();
    char rx_data[50];

    while ((HAL_GetTick() - startTick) < timeout_ms) {
        memset(rx_data, 0, sizeof(rx_data));
        SIM7670_SendCommand(huart, "AT+CREG?\r\n");
        
        if (HAL_UART_Receive(huart, (uint8_t*)rx_data, 30, 1000) == HAL_OK) {
            // Check if registered (Home: 0,1 or Roaming: 0,5)
            if (strstr(rx_data, "+CREG: 0,1") != NULL || strstr(rx_data, "+CREG: 0,5") != NULL) {
                return 1; 
            }
        }
        HAL_Delay(2000); 
    }
    return 0; 
}

/**
  * @brief  Sends an SMS message
  */
void SIM7670_SendSMS(UART_HandleTypeDef *huart, char *phone_number, char *message) {
    char cmd[50];
    uint8_t ctrlZ = 0x1A; 

    sprintf(cmd, "AT+CMGS=\"%s\"\r\n", phone_number);
    SIM7670_SendCommand(huart, cmd);
    HAL_Delay(1000); 

    HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), 1000);
    HAL_Delay(500);

    HAL_UART_Transmit(huart, &ctrlZ, 1, 1000);
    HAL_Delay(5000); 
}

/**
  * @brief  Puts the SIM7670 into Minimum Functionality Mode (Sleep / RF Disabled)
  */
void SIM7670_Sleep(UART_HandleTypeDef *huart) {
    SIM7670_SendCommand(huart, "AT+CFUN=0\r\n");
    HAL_Delay(1000);
}

/**
  * @brief  Wakes the SIM7670 up back to Full Functionality Mode (RF Enabled)
  */
void SIM7670_WakeUp(UART_HandleTypeDef *huart) {
    SIM7670_SendCommand(huart, "AT+CFUN=1\r\n");
    HAL_Delay(3000); // Allow time to re-initialize RF and connect to network
}

/**
  * @brief  Generic helper to send AT commands
  */
void SIM7670_SendCommand(UART_HandleTypeDef *huart, char *command) {
    HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000);
}