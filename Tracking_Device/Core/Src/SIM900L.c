#include "SIM900L.h"
#include <string.h>
#include <stdio.h>

/**
  * @brief  Initializes the SIM900L module
  */
void SIM900_Init(UART_HandleTypeDef *huart) {
    // Check communication
    SIM900_SendCommand(huart, "AT\r\n");
    HAL_Delay(1000);
    
    // Set SMS to Text Mode
    SIM900_SendCommand(huart, "AT+CMGF=1\r\n");
    HAL_Delay(1000);
    
    // Select character set
    SIM900_SendCommand(huart, "AT+CSCS=\"GSM\"\r\n");
    HAL_Delay(1000);
}

/**
  * @brief  Sends an SMS message
  * @param  phone_number: Recipient number (e.g., "+123456789")
  * @param  message: The content to send
  */
void SIM900_SendSMS(UART_HandleTypeDef *huart, char *phone_number, char *message) {
    char cmd[50];
    uint8_t ctrlZ = 0x1A; // ASCII for Ctrl+Z (End of message)

    // 1. Set the destination number
    sprintf(cmd, "AT+CMGS=\"%s\"\r\n", phone_number);
    SIM900_SendCommand(huart, cmd);
    HAL_Delay(1000); // Wait for '>' prompt

    // 2. Send the actual message body
    HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), 1000);
    HAL_Delay(500);

    // 3. Send Ctrl+Z to finish the SMS
    HAL_UART_Transmit(huart, &ctrlZ, 1, 1000);
    HAL_Delay(5000); // Give it time to send over the network
}

/**
  * @brief  Generic helper to send AT commands
  */
void SIM900_SendCommand(UART_HandleTypeDef *huart, char *command) {
    HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000);
}