#ifndef __GPS_H
#define __GPS_H

#include "stm32f1xx_hal.h"

typedef struct {
    char latitude[15];
    char lat_direction; // N or S
    char longitude[15];
    char lon_direction; // E or W
    char time[15];      // UTC Time
    uint8_t fix;        // 0 = No Fix, 1 = Fix valid
} GPS_Data_t;

void GPS_Init(void);
void GPS_Set_EcoMode(UART_HandleTypeDef *huart);
uint8_t GPS_Parse_NMEA(char *nmea_sentence, GPS_Data_t *gps);

#endif