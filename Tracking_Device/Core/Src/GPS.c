#include "GPS.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/**
  * @brief  Initializes the GPS driver. 
  */
void GPS_Init(void) {
    // Standard UART is ready. 
    // Note: To enable Eco Mode automatically via software without hardware jumpers, 
    // it is strongly recommended to configure the module once using u-blox u-center 
    // and save to its internal EEPROM (as per Section 1.15.1 of the NEO-6 Data Sheet).
}

/**
  * @brief  Optional: Sends a UBX command to set Power Save / Eco Mode dynamically
  * @param  huart: Pointer to UART handle connected to NEO-6M
  */
void GPS_Set_EcoMode(UART_HandleTypeDef *huart) {
    // UBX-CFG-PWR (Power Management Setup) 
    // This is the standard UBX binary packet to request Power Save Mode (Eco Mode)
    uint8_t psm_cmd[] = {
        0xB5, 0x62,         // Header (Sync Chars)
        0x06, 0x86,         // Class ID (CFG) / Message ID (PWR)
        0x08, 0x00,         // Payload length (8 bytes)
        0x00, 0x00, 0x00, 0x00, // Version, Reserved, Flags
        0x02, 0x00, 0x00, 0x00, // Backup/Periodic setup (Mode = Power Save)
        0x94, 0xB5          // Checksum (CK_A, CK_B) for default power management packet
    };
    
    HAL_UART_Transmit(huart, psm_cmd, sizeof(psm_cmd), 1000);
}

/**
  * @brief  Parses a raw NMEA sentence, looking specifically for $GPRMC
  * @param  nmea_sentence: Pointer to the raw string received from UART
  * @param  gps: Pointer to the GPS_Data_t structure where parsed data will be stored
  * @retval 1 if parsed successfully with a valid fix, 0 otherwise
  */
uint8_t GPS_Parse_NMEA(char *nmea_sentence, GPS_Data_t *gps) {
    // Check if the sentence is $GPRMC
    if (strncmp(nmea_sentence, "$GPRMC", 6) != 0) {
        return 0; 
    }

    char *token;
    char temp_sentence[100];
    strcpy(temp_sentence, nmea_sentence); // Copy so we don't destroy the original buffer

    int field_index = 0;
    token = strtok(temp_sentence, ",");

    while (token != NULL) {
        switch (field_index) {
            case 1: // UTC Time
                strcpy(gps->time, token);
                break;
            case 2: // Status: A = Data valid (Fix), V = Warning/Invalid
                if (token[0] == 'A') {
                    gps->fix = 1;
                } else {
                    gps->fix = 0;
                }
                break;
            case 3: // Latitude (ddmm.mmmm)
                strcpy(gps->latitude, token);
                break;
            case 4: // Latitude Direction (N/S)
                gps->lat_direction = token[0];
                break;
            case 5: // Longitude (dddmm.mmmm)
                strcpy(gps->longitude, token);
                break;
            case 6: // Longitude Direction (E/W)
                gps->lon_direction = token[0];
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field_index++;
    }

    return gps->fix; // Returns 1 if valid fix is present
}