#include "GPS.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * GPS Module — Internal Implementation
 * ============================================================================ */

#define GPS_DMA_BUFFER_SIZE     256   /**< DMA ring buffer (must be > max NMEA sentence len ~82 B) */
#define NMEA_PARSE_BUFFER_SIZE  128   /**< Temp parse buffer; GPRMC can be up to ~100 chars         */
#define GPS_SPEED_THRESHOLD_KMH 5.0f  /**< Speed threshold for speed callback (Task 4.1)            */

/* ---- Module-private state ------------------------------------------------ */

static UART_HandleTypeDef *gps_huart;
static GPS_State_t          gps_state     = GPS_POWER_SAVE;
static GPS_Data_t           latest_gps_data;

/** DMA receive buffer. Written by DMA hardware; read only in GPS_Process(). */
static uint8_t              dma_rx_buffer[GPS_DMA_BUFFER_SIZE];

/**
 * Number of bytes written to dma_rx_buffer in the last DMA transfer.
 * Set from the 'Size' argument of HAL_UARTEx_RxEventCallback.
 * Volatile because it is written from IRQ context (GPS_UART_IdleCallback)
 * and read from the main loop (GPS_Process).
 */
static volatile uint16_t    dma_rx_size   = 0;
static volatile bool        data_ready_flag = false;

/* ---- Registered callbacks ------------------------------------------------ */
static GPS_ValidFixAcquiredCallback       fix_cb   = NULL;
static GPS_SpeedThresholdExceededCallback speed_cb = NULL;

/* ============================================================================
 * Private helpers
 * ============================================================================ */

/**
 * @brief Send a raw byte sequence to the GPS module over UART (blocking, short).
 *        Used for UBX protocol commands (power-save, wake).
 */
static void GPS_Send_UBX_Command(const uint8_t *cmd, uint16_t len) {
    HAL_UART_Transmit(gps_huart, (uint8_t *)cmd, len, 1000);
}

/**
 * @brief Parse a $GPRMC NMEA sentence into the latest_gps_data struct.
 *
 * $GPRMC format (field indices used here):
 *   0: $GPRMC
 *   1: HHMMSS.ss  (UTC Time)
 *   2: A/V        (Status — A = Active/valid, V = Void/invalid)
 *   3: ddmm.mmmm  (Latitude)
 *   4: N/S
 *   5: dddmm.mmmm (Longitude)
 *   6: E/W
 *   7: sss.ss     (Speed over ground, knots)
 *   8: ddd.dd     (Track angle)
 *   9: DDMMYY     (Date)
 *
 * Uses strncpy with explicit limits to prevent buffer overflow.
 * strtok modifies the working buffer; the original dma_rx_buffer is NOT touched.
 *
 * @param sentence  Pointer into dma_rx_buffer at the '$' of a $GPRMC sentence.
 */
static void GPS_Parse_NMEA(const char *sentence) {
    /* Sanity check: must start with $GPRMC */
    if (strncmp(sentence, "$GPRMC", 6) != 0) return;

    /* Work on a local copy to avoid corrupting the DMA buffer with strtok */
    char temp[NMEA_PARSE_BUFFER_SIZE];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    int  field_index = 0;
    char *token      = strtok(temp, ",");

    while (token != NULL) {
        switch (field_index) {
            case 1:
                strncpy(latest_gps_data.time, token, sizeof(latest_gps_data.time) - 1);
                latest_gps_data.time[sizeof(latest_gps_data.time) - 1] = '\0';
                break;
            case 2:
                latest_gps_data.has_fix = (token[0] == 'A');
                break;
            case 3:
                strncpy(latest_gps_data.latitude, token, sizeof(latest_gps_data.latitude) - 1);
                latest_gps_data.latitude[sizeof(latest_gps_data.latitude) - 1] = '\0';
                break;
            case 4:
                latest_gps_data.lat_direction = token[0];
                break;
            case 5:
                strncpy(latest_gps_data.longitude, token, sizeof(latest_gps_data.longitude) - 1);
                latest_gps_data.longitude[sizeof(latest_gps_data.longitude) - 1] = '\0';
                break;
            case 6:
                latest_gps_data.lon_direction = token[0];
                break;
            case 7:
                /* Convert speed from knots to km/h (1 knot = 1.852 km/h) */
                latest_gps_data.speed_kmh = atof(token) * 1.852f;
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field_index++;
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    memset(&latest_gps_data, 0, sizeof(GPS_Data_t));
    latest_gps_data.has_fix = false;

    /* Start DMA reception with idle-line detection.
     * HAL_UARTEx_ReceiveToIdle_DMA fires HAL_UARTEx_RxEventCallback when the
     * UART line goes idle (end of an NMEA sentence), or when the buffer is full.
     * Both events carry the actual number of received bytes in 'Size'. */
    HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, dma_rx_buffer, GPS_DMA_BUFFER_SIZE);

    DEBUG_PRINT("GPS: Initialized. DMA reception armed.\r\n");
}

void GPS_RegisterCallbacks(GPS_ValidFixAcquiredCallback fixCb,
                            GPS_SpeedThresholdExceededCallback speedCb) {
    fix_cb   = fixCb;
    speed_cb = speedCb;
}

void GPS_SetState(GPS_State_t state) {
    gps_state = state;

    if (state == GPS_POWER_SAVE) {
        /*
         * UBX-CFG-PM2 command: enter Power Save Mode (cyclic tracking).
         * Drops average current from ~45 mA to ~11 mA while retaining ephemeris
         * for a fast hot-start (TTFF ≈ 1 s) on the next wake.
         *
         * TODO: The checksum bytes (0x94, 0xB5) below are example values.
         *       Verify with the u-blox NEO-6M Integration Manual (section on
         *       UBX-CFG-PM2) or generate the correct checksum for your config.
         */
        const uint8_t psm_cmd[] = {
            0xB5, 0x62, 0x06, 0x86, 0x08, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
            0x94, 0xB5
        };
        GPS_Send_UBX_Command(psm_cmd, sizeof(psm_cmd));
        DEBUG_PRINT("GPS: Entering POWER_SAVE mode.\r\n");

    } else if (state == GPS_ACQUIRING_BASELINE || state == GPS_CONTINUOUS_TRACKING) {
        /*
         * Wake the NEO-6M from software Power Save Mode.
         * Sending a 0xFF dummy byte over UART is a standard wake trigger for
         * u-blox modules that support UART-based PSM wake.
         *
         * Alternatively, toggling the EXTINT0 pin (Pin 4 on NEO-6M) low→high
         * achieves the same effect and is more reliable. To use that approach:
         *   TODO: HAL_GPIO_WritePin(GPS_EXTINT_GPIO_Port, GPS_EXTINT_Pin, GPIO_PIN_SET);
         *         (add GPS_EXTINT_GPIO_Port / GPS_EXTINT_Pin to main.h first)
         *
         * After wake, TTFF for a hot-start is ~1 s. Re-arm DMA in case it was
         * stopped.
         */
        const uint8_t wake_cmd[] = { 0xFF };
        GPS_Send_UBX_Command(wake_cmd, sizeof(wake_cmd));
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, dma_rx_buffer, GPS_DMA_BUFFER_SIZE);
        DEBUG_PRINT("GPS: Waking up / entering tracking mode.\r\n");
    }
}

void GPS_Process(void) {
    /* Do nothing while in power-save — DMA is not armed */
    if (gps_state == GPS_POWER_SAVE) return;

    if (data_ready_flag) {
        data_ready_flag = false;

        /* Capture the size atomically (IRQ could re-set flag, but we own it now) */
        uint16_t rx_size = dma_rx_size;

        /* Clamp to buffer bounds and null-terminate at the ACTUAL end of data.
         * This is the critical fix: do NOT null-terminate at buffer end (255),
         * which would force strstr to scan uninitialized memory. */
        if (rx_size >= GPS_DMA_BUFFER_SIZE) {
            rx_size = GPS_DMA_BUFFER_SIZE - 1;
        }
        dma_rx_buffer[rx_size] = '\0';

        /* Find and parse $GPRMC sentence within the received data */
        char *gprmc_start = strstr((char *)dma_rx_buffer, "$GPRMC");
        if (gprmc_start != NULL) {
            GPS_Parse_NMEA(gprmc_start);

            if (gps_state == GPS_ACQUIRING_BASELINE) {
                if (latest_gps_data.has_fix) {
                    DEBUG_PRINT("GPS: Valid baseline fix acquired!\r\n");
                    gps_state = GPS_CONTINUOUS_TRACKING;
                    if (fix_cb) fix_cb();
                }
            } else if (gps_state == GPS_CONTINUOUS_TRACKING) {
                if (latest_gps_data.has_fix &&
                    latest_gps_data.speed_kmh > GPS_SPEED_THRESHOLD_KMH) {
                    if (speed_cb) speed_cb();
                }
            }
        }

        /* Re-arm DMA for the next NMEA sentence */
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, dma_rx_buffer, GPS_DMA_BUFFER_SIZE);
    }
}

/* Called from HAL_UARTEx_RxEventCallback() in main.c — IRQ context, keep short */
void GPS_UART_IdleCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == gps_huart->Instance) {
        /* Store the actual byte count BEFORE setting the flag, so GPS_Process()
         * reads a consistent value when it consumes the flag. */
        dma_rx_size     = Size;
        data_ready_flag = true;
    }
}

GPS_Data_t* GPS_GetLatestData(void) {
    return &latest_gps_data;
}