#ifndef GPS_H
#define GPS_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ============================================================================
 * GPS (u-blox NEO-6M) Module
 * ============================================================================
 * Manages NMEA ingestion via UART DMA and the GPS state machine.
 *
 * State Machine:
 *   GPS_POWER_SAVE         - GPS in UBX software Power Save Mode. DMA is
 *                            paused. Ephemeris retained for fast hot-start.
 *   GPS_ACQUIRING_BASELINE - Active after cold boot. Discards invalid $GPRMC
 *                            packets until first valid fix, then fires
 *                            GPS_ValidFixAcquiredCallback and moves to
 *                            GPS_CONTINUOUS_TRACKING.
 *   GPS_CONTINUOUS_TRACKING - DMA silently buffers 1Hz NMEA. Idle-line IRQ
 *                            wakes CPU only when a full sentence arrives.
 *                            GPS_SpeedThresholdExceededCallback fires if speed
 *                            > 5 km/h, allowing the IMU to reset its 5-minute
 *                            stationary timer.
 *
 * Inter-module communication:
 *   - Does NOT call any other module directly.
 *   - Exposes callbacks (registered by main.c) for validated events.
 *   - GPS_UART_IdleCallback() must be called from HAL_UARTEx_RxEventCallback()
 *     in main.c, forwarding the DMA transfer size.
 * ============================================================================ */

typedef enum {
    GPS_POWER_SAVE = 0,
    GPS_ACQUIRING_BASELINE,
    GPS_CONTINUOUS_TRACKING
} GPS_State_t;

/**
 * @brief Parsed and buffered GPS data from the most recent valid $GPRMC sentence.
 */
typedef struct {
    char     latitude[15];    /**< Raw NMEA latitude string (ddmm.mmmm)       */
    char     lat_direction;   /**< 'N' or 'S'                                  */
    char     longitude[15];   /**< Raw NMEA longitude string (dddmm.mmmm)      */
    char     lon_direction;   /**< 'E' or 'W'                                  */
    char     time[15];        /**< UTC Time (HHMMSS.ss)                         */
    float    speed_kmh;       /**< Speed over ground in km/h (converted from kt)*/
    bool     has_fix;         /**< true if status field is 'A' (Active/valid)   */
} GPS_Data_t;

/* ---- Callback type definitions ------------------------------------------ */

/**
 * @brief Fired once when the first valid $GPRMC fix is received.
 *        Used by main.c to transition SYS_COLD_BOOT → SYS_ACTIVE_TRANSIT.
 */
typedef void (*GPS_ValidFixAcquiredCallback)(void);

/**
 * @brief Fired during GPS_CONTINUOUS_TRACKING whenever speed > 5 km/h.
 *        Used by main.c to call MPU6050_Reset_Stationary_Timer(), preventing
 *        a false sleep entry during smooth highway driving.
 */
typedef void (*GPS_SpeedThresholdExceededCallback)(void);

/* ---- Public API ---------------------------------------------------------- */

/**
 * @brief Initialise the GPS module.
 *
 * Associates the UART handle, clears the data buffer, and starts DMA reception
 * with idle-line detection. Must be called once during system init (Task 1.4).
 *
 * @param huart  Pointer to the UART handle connected to the NEO-6M TX pin.
 *               (e.g., &huart2 on USART2)
 */
void GPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief Register event callbacks with the GPS module.
 *
 * Both callbacks are optional (pass NULL to skip). Must be called before
 * entering the main loop.
 *
 * @param fixCb    Called when the first valid fix is acquired.
 * @param speedCb  Called whenever GPS speed exceeds 5 km/h.
 */
void GPS_RegisterCallbacks(GPS_ValidFixAcquiredCallback fixCb,
                           GPS_SpeedThresholdExceededCallback speedCb);

/**
 * @brief GPS state machine process function. Call every main loop iteration.
 *
 * Checks the data_ready_flag set by GPS_UART_IdleCallback(), parses the
 * buffered NMEA sentence, handles state transitions, fires callbacks, and
 * re-arms DMA reception.
 */
void GPS_Process(void);

/**
 * @brief Notify the GPS module of a completed DMA transfer.
 *
 * Must be called from HAL_UARTEx_RxEventCallback() in main.c.
 * The 'Size' parameter is the actual number of bytes written by DMA into the
 * buffer — used to correctly null-terminate the buffer before parsing.
 *
 * @param huart  The UART handle that fired the event (used for instance check).
 * @param Size   Number of bytes received by DMA in this transfer.
 */
void GPS_UART_IdleCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief Manually set the GPS state and issue the corresponding hardware command.
 *
 * - GPS_POWER_SAVE:          Sends UBX-CFG-PM2 to enter software PSM.
 * - GPS_ACQUIRING_BASELINE:  Sends wake sequence (0xFF dummy byte or UBX wake).
 * - GPS_CONTINUOUS_TRACKING: Same wake sequence; continuous tracking begins.
 *
 * @param state  Target GPS state.
 */
void GPS_SetState(GPS_State_t state);

/**
 * @brief Return a pointer to the most recently parsed GPS data.
 *
 * The caller must check has_fix before using coordinate or speed fields.
 *
 * @return Pointer to the internal GPS_Data_t struct (not thread-safe; read
 *         only from main loop context, not from ISRs).
 */
GPS_Data_t* GPS_GetLatestData(void);

#endif /* GPS_H */