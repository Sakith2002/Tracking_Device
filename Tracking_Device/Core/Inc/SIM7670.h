#ifndef SIM7670_H
#define SIM7670_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ============================================================================
 * SIM7670 (A7670C) Cellular Module
 * ============================================================================
 * Manages the cellular modem state machine for non-blocking HTTP GET uploads.
 *
 * State Machine:
 *   SIM_POWER_OFF         - Modem powered down (PWRKEY held low 2.5s). Lowest
 *                           power state. Entered after final TX before MCU Stop.
 *   SIM_BOOTING           - PWRKEY pulsed 50ms; waiting 11.2s for UART ready.
 *   SIM_NETWORK_CONFIG    - UART ready. Configures APN, enables HTTP context,
 *                           and sends AT+CSCLK=1 for light-sleep capability.
 *   SIM_LIGHT_SLEEP       - DTR pin HIGH. Modem sleeps; UART inactive.
 *                           Stays attached to network. Current ~2–4 mA typical.
 *   SIM_ACTIVE_TX         - DTR asserted LOW. Modem awake. Drives HTTP sub-FSM
 *                           (HTTPINIT → HTTPPARA → HTTPACTION → HTTPTERM).
 *   SIM_AWAITING_RESPONSE - AT command dispatched; waiting for URC or timeout.
 *                           Yields MCU (main loop proceeds) until RX fires.
 *   SIM_BUSY              - Reserved / placeholder for future use (e.g., marking
 *                           the modem as occupied to prevent concurrent requests).
 *
 * Timing constants (from hardware spec):
 *   Cold boot to UART ready : 11.2 s   (SIM_BOOTING wait_timeout)
 *   Power-off PWRKEY pulse  :  2.5 s   (blocking GPIO in SIM7670_PowerOff_Nonblocking)
 *   Reboot buffer time      :  2.0 s   (mandatory off→on gap)
 *   DTR wake settle         : 50 ms    (before first AT command after DTR↓)
 *
 * Inter-module communication:
 *   - Does NOT call any other module directly.
 *   - Fires SIM_TransmitCompleteCallback (registered by main.c) on TX success/fail.
 *   - SIM7670_UART_RxCpltCallback() must be called from HAL_UARTEx_RxEventCallback().
 * ============================================================================ */

typedef enum {
    SIM_POWER_OFF = 0,
    SIM_BOOTING,
    SIM_NETWORK_CONFIG,
    SIM_LIGHT_SLEEP,
    SIM_ACTIVE_TX,
    SIM_AWAITING_RESPONSE,
    SIM_BUSY              /**< Reserved — modem is occupied, reject new requests */
} SIM_State_t;

/**
 * @brief Fired when an HTTP transaction completes or permanently fails.
 *
 * @param success  true  = HTTP 200 received; FIFO may be cleared.
 *                 false = AT error or timeout after retries; data was NOT sent.
 */
typedef void (*SIM_TransmitCompleteCallback)(bool success);

/* ---- Public API ---------------------------------------------------------- */

/**
 * @brief Initialise the SIM module and start DMA reception.
 *
 * Associates the UART handle and arms DMA for incoming URCs.
 * Must be called once during system init (Task 1.5).
 *
 * @param huart  Pointer to the UART handle connected to the A7670C.
 *               Must have RTS/CTS hardware flow control enabled.
 *               (e.g., &huart1 on USART1)
 */
void SIM7670_Init(UART_HandleTypeDef *huart);

/**
 * @brief Register the transmit-complete callback.
 *
 * @param txCb  Callback invoked on HTTP transaction completion or permanent failure.
 */
void SIM7670_RegisterCallback(SIM_TransmitCompleteCallback txCb);

/**
 * @brief SIM state machine process function. Call every main loop iteration.
 *
 * Drives non-blocking boot timing, network config sequencing, AT command
 * dispatch, URC response parsing, and HTTP sub-FSM advancement.
 */
void SIM7670_Process(void);

/**
 * @brief Power on the modem from SIM_POWER_OFF.
 *
 * Pulses PWRKEY low for 50 ms (hardware boot trigger), then transitions to
 * SIM_BOOTING and starts the 11.2 s non-blocking UART-ready timer.
 *
 * Precondition: sim_state == SIM_POWER_OFF.
 * No-op if called in any other state.
 */
void SIM7670_PowerOn(void);

/**
 * @brief Initiate a non-blocking modem power-off sequence.
 *
 * Transitions state to SIM_POWER_OFF immediately so no further AT commands
 * are dispatched. The physical PWRKEY pulse (2.5 s) is handled by the
 * hardware GPIO inside this function.
 *
 * NOTE: After this call, the caller must wait at least Toff(uart) = 1.9 s
 * before attempting to re-enter Stop Mode, to allow the modem UART to
 * fully go silent (handled in On_SIM_TransmitComplete in main.c).
 *
 * TODO: Replace the stub GPIO calls inside SIM7670_Pulse_PWRKEY() with
 *       actual HAL_GPIO_WritePin() using SIM_PWRKEY_GPIO_Port / SIM_PWRKEY_Pin
 *       from main.h, and implement non-blocking timing via SIM7670_Process().
 */
void SIM7670_PowerOff(void);

/**
 * @brief Wake the modem from SIM_LIGHT_SLEEP to prepare for TX.
 *
 * Asserts DTR low and waits 50 ms for UART to wake (handled by process timer),
 * then transitions to SIM_ACTIVE_TX.
 *
 * Precondition: sim_state == SIM_LIGHT_SLEEP.
 *
 * TODO: Add actual GPIO write for DTR pin using SIM_DTR_GPIO_Port / SIM_DTR_Pin
 *       from main.h.
 */
void SIM7670_WakeFromSleep(void);

/**
 * @brief Trigger an HTTP GET upload of one GPS coordinate tuple.
 *
 * Constructs the AT+HTTPPARA URL string and transitions to SIM_ACTIVE_TX.
 * If the modem is in SIM_LIGHT_SLEEP, SIM7670_WakeFromSleep() is called first.
 *
 * The function does NOT block. SIM7670_Process() will advance the HTTP
 * sub-FSM on subsequent main loop iterations.
 *
 * @param time     UTC time string from GPS_Data_t.time (e.g. "123519.00")
 * @param lat      Latitude string from GPS_Data_t.latitude (e.g. "4807.0380")
 * @param lat_dir  'N' or 'S'
 * @param lon      Longitude string from GPS_Data_t.longitude (e.g. "01131.0000")
 * @param lon_dir  'E' or 'W'
 */
void SIM7670_Send_Data(const char* time, const char* lat, char lat_dir,
                       const char* lon, char lon_dir);

/**
 * @brief Notify the SIM module of a completed UART DMA receive event.
 *
 * Must be called from HAL_UARTEx_RxEventCallback() in main.c.
 * Sets the sim_rx_ready flag which SIM_AWAITING_RESPONSE checks each loop.
 *
 * @param huart  The UART handle that fired the event.
 */
void SIM7670_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief Return the current SIM state (for inspection by main.c orchestration).
 *
 * @return Current SIM_State_t value.
 */
SIM_State_t SIM7670_GetState(void);

#endif /* SIM7670_H */