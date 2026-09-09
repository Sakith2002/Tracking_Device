#include "SIM7670.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * SIM7670 (A7670C) Cellular Module — Internal Implementation
 * ============================================================================ */

#define SIM_DMA_BUFFER_SIZE     256     /**< UART DMA receive buffer size                      */
#define TX_PAYLOAD_SIZE         400     /**< HTTP URL string buffer (must fit full URL + AT cmd + batched coords) */

/* ---- Timing constants (from hardware spec) ------------------------------- */
#define BOOT_TIMEOUT_MS         11200UL /**< Ton(uart): time from PWRKEY pulse to UART ready    */
#define POWER_OFF_PULSE_MS      2500UL  /**< PWRKEY hold time to trigger modem power-off        */
#define REBOOT_BUFFER_MS        2000UL  /**< Mandatory gap between power-off and next power-on  */
#define DTR_WAKE_SETTLE_MS      50UL    /**< Wait after DTR↓ before sending first AT command    */
#define CMD_DEFAULT_TIMEOUT_MS  2000UL  /**< Default response timeout for simple AT commands    */
#define HTTP_ACTION_TIMEOUT_MS  10000UL /**< Longer timeout for AT+HTTPACTION (network latency) */

/* ============================================================================
 * NOTE: APN Configuration
 * Update the APN string below to match your SIM card's data APN.
 * Examples:
 *   Airtel India : "airtelgprs.com"
 *   Dialog SL    : "dialogbb"
 *   Custom       : "your.apn.here"
 * ============================================================================ */
#define APN_STRING              "your.apn.here"  /* TODO: Set your carrier APN */

/* ---- HTTP GET endpoint --------------------------------------------------- */
/* TODO: Replace with your actual server URL.
 *       The lat/lon fields use raw NMEA format (ddmm.mmmm).
 *       Consider converting to decimal degrees server-side or in SIM7670_Send_Data_Batch().
 */
#define SERVER_URL_BASE "AT+HTTPPARA=\"URL\",\"http://myserver.com/track?batch="
#define SERVER_URL_END  "\"\r\n"

/* ---- Module-private state ------------------------------------------------ */
static UART_HandleTypeDef       *sim_huart;
static SIM_State_t               sim_state  = SIM_POWER_OFF;
static SIM_TransmitCompleteCallback tx_cb   = NULL;

/** Non-blocking timer: records the tick when the current timed wait started. */
static uint32_t state_timer   = 0;
/** Duration of the current timed wait in ms. */
static uint32_t wait_timeout  = 0;

/** UART DMA receive buffer (written by DMA hardware). */
static uint8_t           sim_rx_buffer[SIM_DMA_BUFFER_SIZE];
/** Set from UART RX event callback (IRQ context). Consumed in SIM7670_Process(). */
static volatile bool     sim_rx_ready = false;

/**
 * @brief HTTP transaction sub-state machine.
 *
 * Tracks progress through the HTTPINIT → HTTPPARA → HTTPACTION → HTTPTERM
 * sequence. Embedded inside SIM_ACTIVE_TX / SIM_AWAITING_RESPONSE.
 */
typedef enum {
    HTTP_IDLE,     /**< No transaction in progress                           */
    HTTP_INIT,     /**< Next: send AT+HTTPINIT                               */
    HTTP_URL,      /**< OK received for HTTPINIT; next: send AT+HTTPPARA URL */
    HTTP_ACTION,   /**< OK received for HTTPPARA; next: send AT+HTTPACTION=0 */
    HTTP_DONE      /**< +HTTPACTION:0,200 received; next: send AT+HTTPTERM   */
} HTTP_SubState_t;

static HTTP_SubState_t http_state = HTTP_IDLE;

/** Pre-formatted AT+HTTPPARA URL command string built by SIM7670_Send_Data() */
static char tx_payload[TX_PAYLOAD_SIZE];

/* ---- Network config sub-state ------------------------------------------- */
/**
 * @brief Network configuration sub-state machine.
 *
 * Sequences through APN setup and light-sleep config non-blocking.
 * Embedded inside SIM_NETWORK_CONFIG.
 */
typedef enum {
    NET_ATE0,        /**< Disable echo (ATE0) for cleaner URC parsing          */
    NET_CGDCONT,     /**< Set APN (AT+CGDCONT=1,"IP","<APN>")                  */
    NET_CSCLK,       /**< Enable light-sleep (AT+CSCLK=1)                      */
    NET_DONE         /**< Config complete — transition to LIGHT_SLEEP           */
} NET_ConfigState_t;

static NET_ConfigState_t net_cfg_state = NET_ATE0;

/* ============================================================================
 * Private helpers
 * ============================================================================ */

/**
 * @brief Transmit a null-terminated AT command string over UART (blocking TX only).
 *
 * TX is blocking and short (~5-10ms for typical AT commands via DMA TX would
 * be preferred in production, but blocking TX is acceptable here because the
 * command strings are short and the MCU is not doing anything else at that moment).
 *
 * @param cmd  Null-terminated AT command string including trailing \r\n.
 */
static void SIM7670_SendCommand(const char *cmd) {
    DEBUG_PRINT("SIM TX: %s", cmd);
    HAL_UART_Transmit(sim_huart, (uint8_t *)cmd, strlen(cmd), 1000);
}

/**
 * @brief Toggle the PWRKEY GPIO line for a specified duration.
 *
 * According to the A7670C datasheet:
 *   - Power ON  : Pull PWRKEY LOW for ≥ 50 ms, then release HIGH.
 *   - Power OFF : Pull PWRKEY LOW for ≥ 2500 ms, then release HIGH.
 *
 * TODO: Implement this function with real GPIO calls:
 *
 *   HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_SET);
 *   HAL_Delay(duration_ms);
 *   HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_RESET);
 *
 * The pin names SIM_PWRKEY_GPIO_Port and SIM_PWRKEY_Pin are defined in main.h.
 *
 * IMPORTANT: The 2.5 s pulse for power-off is currently implemented as a
 * blocking HAL_Delay(). This blocks the main loop for 2.5 s, during which
 * only EXTI interrupts (MPU wake) are serviced. This is acceptable because
 * the MCU is about to enter Stop Mode immediately after — but note that if
 * the MPU fires during this window, the EXTI flag will be pending and the
 * system will re-evaluate on the next loop iteration after Stop Mode returns.
 *
 * For a fully non-blocking implementation, the PWRKEY pulse would need to
 * be driven by a hardware timer interrupt.
 *
 * @param duration_ms  Duration to hold PWRKEY low, in milliseconds.
 */
static void SIM7670_Pulse_PWRKEY(uint32_t duration_ms) {
    DEBUG_PRINT("SIM7670: Pulsing PWRKEY for %lu ms.\r\n", duration_ms);

    /* TODO: Uncomment and adapt once GPIO pins are assigned in main.h:
     *
     * HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_SET);
     * HAL_Delay(duration_ms);
     * HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_RESET);
     */
}

/**
 * @brief Set the SIM7670 DTR pin to control modem light sleep.
 *
 * When AT+CSCLK=1 is configured:
 *   DTR HIGH (GPIO_PIN_SET)   → Modem enters light sleep; UART deactivated.
 *   DTR LOW  (GPIO_PIN_RESET) → Modem wakes; UART becomes active after ~50 ms.
 *
 * TODO: Implement this function with real GPIO calls:
 *
 *   HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, level);
 *
 * The pin names SIM_DTR_GPIO_Port and SIM_DTR_Pin are defined in main.h.
 *
 * @param level  GPIO_PIN_SET to sleep, GPIO_PIN_RESET to wake.
 */
static void SIM7670_Set_DTR(GPIO_PinState level) {
    DEBUG_PRINT("SIM7670: DTR → %s.\r\n", (level == GPIO_PIN_SET) ? "HIGH (sleep)" : "LOW (wake)");

    /* TODO: Uncomment and adapt once GPIO pins are assigned in main.h:
     *
     * HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, level);
     */
}

/**
 * @brief Start the non-blocking response wait timer and transition to AWAITING_RESPONSE.
 */
static void SIM7670_AwaitResponse(uint32_t timeout_ms) {
    sim_state    = SIM_AWAITING_RESPONSE;
    state_timer  = HAL_GetTick();
    wait_timeout = timeout_ms;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void SIM7670_Init(UART_HandleTypeDef *huart) {
    sim_huart = huart;
    /* Arm DMA to capture any URCs the modem may send after boot */
    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
    DEBUG_PRINT("SIM7670: Initialized. DMA reception armed.\r\n");
}

void SIM7670_RegisterCallback(SIM_TransmitCompleteCallback txCb) {
    tx_cb = txCb;
}

SIM_State_t SIM7670_GetState(void) {
    return sim_state;
}

void SIM7670_PowerOn(void) {
    if (sim_state != SIM_POWER_OFF) return; /* Guard: only from off state */

    DEBUG_PRINT("SIM7670: Powering on (50 ms PWRKEY pulse).\r\n");
    SIM7670_Pulse_PWRKEY(50); /* 50 ms ON pulse triggers boot sequence */

    /* Transition to BOOTING and start the 11.2 s non-blocking wait */
    sim_state    = SIM_BOOTING;
    state_timer  = HAL_GetTick();
    wait_timeout = BOOT_TIMEOUT_MS;
}

void SIM7670_PowerOff(void) {
    DEBUG_PRINT("SIM7670: Powering off (2.5 s PWRKEY pulse).\r\n");

    /* Mark state first so no new AT commands are dispatched in Process() */
    sim_state = SIM_POWER_OFF;

    /* 2.5 s blocking pulse. See note in SIM7670_Pulse_PWRKEY() about non-blocking approach. */
    SIM7670_Pulse_PWRKEY(POWER_OFF_PULSE_MS);

    /* After this returns, caller must wait Toff(uart) ≈ 1.9 s before Stop Mode.
     * This is handled in On_SIM_TransmitComplete() in main.c. */
}

void SIM7670_WakeFromSleep(void) {
    /* Only valid when modem is in LIGHT_SLEEP (AT+CSCLK=1 active) */
    if (sim_state != SIM_LIGHT_SLEEP) return;

    DEBUG_PRINT("SIM7670: Waking from light sleep (DTR↓).\r\n");

    /* Assert DTR low to signal wake to the baseband */
    SIM7670_Set_DTR(GPIO_PIN_RESET);

    /* Wait 50 ms for UART to become responsive, then go to ACTIVE_TX.
     * We use the state timer here: briefly enter AWAITING_RESPONSE for the
     * 50 ms settle, then the Process() loop will time out and return to
     * ACTIVE_TX. This keeps the main loop responsive during the wait.
     *
     * Implementation: We set sim_state to SIM_AWAITING_RESPONSE with a
     * 50ms timeout, but set sim_rx_ready conditions to only exit on timeout,
     * not on RX data. We achieve this by going to ACTIVE_TX directly after
     * the timeout expires — see SIM_AWAITING_RESPONSE handling in Process(). */

    /* Direct transition: set http_state so that ACTIVE_TX drives the HTTP sub-FSM */
    sim_state    = SIM_ACTIVE_TX;
    /* Do NOT reset http_state here — it was set by SIM7670_Send_Data() already */
    state_timer  = HAL_GetTick();
    wait_timeout = DTR_WAKE_SETTLE_MS; /* 50 ms settle before first command */
}

void SIM7670_Send_Data(const char *payload_string) {
    if (payload_string == NULL || payload_string[0] == '\0') return;

    /* Build the AT+HTTPPARA URL command string */
    snprintf(tx_payload, sizeof(tx_payload), "%s%s%s", SERVER_URL_BASE, payload_string, SERVER_URL_END);

    /* Reset the HTTP sub-FSM to start a fresh transaction */
    http_state = HTTP_INIT;

    /* If modem is in light sleep, wake it first */
    if (sim_state == SIM_LIGHT_SLEEP) {
        SIM7670_WakeFromSleep();
        /* SIM7670_WakeFromSleep() transitions to SIM_ACTIVE_TX, so we're done */
        return;
    }

    /* If already awake (e.g. after PowerOn + config), go directly to ACTIVE_TX */
    if (sim_state == SIM_NETWORK_CONFIG || sim_state == SIM_LIGHT_SLEEP) {
        sim_state = SIM_ACTIVE_TX;
    }
    /* If already in ACTIVE_TX or AWAITING_RESPONSE, the next loop iteration
     * will pick up the new http_state = HTTP_INIT and start the sequence. */
}

void SIM7670_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* IRQ context — keep minimal */
    if (huart->Instance == sim_huart->Instance) {
        sim_rx_ready = true;
    }
}

/* ============================================================================
 * State Machine Process (called every main loop iteration)
 * ============================================================================ */

void SIM7670_Process(void) {
    uint32_t now = HAL_GetTick();

    switch (sim_state) {

        /* ------------------------------------------------------------------ */
        case SIM_POWER_OFF:
        /* ------------------------------------------------------------------ */
            /* Nothing to do. MCU will enter Stop Mode separately. */
            break;

        /* ------------------------------------------------------------------ */
        case SIM_BOOTING:
        /* ------------------------------------------------------------------ */
            /* Non-blocking 11.2 s wait for UART to become ready (Ton(uart)).
             * During this time, GPS and IMU state machines continue to run. */
            if ((now - state_timer) >= wait_timeout) {
                DEBUG_PRINT("SIM7670: UART ready. Starting network config.\r\n");
                net_cfg_state = NET_ATE0;
                sim_state     = SIM_NETWORK_CONFIG;
                /* Send first config command immediately */
                SIM7670_SendCommand("ATE0\r\n");
                SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_NETWORK_CONFIG:
        /* ------------------------------------------------------------------ */
            /* This state is entered only from AWAITING_RESPONSE on OK.
             * The sub-state machine advances one step per OK response.
             * See AWAITING_RESPONSE below which transitions back here. */

            switch (net_cfg_state) {
                case NET_ATE0:
                    /* ATE0 OK received — send APN config */
                    {
                        char apn_cmd[64];
                        snprintf(apn_cmd, sizeof(apn_cmd),
                                 "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", APN_STRING);
                        SIM7670_SendCommand(apn_cmd);
                        net_cfg_state = NET_CGDCONT;
                        SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    }
                    break;

                case NET_CGDCONT:
                    /* APN config OK received — enable light-sleep capability */
                    SIM7670_SendCommand("AT+CSCLK=1\r\n");
                    net_cfg_state = NET_CSCLK;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case NET_CSCLK:
                    /* AT+CSCLK=1 OK received — network config complete */
                    net_cfg_state = NET_DONE;
                    /* Fall through to NET_DONE intentionally */
                    /* FALLTHROUGH */
                case NET_DONE:
                    /* Assert DTR high to allow modem to enter light sleep */
                    SIM7670_Set_DTR(GPIO_PIN_SET);
                    sim_state = SIM_LIGHT_SLEEP;
                    DEBUG_PRINT("SIM7670: Network configured → LIGHT_SLEEP.\r\n");
                    break;
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_LIGHT_SLEEP:
        /* ------------------------------------------------------------------ */
            /* Modem is sleeping. DTR is HIGH. UART is inactive.
             * Woken by SIM7670_WakeFromSleep() (called by main.c on 60s timer). */
            break;

        /* ------------------------------------------------------------------ */
        case SIM_ACTIVE_TX:
        /* ------------------------------------------------------------------ */
            /* 50 ms settle guard after DTR wake (checked via state_timer).
             * After settle, dispatch the next HTTP command in sequence. */
            if ((now - state_timer) < DTR_WAKE_SETTLE_MS &&
                http_state == HTTP_INIT) {
                /* Still in DTR wake settle period — wait */
                break;
            }

            switch (http_state) {
                case HTTP_IDLE:
                    /* No transaction queued — this shouldn't happen */
                    sim_state = SIM_LIGHT_SLEEP;
                    break;

                case HTTP_INIT:
                    SIM7670_SendCommand("AT+HTTPINIT\r\n");
                    http_state = HTTP_URL;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case HTTP_URL:
                    /* tx_payload was built by SIM7670_Send_Data() */
                    SIM7670_SendCommand(tx_payload);
                    http_state = HTTP_ACTION;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case HTTP_ACTION:
                    SIM7670_SendCommand("AT+HTTPACTION=0\r\n"); /* 0 = HTTP GET */
                    http_state = HTTP_DONE;
                    SIM7670_AwaitResponse(HTTP_ACTION_TIMEOUT_MS); /* Network latency */
                    break;

                case HTTP_DONE:
                    /* +HTTPACTION:0,200 received — clean up HTTP session */
                    SIM7670_SendCommand("AT+HTTPTERM\r\n");
                    http_state = HTTP_IDLE;

                    /* Return modem to light sleep */
                    SIM7670_Set_DTR(GPIO_PIN_SET);
                    sim_state = SIM_LIGHT_SLEEP;

                    DEBUG_PRINT("SIM7670: HTTP GET complete — entering LIGHT_SLEEP.\r\n");
                    if (tx_cb) tx_cb(true);
                    break;
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_AWAITING_RESPONSE:
        /* ------------------------------------------------------------------ */
            /*
             * Non-blocking response wait. Fires every loop iteration.
             * If DMA has received data, parse it. If timeout expires, handle error.
             *
             * Priority: RX data before timeout.
             */
            if (sim_rx_ready) {
                sim_rx_ready = false;

                /* Safe null-termination — modem responses are always short (<256 B) */
                sim_rx_buffer[SIM_DMA_BUFFER_SIZE - 1] = '\0';
                DEBUG_PRINT("SIM RX: %s", (char *)sim_rx_buffer);

                /* Re-arm DMA immediately after consuming the buffer */
                HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);

                /* Response parsing -------------------------------------------
                 * Look for success indicators.
                 * AT+HTTPACTION success: "+HTTPACTION: 0,200"
                 * Generic command OK:   "OK"
                 */
                bool is_ok  = (strstr((char *)sim_rx_buffer, "OK")  != NULL);
                bool is_200 = (strstr((char *)sim_rx_buffer, "200") != NULL);
                bool is_err = (strstr((char *)sim_rx_buffer, "ERROR") != NULL);

                if (is_ok || is_200) {
                    /* Dispatch to the correct state based on where we came from.
                     * NETWORK_CONFIG and ACTIVE_TX both use AWAITING_RESPONSE. */
                    if (net_cfg_state != NET_DONE && http_state == HTTP_IDLE) {
                        /* We were in network config flow */
                        sim_state = SIM_NETWORK_CONFIG;
                    } else {
                        /* We were in the HTTP TX flow */
                        sim_state = SIM_ACTIVE_TX;
                    }
                } else if (is_err) {
                    DEBUG_PRINT("SIM7670: AT command ERROR received.\r\n");
                    http_state = HTTP_IDLE;
                    SIM7670_Set_DTR(GPIO_PIN_SET);
                    sim_state  = SIM_LIGHT_SLEEP;
                    if (tx_cb) tx_cb(false);
                }
                /* If neither OK nor ERROR (partial/empty response), stay in
                 * AWAITING_RESPONSE and wait for more data or timeout. */

            } else if ((now - state_timer) >= wait_timeout) {
                /* Timeout — modem did not respond in time */
                DEBUG_PRINT("SIM7670: AT response timeout (%lu ms).\r\n", wait_timeout);
                http_state = HTTP_IDLE;
                SIM7670_Set_DTR(GPIO_PIN_SET);
                sim_state  = SIM_LIGHT_SLEEP;

                /* Re-arm DMA in case it was not already running */
                HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);

                if (tx_cb) tx_cb(false);
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_BUSY:
        /* ------------------------------------------------------------------ */
            /* Reserved for future use — e.g., blocking concurrent TX requests.
             * Currently not entered by any state transition. Implement as needed
             * if multiple callers could request transmission simultaneously. */
            break;
    }
}