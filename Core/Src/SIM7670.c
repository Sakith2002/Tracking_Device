#include "SIM7670.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * SIM7670 (A7670C) Cellular Module Ã¢â‚¬â€ Internal Implementation
 * ============================================================================ */

#define SIM_DMA_BUFFER_SIZE     256     /**< UART DMA receive buffer size                      */
#define TX_PAYLOAD_SIZE         400     /**< HTTP URL string buffer (must fit full URL + AT cmd + batched coords) */

/* ---- Timing constants (from hardware spec) ------------------------------- */
#define BOOT_TIMEOUT_MS         11200UL /**< Ton(uart): time from PWRKEY pulse to UART ready    */
#define POWER_OFF_PULSE_MS      2500UL  /**< PWRKEY hold time to trigger modem power-off        */
#define REBOOT_BUFFER_MS        2000UL  /**< Mandatory gap between power-off and next power-on  */
#define DTR_WAKE_SETTLE_MS      100UL/**< Wait after DTRÃ¢â€ â€œ before sending first AT command    */
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
#define APN_STRING              "mobitel"

/* ---- HTTP GET endpoint --------------------------------------------------- */
/* TODO: Replace with your actual server URL.
 *       The lat/lon fields use raw NMEA format (ddmm.mmmm).
 *       Consider converting to decimal degrees server-side or in SIM7670_Send_Data_Batch().
 */
/* ThingSpeak free IoT cloud — replace YOUR_WRITE_API_KEY with the key from your channel */
#define THINGSPEAK_API_KEY  "MC800RUKXAHLWI7W"
#define SERVER_URL_BASE "AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/update?api_key=" THINGSPEAK_API_KEY "&"
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
/** Number of bytes written on the last DMA transfer (from HAL_UARTEx_RxEventCallback Size). */
static volatile uint16_t sim_rx_size  = 0;
static bool              atready_seen = false; /**< Set when *ATREADY URC seen; cleared after ATE0 sent */
static bool              probing      = false; /**< True while AT probe is in flight */

/**
 * @brief HTTP transaction sub-state machine.
 *
 * Tracks progress through the HTTPINIT Ã¢â€ â€™ HTTPPARA Ã¢â€ â€™ HTTPACTION Ã¢â€ â€™ HTTPTERM
 * sequence. Embedded inside SIM_ACTIVE_TX / SIM_AWAITING_RESPONSE.
 */
typedef enum {
    HTTP_IDLE,     /**< No transaction in progress                                    */
    HTTP_INIT,     /**< Next: send AT+HTTPINIT                                        */
    HTTP_CID,      /**< HTTPINIT OK; next: send AT+HTTPPARA="CID",1 (bind PDP ctx)   */
    HTTP_URL,      /**< CID OK; next: send AT+HTTPPARA URL                            */
    HTTP_ACTION,   /**< URL OK; next: send AT+HTTPACTION=0                            */
    HTTP_DONE      /**< +HTTPACTION:0,200 received; next: send AT+HTTPTERM            */
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
    NET_CGACT,       /**< Activate PDP context (AT+CGACT=1,1)                  */
    NET_CSCLK,       /**< Enable UART-woken sleep (AT+CSCLK=2)                 */
    NET_SIMINFO,     /**< Query IMSI (AT+CIMI) for debug                       */
    NET_CSQ,         /**< Query signal quality (AT+CSQ) for debug              */
    NET_DONE         /**< Config complete Ã¢â‚¬â€ transition to LIGHT_SLEEP           */
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
    DEBUG_PRINT("TX: %s", cmd);
    HAL_UART_Transmit(sim_huart, (uint8_t *)cmd, strlen(cmd), 1000);
}

/**
 * @brief Toggle the PWRKEY GPIO line for a specified duration.
 *
 * According to the A7670C datasheet:
 *   - Power ON  : Pull PWRKEY LOW for Ã¢â€°Â¥ 50 ms, then release HIGH.
 *   - Power OFF : Pull PWRKEY LOW for Ã¢â€°Â¥ 2500 ms, then release HIGH.
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
 * the MCU is about to enter Stop Mode immediately after Ã¢â‚¬â€ but note that if
 * the MPU fires during this window, the EXTI flag will be pending and the
 * system will re-evaluate on the next loop iteration after Stop Mode returns.
 *
 * For a fully non-blocking implementation, the PWRKEY pulse would need to
 * be driven by a hardware timer interrupt.
 *
 * @param duration_ms  Duration to hold PWRKEY low, in milliseconds.
 */
static void SIM7670_Pulse_PWRKEY(uint32_t duration_ms) {
    DEBUG_PRINT("SIM: PWRKEY %lu ms.\r\n", duration_ms);

    HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_SET);
    HAL_Delay(duration_ms);
    HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_RESET);
}


/* DTR not connected. Sleep/wake handled entirely via AT+CSCLK=2 (UART activity). */

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
    DEBUG_PRINT("SIM: init\r\n");
}

void SIM7670_RegisterCallback(SIM_TransmitCompleteCallback txCb) {
    tx_cb = txCb;
}

SIM_State_t SIM7670_GetState(void) {
    return sim_state;
}

void SIM7670_PowerOn(void) {
    if (sim_state != SIM_POWER_OFF) return;

    /* Probe first: if the module is still on from a previous session (e.g. code
     * re-flash without cycling power), AT\r\n will get OK back within ~200 ms.
     * Only pulse PWRKEY if there is no response (module genuinely off). */
    DEBUG_PRINT("SIM: probe\r\n");
    sim_rx_ready = false;
    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
    SIM7670_SendCommand("AT\r\n");

    probing      = true;
    sim_state    = SIM_BOOTING;
    state_timer  = HAL_GetTick();
    wait_timeout = 500; /* 500 ms probe window */
}

void SIM7670_PowerOff(void) {
    DEBUG_PRINT("SIM: power off\r\n");

    /* Mark state first so no new AT commands are dispatched in Process() */
    sim_state = SIM_POWER_OFF;

    /* 2.5 s blocking pulse. See note in SIM7670_Pulse_PWRKEY() about non-blocking approach. */
    SIM7670_Pulse_PWRKEY(POWER_OFF_PULSE_MS);

    /* After this returns, caller must wait Toff(uart) Ã¢â€°Ë† 1.9 s before Stop Mode.
     * This is handled in On_SIM_TransmitComplete() in main.c. */
}

void SIM7670_WakeFromSleep(void) {
    /* Only valid when modem is in LIGHT_SLEEP (AT+CSCLK=2 active) */
    if (sim_state != SIM_LIGHT_SLEEP) return;

    DEBUG_PRINT("SIM: wake\r\n");

    /* With AT+CSCLK=2, the modem wakes on any UART character.
     * Send a single CR byte — no AT response (echo disabled), purely triggers wake. */
    const uint8_t wake_byte = '\r';
    HAL_UART_Transmit(sim_huart, &wake_byte, 1, 100);

    /* Discard stale RX data and re-arm DMA.
     * AT+CSCLK=2 auto-sleep can trigger an idle-line event that stops the DMA.
     * Always re-arm here so the first HTTP response is captured. */
    sim_rx_ready = false;
    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);

    sim_state    = SIM_ACTIVE_TX;
    state_timer  = HAL_GetTick();
    wait_timeout = DTR_WAKE_SETTLE_MS;
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

    /* For all other states (BOOTING, NETWORK_CONFIG, ACTIVE_TX, AWAITING_RESPONSE):
     * http_state = HTTP_INIT is already set; SIM_ACTIVE_TX will pick it up when
     * the modem reaches that state naturally. No forced transition needed. */
}

void SIM7670_UART_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == sim_huart->Instance) {
        sim_rx_size  = Size;
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
            if (probing) {
                /* --- Probe phase: waiting up to 500 ms for AT\r\n → OK --- */
                if (sim_rx_ready) {
                    uint16_t sz = (sim_rx_size < SIM_DMA_BUFFER_SIZE)
                                  ? sim_rx_size : (SIM_DMA_BUFFER_SIZE - 1);
                    sim_rx_buffer[sz] = '\0';
                    sim_rx_ready = false;
                    if (strstr((char *)sim_rx_buffer, "OK")) {
                        /* Module alive (was in LIGHT_SLEEP or idle) — skip PWRKEY */
                        DEBUG_PRINT("SIM: alive\r\n");
                        probing = false;
                        HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                        net_cfg_state = NET_ATE0;
                        sim_state     = SIM_NETWORK_CONFIG;
                        SIM7670_SendCommand("ATE0\r\n");
                        SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                        break;
                    }
                    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                }
                if ((now - state_timer) >= wait_timeout) {
                    /* No response in 500 ms — module is off, pulse PWRKEY */
                    DEBUG_PRINT("SIM: power on\r\n");
                    probing      = false;
                    SIM7670_Pulse_PWRKEY(50);
                    state_timer  = now;
                    wait_timeout = BOOT_TIMEOUT_MS;
                }
            } else {
                /* --- Boot phase: waiting for *ATREADY or 11.2 s timeout --- */
                if (sim_rx_ready) {
                    uint16_t sz = (sim_rx_size < SIM_DMA_BUFFER_SIZE)
                                  ? sim_rx_size : (SIM_DMA_BUFFER_SIZE - 1);
                    sim_rx_buffer[sz] = '\0';
                    sim_rx_ready = false;
                    if (!atready_seen && strstr((char *)sim_rx_buffer, "ATREADY")) {
                        DEBUG_PRINT("SIM: ATREADY\r\n");
                        atready_seen = true;
                        state_timer  = now;
                        wait_timeout = 5000; /* 5 s drain: +CPIN/SMS/+CGEV URCs can
                                               arrive up to ~4 s after ATREADY */
                    }
                    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                }
                if ((now - state_timer) >= wait_timeout) {
                    DEBUG_PRINT("SIM: ATE0\r\n");
                    atready_seen = false;
                    sim_rx_ready = false;
                    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                    net_cfg_state = NET_ATE0;
                    sim_state     = SIM_NETWORK_CONFIG;
                    SIM7670_SendCommand("ATE0\r\n");
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                }
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
                    /* ATE0 OK received Ã¢â‚¬â€ send APN config */
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
                    /* APN set -- activate the PDP context so HTTP can use it */
                    SIM7670_SendCommand("AT+CGACT=1,1\r\n");
                    net_cfg_state = NET_CGACT;
                    SIM7670_AwaitResponse(10000UL); /* may take several seconds */
                    break;

                case NET_CGACT:
                    /* PDP context active -- enable auto-sleep */
                    SIM7670_SendCommand("AT+CSCLK=2\r\n");
                    net_cfg_state = NET_CSCLK;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case NET_CSCLK:
                    /* AT+CSCLK=2 OK -- query SIM IMSI for debug */
                    SIM7670_SendCommand("AT+CIMI\r\n");
                    net_cfg_state = NET_SIMINFO;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case NET_SIMINFO:
                    /* IMSI printed by RX debug -- query signal quality */
                    SIM7670_SendCommand("AT+CSQ\r\n");
                    net_cfg_state = NET_CSQ;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case NET_CSQ:
                    /* Signal quality printed by RX debug -- config done */
                    net_cfg_state = NET_DONE;
                    /* FALLTHROUGH */
                case NET_DONE:
                    sim_state = SIM_LIGHT_SLEEP;
                    DEBUG_PRINT("SIM: net OK -> LIGHT_SLEEP\r\n");
                    break;
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_LIGHT_SLEEP:
        /* ------------------------------------------------------------------ */
            /* Modem is sleeping (AT+CSCLK=2 auto-sleep after UART inactivity).
             * Woken by SIM7670_WakeFromSleep() → sends UART byte, 50 ms settle. */
            break;

        /* ------------------------------------------------------------------ */
        case SIM_ACTIVE_TX:
        /* ------------------------------------------------------------------ */
            /* 50 ms settle guard after UART wakeup byte (checked via state_timer).
             * After settle, dispatch the next HTTP command in sequence. */
            if ((now - state_timer) < DTR_WAKE_SETTLE_MS &&
                http_state == HTTP_INIT) {
                /* Still in DTR wake settle period Ã¢â‚¬â€ wait */
                break;
            }

            switch (http_state) {
                case HTTP_IDLE:
                    /* No transaction queued Ã¢â‚¬â€ this shouldn't happen */
                    sim_state = SIM_LIGHT_SLEEP;
                    break;

                case HTTP_INIT:
                    SIM7670_SendCommand("AT+HTTPINIT\r\n");
                    http_state = HTTP_URL;
                    SIM7670_AwaitResponse(CMD_DEFAULT_TIMEOUT_MS);
                    break;

                case HTTP_CID:
                    /* Bind HTTP session to PDP context 1 (configured by AT+CGDCONT). */
                    SIM7670_SendCommand("AT+HTTPPARA=\"CID\",1\r\n");
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
                    /* +HTTPACTION:0,200 received Ã¢â‚¬â€ clean up HTTP session */
                    SIM7670_SendCommand("AT+HTTPTERM\r\n");
                    http_state = HTTP_IDLE;

                    /* Modem auto-sleeps on UART inactivity (AT+CSCLK=2). */
                    sim_state = SIM_LIGHT_SLEEP;

                    DEBUG_PRINT("SIM: upload OK\r\n");
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

                /* Null-terminate at actual received length to prevent
                 * strstr scanning stale bytes from a prior transfer. */
                uint16_t sz = (sim_rx_size < SIM_DMA_BUFFER_SIZE)
                              ? sim_rx_size : (SIM_DMA_BUFFER_SIZE - 1);
                sim_rx_buffer[sz] = '\0';
                DEBUG_PRINT("RX: %s", (char *)sim_rx_buffer);

                /* Re-arm DMA immediately after consuming the buffer */
                HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);

                /* Response parsing -------------------------------------------
                 * Look for success indicators.
                 * AT+HTTPACTION success: "+HTTPACTION: 0,200"
                 * Generic command OK:   "OK"
                 */
                bool is_ok      = (strstr((char *)sim_rx_buffer, "OK")          != NULL);
                bool is_200     = (strstr((char *)sim_rx_buffer, "200")         != NULL);
                bool is_err     = (strstr((char *)sim_rx_buffer, "ERROR")       != NULL);
                bool is_httpact = (strstr((char *)sim_rx_buffer, "+HTTPACTION") != NULL);

                if (is_ok || is_200 || is_httpact) {
                    /* AT+HTTPACTION=0 returns OK immediately (command accepted) then
                     * a separate +HTTPACTION:0,nnn URC when the HTTP GET completes.
                     * http_state==HTTP_DONE means we just sent HTTPACTION=0; if we
                     * only see the plain OK, stay here and wait for the real URC. */
                    if (http_state == HTTP_DONE && !is_httpact && !is_200) {
                        /* Plain command-ACK OK — keep waiting for +HTTPACTION URC */
                    } else {
                        /* Dispatch to the correct state based on where we came from */
                        if (net_cfg_state != NET_DONE && http_state == HTTP_IDLE) {
                            sim_state = SIM_NETWORK_CONFIG;
                        } else {
                            sim_state = SIM_ACTIVE_TX;
                        }
                    }
                } else if (is_err) {
                    /* Network config errors are non-fatal: e.g. AT+CGACT may return
                     * ERROR if the LTE bearer auto-activated. Advance config state.
                     * HTTP TX errors clean up the session and return to sleep. */
                    if (net_cfg_state != NET_DONE && http_state == HTTP_IDLE) {
                        DEBUG_PRINT("SIM: cfg ERR, continuing\r\n");
                        sim_state = SIM_NETWORK_CONFIG;
                    } else {
                        DEBUG_PRINT("SIM: AT ERR\r\n");
                        if (http_state > HTTP_INIT) {
                            SIM7670_SendCommand("AT+HTTPTERM\r\n");
                        }
                        http_state = HTTP_IDLE;
                        sim_state  = SIM_LIGHT_SLEEP;
                        HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                        if (tx_cb) tx_cb(false);
                    }
                }
                /* If neither OK nor ERROR (partial/empty response), stay in
                 * AWAITING_RESPONSE and wait for more data or timeout. */

            } else if ((now - state_timer) >= wait_timeout) {
                DEBUG_PRINT("SIM: timeout (%lu ms).\r\n", wait_timeout);
                if (net_cfg_state != NET_DONE && http_state == HTTP_IDLE) {
                    /* Network config command timed out (e.g. ATE0 OK swallowed by
                     * a late URC). Non-fatal: advance the config sub-FSM anyway. */
                    DEBUG_PRINT("SIM: cfg timeout, continuing\r\n");
                    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                    sim_state = SIM_NETWORK_CONFIG;
                } else {
                    /* HTTP transaction timed out — clean up and report failure. */
                    if (http_state > HTTP_INIT) {
                        SIM7670_SendCommand("AT+HTTPTERM\r\n");
                    }
                    http_state = HTTP_IDLE;
                    sim_state  = SIM_LIGHT_SLEEP;
                    HAL_UARTEx_ReceiveToIdle_DMA(sim_huart, sim_rx_buffer, SIM_DMA_BUFFER_SIZE);
                    if (tx_cb) tx_cb(false);
                }
            }
            break;

        /* ------------------------------------------------------------------ */
        case SIM_BUSY:
        /* ------------------------------------------------------------------ */
            /* Reserved for future use Ã¢â‚¬â€ e.g., blocking concurrent TX requests.
             * Currently not entered by any state transition. Implement as needed
             * if multiple callers could request transmission simultaneously. */
            break;
    }
}
