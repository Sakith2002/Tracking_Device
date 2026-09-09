/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — System orchestration for GPS Tracking Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * ARCHITECTURE OVERVIEW
  * =====================
  * This file is the TOP-LEVEL ORCHESTRATOR. The three bottom-level state
  * machines (GPS.c, MPU6050.c, SIM7670.c) encapsulate their own hardware
  * and state. They communicate UPWARD only, via registered callbacks.
  * They DO NOT communicate with each other directly.
  *
  * Cross-module interactions flow through this file:
  *   GPS speed > 5 km/h  → calls MPU6050_Reset_Stationary_Timer()
  *   IMU motion detected  → sets sys_state = SYS_WAKE_UP (handled in main loop)
  *   IMU stationary       → sets sys_state = SYS_SLEEP
  *   GPS fix acquired     → sets sys_state = SYS_ACTIVE_TRANSIT
  *   SIM TX complete      → triggers MCU stop sequence
  *
  * Interrupt Priority (from design spec):
  *   Highest: MPU-6050 INT (EXTI) — wakes MCU from Stop Mode
  *   Medium:  NEO-6M UART DMA Idle Line — wakes from __WFI() to parse NMEA
  *   Lowest:  A7670C UART RX — signals URCs in AWAITING_RESPONSE state
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "SIM7670.h"
#include "GPS.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;  /**< USART1 — A7670C cellular modem (RTS/CTS flow control)  */
UART_HandleTypeDef huart2;  /**< USART2 — NEO-6M GPS module (no flow control)           */

/* USER CODE BEGIN PV */

/** Top-level system state — tracks the overall operating phase */
SYS_State_t sys_state = SYS_POWER_OFF;

/**
 * Timestamp (HAL_GetTick()) of the last 60-second transmission.
 * Used in SYS_ACTIVE_TRANSIT to fire periodic location uploads.
 */
static uint32_t active_tx_timer = 0;

/** 60-second data upload interval (Task 5, design spec Sequence 2) */
#define TX_INTERVAL_MS  60000UL

/**
 * Flag set by On_SIM_TransmitComplete when we need to enter Stop Mode.
 * The actual Stop Mode entry is deferred to the main loop (never from a callback)
 * to ensure clean stack state and pending interrupt handling.
 */
static volatile bool enter_stop_mode_pending = false;

/* Forward declarations of callbacks (defined in USER CODE 0 below) */
static void On_IMU_MotionDetected(void);
static void On_IMU_StationaryDetected(void);
static void On_GPS_ValidFix(void);
static void On_GPS_SpeedExceeded(void);
static void On_SIM_TransmitComplete(bool success);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* ============================================================================
 * HAL Callbacks — called from ISR context; MUST remain short
 * ============================================================================ */

/**
 * @brief EXTI interrupt callback (HAL override).
 *
 * Called when any configured EXTI line fires. Routes the MPU-6050 INT event
 * to MPU6050_Trigger_Wakeup() which sets a flag consumed by MPU6050_Process().
 *
 * NOTE: No I2C reads here — ISR context must be minimal. The INT_STATUS read
 * to clear the hardware latch happens inside MPU6050_Process() after validation.
 *
 * @param GPIO_Pin  The EXTI pin that triggered (bitmask).
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MPU_INT_Pin) {
        /* MPU-6050 motion threshold exceeded. Signal the process loop. */
        MPU6050_Trigger_Wakeup();
    }
}

/**
 * @brief UART RX event callback for DMA + idle-line reception (HAL override).
 *
 * Called when either:
 *   (a) DMA buffer is full, or
 *   (b) UART idle line detected (end of a burst, e.g. end of NMEA sentence).
 *
 * Routes to the correct module by passing the UART handle. Each module
 * internally checks huart->Instance to confirm it owns that peripheral.
 *
 * The 'Size' parameter is the actual byte count received — critical for
 * correct null-termination in the GPS parser.
 *
 * @param huart  The UART that fired the event.
 * @param Size   Number of bytes written to the DMA buffer.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    GPS_UART_IdleCallback(huart, Size);   /* Routes to GPS module if huart == GPS UART  */
    SIM7670_UART_RxCpltCallback(huart);   /* Routes to SIM module if huart == SIM UART  */
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * System-level Callbacks (registered with bottom-level modules)
 * ============================================================================
 * These functions are called by the module state machines when significant
 * hardware events occur. They update the top-level sys_state or cross-module
 * state, but NEVER block or call Stop Mode directly.
 * ============================================================================ */

/**
 * @brief Called by MPU6050_Process() when movement has been validated.
 *
 * Transitions from sleep/off states to SYS_WAKE_UP, which the main loop then
 * handles by waking GPS and SIM.
 */
static void On_IMU_MotionDetected(void) {
    if (sys_state == SYS_SLEEP || sys_state == SYS_POWER_OFF) {
        sys_state = SYS_WAKE_UP;
        DEBUG_PRINT("SYSTEM: IMU motion validated — transitioning to WAKE_UP.\r\n");
    }
}

/**
 * @brief Called by MPU6050_Process() when 5-minute stationary timer expires.
 *
 * Transitions from active tracking to sleep preparation.
 */
static void On_IMU_StationaryDetected(void) {
    if (sys_state == SYS_ACTIVE_TRANSIT) {
        sys_state = SYS_SLEEP;
        DEBUG_PRINT("SYSTEM: 5-min stationary — transitioning to SLEEP.\r\n");
    }
}

/**
 * @brief Called by GPS_Process() when the first valid $GPRMC fix is acquired.
 *
 * During cold boot, advances the system from waiting for baseline to active
 * tracking. Starts the 60-second TX timer.
 */
static void On_GPS_ValidFix(void) {
    if (sys_state == SYS_COLD_BOOT) {
        DEBUG_PRINT("SYSTEM: GPS baseline acquired — entering ACTIVE_TRANSIT.\r\n");
        sys_state       = SYS_ACTIVE_TRANSIT;
        active_tx_timer = HAL_GetTick();
    }
}

/**
 * @brief Called by GPS_Process() when GPS speed exceeds 5 km/h.
 *
 * Resets the IMU's 5-minute stationary timer to prevent false sleep entry
 * during smooth highway driving (Schmitt-trigger hysteresis, Task 4.1).
 */
static void On_GPS_SpeedExceeded(void) {
    MPU6050_Reset_Stationary_Timer();
}

/**
 * @brief Called by SIM7670_Process() when an HTTP transaction completes or fails.
 *
 * If we were in SYS_POWER_OFF (final TX after stationary), sets the
 * enter_stop_mode_pending flag. The actual Stop Mode entry happens in the
 * main loop to ensure clean stack state.
 *
 * @param success  true if HTTP 200 received, false on error or timeout.
 */
static void On_SIM_TransmitComplete(bool success) {
    if (!success) {
        DEBUG_PRINT("SYSTEM: SIM TX failed (error or timeout).\r\n");
        /* TODO: Implement retry logic here if needed.
         *       For now, on failure during POWER_OFF flow, we still proceed
         *       with shutting down to avoid being stuck in an active state. */
    }

    /* If we were preparing to sleep (sys_state was set to SYS_POWER_OFF in
     * the SYS_SLEEP case), trigger the shutdown sequence. */
    if (sys_state == SYS_POWER_OFF) {
        DEBUG_PRINT("SYSTEM: Final TX done — scheduling Stop Mode entry.\r\n");
        /* Set flag; actual Stop Mode entry deferred to main loop (not ISR/callback) */
        enter_stop_mode_pending = true;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* ========================================================================
   * Task 1: Cold Boot / System Init
   * ========================================================================
   * Order matters:
   *   1. IMU must be ready first (to detect motion before anything else).
   *   2. GPS initialized to start acquiring satellite lock in parallel.
   *   3. SIM powered on last (highest power draw, needs GPS baseline first).
   * ======================================================================== */
  sys_state = SYS_COLD_BOOT;

  /* 1.3 — MPU-6050 Setup */
  MPU6050_Init(&hi2c1);
  MPU6050_Calibrate(&hi2c1);
  MPU6050_Config_Interrupt(&hi2c1, 20, 1); /* High wake threshold: THR=20 LSB, DUR=1 ms */
  MPU6050_RegisterCallbacks(On_IMU_MotionDetected, On_IMU_StationaryDetected);

  /* 1.4 — NEO-6M GPS: power on and wait for baseline in ACQUIRING_BASELINE state */
  GPS_Init(&huart2);
  GPS_RegisterCallbacks(On_GPS_ValidFix, On_GPS_SpeedExceeded);
  GPS_SetState(GPS_ACQUIRING_BASELINE);

  /* 1.5 — A7670C Cellular: boot and configure APN asynchronously
   * The SIM state machine will sequence through BOOTING → NETWORK_CONFIG →
   * LIGHT_SLEEP over the next ~11.2 seconds without blocking. */
  SIM7670_Init(&huart1);
  SIM7670_RegisterCallback(On_SIM_TransmitComplete);
  SIM7670_PowerOn();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ====================================================================
       * Bottom-level state machine tick — process each module every iteration.
       * These are non-blocking and return quickly in most states.
       * ==================================================================== */
      MPU6050_Process(&hi2c1);
      GPS_Process();
      SIM7670_Process();

      /* ====================================================================
       * Top-level system orchestration — acts on state transitions signalled
       * by module callbacks. Each case should set sys_state and call module
       * functions as needed, then break.
       * ==================================================================== */
      switch (sys_state) {

          /* ---------------------------------------------------------------- */
          case SYS_COLD_BOOT:
          /* ---------------------------------------------------------------- */
              /* Waiting for GPS_ACQUIRING_BASELINE → GPS_CONTINUOUS_TRACKING.
               * On_GPS_ValidFix() callback will advance sys_state to
               * SYS_ACTIVE_TRANSIT when the first valid fix is received.
               * Meanwhile, SIM is booting and will enter LIGHT_SLEEP autonomously.
               *
               * Low-power hint: __WFI() here would save power while waiting.
               * GPS DMA+idle ISR and SIM HAL_GetTick() timer will still advance. */
              __WFI();
              break;

          /* ---------------------------------------------------------------- */
          case SYS_WAKE_UP:
          /* ---------------------------------------------------------------- */
              /* IMU validated motion (On_IMU_MotionDetected set this state).
               * Wake GPS and SIM, then transition to active tracking.
               *
               * Design spec Task 2 sequence:
               *   T=0ms: EXTI fired, MPU woke MCU
               *   T=0ms: Wake GPS (done here)
               *   T=0ms: SIM boot pulse was already sent at T=0ms in spec;
               *           if SIM was in POWER_OFF, call PowerOn().
               *           if SIM was in LIGHT_SLEEP, call WakeFromSleep() +
               *           then Send_Data() will be triggered on 60s timer. */
              GPS_SetState(GPS_CONTINUOUS_TRACKING);

              /* Wake SIM: if it was powered off (came from Stop Mode), power it on.
               * If it was in light sleep (e.g. re-wake within same session), just wake. */
              if (SIM7670_GetState() == SIM_POWER_OFF) {
                  SIM7670_PowerOn(); /* Will boot for 11.2 s non-blocking */
              } else if (SIM7670_GetState() == SIM_LIGHT_SLEEP) {
                  /* Don't call WakeFromSleep here — it will be called by
                   * SIM7670_Send_Data() when the 60s timer fires below. */
              }

              sys_state       = SYS_ACTIVE_TRANSIT;
              active_tx_timer = HAL_GetTick(); /* Start 60s transmission timer */
              DEBUG_PRINT("SYSTEM: WAKE_UP complete — entering ACTIVE_TRANSIT.\r\n");
              break;

          /* ---------------------------------------------------------------- */
          case SYS_ACTIVE_TRANSIT:
          /* ---------------------------------------------------------------- */
              /* Non-blocking 60-second location upload timer (Task 5 / Sequence 2).
               *
               * The MCU spends most of this state in __WFI() light sleep between
               * NMEA sentence parses (NEO-6M fires idle-line ISR every ~1s).
               * The SIM is in LIGHT_SLEEP between transmissions. */

              if ((HAL_GetTick() - active_tx_timer) >= TX_INTERVAL_MS) {
                  active_tx_timer = HAL_GetTick(); /* Reset 60s timer */

                  GPS_Data_t *gps_data = GPS_GetLatestData();
                  if (gps_data->has_fix) {
                      DEBUG_PRINT("SYSTEM: 60s TX — uploading location data.\r\n");
                      /* SIM7670_Send_Data() will wake the modem from light sleep if
                       * needed, then execute the HTTP GET asynchronously. */
                      SIM7670_Send_Data(gps_data->time,
                                        gps_data->latitude,  gps_data->lat_direction,
                                        gps_data->longitude, gps_data->lon_direction);
                  } else {
                      DEBUG_PRINT("SYSTEM: 60s TX skipped — no GPS fix.\r\n");
                  }
              }

              /* Light sleep between NMEA parses (woken by UART idle line ISR every ~1s).
               * __WFI() here is safe: SysTick, UART DMA, and EXTI all still fire. */
              __WFI();
              break;

          /* ---------------------------------------------------------------- */
          case SYS_SLEEP:
          /* ---------------------------------------------------------------- */
              /* 5-min stationary timer expired (On_IMU_StationaryDetected set this).
               * Design spec Task 6: send final location, then power everything down.
               *
               * We set sys_state to SYS_POWER_OFF BEFORE calling Send_Data().
               * This prevents re-entering this block on the next iteration while the
               * SIM is executing the HTTP transaction. On_SIM_TransmitComplete() will
               * see sys_state == SYS_POWER_OFF and set enter_stop_mode_pending. */
              {
                  GPS_Data_t *gps_data = GPS_GetLatestData();
                  DEBUG_PRINT("SYSTEM: SLEEP — sending final parked location.\r\n");
                  sys_state = SYS_POWER_OFF; /* Guard before TX to prevent re-entry */
                  SIM7670_Send_Data(gps_data->time,
                                    gps_data->latitude,  gps_data->lat_direction,
                                    gps_data->longitude, gps_data->lon_direction);
              }
              break;

          /* ---------------------------------------------------------------- */
          case SYS_POWER_OFF:
          /* ---------------------------------------------------------------- */
              /* Either:
               *   (a) Waiting for the final SIM TX to complete (enter_stop_mode_pending = false),
               *   (b) Stop Mode entry was triggered (enter_stop_mode_pending = true), or
               *   (c) Already returned from Stop Mode after a wake-up.
               *
               * Stop Mode entry is done HERE (main loop body) rather than in the
               * callback, to ensure a clean stack and proper pending-interrupt handling. */

              if (enter_stop_mode_pending) {
                  enter_stop_mode_pending = false;

                  DEBUG_PRINT("SYSTEM: Shutting down modules and entering Stop Mode.\r\n");

                  /* Task 6.4: Send GPS to Power Save Mode */
                  GPS_SetState(GPS_POWER_SAVE);

                  /* Task 6.3: Power off cellular module (2.5 s PWRKEY pulse).
                   * NOTE: SIM7670_PowerOff() currently contains a stub for the GPIO.
                   * Once GPIO is wired, this will physically power down the modem.
                   * After the pulse, Toff(uart) = 1.9 s must elapse before Stop Mode. */
                  SIM7670_PowerOff();

                  /* Wait for modem UART to fully go silent (Toff(uart) = 1.9 s).
                   * HAL_Delay here is acceptable because:
                   *   - We are about to enter Stop Mode anyway.
                   *   - Only EXTI (MPU wake) would abort this, and EXTI is still active.
                   *   - If MPU fires during this delay, wakeup_triggered will be set
                   *     and the state machine will handle it after Stop Mode returns. */
                  HAL_Delay(2000);

                  /* Task 6.2: Clear MPU INT latch and re-arm EXTI.
                   * This must happen AFTER the I2C peripherals are still active
                   * (before Stop Mode which freezes the bus clocks). */
                  MPU6050_Clear_Interrupt(&hi2c1);

                  /* Configure MPU for high-threshold sleep monitoring
                   * (was lowered to driving threshold in ACTIVE_HYSTERESIS). */
                  MPU6050_Config_Interrupt(&hi2c1, 20, 1); /* High wake threshold */

                  /* Task 6.5: Enter Stop Mode.
                   * PWR_MAINREGULATOR_ON: keeps main voltage regulator on for faster wake.
                   * PWR_STOPENTRY_WFI:    wake on any interrupt (EXTI for MPU-6050 INT).
                   *
                   * The system will BLOCK here until an interrupt fires.
                   * The MPU-6050 INT pin (EXTI) will wake the MCU when motion is detected.
                   * HSI is used (not PLL), so no PLL restart is needed after Stop Mode.
                   *
                   * TODO: Consider PWR_LOWPOWERREGULATOR_ON for lower Stop Mode current
                   *       (~14 µA vs ~20 µA) if 5.4 µs extra wake latency is acceptable.
                   */
                  DEBUG_PRINT("SYSTEM: Entering Stop Mode. Goodbye.\r\n");
                  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

                  /* ---- EXECUTION RESUMES HERE AFTER WAKE FROM STOP MODE ---- */

                  /* Re-initialize the system clock (HSI is restored automatically,
                   * but if you switch to PLL-based 72 MHz later, call SystemClock_Config()
                   * here to restart the PLL after Stop Mode. With HSI-only config
                   * (current setup at 8 MHz), this call is a no-op but kept for safety). */
                  SystemClock_Config();

                  DEBUG_PRINT("SYSTEM: Woke from Stop Mode. Resuming.\r\n");

                  /* On_IMU_MotionDetected will have set sys_state = SYS_WAKE_UP
                   * via the EXTI → MPU6050_Trigger_Wakeup() → MPU6050_Process() path.
                   * The main loop will handle SYS_WAKE_UP on the next iteration. */
              }

              /* While waiting for final TX (enter_stop_mode_pending still false),
               * light sleep to save power. SIM7670_Process() will advance the HTTP
               * state machine when woken by UART idle-line interrupt. */
              __WFI();
              break;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *
  * Configured for HSI at 8 MHz (no PLL) for simplicity.
  *
  * NOTE: The STM32F103 Bluepill is typically run at 72 MHz via PLL (HSI→PLL×9).
  * This current config runs at 8 MHz. To switch to 72 MHz:
  *   - Set PLL source to HSI/2 (4 MHz) with multiplier ×18 = 72 MHz... actually
  *     HSI direct with PLLMUL=9 is not typical; HSE (8 MHz crystal) × 9 = 72 MHz.
  *   - If using the on-board 8 MHz HSE crystal, switch OscillatorType to HSE
  *     and set PLLState = RCC_PLL_ON with PLLSource = HSE and PLLMUL = 9.
  *   - After Stop Mode, call SystemClock_Config() again to restart PLL.
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  *
  * Connects to the MPU-6050. Configured at 400 kHz (Fast Mode) per design spec:
  * "I2C1 Transaction (MPU-6050): ~100 µs per byte at 400 kHz Fast Mode."
  *
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;           /* 400 kHz Fast Mode (design spec) */
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  *
  * Connected to the A7670C cellular modem.
  * Hardware flow control (RTS/CTS) is enabled per design spec (Task 1.2).
  *
  * TODO: Verify baud rate matches A7670C default (typically 115200 bps auto-detects).
  *       Some configurations may require 9600 bps initially.
  *
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;  /* Required for A7670C */
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  *
  * Connected to the NEO-6M GPS module.
  * No hardware flow control (NMEA is one-directional, GPS TX → STM32 RX).
  *
  * NOTE: NEO-6M default baud is 9600 bps. This is set to 115200 here.
  *       If the GPS outputs garbled data, configure the NEO-6M to 115200 first
  *       using u-center, or temporarily change this to 9600.
  *
  * TODO: Confirm the NEO-6M baud rate configured on your module.
  *
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;              /* TODO: Change to 115200 if NEO-6M is pre-configured */
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  *
  * Configures:
  *   PB0 (SIM_PWRKEY_Pin) — Output PP, initially LOW (PWRKEY not asserted)
  *   PB1 (SIM_DTR_Pin)    — Output PP, initially HIGH (modem in sleep state)
  *   PB8 (MPU_INT_Pin)    — Input with EXTI, rising-edge triggered (MPU-6050 INT)
  *
  * TODO: These pin assignments are placeholders. Update SIM_PWRKEY_Pin,
  *       SIM_DTR_Pin, and MPU_INT_Pin in main.h to match your schematic.
  *       Add or remove GPIO port clock enables as needed.
  *
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* -------------------------------------------------------------------------
   * Output pins: SIM PWRKEY and DTR
   *
   * PWRKEY: default LOW (not pulsed; pulsing happens in SIM7670_Pulse_PWRKEY())
   * DTR:    default HIGH (modem is in sleep state at power-up)
   * ------------------------------------------------------------------------- */
  HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, GPIO_PIN_RESET); /* PWRKEY idle low */
  HAL_GPIO_WritePin(SIM_DTR_GPIO_Port,    SIM_DTR_Pin,    GPIO_PIN_SET);   /* DTR idle high   */

  /* Configure SIM_PWRKEY_Pin (PB0) as output */
  GPIO_InitStruct.Pin   = SIM_PWRKEY_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_PWRKEY_GPIO_Port, &GPIO_InitStruct);

  /* Configure SIM_DTR_Pin (PB1) as output */
  GPIO_InitStruct.Pin   = SIM_DTR_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_DTR_GPIO_Port, &GPIO_InitStruct);

  /* -------------------------------------------------------------------------
   * Input pin: MPU-6050 INT (PB8) — EXTI, rising edge, internal pull-down
   *
   * The MPU-6050 INT pin is active-HIGH and open-drain by default.
   * A pull-down resistor ensures the line reads LOW when no interrupt is active.
   *
   * IMPORTANT: The EXTI line for PB8 is EXTI8, which shares IRQ EXTI9_5_IRQn
   * with EXTI5–EXTI9. Ensure EXTI9_5_IRQHandler is enabled in stm32f1xx_it.c
   * and calls HAL_GPIO_EXTI_IRQHandler(MPU_INT_Pin).
   *
   * TODO: After verifying the pin, enable the NVIC for EXTI9_5_IRQn in the
   *       .ioc file (STM32CubeMX) or add the NVIC_EnableIRQ call here:
   *
   *   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);  // Highest priority
   *   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
   *
   * This is the CRITICAL fix for the system to ever wake from Stop Mode.
   * Without EXTI configured, the MPU interrupt will never fire.
   * ------------------------------------------------------------------------- */
  GPIO_InitStruct.Pin   = MPU_INT_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;  /* Rising edge = INT asserted HIGH */
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;         /* Pull-down to prevent false triggers */
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* Enable EXTI9_5 IRQ for MPU-6050 INT on PB8.
   * Priority 0 (highest) — this interrupt wakes the MCU from Stop Mode.
   * See design spec: "Highest Priority (Preemptive): MPU-6050 INT (EXTI Line)" */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  DEBUG_PRINT("ASSERT FAIL: %s line %lu\r\n", file, (unsigned long)line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
