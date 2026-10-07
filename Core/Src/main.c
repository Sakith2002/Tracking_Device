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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "SIM7670.h"
#include "GPS.h"
#include <stdio.h>
#include <stdlib.h>
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
 * GPS FIFO for 10-second batching.
 */
#define FIFO_MAX_SIZE 6
static GPS_Data_t gps_fifo[FIFO_MAX_SIZE];
static uint8_t fifo_count = 0;
static uint32_t active_fifo_timer = 0;
#define FIFO_INTERVAL_MS 10000UL

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
    SIM7670_UART_RxCpltCallback(huart, Size);   /* Routes to SIM module if huart == SIM UART  */
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
        DEBUG_PRINT("SYS: motion->WAKE \r\n");
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
        DEBUG_PRINT("SYS: 5min stationary \r\n");
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
        DEBUG_PRINT("SYS: GPS fix \r\n");
        sys_state         = SYS_ACTIVE_TRANSIT;
        active_tx_timer   = HAL_GetTick();
        active_fifo_timer = HAL_GetTick();
        fifo_count        = 0;
        /* Start the stationary timer.  Normally the IMU interrupt starts it, but
         * on cold boot we transition directly here without going through WAKE_UP. */
        MPU6050_ForceActive();
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
        DEBUG_PRINT("SYS: SIM TX fail\r\n");
        /* TODO: Implement retry logic here if needed.
         *       For now, on failure during POWER_OFF flow, we still proceed
         *       with shutting down to avoid being stuck in an active state. */
    }

    /* If we were preparing to sleep (sys_state was set to SYS_POWER_OFF in
     * the SYS_SLEEP case), trigger the shutdown sequence. */
    if (sys_state == SYS_POWER_OFF) {
        DEBUG_PRINT("SYS: final TX done \r\n");
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  DBG_Init(&huart3);
  DBG_Print("SYS: boot\r\n");
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
  MPU6050_Config_Interrupt(&hi2c1, 20, 1);
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
              /* Waiting for GPS baseline fix. On_GPS_ValidFix() will advance
               * sys_state to SYS_ACTIVE_TRANSIT when the first valid fix arrives.
               * SIM boots and reaches LIGHT_SLEEP autonomously in the background. */
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

              sys_state         = SYS_ACTIVE_TRANSIT;
              active_tx_timer   = HAL_GetTick();
              active_fifo_timer = HAL_GetTick();
              fifo_count        = 0;
              /* IMU entered ACTIVE_HYSTERESIS via the interrupt path so the
               * stationary timer is already running; ForceActive is a no-op here
               * but guards against edge cases where state was lost. */
              MPU6050_ForceActive();
              DEBUG_PRINT("SYS: wake->ACTIVE \r\n");
              break;

          /* ---------------------------------------------------------------- */
          case SYS_ACTIVE_TRANSIT:
          /* ---------------------------------------------------------------- */
              /* Non-blocking 60-second location upload timer (Task 5 / Sequence 2).
               *
               * The MCU spends most of this state in __WFI() light sleep between
               * NMEA sentence parses (NEO-6M fires idle-line ISR every ~1s).
               * The SIM is in LIGHT_SLEEP between transmissions. */

              if ((HAL_GetTick() - active_fifo_timer) >= FIFO_INTERVAL_MS) {
                  active_fifo_timer = HAL_GetTick();
                  GPS_Data_t *gps_data = GPS_GetLatestData();
                  if (gps_data->has_fix) {
                      if (fifo_count < FIFO_MAX_SIZE) {
                          gps_fifo[fifo_count] = *gps_data;
                          fifo_count++;
                          DEBUG_PRINT("GPS: buf %d\r\n", fifo_count);
                      }
                  }
              }

              if ((HAL_GetTick() - active_tx_timer) >= TX_INTERVAL_MS) {
                  active_tx_timer = HAL_GetTick(); /* Reset 60s timer */

                  if (fifo_count > 0) {
                      /* Send the most recent fix to ThingSpeak (single-point API).
                       * Convert NMEA ddmm.mmmm format to decimal degrees. */
                      GPS_Data_t *p = &gps_fifo[fifo_count - 1];
                      float lat_n = atof(p->latitude);
                      int   la_d  = (int)(lat_n / 100);
                      float lat   = (float)la_d + (lat_n - (float)(la_d * 100)) / 60.0f;
                      if (p->lat_direction == 'S') lat = -lat;

                      float lon_n = atof(p->longitude);
                      int   lo_d  = (int)(lon_n / 100);
                      float lon   = (float)lo_d + (lon_n - (float)(lo_d * 100)) / 60.0f;
                      if (p->lon_direction == 'W') lon = -lon;

                      char payload[80];
                      snprintf(payload, sizeof(payload),
                               "field1=%.5f&field2=%.5f&field3=%.1f",
                               lat, lon, p->speed_kmh);

                      DEBUG_PRINT("SYS: TX %.5f,%.5f\r\n", lat, lon);
                      SIM7670_Send_Data(payload);
                      fifo_count = 0;
                  } else {
                      DEBUG_PRINT("SYS: TX skip \r\n");
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
                  DEBUG_PRINT("SYS: final TX\r\n");
                  sys_state = SYS_POWER_OFF; /* Guard before TX to prevent re-entry */
                  
                  GPS_Data_t *gps_data = GPS_GetLatestData();
                  if (gps_data->has_fix && fifo_count < FIFO_MAX_SIZE) {
                      gps_fifo[fifo_count] = *gps_data;
                      fifo_count++;
                  }
                  
                  if (fifo_count > 0) {
                      GPS_Data_t *p = &gps_fifo[fifo_count - 1];
                      float lat_n = atof(p->latitude);
                      int   la_d  = (int)(lat_n / 100);
                      float lat   = (float)la_d + (lat_n - (float)(la_d * 100)) / 60.0f;
                      if (p->lat_direction == 'S') lat = -lat;

                      float lon_n = atof(p->longitude);
                      int   lo_d  = (int)(lon_n / 100);
                      float lon   = (float)lo_d + (lon_n - (float)(lo_d * 100)) / 60.0f;
                      if (p->lon_direction == 'W') lon = -lon;

                      char payload[80];
                      snprintf(payload, sizeof(payload),
                               "field1=%.5f&field2=%.5f&field3=%.1f",
                               lat, lon, p->speed_kmh);
                      SIM7670_Send_Data(payload);
                      fifo_count = 0;
                  } else {
                      /* If somehow there's no data to send, go straight to sleep */
                      enter_stop_mode_pending = true;
                  }
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

                  DEBUG_PRINT("SYS: shutdown\r\n");

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
                  MPU6050_Config_Interrupt(&hi2c1, 20, 1);

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
                  DEBUG_PRINT("SYS: stop\r\n");

                  /* Abort UART DMA on both UARTs: idle-line events from the GPS going
                   * into backup mode or SIM powering off can leave a pending DMA
                   * interrupt flag that wakes WFI instantly. */
                  HAL_UART_AbortReceive(&huart1); /* SIM */
                  HAL_UART_AbortReceive(&huart2); /* GPS */

                  /* SysTick fires every 1 ms and is the most common spurious WFI wakeup.
                   * Suspend it here; HAL_Delay() is not used after this point. */
                  HAL_SuspendTick();

                  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

                  /* ---- EXECUTION RESUMES HERE AFTER MPU-6050 EXTI WAKE ---- */

                  HAL_ResumeTick();   /* Re-enable SysTick for HAL_GetTick() */
                  SystemClock_Config();

                  DEBUG_PRINT("SYS: woke\r\n");

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
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
