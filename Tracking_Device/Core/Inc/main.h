/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
 * @brief Top-level system state machine.
 *
 * Orchestrated in main.c. The modules (GPS, IMU, SIM) each have their own
 * internal state machines; this enum controls the high-level system behaviour
 * and transitions between them via callbacks.
 */
typedef enum {
    SYS_POWER_OFF = 0,    /**< SIM powered off, GPS in PSM, MCU in Stop Mode. Waiting for MPU EXTI. */
    SYS_COLD_BOOT,        /**< First power-on. Waiting for GPS baseline fix before going active.     */
    SYS_WAKE_UP,          /**< IMU validated motion. Waking GPS + SIM, then → SYS_ACTIVE_TRANSIT.   */
    SYS_ACTIVE_TRANSIT,   /**< Vehicle is moving. 60s TX timer running. Monitoring for stationarity. */
    SYS_SLEEP             /**< 5-min stationary timer expired. Sends final TX, then → SYS_POWER_OFF. */
} SYS_State_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* ============================================================================
 * GPIO Pin Assignments
 * ============================================================================
 * TODO: Confirm pin assignments with your schematic and update the defines
 *       below to match. After updating, implement the GPIO writes inside
 *       SIM7670_Pulse_PWRKEY() and SIM7670_Set_DTR() in SIM7670.c.
 * ============================================================================ */

/** SIM7670 PWRKEY Control Pin (drives MOSFET gate or direct PWRKEY) */
#define SIM_PWRKEY_GPIO_Port    GPIOB
#define SIM_PWRKEY_Pin          GPIO_PIN_0   /* TODO: Verify pin — currently placeholder PB0 */

/** SIM7670 DTR Pin (pull LOW to wake from AT+CSCLK=1 light-sleep; HIGH to sleep) */
#define SIM_DTR_GPIO_Port       GPIOB
#define SIM_DTR_Pin             GPIO_PIN_1   /* TODO: Verify pin — currently placeholder PB1 */

/** MPU-6050 INT Pin (EXTI input, active-HIGH, latches until INT_STATUS is read) */
#define MPU_INT_GPIO_Port       GPIOB
#define MPU_INT_Pin             GPIO_PIN_8   /* TODO: Verify pin — currently placeholder PB8 */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/**
 * @brief Debug print macro.
 *
 * Maps to printf() which requires _write() syscall retargeting.
 * For hardware debugging, route _write() in syscalls.c to:
 *   - SWO/ITM via ITM_SendChar() for SWD debugger output, OR
 *   - A dedicated debug UART (e.g., UART3 if available).
 *
 * To disable all debug output in production, define NDEBUG before including
 * this header, or replace the macro body with ((void)0).
 */
#define DEBUG_PRINT(...) printf(__VA_ARGS__)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
