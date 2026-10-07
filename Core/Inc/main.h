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
#include "debug.h"
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
    SYS_WAKE_UP,          /**< IMU validated motion. Waking GPS + SIM, then â†’ SYS_ACTIVE_TRANSIT.   */
    SYS_ACTIVE_TRANSIT,   /**< Vehicle is moving. 60s TX timer running. Monitoring for stationarity. */
    SYS_SLEEP             /**< 5-min stationary timer expired. Sends final TX, then â†’ SYS_POWER_OFF. */
} SYS_State_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* ============================================================================
 * GPIO Pin Assignments
 * ============================================================================
 * TODO: Confirm pin assignments with your schematic and update the defines
 *       below to match. After updating, implement the GPIO writes inside
 *       SIM7670_Pulse_PWRKEY() in SIM7670.c. (DTR not connected — no DTR pin needed.)
 * ============================================================================ */


/** SIM7670 PWRKEY Control Pin */
#define SIM_PWRKEY_GPIO_Port    PWRKEY_GPIO_Port
#define SIM_PWRKEY_Pin          PWRKEY_Pin

/* SIM7670 DTR: NOT CONNECTED. Sleep controlled via AT+CSCLK=2 (no pin needed).
 * SIM_DTR_GPIO_Port / SIM_DTR_Pin removed — do not use. */

/** MPU-6050 INT Pin (EXTI input, active-HIGH, latches until INT_STATUS is read) */
#define MPU_INT_GPIO_Port       GPIOB
#define MPU_INT_Pin             GPIO_PIN_8

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWRKEY_Pin GPIO_PIN_4
#define PWRKEY_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_8
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* Route all debug output to USART3 via DBG_Print().
 * To disable in production: change body to ((void)0). */
#define DEBUG_PRINT(...) DBG_Print(__VA_ARGS__)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
