#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ============================================================================
 * MPU-6050 IMU Module
 * ============================================================================
 * Manages the IMU state machine for motion detection and movement validation.
 *
 * State Machine:
 *   MPU_SLEEP_MONITOR      - IMU in hardware Low-Power Accelerometer mode.
 *                            MOT_THR set to high "wake" threshold.
 *                            INT pin fires when acceleration exceeds threshold
 *                            for MOT_DUR consecutive ms (hardware debounce).
 *                            STM32 is in Stop Mode; EXTI wakes it.
 *   MPU_MOVEMENT_VALIDATION- Called after EXTI wake. Samples XYZ at 100ms
 *                            intervals for 1.5 s. Computes dynamic acceleration
 *                            variance via an EMA low-pass filter. If variance
 *                            > engine-idle threshold, fires MotionDetectedCallback
 *                            and moves to MPU_ACTIVE_HYSTERESIS. Otherwise
 *                            re-arms interrupt and returns to MPU_SLEEP_MONITOR.
 *   MPU_ACTIVE_HYSTERESIS  - Vehicle is moving. MOT_THR lowered to sensitive
 *                            "driving" threshold (Schmitt-trigger hysteresis).
 *                            Monitors continuous vibration. GPS speed > 5 km/h
 *                            resets the stationary timer via
 *                            MPU6050_Reset_Stationary_Timer(). After 5 minutes
 *                            with no activity, fires StationaryDetectedCallback
 *                            and returns to MPU_SLEEP_MONITOR.
 *
 * Interrupt Architecture (from design spec):
 *   Highest Priority: MPU-6050 INT → EXTI Line → wakes MCU from Stop Mode.
 *   The INT pin latches HIGH until INT_STATUS is read via I2C (MPU6050_Clear_Interrupt).
 *   Must be cleared BEFORE re-entering Stop Mode, and AFTER movement validation.
 *
 * Inter-module communication:
 *   - Does NOT call any other module directly.
 *   - Exposes callbacks (registered by main.c) for validated motion events.
 *   - MPU6050_Trigger_Wakeup() called from HAL_GPIO_EXTI_Callback() in main.c.
 *   - MPU6050_Reset_Stationary_Timer() called from main.c On_GPS_SpeedExceeded().
 * ============================================================================ */

typedef enum {
    MPU_SLEEP_MONITOR = 0,
    MPU_MOVEMENT_VALIDATION,
    MPU_ACTIVE_HYSTERESIS
} MPU_State_t;

/* ---- Callback type definitions ------------------------------------------ */

/**
 * @brief Fired when movement is validated (variance > engine-idle threshold).
 *        Used by main.c to transition SYS_SLEEP/SYS_POWER_OFF → SYS_WAKE_UP.
 */
typedef void (*MPU_MotionDetectedCallback)(void);

/**
 * @brief Fired when 5-minute stationary timeout expires in ACTIVE_HYSTERESIS.
 *        Used by main.c to transition SYS_ACTIVE_TRANSIT → SYS_SLEEP.
 */
typedef void (*MPU_StationaryDetectedCallback)(void);

/* ---- Public API ---------------------------------------------------------- */

/**
 * @brief Initialise the MPU-6050.
 *
 * Reads WHO_AM_I register to verify device presence, clears PWR_MGMT_1
 * (wakes from reset sleep), and sets sample rate + range config.
 * Must be called once before any other MPU function (Task 1.3).
 *
 * @param hi2c  Pointer to the I2C handle connected to the MPU-6050.
 *              Configure at 400 kHz (Fast Mode) per design spec.
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Collect a calibration baseline for the accelerometer.
 *
 * Blocks for ~500 ms (50 samples × 10 ms). Averages static acceleration to
 * compute per-axis offset values (Accel_X/Y/Z_Offset). These are subtracted
 * during every subsequent MPU6050_Read_Accel() call, nulling out any mounting
 * bias. Z-axis offset accounts for 1g static gravity.
 *
 * Call once during cold boot while device is stationary (Task 1.3).
 *
 * @param hi2c  I2C handle.
 */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c);

/**
 * @brief Configure the MPU-6050 hardware motion interrupt.
 *
 * Writes MOT_THR (0x1F) and MOT_DUR (0x20) registers, then enables MOT_EN
 * bit in INT_ENABLE (0x38). The INT pin will assert HIGH when acceleration
 * exceeds 'threshold' LSBs for 'duration' consecutive milliseconds.
 *
 * Called with HIGH threshold during sleep arming (Task 6.2) and with LOW
 * threshold after movement is validated (Schmitt-trigger hysteresis, Task 4.2).
 *
 * @param hi2c       I2C handle.
 * @param threshold  Motion threshold in LSB units (1 LSB ≈ 2 mg at ±2g range).
 *                   Typical sleep threshold: 20. Typical driving threshold: 10.
 * @param duration   Hardware debounce duration in ms (1 LSB = 1 ms).
 */
void MPU6050_Config_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t threshold,
                               uint8_t duration);

/**
 * @brief Read calibrated accelerometer data.
 *
 * Reads 6 bytes from ACCEL_XOUT_H (0x3B), converts to g, and subtracts
 * the per-axis calibration offsets.
 *
 * @param hi2c  I2C handle.
 * @param Ax    Output: calibrated X-axis acceleration in g.
 * @param Ay    Output: calibrated Y-axis acceleration in g.
 * @param Az    Output: calibrated Z-axis acceleration in g (1.0 g when flat).
 */
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, float* Ax, float* Ay, float* Az);

/**
 * @brief Read raw gyroscope data.
 *
 * Reads 6 bytes from GYRO_XOUT_H (0x43), converts to °/s at ±250 °/s range.
 * Not used in the current state machine but available for future use.
 *
 * @param hi2c  I2C handle.
 * @param Gx    Output: X-axis angular rate in °/s.
 * @param Gy    Output: Y-axis angular rate in °/s.
 * @param Gz    Output: Z-axis angular rate in °/s.
 */
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, float* Gx, float* Gy, float* Gz);

/**
 * @brief Read and discard INT_STATUS register to clear the hardware interrupt latch.
 *
 * The MPU-6050 INT pin remains asserted HIGH until this register is read.
 * Must be called:
 *   1. At the END of Validate_Movement (after sampling completes, Task 2.2).
 *   2. Immediately BEFORE entering Stop Mode to prevent EXTI from firing
 *      instantly on wake (Task 6.2 — "re-arm the INT").
 *
 * @param hi2c  I2C handle.
 */
void MPU6050_Clear_Interrupt(I2C_HandleTypeDef *hi2c);

/**
 * @brief Register IMU event callbacks (called by main.c during init).
 *
 * @param motionCb     Called when valid movement is detected.
 * @param stationaryCb Called when 5-minute stationary timer expires.
 */
void MPU6050_RegisterCallbacks(MPU_MotionDetectedCallback motionCb,
                                MPU_StationaryDetectedCallback stationaryCb);

/**
 * @brief IMU state machine process function. Call every main loop iteration.
 *
 * In MPU_SLEEP_MONITOR:       Checks wakeup_triggered flag (set by EXTI).
 * In MPU_MOVEMENT_VALIDATION: Runs 15-sample validation at 100 ms intervals
 *                             (non-blocking; uses internal timer tracking).
 * In MPU_ACTIVE_HYSTERESIS:   Samples every 100 ms to detect ongoing vibration.
 *                             Checks 5-minute stationary timer.
 *
 * @param hi2c  I2C handle passed down from main.c (not stored permanently to
 *              avoid a circular dependency between modules).
 */
void MPU6050_Process(I2C_HandleTypeDef *hi2c);

/**
 * @brief Signal that the MPU interrupt fired (called from EXTI callback in main.c).
 *
 * Sets wakeup_triggered flag. Must only be called from HAL_GPIO_EXTI_Callback().
 * No I2C is performed here — ISR context must remain short.
 */
void MPU6050_Trigger_Wakeup(void);

/**
 * @brief Reset the 5-minute stationary timer (called from main.c on GPS speed event).
 *
 * Valid only in MPU_ACTIVE_HYSTERESIS state. Resets last_motion_tick to now,
 * preventing a false sleep entry while the vehicle is moving at a steady speed
 * on a smooth road (where vibration alone may not reset the timer).
 */
void MPU6050_Reset_Stationary_Timer(void);

#endif /* MPU6050_H */