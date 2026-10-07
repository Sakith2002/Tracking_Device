#include "MPU6050.h"
#include "main.h"
#include <math.h>

/* ============================================================================
 * MPU-6050 IMU Module — Internal Implementation
 * ============================================================================ */

/* ---- MPU-6050 Register Map (relevant subset) ----------------------------- */
#define MPU6050_I2C_ADDR        0xD0    /**< 7-bit addr 0x68, shifted left for HAL = 0xD0 */
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A    /**< DLPF_CFG lives here (bits [2:0])              */
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_MOT_THR             0x1F    /**< Motion threshold (1 LSB ≈ 2 mg at ±2g)       */
#define REG_MOT_DUR             0x20    /**< Motion duration  (1 LSB = 1 ms)               */
#define REG_ACCEL_XOUT_H        0x3B
#define REG_INT_ENABLE          0x38
#define REG_INT_STATUS          0x3A
#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75
#define GYRO_XOUT_H_REG         0x43

#define MPU6050_WHO_AM_I_VAL    104     /**< Expected WHO_AM_I response                    */

/* ---- Threshold / timing constants --------------------------------------- */
/**
 * Variance threshold for movement validation (dimensionless, in g²).
 * If the variance of the dynamic acceleration magnitude over 15 samples
 * exceeds this value, the motion is considered real vehicle movement.
 * Tune empirically: engine idle typically < 0.005 g², driving > 0.005 g².
 */
#define VALIDATION_VARIANCE_THRESHOLD   0.005f

/**
 * Low threshold for dynamic acceleration in ACTIVE_HYSTERESIS (in g).
 * Any reading above this resets the stationary timer (Schmitt-trigger
 * lower bound). Set lower than the validation threshold to avoid bouncing.
 */
#define HYSTERESIS_ACCEL_THRESHOLD      0.05f

/** 5-minute stationary timer (ms). Must match design spec Task 4.3. */
#define STATIONARY_TIMEOUT_MS           120000UL  /* 2 min for testing (restore to 300000UL for production) */

/** Sampling interval for ACTIVE_HYSTERESIS monitoring (ms). */
#define HYSTERESIS_SAMPLE_INTERVAL_MS   100

/**
 * Movement validation parameters (Task 2.2 / design spec).
 * Spec: "Sample XYZ every 100ms for 1.5s"
 * → 15 samples, 100 ms apart = 1500 ms total.
 */
#define VALIDATION_SAMPLES              15
#define VALIDATION_INTERVAL_MS          100

/** EMA smoothing factor for validation (0 = no smoothing, 1 = no update) */
#define EMA_ALPHA                       0.3f

/** MPU6050 interrupt enable: bit 6 = MOT_EN (motion detection) */
#define INT_ENABLE_MOT_EN               0x40

/* ---- Module-private state ------------------------------------------------ */
static bool        mpu_ok    = false; /**< Set by Init(); guards all other functions */
static MPU_State_t mpu_state = MPU_SLEEP_MONITOR;

static MPU_MotionDetectedCallback    motion_cb    = NULL;
static MPU_StationaryDetectedCallback stationary_cb = NULL;

/** Calibration offsets subtracted in every MPU6050_Read_Accel() call */
static float Accel_X_Offset = 0.0f;
static float Accel_Y_Offset = 0.0f;
static float Accel_Z_Offset = 0.0f;   /**< Z is zeroed relative to 1g (gravity) */

/** Tick at which the last motion evidence was recorded (for stationary timer) */
static uint32_t last_motion_tick = 0;

/** Set from HAL_GPIO_EXTI_Callback (ISR context). Consumed in MPU6050_Process(). */
static volatile bool wakeup_triggered = false;

/* ---- Validation sub-state (used only during MPU_MOVEMENT_VALIDATION) ---- */
static uint32_t val_last_sample_tick = 0;   /**< Tick of last sample during validation */
static int      val_sample_count     = 0;   /**< Number of samples collected so far    */
static float    val_sum_mag          = 0.0f;
static float    val_sum_sq_mag       = 0.0f;
static float    val_filtered_mag     = 0.0f;/**< EMA accumulator, seeded on first sample */

/* ---- Hysteresis sub-state ----------------------------------------------- */
static uint32_t hyst_last_sample_tick = 0;  /**< Tick of last sample in hysteresis mode */

/* ============================================================================
 * Private helpers
 * ============================================================================ */

/**
 * @brief Write one byte to an MPU-6050 register.
 */
static inline void MPU_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/**
 * @brief Read one byte from an MPU-6050 register.
 */
static inline uint8_t MPU_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

/**
 * @brief Reset validation accumulators and start the sample timer.
 *        Call this when entering MPU_MOVEMENT_VALIDATION.
 */
static void Validation_Reset(void) {
    val_sample_count     = 0;
    val_sum_mag          = 0.0f;
    val_sum_sq_mag       = 0.0f;
    val_filtered_mag     = 0.0f;        /* Will be seeded on first sample */
    val_last_sample_tick = HAL_GetTick();
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    /* MPU6050 power-on startup: wait for VDD and oscillator to settle.
     * STM32 boots faster than the MPU6050 on a cold power-on. */
    HAL_Delay(100);

    /* Retry WHO_AM_I up to 3 times — first I2C transaction can fail if the bus
     * capacitance hasn't settled or the device is still completing its POR. */
    uint8_t who_am_i = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        who_am_i = MPU_ReadReg(hi2c, REG_WHO_AM_I);
        if (who_am_i == MPU6050_WHO_AM_I_VAL) break;
        HAL_Delay(50);
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        DEBUG_PRINT("MPU: WHO_AM_I=0x%02X after 3 attempts. Check wiring.\r\n", who_am_i);
        mpu_ok = false;
        return;
    }

    /* Wake the chip: clear SLEEP bit in PWR_MGMT_1 */
    MPU_WriteReg(hi2c, REG_PWR_MGMT_1, 0x00);

    /* Give the internal clock time to stabilise before writing config registers.
     * Without this delay, ACCEL_HPF writes can be silently lost on cold boot. */
    HAL_Delay(10);

    /* Sample rate divider: SMPLRT_DIV=0x07 → Fs = Gyro_Rate / (1+7) = 125 Hz
     * (internal gyro rate is 1 kHz when DLPF enabled). Good balance of
     * responsiveness vs. power. */
    MPU_WriteReg(hi2c, REG_SMPLRT_DIV,  0x07);

    /* DLPF: DLPF_CFG=3 → accel bandwidth 44 Hz, gyro 42 Hz.
     * Attenuates vibration noise above ~50 Hz while keeping vehicle motion intact. */
    MPU_WriteReg(hi2c, REG_CONFIG,       0x03);

    /* Accelerometer ±2g range + 5 Hz digital high-pass filter (ACCEL_HPF = 0x01).
     * The HPF feeds the motion detector; without it the MOT_THR comparator input
     * is always zero and INT never fires. */
    MPU_WriteReg(hi2c, REG_ACCEL_CONFIG, 0x01);

    /* Gyro ±250 °/s range */
    MPU_WriteReg(hi2c, REG_GYRO_CONFIG,  0x00);

    /* Latch the INT pin HIGH until INT_STATUS is read.
     * Default 50 µs pulse can be missed in Stop Mode; latching ensures the EXTI
     * edge is still visible when the MCU wakes. */
    MPU_WriteReg(hi2c, 0x37, 0x20);

    mpu_ok = true;
    DEBUG_PRINT("MPU: OK\r\n");
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c) {
    if (!mpu_ok) return;
    DEBUG_PRINT("MPU: calibrating...\r\n");

    const int CAL_SAMPLES = 150; /* 150 × 10 ms = 1.5 s — better steady-state average */
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    float ax, ay, az;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        MPU6050_Read_Accel(hi2c, &ax, &ay, &az);
        sumX += ax;
        sumY += ay;
        sumZ += az;
        HAL_Delay(10); /* 10 ms between samples → 500 ms total blocking */
    }

    /* Compute per-axis offsets. Z is offset relative to +1g (static gravity). */
    Accel_X_Offset = sumX / CAL_SAMPLES;
    Accel_Y_Offset = sumY / CAL_SAMPLES;
    Accel_Z_Offset = (sumZ / CAL_SAMPLES) - 1.0f;

    /* Sanity check: calibrated magnitude should be ~1.0 g.
     * If far off, the device moved during calibration — re-flash/reset while stationary. */
    float cx = 0, cy = 0, cz = 0;
    MPU6050_Read_Accel(hi2c, &cx, &cy, &cz);
    float cal_mag = sqrtf(cx*cx + cy*cy + cz*cz);
    DEBUG_PRINT("MPU: cal X=%.2f Y=%.2f Z=%.2f mag=%.2f%s\r\n",
                Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset,
                cal_mag, (cal_mag < 0.9f || cal_mag > 1.1f) ? " WARN:move during cal!" : "");
}

void MPU6050_Config_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t threshold, uint8_t duration) {
    MPU_WriteReg(hi2c, REG_MOT_THR,    threshold);
    MPU_WriteReg(hi2c, REG_MOT_DUR,    duration);
    MPU_WriteReg(hi2c, REG_INT_ENABLE, INT_ENABLE_MOT_EN);
    DEBUG_PRINT("MPU: INT thr=%d dur=%d\r\n", threshold, duration);
}

void MPU6050_Clear_Interrupt(I2C_HandleTypeDef *hi2c) {
    MPU_ReadReg(hi2c, REG_INT_STATUS);
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, float *Ax, float *Ay, float *Az) {
    uint8_t raw[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, raw, 6, 100);
    *Ax = ((int16_t)((raw[0] << 8) | raw[1]) / 16384.0f) - Accel_X_Offset;
    *Ay = ((int16_t)((raw[2] << 8) | raw[3]) / 16384.0f) - Accel_Y_Offset;
    *Az = ((int16_t)((raw[4] << 8) | raw[5]) / 16384.0f) - Accel_Z_Offset;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, float *Gx, float *Gy, float *Gz) {
    uint8_t raw[6];
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, GYRO_XOUT_H_REG,
                     I2C_MEMADD_SIZE_8BIT, raw, 6, 100);
    *Gx = (int16_t)((raw[0] << 8) | raw[1]) / 131.0f;
    *Gy = (int16_t)((raw[2] << 8) | raw[3]) / 131.0f;
    *Gz = (int16_t)((raw[4] << 8) | raw[5]) / 131.0f;
}

void MPU6050_RegisterCallbacks(MPU_MotionDetectedCallback motionCb,
                                MPU_StationaryDetectedCallback stationaryCb) {
    motion_cb     = motionCb;
    stationary_cb = stationaryCb;
}

void MPU6050_Trigger_Wakeup(void) {
    if (mpu_state == MPU_SLEEP_MONITOR) {
        wakeup_triggered = true;
    }
}

void MPU6050_Reset_Stationary_Timer(void) {
    if (mpu_state == MPU_ACTIVE_HYSTERESIS) {
        last_motion_tick = HAL_GetTick();
    }
}

void MPU6050_Process(I2C_HandleTypeDef *hi2c) {
    if (!mpu_ok) return;
    uint32_t now = HAL_GetTick();

    switch (mpu_state) {

        case MPU_SLEEP_MONITOR:
            if (wakeup_triggered) {
                wakeup_triggered = false;
                DEBUG_PRINT("MPU: wake->validate\r\n");
                Validation_Reset();
                mpu_state = MPU_MOVEMENT_VALIDATION;
            }
            break;

        /* ------------------------------------------------------------------ */
        case MPU_MOVEMENT_VALIDATION:
        /* ------------------------------------------------------------------ */
            /*
             * Non-blocking validation: take one sample every VALIDATION_INTERVAL_MS.
             * Total duration = VALIDATION_SAMPLES × VALIDATION_INTERVAL_MS = 1.5 s.
             *
             * Design spec (Task 2.2): "Sample XYZ axes every 100ms for 1.5s.
             * Calculate vector magnitude variance. MCU enters __WFI() between samples."
             *
             * This non-blocking approach means SIM7670_Process() continues to run
             * between samples (critical during the 11.2 s SIM boot window).
             * __WFI() could be inserted in the SYS_POWER_OFF case of the main loop
             * to save power during the 100 ms waits if desired.
             */
            if ((now - val_last_sample_tick) >= VALIDATION_INTERVAL_MS) {
                val_last_sample_tick = now;

                float ax, ay, az;
                MPU6050_Read_Accel(hi2c, &ax, &ay, &az);
                float raw_mag = sqrtf(ax*ax + ay*ay + az*az);

                /* Seed the EMA on the first sample to avoid a cold-start transient */
                if (val_sample_count == 0) {
                    val_filtered_mag = raw_mag;
                }

                /* Exponential Moving Average low-pass filter to remove high-freq noise */
                val_filtered_mag = (EMA_ALPHA * raw_mag) + ((1.0f - EMA_ALPHA) * val_filtered_mag);

                /* Isolate dynamic (non-gravity) acceleration component */
                float dynamic_accel = fabsf(val_filtered_mag - 1.0f);

                val_sum_mag    += dynamic_accel;
                val_sum_sq_mag += (dynamic_accel * dynamic_accel);
                val_sample_count++;
            }

            /* Check if we have collected all required samples */
            if (val_sample_count >= VALIDATION_SAMPLES) {
                float mean     = val_sum_mag / VALIDATION_SAMPLES;
                float variance = (val_sum_sq_mag / VALIDATION_SAMPLES) - (mean * mean);

                DEBUG_PRINT("MPU: var — variance=%.6f (threshold=%.6f).\r\n",
                            variance, VALIDATION_VARIANCE_THRESHOLD);

                /* MUST clear the INT latch after validation (design spec Task 2.2 & 6.2) */
                MPU6050_Clear_Interrupt(hi2c);

                if (variance > VALIDATION_VARIANCE_THRESHOLD) {
                    DEBUG_PRINT("MPU: motion->HYST → ACTIVE_HYSTERESIS.\r\n");

                    /* Lower threshold to sensitive driving level (Schmitt-trigger lower bound) */
                    MPU6050_Config_Interrupt(hi2c, 10, 2);

                    last_motion_tick  = HAL_GetTick();
                    hyst_last_sample_tick = HAL_GetTick();
                    mpu_state         = MPU_ACTIVE_HYSTERESIS;

                    if (motion_cb) motion_cb();

                } else {
                    DEBUG_PRINT("MPU: false alarm->sleep\r\n");

                    /* Restore high wake threshold and re-arm */
                    MPU6050_Config_Interrupt(hi2c, 20, 1);
                    mpu_state = MPU_SLEEP_MONITOR;
                }
            }
            break;

        /* ------------------------------------------------------------------ */
        case MPU_ACTIVE_HYSTERESIS:
        /* ------------------------------------------------------------------ */
            /*
             * Non-blocking: sample every HYSTERESIS_SAMPLE_INTERVAL_MS to check
             * for ongoing vibration. Previously this was a raw read every main-loop
             * iteration (potentially thousands of Hz), which saturated the I2C bus.
             *
             * Any dynamic acceleration > HYSTERESIS_ACCEL_THRESHOLD resets the
             * 5-minute stationary timer. GPS speed events from main.c also call
             * MPU6050_Reset_Stationary_Timer() directly.
             */
            if ((now - hyst_last_sample_tick) >= HYSTERESIS_SAMPLE_INTERVAL_MS) {
                hyst_last_sample_tick = now;

                float ax, ay, az;
                MPU6050_Read_Accel(hi2c, &ax, &ay, &az);
                float mag           = sqrtf(ax*ax + ay*ay + az*az);
                float dynamic_accel = fabsf(mag - 1.0f);

                if (dynamic_accel > HYSTERESIS_ACCEL_THRESHOLD) {
                    last_motion_tick = now;
                }
            }

            /* Check 5-minute stationary timer */
            if ((now - last_motion_tick) >= STATIONARY_TIMEOUT_MS) {
                DEBUG_PRINT("MPU6050: 5min stationary → SLEEP_MONITOR.\r\n");

                /* Restore high wake threshold (sleep-mode sensitivity) */
                MPU6050_Config_Interrupt(hi2c, 20, 1);
                mpu_state = MPU_SLEEP_MONITOR;

                if (stationary_cb) stationary_cb();
            }
            break;
    }
}

void MPU6050_ForceActive(void) {
    /* Called when the system enters ACTIVE_TRANSIT via a path that bypasses the
     * normal IMU interrupt (e.g. cold-boot GPS fix).  Forces ACTIVE_HYSTERESIS
     * so the stationary timer starts immediately.  Without this, sleep would
     * never happen if the device never moves hard enough to trip the interrupt. */
    if (mpu_state == MPU_SLEEP_MONITOR) {
        last_motion_tick      = HAL_GetTick();
        hyst_last_sample_tick = HAL_GetTick();
        mpu_state             = MPU_ACTIVE_HYSTERESIS;
        DEBUG_PRINT("MPU: active (forced)\r\n");
    }
}
