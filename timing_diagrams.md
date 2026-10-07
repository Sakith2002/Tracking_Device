# System Timing Diagrams & Timelines

This document outlines the timing requirements, hardware pin states, and processing windows for the tracking device system. I have updated these to use Sequence Diagrams throughout, ensuring the non-blocking architecture and callbacks are clearly visible.

---

## 1. Top-Level System Sequence (The Big Picture)
This shows how the system transitions through its high-level states (`SYS_POWER_OFF` → `SYS_WAKE_UP` → `SYS_ACTIVE_TRANSIT` → `SYS_SLEEP`), highlighting how tasks are handed off to callbacks.

```mermaid
sequenceDiagram
    participant M as Modules (GPS/SIM)
    participant C as Callbacks (DMA/EXTI)
    participant MCU as MCU (STM32)
    
    Note over M, MCU: System in Stop Mode (SYS_POWER_OFF)
    
    M->>C: MPU-6050 EXTI Trigger
    C->>MCU: Wakes MCU
    MCU->>MCU: Movement Validation (Takes 1.5s total)
    
    Note over MCU, M: Transition to SYS_WAKE_UP
    MCU->>M: Trigger GPS Wake & SIM Cold Boot (Non-Blocking)
    
    Note over MCU, M: Transition to SYS_ACTIVE_TRANSIT
    MCU-->>MCU: Enter __WFI() Light Sleep
    
    rect rgb(230, 240, 255)
        Note over MCU, M: Normal Operation Loop
        M->>C: NMEA Sentence Received (DMA)
        C->>MCU: RxCpltCallback (Non-Blocking)
        
        MCU->>MCU: 10s Timer: Buffer GPS Data to FIFO
        MCU->>M: 60s Timer: Start HTTP TX (Non-Blocking)
        M->>C: HTTP Response
        C->>MCU: TransmitCompleteCallback
    end
    
    MCU->>MCU: 5-Minute Stationary Timeout
    Note over MCU, M: Transition to SYS_SLEEP
    MCU->>M: Trigger Final HTTP TX (Non-Blocking)
    M->>C: HTTP Response
    C->>MCU: TransmitCompleteCallback
    
    Note over MCU, M: Transition to SYS_POWER_OFF
    MCU->>M: Power Down SIM (Takes 2.5s pulse) & GPS
    MCU-->>MCU: Enter Stop Mode
```

### Top-Level Abstract Gantt Chart
This Gantt chart shows an abstract timeline of the MCU states mapped against the priority levels of the interrupts and callbacks that drive the transitions.

```mermaid
gantt
    title Top-Level Task Priority & State Transitions
    dateFormat YYYY-MM-DD
    axisFormat %d

    section 1. EXTI (Highest)
    MPU-6050 INT (Wakes MCU)     :milestone, m1, 2024-01-02, 0d

    section 2. DMA (Medium)
    NEO-6M NMEA Rx (1Hz)         :milestone, m2, 2024-01-04, 0d
    SIM7670 HTTP Response Rx     :milestone, m3, 2024-01-07, 0d

    section 3. Timers (Lowest)
    10s FIFO Buffer Update       :milestone, m4, 2024-01-06, 0d
    60s HTTP TX Trigger          :milestone, m5, 2024-01-06, 0d
    5-Min Stationary Timeout     :milestone, m6, 2024-01-09, 0d

    section MCU State Machine
    SYS_POWER_OFF (Stop Mode)    :done, 2024-01-01, 1d
    SYS_WAKE_UP (Validation)     :active, 2024-01-02, 2d
    SYS_ACTIVE_TRANSIT (__WFI)   :active, 2024-01-04, 5d
    SYS_SLEEP (Powering Down)    :crit, 2024-01-09, 2d
    SYS_POWER_OFF (Stop Mode)    :done, 2024-01-11, 2d
```

---

## 2. Wake-up & Movement Validation (Task 2)
Detailed view of what happens exactly when the car starts moving from a cold state (Stop Mode). Note that the entire cellular cold boot sequence is non-blocking.

* **Pin States:**
  * `MPU_INT` (PB8): Transitions `LOW → HIGH`. Remains HIGH until `INT_STATUS` is read at the end of validation.
  * `SIM_PWRKEY` (PB0): Was LOW. Once validation passes, transitions `LOW → HIGH → LOW` to trigger the boot pulse (Takes 50ms).

```mermaid
sequenceDiagram
    participant MPU as MPU-6050
    participant MCU as STM32
    participant SIM as SIM7670
    
    Note over MPU, MCU: Stop Mode
    MPU->>MCU: INT Pin LOW -> HIGH (EXTI)
    MCU-->>MCU: Wake up instantly (Takes < 1ms)
    
    rect rgb(200, 220, 255)
        Note over MCU, MPU: Movement Validation (Takes 1.5s total)
        loop 15 times (every 100ms)
            MCU->>MPU: I2C Read Accel XYZ
            Note over MCU: Enters __WFI() between reads
        end
    end
    
    MCU->>MPU: Read INT_STATUS (Clears Interrupt)
    MPU-->>MCU: INT Pin HIGH -> LOW
    
    Note over MCU, SIM: Full Cellular Cold Boot (Non-Blocking)
    MCU->>SIM: PWRKEY HIGH
    MCU->>SIM: PWRKEY LOW (Takes 50ms pulse)
    Note over SIM: Ton(uart) Boot Wait (Takes 11.2s)
    Note over MCU: MCU returns to main loop (Non-Blocking)
    
    SIM-->>MCU: UART Ready for AT Commands
    Note over SIM, MCU: Network Config (Takes ~2-3s via async Callbacks)
```

---

## 3. Active Transit Loop: Normal Operation (Task 3 & 5)
During normal tracking, the MCU sleeps continuously in `__WFI()`. It is woken briefly by DMA interrupts to parse GPS data, and executes timer-based events to buffer the FIFO and send payloads.

```mermaid
sequenceDiagram
    participant GPS as NEO-6M
    participant DMA as UART DMA
    participant MCU as STM32
    participant SIM as SIM7670
    
    Note over MCU: MCU in __WFI() Light Sleep
    
    rect rgb(230, 240, 255)
        Note over GPS, MCU: Continuous GPS Ingestion (Non-Blocking)
        GPS->>DMA: NMEA Data Stream (1Hz)
        DMA->>MCU: UART Idle IRQ (End of sentence)
        MCU->>MCU: Parse $GPRMC
        MCU-->>MCU: Return to __WFI()
    end
    
    rect rgb(240, 255, 230)
        Note over MCU: Every 10 Seconds
        MCU->>MCU: Read parsed data & add to FIFO
        MCU-->>MCU: Return to __WFI()
    end
    
    rect rgb(255, 240, 230)
        Note over MCU, SIM: Every 60 Seconds
        MCU->>SIM: Trigger SIM7670_Send_Data() (Non-Blocking)
        Note over SIM: HTTP state machine begins (See diagram 4)
        MCU-->>MCU: Reset FIFO & Return to __WFI()
    end
```

---

## 4. SIM7670 Non-Blocking HTTP Transmission
This details the non-blocking state machine for the cellular modem. The MCU does *not* wait in a `while()` loop for `OK`. It sends the command and yields.

```mermaid
sequenceDiagram
    participant MCU as STM32
    participant SIM as SIM7670
    
    Note over MCU, SIM: 60s Timer Expires
    MCU->>SIM: DTR LOW
    Note right of SIM: Wake Settle (Takes 50ms)
    
    MCU->>SIM: AT+HTTPINIT
    Note left of MCU: Main loop continues (Non-Blocking)
    SIM-->>MCU: OK (DMA Idle IRQ)
    
    MCU->>SIM: AT+HTTPPARA="URL","..."
    SIM-->>MCU: OK (DMA Idle IRQ)
    
    MCU->>SIM: AT+HTTPACTION=0
    Note left of MCU: MCU services GPS/IMU while waiting
    Note right of SIM: Network Latency (Takes 2s - 10s)
    SIM-->>MCU: +HTTPACTION: 0,200 (DMA Idle IRQ)
    
    MCU->>SIM: AT+HTTPTERM
    SIM-->>MCU: OK (DMA Idle IRQ)
    
    MCU->>SIM: DTR HIGH
    Note right of SIM: Enters Light Sleep
```

---

## 5. Sleep & Power-Down Routine (Task 6)
Detailed view of how the system safely powers down. Note that the final HTTP transaction is non-blocking, but the `PWRKEY` pulse logic acts as a final blocking guard before sleep.

* **Pin States:**
  * `SIM_PWRKEY` (PB0): Normally LOW. Transitions `LOW → HIGH → LOW` to form a 2.5-second pulse.
  * `MPU_INT` (PB8): Cleared, then re-armed for high threshold.

```mermaid
sequenceDiagram
    participant MCU as STM32
    participant SIM as SIM7670
    participant GPS as NEO-6M
    participant MPU as MPU-6050
    
    Note over MCU: 5-Min Stationary Timer Expires
    MCU->>SIM: Trigger Final HTTP TX (Non-Blocking)
    Note over SIM: (Takes 2s - 10s for Network Latency)
    SIM-->>MCU: TransmitCompleteCallback
    
    Note over MCU: Transition to SYS_POWER_OFF
    
    MCU->>GPS: UBX Power Save Cmd
    Note over GPS: (Takes < 10ms)
    
    MCU->>SIM: PWRKEY HIGH
    Note over SIM: 2.5s Pulse (Blocking)
    MCU->>SIM: PWRKEY LOW
    
    Note over SIM: UART Toff Decay Wait (Takes 1.9s)
    
    MCU->>MPU: Read INT_STATUS & Re-arm EXTI
    Note over MCU: Enter HAL_PWR_EnterSTOPMode()
    Note over MCU, MPU: Stop Mode
```

---

## 6. Hardware Module Wake/Sleep Specifications

The following diagrams show the raw hardware-level timing constraints for each module. The timings listed below have been verified against the respective datasheets and dictate the buffers and timeouts used in our top-level C code.

### SIM7670C Cellular Module

**Power-On Sequence**
Takes **11.2 seconds** from the power-on issue until the UART port is ready to accept commands and the `STATUS` pin outputs a high level.
```mermaid
flowchart LR
    A[Total Power Off] -->|Trigger: PWRKEY Low 50ms| B[Booting Baseband]
    B -->|Wait: 11.2s| C[UART Ready]
```

**Power-Off Sequence**
Takes **1.9 seconds** for the UART interface to fully shut down. A minimum buffer time of 2 seconds is strictly required before initiating a new power-on sequence.
```mermaid
flowchart LR
    A[Active] -->|Trigger: PWRKEY Low 2.5s| B[Shutting Down]
    B -->|Wait: 1.9s| C[Total Power Off]
```

### u-blox NEO-6M GPS

**Hot Start (V_BCKP Maintained)**
```mermaid
flowchart LR
    A[V_BCKP Active] -->|Trigger: VCC Applied| B[Initializing]
    B -->|Wait: 1.0s TTFF| C[1Hz Tracking]
```

**Cold Start (Total Power Loss)**
```mermaid
flowchart LR
    A[Total Power Off] -->|Trigger: VCC Applied| B[Acquiring Satellites]
    B -->|Wait: 27.0s TTFF| C[1Hz Tracking]
```

**Sleep Mode (Software Power Save)**
Sending the `UBX-RXM-PMREQ` command over UART immediately forces the GPS into a low-power backup state (provided `V_BCKP` is active) within ~10ms.
```mermaid
flowchart LR
    A[1Hz Tracking] -->|Trigger: UBX PM2 Cmd| B[Shutting Down]
    B -->|Wait: < 10ms| C[V_BCKP Active]
```

### MPU-6050 IMU

**Sleep Mode Exiting**
The internal Phase-Locked Loop (PLL) requires between 1ms and 10ms to settle. The gyroscope Zero-Rate Output (ZRO) requires 30ms to fully settle from power-on before providing accurate readings.
```mermaid
flowchart LR
    A[Sleep Mode] -->|Trigger: I2C Clear Sleep Bit| B[PLL Settle]
    B -->|Wait: 1ms to 10ms| C[Sensor Settle]
    C -->|Wait: 30ms| D[Sensors Ready]
```

### STM32 MCU (System Controller)

**Stop Mode Exiting**
Entering Stop Mode stops the core clock. Waking requires waiting for the HSI oscillator to stabilize (typically ~5.4µs for the STM32F1xx series).
```mermaid
flowchart LR
    A[Stop Mode] -->|Trigger: MPU EXTI INT| B[HSI Oscillator Startup]
    B -->|Wait: ~5.4µs| C[Executing Code]
```

---

## 7. System Power Profile

The graph below visualizes the system's power consumption over an 8-minute window. It clearly demonstrates the massive difference between the transient active spikes (like the SIM boot and cellular transmission) and the highly efficient baseline sleep states while parked. 

![System Power Profile](file:///C:/Users/Rakindu/.gemini/antigravity-ide/brain/dafd3739-9b20-4bc0-9780-11bea03fd615/power_profile.png)
