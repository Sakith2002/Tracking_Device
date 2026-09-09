# Tasks

**Task 1: Cold Boot / System Init (Run Once)**

* **1.1 Power Assertion:** Check battery voltage via STM32 ADC.  
* **1.2 Peripheral Setup:** Initialize UART1 (DMA), UART2 (RTS/CTS hardware flow control), and I2C1.  
* **1.3 Sensor Setup:** Set MPU-6050 MOT\_THR and MOT\_DUR. The duration parameter acts as a hardware debounce; it dictates exactly how many milliseconds the acceleration must exceed the threshold before the interrupt fires.  
* **1.4 NEO-6M Baseline:** Power the NEO-6M and wait for the first valid $GPRMC NMEA string to establish ephemeris.  
* **1.5 Start SIM Module:** Pulse A7670C PWRKEY low to boot, wait 11.2s for UART readiness, configure APN, and post the first location data.  
* **1.6 Handoff:** Transition directly to the active loop (Task 3); the 5-minute timeout will naturally handle the transition to sleep if the vehicle is parked.

**Task 2: Wake-Up Routine (Stationary \-\> Moving)**

* **2.1 Ext Trigger:** MPU-6050 drives INT pin high; STM32 EXTI line wakes MCU from Stop Mode. read the INT\_STATUS register via I2C here to clear the flag no INT doesnt fire when system active.  
* **2.2 Validate Movement:** Sample the MPU-6050 XYZ axes every 100ms for 1.5s. Calculate vector magnitude variance. If below the engine-idle threshold, read the MPU-6050 INT\_STATUS register to clear the interrupt flag, and return to sleep.  
* **2.3 GPS Wake:** Send the appropriate UBX protocol message (or toggle Pin 4 EXTINT0) to wake the NEO-6M from Power Save Mode.  
* **2.4 Cellular Warmup:** Pulse A7670C PWRKEY for 50ms. Execute a non-blocking 11.2-second wait for UART readiness. Send the modem to light sleep after needed cofigs.  
* **2.5 Timer Setup:** Start the 60s upload timer and 5-minute stationary countdown timer.

**Task 3: Active Transit Loop (Location & Buffering)**

* **3.1 DMA Ingestion:** Set up UART Idle Line Interrupts. The DMA writes incoming NMEA bytes to a circular buffer silently while the STM32 is in \_\_WFI(). The Idle Line interrupt wakes the CPU *only* when a full NMEA sentence has finished arriving, guaranteeing no buffer overruns while asleep.  
* **3.2 Decoding:** Parse valid $GPRMC for Time, Lat, and Lon.  
* **3.3 Data FIFO:** Push {Time, Lat, Lon} to internal memory every 10 seconds.

**Task 4: Movement Monitoring (Hysteresis & Timers)**

* **4.1 Speed Check:** If GPS speed \> 5 km/h, reset the stationary timer.  
* **4.2 Hysteresis Motion Check:** Use a lower acceleration threshold for maintaining active status than the initial Task 2 wake-up threshold. This Schmitt-trigger approach prevents bouncing between states during smooth highway driving.  
* **4.3 Expiry:** If 5 minutes pass without breaching the low threshold or exceeding 5km/h, flag for sleep.

**Task 5: Non-Blocking Data Transmission (60s Loop)**

* **5.1 Modem Wake:** Assert A7670C DTR low.  
* **5.2 Payload Construction:** Batch the cached {Time, Lat, Lon} coordinates into the HTTP GET string.  
* **5.3 State Machine Dispatch:** Send AT commands asynchronously. Yield the MCU until an RX interrupt with the HTTP 200 URC fires.  
* **5.4 Housekeeping:** Clear FIFO, assert DTR high.

**Task 6: Sleep Routine (Moving \-\> Stationary)**

* **6.1 Final Transmission:** Send one last {Time, Lat, Lon} coordinate batch.  
* **6.2 MCU Stop Prep:** The MPU-6050 interrupt stays asserted until cleared. You must read the INT\_STATUS register via I2C here to clear the flag and re-arm the pin before shutting down the other modules. \- here re arm the INT and the MPU int setup.  
* **6.3 Cellular Deep Sleep:** Hold A7670C PWRKEY low for 2.5s. If the MPU-6050 detects motion during this 2.5s window, the EXTI triggers immediately, aborting the sleep sequence.  
* **6.4 GPS Sleep:** Send UBX command to enter software Power Save Mode.  
* **6.5 MCU Stop Mode:** Execute PWR\_EnterSTOPMode().

&nbsp;

# states

**State: POWER\_OFF**

* **Justification:** Achieves the lowest possible leakage current (\<20 µA) when the vehicle is parked for extended periods.  
* **Entry:** Drive PWRKEY low for 2.5 seconds.  
* **Processing:** None. Hardware remains dormant.  
* **Exit:** Pulse PWRKEY low for 50ms to initiate boot sequence.

&nbsp;

**State: AWAITING\_RESPONSE**

* **Justification:** Centralizes all non-blocking UART timeouts and asynchronous RX parsing, preventing spaghetti code when handling callbacks from configuration, HTTP setups, or data transmissions.  
* **Entry:** AT command dispatched to the modem.  
* **Processing:** Yield MCU to main loop. Start a software timeout timer. Wait for the UART RX DMA/interrupt to flag a complete response line (e.g., `OK`, `ERROR`, or `+HTTPACTION: 0,200`).  
* **Exit:**  
  * *On Config Success:* Transition to `LIGHT_SLEEP`.  
  * *On TX Success:* Clear FIFO, transition to `LIGHT_SLEEP` (or `POWER_OFF` if stationary).  
  * *On Timeout/Error:* Increment retry counter, transition to appropriate error handler or retry state.

**State: BOOTING**

* **Justification:** The baseband requires a strict timing window for UART initialization before it can accept commands.  
* **Entry:** Hardware power-on pulse applied.  
* **Processing:** Non-blocking wait for Ton(uart) (11.2 seconds).  
* **Exit:** UART responds to AT command. Transition to NETWORK\_CONFIG.

**State: NETWORK\_CONFIG**

* **Justification:** Prepares the modem for transmission and configures power-saving parameters before yielding the bus.  
* **Entry:** UART communication established.  
* **Processing:** Configure APN, establish HTTP context, and send AT+CSCLK=1 to enable sleep mode capabilities.  
* **Exit:** Configuration acknowledged. Transition to LIGHT\_SLEEP.

**State: LIGHT\_SLEEP**

* **Justification:** Maintains network attachment while dropping current consumption between the 60-second transmission intervals.  
* **Entry:** Pull DTR pin high.  
* **Processing:** Baseband sleeps; UART is deactivated.  
* **Exit:** Pull DTR pin low. Transition to ACTIVE\_TX.

**State: ACTIVE\_TX**

* **Justification:** Handles the non-blocking execution of the HTTP GET request without locking the STM32 main loop.  
* **Entry:** Wait 50ms after pulling DTR low for UART to wake.  
* **Processing:** Dispatch batched {Time, Lat, Lon} string via AT+HTTPPARA. Yield MCU. Await RX interrupt containing the HTTP 200 URC.  
* **Exit:** HTTP session closes successfully. Pull DTR high to return to LIGHT\_SLEEP, or if 5-minute stationary timer expired, transition to POWER\_OFF.

Maybe good to add some state like busy or req in progress \- to show that the sim molecule is currently processing some request and waiting for an response \- this would help in managing non blocking without a mess \- this is a state that sim molecule could jumpt to and from from most of the other sates

**u-blox NEO-6M GPS State Machine**

* **State: POWER\_SAVE**  
  * **Justification:** Drops average current to \~11mA while retaining ephemeris and RTC data for an instant Hot Start.  
  * **Entry:** Dispatch UBX-CFG-PM2 command or assert EXTINT0 pin state.  
  * **Processing:** Internal baseband duty-cycles.  
  * **Exit:** Toggle EXTINT0 or send UBX wake command. Transition to ACQUIRING\_BASELINE (if cold) or CONTINUOUS\_TRACKING (if hot).  
* **State: ACQUIRING\_BASELINE**  
  * **Justification:** Ensures the system has a valid fix before committing to active tracking or allowing sleep.  
  * **Entry:** System cold boot.  
  * **Processing:** Parse incoming NMEA stream. Discard invalid $GPRMC packets.  
  * **Exit:** First valid $GPRMC parsed. Transition to CONTINUOUS\_TRACKING.  
* **State: CONTINUOUS\_TRACKING**  
  * **Justification:** Leverages UART DMA to silently ingest 1Hz navigation data with zero CPU blocking.  
  * **Entry:** Valid baseline established.  
  * **Processing:** DMA handles incoming bytes. Idle line interrupt wakes MCU to parse {Time, Lat, Lon, Speed}.  
  * **Exit:** 5-minute stationary timer expires. Transition to POWER\_SAVE.

&nbsp;

&nbsp;

&nbsp;

&nbsp;

&nbsp;

**MPU-6050 IMU State Machine**

* **State: SLEEP\_MONITOR**  
  * **Justification:** Offloads motion detection entirely to the IMU hardware, utilizing its Low-Power Accelerometer mode to draw minimal current.  
  * **Entry:** Set MOT\_THR (wake threshold) and MOT\_DUR (hardware debounce duration).  
  * **Processing:** Hardware autonomously checks acceleration against MOT\_THR.  
  * **Exit:** Acceleration exceeds threshold for MOT\_DUR. INT pin drives high. Transition to MOVEMENT\_VALIDATION.  
* **State: MOVEMENT\_VALIDATION**  
  * **Justification:** Prevents false wake-ups (e.g., slamming a door) from triggering the high-power GPS and Cellular modules.  
  * **Entry:** STM32 wakes via EXTI.  
  * **Processing:** Sample XYZ axes every 100ms for 1.5s. Compute vector magnitude variance. Read INT\_STATUS via I2C to clear the hardware flag.  
  * **Exit:** If variance \> engine-idle threshold, transition to ACTIVE\_HYSTERESIS. If variance \< threshold, transition to SLEEP\_MONITOR.  
* **State: ACTIVE\_HYSTERESIS**  
  * **Justification:** Prevents the stationary timer from constantly resetting due to road noise, acting as a Schmitt-trigger for movement monitoring.  
  * **Entry:** Movement validated.  
  * **Processing:** Dynamically lower the MOT\_THR to a sensitive "driving" threshold. Monitor for continuous vibration or GPS speed \> 5 km/h.  
  * **Exit:** 5 minutes elapse with no GPS speed and no vibrations exceeding the lowered threshold. Re-arm high threshold and transition to SLEEP\_MONITOR.

&nbsp;

# Timing data

### **Interrupt Architecture & Priority Routing**

* **Highest Priority (Preemptive): MPU-6050** INT **(EXTI Line)**

  * **Role:** Wakes the STM32 from Stop Mode.

&nbsp;

  * **Handling:** The MPU-6050 asserts this pin high upon threshold breach. It remains asserted until the STM32 actively reads the INT\_STATUS register via I2C to clear the latch.  
    &nbsp;  
  * **Re-assertion:** Must be cleared immediately after validation to prevent infinite EXTI loops, and armed immediately before entering Stop Mode.  
    &nbsp;  
* **Medium Priority: NEO-6M UART DMA & Idle Line**  
  

  * **Role:** Silently writes incoming bytes to RAM. The Idle Line interrupt fires only when the line goes quiet (signaling the end of a full NMEA sentence).  
    &nbsp;  
  * **Handling:** Wakes the STM32 from Sleep (\_\_WFI()) to parse the completed buffer.  
    &nbsp;  
* **Lowest Priority: A7670C UART RX (**AWAITING\_RESPONSE**)**

  * **Role:** Signals incoming URCs (Unsolicited Result Codes) like \+HTTPACTION: 0,200.  
    &nbsp;  
  * **Handling:** Updates the non-blocking state machine to proceed or retry.  
    &nbsp;

### **Hardware State Transition Timings**

| Module & Transition | Action Trigger | Settling / Delay Time | Notes |
| :---- | :---- | :---- | :---- |
| **A7670C Cold Boot** | 50ms low pulse on PWRKEY &nbsp; | **11.2 seconds** | Minimum time until UART is active and responds to AT commands.&nbsp; |
| **A7670C Power Off** | 2.5s low pulse on PWRKEY &nbsp; | **1.9 seconds** | Time for UART to fully power down.&nbsp; |
| **A7670C Reboot Buffer** | Transition via off/on | **2.0 seconds** | Mandatory buffer time between a power-off and the next power-on.&nbsp; |
| **NEO-6M Hot Start** | Software wake via UBX | **1.0 second** | Time-To-First-Fix (TTFF) when V\_BCKP maintains ephemeris.&nbsp; |
| **NEO-6M Cold Start** | Initial Power-Up | **27.0 seconds** | Typical TTFF without backup data.&nbsp; |
| **MPU-6050 Wake** | I2C config write | **1ms to 10ms** | PLL settling time after exiting low-power modes.&nbsp; |

### **Active Task Execution Schedule**

| Task | Execution Profile | Blocking? | Estimated Duration |
| :---- | :---- | :---- | :---- |
| **Movement Validation** | I2C sampling at 100ms intervals. | Yields | 1.5 seconds (Total) |
| **NMEA Parsing** | String search on $ and , delimiters. | Yes | \< 1 millisecond |
| **Modem Wake (DTR)** | Pull DTR low. | No | 50 milliseconds |
| **HTTP Dispatch** | UART TX via DMA. | No | \~5 \- 10 milliseconds |
| **Network Latency** | Waiting in AWAITING\_RESPONSE. | No | 1 \- 5 seconds (Network dependent) |

&nbsp;

**STM32F103 (Bluepill) Core Timings**

&nbsp;

* **Wake from Stop Mode (EXTI):** \~5.4 µs (dependent on HSI 8MHz startup time).  
* **Wake from Light Sleep (**\_\_WFI()**):** \~2 CPU cycles (\~27 ns at 72 MHz).  
* **GPIO Toggle (MOSFET/PWRKEY control):** \~14 ns.  
* **I2C1 Transaction (MPU-6050):** \~100 µs per byte at 400kHz Fast Mode.  
* **UART String Parsing:** \< 1 ms per NMEA sentence.  
  &nbsp;

**Sequence 1: Wake-Up & Validation (Stationary to Moving)**

&nbsp;

* **T=0.00 ms:** MPU-6050 detects motion threshold \> MOT\_DUR. Hardware drives INT pin high.  
* **T=0.01 ms:** STM32 EXTI fires; CPU wakes from Stop Mode (\~5.4 µs).  
* **T=0.02 ms:** STM32 toggles GPIO to switch on NEO-6M $V\_{CC}$.  
* **T=0.03 ms:** STM32 pulls A7670C PWRKEY low.  
* **T=50.03 ms:** STM32 releases PWRKEY (minimum 50ms pulse required to trigger boot).  
* **T=55.00 ms to 1550.00 ms:** STM32 samples MPU-6050 XYZ axes via I2C every 100ms to calculate variance. MCU enters \_\_WFI() between samples to save power  
* **T=1550.00 ms:** Movement validated. STM32 reads INT\_STATUS via I2C to clear the MPU-6050 interrupt latch.  
* **T=11250.00 ms:** A7670C UART becomes fully responsive (Ton(uart) \= 11.2s). STM32 configures network and sends AT+CSCLK=1.

&nbsp;

**Sequence 2: Active Transit Loop (60s Cycle)**

&nbsp;

* **T=0.00 s to 59.99 s:** STM32 resides in \_\_WFI() Light Sleep.  
* **1 Hz Intervals (Background):** NEO-6M streams NMEA. Idle Line Interrupt wakes STM32 (\~27 ns). Parsing takes \<1 ms. STM32 returns to \_\_WFI().  
* **T=60.00 s:** Timer expires. STM32 pulls A7670C DTR low to exit baseband light sleep.  
* **T=60.05 s:** STM32 waits 50ms for UART wake, then transmits HTTP GET string via DMA.  
* **T=60.05 s to 65.00 s:** STM32 enters \_\_WFI() in AWAITING\_RESPONSE state.  
* **T=65.00 s (Variable):** RX interrupt fires with HTTP 200\. STM32 clears data FIFO and pulls DTR high to resume A7670C light sleep.

&nbsp;

**Sequence 3: Sleep Preparation (Moving to Stationary)**

&nbsp;

* **T=0.00 s:** 5-minute stationary timer expires.  
* **T=0.01 s:** STM32 transmits final parked location (follows Sequence 2 timings).  
* **T=5.00 s:** Post-transmission, STM32 sends UBX software Power Save command to NEO-6M.  
* **T=5.01 s:** STM32 configures MPU-6050 MOT\_THR via I2C (\~0.5 ms).  
* **T=5.02 s:** STM32 pulls A7670C PWRKEY low.  
* **T=7.52 s:** STM32 releases PWRKEY (2.5s pulse required for deep power off).  
* **T=9.42 s:** A7670C completes internal power down (Toff(uart) \= 1.9s).  
* **T=9.43 s:** STM32 clears all pending interrupt flags, arms the EXTI line for the MPU-6050, and executes PWR\_EnterSTOPMode().

&nbsp;