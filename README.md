# no-long-lie 🛌🚫

**Real-time IoT Fall Detection & Alert System**

**no-long-lie** is an embedded healthcare solution built on the **STM32L4** platform. It uses multi-sensor fusion and high-speed Wi-Fi telemetry to detect human falls—specifically distinguishing between vertical collapses and rotational trips—and instantly notifies caregivers via a live web dashboard.

---

## 🚀 Key Features

* **Multi-Sensor Fusion**: Integrates 3-axis Accelerometer, Gyroscope, and Barometer data for high-accuracy fall validation.
* **Low-Latency Filtering**: Employs a custom **ARM Assembly** moving average filter for hardware-level signal processing.
* **Dynamic Altitude Tracking**: Implemented an Exponential Moving Average (EMA) to lock the baseline pressure $P_{base}$ at the moment of fall detection:
  $$P_{base} = \alpha \cdot P_{current} + (1 - \alpha) \cdot P_{prev}$$
* **Non-blocking FSM**: A deterministic **10Hz Finite State Machine** ensures sensor sampling isn't interrupted by Wi-Fi network latency.
* **Instant Alert System**: Features a local piezo-buzzer alarm and an auto-refreshing HTML dashboard.

---

## 🛠️ Technical Specifications

* **MCU**: STM32L475VG (ARM Cortex-M4).
* **Clock Configuration**: Boosted from 4MHz to **80MHz** via PLL to ensure stable SPI communication with the Wi-Fi module.
* **Timing**: 100ms cycle (80ms hardware delay + 20ms non-blocking Wi-Fi polling).
* **Languages**: C, ARM Assembly.

---

## 📊 System Architecture

The system operates as a high-speed loop, prioritizing sensor integrity while serving web requests.

```mermaid
graph LR
    Start([Start Cycle]) --> Read[Read & Filter Sensor Data]
    Read --> Stable{User Stable?}
    Stable -- Yes --> Base[Update Pressure EMA]
    Stable -- No --> CheckFall{Freefall Detected?}
    Base --> CheckFall
    CheckFall -- Yes --> Confirm{Rotation OR Altitude?}
    CheckFall -- No --> ResetTimer[Reset Timer]
    Confirm -- No --> ResetTimer
    Confirm -- Yes --> Alarm((ALARM ON))
    ResetTimer --> Btn{B2 Pressed?}
    Alarm --> Btn
    Btn -- Yes --> Safe((SAFE OFF))
    Btn -- No --> Output[Update Web UI]
    Safe --> Output
    Output -.-> Start
