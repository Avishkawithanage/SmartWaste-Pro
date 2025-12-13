
# SmartWaste Pro – IoT Layer (Plastic Compartment)

## Project Overview

SmartWaste Pro is a product-based IoT project designed as a **multi-compartment smart waste bin** for efficient waste monitoring.
This repository contains the **IoT layer implementation for the plastic waste compartment only**.

The objective of this phase is to validate **sensor integration, decision logic, and ESP32-based processing** before extending the system to additional compartments and application layers.

---

## Scope of This Implementation

✔ Plastic waste compartment only
✔ IoT layer only
✔ Simulation-based prototype
✔ ESP32 sensor logic and decision-making
✘ No mobile or web dashboard
✘ No buyer/seller marketplace logic

Other compartments (food, metal) and application layers will be implemented in future phases.

---

## Problem Statement

Traditional waste bins are monitored manually and often rely on single-sensor detection, which leads to inaccurate bin status reporting. Lightweight or unevenly distributed waste can cause false full detection. A reliable IoT-based solution is required to monitor bin status using multiple parameters before integrating with higher-level applications.

---

## Key Features

* Fill-level detection using an IR sensor
* Weight measurement using a 5 kg load cell with HX711
* ESP32-based sensor data acquisition
* Sensor fusion logic for accurate bin status detection
* Bin status classification: **EMPTY, PARTIAL, FULL**
* Noise handling and false-full prevention
* Cloud-ready serial output for future integration

---

## Hardware Components

### Planned / Simulated Hardware

* ESP32 microcontroller
* IR proximity sensor (fill-level detection)
* 5 kg load cell with HX711 amplifier (weight detection)
* LED indicator (FULL status)
* Power supply (USB / 5V)

> Note: In the current phase, sensor behavior is simulated using Wokwi. The same hardware configuration will be used during physical implementation.

---

## Software Components

* Arduino framework (ESP32)
* HX711 Arduino library
* Wokwi IoT Simulator
* Serial Monitor
* GitHub for version control

---

## Core Functionality Demonstration

The IoT logic was validated using a **simulation-based prototype**.
An IR sensor simulates fill-level input, while a 5 kg load cell with HX711 simulates weight input. The ESP32 processes both inputs using sensor fusion logic to determine bin status.

### Decision Logic Summary

| Fill Level | Weight | Status  |
| ---------- | ------ | ------- |
| 0%         | 0.5 kg | EMPTY   |
| 30%        | 1.0 kg | EMPTY   |
| 50%        | 2.0 kg | PARTIAL |
| 90%        | 4.0 kg | FULL    |

---

## Simulation Environment

* Platform: **Wokwi IoT Simulator**
* ESP32 Dev Module
* HX711 Load Cell (5 kg)
* Potentiometer used to simulate IR sensor input

Screenshots of the simulator and serial output are included as evidence.

---

## Repository Structure

```
SmartWaste-Pro-IoT/
│
├── sketch.ino        # ESP32 IoT logic (final version)
├── libraries.txt     # HX711 dependency
├── README.md         # Project documentation
```

---

## How to Run the Simulation

1. Open the project in **Wokwi**
2. Load `sketch.ino`
3. Ensure `libraries.txt` contains:

   ```
   HX711
   ```
4. Start the simulation
5. Adjust:

   * Potentiometer → Fill level
   * HX711 slider → Weight
6. Observe bin status in the Serial Monitor

---

## Output Example

```
Bin ID        : SWP-001
Compartment   : Plastic
Fill Level    : 50%
Weight        : 2.00 kg
Status        : PARTIAL
```

---

## Future Work

* Add food and metal waste compartments
* Integrate with cloud database (Firebase)
* Connect to mobile/web application layer
* Implement real hardware prototype
* Add notifications and analytics

---

## Author

**Avishka ITBNM - 22211 - 0199**
BIT (Hons) in Networking & Mobile Computing
Horizon Campus

---

## License

This project is developed for **academic and educational purposes**.

