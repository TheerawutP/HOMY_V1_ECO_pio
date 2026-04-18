# LuckD Project Overview

This project is a comprehensive elevator control system based on the ESP32 platform, featuring a central controller unit and several specialized sub-stations.

## Project Structure

The repository is organized into the following main directories:

- `main_stations/`: Contains the primary control logic and central hub.
- `sub_stations/`: Contains the firmware for various remote units located around the elevator system.
- `docs/`: Project documentation.
- `common/`: Shared resources and definitions (reserved for future use).

---

## 1. Main Controller Unit (MCU)
**Location:** `main_stations/main_controller_unit/`

The MCU is the "brain" of the elevator system. It manages the state machine, processes sensor inputs, and coordinates communication between all sub-stations and external interfaces.

### Key Features:
- **Core Logic:** Implements the elevator state machine (Idle, Moving Up/Down, Door Operations, Emergency).
- **Multi-tasking:** Uses FreeRTOS tasks to handle concurrent operations (Logic, Sensors, Communication).
- **Communication Protocols:**
  - **ESP-NOW:** Low-latency wireless communication with sub-stations.
  - **MQTT:** Remote monitoring and control via a cloud broker.
  - **Web Interface:** Built-in web server and WebSockets for real-time status and configuration.
  - **Modbus RTU:** Integration with industrial components like motor drives.
  - **RF 433MHz:** Support for standard RF remote controls.
- **Data Persistence:** Uses Preferences or SPIFFS to store configuration and state data.

### Web Interface Assets
Located in `main_stations/main_controller_unit/data/`, these files are served by the MCU:
- `index.html`: The main user dashboard for monitoring and manual control.
- `setup.html`: System configuration and calibration page.
- `wifi_index.html` & `wifimanager.js`: Interface for managing WiFi connectivity.
- `websocketcode.js`: Handles real-time communication between the web UI and the ESP32.

---

## 2. Sub-Stations
Sub-stations are remote ESP32-based units that communicate with the MCU via ESP-NOW.

### Cabin Station
**Location:** `sub_stations/cabin_station/`
- **Role:** The user interface inside the elevator car.
- **Elements:** 
  - Floor selection buttons (Up, Down, Stop).
  - Emergency button.
  - Audio feedback via DFPlayer Mini.
  - Control for cabin lighting and door solenoids.

### Hall Station
**Location:** `sub_stations/hall_station/`
- **Role:** Floor-level call buttons.
- **Elements:** Call buttons and status indicators for users waiting for the elevator.

### VSG Station (Vertical Safety Guard)
**Location:** `sub_stations/vsg_station/`
- **Role:** Safety and obstacle detection.
- **Elements:** 
  - Array of ultrasonic sensors (HC-SR04) for distance measurement and object detection.
  - LED indicators for visual feedback.
  - Buzzer for audible alerts.

### VTG Station
**Location:** `sub_stations/vtg_station/`
- **Role:** Auxiliary input and alarm management.
- **Elements:** Digital inputs for various sensors and relay/alarm outputs.

---

## 3. Technology Stack
- **Hardware:** ESP32 (NodeMCU-32S and similar variants).
- **Framework:** Arduino with PlatformIO.
- **Libraries:**
  - `ESPAsyncWebServer`: High-performance web server.
  - `ArduinoJson`: JSON serialization for API and config.
  - `PubSubClient`: MQTT client for IoT connectivity.
  - `ModbusMaster`: Industrial protocol support.
  - `rc-switch`: RF remote signal decoding.
  - `DFRobotDFPlayerMini`: MP3 audio playback.

---

## 4. Hardware Mapping (MCU)
Refer to `main_stations/main_controller_unit/include/Config.h` for pin definitions:
- **Motion Control:** `PIN_UP` (19), `PIN_DOWN` (18), `PIN_BRAKE` (5).
- **Sensors:** `PIN_SS_FLOOR_1` (32), `PIN_SS_FLOOR_2` (33).
- **Safety:** `PIN_EMO` (21), `PIN_SLING` (26), `PIN_SPEED` (14).
