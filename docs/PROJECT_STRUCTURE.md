# LuckD Project Overview

The LuckD project is an advanced elevator control system powered by the ESP32 platform. It utilizes a distributed architecture consisting of a central **Main Controller Unit (MCU)** and multiple specialized **Sub-Stations** that communicate wirelessly via ESP-NOW.

## 1. High-Level Architecture

The system follows a modular design pattern to ensure scalability and ease of maintenance.

- **Main Controller Unit (MCU):** The central hub that orchestrates the entire system, manages state, and handles external communications (Web, MQTT, Modbus).
- **Sub-Stations:** Peripheral units responsible for local interactions (cabin buttons, floor calls) and safety monitoring (ultrasonic guards).
- **Wireless Backbone:** Uses ESP-NOW for low-latency, point-to-multipoint communication between the MCU and sub-stations.

---

## 2. Main Controller Unit (MCU)
**Location:** `main_stations/main_controller_unit/`

The MCU is organized into a modular library structure to separate concerns:

### Modular Library Structure (`lib/`)
- **`core/` (ElevatorLogic):** Contains the `Orchestrator` class which implements the main elevator state machine and decision-making logic.
- **`hal/` (ElevatorHal):** Hardware Abstraction Layer. Manages physical I/O pins, motor controls, and sensor readings.
- **`comms/`:**
    - `NetworkManager`: Handles WiFi connectivity.
    - `WebManager`: Manages the AsyncWebServer and WebSockets for the user dashboard.
    - `MqttManager`: Integration with cloud brokers for remote monitoring.
- **`protocols/`:**
    - `EspNowManager`: Handles wireless communication with sub-stations.
    - `ModbusManager`: Industrial communication for motor drives or PLC integration.
    - `RFManager`: Support for 433MHz remote controls.
- **`storage/`:** Manages non-volatile data persistence (Preferences/SPIFFS).

### Web Interface (`data/`)
The MCU serves a rich web interface for real-time monitoring and configuration:
- `index.html`: Main dashboard with live status updates.
- `setup.html`: Configuration and calibration parameters.
- `websocketcode.js`: Real-time data sync between ESP32 and browser.

---

## 3. Sub-Stations
Each sub-station is a dedicated ESP32 unit performing a specific role.

| Station | Location | Role |
| --- | --- | --- |
| **Cabin Station** | `sub_stations/cabin_station/` | Interface inside the elevator car; handles floor selection and audio feedback (DFPlayer). |
| **Hall Station** | `sub_stations/hall_station/` | Floor-level call buttons and status indicators. |
| **VSG Station** | `sub_stations/vsg_station/` | **Vertical Safety Guard**: Obstacle detection using ultrasonic sensors. |
| **VTG Station** | `sub_stations/vtg_station/` | Auxiliary inputs and system-wide alarm management. |

---

## 4. Communication Flow
1. **Inputs:** Sensors (HAL), Web UI (WebSocket), or Sub-stations (ESP-NOW) trigger events.
2. **Processing:** The `Orchestrator` receives events and updates the internal state machine.
3. **Outputs:** The `Orchestrator` commands the `ElevatorHal` (motor/brakes) and notifies sub-stations or the web dashboard of status changes.
4. **Cloud:** Status is periodically published via `MqttManager` for remote telemetry.

---

## 5. Technology Stack
- **Firmware Framework:** Arduino Core for ESP32 with PlatformIO.
- **OS:** FreeRTOS (Task-based concurrency).
- **Key Libraries:**
    - `ESPAsyncWebServer`: High-performance asynchronous web server.
    - `ArduinoJson`: JSON processing for configuration and messaging.
    - `PubSubClient`: MQTT connectivity.
    - `ModbusMaster`: Industrial RTU protocol support.
    - `DFRobotDFPlayerMini`: MP3 audio management.

---

## 6. Directory Tree
```text
C:.
├── common/                  # Shared resources (future use)
├── docs/                    # Documentation
│   └── PROJECT_STRUCTURE.md
├── main_stations/
│   └── main_controller_unit/
│       ├── data/            # Web assets (SPIFFS)
│       ├── include/         # System headers (Config.h, Types)
│       ├── lib/             # Modular Managers (Core, HAL, Comms)
│       ├── src/             # Application entry point
│       └── test/            # Unit & Integration tests
└── sub_stations/
    ├── cabin_station/       # In-car interface
    ├── hall_station/        # Floor-level calls
    ├── vsg_station/         # Safety & Ultrasonic sensors
    └── vtg_station/         # Alarms & Auxiliary IO
```
