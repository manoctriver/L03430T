# l0r430t(LORABOT) - Long Range Remote Control Framework

![Project Badge](https://img.shields.io/badge/ESPHome-Compatible-blue)
![Version](https://img.shields.io/badge/Version-1.0.0-green)
![License](https://img.shields.io/badge/License-ManRiver-orange)

## Overview

**l0r430t(LORABOT)** is a versatile, long-range remote control framework built on ESPHome that leverages LoRa (Long Range) radio technology to provide wireless control over distances far exceeding traditional WiFi or Bluetooth solutions. The system is designed for universal applications including drones, robots, pan-tilt cameras, and any remote-controlled device requiring reliable long-distance communication.

### Key Features
- **Long Range Communication**: LoRa radio support with customizable frequency bands (904.2 MHz default for US)
- **Dual Input Methods**: 
  - Bluetooth Low Energy (BLE) controller support (Xbox controllers)
  - PC-hosted input via USB-C and Python XInput script
- **Multi-Node Architecture**: Transmitter (LRTX), Receiver (LRRX), and Slave nodes
- **Real-time Control**: Low-latency transmission optimized for responsive control
- **Flexible Output**: Motor control, servo control, and custom GPIO outputs
- **Web Interface**: Built-in web server for monitoring and configuration
- **Battery Monitoring**: Real-time battery status and power management

## System Architecture

The l0r430t system consists of three main components:

### 1. LRTX (LoRa Transmit Node)
**Hardware**: Heltec WiFi LoRa 32 V3 (ESP32-S3)
- **Primary Function**: Control input aggregation and LoRa transmission
- **Input Sources**:
  - BLE HID controllers (Xbox controllers via Bluetooth)
  - USB-C serial connection from PC (using xinputToL0r430T.py)
- **Features**:
  - Built-in OLED display for status monitoring
  - Battery voltage monitoring
  - WiFi connectivity for configuration
  - Real-time controller input processing

### 2. LRRX (LoRa Receive Node)
**Hardware**: Seeed XIAO ESP32-S3 with LoRa HAT
- **Primary Function**: LoRa packet reception and command relay
- **Communication**: UART P2P to slave devices
- **Features**:
  - Packet processing and validation
  - Command forwarding to slave nodes
  - Status LED indicators
  - Low power operation (80MHz CPU)

### 3. 514v31 (Slave Node)
**Hardware**: ESP32-C3 Development Board
- **Primary Function**: End effector control and motor/servo operation
- **Capabilities**:
  - Motor controller interface (AIN1/AIN2 direction control)
  - PWM speed control via Fan component
  - Servo control (GPIO4)
  - RGB LED status indication
  - WiFi control relay from remote nodes

#### 3a. Hexapod Implementation (Proof of Concept)
**Hardware**: ESP32-C3 + Freenove Hexapod Kit
- **Advanced Features**:
  - **18-Servo Control**: 6 legs × 3 joints each (Coxa, Femur, Tibia)
  - **Dual PCA9685 PWM Controllers**: 16-channel servo drivers at 0x40 and 0x41
  - **IMU Integration**: MPU6050 for real-time orientation sensing
  - **Inverse Kinematics Engine**: Real-time leg positioning calculations
  - **Kalman Filter**: Advanced sensor fusion for stability control
  - **PID-based Balance**: Automatic body leveling and posture correction
  - **Battery Monitoring**: Dual voltage monitoring via ADS7830 ADC
  - **Ultrasonic Sensing**: Distance measurement for obstacle avoidance
  - **Audio Feedback**: RTTTL buzzer for status notifications

- **Walking Robot Capabilities**:
  - **6-DOF Body Control**: X/Y/Z translation and pitch/roll/yaw rotation
  - **Gait Generation**: Programmable walking patterns and sequences
  - **Terrain Adaptation**: Real-time leg adjustment based on IMU feedback
  - **Power Management**: Intelligent standby mode with servo power control
  - **Safety Systems**: Emergency stops, servo limit protection, collision avoidance
  - **Remote Teleoperation**: Full LoRa-based remote control integration

- **Technical Specifications**:
  - **Leg Geometry**: Coxa: 33mm, Femur: 90mm, Tibia: 110mm (Freenove standard)
  - **Servo Range**: ±100 units with configurable safety limits
  - **Update Rate**: 50ms IMU processing, 10ms sensor polling
  - **Stability Control**: PID gains (Kp: 0.5, Ki: 0.0, Kd: 0.0025)
  - **IK Safety**: Conservative scaling factors to prevent servo damage
  - **Operating Modes**: Manual control, auto-balance, programmed gaits, standby

## Technical Specifications

### LoRa Configuration
```yaml
Frequency: 904.2 MHz (US Band)
Bandwidth: 500.0 kHz
Spreading Factor: 8
Coding Rate: 8 (4/8)
TX Power: -6 dBm (LRTX), -2 dBm (LRRX)
Sync Word: 0xf7
Data Rate: 80MHz controller
```

### Communication Protocols

#### XInput Mapping (PC to MCU via Serial)
| Button/Axis | Address | Type | Description |
|-------------|---------|------|-------------|
| A Button | B:12 | Binary | Primary action button |
| B Button | B:13 | Binary | Secondary action button |
| X Button | B:14 | Binary | Tertiary action button |
| Y Button | B:15 | Binary | Quaternary action button |
| DPad Up | B:0 | Binary | Directional pad up |
| DPad Down | B:1 | Binary | Directional pad down |
| DPad Left | B:2 | Binary | Directional pad left |
| DPad Right | B:3 | Binary | Directional pad right |
| Left Bumper | B:8 | Binary | Left shoulder button |
| Right Bumper | B:9 | Binary | Right shoulder button |
| Back | B:4 | Binary | Back/Select button |
| Start | B:5 | Binary | Start/Menu button |
| Left Stick Click | B:6 | Binary | Left thumbstick button |
| Right Stick Click | B:7 | Binary | Right thumbstick button |
| Left Trigger | A:1 | Analog | Left trigger (0-1) |
| Right Trigger | A:2 | Analog | Right trigger (0-1) |
| Left Thumb X | A:3 | Analog | Left stick horizontal |
| Left Thumb Y | A:4 | Analog | Left stick vertical |
| Right Thumb X | A:5 | Analog | Right stick horizontal |
| Right Thumb Y | A:6 | Analog | Right stick vertical |

#### UART P2P Addressing Scheme
| Sensor/Control | Address | Data Type | Purpose |
|----------------|---------|-----------|---------|
| Remote Servo | 0x01 | Float | Servo position control |
| Remote Throttle | 0x02 | Float | Forward motor speed |
| Remote Reverse | 0x03 | Float | Reverse motor speed |
| Left Thumb Y | 0x04 | Float | Left stick Y-axis |
| Right Thumb Y | 0x05 | Float | Right stick Y-axis |
| WiFi State | 0x06 | Boolean | WiFi control relay |
| BLE Hat Control | 0x15 | Float | BLE-specific control |
| LoRa RX Activity | 0x18 | Boolean | Communication status |

## Hardware Requirements

### LRTX Node (Transmitter)
- **MCU**: Heltec WiFi LoRa 32 V3
- **CPU**: ESP32-S3 @ 200MHz
- **Memory**: 8MB Flash
- **Display**: SSD1306 OLED (I2C)
- **Radio**: SX1262 LoRa transceiver
- **Connectivity**: WiFi, Bluetooth, USB-C
- **Power**: Battery monitoring via ADC

### LRRX Node (Receiver)
- **MCU**: Seeed XIAO ESP32-S3
- **CPU**: ESP32-S3 @ 80MHz
- **Memory**: 8MB Flash
- **Radio**: WIO-SX1262 LoRa HAT
- **Connectivity**: WiFi, USB-C
- **Communication**: UART P2P to slave

### Slave Node (End Effector)
- **MCU**: ESP32-C3 Development Board
- **CPU**: ESP32-C3 @ 80MHz
- **Memory**: 4MB Flash
- **Outputs**: Motor control, Servo control, RGB LED
- **Power**: GPIO-controlled motor driver interface

## Software Components

### External Components Used
- **sx1262**: LoRa radio driver
- **ble_client_hid**: Bluetooth HID client for controller input
- **uart_p2p_transmitter/receiver**: Custom UART point-to-point communication

### Key Scripts and Automations
- **lora_tx_script**: Manages LoRa packet transmission timing
- **lora_rx_script**: Handles received packet processing
- **uart_host_script**: PC serial communication management
- **mcu_idle**: Power management and idle state handling

## Control Flow

```
[BLE Controller] ─┐
                  ├─► [LRTX Node] ──[LoRa]──► [LRRX Node] ──[UART P2P]──► [Slave Node] ──► [Motors/Servos]
[PC + XInput] ────┘
```

### Detailed Signal Path
1. **Input Acquisition**: 
   - BLE controller connects to LRTX via Bluetooth HID
   - OR PC sends XInput data via USB-C serial to LRTX
2. **Signal Processing**: LRTX processes and normalizes input data
3. **LoRa Transmission**: Formatted packets sent over 904.2MHz LoRa link
4. **Reception**: LRRX receives and validates LoRa packets
5. **Command Relay**: LRRX forwards commands via UART P2P to slave
6. **Execution**: Slave node controls motors, servos, and other outputs

## Performance Characteristics

### Latency Optimization
- **Heartbeat Filtering**: 0.04s (25Hz) update rate for consistent data flow
- **Zero Buffer UART**: Minimized latency on serial connections
- **Optimized CPU Speeds**: Balanced performance vs. power consumption
- **Direct Memory Access**: Efficient packet handling without buffering delays

### Range and Reliability
- **LoRa Range**: Several kilometers depending on terrain and obstacles
- **Adaptive Power**: Configurable TX power levels for range vs. battery life
- **Error Correction**: Built-in LoRa coding rate provides packet integrity
- **Connection Monitoring**: Real-time RSSI and SNR reporting

## Configuration and Setup

### Basic Configuration Steps
1. **Flash ESPHome configurations** to respective devices
2. **Configure secrets.yaml** with WiFi credentials and device MAC addresses
3. **Install xinputToL0r430T.py dependencies** on host PC
4. **Pair BLE controller** with LRTX node
5. **Verify LoRa communication** between LRTX and LRRX nodes
6. **Test end-to-end control** through slave node outputs

### Frequency Band Configuration
The system supports multiple regional frequency bands:
- **US**: 902.0 - 928.0 MHz (Current: 904.2 MHz)
- **EU 868**: 869.4 - 869.65 MHz  
- **EU 433**: 433.0 - 434.0 MHz
- **CN**: 470.0 - 510.0 MHz
- **JP**: 920.8 - 927.8 MHz
- **ANZ**: 915.0 - 928.0 MHz

## Use Cases and Applications

### Robotics
- **Mobile Robots**: Long-range teleoperation beyond WiFi limits
- **Outdoor Rovers**: Reliable control in remote environments
- **Agricultural Automation**: Farm equipment remote operation
- **Hexapod Walker** (Proof of Concept): 6-legged walking robot with advanced kinematics

#### Featured Implementation: Hexapod Walking Robot
The l0r430t system demonstrates its versatility through a sophisticated hexapod robot implementation based on the Freenove hexapod kit. This proof-of-concept showcases:

**Advanced Control Features**:
- **Real-time Inverse Kinematics**: Calculate joint angles for desired foot positions
- **IMU-based Stability**: Automatic body leveling using MPU6050 and Kalman filtering
- **PID Balance Control**: Maintains stable posture on uneven terrain
- **Intelligent Power Management**: Automatic standby mode with configurable timeouts
- **Safety Systems**: Emergency stops, servo protection, and conservative movement limits

**Walking Capabilities**:
- **Gait Programming**: Multiple walking patterns and movement sequences
- **Terrain Adaptation**: Real-time leg adjustment based on sensor feedback
- **Remote Teleoperation**: Full LoRa-based control from transmitter node
- **Body Positioning**: 6-DOF body control (X/Y/Z translation, pitch/roll/yaw)
- **Obstacle Avoidance**: Ultrasonic sensor integration for navigation

**Technical Achievements**:
- **18-Servo Coordination**: Simultaneous control of all leg joints
- **50ms Control Loop**: Real-time processing for smooth movement
- **Dual Battery Monitoring**: Separate power tracking for servos and MCU
- **Conservative Scaling**: Servo protection through configurable safety limits
- **Web-based Tuning**: Live adjustment of PID parameters and movement limits

### Surveillance and Monitoring
- **Pan-Tilt Cameras**: Remote camera positioning and control
- **Security Systems**: Long-range monitoring station control
- **Environmental Sensors**: Remote data collection and device control

### Hobby and Educational
- **RC Vehicles**: Enhanced range for RC cars, boats, planes
- **Educational Projects**: STEM learning platform for wireless communication
- **Maker Projects**: Foundation for custom remote control applications

## Troubleshooting and Diagnostics

### Built-in Monitoring
- **Web Interfaces**: Real-time status monitoring on all nodes
- **LED Indicators**: Visual feedback for system status
- **OLED Display**: Local status information on LRTX node
- **Battery Monitoring**: Low voltage warnings and power management

### Common Issues and Solutions
- **Connection Problems**: Check frequency configuration and sync words
- **Latency Issues**: Verify heartbeat timing and CPU speeds
- **Range Limitations**: Adjust TX power and antenna positioning
- **Power Management**: Monitor battery levels and optimize CPU speeds

## Future Enhancements

### Planned Features
- **Multi-Controller Support**: Multiple simultaneous input devices
- **Mesh Networking**: Multiple slave nodes from single LRRX
- **Advanced Telemetry**: Enhanced sensor feedback from remote nodes
- **Mobile App Integration**: Smartphone control interface

### Development Roadmap
- [ ] Protocol documentation and standardization
- [ ] Additional input device support (PS4/PS5 controllers)
- [ ] Encrypted communication for security applications
- [ ] Advanced motor control algorithms (PID, trajectory planning)

---

## Contributing

This project is developed by ManRiver as part of the L0R430T project family. For questions, improvements, or collaboration opportunities, please reach out through the appropriate channels.

## License

Copyright (c) 2025 ManRiver. All rights reserved.

---

*Last Updated: June 29, 2025*