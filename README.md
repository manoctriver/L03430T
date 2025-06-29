# L0R430T - LoRaBot ESPHome Project

A comprehensive ESPHome-based long-range radio communication system for robot control using LoRa technology and Bluetooth HID or PC host as input.

## Project Overview

This repository contains ESPHome configurations for a distributed robot control system that enables long-range wireless communication between a transmitter (LRTX) and receiver (LRRX) using LoRa radio technology.

### Key Features

- **Long-Range Communication**: LoRa radio support for multi-kilometer range
- **Xbox Controller Integration**: Bluetooth HID client for Xbox controller input
- **Real-time Control**: Low-latency packet processing for responsive robot control
- **Dual Communication Modes**: ASCII and binary packet formats for optimal transmission
- **Visual Feedback**: OLED display with HUD elements for system status
- **Power Management**: Battery monitoring and power optimization features

## Hardware Platforms

### Primary Devices
- **Heltec WiFi LoRa 32 V3** (ESP32-S3 based)
- **ESP32-C3** variants for compact implementations
- **ESP32-S3** development boards

### Supported Peripherals
- SX1262 LoRa radio modules
- OLED displays (I2C)
- Xbox Wireless Controllers (Bluetooth)
- Various sensors and actuators

## Main Configuration Files

### Core Control Nodes
- `l0r430t-lrtx.yaml` - **Transmitter Node**: Central control hub with Xbox controller integration
- `l0r430t-lrrx.yaml` - **Receiver Node**: Robot-side control and actuator management
- `l0r430t-514v31.yaml` - Main robot controller
- `l0r430t-514v31-hexapod-IK.yaml` - Hexapod robot with inverse kinematics


### Sensor Nodes
- `mr60bha2.yaml` - Motion sensor configuration
- Various environmental sensors in `archive/` folder

## Custom Components

The project includes several custom ESPHome components located in the `components/` directory:

### Core Components
- **ble_client_hid**: Enhanced Bluetooth HID client with Xbox controller support
- **sx1262**: LoRa radio driver for SX1262 modules
- **uart_p2p_transmitter/receiver**: UART peer-to-peer communication
- **ld2410s**: Advanced presence detection sensor driver

### Installation
Custom components are automatically loaded from the local `components/` directory when using these configurations.

## Communication Protocol

### Packet Formats

#### Axis Control (7 bytes)
```
[0x41][axis_id][float32_value][0x0A]
```
- **axis_id**: 1-6 (triggers, thumbsticks)
- **float32_value**: IEEE 754 normalized value

#### Button Control (4 bytes)
```
[0x42][button_id][pressed_state][0x0A]
```
- **button_id**: 0-17 (XInput button mapping)
- **pressed_state**: 0=released, 1=pressed

#### ASCII Format (LoRa transmission)
```
A:axis_id:value    # Axis data: "A:3:0.500"
B:button_id:state  # Button data: "B:5:1"
```

## Setup Instructions

### Prerequisites
1. **ESPHome**: Install ESPHome (2024.6.0 or later recommended)
2. **Hardware**: Heltec WiFi LoRa 32 V3 or compatible ESP32-S3 board
3. **Xbox Controller**: Any Xbox Wireless Controller with Bluetooth

### Configuration Steps

1. **Clone this repository**
2. **Create secrets.yaml** with your network credentials:
   ```yaml
   wifi_ssid: "YourWiFiNetwork"
   wifi_password4: "YourWiFiPassword"
   domain: "your.local.domain"
   api: "your-api-encryption-key"
   ota: "your-ota-password"
   xboxblue: "XX:XX:XX:XX:XX:XX"  # Your Xbox controller MAC
   ```
3. **Flash the transmitter**: `esphome run l0r430t-lrtx.yaml`
4. **Flash the receiver**: `esphome run l0r430t-lrrx.yaml`
5. **Pair Xbox controller** with the transmitter device

### Usage

1. Power on both transmitter and receiver nodes
2. Wait for LoRa initialization (LED indicators will show status)
3. Press Xbox controller's Xbox button to initiate Bluetooth pairing
4. Control your robot using:
   - **Left trigger**: Reverse/brake
   - **Right trigger**: Forward throttle  
   - **Left thumbstick X**: Steering
   - **Right thumbstick**: Camera/turret control

## Development

### Component Updates

The BLE HID component has been updated to comply with ESPHome 2025.11.0 deprecation requirements:
- Replaced `text_sensor.TEXT_SENSOR_SCHEMA` with `text_sensor.text_sensor_schema()`
- Reference: [ESPHome Schema Deprecations](https://developers.esphome.io/blog/2025/05/14/_schema-deprecations/)

### Adding New Devices

1. Copy an existing configuration file
2. Modify device-specific settings (name, pins, features)
3. Update the hardware-specific build flags if needed
4. Test thoroughly before deployment

## Troubleshooting

### Common Issues

**LoRa transmission problems:**
- Check antenna connections
- Verify frequency settings match between TX/RX
- Ensure power supply is adequate (3.3V/5V as required)

**Bluetooth pairing issues:**
- Clear paired devices on controller
- Reset ESP32 and retry pairing
- Check MAC address in secrets.yaml

**OLED display not working:**
- Verify I2C connections (SDA/SCL)
- Check VEXT power control (GPIO36 on Heltec V3)
- Ensure display initialization sequence runs

## License

This project is open source. Please respect any component-specific licenses in the `components/` directory.

## Contributing

1. Fork this repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request with detailed description

## Acknowledgments

- ESPHome community for the excellent framework
- Heltec for the versatile WiFi LoRa 32 V3 hardware
- Component authors: @fsievers22 (BLE HID), @christianhubmann (SX1262)

---

**Warning**: This project involves radio frequency transmission. Ensure compliance with local regulations regarding LoRa frequency usage and power limits.
