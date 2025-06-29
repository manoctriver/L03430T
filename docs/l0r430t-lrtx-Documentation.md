# l0r430t-lrtx - LoRa Transmit Node Documentation

![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-blue)
![Hardware](https://img.shields.io/badge/Hardware-Heltec_WiFi_LoRa_32_V3-orange)
![Platform](https://img.shields.io/badge/Platform-ESP32S3-green)
![Version](https://img.shields.io/badge/Version-1.0.0-brightgreen)

## Overview

The **l0r430t-lrtx** is the transmitter node in the l0r430t(LORABOT) framework, serving as the primary control hub that aggregates input from multiple sources and broadcasts commands via LoRa radio. Built on the Heltec WiFi LoRa 32 V3 (ESP32-S3), this node bridges the gap between local control interfaces and long-range wireless communication.

## Hardware Platform

### Heltec WiFi LoRa 32 V3 Specifications
- **MCU**: ESP32-S3 @ 200MHz (configurable 160-240MHz)
- **Flash Memory**: 8MB with custom partition table
- **LoRa Radio**: SX1262 transceiver integrated on-board
- **Display**: 128x64 OLED (SSD1306) via I2C
- **Connectivity**: 
  - WiFi 802.11 b/g/n
  - Bluetooth Low Energy 5.0
  - USB-C with CP2102 USB-to-Serial converter
- **Power Management**: 
  - Built-in LiPo battery connector (SH1.25-2)
  - Battery voltage monitoring via GPIO1
  - VEXT control for power-efficient operation
- **GPIO**: Rich set of digital/analog pins for expansion

### Key Hardware Features
```yaml
# Core specifications from platformio_options
CPU Frequency: 200MHz (optimized for stability)
Flash Size: 8MB (default_8MB.csv partition)
Upload Speed: 460800 baud
USB CDC on Boot: Enabled (Windows COM port compatibility)
Arduino Core: Latest stable (6.11.0)
```

## System Architecture

### Dual Input Philosophy
The LRTX node implements a sophisticated dual-input system supporting both local and remote control sources:

#### 1. PC Serial Input (Primary Path)
- **Protocol**: Custom binary packets via USB-C
- **Baud Rate**: 460800 (high-speed, low-latency)
- **Source**: `xinputToL03430T.py` Python script
- **Packet Types**:
  - **Axis Data**: `[0x41, axis_id, float32_value, 0x0A]`
  - **Button Data**: `[0x42, button_id, pressed_state, 0x0A]`

#### 2. Bluetooth Low Energy Input (Secondary Path)
- **Protocol**: BLE HID (Human Interface Device)
- **Target Device**: Xbox controllers (configurable MAC address)
- **Processing**: Real-time HID parsing with custom usage mapping
- **Features**: Auto-reconnection, battery monitoring, RSSI tracking

### Data Flow Architecture
```
Input Sources → Processing Layer → LoRa Transmission → Remote Nodes
     ↓               ↓                    ↓              ↓
PC Serial     →  Binary Parsing  →  SX1262 Radio  →  LRRX Node
BLE HID       →  HID Processing   →  904.2MHz      →  Slave Devices
```

## Feature Analysis

### 1. Advanced Input Processing

#### USB-C Serial Processing (UART Debug Sequence)
The most sophisticated feature of the LRTX node is its real-time binary packet processing:

```cpp
// Axis packet processing (7-byte packets)
if (bytes.size() == 7 && bytes[0] == 0x41 && id(lora_rf).state == true) {
    uint8_t axis_id = bytes[1];
    float value;
    memcpy(&value, &bytes[2], 4);  // Extract IEEE 754 float
    
    // Format for LoRa transmission
    char buf[10];
    snprintf(buf, sizeof(buf), "A:%d:%.3f", axis_id, value);
    std::string msg(buf);
    
    // Update local HUD feedback
    if (axis_id == 0x01) {
        id(hud_reverse).make_call().set_value(value).perform();
    }
    
    // Transmit via LoRa
    id(lora)->send_packet(std::vector<uint8_t>(msg.begin(), msg.end()));
}
```

**Key Features**:
- **Real-time Processing**: Zero-copy memory operations for minimal latency
- **HUD Feedback**: Simultaneous local display updates
- **Conditional Transmission**: RF state checking prevents unnecessary broadcasts
- **Format Conversion**: Binary-to-ASCII conversion optimized for LoRa efficiency

#### BLE HID Processing Engine
The BLE processing system implements advanced filtering and mapping:

```cpp
// Throttle trigger processing with flood prevention
if (id(ble_hid_usage).state == "2_196") {
    float v = (x / 1023.0) * 1.0;
    if (std::abs(v - last_2_196) >= 0.7f) {  // Threshold filtering
        last_2_196 = v;
        // Generate optimized packet
        std::vector<uint8_t> packet;
        packet.push_back(0x02);  // Throttle axis ID
        uint8_t *val_bytes = reinterpret_cast<uint8_t*>(&v);
        packet.insert(packet.end(), val_bytes, val_bytes + sizeof(float));
        return packet;
    }
}
```

**Advanced Features**:
- **Static Value Caching**: Prevents redundant transmissions
- **Adaptive Thresholds**: Different sensitivity for different controls
- **Binary Packet Generation**: Optimized for LoRa bandwidth
- **Usage-based Filtering**: Smart axis exclusion (Y, Rz filtering)

### 2. LoRa Radio Configuration

#### SX1262 Optimization
```yaml
frequency: 904.2 MHz        # US ISM band, Z-Wave adjacent
bandwidth: 500.0 kHz        # High bandwidth for low latency
spreading_factor: 8         # Balanced range/speed trade-off
coding_rate: 8              # 4/8 coding for error resilience
tx_power: -6 dBm           # Conservative power for battery life
sync_word: 0xf7            # Custom sync word (non-LoRaWAN)
data_rate: 80MHz           # Controller clock rate
preamble_length: 10        # Extended preamble for reliability
```

**Performance Characteristics**:
- **Data Rate**: ~2.7 kbps theoretical (SF8, 500kHz BW)
- **Range**: 100m-2km depending on environment and obstacles
- **Latency**: ~40-80ms end-to-end (including processing)
- **Packet Size**: Optimized 5-15 byte payloads

### 3. Display and User Interface

#### OLED Display Integration
```yaml
# I2C Configuration
i2c:
  sda: GPIO17
  scl: GPIO18
  frequency: 400kHz
  scan: True

# Display with custom initialization
display:
  platform: ssd1306_i2c
  model: "SSD1306 128x64"
  reset_pin: GPIO21
  address: 0x3C
```

**Display Features**:
- **Status Monitoring**: Real-time connection status, battery level, signal strength
- **Control Feedback**: Live axis values and button states
- **System Information**: IP address, uptime, temperature
- **Visual Indicators**: LoRa activity, BLE connection status

**Custom Initialization Sequence**:
The LRTX implements a critical OLED initialization fix:
```cpp
// Boot sequence priority 1000 - Hardware initialization
pinMode(36, OUTPUT);      // VEXT control
digitalWrite(36, LOW);    // Enable display power
delay(100);              // Heltec-recommended delay
```

### 4. Power Management System

#### Battery Monitoring
```yaml
sensor:
  - platform: adc
    pin: GPIO1                    # Heltec V3 battery sense
    name: "MCU Battery Pack(HeltecV3)"
    attenuation: 12db            # Full-scale voltage range
    accuracy_decimals: 2
    filters:
      - multiply: 4.9            # Voltage divider compensation
    update_interval: 10s
```

**Power Features**:
- **Real-time Monitoring**: 10-second battery voltage updates
- **Voltage Scaling**: Automatic compensation for onboard voltage divider
- **Low Power Modes**: Configurable CPU frequency scaling
- **VEXT Control**: Peripheral power management for extended battery life

### 5. Web Server and Remote Management

#### Advanced Web Interface
```yaml
web_server:
  port: 80
  version: 3
  local: True
  sorting_groups:
    - id: sorting_group_robotcontrol
      name: "R030tC0ntr0l"
      sorting_weight: 10
    - id: sorting_group_bluetooth  
      name: "Bluetooth"
      sorting_weight: 20
    - id: sorting_group_longrangeradio
      name: "LongRangeRadio" 
      sorting_weight: 30
    - id: sorting_group_mcu
      name: "MCU"
      sorting_weight: 40
```

**Web Interface Features**:
- **Organized Layout**: Logical grouping of controls and sensors
- **Real-time Updates**: Live sensor data and control states
- **Remote Configuration**: Parameter adjustment without reflashing
- **Diagnostic Tools**: Connection status, signal strength, error rates

## Control Mapping and Protocol

### Input Source Comparison

| Feature | PC Serial (xinputToL03430T.py) | BLE HID (Xbox Controller) |
|---------|--------------------------------|---------------------------|
| **Latency** | ~20-40ms (optimized) | ~40-80ms (BLE overhead) |
| **Reliability** | Very High (wired) | High (wireless, auto-reconnect) |
| **Range** | USB-C cable length | ~10m BLE range |
| **Power** | PC-powered | Controller battery |
| **Precision** | 32-bit float precision | 16-bit integer precision |
| **Setup** | Python script required | Plug-and-play |

### Axis Mapping Specification

#### PC Serial Axis Mapping
| Axis ID | Source | Range | ESPHome Mapping | Purpose |
|---------|--------|-------|-----------------|---------|
| 0x01 | Left Trigger | 0.0 - 1.0 | A:1 | Reverse/Brake control |
| 0x02 | Right Trigger | 0.0 - 1.0 | A:2 | Forward throttle |
| 0x03 | Left Thumb X | -2.0 - +2.0 | A:3 | Steering/Servo control |
| 0x04 | Left Thumb Y | -2.0 - +2.0 | A:4 | Secondary Y-axis |
| 0x05 | Right Thumb X | -2.0 - +2.0 | A:5 | Camera pan/Turret |
| 0x06 | Right Thumb Y | -2.0 - +2.0 | A:6 | Camera tilt/Elevation |

#### BLE HID Usage Mapping
| HID Usage | Internal Mapping | Axis ID | Threshold | Purpose |
|-----------|------------------|---------|-----------|---------|
| "X" | Left stick horizontal | 0x03 | 0.3f | Steering control |
| "Z" | Right stick horizontal | 0x05 | 0.3f | Camera pan |
| "2_196" | Right trigger | 0x02 | 0.7f | Forward throttle |
| "2_197" | Left trigger | 0x01 | 0.7f | Reverse control |

### Button Protocol Specification

#### Button Event Structure
```cpp
// Binary button packet (4 bytes)
[0x42, button_id, pressed_state, 0x0A]

// Button ID mapping (XInput standard)
0-3:   DPad directions (Up, Down, Left, Right)
4-5:   Back/Start buttons
6-7:   Thumbstick clicks
8-9:   Shoulder buttons (LB, RB)
12-15: Face buttons (A, B, X, Y)
```

## Performance Tuning Guide

### 1. LoRa Transmission Optimization

#### Range vs. Latency Tuning
```yaml
# Close Range Configuration (<100m)
spreading_factor: 7         # Faster transmission
bandwidth: 500.0           # Maximum bandwidth
tx_power: -6               # Conservative power
# Expected latency: ~20-40ms

# Medium Range Configuration (100-500m)  
spreading_factor: 8         # Current default
bandwidth: 500.0           # High bandwidth maintained
tx_power: -2               # Slight power increase
# Expected latency: ~40-80ms

# Long Range Configuration (>500m)
spreading_factor: 9         # Enhanced sensitivity
bandwidth: 250.0           # Reduced bandwidth
tx_power: 0                # Increased power
# Expected latency: ~80-150ms
```

#### Flood Prevention Tuning
```yaml
# BLE threshold adjustments for different scenarios
# Precise Control (Indoor, close range)
threshold_thumbstick: 0.1f  # High sensitivity
threshold_trigger: 0.3f    # Moderate trigger response

# Standard Control (Current configuration)
threshold_thumbstick: 0.3f  # Balanced sensitivity  
threshold_trigger: 0.7f    # Conservative triggers

# Coarse Control (Long range, battery saving)
threshold_thumbstick: 0.5f  # Reduced sensitivity
threshold_trigger: 0.9f    # High trigger threshold
```

### 2. CPU Performance Optimization

#### Frequency Scaling
```yaml
# High Performance (200-240MHz)
board_build.f_cpu: 240000000L
# Benefits: Lowest latency, maximum processing power
# Drawbacks: Higher power consumption, increased heat

# Balanced Performance (200MHz - Default)
board_build.f_cpu: 200000000L  
# Benefits: Good balance of performance and efficiency
# Drawbacks: None significant

# Power Saving (160MHz)
board_build.f_cpu: 160000000L
# Benefits: Extended battery life, reduced heat
# Drawbacks: Slightly increased processing latency
```

#### Memory Management
```yaml
# UART buffer optimization
uart:
  rx_buffer_size: 0          # Minimum latency (current)
  # rx_buffer_size: 256      # Reliability mode
  # rx_buffer_size: 512      # High throughput mode
```

### 3. Input Processing Optimization

#### Heartbeat Filter Tuning
```yaml
# BLE sensor configuration
sensor:
  - platform: ble_client_hid
    filters:
      - heartbeat: 0.04s      # 25Hz (current - balanced)
      # - heartbeat: 0.02s    # 50Hz (high performance)
      # - heartbeat: 0.08s    # 12.5Hz (battery saving)
```

#### Serial Processing Efficiency
```cpp
// Optimize binary parsing for different use cases

// Low Latency Mode (disable HUD feedback)
if (axis_id == 0x01) {
    // Remove HUD calls for minimal latency
    id(lora)->send_packet(std::vector<uint8_t>(msg.begin(), msg.end()));
}

// Standard Mode (current - with HUD feedback)
if (axis_id == 0x01) {
    auto call = id(hud_reverse).make_call();
    call.set_value(value);
    call.perform();
    id(lora)->send_packet(std::vector<uint8_t>(msg.begin(), msg.end()));
}
```

## Troubleshooting Guide

### 1. Hardware Issues

#### OLED Display Not Working
**Symptoms**: Blank display, no boot messages
**Solutions**:
```yaml
# Verify VEXT initialization in boot sequence
on_boot:
  - priority: 1000
    then:
      - lambda: |-
          pinMode(36, OUTPUT);     # Critical for Heltec V3
          digitalWrite(36, LOW);   # Enable display power
          delay(100);             # Required delay
```

**Additional Checks**:
- Verify I2C wiring (GPIO17=SDA, GPIO18=SCL)
- Check I2C scan results in logs
- Ensure display reset pin (GPIO21) is not conflicting

#### USB-C Serial Communication Failures
**Symptoms**: Device not recognized as COM port, serial errors
**Solutions**:
```yaml
# Essential build flags for Windows compatibility
platformio_options:
  build_flags:
    - '-DARDUINO_USB_CDC_ON_BOOT=1'  # Critical for COM port
    - '-DARDUINO_heltec_wifi_lora_32_V3'
```

**Diagnostic Steps**:
1. Check Device Manager for CP210x USB to UART Bridge
2. Verify baud rate matching (460800)
3. Test with different USB-C cable (data-capable required)
4. Try lower baud rates (115200) for troubleshooting

#### LoRa Radio Not Transmitting
**Symptoms**: No LoRa activity indicators, packets not received
**Solutions**:
```yaml
# Verify SX1262 pin configuration
sx1262:
  clk_pin: 9     # SPI Clock
  mosi_pin: 10   # SPI MOSI  
  miso_pin: 11   # SPI MISO
  cs_pin: 8      # Chip Select
  irq_pin: 14    # Interrupt
  reset_pin: 12  # Reset
  busy_pin: 13   # Busy indicator
```

**Diagnostic Commands**:
```cpp
// Add to lambda for radio diagnostics
ESP_LOGI("sx1262", "Radio status: %d", radio_status);
ESP_LOGI("sx1262", "Frequency: %.1f MHz", frequency);
```

### 2. Communication Issues

#### High Latency Between Input and LoRa
**Symptoms**: Noticeable delay between controller input and transmission
**Root Causes & Solutions**:

1. **UART Buffer Accumulation**:
   ```yaml
   uart:
     rx_buffer_size: 0    # Set to zero for minimum latency
   ```

2. **BLE Processing Overhead**:
   ```yaml
   # Reduce heartbeat filter interval
   filters:
     - heartbeat: 0.02s   # Increase from 0.04s
   ```

3. **Script Execution Delays**:
   ```cpp
   // Remove unnecessary script calls
   // id(lora_tx_script).execute();  // Comment out if not needed
   ```

#### Packet Loss or Corruption
**Symptoms**: Missing inputs, erratic behavior on remote end
**Solutions**:

1. **Increase LoRa Power**:
   ```yaml
   sx1262:
     tx_power: 0    # Increase from -6 dBm
   ```

2. **Improve Error Correction**:
   ```yaml
   sx1262:
     coding_rate: 6    # Increase from 8 (4/6 vs 4/8)
   ```

3. **Reduce Transmission Rate**:
   ```cpp
   // Increase thresholds to reduce packet frequency
   if (std::abs(v - last_value) >= 0.5f) {  // Increase from 0.3f
   ```

### 3. Power and Battery Issues

#### Rapid Battery Drain
**Symptoms**: Short operating time, excessive heat
**Solutions**:

1. **CPU Frequency Reduction**:
   ```yaml
   platformio_options:
     board_build.f_cpu: 160000000L  # Reduce from 200MHz
   ```

2. **WiFi Power Management**:
   ```yaml
   wifi:
     power_save_mode: light    # Enable power saving
   ```

3. **Reduce Transmission Power**:
   ```yaml
   sx1262:
     tx_power: -10   # Further reduce for battery savings
   ```

#### Inaccurate Battery Readings
**Symptoms**: Wrong voltage readings, sudden power-offs
**Solutions**:
```yaml
# Calibrate ADC multiplier
sensor:
  - platform: adc
    filters:
      - multiply: 4.85    # Adjust based on actual voltage measurement
      - median:           # Add filtering for stability
          window_size: 5
```

### 4. BLE Connection Issues

#### Xbox Controller Won't Connect
**Symptoms**: No BLE client connection, pairing failures
**Solutions**:

1. **Verify MAC Address**:
   ```yaml
   ble_client:
     - mac_address: !secret xboxblue  # Ensure correct MAC
   ```

2. **Reset BLE Stack**:
   ```cpp
   // Add to button for BLE reset
   - esp32_ble_tracker.stop_scan
   - delay: 2s
   - esp32_ble_tracker.start_scan
   ```

3. **Controller Pairing Mode**:
   - Hold Xbox + Sync buttons until rapid flashing
   - Ensure controller is in pairing mode during ESP32 boot

#### Frequent BLE Disconnections
**Symptoms**: Intermittent controller drops, reconnection cycles
**Solutions**:

1. **Improve Signal Strength**:
   - Reduce distance between controller and ESP32
   - Check for 2.4GHz interference (WiFi, microwaves)

2. **Connection Stability**:
   ```yaml
   ble_client:
     auto_connect: true     # Ensure auto-reconnection enabled
   ```

3. **Scan Parameter Optimization**:
   ```yaml
   esp32_ble_tracker:
     scan_parameters:
       continuous: False    # Reduce interference with connection
   ```

## Performance Monitoring

### Key Metrics to Monitor

#### 1. Communication Health
```yaml
# Monitor these sensors for system health
- LoRa RSSI: Signal strength to remote nodes
- LoRa SNR: Signal-to-noise ratio quality  
- BLE RSSI: Controller connection strength
- WiFi Signal: Network connectivity strength
```

#### 2. System Resources
```yaml
# Resource utilization indicators
- MCU Temperature: Thermal management
- Battery Voltage: Power system health
- Memory Usage: Available heap space
- CPU Usage: Processing load
```

#### 3. Input Processing Stats
```cpp
// Add custom sensors for monitoring
sensor:
  - platform: template
    name: "Packets Per Second"
    id: packet_rate
    # Updated via lambda in main processing loop
    
  - platform: template  
    name: "Processing Latency"
    id: proc_latency
    unit_of_measurement: "ms"
```

### Diagnostic Commands

#### Enable Debug Logging
```yaml
logger:
  level: DEBUG
  logs:
    sx1262: DEBUG          # LoRa radio debugging
    ble_client_hid: DEBUG  # BLE HID debugging
    uart: DEBUG            # Serial communication debugging
```

#### Performance Profiling
```cpp
// Add timing measurements to critical paths
auto start_time = millis();
// ... processing code ...
auto end_time = millis();
ESP_LOGI("perf", "Processing time: %lu ms", end_time - start_time);
```

## Advanced Configuration

### Custom Input Sources

#### Adding New Serial Protocols
```cpp
// Extend UART debug sequence for new packet types
if (bytes.size() == X && bytes[0] == 0xXX) {
    // Custom packet processing
    uint8_t custom_id = bytes[1];
    // ... process custom data ...
    id(lora)->send_packet(custom_packet);
}
```

#### Multiple Controller Support
```yaml
# Additional BLE clients for multiple controllers
ble_client:
  - id: ble_client_1
    mac_address: !secret xbox_controller_1
  - id: ble_client_2  
    mac_address: !secret xbox_controller_2
```

### Environmental Adaptations

#### Outdoor Operation Mode
```yaml
# Optimized for long-range outdoor use
sx1262:
  spreading_factor: 9      # Extended range
  tx_power: 3             # Maximum power
  bandwidth: 250.0        # Improved penetration

# Reduced update rates for battery conservation
sensor:
  filters:
    - heartbeat: 0.1s     # 10Hz update rate
```

#### Indoor High-Performance Mode  
```yaml
# Optimized for low-latency indoor use
sx1262:
  spreading_factor: 7      # High speed
  tx_power: -10           # Minimal power needed
  bandwidth: 500.0        # Maximum bandwidth

# High update rates for responsive control
sensor:
  filters:
    - heartbeat: 0.02s    # 50Hz update rate
```

## Future Enhancement Roadmap

### Planned Features
- **Multi-hop LoRa Networks**: Mesh networking capabilities
- **Encrypted Communications**: AES encryption for secure control
- **Adaptive Power Control**: Dynamic power adjustment based on RSSI
- **Machine Learning**: Predictive input processing for reduced latency
- **Cloud Integration**: Remote monitoring and configuration via MQTT
- **Custom HUD Themes**: Configurable display layouts and graphics

### Extension Points
- **Additional Radio Modules**: Support for other LoRa frequencies
- **Alternative Input Methods**: Support for other game controllers
- **Protocol Bridges**: Integration with other automation systems
- **Mobile App Interface**: Smartphone-based control and monitoring

---

*Last Updated: June 29, 2025*  
*Part of the l0r430t(LORABOT) Framework - ManRiver Project*
