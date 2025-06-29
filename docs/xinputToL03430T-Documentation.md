# xinputToL03430T.py - PC-to-LoRa Controller Bridge Documentation

![Python](https://img.shields.io/badge/Python-3.6+-blue)
![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey)
![License](https://img.shields.io/badge/License-MIT-green)

## Overview

The `xinputToL03430T.py` script serves as a critical PC-side component of the l0r430t(LORABOT) framework, acting as a bridge between Windows-based XInput controllers (Xbox controllers) and the LRTX (LoRa Transmit) node. This script captures real-time controller input and transmits it via USB-C serial connection to the ESP32-S3 based LRTX node for long-range LoRa transmission.

## Purpose and Role in l0r430t Framework

### Primary Functions
1. **XInput Controller Interface**: Direct access to Xbox 360/One controllers via Windows XInput API
2. **Serial Communication**: High-speed USB-C serial transmission to LRTX node (460800 baud)
3. **Real-time Processing**: Low-latency controller input processing with configurable polling rates
4. **Data Serialization**: Binary packet formatting optimized for LoRa transmission
5. **Noise Reduction**: Intelligent filtering and debouncing for analog inputs

### Integration Points
- **Input Source**: PC-hosted XInput controllers
- **Output Target**: LRTX node (Heltec WiFi LoRa 32 V3)
- **Communication Method**: USB-C serial at 460800 baud
- **Data Flow**: PC Controller → Serial → LRTX → LoRa → LRRX → UART P2P → Slave Node

## Technical Architecture

### Core Components

#### 1. XInput Interface Layer
```python
class XInputJoystick(event.EventDispatcher)
```
- **Purpose**: Windows XInput API wrapper using ctypes
- **Capabilities**: 
  - Controller enumeration and connection management
  - State polling and change detection
  - Battery level monitoring
  - Vibration feedback control
- **Data Structures**: 
  - `XINPUT_GAMEPAD`: Button states and analog values
  - `XINPUT_STATE`: Packet numbering for missed packet detection
  - `XINPUT_VIBRATION`: Motor speed control for haptic feedback

#### 2. Serial Communication System
```python
serial_queue = queue.Queue()
def serial_writer():
```
- **Threading Model**: Asynchronous queue-based serial transmission
- **Buffer Management**: Thread-safe message queuing
- **Error Handling**: Exception catching with graceful degradation
- **Performance**: Non-blocking serial writes to maintain real-time responsiveness

#### 3. Data Serialization Protocol
The script implements a custom binary protocol for efficient transmission:

**Button Events:**
```
Packet Format: [0x42, button_id, pressed_state, 0x0A]
- Header: 0x42 (Button identifier)
- Button ID: 0-15 (XInput button mapping)
- State: 0 (released) / 1 (pressed)
- Terminator: 0x0A (Line feed)
```

**Axis Events:**
```
Packet Format: [0x41, axis_id, float32_value, 0x0A]
- Header: 0x41 (Axis identifier)
- Axis ID: 1-6 (Trigger and thumbstick mapping)
- Value: IEEE 754 float32 (-1.0 to +1.0 normalized)
- Terminator: 0x0A (Line feed)
```

### Input Mapping and Processing

#### Button Mapping (XInput → Binary)
| XInput Button | Button ID | ESPHome Mapping | Function |
|---------------|-----------|-----------------|----------|
| DPad Up | 0 | B:0 | Directional control |
| DPad Down | 1 | B:1 | Directional control |
| DPad Left | 2 | B:2 | Directional control |
| DPad Right | 3 | B:3 | Directional control |
| Back/Select | 4 | B:4 | System functions |
| Start/Menu | 5 | B:5 | System functions |
| Left Stick Click | 6 | B:6 | Stick button |
| Right Stick Click | 7 | B:7 | Stick button |
| Left Bumper | 8 | B:8 | Shoulder buttons |
| Right Bumper | 9 | B:9 | Shoulder buttons |
| A Button | 12 | B:12 | Primary action |
| B Button | 13 | B:13 | Secondary action |
| X Button | 14 | B:14 | Tertiary action |
| Y Button | 15 | B:15 | Quaternary action |

#### Analog Axis Processing
| Axis Name | Axis ID | Range | Processing | Purpose |
|-----------|---------|-------|------------|---------|
| Left Trigger | 1 | 0.0 - 1.0 | Debounced, 2 decimal precision | Throttle control |
| Right Trigger | 2 | 0.0 - 1.0 | Debounced, 2 decimal precision | Secondary control |
| Left Thumb X | 3 | -1.0 - +1.0 | 2x multiplier, 3 decimal precision | Steering/Yaw |
| Left Thumb Y | 4 | -1.0 - +1.0 | 2x multiplier, 3 decimal precision | Forward/Backward |
| Right Thumb X | 5 | -1.0 - +1.0 | 2x multiplier, 3 decimal precision | Camera/Turret pan |
| Right Thumb Y | 6 | -1.0 - +1.0 | 2x multiplier, 3 decimal precision | Camera/Turret tilt |

### Advanced Features

#### 1. Noise Reduction and Debouncing
```python
# Trigger debouncing
threshold = 0.001  # Minimum change threshold
if abs(value - last_val) < threshold and value != 0:
    return  # Ignore noise

# Thumbstick filtering
thumb_threshold = 0.004  # Thumbstick sensitivity
```

**Benefits:**
- Reduces LoRa transmission overhead
- Prevents jitter in servo/motor control
- Extends battery life on remote nodes
- Improves control stability

#### 2. Haptic Feedback Integration
```python
@j.event
def on_axis(axis, value):
    if axis == "left_trigger":
        left_speed = value
    elif axis == "right_trigger":
        right_speed = value
    j.set_vibration(left_speed, right_speed)
```

**Features:**
- Trigger-to-vibration mapping
- Real-time haptic response
- Motor speed proportional to trigger pressure
- Enhanced user feedback

#### 3. Performance Optimization
```python
time.sleep(.040)  # 25Hz polling rate - LoRa magic number
```

**Optimization Strategies:**
- **Polling Rate**: 25Hz (40ms intervals) optimized for LoRa transmission timing
- **Queue-based Serial**: Non-blocking transmission prevents input lag
- **Selective Transmission**: Only send changed values to reduce bandwidth
- **Binary Protocol**: Compact packet structure for minimal overhead

## Installation and Setup

### Prerequisites
```bash
# Required Python packages
pip install pyglet pyserial
```

### System Requirements
- **OS**: Windows 10/11 (XInput API dependency)
- **Python**: 3.6 or higher
- **Hardware**: 
  - Xbox 360/One/Series controller (wired or wireless with adapter)
  - USB-C cable for LRTX connection
  - Compatible LRTX node (Heltec WiFi LoRa 32 V3)

### Configuration Steps

1. **Connect Hardware**:
   - Connect Xbox controller to PC (USB or wireless adapter)
   - Connect LRTX node via USB-C cable
   - Ensure LRTX node is running l0r430t-lrtx.yaml firmware

2. **Launch Script**:
   ```bash
   python xinputToL03430T.py
   ```

3. **Port Selection**:
   - Script will enumerate available COM ports
   - Select the port corresponding to your LRTX node
   - Typically shows as "CP210x USB to UART Bridge"

4. **Output Control**:
   - Choose whether to disable console output for performance
   - Recommended to disable for production use

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. Controller Not Detected
**Symptoms**: "No devices found" message
**Solutions**:
- Verify controller is properly connected and drivers installed
- Check Windows Device Manager for XInput-compatible devices
- Try different USB port or wireless adapter
- Ensure controller batteries are charged (wireless)
- Test controller with Windows Game Controller settings

#### 2. Serial Port Connection Failures
**Symptoms**: Serial write errors, connection timeouts
**Solutions**:
```python
# Check available ports
python -c "import serial.tools.list_ports; print([p.device for p in serial.tools.list_ports.comports()])"
```
- Verify LRTX node is connected and powered
- Check USB-C cable integrity (data-capable cable required)
- Ensure no other applications are using the serial port
- Try different baud rates if communication fails:
  ```python
  arduino = serial.Serial(port=port, baudrate=115200, timeout=.1)  # Fallback rate
  ```

#### 3. Input Lag or Missed Inputs
**Symptoms**: Delayed response, choppy control
**Solutions**:
- Reduce polling interval (with caution):
  ```python
  time.sleep(.020)  # Increase to 50Hz (test with LoRa capacity)
  ```
- Disable console printing for performance
- Close unnecessary background applications
- Check for USB bandwidth contention

#### 4. Excessive LoRa Traffic
**Symptoms**: Communication drops, range reduction
**Solutions**:
- Increase debounce thresholds:
  ```python
  threshold = 0.005  # Increase trigger threshold
  thumb_threshold = 0.008  # Increase thumbstick threshold
  ```
- Reduce thumbstick multiplier:
  ```python
  value = round(value * 1.5, 3)  # Reduce from 2x to 1.5x
  ```

#### 5. Serial Buffer Overflow
**Symptoms**: Serial write exceptions, data loss
**Solutions**:
- Increase serial timeout:
  ```python
  arduino = serial.Serial(port=port, baudrate=460800, timeout=.5)
  ```
- Implement queue size limiting:
  ```python
  if serial_queue.qsize() < 100:  # Prevent excessive queuing
      serial_queue.put(packet)
  ```

### Diagnostic Tools

#### 1. Connection Testing
```python
# Test controller connectivity
joysticks = XInputJoystick.enumerate_devices()
for j in joysticks:
    print(f"Device {j.device_number}: {j.is_connected()}")
    print(f"Battery: {j.get_battery_information()}")
```

#### 2. Serial Port Verification
```python
# Test serial communication
arduino = serial.Serial(port='COM_X', baudrate=460800, timeout=1)
arduino.write(b'TEST\n')
response = arduino.readline()
print(f"Response: {response}")
```

#### 3. Performance Monitoring
```python
# Monitor packet statistics
print(f"Received: {j.received_packets}, Missed: {j.missed_packets}")
reliability = j.received_packets / (j.received_packets + j.missed_packets)
print(f"Reliability: {reliability:.3f}")
```

## Performance Tuning

### Optimization Parameters

#### 1. Polling Rate Optimization
```python
# Standard rate (recommended)
time.sleep(.040)  # 25Hz - Balanced performance/latency

# High performance (test carefully)
time.sleep(.020)  # 50Hz - May overwhelm LoRa

# Conservative (high range scenarios)
time.sleep(.080)  # 12.5Hz - Extended range operation
```

#### 2. Noise Filtering Adjustment
```python
# Sensitive control (racing, precision)
threshold = 0.0005          # Hair-trigger response
thumb_threshold = 0.002     # Precise thumbstick control

# Standard control (general purpose)
threshold = 0.001           # Default settings
thumb_threshold = 0.004

# Coarse control (outdoor, high-noise)
threshold = 0.005           # Reduced LoRa traffic
thumb_threshold = 0.010     # Stable in windy conditions
```

#### 3. Serial Buffer Tuning
```python
# High-throughput configuration
arduino = serial.Serial(
    port=port, 
    baudrate=921600,           # Double speed (if supported)
    timeout=.05,               # Faster timeout
    write_timeout=.02          # Prevent blocking
)

# Reliable configuration
arduino = serial.Serial(
    port=port, 
    baudrate=460800,           # Tested stable rate
    timeout=.2,                # Conservative timeout
    write_timeout=.1           # Allow for retransmission
)
```

### Range vs. Responsiveness Trade-offs

| Parameter | Close Range (<100m) | Medium Range (100-500m) | Long Range (>500m) |
|-----------|-------------------|------------------------|-------------------|
| Polling Rate | 50Hz (20ms) | 25Hz (40ms) | 12.5Hz (80ms) |
| Trigger Threshold | 0.0005 | 0.001 | 0.005 |
| Thumbstick Threshold | 0.002 | 0.004 | 0.010 |
| Value Precision | 3 decimals | 3 decimals | 2 decimals |
| Multiplier | 2.0x | 2.0x | 1.5x |

### Advanced Configuration

#### 1. Environment-Specific Presets
```python
# Indoor/Short Range Preset
INDOOR_CONFIG = {
    'poll_rate': 0.020,        # 50Hz
    'trigger_threshold': 0.0005,
    'thumb_threshold': 0.002,
    'multiplier': 2.0,
    'precision': 3
}

# Outdoor/Long Range Preset  
OUTDOOR_CONFIG = {
    'poll_rate': 0.080,        # 12.5Hz
    'trigger_threshold': 0.005,
    'thumb_threshold': 0.010,
    'multiplier': 1.5,
    'precision': 2
}
```

#### 2. Dynamic Rate Adjustment
```python
# Adaptive polling based on range/RSSI feedback
def adjust_poll_rate(rssi_value):
    if rssi_value > -70:       # Strong signal
        return 0.020           # High rate
    elif rssi_value > -90:     # Medium signal  
        return 0.040           # Standard rate
    else:                      # Weak signal
        return 0.080           # Conservative rate
```

## Integration with l0r430t Ecosystem

### Data Flow Architecture
```
PC Controller → xinputToL03430T.py → USB-C Serial → LRTX Node → LoRa Radio
                     ↓
              [Binary Protocol]
                     ↓
         [0x41/0x42 Packet Format]
                     ↓
            [ESPHome UART Component]
                     ↓
          [LoRa Transmission Queue]
```

### ESPHome Integration Points
The script's output directly interfaces with the LRTX ESPHome configuration:

```yaml
# In l0r430t-lrtx.yaml
uart:
  - id: usb_uart
    tx_pin: GPIO43
    rx_pin: GPIO44
    baud_rate: 460800
    rx_buffer_size: 0
    debug:
      direction: RX
      after:
        delimiter: "\n"
```

### Packet Processing Flow
1. **PC Script**: Generates binary packets (0x41/0x42)
2. **LRTX UART**: Receives and validates packets
3. **ESPHome Lambda**: Parses button/axis data
4. **LoRa Transmission**: Forwards to LRRX via sx1262 component
5. **UART P2P**: LRRX relays to slave nodes
6. **Motor/Servo Control**: Final actuation on slave devices

## Future Enhancements

### Planned Features
- **Multiple Controller Support**: Support for up to 4 simultaneous controllers
- **Configuration Profiles**: Save/load different tuning presets
- **Real-time Monitoring**: GUI interface for connection status and performance metrics
- **Adaptive Rate Control**: Dynamic adjustment based on LoRa link quality
- **Custom Button Mapping**: User-configurable button assignments
- **Macro Support**: Programmable button sequences and combinations

### Extension Points
- **Plugin Architecture**: Modular input source support
- **Protocol Abstraction**: Support for multiple serial protocols
- **Cloud Integration**: Remote monitoring and control capabilities
- **Machine Learning**: Predictive input processing for reduced latency

---

*Last Updated: June 29, 2025*
*Part of the l0r430t(LORABOT) Framework - ManRiver Project*
