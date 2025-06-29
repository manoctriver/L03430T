# L0R430T-514V31 Hexapod Robot Documentation

## Project Overview

The **L0R430T-HEXAPOD** is an advanced ESPHome-based hexapod robot built on the l0r430t(lorabot) framework, based Freenove Hexapod Robot Kit platform. This project transforms the original Raspberry Pi-based kit into a sophisticated ESP32-C3 powered robot with inverse kinematics, PID stability control, and wireless connectivity.

### Key Features
- ✅ **18-DOF Motion**: 6 legs × 3 joints (Coxa, Femur, Tibia)
- ✅ **Inverse Kinematics Engine**: Real-time 3D coordinate to servo angle calculation
- ✅ **PID Stability Control**: IMU-based automatic balance correction with Kalman filtering
- ✅ **Web Interface**: ESPHome Web Server v3 with organized control groups
- ✅ **Long Range Control**: l0r430t(LORABOT) framework with LoRa P2P communication
- ✅ **Multi-Input Support**: Xbox controller (BLE) + PC XInput via UART P2P
- ✅ **Power Management**: Intelligent standby mode with configurable auto-sleep
- ✅ **Safety Systems**: Emergency stops, servo limits, and damage prevention
- ✅ **Advanced Telemetry**: Real-time battery monitoring and status feedback

---

## Hardware Architecture

### Core Components
| Component | Model | Function |
|-----------|-------|----------|
| **Main Controller** | ESP32-C3-DevKitM-1 | Primary processing unit |
| **Servo Drivers** | 2× PCA9685 (I2C: 0x40, 0x41) | 16-channel PWM control |
| **Servo Motors** | 18× Standard Servos | Joint actuation |
| **IMU Sensor** | MPU6050 | Accelerometer + Gyroscope |
| **Status LED** | WS2812B RGB Strip | Visual feedback |
| **Audio** | Piezo Buzzer (RTTTL) | Sound notifications |
| **Distance Sensor** | HC-SR04 Ultrasonic | Environment sensing |
| **Power Monitoring** | ADS7830 ADC | Battery voltage tracking |

### Pin Mapping (Freenove Shield → ESP32-C3)
```
I2C Communication:
├── SDA: Shield GPIO2 → ESP32-C3 GPIO4
├── SCL: Shield GPIO3 → ESP32-C3 GPIO5
└── PCA9685 Power: Shield GPIO4 → ESP32-C3 GPIO6 (inverted)

Peripherals:
├── Buzzer: Shield GPIO17 → ESP32-C3 GPIO7
├── Ultrasonic Trigger: Shield GPIO27 → ESP32-C3 GPIO8
├── Ultrasonic Echo: Shield GPIO22 → ESP32-C3 GPIO9
├── RGB LED: Shield GPIO18 → ESP32-C3 GPIO10
└── UART P2P: GPIO18 (TX), GPIO19 (RX) - 460800 baud
```

### Servo Layout & Mapping
```
Leg Configuration (viewed from above):
    6 ─────── 1
    │  BODY   │
    5 ─────── 2
    │         │
    4 ─────── 3

Each leg has 3 servos:
├── Coxa (c1-c6): Horizontal rotation
├── Femur (f1-f6): Vertical lift/lower
└── Tibia (t1-t6): Leg extension/retraction
```

**Servo Direction Convention (UNIFIED):**
- **All Coxa**: Positive = Clockwise rotation
- **All Femur**: Negative = Down/Standing position
- **All Tibia**: Negative = Extended/Standing position

---

## Software Architecture

### ESPHome Configuration Structure
```yaml
Device: l0r430t-514v31-hp (friendly_name: 514v31-hexapod)
Platform: ESP32-C3-DevKitM-1 (ESP-IDF framework)
CPU: 160MHz (optimized for P2P communication and servo control)
Flash: 4MB
Features:
├── Web Server (Port 80, Version 3, Local-only mode)
├── OTA Updates with password protection
├── WiFi + Captive Portal
├── API with encryption
├── Logger with configurable levels (Debug with selective filtering)
└── UART P2P via USB_SERIAL_JTAG interface
```

### Core Systems

#### 1. **Inverse Kinematics Engine**
- **Function**: Converts 3D target coordinates to servo angles
- **Algorithm**: Based on Freenove's coordinateToAngle function
- **Geometry**: l1=33mm (coxa), l2=90mm (femur), l3=110mm (tibia)
- **Safety**: Multi-layer bounds checking and servo limits

#### 2. **PID Stability Control**
- **Input**: MPU6050 IMU data (accelerometer + gyroscope)
- **Filter**: Kalman filter fusion (α=0.98)
- **Control**: Separate PID loops for pitch and roll
- **Output**: Real-time leg position adjustments

#### 3. **Power Management**
- **Standby Mode**: Automatic sleep after configurable timeout
- **Battery Monitoring**: Dual voltage sensors (servo + MCU power)
- **Power Control**: PCA9685 enable/disable for energy savings

#### 4. **Communication System**
- **UART P2P**: 460800 baud via USB_SERIAL_JTAG interface
- **Hardware**: ESP32-C3 GPIO18 (TX), GPIO19 (RX)
- **Protocol**: Custom sensor/binary sensor data exchange via uart_p2p_receiver
- **Upstream**: l0r430t(LORABOT) LRRX node (Seeed XIAO ESP32-S3 + LoRa HAT)
- **Controller Support**: Xbox BLE controllers + PC XInput via Python script
- **Status Feedback**: Real-time communication activity indicators

#### 5. **l0r430t(LORABOT) Integration**
- **Framework**: Long-range remote control system with LoRa radio (904.2 MHz)
- **LRTX Node**: Heltec WiFi LoRa 32 V3 with OLED display and controller input
- **LRRX Node**: Seeed XIAO ESP32-S3 with LoRa HAT for packet relay
- **514v31 Slave**: ESP32-C3 hexapod as end effector with full servo control
- **Input Methods**: BLE controllers, PC XInput mapping, web interface

---

## Standby & Power Management Systems

### **Intelligent Standby Mode**

The hexapod features an advanced power management system designed to extend battery life and prevent servo wear during idle periods.

#### **Standby Operation**
```yaml
Monitoring Interval: 30 seconds
Default Timeout: 5 minutes (configurable 1-20 minutes)
Activity Detection: User interactions, button presses, servo commands
Power Savings: ~70% reduction in current consumption
```

#### **Standby Entry Process**
1. **Activity Monitoring**: System tracks all user interactions
2. **Timeout Detection**: Enters standby after configured inactivity period
3. **Safe Positioning**: Executes RETRACT sequence for safe servo positions
4. **Power Reduction**: Disables PCA9685 servo drivers
5. **Status Indication**: Plays audio notification and dims RGB LED
6. **Low Power State**: Maintains WiFi and core systems only

#### **Standby Exit Process**
1. **Wake Triggers**: Manual wake button, web interface interaction, or remote command
2. **Power Restoration**: Re-enables PCA9685 servo drivers
3. **System Initialization**: 1-second power stabilization delay
4. **Visual Feedback**: Rainbow LED effect during wake sequence
5. **Safe Positioning**: Returns to RETRACT position
6. **Ready State**: Full functionality restored, activity timer reset

### **Battery Monitoring System**

Advanced dual-channel voltage monitoring provides comprehensive power status tracking for both servo and MCU power rails.

#### **Voltage Sensors**
| Channel | Purpose | Range | Update Rate | Hardware |
|---------|---------|-------|-------------|----------|
| **ADS7830 Ch0** | Servo Battery | 0-12V | 10 seconds | I2C ADC |
| **ADS7830 Ch1** | MCU Battery | 0-5V | 10 seconds | I2C ADC |

#### **Power Management Features**
- **Real-time Monitoring**: Continuous voltage tracking for both power rails
- **Low Battery Detection**: Configurable thresholds for automatic protection
- **Standby Integration**: Battery levels influence standby timeout behavior
- **Web Dashboard**: Live voltage display in ESPHome interface
- **Historical Tracking**: Voltage trends for maintenance planning

#### **Power States & Actions**
```yaml
Normal Operation (>11V servo, >4V MCU):
  - Full functionality enabled
  - Standard standby timeout (5 minutes)
  - All safety systems active

Low Battery (9-11V servo, 3.5-4V MCU):
  - Reduced standby timeout (2 minutes)
  - Conservative servo limits enforced
  - Audio warnings enabled

Critical Battery (<9V servo, <3.5V MCU):
  - Immediate standby mode entry
  - Emergency position sequence
  - Servo power disable
  - Continuous audio alerts
```

### **Energy Optimization Features**

#### **Smart Power Control**
- **PCA9685 Management**: Complete servo driver shutdown during standby
- **CPU Frequency**: Optimized 160MHz for balance of performance and efficiency
- **WiFi Optimization**: Local-only mode option to reduce power consumption
- **Servo Transitions**: Smooth 1-second transitions reduce current spikes

#### **Standby Configuration**
```yaml
Auto Standby Settings:
├── Enable/Disable: Toggle automatic standby mode
├── Timeout: 1-20 minutes (slider control)
├── Activity Detection: Button presses, servo commands, web interactions
└── Manual Override: Wake button and emergency controls always active

Power Monitoring:
├── Voltage Thresholds: Configurable warning and critical levels
├── Update Rate: 10-second intervals for battery status
├── Logging: Voltage history and power events
└── Integration: Standby decisions based on battery levels
```

#### **Standby Status Indicators**
- **RGB LED**: Off during standby, rainbow during wake
- **Audio Alerts**: Different tones for standby entry/exit and low battery
- **Web Interface**: Real-time standby status and battery levels
- **Activity Timer**: Shows time since last interaction

### **Standby Safety Features**

#### **Safe Position Management**
- **Entry Protocol**: Always executes RETRACT sequence before standby
- **Position Memory**: Optionally saves servo positions before sleep
- **Wake Protocol**: Returns to known safe position on wake
- **Emergency Override**: Manual controls remain active during standby

#### **System Protection**
- **Servo Protection**: Eliminates continuous power draw during idle
- **Temperature Management**: Prevents overheating during extended operation
- **Battery Protection**: Automatic standby on low voltage conditions
- **Data Integrity**: All configuration settings preserved during standby

---

## Control Parameters Documentation

#### **PID Control System**

The PID controller uses IMU data (MPU6050) with Kalman filter fusion to automatically balance the robot by adjusting leg positions.

#### **Kalman Filter Implementation**
- **Update Rate**: 50ms for enhanced stability
- **Fusion Algorithm**: α=0.98 accelerometer/gyroscope blend
- **Data Validation**: Automatic sanity checks for sensor values
- **Range Limits**: Accelerometer: ±50 m/s², Gyroscope: ±5000°/s
- **Initialization**: Auto-calibration on first valid sensor reading

#### **Pitch Control (Forward/Backward Balance)**

**Pitch Kp (Proportional Gain)**
- Range: 0.0 to 2.0 | Default: 0.5
- Function: Response aggressiveness to forward/backward tilt
- Use Cases:
  - 0.2: Gentle terrain correction
  - 0.5: Normal operation (Freenove standard)
  - 1.0: Dynamic walking/load carrying
  - 1.5+: Emergency response

**Pitch Ki (Integral Gain)**
- Range: 0.0 to 1.0 | Default: 0.0
- Function: Eliminates steady-state tilt errors
- Use Cases:
  - 0.0: Normal operation (prevents windup)
  - 0.1: Weight imbalance compensation
  - 0.3: Mechanical wear compensation

**Pitch Kd (Derivative Gain)**
- Range: 0.0 to 0.1 | Default: 0.0025
- Function: Reduces oscillation and overshoot
- Use Cases:
  - 0.001: Smooth surfaces
  - 0.0025: Normal operation
  - 0.01: Rough terrain
  - 0.05: High vibration environments

#### **Roll Control (Side-to-Side Balance)**
- Same parameter ranges and functions as Pitch
- Controls left/right balance instead of forward/backward

---

### Inverse Kinematics Parameters

#### **IK Servo Scale Factor**
- Range: 0.1 to 0.8 units/degree | Default: 0.5
- Function: Servo movement per calculated angle degree
- Formula: `servo_value = angle_degrees × scale_factor`
- Use Cases:
  - 0.2: Ultra-safe, minimal movement
  - 0.5: Conservative normal operation
  - 0.7: Dynamic movement
  - 0.8: Maximum range (use with caution)

#### **IK Safety Limit**
- Range: 10 to 50 units | Default: 30
- Function: Maximum allowed servo value (prevents damage)
- Effect: Creates range of -limit to +limit
- Use Cases:
  - 15: Post-damage ultra-safe mode
  - 30: Current conservative setting
  - 40: Normal tested operation
  - 50: Maximum range (validated hardware only)

#### **Coxa Range Limit**
- Range: 20 to 60 units | Default: 40
- Function: Limits leg horizontal rotation
- Purpose: Prevents leg-to-leg collisions
- Use Cases:
  - 25: Tight spaces
  - 40: Normal walking/turning
  - 55: Wide turning radius, crab walking

---

### Body Position Controls

#### **Body Height**
- Range: -50 to 0mm | Default: -25mm
- Function: Robot stance height
- Note: More negative = lower stance
- Use Cases:
  - -10mm: High stance, easy movement
  - -25mm: Normal walking height
  - -40mm: Low stance, stable but limited
  - -50mm: Crawling mode

#### **Body X/Y Position**
- X Range: -40 to +40mm | Default: 0mm (left/right shift)
- Y Range: -40 to +40mm | Default: 10mm (forward/backward shift)
- Use Cases:
  - Weight compensation
  - Terrain adaptation
  - Dynamic maneuvering

---

## Test Functions & Safety Features

### **Built-in Test Suite**

1. **Test IK - Very Conservative**
   - Coordinates: (0, 50, -10)
   - Purpose: Verify IK math safely
   - Expected: Values under ±30 limit

2. **Test IK - Lower Stance (SAFE)**
   - Coordinates: (0, 60, -20)
   - Purpose: Test stable low position
   - Expected: Lower body, extended legs

3. **Test Safe Movement**
   - Method: Direct servo commands (bypasses IK)
   - Purpose: Basic servo verification
   - Use: When IK system has issues

### **Safety Systems**

#### **Emergency Controls**
- 🚨 **Emergency IK Disable**: Instantly disables IK and returns to safe position
- **Wake Button**: Manual standby exit
- **Power Control**: PCA9685 enable/disable

#### **Automatic Protection**
- **Servo Limits**: Multi-layer value clamping
- **Reachability Checks**: IK workspace validation
- **Temperature Monitoring**: Prevent servo overheating
- **Activity Timeout**: Auto-standby after inactivity

---

## Operational Modes

### **Manual Control Mode**
- Individual servo control (c1-c6, f1-f6, t1-t6)
- Direct value input (-100 to +100 range)
- Real-time position feedback
- Safety limits enforced

### **IK Control Mode**
- 3D coordinate target setting
- Automatic servo calculation
- Workspace boundary enforcement
- Smooth interpolation

### **Stability Control Mode**
- IMU-based automatic balance
- PID correction loops
- Real-time leg adjustments
- Configurable response characteristics

### **Standby Mode**
- Low power consumption
- Servo power disable
- Activity monitoring
- Automatic wake on interaction

---

## Practical Tuning Profiles

### **Carpet Walking**
```yaml
Pitch/Roll Kp: 0.3      # Gentle correction
IK Scale: 0.4           # Smaller steps
Body Height: -20mm      # Higher stance
Safety Limit: 25        # Conservative
```

### **Stable Standing**
```yaml
Pitch/Roll Kp: 0.7      # Strong correction
Pitch/Roll Kd: 0.005    # Prevent wobble
Body Height: -35mm      # Lower, stable
Safety Limit: 35        # Allow correction range
```

### **Post-Damage Recovery**
```yaml
Safety Limit: 15        # Ultra-conservative
IK Scale: 0.2           # Minimal movement
Coxa Range: 25          # Prevent overextension
All PID: 0.2            # Gentle corrections
```

### **Dynamic Movement**
```yaml
Pitch/Roll Kp: 0.8      # Responsive
IK Scale: 0.6           # Larger steps
Body Height: -30mm      # Moderate stance
Safety Limit: 40        # Full movement
```

---

## Technical Achievements

### **Servo Direction Unification**
**Problem Solved**: Original code had asymmetric left/right servo mapping causing balance issues.

**Solution**: Unified all servos to use consistent negative values for standing positions, matching actual hardware behavior observed in STAND button (all femurs/tibias = -10).

**Result**: Eliminated roll bias, improved stability, simplified control logic.

### **Enhanced Safety Systems**
- **Conservative Scaling**: Default 0.5 units/degree prevents damage
- **Multi-layer Limits**: Global safety + per-servo + IK workspace bounds
- **Emergency Stops**: Instant disable with safe position return
- **Activity Monitoring**: Automatic standby prevents servo wear

### **Advanced Control Integration**
- **Real-time IK**: 50ms update rate with Kalman filtering
- **PID Tuning**: Configurable parameters with Freenove-compatible defaults
- **Web Interface**: Live parameter adjustment and monitoring
- **Remote Control**: l0r430t(LORABOT) integration with Xbox controller support

### **l0r430t Framework Integration Achievements**
- **Long Range Control**: LoRa radio communication up to several kilometers
- **Multi-Input Support**: Xbox BLE controllers + PC XInput seamless integration
- **Low Latency**: <50ms end-to-end control response via UART P2P
- **Robust Protocol**: 460800 baud with automatic error detection and recovery
- **Status Feedback**: Real-time communication health monitoring and visual indicators

---

## Research References & Inspiration

- [Freenove Hexapod Kit](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi) - Original hardware platform and kinematics base
- [JiroRobotics Hexapod v4](https://github.com/JiroRobotics/Hexapod_v4) - Advanced gait algorithms
- [Marcus CW Hexapod](https://github.com/marcuscw/Hexapod) - Servo control optimization
- [Kim Andre Pettersen's Implementation](https://github.com/KimAndrePettersen/Hexapod) - Stability control concepts
- [HexapodBot Project](https://github.com/temp3rr0r/HexapodBot) - Walking pattern research
- [Hexapod Project v1](https://github.com/hexapod-project/hexapod-v1) - Modular design principles
- [Spider Gait Controller](https://github.com/adityamanglik/Hexapod-Controller_Arduino) - Arduino-based control systems
- [MPU6050 Control Analysis](https://sites.duke.edu/memscapstone/decoding-the-mpu6050-and-the-control-action/) - IMU sensor fusion techniques
- [ESPHome Documentation](https://esphome.io/) - Platform-specific implementations
- [ESP32-C3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) - Hardware optimization guide

---

## Troubleshooting Guide

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Robot oscillates | High Kd values | Reduce Kd, increase damping |
| Slow tilt response | Low Kp values | Increase Kp gradually |
| Persistent lean | No integral correction | Increase Ki slightly |
| Servo overheating | High safety limits | Reduce Safety Limit and IK Scale |
| Legs collide | Wide coxa range | Reduce Coxa Range Limit |
| IK calculation fails | Target out of workspace | Check coordinate bounds |
| Standby not working | Activity detection issue | Check timeout settings and web interface interaction |
| Communication lost | UART P2P problems | Verify 460800 baud rate, GPIO18/19 connections, LRRX node status |
| LoRa range issues | Radio configuration | Check 904.2MHz frequency, antenna connections, power settings |
| Controller lag | High communication latency | Verify Xbox BLE pairing, reduce interference, check LRTX battery |
| Web interface slow | Network congestion | Enable local-only mode, check WiFi signal strength |

---

*Document Version: 2.0 - Last Updated: June 29, 2025*
*Updated with l0r430t(LORABOT) framework integration and current project specifications*