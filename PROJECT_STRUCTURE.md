# L0R430T Project Structure

This document describes the organized structure of the L0R430T LoRaBot project for GitHub.

## Directory Structure

```
l0r430t(lorabot)/
├── README.md                           # Main project documentation
├── LICENSE                             # MIT License
├── .gitignore                         # Git ignore rules
├── secrets.yaml.template              # Template for sensitive configuration
│
├── firmware/                           # ESPHome firmware configurations
│   ├── l0r430t-lrtx.yaml             # Transmitter Node (Xbox controller input)
│   ├── l0r430t-lrrx.yaml             # Receiver Node (robot control)
│   ├── l0r430t-514v31.yaml           # Main robot controller
│   └── l0r430t-514v31-hexapod-IK.yaml # Hexapod with inverse kinematics
│
├── components/                         # Custom ESPHome components
│   └── ads7830/                       # ADS7830 ADC sensor component
│       ├── ads7830.cpp               # C++ implementation
│       ├── ads7830.h                 # Header file
│       ├── ads7830_sensor.cpp        # Sensor implementation
│       ├── ads7830_sensor.h          # Sensor header
│       ├── component.yaml            # Component manifest
│       ├── sensor.py                 # Python platform integration
│       └── __init__.py               # Python package init
│
├── software/                          # Supporting software
│   └── xinputToL03430T.py            # Xbox controller to UART bridge
│
└── docs/                              # Project documentation
    ├── l0r430t(LORABOT).md           # Overall project documentation
    ├── l0r430t-514v31-hexapod-KI.md  # Hexapod kinematics documentation
    ├── l0r430t-lrtx-Documentation.md  # Transmitter documentation
    └── xinputToL03430T-Documentation.md # Software bridge documentation
```

## File Descriptions

### Root Files
- **README.md**: Comprehensive project overview, features, setup instructions
- **LICENSE**: MIT license with RF compliance notes
- **secrets.yaml.template**: Template showing required configuration secrets

### Firmware Directory
Contains all ESPHome YAML configuration files for different hardware nodes:

- **l0r430t-lrtx.yaml**: Transmitter node with Xbox controller Bluetooth integration
- **l0r430t-lrrx.yaml**: Receiver node for robot-side control and actuators
- **l0r430t-514v31.yaml**: Main robot controller configuration
- **l0r430t-514v31-hexapod-IK.yaml**: Advanced hexapod robot with inverse kinematics

### Components Directory
Custom ESPHome components not available in the standard library:

- **ads7830/**: Complete implementation of ADS7830 8-channel ADC sensor

### Software Directory
Supporting software and utilities:

- **xinputToL03430T.py**: Python script that bridges Xbox controller input to UART for ESPHome processing

### Documentation Directory
Detailed documentation for different aspects of the project:

- **l0r430t(LORABOT).md**: Project overview and architecture
- **l0r430t-514v31-hexapod-KI.md**: Hexapod inverse kinematics documentation
- **l0r430t-lrtx-Documentation.md**: Detailed transmitter node documentation
- **xinputToL03430T-Documentation.md**: Software bridge usage and configuration

## Usage Workflow

1. **Hardware Setup**: Follow main README.md for hardware connections
2. **Configuration**: Copy `secrets.yaml.template` to `secrets.yaml` and configure
3. **Firmware Upload**: Flash appropriate YAML files from `firmware/` directory
4. **Software Setup**: Run `software/xinputToL03430T.py` for controller input
5. **Documentation**: Refer to specific docs in `docs/` directory for detailed information

## Development

### Adding New Firmware
1. Create new YAML file in `firmware/` directory
2. Follow naming convention: `l0r430t-[device-name].yaml`
3. Update main README.md with new device description

### Adding Components
1. Create new directory in `components/` with component name
2. Implement ESPHome component following standard structure
3. Document in main README.md and create specific documentation in `docs/`

### Documentation Updates
1. Keep README.md as the main entry point
2. Add detailed documentation to appropriate files in `docs/`
3. Update this PROJECT_STRUCTURE.md when adding new directories or significant files

## GitHub Upload Priority

When uploading to GitHub, prioritize files in this order:

1. **Essential**: README.md, LICENSE, .gitignore
2. **Core firmware**: l0r430t-lrtx.yaml, l0r430t-lrrx.yaml
3. **Components**: Custom components directory
4. **Additional firmware**: Other YAML configurations
5. **Software**: Python scripts and utilities
6. **Documentation**: Detailed documentation files

This organization ensures the project is professional, maintainable, and accessible to other developers in the ESPHome and robotics communities.
