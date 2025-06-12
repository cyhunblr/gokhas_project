# GOKHAS Communication Package

This package provides a ROS Noetic-based UART communication system for a robotic arm with airsoft capability. It is designed for bidirectional data exchange with STM32 microcontroller via serial port.

## Features

- Bidirectional UART communication with STM32
- Trigger-based transmission (only when commands received)
- Continuous data reception from STM32
- Robot joint position and speed control
- Airsoft system control and feedback
- Real-time data exchange (100 Hz)
- Compact packet-based protocol (8 bytes)
- ROS topic-based communication
- Fault-tolerant communication with simulation mode

## Package Structure

```
gokhas_communication/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── ControlMessage.msg    # Communication and calibration control (2 bytes)
│   └── JointMessage.msg      # Joint and airsoft control (6 bytes)
└── scripts/
    └── uart_com.py          # Main UART communication script
```

## Message Structures

### ControlMessage.msg (2 bytes)
```
uint8 comStatus          # Communication status (0=inactive, 1=active)
uint8 calibStatus        # Calibration status (0=normal, 1=calibration_mode)
```

### JointMessage.msg (6 bytes)
```
uint8 control_bits      # Bit field: [bit1: airsoft_trigger, bit2: j1p_sign, bit3: j2p_sign, bit4-8: reserved]
                        # j1p_sign: 0=positive(+), 1=negative(-)
                        # j2p_sign: 0=positive(+), 1=negative(-)
uint8 j1p               # Joint 1 Position (0-135, sign in control_bits)
uint8 j1s               # Joint 1 Speed/Power (0-100 percent)
uint8 j2p               # Joint 2 Position (0-135, sign in control_bits)  
uint8 j2s               # Joint 2 Speed/Power (0-100 percent)
uint8 ap                # Airsoft Power (0-100 percent)
```

## Installation

### 1. Dependencies

```bash
sudo apt-get install ros-noetic-std-msgs
sudo apt-get install python3-serial
```

### 2. Building the Workspace

```bash
cd ~/bitirme_ws
catkin build gokhas_communication
source devel/setup.bash
```

### 3. UART Device Setup

Grant necessary permissions for USB-UART converter:

```bash
sudo chmod 666 /dev/ttyUSB0
# or permanently:
sudo usermod -a -G dialout $USER
```

## Usage

### Starting the Node

```bash
rosrun gokhas_communication uart_com.py
```

### Parameters

Hardcoded parameters defined in the script:

- **Port**: `/dev/ttyUSB0`
- **Baudrate**: `9600`
- **Data Bits**: `8`
- **Parity**: `None`
- **Stop Bits**: `1`
- **Timeout**: `0.1 second` (non-blocking)
- **Rate**: `100 Hz`
- **Packet Size**: `8 bytes`

### Topics

#### Subscriber Topics (Commands from UI)

- `/gokhas/control_commands` ([`ControlMessage`]) - Communication and calibration commands
- `/gokhas/joint_commands` ([`JointMessage`]) - Joint and airsoft control commands

#### Publisher Topics (Feedback from STM32)

- `/gokhas/control_status` ([`ControlMessage`]) - Communication and calibration status
- `/gokhas/joint_feedback` ([`JointMessage`]) - Joint positions and airsoft feedback
- `/gokhas/communication_active` ([`std_msgs/Bool`]) - Overall communication status

## Data Protocol

### Packet Format: `'BBBBBBBB'` (8 bytes total)

| Byte | Field | Description | Range |
|------|-------|-------------|-------|
| 0 | comStatus | Communication status | 0-1 |
| 1 | calibStatus | Calibration status | 0-1 |
| 2 | control_bits | Control bit field | 0-255 |
| 3 | j1p | Joint 1 position | 0-135 |
| 4 | j1s | Joint 1 speed/power | 0-100 |
| 5 | j2p | Joint 2 position | 0-135 |
| 6 | j2s | Joint 2 speed/power | 0-100 |
| 7 | ap | Airsoft power | 0-100 |

### Control Bits Structure
```
Bit 1: airsoft_trigger (0=OFF, 1=ON)
Bit 2: j1p_sign (0=positive, 1=negative)
Bit 3: j2p_sign (0=positive, 1=negative)
Bit 4-8: Reserved for future use
```

## Communication Flow

### 1. Trigger-Based Transmission
- Node waits for commands from UI topics
- When command received, sets transmission flags
- Transmits combined 8-byte packet to STM32
- Clears flags after successful transmission

### 2. Continuous Reception
- Continuously monitors serial port for incoming data
- Receives 8-byte packets from STM32
- Parses and publishes data to feedback topics
- Non-blocking operation with timeout

### 3. Simulation Mode
- Automatically activates when no serial port available
- Echoes transmitted data as received data
- Enables testing without hardware

## Example Usage

### Sending Joint Commands

```bash
# Publish joint command
rostopic pub /gokhas/joint_commands gokhas_communication/JointMessage "
control_bits: 5
j1p: 90
j1s: 50
j2p: 45
j2s: 75
ap: 80"
```

### Sending Control Commands

```bash
# Activate communication
rostopic pub /gokhas/control_commands gokhas_communication/ControlMessage "
comStatus: 1
calibStatus: 0"
```

### Monitoring Feedback

```bash
# Monitor joint feedback
rostopic echo /gokhas/joint_feedback

# Monitor communication status
rostopic echo /gokhas/communication_active
```

## Control Bits Examples

```
Binary: 00000001 → Airsoft ON, J1 positive, J2 positive
Binary: 00000010 → Airsoft OFF, J1 negative, J2 positive  
Binary: 00000100 → Airsoft OFF, J1 positive, J2 negative
Binary: 00000101 → Airsoft ON, J1 positive, J2 negative
```

## Architecture

```
User Interface
     ↓ (Commands)
ROS Topics (/gokhas/control_commands, /gokhas/joint_commands)
     ↓
UART Communication Node
     ↓ (8-byte packets)
STM32 Microcontroller
     ↓ (Feedback packets)
UART Communication Node
     ↓
ROS Topics (/gokhas/control_status, /gokhas/joint_feedback)
     ↓ (Feedback)
User Interface
```

## Troubleshooting

### Serial Port Issues

```bash
# List USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB*

# Check port permissions
sudo chmod 666 /dev/ttyUSB0

# Test serial connection
sudo minicom -D /dev/ttyUSB0 -b 9600
```

### ROS Node Debug

```bash
# Check node status
rosnode list
rosnode info /uart_communication_node

# Check topics
rostopic list
rostopic echo /gokhas/control_status
rostopic echo /gokhas/joint_feedback

# Check message definitions
rosmsg show gokhas_communication/ControlMessage
rosmsg show gokhas_communication/JointMessage
```

### Common Issues

1. **No Serial Port**: Node runs in simulation mode automatically
2. **Permission Denied**: Run `sudo chmod 666 /dev/ttyUSB0`
3. **No Data Received**: Check STM32 connection and baud rate
4. **Communication Not Active**: Send control command with `comStatus: 1`

## Performance

- **Communication Rate**: 100 Hz
- **Packet Size**: 8 bytes (highly optimized)
- **Latency**: < 10ms typical
- **Reliability**: Automatic error handling and recovery

## TODO
[ ] CRC implementation

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/new-feature`)
5. Create a Pull Request

## License

This project is licensed under [GNU AGPL v3](LICENSE) license.

## Contact

You can open issues for questions and bug reports.
