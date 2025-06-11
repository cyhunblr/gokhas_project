# GOKHAS Communication Package

This package provides a ROS Noetic-based UART communication system. It is designed for data exchange with STM32 microcontroller via serial port.

## Features

- Bidirectional UART communication with STM32
- Motor position and speed control
- Real-time data exchange (100 Hz)
- Packet-based data protocol (26 bytes)
- ROS topic publication
- Fault-tolerant communication

## Package Structure

```
gokhas_communication/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── communication.msg      # Communication message definition
└── scripts/
    └── uart_com.py           # Main UART communication script
```

## Message Structure

The [`communication.msg`](msg/communication.msg) file contains the following fields:

```
# Mode States
bool comStat    # Communication status
bool modStat    # Mode status (Manual/Automatic)

# Receive States (from STM32)
int16 rJ1p      # Joint 1 position
int16 rJ1s      # Joint 1 speed
int16 rJ2p      # Joint 2 position
int16 rJ2s      # Joint 2 speed
int16 rAp       # Actuator position
bool rAt        # Actuator trigger

# Transmit States (to STM32)
int16 tJ1p      # Joint 1 position
int16 tJ1s      # Joint 1 speed
int16 tJ2p      # Joint 2 position
int16 tJ2s      # Joint 2 speed
int16 tAp       # Actuator position
bool tAt        # Actuator trigger

# Control States
bool cJ1        # Joint 1 control
bool cJ2        # Joint 2 control
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
- **Timeout**: `1 second`
- **Rate**: `100 Hz`
- **Packet Size**: `26 bytes`

### Topics

#### Published Topics

- `/com/communication_status` ([`std_msgs/Bool`]) - Communication status
- `/com/mode_status` ([`std_msgs/Bool`]) - Mode status
- (Code contains `motor_pub` definition but incomplete - needs fixing)

## Data Protocol

Packet format: `'BBhhhhBBhhhhBBBB'` (26 bytes total)

| Byte | Description | Type |
|------|-------------|------|
| 0 | Communication Status | uint8 |
| 1 | Mode Status | uint8 |
| 2-3 | Transmit J1 Position | int16 |
| 4-5 | Transmit J1 Speed | int16 |
| 6-7 | Transmit J2 Position | int16 |
| 8-9 | Transmit J2 Speed | int16 |
| 10 | Transmit Actuator Position | uint8 |
| 11 | Transmit Actuator Trigger | uint8 |
| 12-13 | Receive J1 Position | int16 |
| 14-15 | Receive J1 Speed | int16 |
| 16-17 | Receive J2 Position | int16 |
| 18-19 | Receive J2 Speed | int16 |
| 20 | Receive Actuator Position | uint8 |
| 21 | Receive Actuator Trigger | uint8 |
| 22 | Control J1 | uint8 |
| 23 | Control J2 | uint8 |

## Example Usage

### Reading Data from STM32

```python
# uart_com.py script automatically reads data from STM32
# and publishes to ROS topics
```

### Manual Command Sending

```python
# cmd_vel_callback function provides example implementation
# Converts geometry messages to motor commands
```

## Known Issues and Fixes

1. **Missing Publisher**: Line 116 has `motor_pub` defined but not created in `__init__` method
   
   ```python
   # To be added in __init__ method:
   self.motor_pub = rospy.Publisher('/gokhas/communication', communication, queue_size=1)
   ```

2. **Main Function**: Line 147 has `"_main_"` written, should be `"__main__"`

3. **Missing Subscriber**: `cmd_vel_callback` is defined but no subscriber created

## Troubleshooting

### Serial Port Issues

```bash
# List USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB*

# Check port permissions
sudo chmod 666 /dev/ttyUSB0
```

### ROS Node Debug

```bash
# Check node status
rosnode list
rosnode info /communication_node

# Check topics
rostopic list
rostopic echo /com/communication_status
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/new-feature`)
5. Create a Pull Request

## License

This project is licensed under [GNU AGPL v3](LICENSE) license.

## Contact

You can open issues for questions.