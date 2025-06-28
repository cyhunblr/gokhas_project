# GokHAS Interface

This advanced PyQt6-based GUI provides comprehensive control and monitoring for the GokHAS robotic turret system. Features include real-time STM32 communication, intelligent calibration system, and seamless ROS integration for both simulation and hardware modes.

## 🚀 Latest Features (2025)

### Enhanced Calibration System
- **STM32 Response-Based Calibration**: No more fixed timers - waits for actual hardware confirmation
- **Intelligent Timeout Handling**: 15-second timeout with graceful error handling
- **Visual State Management**: 
  - 🔴 Default (Ready) → 🟡 Processing (Pulsing) → 🟢 Completed (Success)
  - ⚠️ Timeout (Returns to Default with error message)
- **Concurrent Protection**: Prevents multiple simultaneous calibrations

### Advanced Communication
- **Bidirectional UART**: Real-time communication with STM32 microcontroller
- **Simulation Mode**: Complete testing environment without hardware
- **Error Recovery**: Automatic reconnection and fault tolerance
- **Message Validation**: Robust protocol with status confirmation

### Configuration Management
- **Centralized Parameters**: All settings in `config/params.yaml`
- **Dynamic Loading**: ROS parameter integration with live updates
- **Mode Switching**: Easy simulation/hardware mode toggle

## Features

- **Real-Time Video Display**  
  Subscribes to YOLOv8 processed video from `/yolov8/image_raw` ROS topic with detection overlays
  
- **Intelligent Joint & Effector Control**  
  - Precision **Position** control (–135° to +135°) and **Power** management (0% to 100%)
  - Individual activation buttons for `Joint1`, `Joint2`, and `Effector`
  - Real-time value display with instant ROS topic publishing
  - State-aware control locking during calibration
  
- **System Status Management**  
  - **ACTIVATION Toggle**: DEACTIVATED ↔ ACTIVATED with STM32 confirmation
  - **MODE Toggle**: MANUAL ↔ AUTONOMOUS with control state management
  - Visual feedback with color-coded status indicators
  
- **Next-Generation Calibration System**  
  - **Hardware-Synchronized**: Waits for STM32 `calibStatus: 1` confirmation
  - **Timeout Protection**: 15-second timeout prevents infinite waiting
  - **Visual Feedback**: Pulsing yellow animation during processing
  - **Error Handling**: Graceful recovery from communication failures
  - **State Locking**: Prevents conflicts during calibration process
  
- **Advanced Theme System**  
  - **Light/Dark Themes**: Professional styling with dynamic switching
  - **Responsive Design**: Auto-adapting layouts and component sizing
  - **Theme Persistence**: Remembers user preferences
  
- **Professional UI Features**  
  - **Enhanced LOG Panel**: Timestamped entries with categorized messages
  - **Context Help**: Interactive help system with tooltips
  - **Debug Tools**: Border toggle and performance monitoring
  - **Resource Management**: Efficient memory usage and cleanup

## Installation

1. **Prerequisites:**
   ```bash
   # Ensure ROS Noetic is installed
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   
   # Install Python dependencies
   sudo apt-get install python3-pyqt6 python3-pyqt6.qtcore python3-pyqt6.qtgui python3-pyqt6.qtwidgets
   pip3 install pyserial opencv-python rospkg
   ```

2. **Build the workspace:**
   ```bash
   cd ~/bitirme_ws
   catkin build gokhas_interface
   source devel/setup.bash
   ```

## Configuration

The system uses a centralized configuration file: `config/params.yaml`

### Key Parameters:

```yaml
# STM32 Communication
stm32:
  simulation_mode: false          # Toggle hardware/simulation mode
  serial_port: "/dev/ttyUSB0"     # Hardware serial port
  baud_rate: 9600                 # Communication speed
  timeout: 1.0                    # Serial timeout

# System Timing
system:
  activation_timeout: 5000        # STM32 activation timeout (ms)
  calibration_timeout: 15000      # Calibration response timeout (ms)
  simulation_response_delay: 1000 # Simulation delay (ms)

# Joint Control
joints:
  position_range:
    min: -135                     # Minimum joint angle (degrees)
    max: 135                      # Maximum joint angle (degrees)
  power_range:
    min: 0                        # Minimum power (%)
    max: 100                      # Maximum power (%)
```

## Usage

### Basic Launch
```bash
# Standard launch with hardware
roslaunch gokhas_interface gokhas_interface.launch

# Simulation mode (no hardware required)
roslaunch gokhas_interface gokhas_interface.launch simulation:=true
```

### Testing Calibration System
```bash
# Test calibration timeout functionality
rosrun gokhas_interface test_calibration_timeout.py
```

### Interface Guide

- **Manual Control**: Use sliders and buttons in MANUAL mode for direct joint control
- **Calibration**: Click calibration buttons (one at a time) and wait for green completion
- **Theme Switching**: Use ☀️/🌙 buttons in bottom-right corner
- **Help**: Click the **?** button on camera view for manual mode instructions
- **Logs**: Monitor system activity in the LOG panel
- **Shutdown**: Use the red close button for safe application exit

## Project Structure

```
gokhas_interface/
├── launch/
│   └── interface.launch           # ROS launch file
├── scripts/
│   ├── launch_interface.py        # Main GUI launcher with signal handling
│   ├── camera_publisher.py        # Test video publisher
│   └── src/                       # Source code directory
│       ├── __init__.py
│       ├── main.py                # Entry point
│       ├── ui/                    # User Interface components
│       │   ├── __init__.py
│       │   ├── main_window.py     # Main Qt6 GUI window
│       │   ├── control_handlers.py # Button logic and event handlers
│       │   └── resources/         # UI resources and styling
│       │       ├── __init__.py
│       │       ├── resource.py    # Resource manager
│       │       ├── images/        # Icons and graphics
│       │       │   ├── closeApp.png
│       │       │   └── logo.png
│       │       └── styles/        # QSS stylesheet files
│       │           ├── calibration_buttons.qss
│       │           ├── close_button.qss
│       │           ├── dark_theme.qss
│       │           ├── joint_buttons.qss
│       │           ├── labels.qss
│       │           ├── light_theme.qss
│       │           ├── theme_buttons.qss
│       │           └── toggle_buttons.qss
│       └── ros/                   # ROS Integration
│           ├── __init__.py
│           └── ros_bridge.py      # ROS-Qt bridge with image processing
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

## Architecture

### Component Overview

- **MainWindow**: Core Qt6 interface with layout management
- **ControlHandlers**: Business logic for all button interactions
- **ROSBridge**: ROS topic/service integration with Qt signals
- **ResourceManager**: Theme and asset management system
- **HeightSyncWidget**: Custom widget for responsive layouts

### Theme System

The interface supports dynamic theming through:
- **QSS Files**: Component-specific stylesheets
- **Resource Manager**: Centralized theme switching
- **Visual Feedback**: Real-time UI updates

### ROS Topics (Planned)

- `/yolov8/image_raw` - Video input (subscribed)
- `/joint_control` - Joint commands (published)
- `/joint_position` - Position commands (published)
- `/joint_power` - Power commands (published)
- `/system_activation` - Activation status (published)
- `/system_mode` - Operation mode (published)

## Development

### Adding New Features

1. **UI Components**: Add to `main_window.py` with appropriate layout
2. **Business Logic**: Implement handlers in `control_handlers.py`
3. **Styling**: Create/update QSS files in `resources/styles/`
4. **ROS Integration**: Extend `ros_bridge.py` for new topics/services

### Debugging

- Use `show_borders = True` for layout debugging
- Check LOG panel for runtime information
- Monitor console output for ROS connection status

## Contributing

Feel free to open issues or pull requests for new features, improvements, or bug fixes.

## License

This project is licensed under the MIT License.

**Copyright (c) 2025 Ahmet Ceyhun Bilir**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

**Author:** Ahmet Ceyhun Bilir <ahmetceyhunbilir16@gmail.com>  
**Project:** GokHAS - Advanced PyQt6 Interface for Robotic Turret System