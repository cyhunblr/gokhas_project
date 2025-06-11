# GokHAS Interface

This Python & Qt6-based GUI simplifies interaction with ROS Noetic systems.  
Through a visual interface, you can manage **joint** and **effector** controls in manual or autonomous modes, perform calibration steps, and view real-time camera feed with advanced styling and theme support.

## Features

- **Real-Time Video Display**  
  Subscribes to YOLOv8 processed video from `/yolov8/image_raw` ROS topic and displays incoming frames
  
- **Manual Joint & Effector Control**  
  - Adjustable **Position** sliders (–135° to +135°) and **Power** sliders (0% to 100%) for each joint  
  - Individual control buttons for `Joint1`, `Joint2`, and `Effector`
  - Real-time slider value display and ROS topic integration
  
- **System Status Management**  
  - **ACTIVATION Toggle**: DEACTIVATED ↔ ACTIVATED (Red/Green)
  - **MODE Toggle**: MANUAL ↔ AUTONOMOUS (Yellow/Blue)
  - Status-based color coding for visual feedback
  
- **Advanced Calibration System**  
  - Dedicated calibration buttons for each joint
  - Visual feedback: Default (Gray) → Processing (Yellow/Pulsing) → Completed (Green)
  - Prevention of simultaneous calibrations with button locking
  - 5-second calibration process simulation
  
- **Theme System**  
  - **Light Theme** and **Dark Theme** support
  - Dynamic theme switching with visual button indicators
  - Comprehensive styling with CSS-like QSS files
  - Theme-aware UI components
  
- **Advanced UI Features**  
  - **LOG Panel**: Terminal-style logging with timestamps
  - **Help System**: Context-sensitive help with question mark button
  - **Responsive Design**: Auto-adjusting layouts and height synchronization
  - **Border Toggle**: Debug borders for development
  
- **ROS Integration**  
  - ROS Bridge for seamless topic/service communication
  - Connection monitoring and status reporting
  - Safe shutdown handling and resource cleanup

## Installation

1. Ensure Python 3 and ROS Noetic are installed.  
2. Install required Python packages:  
   ```bash
   pip install PyQt6 rospkg opencv-python
   ```  
3. Build the ROS package:  
   ```bash
   cd ~/bitirme_ws
   catkin build
   source devel/setup.bash
   ```

## Usage

Launch the interface and connect to all necessary ROS topics with:

```bash
roslaunch gokhas_interface interface.launch
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

MIT © 2025