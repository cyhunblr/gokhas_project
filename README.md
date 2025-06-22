# GokHAS Project

GokHAS was developed as an undergraduate capstone project, featuring a custom two-axis turret integrated with an Airsoft system. A GUI-based centralized management interface provides a robust software architecture for target acquisition and engagement. On the embedded side, an STM32 microcontroller handles real-time control and telemetry over UART communication. This end-to-end design demonstrates seamless coordination between high-level command software and low-level embedded hardware.

**Latest Updates (2025):**
- ✅ **Enhanced Calibration System**: Real-time STM32 response-based calibration with timeout handling
- ✅ **Improved Communication**: Robust bidirectional UART communication with error handling
- ✅ **Advanced UI Controls**: Dynamic button states and visual feedback system
- ✅ **Configuration Management**: Centralized parameter system with ROS integration

## Project Overview

The GokHAS system consists of three main ROS packages:

- **[gokhas_perception](gokhas_perception/)** - Computer vision and perception stack for target detection and tracking using YOLOv8
- **[gokhas_communication](gokhas_communication/)** - Bidirectional UART communication interface with STM32 microcontroller
- **[gokhas_interface](gokhas_interface/)** - Advanced PyQt6-based GUI with real-time calibration and control system

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  GUI Interface  │◄──►│   ROS Messages   │◄──►│   Perception    │
│   (gokhas_      │    │ (gokhas_commu-   │    │  (gokhas_per-   │
│   interface)    │    │  nication)       │    │   ception)      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         v                       v                       v
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     PyQt6       │    │   STM32 MCU      │    │  Camera/Sensors │
│      GUI        │    │   (UART Comm)    │    │   (ZED Camera)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Prerequisites

### System Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- OpenCV 4.x
- PCL (Point Cloud Library)

### Hardware Requirements
- ZED Stereo Camera (or compatible RGB-D camera)
- STM32 Microcontroller
- Two-axis servo/stepper motor system
- Airsoft mechanism

### Dependencies

#### ROS Packages
```bash
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-image-transport
sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-vision-msgs
sudo apt-get install ros-noetic-image-view
sudo apt-get install ros-noetic-rviz
```

#### Python Dependencies
```bash
sudo apt-get install python3-pyqt6
sudo apt-get install python3-pyqt6.qtcore
sudo apt-get install python3-pyqt6.qtgui
sudo apt-get install python3-pyqt6.qtwidgets
```

#### ZED SDK (if using ZED camera)
Download and install from [Stereolabs ZED SDK](https://www.stereolabs.com/developers/release/)

## Installation

1. **Clone the repository:**
```bash
cd ~/catkin_ws/src
git clone <repository-url> gokhas_project
```

2. **Install dependencies:**
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**
```bash
catkin build
# or
catkin_make
```

4. **Source the workspace:**
```bash
source ~/catkin_ws/devel/setup.bash
```

## Package Details

### gokhas_perception

**Purpose:** Computer vision and perception stack for target detection and tracking

**Key Features:**
- YOLOv8-based object detection and segmentation
- Real-time 3D point cloud processing
- Advanced target tracking with multiple algorithms
- 2D/3D bounding box generation
- Custom message types for detection data

**Nodes:**
- `tracker_node` - Real-time 2D object detection and tracking
- `tracker_with_cloud_node` - 3D object detection with LiDAR integration

**Custom Messages:**
- `Detection2D` - 2D bounding box detection
- `Detection2DArray` - Array of 2D detections
- `YoloResult` - YOLO detection results with segmentation masks

**Dependencies:**
- Ultralytics YOLO models
- OpenCV (cv_bridge)
- PCL (pcl_ros)
- Image processing (image_transport, image_geometry)
- Vision messages (vision_msgs)

### gokhas_communication

**Purpose:** Bidirectional UART communication interface with STM32 microcontroller

**Key Features:**
- Real-time UART communication (100 Hz)
- Compact 8-byte packet protocol
- Trigger-based transmission system
- Joint position and speed control
- Airsoft system control and feedback
- Simulation mode for testing
- Fault-tolerant communication with error handling

**Message Types:**
- `ControlMessage` - Communication and calibration control (2 bytes)
- `JointMessage` - Joint and airsoft control (6 bytes)

**Topics:**
- `/control_topic` - Control and calibration commands
- `/joint_command_topic` - Joint movement commands

**Dependencies:**
- Python serial library
- ROS Python (rospy)
- Custom message generation

### gokhas_interface

**Purpose:** Advanced PyQt6-based GUI with real-time calibration and control system

**Key Features:**
- **Enhanced Calibration System:**
  - STM32 response-based calibration (no fixed timers)
  - Visual feedback: Default → Processing (Yellow/Pulsing) → Completed (Green)
  - Timeout handling (15s) for non-responsive hardware
  - Prevention of simultaneous calibrations
- **Real-time Control:**
  - Manual/autonomous mode switching
  - Individual joint control with position (-135° to +135°) and power (0-100%)
  - Live camera feed display with YOLOv8 detection overlay
- **Advanced UI:**
  - Dynamic theme system (Light/Dark)
  - Comprehensive logging panel with timestamps
  - Configuration management via ROS parameters
  - Responsive design with status-based color coding
- **Communication:**
  - ROS Bridge for seamless topic integration
  - Connection monitoring and status reporting
  - Safe shutdown with resource cleanup

**Configuration:**
- Centralized parameter system in `config/params.yaml`
- Simulation/real hardware mode switching
- Configurable timeouts and delays

**Dependencies:**
- PyQt6 framework
- ROS integration (rospy)
- OpenCV for image processing
- Image processing for display
- Custom message handling

## Usage

### 1. Launch the Camera Node (ZED Camera)
```bash
roslaunch zed_wrapper zed2.launch
```

### 2. Start the Perception Stack
```bash
roslaunch gokhas_perception perception.launch
```

### 3. Launch Communication Node
```bash
roslaunch gokhas_communication communication.launch
```

### 4. Start the GUI Interface
```bash
roslaunch gokhas_interface interface.launch
```

### Alternative: Launch Everything
```bash
roslaunch gokhas_project full_system.launch
```

## Configuration

### Camera Configuration
Edit camera parameters in:
```
gokhas_perception/config/camera_params.yaml
```

### Detection Parameters
Configure YOLO detection settings:
```
gokhas_perception/config/detection_params.yaml
```

### Communication Settings
Set UART and communication parameters:
```
gokhas_communication/config/comm_params.yaml
```

## Topics and Services

### Published Topics
- `/gokhas/detections` - Detection results
- `/gokhas/target_position` - Target coordinates
- `/gokhas/system_status` - System health status

### Subscribed Topics
- `/camera/image_raw` - Camera feed
- `/camera/depth/points` - Point cloud data
- `/gokhas/control_commands` - Manual control inputs

### Services
- `/gokhas/set_mode` - Switch between manual/auto modes
- `/gokhas/calibrate` - System calibration
- `/gokhas/emergency_stop` - Emergency shutdown

## Development

### Building Custom Messages
After adding new message files:
```bash
catkin build gokhas_perception
catkin build gokhas_communication
```

### Running Tests
```bash
catkin_make run_tests
```

### Code Style
- Follow ROS coding standards
- Use meaningful variable names
- Document all public functions
- Add appropriate error handling

## Troubleshooting

### Common Issues

1. **Camera not detected:**
   - Check USB connections
   - Verify ZED SDK installation
   - Check camera permissions

2. **Communication timeout:**
   - Verify STM32 connection
   - Check UART settings
   - Ensure correct baud rate

3. **GUI not responding:**
   - Check PyQt6 installation
   - Verify ROS node connections
   - Check message dependencies

### Debug Commands
```bash
# Check node status
rosnode list
rosnode info /gokhas_perception

# Monitor topics
rostopic echo /gokhas/detections
rostopic hz /camera/image_raw

# View system graph
rosrun rqt_graph rqt_graph
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

- **Ahmet Ceyhun Bilir** - *Lead Developer* - ahmetceyhunbilir16@gmail.com
- **Beyza Meltem Amanet** - *Communication Module* - meltem.aman@gmail.com

## Acknowledgments

- Bachelor's thesis project
- ROS community for excellent documentation
- OpenCV and PCL communities
- Stereolabs for ZED SDK support

## Project Status

This project is actively developed as part of a bachelor's thesis. Current focus areas:
- Improving detection accuracy
- Enhancing real-time performance
- Expanding GUI functionality
- Adding safety features

For questions or support, please contact the maintainers or create an issue in the repository.
