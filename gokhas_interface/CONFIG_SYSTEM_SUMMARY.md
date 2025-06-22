# GokHAS Interface Configuration System

## Overview
Successfully implemented a comprehensive configuration system for the gokhas_interface package that allows dynamic configuration through ROS parameters.

## Completed Features

### 1. Configuration Directory and Files
- **Created**: `/home/cyhunblr/bitirme_ws/src/gokhas_project/gokhas_interface/config/params.yaml`
- **Structure**: Well-organized YAML configuration with sections for STM32, system, joints, and logging parameters

### 2. STM32 Communication Configuration
```yaml
stm32:
  simulation_mode: false          # Enable/disable simulation mode
  serial_port: "/dev/ttyUSB0"     # Configurable serial port
  baud_rate: 9600                 # Configurable baud rate
  timeout: 1.0                    # Serial timeout
```

### 3. System Parameters
```yaml
system:
  activation_timeout: 5000        # Activation timeout in milliseconds
  calibration_duration: 5000      # Calibration duration in milliseconds
  simulation_response_delay: 1000 # Simulation response delay
```

### 4. Joint Control Parameters
```yaml
joints:
  position_range:
    min: -135
    max: 135
  power_range:
    min: 0
    max: 100
```

### 5. Integration with Control Handlers
- **Created**: `ControlHandlersConfig` class in `control_handlers.py`
- **Features**:
  - Safe parameter loading with type conversion
  - Error handling for missing or invalid parameters
  - Automatic fallback to default values
  - Logging of configuration status

### 6. UART Communication Integration
- **Modified**: `uart_com.py` to use configuration parameters
- **Created**: `UartConfig` class for UART-specific parameters
- **Features**:
  - Dynamic serial port configuration
  - Configurable baud rate and timeout
  - Safe parameter type conversion

### 7. Launch File Integration
- **Updated**: `interface.launch` to load parameters before starting nodes
- **Added**: `test_config.launch` for testing configuration system

## Configuration Classes

### ControlHandlersConfig
```python
class ControlHandlersConfig:
    # Default values
    SIMULATION_MODE = True
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200
    # ... other parameters
    
    @classmethod
    def load_config(cls):
        # Load from ROS parameters with safe type conversion
```

### UartConfig
```python
class UartConfig:
    # Default values
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 9600
    TIMEOUT = 1.0
    
    @classmethod
    def load_config(cls):
        # Load UART-specific parameters
```

## Key Improvements

### 1. Replaced Global Variables
- **Before**: `STM32_SIMULATION_MODE = True` (hardcoded global)
- **After**: `ControlHandlersConfig.SIMULATION_MODE` (configurable class variable)

### 2. Dynamic Serial Port Configuration
- **Before**: Hardcoded `/dev/ttyUSB0` in uart_com.py
- **After**: Configurable through ROS parameters

### 3. Safe Parameter Loading
- Type checking and conversion for numeric parameters
- Graceful fallback to defaults on errors
- Comprehensive error logging

### 4. Centralized Configuration
- All system parameters in one YAML file
- Consistent configuration interface across modules
- Easy modification without code changes

## Testing Results

### Configuration Loading Test
```
=== Testing Interface Configuration ===
Before loading config:
  Simulation Mode: True
  Serial Port: /dev/ttyUSB0
  Baud Rate: 115200

After loading config:
  Simulation Mode: False          # ✓ Loaded from params.yaml
  Serial Port: /dev/ttyUSB0      # ✓ Loaded from params.yaml  
  Baud Rate: 9600                # ✓ Loaded from params.yaml
```

### ROS Parameter Integration
```
PARAMETERS
 * /stm32/simulation_mode: False
 * /stm32/serial_port: /dev/ttyUSB0
 * /stm32/baud_rate: 9600
 * /stm32/timeout: 1.0
 * /system/activation_timeout: 5000
 * /system/calibration_duration: 5000
```

## Usage

### 1. Modify Configuration
Edit `/home/cyhunblr/bitirme_ws/src/gokhas_project/gokhas_interface/config/params.yaml`

### 2. Launch Interface
```bash
roslaunch gokhas_interface interface.launch
```

### 3. Test Configuration
```bash
roslaunch gokhas_interface test_config.launch
```

## Benefits

1. **Flexibility**: Easy to switch between simulation and real hardware
2. **Maintainability**: All configuration in one place
3. **Robustness**: Safe parameter loading with error handling
4. **Consistency**: Unified configuration approach across modules
5. **Debugging**: Clear logging of configuration status

## Files Modified

1. **NEW**: `/config/params.yaml` - Main configuration file
2. **UPDATED**: `/scripts/src/ui/control_handlers.py` - Added ControlHandlersConfig class
3. **UPDATED**: `/scripts/uart_com.py` - Added UartConfig class  
4. **UPDATED**: `/launch/interface.launch` - Added parameter loading
5. **NEW**: `/launch/test_config.launch` - Configuration testing
6. **NEW**: `/scripts/test_config.py` - Configuration test script

## Configuration System Status: ✅ COMPLETE AND TESTED
