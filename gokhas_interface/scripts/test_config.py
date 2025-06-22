#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to verify configuration loading functionality
"""

import sys
import os
import rospy

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def test_interface_config():
    """Test the ControlHandlersConfig loading"""
    from ui.control_handlers import ControlHandlersConfig
    
    print("=== Testing Interface Configuration ===")
    print("Before loading config:")
    print(f"  Simulation Mode: {ControlHandlersConfig.SIMULATION_MODE}")
    print(f"  Serial Port: {ControlHandlersConfig.SERIAL_PORT}")
    print(f"  Baud Rate: {ControlHandlersConfig.BAUD_RATE}")
    print(f"  Timeout: {ControlHandlersConfig.SERIAL_TIMEOUT}")
    print(f"  Activation Timeout: {ControlHandlersConfig.ACTIVATION_TIMEOUT}")
    print(f"  Position Range: {ControlHandlersConfig.POSITION_MIN} to {ControlHandlersConfig.POSITION_MAX}")
    
    # Load configuration from ROS parameters
    ControlHandlersConfig.load_config()
    
    print("\nAfter loading config:")
    print(f"  Simulation Mode: {ControlHandlersConfig.SIMULATION_MODE}")
    print(f"  Serial Port: {ControlHandlersConfig.SERIAL_PORT}")
    print(f"  Baud Rate: {ControlHandlersConfig.BAUD_RATE}")
    print(f"  Timeout: {ControlHandlersConfig.SERIAL_TIMEOUT}")
    print(f"  Activation Timeout: {ControlHandlersConfig.ACTIVATION_TIMEOUT}")
    print(f"  Position Range: {ControlHandlersConfig.POSITION_MIN} to {ControlHandlersConfig.POSITION_MAX}")

def test_uart_config():
    """Test the UartConfig loading"""
    # Add the communication package path
    comm_path = os.path.join(os.path.dirname(__file__), '..', '..', 'gokhas_communication', 'scripts')
    sys.path.append(comm_path)
    
    from uart_com import UartConfig
    
    print("\n=== Testing UART Configuration ===")
    print("Before loading config:")
    print(f"  Serial Port: {UartConfig.SERIAL_PORT}")
    print(f"  Baud Rate: {UartConfig.BAUD_RATE}")
    print(f"  Timeout: {UartConfig.TIMEOUT}")
    
    # Load configuration from ROS parameters
    UartConfig.load_config()
    
    print("\nAfter loading config:")
    print(f"  Serial Port: {UartConfig.SERIAL_PORT}")
    print(f"  Baud Rate: {UartConfig.BAUD_RATE}")
    print(f"  Timeout: {UartConfig.TIMEOUT}")

def print_ros_parameters():
    """Print all loaded ROS parameters"""
    print("\n=== ROS Parameters ===")
    try:
        # Get all parameters under /stm32
        stm32_params = rospy.get_param('/stm32', {})
        print("STM32 Parameters:")
        for key, value in stm32_params.items():
            print(f"  /stm32/{key}: {value} (type: {type(value).__name__})")
        
        # Get all parameters under /system
        system_params = rospy.get_param('/system', {})
        print("\nSystem Parameters:")
        for key, value in system_params.items():
            print(f"  /system/{key}: {value} (type: {type(value).__name__})")
        
        # Get all parameters under /joints
        joints_params = rospy.get_param('/joints', {})
        print("\nJoints Parameters:")
        for key, value in joints_params.items():
            if isinstance(value, dict):
                print(f"  /joints/{key}:")
                for subkey, subvalue in value.items():
                    print(f"    {subkey}: {subvalue} (type: {type(subvalue).__name__})")
            else:
                print(f"  /joints/{key}: {value} (type: {type(value).__name__})")
                
    except Exception as e:
        print(f"Error getting ROS parameters: {e}")

if __name__ == "__main__":
    try:
        # Initialize ROS node
        rospy.init_node('config_test_node', anonymous=True)
        
        print("Configuration System Test")
        print("=" * 50)
        
        # Print loaded ROS parameters
        print_ros_parameters()
        
        # Test interface configuration
        test_interface_config()
        
        # Test UART configuration
        test_uart_config()
        
        print("\nConfiguration test completed successfully!")
        
    except rospy.ROSInterruptException:
        print("ROS interrupted")
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
