#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - UART Communication Node
========================================

Copyright (c) 2025 Ahmet Ceyhun Bilir
Author: Ahmet Ceyhun Bilir <ahmetceyhunbilir16@gmail.com>

This file is part of the GokHAS project, developed as a graduation thesis project.

License: MIT License

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

==============================================================================

UART Communication Node for GokHAS Project
==========================================

This script implements UART serial communication between ROS and STM32 microcontroller.
It handles bidirectional data transfer using separate packet structures for control 
commands and joint feedback. The system supports both real hardware communication
and simulation mode when no serial port is available.

Features:
- Bidirectional UART communication
- Separate packet structures for control and joint data
- Hardware simulation when no serial port available
- ROS parameter configuration
- Error handling and recovery
"""

import time
import rospy
from gokhas_communication.msg import ControlMessage, JointMessage
from std_msgs.msg import Bool
import serial
import struct

# Packet formats based on message definitions
CONTROL_FORMAT = 'BB'      # ControlMessage: 2 bytes
JOINT_FORMAT = 'BBBBBB'    # JointMessage: 6 bytes
TOTAL_FORMAT = 'BBBBBBBB'  # Combined: 8 bytes total

class UartConfig:
    """
    Configuration class for UART communication parameters
    
    Handles UART configuration loading from ROS parameters
    """
    
    # Default values
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 9600
    TIMEOUT = 1.0
    LOOP_RATE = 10  # Hz
    
    @classmethod
    def load_config(cls):
        """
        Load configuration from ROS parameters
        
        Loads UART communication settings from ROS parameter server
        """
        try:
            # Load STM32 communication parameters
            serial_port_param = rospy.get_param('/stm32/serial_port', '/dev/ttyUSB0')
            cls.SERIAL_PORT = str(serial_port_param) if serial_port_param is not None else '/dev/ttyUSB0'
            
            # Safe type conversion for numeric parameters
            baud_rate_param = rospy.get_param('/stm32/baud_rate', 9600)
            cls.BAUD_RATE = int(baud_rate_param) if isinstance(baud_rate_param, (int, float, str)) else 9600
            
            timeout_param = rospy.get_param('/stm32/timeout', 1.0)
            cls.TIMEOUT = float(timeout_param) if isinstance(timeout_param, (int, float, str)) else 1.0
            
            rospy.loginfo("UART Configuration loaded from ROS parameters")
            rospy.loginfo(f"Serial Port: {cls.SERIAL_PORT}")
            rospy.loginfo(f"Baud Rate: {cls.BAUD_RATE}")
            rospy.loginfo(f"Timeout: {cls.TIMEOUT}")
            
        except Exception as e:
            rospy.logwarn(f"Failed to load UART config parameters: {e}")
            rospy.loginfo("Using default UART configuration values")

class CommunicationClass:
    """
    Main UART communication class for STM32 interface
    
    Handles all serial communication with STM32 microcontroller
    """
    def __init__(self):
        """
        Initialize UART communication node
        
        Sets up ROS node, publishers, subscribers and serial communication
        """
        rospy.init_node('uart_communication_node', anonymous=True)
        
        # Load configuration from ROS parameters
        UartConfig.load_config()
        
        # Check simulation mode from ROS parameters
        self.simulation_mode = rospy.get_param('/stm32/simulation_mode', False)
        rospy.loginfo(f"STM32 Simulation Mode: {'ENABLED' if self.simulation_mode else 'DISABLED'}")

        # Publishers - Publish data received from STM32
        self.control_pub = rospy.Publisher('/gokhas/control_status', ControlMessage, queue_size=1)
        self.joint_pub = rospy.Publisher('/gokhas/joint_feedback', JointMessage, queue_size=1)
        self.com_status_pub = rospy.Publisher('/gokhas/communication_active', Bool, queue_size=1)
        
        # Subscribers - Receive commands from user interface
        self.control_sub = rospy.Subscriber('/gokhas/control_commands', ControlMessage, self.control_command_callback)
        self.joint_sub = rospy.Subscriber('/gokhas/joint_commands', JointMessage, self.joint_command_callback)
        
        # Serial port setup using configuration
        self._setup_serial_port()

        # Calculate packet sizes for separate structures
        self.control_packet_size = struct.calcsize(CONTROL_FORMAT)  # 2 bytes
        self.joint_packet_size = struct.calcsize(JOINT_FORMAT)     # 6 bytes
        self.rate = rospy.Rate(10)  # 10Hz main loop
        
        # Transmit trigger flags
        self.transmit_control_flag = False
        self.transmit_joint_flag = False
        
        # Data to be transmitted
        self.pending_control = ControlMessage()
        self.pending_joint = JointMessage()
        
        # Received data
        self.received_control = ControlMessage()
        self.received_joint = JointMessage()
        
        # Communication status
        self.communication_active = False
        
        # Track if we received actual data from STM32
        self.data_received_from_stm32 = False
        
        rospy.loginfo("UART Communication Node initialized")

    def _setup_serial_port(self):
        """Setup serial port using configuration parameters"""
        if self.simulation_mode:
            rospy.loginfo("STM32 Simulation Mode ENABLED - skipping serial port setup")
            self.serialPort = None
            self.serial_available = False
            return
            
        try:
            self.serialPort = serial.Serial(
                port=UartConfig.SERIAL_PORT,
                baudrate=UartConfig.BAUD_RATE,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=UartConfig.TIMEOUT
            )
            self.serialPort.reset_input_buffer()
            self.serialPort.reset_output_buffer()
            rospy.loginfo(f"Serial port opened successfully: {UartConfig.SERIAL_PORT} @ {UartConfig.BAUD_RATE} baud")
            self.serial_available = True
        except Exception as e:
            rospy.logerr(f"Failed to open serial port {UartConfig.SERIAL_PORT}: {e}")
            self.serialPort = None
            self.serial_available = False

    def control_command_callback(self, msg):
        """Process ControlMessage commands from user interface"""
        rospy.loginfo(f"Control command received: comStatus={msg.comStatus}, calibStatus={msg.calibStatus}")
        
        self.pending_control = msg
        self.transmit_control_flag = True
        
        # Activate communication if comStatus=1
        if msg.comStatus == 1:
            self.communication_active = True
            rospy.loginfo("Communication activated")

    def joint_command_callback(self, msg):
        """Process JointMessage commands from user interface"""
        rospy.loginfo(f"Joint command received: j1p={msg.j1p}, j2p={msg.j2p}, control_bits={msg.control_bits:08b}")
        
        self.pending_joint = msg
        self.transmit_joint_flag = True

    def transmit_data(self):
        """Transmit data when triggered - separate packets for Control and Joint messages"""
        if self.simulation_mode or not self.serial_available:
            # Simulation when explicitly enabled or serial port is not available
            if self.transmit_control_flag or self.transmit_joint_flag:
                if self.simulation_mode:
                    rospy.loginfo("Simulating transmit - simulation mode enabled")
                else:
                    rospy.loginfo("Simulating transmit - no serial port available")
                # Simulation: sent data returns as received data
                if self.transmit_control_flag:
                    self.received_control = self.pending_control
                    self.data_received_from_stm32 = True  # Mark simulation data as received
                    rospy.loginfo("Simulated Structure 1 (Control) transmission")
                if self.transmit_joint_flag:
                    self.received_joint = self.pending_joint
                    self.data_received_from_stm32 = True  # Mark simulation data as received
                    rospy.loginfo("Simulated Structure 2 (Joint) transmission")
                
                self.transmit_control_flag = False
                self.transmit_joint_flag = False
            return

        # Real serial transmission - separate packets
        try:
            # Transmit Structure 1 (ControlMessage) if flagged
            if self.transmit_control_flag:
                control_packet = struct.pack(CONTROL_FORMAT,
                    self.pending_control.comStatus,
                    self.pending_control.calibStatus
                )
                
                if self.serialPort is not None:
                    self.serialPort.write(control_packet)
                    self.serialPort.flush()
                    rospy.loginfo(f"Transmitted Structure 1 (Control): {len(control_packet)} bytes to STM32")
                else:
                    rospy.logwarn("Serial port is None, cannot transmit Control data")
                
                self.transmit_control_flag = False

            # Transmit Structure 2 (JointMessage) if flagged  
            if self.transmit_joint_flag:
                joint_packet = struct.pack(JOINT_FORMAT,
                    self.pending_joint.control_bits,
                    self.pending_joint.j1p,
                    self.pending_joint.j1s,
                    self.pending_joint.j2p,
                    self.pending_joint.j2s,
                    self.pending_joint.ap
                )
                
                if self.serialPort is not None:
                    self.serialPort.write(joint_packet)
                    self.serialPort.flush()
                    rospy.loginfo(f"Transmitted Structure 2 (Joint): {len(joint_packet)} bytes to STM32")
                else:
                    rospy.logwarn("Serial port is None, cannot transmit Joint data")
                
                self.transmit_joint_flag = False
                
        except Exception as e:
            rospy.logerr(f"Transmit error: {e}")

    def receive_data(self):
        """Continuously receive data - separate packets for Control and Joint messages"""
        if self.simulation_mode or not self.serial_available:
            return

        try:
            # Check for incoming data continuously
            if self.serialPort and self.serialPort.in_waiting > 0:
                
                # Try to receive Structure 1 (ControlMessage - 2 bytes)
                if self.serialPort.in_waiting >= struct.calcsize(CONTROL_FORMAT):
                    control_data = self.serialPort.read(struct.calcsize(CONTROL_FORMAT))
                    
                    if len(control_data) == struct.calcsize(CONTROL_FORMAT):
                        # Unpack ControlMessage
                        unpacked_control = struct.unpack(CONTROL_FORMAT, control_data)
                        
                        # Update received control data
                        self.received_control.comStatus = unpacked_control[0]
                        self.received_control.calibStatus = unpacked_control[1]
                        
                        # Mark that we received actual data from STM32
                        self.data_received_from_stm32 = True
                        
                        rospy.logdebug(f"Received Structure 1 (Control): {len(control_data)} bytes from STM32")
                        
                # Try to receive Structure 2 (JointMessage - 6 bytes) 
                if self.serialPort.in_waiting >= struct.calcsize(JOINT_FORMAT):
                    joint_data = self.serialPort.read(struct.calcsize(JOINT_FORMAT))
                    
                    if len(joint_data) == struct.calcsize(JOINT_FORMAT):
                        # Unpack JointMessage
                        unpacked_joint = struct.unpack(JOINT_FORMAT, joint_data)
                        
                        # Update received joint data
                        self.received_joint.control_bits = unpacked_joint[0]
                        self.received_joint.j1p = unpacked_joint[1]
                        self.received_joint.j1s = unpacked_joint[2]
                        self.received_joint.j2p = unpacked_joint[3]
                        self.received_joint.j2s = unpacked_joint[4]
                        self.received_joint.ap = unpacked_joint[5]
                        
                        # Mark that we received actual data from STM32
                        self.data_received_from_stm32 = True
                        
                        rospy.logdebug(f"Received Structure 2 (Joint): {len(joint_data)} bytes from STM32")
                        
        except Exception as e:
            rospy.logerr(f"Receive error: {e}")

    def publish_status(self):
        """Publish current status only if we received data from STM32"""
        # Only publish if we have received actual data from STM32
        if self.data_received_from_stm32:
            # Publish control status
            self.control_pub.publish(self.received_control)
            
            # Publish joint feedback
            self.joint_pub.publish(self.received_joint)
            
            rospy.logdebug("Published STM32 data to topics")
        
        # Always publish communication status (independent of STM32 data)
        com_status = Bool()
        com_status.data = self.communication_active and (self.received_control.comStatus == 1)
        self.com_status_pub.publish(com_status)

    def run(self):
        """Main loop"""
        rospy.loginfo("Starting UART communication loop")
        
        while not rospy.is_shutdown():
            # 1. Transmit if triggered
            if self.communication_active:
                self.transmit_data()
            
            # 2. Continuously receive
            self.receive_data()
            
            # 3. Publish status
            self.publish_status()
            
            # 4. Rate control
            self.rate.sleep()

    def shutdown(self):
        """Cleanup operations"""
        if self.serialPort and self.serialPort.is_open:
            self.serialPort.close()
            rospy.loginfo("Serial port closed")


if __name__ == "__main__":
    try:
        uart_node = CommunicationClass()
        rospy.on_shutdown(uart_node.shutdown)
        uart_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("UART Communication Node stopped")