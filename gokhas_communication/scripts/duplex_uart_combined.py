#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Legacy UART Communication Node
===============================================

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

Legacy UART Communication Node for GokHAS Project (Combined Packet Version)
===========================================================================

This is the legacy version of UART communication that uses combined 8-byte packets
containing both control and joint data. This version is kept for backward compatibility
with older STM32 firmware versions that expect combined packet structure.

Note: For new implementations, use uart_com_updated.py which supports separate packet structures.

Features:
- Combined 8-byte packet transmission
- Legacy STM32 firmware compatibility
- Control + Joint data in single packet
"""

import time
import rospy
from gokhas_communication.msg import ControlMessage, JointMessage
from std_msgs.msg import Bool
import serial
import struct

# Packet formats - OLD VERSION: Combined transmission
CONTROL_FORMAT = 'BB'        # ControlMessage: 2 bytes
JOINT_FORMAT = 'BBBBBB'      # JointMessage: 6 bytes
COMBINED_FORMAT = 'BBBBBBBB' # Combined: 8 bytes total (Structure1+Structure2)

class UartConfig:
    """
    Configuration class for UART communication parameters (Legacy Version)
    
    Handles UART configuration for legacy combined packet transmission
    """
    
    # Default values
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200
    TIMEOUT = 1.0
    LOOP_RATE = 10  # Hz
    
    @classmethod
    def load_config(cls):
        """Load configuration from ROS parameters"""
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
    def __init__(self):
        rospy.init_node('uart_communication_node_old', anonymous=True)
        
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

        # Calculate packet sizes - OLD VERSION: Combined 8-byte packet
        self.combined_packet_size = struct.calcsize(COMBINED_FORMAT)  # 8 bytes total
        self.rate = rospy.Rate(100)  # 100Hz main loop
        
        # OLD VERSION: Single combined transmission flag
        self.transmit_combined_flag = False
        
        # Data to be transmitted - OLD VERSION: Combined packet
        self.pending_control = ControlMessage()
        self.pending_joint = JointMessage()
        
        # Received data
        self.received_control = ControlMessage()
        self.received_joint = JointMessage()
        
        # Communication status
        self.communication_active = False
        
        # Track if we received actual data from STM32
        self.data_received_from_stm32 = False
        
        # Calibration completion tracking
        self.calibration_completed = False
        self.send_calibration_reset = False
        
        rospy.loginfo("UART Communication Node (OLD VERSION - Combined 8-byte packets) initialized")

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
            # Hardware connection failed - but DO NOT automatically enable simulation mode
            # Let the interface handle timeout behavior according to user configuration

    def control_command_callback(self, msg):
        """Process ControlMessage commands from user interface - OLD VERSION: Triggers combined transmission"""
        rospy.loginfo(f"Control command received: comStatus={msg.comStatus}, calibStatus={msg.calibStatus}")
        
        self.pending_control = msg
        
        # If communication is being activated and serial port is not available, try to reconnect
        if msg.comStatus == 1 and not self.serial_available and not self.simulation_mode:
            rospy.loginfo("Communication activation requested - attempting to reconnect to STM32...")
            self._setup_serial_port()
        
        # Always trigger transmission when a control command is received
        self.transmit_combined_flag = True
        
        # Set communication_active based on comStatus for continuous receiving
        if msg.comStatus == 1:
            self.communication_active = True
            rospy.loginfo("Communication activated - continuous receiving enabled")
        else:
            self.communication_active = False
            rospy.loginfo("Communication deactivated - receiving disabled")

    def joint_command_callback(self, msg):
        """Process JointMessage commands from user interface - OLD VERSION: Updates joint data for combined packet"""
        rospy.loginfo(f"Joint command received: j1p={msg.j1p}, j2p={msg.j2p}, control_bits={msg.control_bits:08b}")
        
        self.pending_joint = msg
        
        # Trigger transmission when joint command is received
        self.transmit_combined_flag = True
        rospy.loginfo("Joint command triggered transmission")

    def transmit_data(self):
        """Transmit data - OLD VERSION: Combined 8-byte packet (Structure1+Structure2)"""
        if not self.transmit_combined_flag:
            return
            
        if self.simulation_mode:
            # Only simulate when explicitly enabled in configuration
            rospy.loginfo("Simulating combined 8-byte packet transmit - simulation mode enabled")
            # Simulation: sent data returns as received data
            self.received_control = self.pending_control
            self.received_joint = self.pending_joint
            self.data_received_from_stm32 = True  # Mark simulation data as received
            rospy.loginfo("Simulated combined packet (Structure1+Structure2) transmission")
            self.transmit_combined_flag = False
            return

        if not self.serial_available:
            # Hardware not available - do NOT simulate automatically
            # Let interface handle timeout behavior
            rospy.logwarn("Cannot transmit - serial port not available")
            self.transmit_combined_flag = False
            return

        # Real serial transmission - OLD VERSION: Combined packet
        try:
            # OLD VERSION: Create combined 8-byte packet (Structure1 + Structure2)
            combined_packet = struct.pack(COMBINED_FORMAT,
                # Structure 1 (ControlMessage) - 2 bytes
                self.pending_control.comStatus,
                self.pending_control.calibStatus,
                # Structure 2 (JointMessage) - 6 bytes
                self.pending_joint.control_bits,
                self.pending_joint.j1p,
                self.pending_joint.j1s,
                self.pending_joint.j2p,
                self.pending_joint.j2s,
                self.pending_joint.ap
            )
            
            if self.serialPort is not None:
                self.serialPort.write(combined_packet)
                self.serialPort.flush()
                rospy.loginfo(f"OLD VERSION: Transmitted combined packet (Structure1+Structure2): {len(combined_packet)} bytes to STM32")
                rospy.loginfo(f"Packet content: Control({self.pending_control.comStatus},{self.pending_control.calibStatus}) + Joint({self.pending_joint.control_bits},{self.pending_joint.j1p},{self.pending_joint.j1s},{self.pending_joint.j2p},{self.pending_joint.j2s},{self.pending_joint.ap})")
            else:
                rospy.logwarn("Serial port is None, cannot transmit combined data")
            
            self.transmit_combined_flag = False
                
        except Exception as e:
            rospy.logerr(f"Transmit error: {e}")

    def receive_data(self):
        """Receive data - OLD VERSION: Expects combined 8-byte response packet"""
        if self.simulation_mode or not self.serial_available:
            return

        try:
            # Check for incoming data continuously
            if self.serialPort and self.serialPort.in_waiting > 0:
                
                # OLD VERSION: Try to receive combined packet (8 bytes)
                if self.serialPort.in_waiting >= self.combined_packet_size:
                    combined_data = self.serialPort.read(self.combined_packet_size)
                    
                    if len(combined_data) == self.combined_packet_size:
                        # Unpack combined packet (Structure1 + Structure2)
                        unpacked_data = struct.unpack(COMBINED_FORMAT, combined_data)
                        
                        # Extract Structure 1 (ControlMessage)
                        self.received_control.comStatus = unpacked_data[0]
                        self.received_control.calibStatus = unpacked_data[1]
                        
                        # Check if calibration is completed (received calibStatus=1)
                        if self.received_control.calibStatus == 1 and not self.calibration_completed:
                            rospy.loginfo("Calibration completed! STM32 sent calibStatus=1")
                            self.calibration_completed = True
                            self.send_calibration_reset = True
                            rospy.loginfo("Will send calibStatus=0 to reset calibration state")
                        
                        # Extract Structure 2 (JointMessage)
                        self.received_joint.control_bits = unpacked_data[2]
                        self.received_joint.j1p = unpacked_data[3]
                        self.received_joint.j1s = unpacked_data[4]
                        self.received_joint.j2p = unpacked_data[5]
                        self.received_joint.j2s = unpacked_data[6]
                        self.received_joint.ap = unpacked_data[7]
                        
                        # Mark that we received actual data from STM32
                        self.data_received_from_stm32 = True
                        
                        rospy.loginfo(f"OLD VERSION: Received combined packet: {len(combined_data)} bytes from STM32")
                        rospy.logdebug(f"Control: comStatus={self.received_control.comStatus}, calibStatus={self.received_control.calibStatus}")
                        rospy.logdebug(f"Joint: control_bits={self.received_joint.control_bits}, j1p={self.received_joint.j1p}")
                        
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
        rospy.loginfo("Starting UART communication loop (OLD VERSION - Combined 8-byte packets)")
        
        while not rospy.is_shutdown():
            # 1. Check if calibration reset needs to be sent
            if self.send_calibration_reset:
                rospy.loginfo("Sending calibration reset: calibStatus=0")
                reset_control = ControlMessage()
                reset_control.comStatus = 1  # Keep communication active
                reset_control.calibStatus = 0  # Reset calibration
                
                # Update pending control and trigger transmission
                self.pending_control = reset_control
                self.transmit_combined_flag = True
                self.send_calibration_reset = False
            
            # 2. Transmit combined packet if triggered (independent of communication_active)
            self.transmit_data()
            
            # 3. Continuously receive combined packets only when communication is active
            if self.communication_active:
                self.receive_data()
            
            # 4. Publish status
            self.publish_status()
            
            # 5. Rate control
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
        rospy.loginfo("UART Communication Node (OLD VERSION) stopped")
