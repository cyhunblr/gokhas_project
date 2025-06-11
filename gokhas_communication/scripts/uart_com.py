#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/cyhunblr/bitirme_ws/src/gokhas_project/gokhas_communication/scripts/uart_com.py

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

class CommunicationClass:
    def __init__(self):
        rospy.init_node('uart_communication_node', anonymous=True)

        # Publishers - Publish data received from STM32
        self.control_pub = rospy.Publisher('/gokhas/control_status', ControlMessage, queue_size=1)
        self.joint_pub = rospy.Publisher('/gokhas/joint_feedback', JointMessage, queue_size=1)
        self.com_status_pub = rospy.Publisher('/gokhas/communication_active', Bool, queue_size=1)
        
        # Subscribers - Receive commands from user interface
        self.control_sub = rospy.Subscriber('/gokhas/control_commands', ControlMessage, self.control_command_callback)
        self.joint_sub = rospy.Subscriber('/gokhas/joint_commands', JointMessage, self.joint_command_callback)
        
        # Serial port setup
        try:
            self.serialPort = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.1  # Short timeout for non-blocking receive
            )
            self.serialPort.flushInput()
            self.serialPort.flushOutput()
            rospy.loginfo("Serial port opened successfully")
            self.serial_available = True
        except Exception as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            self.serialPort = None
            self.serial_available = False

        self.packet_size = struct.calcsize(TOTAL_FORMAT)
        self.rate = rospy.Rate(100)  # 100Hz main loop
        
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
        
        rospy.loginfo("UART Communication Node initialized")

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
        """Transmit data when triggered"""
        if not self.serial_available:
            # Simulation when serial port is not available
            if self.transmit_control_flag or self.transmit_joint_flag:
                rospy.loginfo("Simulating transmit - no serial port available")
                # Simulation: sent data returns as received data
                if self.transmit_control_flag:
                    self.received_control = self.pending_control
                if self.transmit_joint_flag:
                    self.received_joint = self.pending_joint
                
                self.transmit_control_flag = False
                self.transmit_joint_flag = False
            return

        # Real serial transmission
        if self.transmit_control_flag or self.transmit_joint_flag:
            try:
                # Create packet (8 bytes)
                packet_data = struct.pack(TOTAL_FORMAT,
                    self.pending_control.comStatus,
                    self.pending_control.calibStatus,
                    self.pending_joint.control_bits,
                    self.pending_joint.j1p,
                    self.pending_joint.j1s,
                    self.pending_joint.j2p,
                    self.pending_joint.j2s,
                    self.pending_joint.ap
                )
                
                self.serialPort.write(packet_data)
                self.serialPort.flush()
                
                rospy.loginfo(f"Transmitted {len(packet_data)} bytes to STM32")
                
                # Clear flags
                self.transmit_control_flag = False
                self.transmit_joint_flag = False
                
            except Exception as e:
                rospy.logerr(f"Transmit error: {e}")

    def receive_data(self):
        """Continuously receive data"""
        if not self.serial_available:
            return

        try:
            # Check if data is available
            if self.serialPort.in_waiting >= self.packet_size:
                data = self.serialPort.read(self.packet_size)
                
                if len(data) == self.packet_size:
                    # Unpack the packet
                    unpacked = struct.unpack(TOTAL_FORMAT, data)
                    
                    # Update ControlMessage
                    self.received_control.comStatus = unpacked[0]
                    self.received_control.calibStatus = unpacked[1]
                    
                    # Update JointMessage
                    self.received_joint.control_bits = unpacked[2]
                    self.received_joint.j1p = unpacked[3]
                    self.received_joint.j1s = unpacked[4]
                    self.received_joint.j2p = unpacked[5]
                    self.received_joint.j2s = unpacked[6]
                    self.received_joint.ap = unpacked[7]
                    
                    rospy.logdebug(f"Received {len(data)} bytes from STM32")
                    
        except Exception as e:
            rospy.logerr(f"Receive error: {e}")

    def publish_status(self):
        """Publish current status"""
        # Publish control status
        self.control_pub.publish(self.received_control)
        
        # Publish joint feedback
        self.joint_pub.publish(self.received_joint)
        
        # Publish communication status
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