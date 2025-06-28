#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Coordinate Publisher Module
============================================

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

Coordinate Publisher Module:
Publishes clicked coordinates and trigger status from the interface using gokhas_communication.msg.
"""

import rospy
from gokhas_communication.msg import JointMessage


class CoordinatePublisher:
    """Class that publishes coordinate and trigger information using JointMessage"""
    
    def __init__(self):
        """Initialize the publisher"""
        # Start ROS node if not already initialized
        if not rospy.get_node_uri():
            rospy.init_node('coordinate_publisher', anonymous=True)
        
        # Single publisher for joint commands
        self.joint_pub = rospy.Publisher('/gokhas/joint_commands', JointMessage, queue_size=10)
        
        # Default joint message template
        self.joint_msg = JointMessage()
        self._init_default_message()
        
        rospy.loginfo("Coordinate Publisher initialized")
        print("[CoordinatePublisher] Publisher started - using gokhas_communication.msg")
    
    def _init_default_message(self):
        """Set default message values for JointMessage"""
        # Initialize JointMessage fields
        self.joint_msg.control_bits = 0  # No airsoft trigger, positive joints
        self.joint_msg.j1p = 0          # Joint 1 position (will be used for X coordinate)
        self.joint_msg.j1s = 0          # Joint 1 speed (not used for coordinates)
        self.joint_msg.j2p = 0          # Joint 2 position (will be used for Y coordinate)
        self.joint_msg.j2s = 0          # Joint 2 speed (not used for coordinates)
        self.joint_msg.ap = 0           # Airsoft power (not used for coordinates)
    
    def publish_coordinates(self, x, y):
        """
        Publish coordinates using JointMessage
        
        Args:
            x (float): X coordinate (mapped to j1p)
            y (float): Y coordinate (mapped to j2p)
        """
        try:
            # Print to terminal for debugging
            print(f"[CLICK] Coordinates: x={int(x)}, y={int(y)}")
            
            # Map coordinates to joint positions (clamp to valid range 0-135)
            self.joint_msg.j1p = min(135, max(0, int(abs(x))))  # X coordinate to Joint 1 position
            self.joint_msg.j2p = min(135, max(0, int(abs(y))))  # Y coordinate to Joint 2 position
            
            # Set control bits for coordinate signs
            control_bits = 0
            if x < 0:
                control_bits |= 0b00000010  # Set j1p_sign bit (bit 2)
            if y < 0:
                control_bits |= 0b00000100  # Set j2p_sign bit (bit 3)
            # No airsoft trigger for coordinate-only messages
            
            self.joint_msg.control_bits = control_bits
            
            # Publish to ROS topic
            self.joint_pub.publish(self.joint_msg)
            
            rospy.loginfo(f"Published coordinates via JointMessage: j1p={self.joint_msg.j1p}, j2p={self.joint_msg.j2p}")
            print(f"[JOINT_MSG] j1p(X)={self.joint_msg.j1p}, j2p(Y)={self.joint_msg.j2p}, bits={control_bits:08b}")
            
        except Exception as e:
            print(f"[ERROR] Coordinate publishing error: {e}")
            rospy.logerr(f"Coordinate publishing error: {e}")
    
    def publish_trigger(self, triggered=True):
        """
        Publish trigger status using JointMessage
        
        Args:
            triggered (bool): Trigger status (True for triggered, False for not triggered)
        """
        try:
            # Print to terminal for debugging
            trigger_text = "TRIGGERED" if triggered else "NOT_TRIGGERED"
            print(f"[TRIGGER] Status: {trigger_text}")
            
            # Set airsoft trigger bit in control_bits
            if triggered:
                self.joint_msg.control_bits |= 0b00000001  # Set airsoft_trigger bit (bit 1)
            else:
                self.joint_msg.control_bits &= 0b11111110  # Clear airsoft_trigger bit
            
            # Reset coordinates for trigger-only messages
            self.joint_msg.j1p = 0
            self.joint_msg.j2p = 0
            
            # Publish to ROS topic
            self.joint_pub.publish(self.joint_msg)
            
            rospy.loginfo(f"Published trigger via JointMessage: trigger={triggered}")
            print(f"[JOINT_MSG] trigger={triggered}, j1p=0, j2p=0, bits={self.joint_msg.control_bits:08b}")
            
        except Exception as e:
            print(f"[ERROR] Trigger publishing error: {e}")
            rospy.logerr(f"Trigger publishing error: {e}")
    
    def publish_coordinates_and_trigger(self, x, y, triggered=False):
        """
        Publish coordinates and trigger status simultaneously using JointMessage
        
        Args:
            x (float): X coordinate of mouse click
            y (float): Y coordinate of mouse click
            triggered (bool): Trigger status
        """
        try:
            # Print to terminal for debugging
            print(f"[CLICK+TRIGGER] Coordinates: x={int(x)}, y={int(y)}, Trigger: {triggered}")
            
            # Map coordinates to joint positions (clamp to valid range 0-135)
            self.joint_msg.j1p = min(135, max(0, int(abs(x))))
            self.joint_msg.j2p = min(135, max(0, int(abs(y))))
            
            # Set control bits for coordinates and trigger
            control_bits = 0
            if triggered:
                control_bits |= 0b00000001  # Set airsoft_trigger bit (bit 1)
            if x < 0:
                control_bits |= 0b00000010  # Set j1p_sign bit (bit 2)
            if y < 0:
                control_bits |= 0b00000100  # Set j2p_sign bit (bit 3)
                
            self.joint_msg.control_bits = control_bits
            
            # Publish to ROS topic
            self.joint_pub.publish(self.joint_msg)
            
            rospy.loginfo(f"Published coordinates+trigger via JointMessage: j1p={self.joint_msg.j1p}, j2p={self.joint_msg.j2p}, trigger={triggered}")
            print(f"[JOINT_MSG] j1p(X)={self.joint_msg.j1p}, j2p(Y)={self.joint_msg.j2p}, trigger={triggered}, bits={control_bits:08b}")
            
        except Exception as e:
            print(f"[ERROR] Coordinate+Trigger publishing error: {e}")
            rospy.logerr(f"Coordinate+Trigger publishing error: {e}")
    
    def print_mouse_event(self, event_type, x=None, y=None):
        """
        Print mouse events in detail to terminal and publish them
        
        This method provides a unified interface for handling different types
        of mouse events and automatically publishes the appropriate messages.
        
        Args:
            event_type (str): Event type ('left_click', 'right_click', 'move')
            x (float, optional): X coordinate (required for click events)
            y (float, optional): Y coordinate (required for click events)
        """
        try:
            timestamp = rospy.Time.now()
            
            if event_type == "left_click" and x is not None and y is not None:
                print(f"[{timestamp}] LEFT CLICK - X: {int(x)}, Y: {int(y)}")
                self.publish_coordinates(x, y)
                
            elif event_type == "right_click":
                print(f"[{timestamp}] RIGHT CLICK - TRIGGER ACTIVATED")
                self.publish_trigger(True)
                
            elif event_type == "left_click_with_trigger" and x is not None and y is not None:
                print(f"[{timestamp}] LEFT CLICK + TRIGGER - X: {int(x)}, Y: {int(y)}")
                self.publish_coordinates_and_trigger(x, y, True)
                
            elif event_type == "move" and x is not None and y is not None:
                # Only print move events in debug mode to avoid spam
                if rospy.get_param('~debug_mouse_move', False):
                    print(f"[{timestamp}] MOUSE MOVE - X: {int(x)}, Y: {int(y)}")
                    
        except Exception as e:
            print(f"[ERROR] Mouse event print error: {e}")
            rospy.logerr(f"Mouse event print error: {e}")
    
    def print_joint_status(self):
        """Print current joint message status for debugging"""
        try:
            print("\\n[JOINT_STATUS] Current joint message state:")
            print(f"  POSITIONS: j1p={self.joint_msg.j1p}, j2p={self.joint_msg.j2p}")
            print(f"  SPEEDS: j1s={self.joint_msg.j1s}, j2s={self.joint_msg.j2s}")
            print(f"  AIRSOFT: ap={self.joint_msg.ap}")
            print(f"  CONTROL_BITS: {self.joint_msg.control_bits:08b}")
            print("")
        except Exception as e:
            print(f"[ERROR] Status print error: {e}")
    
    def shutdown(self):
        """Close the publisher and clean up resources"""
        try:
            print("[CoordinatePublisher] Shutting down...")
            rospy.loginfo("Coordinate Publisher shutting down")
        except Exception as e:
            print(f"[ERROR] Shutdown error: {e}")


# Global publisher instance for singleton pattern
coordinate_publisher = None

def get_coordinate_publisher():
    """Return global coordinate publisher instance (singleton pattern)"""
    global coordinate_publisher
    if coordinate_publisher is None:
        coordinate_publisher = CoordinatePublisher()
    return coordinate_publisher

def publish_click_coordinates(x, y):
    """
    Convenience function: Publish clicked coordinates
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.publish_coordinates(x, y)

def publish_trigger_event():
    """Convenience function: Publish trigger event"""
    publisher = get_coordinate_publisher()
    publisher.publish_trigger(True)

def publish_click_with_trigger(x, y):
    """
    Convenience function: Publish coordinates and trigger together
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.publish_coordinates_and_trigger(x, y, True)

def print_click_event(x, y):
    """
    Convenience function: Print left click event
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("left_click", x, y)

def print_trigger_event():
    """Convenience function: Print right click (trigger) event"""
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("right_click")

def print_click_trigger_event(x, y):
    """
    Convenience function: Print left click + trigger event
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("left_click_with_trigger", x, y)


if __name__ == "__main__":
    """Main function for testing purposes"""
    try:
        # Initialize publisher
        pub = CoordinatePublisher()
        
        # Show initial status
        pub.print_joint_status()
        
        # Test coordinates for demonstration
        test_coordinates = [(100, 150), (250, 300), (400, 200)]
        
        print("Testing coordinate publishing with JointMessage...")
        
        # Publish test coordinates
        for i, (x, y) in enumerate(test_coordinates):
            print(f"\n--- Test {i+1} ---")
            pub.print_mouse_event("left_click", x, y)
            rospy.sleep(1)
            pub.print_joint_status()
        
        # Test trigger functionality
        print("\n--- Trigger Test ---")
        pub.print_mouse_event("right_click")
        rospy.sleep(1)
        pub.print_joint_status()
        
        # Test coordinate + trigger combination
        print("\n--- Coordinate + Trigger Test ---")
        pub.print_mouse_event("left_click_with_trigger", 500, 400)
        rospy.sleep(1)
        pub.print_joint_status()
        
        print("Test completed successfully.")
        
    except rospy.ROSInterruptException:
        print("ROS interrupted")
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Test error: {e}")