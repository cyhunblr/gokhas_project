#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Autonomous 2D Targeting System
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

Autonomous 2D Targeting System for GokHAS Project
=================================================

This script implements a 2D autonomous targeting system that tracks detected objects 
using YOLO computer vision and controls robot joints to center the target in the camera view.
The system provides dynamic speed control based on target distance and includes dead zones
to prevent oscillation around the center point.

Features:
- Real-time YOLO object detection integration
- Dynamic speed control based on target distance
- Dead zone implementation to prevent oscillation
- Autonomous mode toggle
- Joint command publishing for robot control
"""

import rospy
from std_msgs.msg import Bool
from gokhas_communication.msg import JointMessage
from gokhas_perception.msg import YoloResult

class AutonomousTargeting2D:
    def __init__(self):
        """
        Initialize the 2D Autonomous Targeting System
        
        Sets up ROS node, publishers, subscribers and control parameters
        """
        rospy.init_node('autonomous_targeting_2d', anonymous=True)
        
        # Publishers
        self.joint_pub = rospy.Publisher('/gokhas/joint_commands', JointMessage, queue_size=10)
        
        # Subscribers
        self.autonomous_sub = rospy.Subscriber('/gokhas/autonomous_mode', Bool, self.autonomous_callback)
        self.yolo_sub = rospy.Subscriber('/zedm/zed_node/rgb/yolo_result', YoloResult, self.yolo_callback)
        
        # State variables
        self.autonomous_mode = False  # Autonomous mode status
        self.image_width = 1920       # YOLOv8 image width
        self.image_height = 1080      # YOLOv8 image height
        self.center_x = self.image_width // 2   # Image center X (960)
        self.center_y = self.image_height // 2  # Image center Y (540)
        
        # Control parameters
        self.angle_step = 1.0         # Degree step per movement
        self.dead_zone_x = 100        # Horizontal dead zone (pixels)
        self.dead_zone_y = 100        # Vertical dead zone (pixels)
        self.last_command_time = 0.0  # Last command time
        self.command_interval = 0.001 # Interval between commands
        
        # Dynamic speed parameters
        self.min_speed_horizontal = 0   # Minimum horizontal speed
        self.max_speed_horizontal = 100 # Maximum horizontal speed
        self.min_speed_vertical = 0     # Minimum vertical speed  
        self.max_speed_vertical = 40    # Maximum vertical speed
        self.horizontal_speed_threshold = self.image_width / 6   # Horizontal speed threshold
        self.vertical_speed_threshold = self.image_height        # Vertical speed threshold

        # Log initialization
        rospy.loginfo("Autonomous Targeting 2D initialized")
        rospy.loginfo(f"Image center: ({self.center_x}, {self.center_y})")
        rospy.loginfo(f"Dead zone: ±{self.dead_zone_x}px horizontal, ±{self.dead_zone_y}px vertical")
        rospy.loginfo(f"Speed range: {self.min_speed_horizontal}% - {self.max_speed_horizontal}% (H threshold: {self.horizontal_speed_threshold}px, V threshold: {self.vertical_speed_threshold}px)")
        
    def autonomous_callback(self, msg):
        """
        Autonomous mode toggle callback
        
        Activates/deactivates autonomous targeting mode
        """
        self.autonomous_mode = msg.data
        if self.autonomous_mode:
            rospy.loginfo("Autonomous 2D targeting ACTIVATED")
        else:
            rospy.loginfo("Autonomous 2D targeting DEACTIVATED")
    
    def calculate_dynamic_speed(self, h_distance, v_distance):
        """
        Calculate dynamic speed based on pixel distance from target
        
        Calculates motor speed based on how far the target is from center
        """
        # Calculate horizontal and vertical speeds separately
        h_abs = abs(h_distance)
        v_abs = abs(v_distance)
        
        # Calculate horizontal speed
        if h_abs <= self.dead_zone_x:
            h_normalized = 0.0
        elif h_abs >= self.horizontal_speed_threshold:
            h_normalized = 1.0
        else:
            h_normalized = (h_abs - self.dead_zone_x) / (self.horizontal_speed_threshold - self.dead_zone_x)
        
        # Calculate vertical speed
        if v_abs <= self.dead_zone_y:
            v_normalized = 0.0
        elif v_abs >= self.vertical_speed_threshold:
            v_normalized = 1.0
        else:
            v_normalized = (v_abs - self.dead_zone_y) / (self.vertical_speed_threshold - self.dead_zone_y)
        
        # Calculate actual speeds for each axis
        h_speed = int(self.min_speed_horizontal + (self.max_speed_horizontal - self.min_speed_horizontal) * h_normalized)
        v_speed = int(self.min_speed_vertical + (self.max_speed_vertical - self.min_speed_vertical) * v_normalized)
        
        # Use the maximum of horizontal and vertical speeds
        # This ensures we use higher speed if either axis requires it
        dynamic_speed = max(h_speed, v_speed)
        
        # Clamp to valid range (use overall min/max from both axes)
        overall_min = min(self.min_speed_horizontal, self.min_speed_vertical)
        overall_max = max(self.max_speed_horizontal, self.max_speed_vertical)
        dynamic_speed = max(overall_min, min(overall_max, dynamic_speed))
        
        return dynamic_speed
    
    def yolo_callback(self, msg):
        """
        Process YOLO detection results for 2D tracking
        
        Processes detected objects and calculates joint movements to center target
        """
        if not self.autonomous_mode:
            return
            
        # Find first detected object from detections array
        target_detection = None
        if msg.detections.detections:  # If there are any detections
            target_detection = msg.detections.detections[0]  # Take first detection
        
        if target_detection is None:
            rospy.logwarn("No target detected")
            return
            
        # Calculate object center from bounding box
        obj_center_x = target_detection.bbox.center.x
        obj_center_y = target_detection.bbox.center.y
        
        # Calculate movement directions with dead zone
        horizontal_angle = 0.0
        vertical_angle = 0.0
        
        # Calculate pixel distances from center
        h_distance = obj_center_x - self.center_x
        v_distance = obj_center_y - self.center_y
        
        # Rate limiting - prevent too frequent commands
        current_time = rospy.get_time()
        if (current_time - self.last_command_time) < self.command_interval:
            return
        
        # Horizontal movement with dead zone
        if h_distance > self.dead_zone_x:
            # Object is on the right side -> move right (+1 degree)
            horizontal_angle = self.angle_step
            rospy.loginfo(f"Moving RIGHT (+1°) - distance: {h_distance:.1f}px")
        elif h_distance < -self.dead_zone_x:
            # Object is on the left side -> move left (-1 degree)
            horizontal_angle = -self.angle_step
            rospy.loginfo(f"Moving LEFT (-1°) - distance: {h_distance:.1f}px")
            
        # Vertical movement with dead zone
        if v_distance > self.dead_zone_y:
            # Object is on the lower side -> move down (-1 degree)
            vertical_angle = -self.angle_step
            rospy.loginfo(f"Moving DOWN (-1°) - distance: {v_distance:.1f}px")
        elif v_distance < -self.dead_zone_y:
            # Object is on the upper side -> move up (+1 degree)
            vertical_angle = self.angle_step
            rospy.loginfo(f"Moving UP (+1°) - distance: {v_distance:.1f}px")
        
        # Only send command if there's actual movement needed
        if horizontal_angle == 0.0 and vertical_angle == 0.0:
            rospy.loginfo("Target CENTERED - No movement needed")
            return
            
        # Update last command time
        self.last_command_time = current_time
        
        # Send joint commands
        self.send_joint_command(horizontal_angle, vertical_angle, h_distance, v_distance)
    
    def send_joint_command(self, j1_angle, j2_angle, h_distance=0, v_distance=0):
        """
        Send joint movement commands to the robot with dynamic speed
        
        Sends joint commands with appropriate control bits and dynamic speed
        """
        joint_msg = JointMessage()
        joint_msg.control_bits = 0
        
        # Calculate dynamic speed based on distance from target
        dynamic_speed = self.calculate_dynamic_speed(h_distance, v_distance)
        
        # Joint 1 (Horizontal) - Convert to unsigned integer
        if j1_angle < 0:
            joint_msg.control_bits |= 0b00001010  # Set bit 1 for negative
            joint_msg.j1p = int(abs(j1_angle))
        else:
            joint_msg.control_bits |= 0b00001000  # Set bit 3 for positive
            joint_msg.j1p = int(j1_angle)
            
        # Joint 2 (Vertical) - Convert to unsigned integer
        if j2_angle < 0:
            joint_msg.control_bits |= 0b00001100  # Set bit 2 for negative
            joint_msg.j2p = int(abs(j2_angle))
        else:
            joint_msg.control_bits |= 0b00001000  # Set bit 3 for positive
            joint_msg.j2p = int(j2_angle)
        
        # Set dynamic speed for both joints
        joint_msg.j1s = dynamic_speed  # Joint 1 speed (0-100) - Dynamic
        joint_msg.j2s = dynamic_speed  # Joint 2 speed (0-100) - Dynamic
        joint_msg.ap = 0    # Airsoft power (0-100)
        
        # Publish command
        self.joint_pub.publish(joint_msg)
        
        # Calculate total distance for logging
        total_distance = (h_distance**2 + v_distance**2)**0.5
        rospy.loginfo(f"Joint command sent: J1={j1_angle}°, J2={j2_angle}°, Speed={dynamic_speed}% (dist:{total_distance:.1f}px)")

def main():
    """
    Main function
    
    Initializes and runs the autonomous targeting system
    """
    try:
        targeting = AutonomousTargeting2D()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous Targeting 2D node terminated")

if __name__ == '__main__':
    main()