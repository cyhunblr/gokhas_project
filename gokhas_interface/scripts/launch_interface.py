#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Interface Launcher
===================================

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

GokHAS Interface Launcher for PyQt6 GUI
=======================================

This script launches the main PyQt6 graphical user interface for the GokHAS project.
It handles proper initialization of both ROS and Qt components, provides signal
handling for clean shutdown, and manages the integration between ROS nodes and GUI.

Features:
- PyQt6 GUI initialization
- ROS-Qt integration
- Signal handling for clean shutdown
- Error handling and recovery
"""

import sys
import os
import signal
import rospy
from PyQt6.QtWidgets import QApplication

# Add the scripts directory to Python path to find ui module
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, 'src')
sys.path.insert(0, src_dir)

from ui.main_window import MainWindow

class InterfaceLauncher:
    """
    Simple and clean Qt-ROS interface launcher
    
    Manages the startup and shutdown of the PyQt6 interface with ROS integration
    """
    
    def __init__(self):
        """
        Initialize the interface launcher
        
        Sets up shutdown flags and Qt/ROS component references
        """
        self.shutdown_initiated = False
        self.app = None
        self.window = None
    
    def signal_handler(self, sig, frame):
        """
        Handle shutdown signals (Ctrl+C, etc.)
        
        Catches system signals for graceful shutdown
        """
        if self.shutdown_initiated:
            return
            
        print("Shutdown signal received...")
        self.shutdown()
    
    def shutdown(self):
        """
        Clean shutdown sequence
        
        Properly closes ROS and Qt components in correct order
        """
        if self.shutdown_initiated:
            return
        self.shutdown_initiated = True
        
        print("Shutting down interface...")
        
        # Close ROS
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("Interface shutdown")
        except Exception as e:
            print(f"ROS shutdown warning: {e}")
        
        # Close Qt
        if self.app:
            self.app.quit()
        
        sys.exit(0)
    
    def run(self):
        """
        Main execution function
        
        Initializes ROS and Qt components, then starts the main event loop
        """
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Initialize ROS
        rospy.init_node('qt_interface_node', anonymous=True)
        rospy.loginfo("ROS node initialized")
        
        # Initialize Qt
        self.app = QApplication(sys.argv)
        self.window = MainWindow()
        
        # Connect Qt shutdown to our handler
        self.app.aboutToQuit.connect(self.shutdown)
        
        # Show window and start
        self.window.show()
        rospy.loginfo("Qt interface started!")
        
        # Run Qt event loop
        try:
            exit_code = self.app.exec()
            sys.exit(exit_code)
        except KeyboardInterrupt:
            self.signal_handler(signal.SIGINT, None)

def main():
    """
    Entry point
    
    Main entry point for the interface launcher
    """
    launcher = InterfaceLauncher()
    launcher.run()

if __name__ == "__main__":
    main()