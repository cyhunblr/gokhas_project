#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    """Simple and clean Qt-ROS interface launcher"""
    
    def __init__(self):
        self.shutdown_initiated = False
        self.app = None
        self.window = None
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals (Ctrl+C, etc.)"""
        if self.shutdown_initiated:
            return
            
        print("Shutdown signal received...")
        self.shutdown()
    
    def shutdown(self):
        """Clean shutdown sequence"""
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
        """Main execution function"""
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
    """Entry point"""
    launcher = InterfaceLauncher()
    launcher.run()

if __name__ == "__main__":
    main()