#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Delayed Launch Manager
=======================================

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
"""

import rospy
import subprocess
import threading
import time
import sys

class DelayedLauncher:
    def __init__(self):
        rospy.init_node('delayed_launcher', anonymous=True)
        
        # List of launch configurations: (package, launch_file, delay_seconds)
        self.launch_configs = [
            ('zed_wrapper', 'zedm.launch', 2.0),
            ('gokhas_perception', 'tracker.launch', 3.0),
            ('gokhas_communication', 'communication.launch', 1.0),
        ]
        
        self.processes = []
        
    def launch_with_delay(self, package, launch_file, delay):
        """Launch a roslaunch command after a specified delay"""
        def delayed_launch():
            rospy.loginfo(f"Waiting {delay}s before launching {package} {launch_file}")
            time.sleep(delay)
            
            if rospy.is_shutdown():
                return
                
            rospy.loginfo(f"Launching {package} {launch_file}")
            try:
                cmd = ['roslaunch', package, launch_file]
                process = subprocess.Popen(cmd)
                self.processes.append(process)
                rospy.loginfo(f"Successfully started {package} {launch_file} with PID {process.pid}")
            except Exception as e:
                rospy.logerr(f"Failed to launch {package} {launch_file}: {e}")
        
        thread = threading.Thread(target=delayed_launch)
        thread.daemon = True
        thread.start()
    
    def start_delayed_launches(self):
        """Start all delayed launches"""
        rospy.loginfo("Starting delayed launcher")
        
        for package, launch_file, delay in self.launch_configs:
            self.launch_with_delay(package, launch_file, delay)
    
    def cleanup(self):
        """Cleanup all spawned processes"""
        rospy.loginfo("Cleaning up delayed launcher processes")
        for process in self.processes:
            if process.poll() is None:  # Process is still running
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()

def main():
    try:
        launcher = DelayedLauncher()
        launcher.start_delayed_launches()
        
        rospy.loginfo("Delayed launcher initialized. Press Ctrl+C to exit.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Delayed launcher interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("Delayed launcher interrupted by user")
    finally:
        if 'launcher' in locals():
            launcher.cleanup()

if __name__ == '__main__':
    main()
