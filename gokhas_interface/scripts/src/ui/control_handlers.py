#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Control Handlers Module
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

Control Handlers Module
=======================

This module manages interface button functions and system control logic for the
GokHAS project's graphical user interface.
"""

import rospy
from PyQt6.QtWidgets import (
    QWidget, QLabel, QFrame, QPushButton, QSlider, 
    QGridLayout, QHBoxLayout, QVBoxLayout, QTextEdit, 
    QApplication, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QMouseEvent

class ControlHandlersConfig:
    """Configuration constants for control handlers - loads from ROS parameters"""
    
    # Default values - will be overridden by load_config()
    SIMULATION_MODE = True
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200
    SERIAL_TIMEOUT = 1.0
    ACTIVATION_TIMEOUT = 5000
    CALIBRATION_DURATION = 5000
    CALIBRATION_TIMEOUT = 15000
    SIMULATION_RESPONSE_DELAY = 1000
    PULSE_INTERVAL = 250
    POSITION_MIN = -135
    POSITION_MAX = 135
    POWER_MIN = 0
    POWER_MAX = 100
    
    # Logging Configuration
    LOGGING_LEVEL = 'INFO'
    CONSOLE_OUTPUT = True
    
    # Object names for UI styling
    JOINT_ACTIVE = "joint-button-active"
    JOINT_INACTIVE = "joint-button-inactive"
    TOGGLE_WAITING = "toggle-waiting"
    TOGGLE_ACTIVE = "toggle-active"
    TOGGLE_INACTIVE = "toggle-inactive"
    
    @classmethod
    def load_config(cls):
        """Load configuration from ROS parameters"""
        try:
            # STM32 Communication Settings
            cls.SIMULATION_MODE = rospy.get_param('/stm32/simulation_mode', True)
            cls.SERIAL_PORT = rospy.get_param('/stm32/serial_port', '/dev/ttyUSB0')
            
            # Safe type conversion for numeric parameters
            baud_rate_param = rospy.get_param('/stm32/baud_rate', 115200)
            cls.BAUD_RATE = int(baud_rate_param) if isinstance(baud_rate_param, (int, float, str)) else 115200
            
            timeout_param = rospy.get_param('/stm32/timeout', 1.0)
            cls.SERIAL_TIMEOUT = float(timeout_param) if isinstance(timeout_param, (int, float, str)) else 1.0
            
            # System Parameters - safe integer conversion
            activation_timeout_param = rospy.get_param('/system/activation_timeout', 5000)
            cls.ACTIVATION_TIMEOUT = int(activation_timeout_param) if isinstance(activation_timeout_param, (int, float, str)) else 5000
            
            calibration_duration_param = rospy.get_param('/system/calibration_duration', 5000)
            cls.CALIBRATION_DURATION = int(calibration_duration_param) if isinstance(calibration_duration_param, (int, float, str)) else 5000
            
            calibration_timeout_param = rospy.get_param('/system/calibration_timeout', 15000)
            cls.CALIBRATION_TIMEOUT = int(calibration_timeout_param) if isinstance(calibration_timeout_param, (int, float, str)) else 15000
            
            simulation_delay_param = rospy.get_param('/system/simulation_response_delay', 1000)
            cls.SIMULATION_RESPONSE_DELAY = int(simulation_delay_param) if isinstance(simulation_delay_param, (int, float, str)) else 1000
            
            # Joint Parameters - safe integer conversion
            position_min_param = rospy.get_param('/joints/position_range/min', -135)
            cls.POSITION_MIN = int(position_min_param) if isinstance(position_min_param, (int, float, str)) else -135
            
            position_max_param = rospy.get_param('/joints/position_range/max', 135)
            cls.POSITION_MAX = int(position_max_param) if isinstance(position_max_param, (int, float, str)) else 135
            
            power_min_param = rospy.get_param('/joints/power_range/min', 0)
            cls.POWER_MIN = int(power_min_param) if isinstance(power_min_param, (int, float, str)) else 0
            
            power_max_param = rospy.get_param('/joints/power_range/max', 100)
            cls.POWER_MAX = int(power_max_param) if isinstance(power_max_param, (int, float, str)) else 100
            
            # Logging Configuration - safe string conversion
            logging_level_param = rospy.get_param('/logging/level', 'INFO')
            cls.LOGGING_LEVEL = str(logging_level_param) if isinstance(logging_level_param, (str, int, float)) else 'INFO'
            cls.CONSOLE_OUTPUT = rospy.get_param('/logging/console_output', True)
            
            # Configure ROS logging level if DEBUG is enabled
            if cls.LOGGING_LEVEL.upper() == 'DEBUG':
                rospy.loginfo("DEBUG logging level enabled - verbose terminal output activated")
                cls.DEBUG_MODE = True
            else:
                rospy.loginfo(f"Logging level set to: {cls.LOGGING_LEVEL}")
                cls.DEBUG_MODE = False
            
            rospy.loginfo("Configuration loaded from ROS parameters")
            rospy.loginfo(f"STM32 Serial Port: {cls.SERIAL_PORT}")
            rospy.loginfo(f"Baud Rate: {cls.BAUD_RATE}")
            rospy.loginfo(f"STM32 Simulation Mode: {'ENABLED' if cls.SIMULATION_MODE else 'DISABLED'}")
            
        except Exception as e:
            rospy.logwarn(f"Failed to load some config parameters: {e}")
            rospy.loginfo("Using default configuration values")

class ControlHandlers:
    """Class that manages interface button functions and system control logic"""
    
    def __init__(self, main_window):
        """Initialize control handlers with main window reference"""
        # Load configuration from ROS parameters
        ControlHandlersConfig.load_config()
        
        self.parent = main_window
        self.main_window = main_window
        self.pulse_timers = {}  # Dictionary to store animation timers
        self.calibration_in_progress = False  # Flag to track calibration state
        self.current_calibrating_button = None  # Reference to currently calibrating button
        self.current_calibrating_all_buttons = None  # Reference to all calibration buttons
        self.activation_timeout_timer = None  # Timer for activation timeout
        self.simulation_timer = None  # Timer for STM32 simulation
        self.calibration_timeout_timer = None  # Timer for calibration timeout
        
        # Joint state tracking - save joint states when system is deactivated/activated
        self._saved_joint_states = {}
        
        # System controls state tracking - prevent duplicate log messages
        self._last_system_controls_state = None
        
        # Mode reset state tracking - prevent duplicate "Mode reset to MANUAL" messages
        self._last_mode_reset_state = None
        
        # Performance: Cache frequently accessed UI elements
        self._button_cache = {
            'joint_buttons': [],
            'calibration_buttons': [],
            'mode_button': None,
            'activation_button': None
        }
        self._slider_cache = {
            'position_sliders': {},
            'power_sliders': {}
        }
        
        # Initialize cache after UI is ready
        if self.main_window:
            QTimer.singleShot(100, self._cache_ui_elements)
            # INITIALIZE SYSTEM WITH CONTROLS DISABLED AT STARTUP
            QTimer.singleShot(200, self._initialize_system_state)

        rospy.loginfo(f"STM32 Simulation Mode: {'ENABLED' if ControlHandlersConfig.SIMULATION_MODE else 'DISABLED'}")
        self._debug_log("ControlHandlers initialized successfully")
        self._debug_log("ControlHandlers initialization complete")
    
    def _debug_log(self, message, also_to_ui=False):
        """Helper method for debug logging - logs to terminal if DEBUG mode is enabled
        Args:
            message: Debug message to log
            also_to_ui: If True, also add to UI log panel
        """
        if hasattr(ControlHandlersConfig, 'DEBUG_MODE') and ControlHandlersConfig.DEBUG_MODE:
            rospy.loginfo(f"[DEBUG] {message}")
            if also_to_ui and self.main_window:
                self.main_window.add_log_message(f"[DEBUG] {message}")
    
    def _add_system_log(self, message, level="INFO"):
        """Add system event to UI log with timestamp and level"""
        if self.main_window:
            timestamp = rospy.Time.now().to_sec()
            log_message = f"[{level}] {message}"
            self.main_window.add_log_message(log_message)
            
            # Also log to terminal based on level
            if level == "ERROR":
                rospy.logerr(message)
            elif level == "WARN":
                rospy.logwarn(message)
            else:
                rospy.loginfo(message)
                
        # Add some useful system event logs
        if hasattr(ControlHandlersConfig, 'DEBUG_MODE') and ControlHandlersConfig.DEBUG_MODE:
            self._debug_log(f"System event logged: [{level}] {message}")
    
    def _log_system_event(self, event_type, details=""):
        """Log important system events to both terminal and UI"""
        timestamp = rospy.Time.now().to_sec()
        message = f"SYSTEM EVENT: {event_type}"
        if details:
            message += f" - {details}"
            
        self._debug_log(message, also_to_ui=True)
        
        # Log specific events of interest
        if event_type in ["ACTIVATION_TIMEOUT", "MODE_RESET", "SYSTEM_STATE_CHANGE"]:
            rospy.logwarn(f"Critical system event: {message}")
    
    def _initialize_system_state(self):
        """Set initial system state - system should be deactivated at startup"""
        # System starts in deactivated state
        self.main_window.system_activated = False
        
        # Set mode button to manual state but disabled
        self._reset_mode_to_manual_initial()
        
        # Disable all system controls
        self._set_system_controls_enabled(False)
        
        rospy.loginfo("System initialized in DEACTIVATED state")

    def _reset_mode_to_manual_initial(self):
        """Set mode button to MANUAL state at system startup"""
        mode_button = self._button_cache.get('mode_button')
        
        if not mode_button:
            # Cache miss - find and cache the mode button
            for btn in self.main_window.findChildren(QPushButton):
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    mode_button = btn
                    self._button_cache['mode_button'] = btn
                    break

        if mode_button:
            # PREVENT SIGNAL TRIGGERING
            mode_button.blockSignals(True)
            
            # Set mode button to MANUAL state
            mode_button.setText("MANUAL")
            mode_button.setObjectName("toggle-inactive")  # Red color (MANUAL)
            mode_button.setChecked(False)  # Reset toggle state
            mode_button.setEnabled(False)  # Disabled at startup
            mode_button.setStyleSheet("color: #888888 !important; background-color: #cccccc !important;")
            
            # RE-ENABLE SIGNALS
            mode_button.blockSignals(False)
            
            # Update internal state
            self.main_window.manual_mode_active = True
            
            # UPDATE STYLES
            if self.parent:
                self.parent.apply_styles()
            
            rospy.loginfo("Mode button initialized to MANUAL (disabled)")
        else:
            rospy.logwarn("Mode button not found for initialization!")

    # --- JOINT CONTROL FUNCTIONS ---
    def joint_button_clicked(self, joint_name):
        """Function called when joint control buttons are clicked"""
        print(f"Joint button pressed: {joint_name}")
        rospy.loginfo(f"Joint button pressed: {joint_name}")
        
        # TODO: Send joint control command to ROS topic
        # Example:
        # joint_msg = JointCommand()
        # joint_msg.joint_name = joint_name
        # self.joint_pub.publish(joint_msg)

    def handle_joint_click(self, button):
        """Simplified joint click handler - toggles joint activation state"""
        joint_name = button.text()
        is_active = button.objectName() == "joint-button-active"
        
        if is_active:
            self._deactivate_joint(button, joint_name)
        else:
            self._activate_joint(button, joint_name)
        
        self._update_joint_controls_state()
        self.main_window.apply_styles()
    
    def _activate_joint(self, button, joint_name):
        """Helper function to activate a joint"""
        button.setObjectName("joint-button-active")
        self.main_window.add_log_message(f"Joint activated: {joint_name}")
        self._set_joint_sliders_enabled(joint_name, True)
        self._log_combined_values(joint_name)
        
        # Publish initial joint command when joint is activated
        self._publish_joint_command()
        
        # Disable other controls when joint is active
        self._set_calibration_buttons_enabled(False)
        self._set_mode_button_enabled(False)
    
    def _deactivate_joint(self, button, joint_name):
        """Helper function to deactivate a joint"""
        button.setObjectName("joint-button-inactive")
        self.main_window.add_log_message(f"Joint deactivated: {joint_name}")
        self._set_joint_sliders_enabled(joint_name, False)
        
        # Don't publish any command when deactivating joint - just stop controlling it
        # This prevents unwanted zero messages when closing joint controls

    def _update_joint_controls_state(self):
        """Update the state of joint controls based on current configuration"""
        any_joint_active = any(btn.objectName() == "joint-button-active" for btn in self.main_window.ui_elements["joint_buttons"])
        self.main_window.joint_controls_active = any_joint_active
        
        if any_joint_active:
            # Disable calibration buttons when any joint is active
            self._set_calibration_buttons_enabled(False)
            # Disable mode button when joint is active
            self._set_mode_button_enabled(False)
        else:
            # Enable calibration buttons when no joint is active
            self._set_calibration_buttons_enabled(True)
            # Enable mode button when no joint is active
            self._set_mode_button_enabled(True)

    def _safe_operation(self, operation, error_msg="Operation failed"):
        """Safely execute operations with error handling"""
        try:
            return operation()
        except Exception as e:
            rospy.logerr(f"{error_msg}: {e}")
            return None
    
    def _set_joint_sliders_enabled(self, joint_name, enabled):
        """Safer slider management - enable/disable sliders for specific joint"""
        def enable_sliders():
            position_slider = self.main_window.findChild(QSlider, f"position-slider-{joint_name}")
            power_slider = self.main_window.findChild(QSlider, f"power-slider-{joint_name}")
            
            for slider in [position_slider, power_slider]:
                if slider:
                    slider.setEnabled(enabled)
                    self._enable_arrow_buttons(slider.parent(), enabled)
        
        self._safe_operation(enable_sliders, f"Failed to set sliders for {joint_name}")

    def position_slider_changed(self, joint_name, value):
        """Called when position slider value changes - publishes JointMessage"""
        print(f"Position changed for {joint_name}: {value}")
        # Log only for active joints - to log together with power value
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)
            # Publish joint command when slider changes
            self._publish_joint_command()

    def power_slider_changed(self, joint_name, value):
        """Called when power slider value changes - publishes JointMessage"""
        print(f"Power changed for {joint_name}: {value}")
        # Log only for active joints - to log together with position value
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)
            # Publish joint command when slider changes
            self._publish_joint_command()

    def _is_joint_active(self, joint_name):
        """Check if specified joint is currently active"""
        for widget in self.main_window.findChildren(QPushButton):
            if (widget.objectName() == "joint-button-active" and 
                widget.text() == joint_name):
                return True
        return False

    def _log_combined_values(self, joint_name):
        """Log position and power values together for a joint"""
        position_value, power_value = self._get_joint_values(joint_name)
        self.main_window.add_log_message(f"{joint_name} - Position: {position_value}°, Power: {power_value}%")

    def _get_joint_values(self, joint_name):
        """Get current position and power values for specified joint"""
        pos_slider = self.main_window.ui_elements["position_sliders"].get(joint_name)
        pow_slider = self.main_window.ui_elements["power_sliders"].get(joint_name)
        
        position_value = pos_slider.value() if pos_slider else 0
        power_value = pow_slider.value() if pow_slider else 0
        
        return position_value, power_value

    def _log_joint_values(self, joint_name):
        """Log current values when joint is activated"""
        self._log_combined_values(joint_name)

    def _get_active_joint(self):
        """Get the name of currently active (green) joint"""
        for widget in self.main_window.findChildren(QPushButton):
            if widget.objectName() == "joint-button-active":
                return widget.text()
        return None

    # --- CALIBRATION FUNCTIONS ---
    def calibration_button_clicked(self, calibration_name):
        """Function called when calibration buttons are clicked"""
        print(f"Calibration button pressed: {calibration_name}")
        rospy.loginfo(f"Calibration button pressed: {calibration_name}")
        
        # Create communication message for calibration
        comm_msg = self._create_communication_message(comStat=1, calibStat=1)
        
        # Publish calibration command to STM32
        self._publish_communication(comm_msg)
        
        # Add log message to UI
        self.main_window.add_log_message(f"Calibration started: {calibration_name}")
        
        rospy.loginfo(f"Calibration command sent to STM32: {calibration_name}")

    def handle_calibration_click(self, btn, all_buttons):
        """Handle calibration button click with visual feedback and process management"""
        print(f"DEBUG: Calibration button clicked: {btn.text()}")
        
        # If calibration is already in progress, don't start new one
        if self.calibration_in_progress:
            rospy.logwarn(f"Calibration already in progress for {self.current_calibrating_button.text() if self.current_calibrating_button else 'unknown'}. Please wait...")
            print(f"Calibration blocked: {btn.text()} - Another calibration in progress")
            return
        
        # Disable joint buttons when calibration starts
        self._set_joint_buttons_enabled(False)
        
        # Start calibration process
        self.calibration_in_progress = True
        self.current_calibrating_button = btn
        self.main_window.calibration_active = True  # Update state
        
        # Disable mode button when calibration starts
        print("DEBUG: Calibration started - disabling mode button")
        self._set_mode_button_enabled(False)
        
        # First call the handler
        self.calibration_button_clicked(btn.text())
        
        # UI update - disable ALL buttons (including processing one)
        for b in all_buttons:
            if b == btn:
                b.setObjectName("calibration-processing")
                b.setEnabled(False)  # Disable processing button too
            else:
                b.setObjectName("calibration-disabled")
                b.setEnabled(False)  # Disable other buttons too

        # Send style apply signal to parent
        if self.parent:
            self.parent.apply_styles()
            
            # Start pulse animation
            self._start_pulse_animation(btn)
            
            # Store current calibrating button for response handling
            self.current_calibrating_button = btn
            self.current_calibrating_all_buttons = all_buttons
            
            # Check if simulation mode - send automatic response
            if ControlHandlersConfig.SIMULATION_MODE:
                # In simulation mode, send automatic calibration completion response
                self.simulation_timer = QTimer()
                self.simulation_timer.setSingleShot(True)
                self.simulation_timer.timeout.connect(self._simulate_calibration_response)
                self.simulation_timer.start(ControlHandlersConfig.SIMULATION_RESPONSE_DELAY)
                rospy.loginfo(f"SIMULATION: Calibration {btn.text()} - auto-response in {ControlHandlersConfig.SIMULATION_RESPONSE_DELAY/1000} seconds")
            else:
                # Real mode - wait for STM32 response with timeout
                self._start_calibration_timeout()
                rospy.loginfo(f"Calibration started: {btn.text()} - waiting for STM32 response (timeout: {ControlHandlersConfig.CALIBRATION_TIMEOUT/1000}s)...")

    def _start_pulse_animation(self, btn):
        """Start pulse animation for calibration button during processing"""
        btn_id = id(btn)
        if btn_id in self.pulse_timers:
            self.pulse_timers[btn_id].stop()
            del self.pulse_timers[btn_id]
        
        timer = QTimer()
        pulse_state = [True]  # Use list for mutable reference
        
        def toggle_opacity():
            if hasattr(btn, 'setStyleSheet') and btn.objectName() == "calibration-processing":
                if pulse_state[0]:
                    # Opaque state
                    btn.setStyleSheet("""
                        QPushButton#calibration-processing {
                            background-color: #ffc107 !important;
                            color: #212529 !important;
                            border: 2px solid #e0a800;
                            border-radius: 8px;
                            padding: 8px 16px;
                            font-weight: bold;
                            font-size: 12px;
                        }
                    """)
                else:
                    # Transparent state
                    btn.setStyleSheet("""
                        QPushButton#calibration-processing {
                            background-color: rgba(255, 193, 7, 0.6) !important;
                            color: #212529 !important;
                            border: 2px solid #e0a800;
                            border-radius: 8px;
                            padding: 8px 16px;
                            font-weight: bold;
                            font-size: 12px;
                        }
                    """)
                pulse_state[0] = not pulse_state[0]
        
        timer.timeout.connect(toggle_opacity)
        timer.start(ControlHandlersConfig.PULSE_INTERVAL)  # Use config
        self.pulse_timers[btn_id] = timer

    def finish_calibration(self, btn, all_buttons):
        """Mark calibration as completed and reset UI state"""
        # First check if calibration is still in progress
        if not self.calibration_in_progress:
            rospy.loginfo("Calibration finish ignored - calibration was reset")
            return
        
        # Timer cleanup
        btn_id = id(btn)
        completion_timer_id = f"completion_{btn_id}"
        
        # Stop calibration timeout timer
        self._stop_calibration_timeout()
        
        # Stop pulse timer
        if btn_id in self.pulse_timers:
            try:
                self.pulse_timers[btn_id].stop()
                del self.pulse_timers[btn_id]
            except Exception as e:
                print(f"Pulse timer stop error: {e}")
    
        # Stop completion timer
        if completion_timer_id in self.pulse_timers:
            try:
                self.pulse_timers[completion_timer_id].stop()
                del self.pulse_timers[completion_timer_id]
            except Exception as e:
                print(f"Completion timer stop error: {e}")

        # Clear style and set to completed state
        if hasattr(btn, 'setStyleSheet'):
            btn.setStyleSheet("")  # Clear inline style
            btn.setObjectName("calibration-completed")
            btn.setEnabled(True)  # Re-enable completed button

            # Re-enable other buttons and set to default state
            for b in all_buttons:
                if b != btn:
                    b.setObjectName("calibration-default")
                    b.setEnabled(True)  # Re-enable buttons
    
        # Re-enable joint buttons when calibration completes
        self._set_joint_buttons_enabled(True)
        
        # Reset joint button text color to normal
        joint_buttons = self.main_window.findChildren(QPushButton)
        for joint_btn in joint_buttons:
            if joint_btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                joint_btn.setStyleSheet("")  # Clear gray color, return to normal

        # Reset calibration state
        self.calibration_in_progress = False
        self.current_calibrating_button = None
        self.main_window.calibration_active = False  # Update state
        
        # Re-enable mode button when calibration completes
        self._set_mode_button_enabled(True)
        
        if self.parent:
            self.parent.apply_styles()

    def _set_mode_button_enabled(self, enabled):
        """Enable/disable MODE button using cache"""
        mode_button = self._button_cache.get('mode_button')
        
        if not mode_button:
            # Cache miss - find and cache
            for btn in self.main_window.findChildren(QPushButton):
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    mode_button = btn
                    self._button_cache['mode_button'] = btn
                    break

        if mode_button:
            mode_button.setEnabled(enabled)
            
            if not enabled:
                mode_button.setStyleSheet("""
                    QPushButton {
                        color: #888888 !important; 
                        background-color: #cccccc !important;
                        border: 2px solid #999999 !important;
                    }
                """)
                mode_button.setProperty("mode_disabled", True)
                self.main_window.add_log_message("Mode button disabled")
            else:
                mode_button.setStyleSheet("")
                mode_button.setProperty("mode_disabled", False)
                self.main_window.add_log_message("Mode button enabled")
        
            mode_button.update()
            if self.parent:
                self.parent.apply_styles()
        else:
            print("DEBUG: Mode button not found!")

    def _set_calibration_buttons_enabled(self, enabled):
        """Enable/disable calibration buttons"""
        if not self.main_window:
            return
    
        calibration_buttons = self.main_window.findChildren(QPushButton)
        for btn in calibration_buttons:
            if btn.objectName() in ["calibration-default", "calibration-completed", "calibration-disabled"]:
                btn.setEnabled(enabled)
                if enabled:
                    # When enabling: set object name based on previous state
                    if btn.objectName() == "calibration-disabled":
                        btn.setObjectName("calibration-default")
                    # Normal color
                    btn.setStyleSheet("")
                else:
                    # When disabling: set to disabled
                    btn.setObjectName("calibration-disabled")
                    # Gray text color
                    btn.setStyleSheet("color: #888888 !important;")

        # Refresh styles
        if self.parent:
            self.parent.apply_styles()

    def _set_joint_buttons_enabled(self, enabled):
        """Enable/disable joint buttons"""
        if not self.main_window:
            return
    
        joint_buttons = self.main_window.findChildren(QPushButton)
        for btn in joint_buttons:
            if btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                btn.setEnabled(enabled)
                if not enabled:
                    # When disabling: also deactivate active ones
                    if btn.objectName() == "joint-button-active":
                        btn.setObjectName("joint-button-inactive")
                        joint_name = btn.text()
                        self._set_joint_sliders_enabled(joint_name, False)
                # Gray text color
                btn.setStyleSheet("color: #888888 !important;")
            else:
                # Normal color
                btn.setStyleSheet("")

        # Refresh styles
        if self.parent:
            self.parent.apply_styles()

    def handle_toggle(self, button, checked, on_text, off_text, handler):
        """Generic toggle button handler with visual feedback"""
        if checked:
            on_text_display = on_text
            off_text_display = off_text
            
            # Call handler first
            handler(on_text)
            
            # Special case for activation buttons - color is set in handler
            if "ACTIVATED" in on_text or "DEACTIVATED" in on_text:
                # Let handler handle color setting for activation buttons
                button.setText(on_text_display)
                return
            
            # Normal toggle logic for other buttons
            button.setText(on_text_display)
            button.setObjectName("toggle-active")
            print(f"Button set to: {on_text_display} (active/green)")
            
        else:
            on_text_display = off_text
            off_text_display = on_text
            
            # Call handler first
            handler(off_text)
            
            # Special case for activation buttons
            if "ACTIVATED" in off_text or "DEACTIVATED" in off_text:
                button.setText(on_text_display)
                button.setObjectName("toggle-inactive")  # Red
                print(f"Button set to: {on_text_display} (inactive/red)")
                return
            
            # Normal toggle logic for other buttons
            button.setText(on_text_display)
            button.setObjectName("toggle-inactive")
            print(f"Button set to: {on_text_display} (inactive/red)")
    
        # Update styles
        if self.parent:
            self.parent.apply_styles()

    # --- ACTIVATION AND MODE FUNCTIONS ---
    def activation_button_clicked(self, status):
        """Refactored activation handler - manages system activation state"""
        rospy.loginfo(f"Activation status changed: {status}")
        self._debug_log(f"Activation button clicked: {status}")
        
        # Get from cache, if not found, find and cache
        activation_button = self._button_cache.get('activation_button')
        if not activation_button:
            all_buttons = self.main_window.findChildren(QPushButton)
            for btn in all_buttons:
                if "ACTIVATED" in btn.text() or "DEACTIVATED" in btn.text():
                    activation_button = btn
                    self._button_cache['activation_button'] = btn
                    break

        if status == "ACTIVATED":
            self._debug_log("Processing ACTIVATED state")
            # Create communication message for activation
            comm_msg = self._create_communication_message(comStat=1)
            
            # Set button to waiting state
            if activation_button:
                self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_WAITING, "ACTIVATED")
                self.main_window.add_log_message("Waiting for STM32 communication confirmation...")
                self._start_activation_timeout()
                self._debug_log("Activation timeout started")
            else:
                rospy.logerr("Activation button not found!")
                
        else:  # DEACTIVATED
            self._debug_log("Processing DEACTIVATED state - starting deactivation sequence")
            # Create communication message for deactivation
            comm_msg = self._create_communication_message(comStat=0)
            
            # Stop timeout and set button to inactive
            self._stop_activation_timeout()
            if activation_button:
                self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_INACTIVE, "DEACTIVATED")
            
            # 1. FIRST CHANGE SYSTEM STATE
            self.main_window.system_activated = False
            self._debug_log("System state set to deactivated")
            
            # 2. RESET CALIBRATION AND MODE STATE (without affecting controls)
            self._reset_calibration_state()
            self._debug_log("Calibration state reset")
            self._reset_mode_to_manual()
            self._debug_log("Mode reset to manual completed")
            
            # 3. FINALLY DISABLE CONTROLS
            self._set_system_controls_enabled(False)
    
        # Publish message
        self._publish_communication(comm_msg)

    def _reset_mode_to_manual(self):
        """Reset mode button to MANUAL when system is deactivated"""
        self._debug_log(f"_reset_mode_to_manual called - current state: {self._last_mode_reset_state}")
        
        # Prevent duplicate log messages - check if mode is already reset
        if self._last_mode_reset_state == "MANUAL":
            self._debug_log("Mode already reset to MANUAL - skipping duplicate reset")
            return
        
        # Set state immediately to prevent race conditions
        self._last_mode_reset_state = "MANUAL"
        self._debug_log("Mode reset state set to MANUAL")
        self._log_system_event("MODE_RESET", "Resetting mode to MANUAL")
        
        mode_button = self._button_cache.get('mode_button')
        
        if not mode_button:
            # Cache miss - find and cache
            for btn in self.main_window.findChildren(QPushButton):
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    mode_button = btn
                    self._button_cache['mode_button'] = btn
                    break

        if mode_button:
            # TEMPORARILY DISCONNECT SIGNALS
            try:
                mode_button.clicked.disconnect()
            except:
                pass
            
            # PREVENT SIGNAL TRIGGERING
            mode_button.blockSignals(True)
            
            # Set mode button to MANUAL state
            mode_button.setText("MANUAL")
            mode_button.setObjectName("toggle-inactive")  # Red color (MANUAL)
            mode_button.setChecked(False)  # Reset toggle state
            
            # RE-ENABLE SIGNALS
            mode_button.blockSignals(False)
            
            # Update internal state
            self.main_window.manual_mode_active = True
            
            # Publish autonomous mode deactivation to ROS when resetting to MANUAL
            if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
                self.main_window.ros_bridge.publish_autonomous_mode(False)
            
            # Log message
            self.main_window.add_log_message("Mode reset to MANUAL")
            rospy.loginfo("Mode reset to MANUAL on system deactivation")
            self._debug_log("Mode reset to MANUAL completed - UI and state updated")
            
            # RECONNECT SIGNALS - BUT MANUAL CONNECTION!
            # mode_button.clicked.connect(...) - REMOVE THIS LINE
        else:
            rospy.logwarn("Mode button not found for reset!")
            self._debug_log("Mode button not found for reset!")

    def _reset_calibration_state(self):
        """Reset calibration state when system is deactivated"""
        if self.calibration_in_progress:
            rospy.loginfo("System deactivated - resetting calibration state")
            
            # STOP ALL TIMERS (pulse + completion + calibration timeout timers)
            self._stop_calibration_timeout()
            
            for timer_id, timer in list(self.pulse_timers.items()):
                try:
                    timer.stop()
                    timer.deleteLater()
                except Exception as e:
                    rospy.logwarn(f"Timer cleanup error: {e}")
            self.pulse_timers.clear()
            
            # Set calibration buttons to default state - BUT DON'T ENABLE!
            calibration_buttons = self.main_window.findChildren(QPushButton)
            for btn in calibration_buttons:
                if btn.objectName() in ["calibration-processing", "calibration-completed"]:
                    btn.setObjectName("calibration-default")
                    btn.setStyleSheet("")  # Clear inline styles
                    # btn.setEnabled(True) REMOVE THIS - Don't enable buttons when system is deactivated
            
            # Reset calibration state
            self.calibration_in_progress = False
            self.current_calibrating_button = None
            self.current_calibrating_all_buttons = None
            self.main_window.calibration_active = False
            
            # Log message
            self.main_window.add_log_message("Calibration state reset on system deactivation")
            
            rospy.loginfo("All calibration timers stopped and state reset")
        else:
            # Even if calibration is not in progress, check buttons (for safety)
            calibration_buttons = self.main_window.findChildren(QPushButton)
            for btn in calibration_buttons:
                if btn.objectName() in ["calibration-processing", "calibration-completed"]:
                    btn.setObjectName("calibration-default")
                    btn.setStyleSheet("")

    def _start_activation_timeout(self):
        """Start activation timeout timer"""
        self._stop_activation_timeout()
        
        # In simulation mode, send automatic successful response
        if ControlHandlersConfig.SIMULATION_MODE:
            self._start_simulation_response()
            rospy.loginfo(f"STM32 Simulation Mode: Auto-response in {ControlHandlersConfig.SIMULATION_RESPONSE_DELAY/1000} seconds")
        else:
            # Normal timeout timer - button will turn RED if timeout expires
            self.activation_timeout_timer = QTimer()
            self.activation_timeout_timer.setSingleShot(True)
            self.activation_timeout_timer.timeout.connect(self._activation_timeout_callback)
            self.activation_timeout_timer.start(ControlHandlersConfig.ACTIVATION_TIMEOUT)
            rospy.loginfo(f"Activation timeout started ({ControlHandlersConfig.ACTIVATION_TIMEOUT/1000} seconds) - waiting for STM32 response")

    def _start_simulation_response(self):
        """Simulate STM32 response in simulation mode"""
        self.simulation_timer = QTimer()
        self.simulation_timer.setSingleShot(True)
        self.simulation_timer.timeout.connect(self._simulate_stm32_response)
        self.simulation_timer.start(ControlHandlersConfig.SIMULATION_RESPONSE_DELAY)

    def _simulate_stm32_response(self):
        """Simulate successful response from STM32"""
        rospy.loginfo("SIMULATION: STM32 communication successful")
        
        # Create mock ControlMessage
        class MockControlMsg:
            def __init__(self):
                self.comStatus = 1  # 1 = active
                self.calibStatus = 0  # 0 = normal
        
        # Call handle_communication
        self.main_window.handle_communication(MockControlMsg())
        self.simulation_timer = None

    def _simulate_calibration_response(self):
        """Simulate calibration completion response from STM32"""
        rospy.loginfo("SIMULATION: STM32 calibration completed")
        
        # Create mock ControlMessage with calibration status
        class MockControlMsg:
            def __init__(self):
                self.comStatus = 1  # 1 = active
                self.calibStatus = 1  # 1 = calibration completed
        
        # Call handle_communication
        self.main_window.handle_communication(MockControlMsg())
        self.simulation_timer = None

    def _stop_activation_timeout(self):
        """Stop timeout timer"""
        if hasattr(self, 'activation_timeout_timer') and self.activation_timeout_timer:
            self.activation_timeout_timer.stop()
            self.activation_timeout_timer = None
        
        # Also stop simulation timer
        if hasattr(self, 'simulation_timer') and self.simulation_timer:
            self.simulation_timer.stop()
            self.simulation_timer = None
            
        # Also stop calibration timeout
        self._stop_calibration_timeout()

    def toggle_simulation_mode(self):
        """Toggle simulation mode on/off"""
        ControlHandlersConfig.SIMULATION_MODE = not ControlHandlersConfig.SIMULATION_MODE
        status = "ENABLED" if ControlHandlersConfig.SIMULATION_MODE else "DISABLED"
        rospy.loginfo(f"STM32 Simulation Mode: {status}")
        self.main_window.add_log_message(f"STM32 Simulation Mode: {status}")

    def _activation_timeout_callback(self):
        """Activation timeout callback - called when STM32 doesn't respond"""
        rospy.logwarn("STM32 communication timeout - activation failed")
        self._debug_log("TIMEOUT CALLBACK: STM32 activation timeout - starting timeout recovery sequence")
        self._log_system_event("ACTIVATION_TIMEOUT", "STM32 communication failed")
        
        # Get activation button from cache
        activation_button = self._button_cache.get('activation_button')
        if not activation_button:
            # Fallback - search directly
            all_buttons = self.main_window.findChildren(QPushButton)
            for btn in all_buttons:
                if "ACTIVATED" in btn.text():
                    activation_button = btn
                    break
    
        if activation_button:
            # Properly reset button state to DEACTIVATED
            self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_INACTIVE, "DEACTIVATED")
            self.main_window.add_log_message("STM32 communication timeout - activation failed")
            
            # Reset system state completely
            self.main_window.system_activated = False
            self._debug_log("TIMEOUT CALLBACK: System state set to deactivated")
            self._log_system_event("SYSTEM_STATE_CHANGE", "System deactivated due to timeout")
            
            # Reset calibration state
            self._reset_calibration_state()
            self._debug_log("TIMEOUT CALLBACK: Calibration state reset")
            
            # Reset mode to MANUAL (without triggering signals)
            self._debug_log("TIMEOUT CALLBACK: About to call _reset_mode_to_manual")
            self._reset_mode_to_manual()
            self._debug_log("TIMEOUT CALLBACK: _reset_mode_to_manual completed")
            
            # Disable all system controls AFTER resetting states
            self._set_system_controls_enabled(False)
            
            # Clear any pending activation state
            self._clear_activation_state()
    
        self.activation_timeout_timer = None

    def _clear_activation_state(self):
        """Clear any pending activation state to ensure clean state"""
        # Stop any running timers
        self._stop_activation_timeout()
        
        # Reset internal flags
        if hasattr(self, '_activation_in_progress'):
            self._activation_in_progress = False
            
        # Clear button cache states to prevent stale references
        if 'activation_button' in self._button_cache:
            activation_button = self._button_cache['activation_button']
            # Ensure button is visually reset
            if activation_button and activation_button.isVisible():
                activation_button.setChecked(False)
                activation_button.repaint()  # Force repaint
                
        # Ensure system is in proper deactivated state
        self.main_window.system_activated = False
        
        rospy.loginfo("Activation state cleared - system ready for new activation attempt")

    def mode_button_clicked(self, button_text):
        """Called when mode button (MANUAL/AUTONOMOUS) is clicked"""
        rospy.loginfo(f"Mode button clicked: {button_text}")
        
        # Reset mode reset state tracking when user manually changes mode
        # This allows the "Mode reset to MANUAL" message to appear again when needed
        self._last_mode_reset_state = None
        
        # Update manual mode state
        if button_text == "MANUAL":
            self.main_window.manual_mode_active = True
            rospy.loginfo("Manual mode activated - image clicks enabled")
            
            # Enable controls in MANUAL mode
            self._set_manual_controls_enabled(True)
            
            # Publish autonomous mode deactivation to ROS
            if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
                self.main_window.ros_bridge.publish_autonomous_mode(False)
            
        elif button_text == "AUTONOMOUS":
            self.main_window.manual_mode_active = False
            rospy.loginfo("Autonomous mode activated - image clicks disabled")
            
            # Disable controls in AUTONOMOUS mode
            self._set_manual_controls_enabled(False)
            
            # Publish autonomous mode activation to ROS
            if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
                self.main_window.ros_bridge.publish_autonomous_mode(True)
        
        # Add log message
        self.main_window.add_log_message(f"Mode changed to: {button_text}")

    def _set_manual_controls_enabled(self, enabled):
        """Enable/disable manual controls (joint and calibration) based on mode"""
        if not self.main_window:
            return
    
        ui = self.main_window.ui_elements

        # Joint buttons
        for btn in ui["joint_buttons"]:
            btn.setEnabled(enabled)
            if not enabled:
                # Deactivate active joints in AUTONOMOUS mode
                if btn.objectName() == "joint-button-active":
                    btn.setObjectName("joint-button-inactive")
                    self._set_joint_sliders_enabled(btn.text(), False)
                btn.setStyleSheet("color: #888888 !important;")
            else:
                btn.setStyleSheet("")

        # Calibration buttons
        for btn in ui["calibration_buttons"]:
            btn.setEnabled(enabled)
            if not enabled:
                btn.setObjectName("calibration-disabled")
                btn.setStyleSheet("color: #888888 !important;")
            else:
                if btn.objectName() == "calibration-disabled":
                    btn.setObjectName("calibration-default")
                btn.setStyleSheet("")

        # Sliders - ONLY ENABLE SLIDERS OF ACTIVE JOINTS
        if enabled:
            # When switching to MANUAL mode: only enable sliders of active joints
            for joint_name, slider in ui["position_sliders"].items():
                joint_is_active = self._is_joint_active(joint_name)
                slider.setEnabled(joint_is_active)
                
            for joint_name, slider in ui["power_sliders"].items():
                joint_is_active = self._is_joint_active(joint_name)
                slider.setEnabled(joint_is_active)
        else:
            # When switching to AUTONOMOUS mode: disable all sliders
            all_sliders = list(ui["position_sliders"].values()) + list(ui["power_sliders"].values())
            for slider in all_sliders:
                slider.setEnabled(False)

        # Arrow buttons - ONLY ENABLE ARROW BUTTONS OF ACTIVE JOINTS
        if enabled:
            # When switching to MANUAL mode: check for each joint
            for btn in ui["arrow_buttons"]:
                # Find which joint this arrow button belongs to
                parent_widget = btn.parent()
                if parent_widget:
                    # Find slider in parent widget
                    sliders = parent_widget.findChildren(QSlider)
                    if sliders:
                        slider = sliders[0]
                        # If slider is active, arrow button should be active too
                        btn.setEnabled(slider.isEnabled())
                        if not slider.isEnabled():
                            btn.setStyleSheet("color: #888888 !important;")
                        else:
                            btn.setStyleSheet("")
        else:
            # When switching to AUTONOMOUS mode: disable all arrow buttons
            for btn in ui["arrow_buttons"]:
                btn.setEnabled(False)
                btn.setStyleSheet("color: #888888 !important;")

        # Update joint controls state
        if not enabled:
            self.main_window.joint_controls_active = False

        # Status message
        status = "enabled" if enabled else "disabled"
        self.main_window.add_log_message(f"Manual controls {status}")
        rospy.loginfo(f"Manual controls {status}")
        
        # Refresh styles
        if self.parent:
            self.parent.apply_styles()

    # --- HELP AND IMAGE FUNCTIONS ---
    def show_manual_help(self):
        """Show manual mode help message"""
        print("Help button pressed")
        if self.parent:
            QMessageBox.information(
                self.parent,
                "Manual Mode Help",
                "In this mode, you can manually adjust joint and effector controls.\n"
                "• Click buttons.\n"
                "• Drag sliders.\n"
                "• Values are updated instantly.\n\n"
                "Camera Image Mouse Controls:\n"
                "• Left Click: Shows pixel coordinates in log\n"
                "• Right Click: Triggers 'triggered' message in log\n"
                "• Only works when ACTIVATED + MANUAL mode + no active joints/calibration"
            )

    def create_help_button(self, parent_widget):
        """Create '?' button over camera image"""
        help_btn = QPushButton("❓", parent=parent_widget)
        help_btn.setObjectName("help-button")
        help_btn.setFixedSize(40, 40)
        
        help_btn.clicked.connect(self.show_manual_help)
        QTimer.singleShot(0, lambda: self._reposition_help_btn(help_btn, parent_widget))
        
        return help_btn

    def _reposition_help_btn(self, help_btn, parent_widget):
        """Position help button in top-right corner of parent widget"""
        if parent_widget and help_btn:
            size = help_btn.size()
            margin = 10
            parent_width = parent_widget.width()
            help_btn.move(parent_width - size.width() - margin, margin)

    def reposition_help_button(self, help_btn, parent_widget):
        """External callable helper method for positioning help button"""
        self._reposition_help_btn(help_btn, parent_widget)

    # --- THEME AND SYSTEM FUNCTIONS ---
    def change_theme(self, theme_name):
        """Function for theme change"""
        self._debug_log(f"Theme changed: {theme_name}", also_to_ui=False)
        
        # Apply theme through resource manager
        if self.parent:
            from ui.resources.resource import resource_manager
            resource_manager.set_theme(theme_name)

    def handle_theme_change(self, theme):
        """Handle theme change with style application and button state update"""
        self.change_theme(theme)
        
        # Update theme button styles
        self._update_theme_button_styles(theme)
        
        if self.parent:
            self.parent.apply_styles()

    def _update_theme_button_styles(self, active_theme):
        """Update theme button styles to show which one is active"""
        if not self.parent:
            return
            
        # Find all theme buttons and update their style
        theme_buttons = self.parent.findChildren(QPushButton)
        
        for btn in theme_buttons:
            # Only check theme buttons
            if btn.objectName() in ["theme-button-normal", "theme-button-active"]:
                # Determine which theme this button belongs to from tooltip
                is_light_btn = "Light Theme" in btn.toolTip() or "Açık Tema" in btn.toolTip()
                is_dark_btn = "Dark Theme" in btn.toolTip() or "Koyu Tema" in btn.toolTip()
                
                # Make button blue if it matches active theme
                if (is_light_btn and active_theme == "light_theme") or \
                   (is_dark_btn and active_theme == "dark_theme"):
                    btn.setObjectName("theme-button-active")
                else:
                    btn.setObjectName("theme-button-normal")

    def close_app(self):
        """Close application safely"""
        if self.parent and hasattr(self.parent, 'shutdown_initiated'):
            if self.parent.shutdown_initiated:
                return
            self.parent.shutdown_initiated = True
            
            print("Close button pressed - Closing application...")
            rospy.loginfo("Close button pressed - Closing application...")
            
            # Close ROS bridge - safe check
            if hasattr(self.parent, 'ros_bridge') and hasattr(self.parent.ros_bridge, 'shutdown'):
                try:
                    self.parent.ros_bridge.shutdown()
                except Exception as e:
                    rospy.logwarn(f"ROS bridge shutdown error: {e}")
            
            # Close ROS (only once)
            try:
                if not rospy.is_shutdown():
                    rospy.signal_shutdown("User exited from interface.")
            except Exception as e:
                print(f"ROS shutdown error (normal): {e}")
            
            # Close Qt application directly
            if self.parent:
                self.parent.close()

    def handle_border_toggle(self, btn):
        """Handle border toggle button click for debug visualization"""
        # Change border state through parent
        if self.parent:
            self.parent.show_borders = not self.parent.show_borders
            self.parent.update_borders()
            
            # Update button style
            if self.parent.show_borders:
                btn.setObjectName("border-button-active")  # Red (on)
                rospy.loginfo("Debug borders enabled")
                print("Debug borders enabled")
            else:
                btn.setObjectName("border-button-normal")   # Transparent (off)
                rospy.loginfo("Debug borders disabled")
                print("Debug borders disabled")
        
            # Re-apply style
            self.parent.apply_styles()
        else:
            rospy.logwarn("Parent window not found for border toggle")

    def _set_system_controls_enabled(self, enabled):
        """Enable/disable all controls based on system activation state"""
        if not self.main_window:
            return

        self._debug_log(f"_set_system_controls_enabled called with enabled={enabled}")

        # Check if we already have this state - prevent duplicate log messages
        if hasattr(self, '_last_system_controls_state') and self._last_system_controls_state == enabled:
            self._debug_log("System controls state unchanged - skipping duplicate call")
            return  # No change needed, avoid duplicate log messages
        
        # Store the new state
        self._last_system_controls_state = enabled
        self._debug_log(f"System controls state changed to: {enabled}")

        ui = self.main_window.ui_elements

        # Joint buttons - SAVE STATE WHEN SYSTEM IS DEACTIVATED
        if not enabled:
            # When disabling: save state of active joints
            self._saved_joint_states = {}
            for btn in ui["joint_buttons"]:
                joint_name = btn.text()
                was_active = btn.objectName() == "joint-button-active"
                self._saved_joint_states[joint_name] = was_active
                
                btn.setEnabled(False)
                if was_active:
                    btn.setObjectName("joint-button-inactive")
                    self._set_joint_sliders_enabled(joint_name, False)
                btn.setStyleSheet("color: #888888 !important;")
        else:
            # When enabling: START ALL JOINTS AS INACTIVE (for image clicks to work)
            for btn in ui["joint_buttons"]:
                joint_name = btn.text()
                btn.setEnabled(True)
                btn.setStyleSheet("")
                
                # When system is re-activated, joints should start inactive
                # This way image clicks can work
                btn.setObjectName("joint-button-inactive")
                self._set_joint_sliders_enabled(joint_name, False)

        # Calibration buttons - FORCE GRAY
        for btn in ui["calibration_buttons"]:
            btn.setEnabled(enabled)
            if not enabled:
                # Set object name to disabled
                if btn.objectName() not in ["calibration-disabled"]:
                    btn.setObjectName("calibration-disabled")
                # Gray in both AUTONOMOUS and DEACTIVATED modes
                btn.setStyleSheet("color: #888888 !important;")
            else:
                # Return to default state when system is active
                if btn.objectName() == "calibration-disabled":
                    btn.setObjectName("calibration-default")
                btn.setStyleSheet("")

        # Mode button - Should be disabled when system is DEACTIVATED
        mode_button = self._button_cache.get('mode_button')
        if not mode_button:
            # If not found in ui_elements, search with findChildren
            all_buttons = self.main_window.findChildren(QPushButton)
            for btn in all_buttons:
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    mode_button = btn
                    self._button_cache['mode_button'] = btn
                    break

        if mode_button:
            mode_button.setEnabled(enabled)
            if not enabled:
                mode_button.setStyleSheet("color: #888888 !important; background-color: #cccccc !important;")
            else:
                mode_button.setStyleSheet("")

        # Sliders - FORCE DISABLE
        if hasattr(ui, 'position_sliders') and hasattr(ui, 'power_sliders'):
            all_sliders = list(ui["position_sliders"].values()) + list(ui["power_sliders"].values())
            for slider in all_sliders:
                if not enabled:
                    slider.setEnabled(False)
                # When enabled, slider activity depends on mode

        # Arrow buttons - FORCE GRAY
        if hasattr(ui, 'arrow_buttons'):
            for btn in ui["arrow_buttons"]:
                if not enabled:
                    btn.setEnabled(False)
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    # When enabled, arrow button activity depends on mode
                    btn.setStyleSheet("")

        # Update joint controls state - AFTER RESTORE
        if enabled:
            # When system is active, recalculate joint controls state
            any_joint_active = any(btn.objectName() == "joint-button-active" for btn in ui["joint_buttons"])
            self.main_window.joint_controls_active = any_joint_active
            
            # If joint is active, disable other controls
            if any_joint_active:
                self._set_calibration_buttons_enabled(False)
                self._set_mode_button_enabled(False)
            else:
                self._set_calibration_buttons_enabled(True)
                self._set_mode_button_enabled(True)

        # Status message - only log when state actually changes
        status = "enabled" if enabled else "disabled"
        self.main_window.add_log_message(f"System controls {status}")
        
        # REMOVE APPLY_STYLES CALL - This re-enables buttons
        # self.main_window.apply_styles()

    # --- COMMUNICATION FUNCTIONS ---
    def _create_communication_message(self, **kwargs):
        """Helper to create control message"""
        from gokhas_communication.msg import ControlMessage
        control_msg = ControlMessage()
        
        # Set control message values
        control_msg.comStatus = kwargs.get('comStat', 0)      # 0=inactive, 1=active
        control_msg.calibStatus = kwargs.get('calibStat', 0)  # 0=normal, 1=calibration_mode
        
        return control_msg
    
    def _publish_communication(self, control_msg):
        """Safely publish control message"""
        if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
            try:
                self.main_window.ros_bridge.publish_control_command(control_msg)
            except Exception as e:
                rospy.logerr(f"Control message publish failed: {e}")

    def _create_joint_message(self):
        """Helper to create joint message from current slider values"""
        from gokhas_communication.msg import JointMessage
        joint_msg = JointMessage()
        
        # Get active joints and their values
        active_joints = []
        joint_values = {}
        
        for joint_name in ["Joint1", "Joint2", "Effector"]:
            if self._is_joint_active(joint_name):
                active_joints.append(joint_name)
                pos_val, pow_val = self._get_joint_values(joint_name)
                joint_values[joint_name] = {"position": pos_val, "power": pow_val}
        
        # Map joint values to JointMessage fields
        if "Joint1" in joint_values:
            joint_msg.j1p = abs(joint_values["Joint1"]["position"])  # 0-135
            joint_msg.j1s = joint_values["Joint1"]["power"]          # 0-100
        
        if "Joint2" in joint_values:
            joint_msg.j2p = abs(joint_values["Joint2"]["position"])  # 0-135
            joint_msg.j2s = joint_values["Joint2"]["power"]          # 0-100
        
        if "Effector" in joint_values:
            joint_msg.ap = joint_values["Effector"]["power"]         # 0-100 (Effector has no position)
        
        # Set control bits for position signs
        control_bits = 0
        if "Joint1" in joint_values and joint_values["Joint1"]["position"] < 0:
            control_bits |= 0b00000010  # Set j1p_sign bit (bit 2)
        if "Joint2" in joint_values and joint_values["Joint2"]["position"] < 0:
            control_bits |= 0b00000100  # Set j2p_sign bit (bit 3)
        
        joint_msg.control_bits = control_bits
        return joint_msg
    
    def _publish_joint_command(self):
        """Create and publish joint command from current slider values"""
        if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
            try:
                joint_msg = self._create_joint_message()
                self.main_window.ros_bridge.publish_joint_command(joint_msg)
                rospy.logdebug(f"Published joint command: J1({joint_msg.j1p},{joint_msg.j1s}) J2({joint_msg.j2p},{joint_msg.j2s}) Effector({joint_msg.ap})")
            except Exception as e:
                rospy.logerr(f"Joint message publish failed: {e}")

    def _set_button_state(self, button, object_name, text=None, enabled=True):
        """Safely set button state"""
        if button:
            if text:
                button.setText(text)
            button.setObjectName(object_name)
            button.setEnabled(enabled)
            if self.parent:
                self.parent.apply_styles()

    def _cleanup_timers(self):
        """Cleanup all timers to prevent memory leaks"""
        # Stop activation timeout
        self._stop_activation_timeout()
        
        # Stop all pulse timers
        for timer_id, timer in list(self.pulse_timers.items()):
            try:
                timer.stop()
                timer.deleteLater()
            except:
                pass
        self.pulse_timers.clear()
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self._cleanup_timers()

    def _disconnect_all_signals(self):
        """Disconnect all signals to prevent memory leaks"""
        try:
            if hasattr(self, '_button_cache'):
                for button_list in self._button_cache.values():
                    if isinstance(button_list, list):
                        for btn in button_list:
                            if btn:
                                btn.clicked.disconnect()
                    elif button_list:
                        button_list.clicked.disconnect()
        except:
            pass

    def _enable_arrow_buttons(self, parent_widget, enabled):
        """Enable/disable arrow buttons next to sliders"""
        if not parent_widget:
            return
            
        arrow_buttons = parent_widget.findChildren(QPushButton)
        for btn in arrow_buttons:
            if "arrow" in btn.objectName().lower() or "increment" in btn.objectName().lower() or "decrement" in btn.objectName().lower():
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    btn.setStyleSheet("")

    def handle_image_click(self, click_x, click_y):
        """Handle left click event on image - Move robot to clicked coordinates"""
        # Convert clicked coordinates from scaled display to original image coordinates
        original_x, original_y = self._convert_display_to_original_coords(click_x, click_y)
        
        rospy.loginfo(f"Left click at display ({click_x:.1f}, {click_y:.1f}) → original ({original_x}, {original_y}) - Moving robot")
        self.main_window.add_log_message(f"Moving to: ({original_x}, {original_y})")
        
        # Call 2D to 3D service to get joint angles for clicked coordinates
        try:
            # Import service here to avoid circular imports
            from gokhas_perception.srv import PixelTo3D, PixelTo3DRequest
            
            # Wait for service if not available
            rospy.wait_for_service('pixel_to_3d_angles', timeout=3.0)
            service_client = rospy.ServiceProxy('pixel_to_3d_angles', PixelTo3D)
            
            # Create service request with original image coordinates
            request = PixelTo3DRequest()
            request.autonomous = False  # Manual mode
            request.pixel_x = int(original_x)
            request.pixel_y = int(original_y)
            
            # Call service
            response = service_client(request)
            
            # Check if valid response received
            if response.joint1degree == 0 and response.joint2degree == 0:
                rospy.logwarn(f"No valid 3D point found at original coords ({original_x}, {original_y})")
                self.main_window.add_log_message(f"Target unreachable: No 3D data at ({original_x}, {original_y})")
                return
            
            # Create and send joint command
            self._send_manual_targeting_command(response.joint1degree, response.joint2degree)
            
            rospy.loginfo(f"Manual targeting: J1={response.joint1degree}° J2={response.joint2degree}°")
            self.main_window.add_log_message(f"Robot moving: J1={response.joint1degree}° J2={response.joint2degree}°")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"2D to 3D service call failed: {e}")
            self.main_window.add_log_message(f"Service error: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS service timeout: {e}")
            self.main_window.add_log_message("Service timeout: 2D to 3D service not available")
        except Exception as e:
            rospy.logerr(f"Unexpected error in image click: {e}")
            self.main_window.add_log_message(f"Error: {e}")

    def handle_image_trigger(self, target_x, target_y):
        """Handle right click trigger event on image - Fire airsoft"""
        # Convert coordinates to original image coordinates
        original_x, original_y = self._convert_display_to_original_coords(target_x, target_y)
        
        rospy.loginfo(f"Right click trigger at display ({target_x:.1f}, {target_y:.1f}) → original ({original_x}, {original_y}) - Firing airsoft")
        self.main_window.add_log_message(f"FIRING at: ({original_x}, {original_y})")
        
        # Send airsoft trigger command
        self._send_airsoft_trigger_command()

    def _send_manual_targeting_command(self, joint1_degree, joint2_degree):
        """Send joint command for manual targeting from image click"""
        try:
            from gokhas_communication.msg import JointMessage
            
            joint_msg = JointMessage()
            joint_msg.control_bits = 0  # Initialize control bits
            
            # Joint 1 (Horizontal) - Handle sign and value
            if joint1_degree < 0:
                joint_msg.control_bits |= 0b00000010  # Set j1p_sign bit (bit 1) for negative
                joint_msg.j1p = min(abs(joint1_degree), 135)
            else:
                joint_msg.j1p = min(joint1_degree, 135)
            
            # Joint 2 (Vertical) - Handle sign and value  
            if joint2_degree < 0:
                joint_msg.control_bits |= 0b00000100  # Set j2p_sign bit (bit 2) for negative
                joint_msg.j2p = min(abs(joint2_degree), 135)
            else:
                joint_msg.j2p = min(joint2_degree, 135)
            
            # Set manual mode speeds
            joint_msg.j1s = 70  # Faster speed for manual targeting
            joint_msg.j2s = 70  # Faster speed for manual targeting
            joint_msg.ap = 0    # No airsoft in movement command
            
            # Publish joint command
            if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
                self.main_window.ros_bridge.publish_joint_command(joint_msg)
                rospy.loginfo(f"Manual targeting command sent: j1p={joint_msg.j1p} j2p={joint_msg.j2p} bits={joint_msg.control_bits:08b}")
            else:
                rospy.logwarn("ROS bridge not available for joint command")
                
        except Exception as e:
            rospy.logerr(f"Error sending manual targeting command: {e}")

    def _send_airsoft_trigger_command(self):
        """Send airsoft trigger command (right click)"""
        try:
            from gokhas_communication.msg import JointMessage
            
            joint_msg = JointMessage()
            joint_msg.control_bits = 0b00000001  # Set airsoft trigger bit (bit 0)
            
            # Keep current joint positions (don't move)
            joint_msg.j1p = 0
            joint_msg.j2p = 0
            joint_msg.j1s = 0  # No movement speed
            joint_msg.j2s = 0  # No movement speed
            joint_msg.ap = 100  # Full airsoft power
            
            # Publish airsoft command
            if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
                self.main_window.ros_bridge.publish_joint_command(joint_msg)
                rospy.loginfo(f"Airsoft trigger fired: ap={joint_msg.ap} bits={joint_msg.control_bits:08b}")
            else:
                rospy.logwarn("ROS bridge not available for airsoft command")
                
        except Exception as e:
            rospy.logerr(f"Error sending airsoft trigger command: {e}")

    def _cache_ui_elements(self):
        """Cache UI elements for performance"""
        if not self.main_window:
            return
        
        try:
            # Cache joint buttons
            joint_buttons = []
            calibration_buttons = []
            
            all_buttons = self.main_window.findChildren(QPushButton)
            for btn in all_buttons:
                if btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                    joint_buttons.append(btn)
                elif btn.objectName() in ["calibration-default", "calibration-completed", "calibration-disabled", "calibration-processing"]:
                    calibration_buttons.append(btn)
                elif "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    self._button_cache['mode_button'] = btn
                elif "ACTIVATED" in btn.text() or "DEACTIVATED" in btn.text():
                    self._button_cache['activation_button'] = btn
            
            self._button_cache['joint_buttons'] = joint_buttons
            self._button_cache['calibration_buttons'] = calibration_buttons
            
            # Cache sliders
            if hasattr(self.main_window, 'ui_elements'):
                ui = self.main_window.ui_elements
                self._slider_cache['position_sliders'] = ui.get("position_sliders", {})
                self._slider_cache['power_sliders'] = ui.get("power_sliders", {})
            
            rospy.loginfo("UI elements cached successfully")
            
        except Exception as e:
            rospy.logwarn(f"UI cache initialization failed: {e}")

    def _start_calibration_timeout(self):
        """Start calibration timeout timer to handle STM32 non-response"""
        self._stop_calibration_timeout()
        
        # Start timeout timer for real hardware mode
        self.calibration_timeout_timer = QTimer()
        self.calibration_timeout_timer.setSingleShot(True)
        self.calibration_timeout_timer.timeout.connect(self._calibration_timeout_callback)
        self.calibration_timeout_timer.start(ControlHandlersConfig.CALIBRATION_TIMEOUT)
        rospy.loginfo(f"Calibration timeout started ({ControlHandlersConfig.CALIBRATION_TIMEOUT/1000} seconds)")

    def _stop_calibration_timeout(self):
        """Stop calibration timeout timer"""
        if hasattr(self, 'calibration_timeout_timer') and self.calibration_timeout_timer:
            self.calibration_timeout_timer.stop()
            self.calibration_timeout_timer = None

    def _calibration_timeout_callback(self):
        """Called when calibration timeout expires - STM32 didn't respond"""
        rospy.logwarn("Calibration timeout - STM32 did not respond to calibration request")
        
        if self.calibration_in_progress and self.current_calibrating_button and self.current_calibrating_all_buttons:
            # Log the timeout
            self.main_window.add_log_message(f"Calibration timeout: {self.current_calibrating_button.text()} - STM32 did not respond")
            
            # Reset the calibration state - mark as failed
            self._reset_calibration_on_timeout()
        
        self.calibration_timeout_timer = None

    def _reset_calibration_on_timeout(self):
        """Reset calibration state when timeout occurs"""
        if not self.calibration_in_progress:
            return
            
        rospy.loginfo("Resetting calibration state due to timeout")
        
        # Stop all timers
        for timer_id, timer in list(self.pulse_timers.items()):
            try:
                timer.stop()
                timer.deleteLater()
            except Exception as e:
                rospy.logwarn(f"Timer cleanup error: {e}")
        self.pulse_timers.clear()
        
        # Reset button states - set to default (red) instead of completed (green)
        if self.current_calibrating_button and self.current_calibrating_all_buttons:
            # Set the failed button back to default state
            self.current_calibrating_button.setObjectName("calibration-default")
            self.current_calibrating_button.setStyleSheet("")
            self.current_calibrating_button.setEnabled(True)
            
            # Re-enable other calibration buttons
            for btn in self.current_calibrating_all_buttons:
                if btn != self.current_calibrating_button:
                    btn.setObjectName("calibration-default")
                    btn.setStyleSheet("")
                    btn.setEnabled(True)
        
        # Re-enable joint buttons
        self._set_joint_buttons_enabled(True)
        
        # Re-enable mode button
        self._set_mode_button_enabled(True)
        
        # Reset calibration state variables
        self.calibration_in_progress = False
        self.current_calibrating_button = None
        self.current_calibrating_all_buttons = None
        self.main_window.calibration_active = False
        
        # Apply styles
        if self.parent:
            self.parent.apply_styles()
        
        rospy.loginfo("Calibration state reset due to timeout - ready for new calibration attempt")

    def _stop_pulse_and_set_waiting(self, btn):
        """Stop pulse animation and set button to steady yellow (waiting for completion)"""
        btn_id = id(btn)
        
        # Stop pulse animation
        if btn_id in self.pulse_timers:
            try:
                self.pulse_timers[btn_id].stop()
                del self.pulse_timers[btn_id]
            except Exception as e:
                rospy.logwarn(f"Pulse timer stop error: {e}")
        
        # Set button to steady yellow (waiting state)
        if hasattr(btn, 'setStyleSheet'):
            btn.setObjectName("calibration-waiting")
            btn.setStyleSheet("""
                QPushButton#calibration-waiting {
                    background-color: #ffc107 !important;
                    color: #212529 !important;
                    border: 2px solid #e0a800;
                    border-radius: 8px;
                    padding: 8px 16px;
                    font-weight: bold;
                    font-size: 12px;
                }
            """)
            
        rospy.loginfo("Calibration button set to waiting state (steady yellow)")
    
    def _convert_display_to_original_coords(self, display_x, display_y):
        """Convert display coordinates to original image coordinates (1920x1080)"""
        try:
            # Get the image label widget
            image_label = self.main_window.image_label
            
            # Get current pixmap and label sizes
            pixmap = image_label.pixmap()
            if not pixmap:
                rospy.logwarn("No image available for coordinate conversion")
                return display_x, display_y
            
            label_size = image_label.size()
            pixmap_size = pixmap.size()
            
            # Calculate the actual displayed image rectangle within the label
            # QLabel centers the scaled image, so we need to find the offset
            label_width = label_size.width()
            label_height = label_size.height()
            pixmap_width = pixmap_size.width()
            pixmap_height = pixmap_size.height()
            
            # Calculate the offset of the image within the label (centering)
            x_offset = max(0, (label_width - pixmap_width) / 2)
            y_offset = max(0, (label_height - pixmap_height) / 2)
            
            # Adjust click coordinates to be relative to the actual image
            image_x = display_x - x_offset
            image_y = display_y - y_offset
            
            # Clamp to image bounds
            image_x = max(0, min(image_x, pixmap_width))
            image_y = max(0, min(image_y, pixmap_height))
            
            # Calculate scale factors from displayed size to original (1920x1080)
            original_width = 1920
            original_height = 1080
            
            scale_x = original_width / pixmap_width if pixmap_width > 0 else 1.0
            scale_y = original_height / pixmap_height if pixmap_height > 0 else 1.0
            
            # Convert to original coordinates
            original_x = int(image_x * scale_x)
            original_y = int(image_y * scale_y)
            
            # Clamp to original image bounds
            original_x = max(0, min(original_x, original_width - 1))
            original_y = max(0, min(original_y, original_height - 1))
            
            rospy.logdebug(f"Coord conversion: display({display_x:.1f},{display_y:.1f}) → image({image_x:.1f},{image_y:.1f}) → original({original_x},{original_y})")
            rospy.logdebug(f"Label size: {label_width}x{label_height}, Pixmap size: {pixmap_width}x{pixmap_height}, Scale: {scale_x:.2f}x{scale_y:.2f}")
            
            return original_x, original_y
            
        except Exception as e:
            rospy.logerr(f"Error in coordinate conversion: {e}")
            # Fallback to original coordinates
            return int(display_x), int(display_y)