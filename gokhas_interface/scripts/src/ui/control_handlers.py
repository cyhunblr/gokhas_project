#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from PyQt6.QtWidgets import (
    QWidget, QLabel, QFrame, QPushButton, QSlider, 
    QGridLayout, QHBoxLayout, QVBoxLayout, QTextEdit, 
    QApplication, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QMouseEvent

class ControlHandlersConfig:
    """Configuration constants for control handlers"""
    ACTIVATION_TIMEOUT = 5000  # ms - timeout for system activation
    PULSE_INTERVAL = 250       # ms - pulse animation interval
    CALIBRATION_DURATION = 5000 # ms - calibration process duration
    STM32_SIMULATION_MODE = True  # Simulation flag - control directly here
    SIMULATION_RESPONSE_DELAY = 1000  # ms - simulated response delay
    
    # Object names for UI styling
    JOINT_ACTIVE = "joint-button-active"
    JOINT_INACTIVE = "joint-button-inactive"
    TOGGLE_WAITING = "toggle-waiting"
    TOGGLE_ACTIVE = "toggle-active"
    TOGGLE_INACTIVE = "toggle-inactive"

class ControlHandlers:
    """Class that manages interface button functions and system control logic"""
    
    def __init__(self, main_window):
        """Initialize control handlers with main window reference"""
        self.parent = main_window
        self.main_window = main_window
        self.pulse_timers = {}  # Dictionary to store animation timers
        self.calibration_in_progress = False  # Flag to track calibration state
        self.current_calibrating_button = None  # Reference to currently calibrating button
        self.activation_timeout_timer = None  # Timer for activation timeout
        self.simulation_timer = None  # Timer for STM32 simulation
        
        # Joint state tracking - save joint states when system is deactivated/activated
        self._saved_joint_states = {}
        
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

        rospy.loginfo(f"STM32 Simulation Mode: {'ENABLED' if ControlHandlersConfig.STM32_SIMULATION_MODE else 'DISABLED'}")

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
        
        # Disable other controls when joint is active
        self._set_calibration_buttons_enabled(False)
        self._set_mode_button_enabled(False)
    
    def _deactivate_joint(self, button, joint_name):
        """Helper function to deactivate a joint"""
        button.setObjectName("joint-button-inactive")
        self.main_window.add_log_message(f"Joint deactivated: {joint_name}")
        self._set_joint_sliders_enabled(joint_name, False)

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
        """Called when position slider value changes"""
        print(f"Position changed for {joint_name}: {value}")
        # Log only for active joints - to log together with power value
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)

    def power_slider_changed(self, joint_name, value):
        """Called when power slider value changes"""
        print(f"Power changed for {joint_name}: {value}")
        # Log only for active joints - to log together with position value
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)

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
        
        # TODO: Send calibration command to ROS service
        # Example:
        # try:
        #     calibration_service = rospy.ServiceProxy('calibrate_joint', CalibrateJoint)
        #     response = calibration_service(joint_name=calibration_name)
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Calibration service call failed: {e}")

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
            
            # Complete after 5 seconds - STORE THE TIMER!
            completion_timer = QTimer()
            completion_timer.setSingleShot(True)
            completion_timer.timeout.connect(lambda: self.finish_calibration(btn, all_buttons))
            completion_timer.start(ControlHandlersConfig.CALIBRATION_DURATION)
            
            # Add timer to pulse_timers so it can be stopped
            btn_id = f"completion_{id(btn)}"
            self.pulse_timers[btn_id] = completion_timer

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
            # Create communication message for activation
            comm_msg = self._create_communication_message(comStat=True)
            
            # Set button to waiting state
            if activation_button:
                self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_WAITING, "ACTIVATED")
                self.main_window.add_log_message("Waiting for STM32 communication confirmation...")
                self._start_activation_timeout()
            else:
                rospy.logerr("Activation button not found!")
                
        else:  # DEACTIVATED
            # Create communication message for deactivation
            comm_msg = self._create_communication_message(comStat=False)
            
            # Stop timeout and set button to inactive
            self._stop_activation_timeout()
            if activation_button:
                self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_INACTIVE, "DEACTIVATED")
            
            # 1. FIRST CHANGE SYSTEM STATE
            self.main_window.system_activated = False
            
            # 2. RESET CALIBRATION AND MODE STATE (without affecting controls)
            self._reset_calibration_state()
            self._reset_mode_to_manual()
            
            # 3. FINALLY DISABLE CONTROLS
            self._set_system_controls_enabled(False)
    
        # Publish message
        self._publish_communication(comm_msg)

    def _reset_mode_to_manual(self):
        """Reset mode button to MANUAL when system is deactivated"""
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
            
            # Log message
            self.main_window.add_log_message("Mode reset to MANUAL")
            rospy.loginfo("Mode reset to MANUAL on system deactivation")
            
            # RECONNECT SIGNALS - BUT MANUAL CONNECTION!
            # mode_button.clicked.connect(...) - REMOVE THIS LINE
        else:
            rospy.logwarn("Mode button not found for reset!")

    def _reset_calibration_state(self):
        """Reset calibration state when system is deactivated"""
        if self.calibration_in_progress:
            rospy.loginfo("System deactivated - resetting calibration state")
            
            # STOP ALL TIMERS (pulse + completion timers)
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
        if ControlHandlersConfig.STM32_SIMULATION_MODE:
            self._start_simulation_response()
            rospy.loginfo(f"STM32 Simulation Mode: Auto-response in {ControlHandlersConfig.SIMULATION_RESPONSE_DELAY/1000} seconds")
        else:
            # Normal timeout timer
            self.activation_timeout_timer = QTimer()
            self.activation_timeout_timer.setSingleShot(True)
            self.activation_timeout_timer.timeout.connect(self._activation_timeout_callback)
            self.activation_timeout_timer.start(ControlHandlersConfig.ACTIVATION_TIMEOUT)
            rospy.loginfo(f"Activation timeout started ({ControlHandlersConfig.ACTIVATION_TIMEOUT/1000} seconds)")

    def _start_simulation_response(self):
        """Simulate STM32 response in simulation mode"""
        self.simulation_timer = QTimer()
        self.simulation_timer.setSingleShot(True)
        self.simulation_timer.timeout.connect(self._simulate_stm32_response)
        self.simulation_timer.start(ControlHandlersConfig.SIMULATION_RESPONSE_DELAY)

    def _simulate_stm32_response(self):
        """Simulate successful response from STM32"""
        rospy.loginfo("SIMULATION: STM32 communication successful")
        
        # Create mock communication message
        class MockCommMsg:
            def __init__(self):
                self.comStat = True
        
        # Call handle_communication
        self.main_window.handle_communication(MockCommMsg())
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

    def toggle_simulation_mode(self):
        """Toggle simulation mode on/off"""
        ControlHandlersConfig.STM32_SIMULATION_MODE = not ControlHandlersConfig.STM32_SIMULATION_MODE
        status = "ENABLED" if ControlHandlersConfig.STM32_SIMULATION_MODE else "DISABLED"
        rospy.loginfo(f"STM32 Simulation Mode: {status}")
        self.main_window.add_log_message(f"STM32 Simulation Mode: {status}")

    def _activation_timeout_callback(self):
        """Activation timeout callback - called when STM32 doesn't respond"""
        rospy.logwarn("STM32 communication timeout - activation failed")
        
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
            self._set_button_state(activation_button, ControlHandlersConfig.TOGGLE_INACTIVE, "DEACTIVATED")
            self.main_window.add_log_message("STM32 communication timeout - activation failed")
            
            # Disable system controls
            self.main_window.system_activated = False
            self._set_system_controls_enabled(False)
    
        self.activation_timeout_timer = None

    def mode_button_clicked(self, button_text):
        """Called when mode button (MANUAL/AUTONOMOUS) is clicked"""
        rospy.loginfo(f"Mode button clicked: {button_text}")
        
        # Update manual mode state
        if button_text == "MANUAL":
            self.main_window.manual_mode_active = True
            rospy.loginfo("Manual mode activated - image clicks enabled")
            
            # Enable controls in MANUAL mode
            self._set_manual_controls_enabled(True)
            
        elif button_text == "AUTONOMOUS":
            self.main_window.manual_mode_active = False
            rospy.loginfo("Autonomous mode activated - image clicks disabled")
            
            # Disable controls in AUTONOMOUS mode
            self._set_manual_controls_enabled(False)
        
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
        print(f"Theme changed: {theme_name}")
        rospy.loginfo(f"Theme changed: {theme_name}")
        
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

        # Status message
        status = "enabled" if enabled else "disabled"
        self.main_window.add_log_message(f"System controls {status}")
        
        # REMOVE APPLY_STYLES CALL - This re-enables buttons
        # self.main_window.apply_styles()

    # --- COMMUNICATION FUNCTIONS ---
    def _create_communication_message(self, **kwargs):
        """Helper to create communication message"""
        from gokhas_communication.msg import communication
        comm_msg = communication()
        
        # Default values
        defaults = {
            'comStat': False, 'modStat': False,
            'rJ1p': 0, 'rJ1s': 0, 'rJ2p': 0, 'rJ2s': 0, 'rAp': 0, 'rAt': False,
            'tJ1p': 0, 'tJ1s': 0, 'tJ2p': 0, 'tJ2s': 0, 'tAp': 0, 'tAt': False,
            'cJ1': False, 'cJ2': False
        }
        
        # Update with provided values
        for key, value in defaults.items():
            setattr(comm_msg, key, kwargs.get(key, value))
        
        return comm_msg
    
    def _publish_communication(self, comm_msg):
        """Safely publish communication message"""
        if hasattr(self.main_window, 'ros_bridge') and self.main_window.ros_bridge:
            try:
                self.main_window.ros_bridge.publish_communication(comm_msg)
            except Exception as e:
                rospy.logerr(f"Communication publish failed: {e}")

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
        """Handle left click event on image"""
        rospy.loginfo(f"Image clicked at coordinates: ({click_x:.1f}, {click_y:.1f})")
        self.main_window.add_log_message(f"Target selected: ({click_x:.1f}, {click_y:.1f})")

    def handle_image_trigger(self, target_x, target_y):
        """Handle right click trigger event on image"""
        rospy.loginfo(f"Image trigger at coordinates: ({target_x:.1f}, {target_y:.1f})")
        self.main_window.add_log_message(f"TRIGGERED at: ({target_x:.1f}, {target_y:.1f})")
        
        # Here you can publish trigger command to ROS topic
        # Example: self.main_window.ros_bridge.publish_trigger_command(target_x, target_y)

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