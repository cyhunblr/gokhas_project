#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from PyQt6.QtGui import QIcon, QPixmap

class ResourceManager:
    """
    Resource management class for the robotics control interface.
    
    This class handles loading and caching of:
    - Icons (button icons, status indicators)
    - Images (logos, backgrounds)
    - Stylesheets (themes, component styles)
    
    Provides centralized access to all UI resources and manages
    theme switching for the entire application.
    """
    
    def __init__(self):
        """
        Initialize resource manager with directory paths and default settings.
        
        Sets up resource directories relative to this file's location
        and initializes the styling system with light theme as default.
        """
        # Get the directory where this resource.py file is located
        self.resources_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Set up subdirectories for different resource types
        self.icons_dir = os.path.join(self.resources_dir, "icons")      # Button icons, indicators
        self.images_dir = os.path.join(self.resources_dir, "images")    # Logos, backgrounds
        self.styles_dir = os.path.join(self.resources_dir, "styles")    # QSS stylesheet files
        
        # Set default theme - light theme provides better visibility for robotics control
        self.current_theme = "light_theme"
        
        # Cache for loaded stylesheets to improve performance
        # Avoids re-reading files from disk on every style application
        self._style_cache = {}
    
    def get_icon(self, icon_name):
        """
        Load and return an icon file as QIcon object.
        
        Args:
            icon_name: Filename of the icon (e.g., "closeApp.png")
            
        Returns:
            QIcon: Icon object for use in buttons, or empty QIcon if file not found
        """
        icon_path = os.path.join(self.icons_dir, icon_name)
        if os.path.exists(icon_path):
            return QIcon(icon_path)
        return QIcon()  # Return empty icon if file doesn't exist
    
    def get_image(self, image_name):
        """
        Load and return an image file as QPixmap object.
        
        Args:
            image_name: Filename of the image (e.g., "logo.png")
            
        Returns:
            QPixmap: Image object for display in labels, or empty QPixmap if file not found
        """
        image_path = os.path.join(self.images_dir, image_name)
        if os.path.exists(image_path):
            return QPixmap(image_path)
        return QPixmap()  # Return empty pixmap if file doesn't exist
    
    def get_style(self, style_name):
        """
        Load and return a QSS stylesheet file as string.
        
        Uses caching to avoid repeatedly reading files from disk.
        This improves performance when styles are applied frequently.
        
        Args:
            style_name: Name of the style file without .qss extension
            
        Returns:
            str: CSS-like stylesheet content, or empty string if file not found
        """
        # Check if style is already cached to avoid disk I/O
        if style_name in self._style_cache:
            return self._style_cache[style_name]
            
        # Construct full path to stylesheet file
        style_path = os.path.join(self.styles_dir, f"{style_name}.qss")
        if os.path.exists(style_path):
            with open(style_path, 'r', encoding='utf-8') as f:
                content = f.read()
                # Cache the content for future use
                self._style_cache[style_name] = content
                return content
        return ""  # Return empty string if file doesn't exist
    
    def get_current_theme(self):
        """
        Get the currently active theme stylesheet.
        
        Returns:
            str: Complete theme stylesheet content
        """
        return self.get_style(self.current_theme)
    
    def set_theme(self, theme_name):
        """
        Change the active theme for the entire application.
        
        Args:
            theme_name: Name of the theme (e.g., "light_theme", "dark_theme")
            
        Returns:
            str: New theme stylesheet content
        """
        self.current_theme = theme_name
        return self.get_style(theme_name)
    
    def get_available_themes(self):
        """
        List all available theme files in the styles directory.
        
        Scans the styles directory for files ending with '_theme.qss'
        and returns their names without the file extension.
        
        Returns:
            list: Available theme names (e.g., ["light_theme", "dark_theme"])
        """
        themes = []
        if os.path.exists(self.styles_dir):
            for file in os.listdir(self.styles_dir):
                # Look for theme files with specific naming pattern
                if file.endswith('_theme.qss'):  # Example: light_theme.qss, dark_theme.qss
                    theme_name = file[:-4]  # Remove .qss extension
                    themes.append(theme_name)
        return themes
    
    # === COMPONENT-SPECIFIC STYLE METHODS ===
    # These methods provide easy access to styles for specific UI components
    
    def get_joint_button_style(self):
        """
        Get stylesheet for joint control buttons.
        
        Joint buttons control individual robot joints and have different
        states (active/inactive) with corresponding colors.
        
        Returns:
            str: Joint button stylesheet content
        """
        return self.get_style("joint_controls")
    
    def get_calibration_style(self, state="default"):
        """
        Get stylesheet for calibration buttons based on their current state.
        
        Calibration buttons go through multiple states during the calibration process:
        - default: Ready for calibration
        - processing: Currently calibrating (animated)
        - completed: Calibration finished successfully
        
        Args:
            state: Button state ("default", "processing", "completed")
            
        Returns:
            str: Appropriate stylesheet for the requested state
        """
        styles = self.get_style("calibration_buttons")
        if state == "default":
            return ".calibration-default"
        elif state == "processing":
            return ".calibration-processing"
        elif state == "completed":
            return ".calibration-completed"
        return styles
    
    def get_toggle_style(self, active=False):
        """
        Get stylesheet for toggle buttons (MANUAL/AUTONOMOUS, ACTIVATED/DEACTIVATED).
        
        Toggle buttons have two visual states:
        - Active (green): Feature is enabled/on
        - Inactive (red): Feature is disabled/off
        
        Args:
            active: Whether button is in active state
            
        Returns:
            str: Appropriate toggle button style
        """
        styles = self.get_style("toggle_buttons")
        if active:
            return ".toggle-active"
        else:
            return ".toggle-inactive"
    
    def get_close_button_style(self):
        """
        Get stylesheet for the application close button.
        
        Close button typically has special styling to indicate
        it will terminate the entire robotics control application.
        
        Returns:
            str: Close button stylesheet content
        """
        return self.get_style("close_button")
    
    def get_theme_button_style(self, active=False):
        """
        Get stylesheet for theme selection buttons.
        
        Theme buttons allow switching between light and dark modes.
        Active button shows which theme is currently selected.
        
        Args:
            active: Whether this theme button represents the active theme
            
        Returns:
            str: Appropriate theme button style
        """
        styles = self.get_style("theme_buttons")
        if active:
            return ".theme-button-active"
        else:
            return ".theme-button-normal"
    
    def get_label_style(self, type="header"):
        """
        Get stylesheet for different types of labels.
        
        Labels serve different purposes and need different styling:
        - header: Section titles, panel headers
        - theme: Theme selection area labels
        
        Args:
            type: Label type ("header", "theme")
            
        Returns:
            str: Appropriate label stylesheet
        """
        styles = self.get_style("labels")
        if type == "header":
            return ".header-label"
        elif type == "theme":
            return ".theme-label"
        return styles
    
    def get_helper_button_style(self):
        """
        Get stylesheet for the help button displayed over the camera image.
        
        Help button provides user assistance and needs special styling
        to be visible over the camera feed without interfering with image clicks.
        
        Returns:
            str: Helper button stylesheet content
        """
        return self.get_style("helper_button")

# Global resource manager instance
# This singleton provides centralized access to resources throughout the application
resource_manager = ResourceManager()