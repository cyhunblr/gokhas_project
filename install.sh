#!/bin/bash

# GokHAS Project Dependency Installation Script
# This script installs all required dependencies for the GokHAS project packages:
# - gokhas_communication
# - gokhas_interface  
# - gokhas_perception
# - gokhas_zed_ros_wrapper

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if running as root/sudo
check_not_sudo() {
    if [ "$EUID" -eq 0 ]; then
        print_error "This script should NOT be run with sudo!"
        print_error "Please execute without sudo: ./install.sh"
        print_error "The script will ask for sudo password when needed for specific commands."
        exit 1
    fi
}

# Function to check if ROS is installed
check_ros() {
    print_status "Checking ROS installation..."
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash file."
        print_error "Example: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
    print_success "ROS $ROS_DISTRO detected"
}

# Function to fix ROS GPG key if expired
fix_ros_gpg_key() {
    print_status "Checking ROS GPG key..."
    
    # Try to update package list and check for GPG key errors
    if ! sudo apt-get update 2>&1 | grep -q "NO_PUBKEY\|EXPKEYSIG\|expired"; then
        print_success "ROS GPG key is valid"
        return 0
    fi
    
    print_warning "ROS GPG key issue detected. Fixing..."
    print_status "Adding/updating ROS GPG key..."
    
    # Fix the ROS GPG key
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
    
    print_status "Updating package list again..."
    sudo apt-get update
    
    print_success "ROS GPG key fixed"
}

# Function to install system dependencies
install_system_deps() {
    print_status "Installing system dependencies..."
    
    # Fix ROS GPG key if needed
    fix_ros_gpg_key
    
    # Install essential packages
    sudo apt-get install -y \
        python3-pip \
        python3-dev \
        python3-setuptools \
        python3-wheel \
        build-essential \
        cmake \
        git \
        pkg-config
        
    print_success "System dependencies installed"
}

# Function to install and setup rosdep
setup_rosdep() {
    print_status "Setting up rosdep..."
    
    if ! command_exists rosdep; then
        print_status "Installing rosdep..."
        sudo apt-get install -y python3-rosdep
    fi
    
    # Initialize rosdep if not already done
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        print_status "Initializing rosdep..."
        sudo rosdep init
    else
        print_status "rosdep already initialized"
    fi
    
    # Update rosdep
    print_status "Updating rosdep..."
    rosdep update
    
    print_success "rosdep setup complete"
}

# Function to install ROS dependencies
install_ros_deps() {
    print_status "Installing ROS package dependencies..."
    
    # Navigate to the workspace source directory
    WORKSPACE_DIR="/home/cyhunblr/bitirme_ws"
    
    if [ ! -d "$WORKSPACE_DIR" ]; then
        print_error "Workspace directory $WORKSPACE_DIR not found!"
        exit 1
    fi
    
    cd "$WORKSPACE_DIR"
    
    # Install dependencies for all packages
    print_status "Running rosdep install for all packages..."
    rosdep install --from-paths src --ignore-src -r -y
    
    print_success "ROS dependencies installed"
}

# Function to install Python dependencies
install_python_deps() {
    print_status "Installing Python dependencies..."
    
    # Upgrade pip
    python3 -m pip install --upgrade pip
    
    # Install PyQt6 for interface package
    print_status "Installing PyQt6 for gokhas_interface..."
    python3 -m pip install PyQt6 pyserial
    
    # Install OpenCV and computer vision dependencies
    print_status "Installing OpenCV and vision dependencies..."
    python3 -m pip install opencv-python
    
    # Install perception package specific requirements
    if [ -f "/home/cyhunblr/bitirme_ws/src/gokhas_project/gokhas_perception/requirements.txt" ]; then
        print_status "Installing gokhas_perception requirements..."
        python3 -m pip install -r /home/cyhunblr/bitirme_ws/src/gokhas_project/gokhas_perception/requirements.txt
    fi
    
    # Install additional Python packages that might be needed
    print_status "Installing additional Python packages..."
    python3 -m pip install \
        rospkg \
        pyyaml \
        catkin_pkg \
        empy \
        defusedxml \
        netifaces
    
    print_success "Python dependencies installed"
}

# Function to install ZED SDK dependencies (if needed)
install_zed_deps() {
    print_status "Checking for ZED SDK dependencies..."
    
    # Check if ZED SDK is already installed
    if [ -d "/usr/local/zed" ]; then
        print_success "ZED SDK found at /usr/local/zed"
    else
        print_warning "ZED SDK not found. This is required for gokhas_zed_ros_wrapper to work properly."
        echo
        print_status "To install ZED SDK, run the dedicated installer:"
        print_status "cd /home/cyhunblr/bitirme_ws/src/gokhas_project"
        print_status "./install_zed_sdk.sh"
        echo
        print_warning "Alternatively, you can download manually from:"
        print_warning "https://www.stereolabs.com/developers/release/"
    fi
}

# Function to install additional ROS packages that might be missing
install_additional_ros_packages() {
    print_status "Installing additional ROS packages..."
    
    # Common ROS packages that might be needed
    sudo apt-get install -y \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-view \
        ros-$ROS_DISTRO-vision-msgs \
        ros-$ROS_DISTRO-pcl-ros \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-visualization-msgs \
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-std-msgs \
        ros-$ROS_DISTRO-message-generation \
        ros-$ROS_DISTRO-message-runtime \
        ros-$ROS_DISTRO-message-filters \
        ros-$ROS_DISTRO-rviz || true
    
    print_success "Additional ROS packages installed"
}

# Function to verify installation
verify_installation() {
    print_status "Verifying installation..."
    
    # Check Python packages
    python3 -c "import PyQt6; print('PyQt6: OK')" 2>/dev/null || print_warning "PyQt6 not found"
    python3 -c "import cv2; print('OpenCV: OK')" 2>/dev/null || print_warning "OpenCV not found"
    python3 -c "import ultralytics; print('Ultralytics: OK')" 2>/dev/null || print_warning "Ultralytics not found"
    python3 -c "import numpy; print('NumPy: OK')" 2>/dev/null || print_warning "NumPy not found"
    python3 -c "import onnx; print('ONNX: OK')" 2>/dev/null || print_warning "ONNX not found"
    
    # Check if workspace can be built
    cd /home/cyhunblr/bitirme_ws
    if command_exists catkin_make; then
        print_status "Testing workspace build..."
        catkin_make --dry-run > /dev/null 2>&1 && print_success "Workspace build test passed" || print_warning "Workspace build test failed"
    fi
    
    print_success "Installation verification complete"
}

# Main installation process
main() {
    echo "======================================"
    echo "  GokHAS Project Dependency Installer  "
    echo "======================================"
    echo
    
    print_status "Starting installation process..."
    echo
    
    # Check if running as root/sudo
    check_not_sudo
    
    # Check prerequisites
    check_ros
    
    # Install system dependencies
    install_system_deps
    echo
    
    # Setup rosdep
    setup_rosdep
    echo
    
    # Install additional ROS packages
    install_additional_ros_packages
    echo
    
    # Install ROS dependencies
    install_ros_deps
    echo
    
    # Install Python dependencies
    install_python_deps
    echo
    
    # Check ZED dependencies
    install_zed_deps
    echo
    
    # Verify installation
    verify_installation
    echo
    
    print_success "GokHAS project dependency installation complete!"
    echo
    print_status "Next steps:"
    echo "  1. Source your ROS workspace: source /home/cyhunblr/bitirme_ws/devel/setup.bash"
    echo "  2. Build the workspace: cd /home/cyhunblr/bitirme_ws && catkin_make"
    echo "  3. If using ZED camera, install ZED SDK from https://www.stereolabs.com/developers/release/"
    echo
}

# Run main function
main "$@"