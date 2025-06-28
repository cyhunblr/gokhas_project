#!/bin/bash

# ZED SDK Installation Script for GokHAS Project
# This script downloads and installs ZED SDK from Google Drive
# Google Drive Link: https://drive.google.com/file/d/1hKYi1IH8NF4_Oen5nfC2h6UZZxwa9cqQ/view?usp=sharing

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
        print_error "Please execute without sudo: ./install_zed_sdk.sh"
        print_error "The script will ask for sudo password when needed for installation."
        exit 1
    fi
}

# Function to install gdown if not present
install_gdown() {
    if ! command_exists gdown; then
        print_status "Installing gdown to download from Google Drive..."
        python3 -m pip install gdown
        print_success "gdown installed successfully"
    else
        print_success "gdown is already installed"
    fi
}

# Function to download ZED SDK from Google Drive
download_zed_sdk() {
    print_status "Downloading ZED SDK from Google Drive..."
    
    # Create temporary directory for download
    TEMP_DIR="/tmp/zed_sdk_install"
    mkdir -p "$TEMP_DIR"
    cd "$TEMP_DIR"
    
    # Google Drive file ID from the link
    FILE_ID="1hKYi1IH8NF4_Oen5nfC2h6UZZxwa9cqQ"
    
    # Download using gdown
    print_status "Downloading ZED SDK installer..."
    gdown "https://drive.google.com/uc?id=${FILE_ID}" -O zed_sdk_installer.run
    
    # Check if download was successful
    if [ ! -f "zed_sdk_installer.run" ]; then
        print_error "Failed to download ZED SDK installer!"
        exit 1
    fi
    
    # Make installer executable
    chmod +x zed_sdk_installer.run
    
    print_success "ZED SDK installer downloaded successfully"
}

# Function to check system requirements
check_system_requirements() {
    print_status "Checking system requirements..."
    
    # Check if NVIDIA GPU is present
    if command_exists nvidia-smi; then
        print_status "NVIDIA GPU detected:"
        nvidia-smi --query-gpu=name --format=csv,noheader,nounits || print_warning "Could not query GPU information"
    else
        print_warning "nvidia-smi not found. Make sure NVIDIA drivers are installed."
        print_warning "ZED SDK requires NVIDIA GPU and drivers."
    fi
    
    # Check CUDA installation
    if command_exists nvcc; then
        CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | sed 's/,//')
        print_status "CUDA detected: $CUDA_VERSION"
    else
        print_warning "CUDA not found. ZED SDK may require CUDA for full functionality."
    fi
    
    # Check available disk space (ZED SDK needs ~2-3GB)
    AVAILABLE_SPACE=$(df /usr/local --output=avail | tail -1)
    if [ "$AVAILABLE_SPACE" -lt 3000000 ]; then  # 3GB in KB
        print_warning "Low disk space. ZED SDK requires at least 3GB free space in /usr/local"
    fi
    
    print_success "System requirements check completed"
}

# Function to install ZED SDK
install_zed_sdk() {
    print_status "Starting ZED SDK installation..."
    
    cd "/tmp/zed_sdk_install"
    
    print_warning "The ZED SDK installer will ask several questions:"
    print_warning "- Accept the license agreement (type 'y' or 'yes')"
    print_warning "- Choose installation path (default: /usr/local/zed)"
    print_warning "- Install Python API (recommended: 'y')"
    print_warning "- Install samples (optional: 'y' if you want examples)"
    print_warning "- Install tools (recommended: 'y')"
    echo
    
    print_status "Starting interactive installation..."
    echo "========================================"
    
    # Run the installer
    sudo ./zed_sdk_installer.run
    
    print_success "ZED SDK installation completed!"
}

# Function to verify ZED SDK installation
verify_installation() {
    print_status "Verifying ZED SDK installation..."
    
    # Check if ZED SDK directory exists
    if [ -d "/usr/local/zed" ]; then
        print_success "ZED SDK installed at /usr/local/zed"
        
        # Check ZED tools
        if [ -f "/usr/local/zed/tools/ZED_Explorer" ]; then
            print_success "ZED Explorer tool found"
        fi
        
        if [ -f "/usr/local/zed/tools/ZED_Depth_Viewer" ]; then
            print_success "ZED Depth Viewer tool found"
        fi
        
        # Check Python API
        python3 -c "import pyzed.sl as sl; print('ZED Python API: OK')" 2>/dev/null && print_success "ZED Python API installed" || print_warning "ZED Python API not found"
        
        # Check version
        if [ -f "/usr/local/zed/settings/version.txt" ]; then
            ZED_VERSION=$(cat /usr/local/zed/settings/version.txt)
            print_status "ZED SDK Version: $ZED_VERSION"
        fi
        
    else
        print_error "ZED SDK installation directory not found!"
        return 1
    fi
    
    print_success "ZED SDK verification completed"
}

# Function to setup environment
setup_environment() {
    print_status "Setting up ZED SDK environment..."
    
    # Add ZED to PATH if not already there
    if ! echo "$PATH" | grep -q "/usr/local/zed/bin"; then
        print_status "Adding ZED SDK to PATH..."
        echo 'export PATH="/usr/local/zed/bin:$PATH"' >> ~/.bashrc
        print_status "Added ZED SDK to ~/.bashrc"
    fi
    
    # Add ZED libraries to LD_LIBRARY_PATH
    if ! echo "$LD_LIBRARY_PATH" | grep -q "/usr/local/zed/lib"; then
        print_status "Adding ZED libraries to LD_LIBRARY_PATH..."
        echo 'export LD_LIBRARY_PATH="/usr/local/zed/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
        print_status "Added ZED libraries to ~/.bashrc"
    fi
    
    print_success "Environment setup completed"
    print_warning "Please restart your terminal or run 'source ~/.bashrc' to apply changes"
}

# Function to cleanup temporary files
cleanup() {
    print_status "Cleaning up temporary files..."
    rm -rf "/tmp/zed_sdk_install"
    print_success "Cleanup completed"
}

# Main installation process
main() {
    echo "========================================"
    echo "    ZED SDK Installer for GokHAS       "
    echo "========================================"
    echo
    
    print_status "Starting ZED SDK installation process..."
    echo
    
    # Check if running as root/sudo
    check_not_sudo
    
    # Check system requirements
    check_system_requirements
    echo
    
    # Install gdown for Google Drive download
    install_gdown
    echo
    
    # Download ZED SDK
    download_zed_sdk
    echo
    
    # Install ZED SDK
    install_zed_sdk
    echo
    
    # Verify installation
    verify_installation
    echo
    
    # Setup environment
    setup_environment
    echo
    
    # Cleanup
    cleanup
    echo
    
    print_success "ZED SDK installation complete!"
    echo
    print_status "Next steps:"
    echo "  1. Restart your terminal or run: source ~/.bashrc"
    echo "  2. Connect your ZED camera"
    echo "  3. Test with: /usr/local/zed/tools/ZED_Explorer"
    echo "  4. Build your ROS workspace: cd /home/cyhunblr/bitirme_ws && catkin_make"
    echo
    print_warning "Note: Make sure your NVIDIA drivers and CUDA are properly installed"
    print_warning "ZED SDK requires NVIDIA GPU for optimal performance"
    echo
}

# Trap to cleanup on script exit
trap cleanup EXIT

# Run main function
main "$@"