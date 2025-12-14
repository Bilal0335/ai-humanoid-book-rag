---
title: ROS 2 Installation & Workspace Setup for Humanoid Robotics
sidebar_label: Installation & Workspace
description: Comprehensive guide to installing ROS 2 Humble Hawksbill and setting up workspaces for humanoid robotics development
---

# ROS 2 Installation & Workspace Setup for Humanoid Robotics

## Learning Objectives

- Install ROS 2 Humble Hawksbill on Ubuntu 22.04 with proper configuration for humanoid robotics
- Set up ROS 2 workspaces following best practices for complex robotic systems
- Configure environment variables and system settings for optimal performance
- Create and manage packages for humanoid robot components
- Validate installation with basic ROS 2 functionality tests

## Introduction

Setting up a proper ROS 2 development environment is the first critical step in building humanoid robotics applications. Unlike simpler robotic systems, humanoid robots require sophisticated software stacks that handle multiple sensors, actuators, and complex control algorithms. This section provides a comprehensive guide to installing ROS 2 Humble Hawksbill and configuring your development environment for humanoid robotics applications.

## System Requirements for Humanoid Robotics

Before installing ROS 2, ensure your system meets the requirements for humanoid robotics development:

### Minimum Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or WSL2 on Windows
- **RAM**: 16GB (32GB recommended for simulation environments)
- **Storage**: 50GB free space for ROS 2 and humanoid development
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen equivalent)
- **Network**: Reliable internet connection for package installation and updates

### Recommended Requirements
- **GPU**: NVIDIA GPU with 8GB+ VRAM (for Isaac Sim, Unity, and perception processing)
- **RAM**: 32GB+ (for running simulations and multiple nodes simultaneously)
- **Processor**: Intel i9 or AMD Threadripper for complex computations

## Installing ROS 2 Humble Hawksbill

### 1. Setting Up Your Sources

First, add the ROS 2 repository to your system:

```bash
# Add the ROS 2 repository key
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Installing ROS 2 Packages

Install the core ROS 2 packages needed for humanoid robotics:

```bash
# Update package lists
sudo apt update

# Install ROS 2 desktop package (includes GUI tools)
sudo apt install -y ros-humble-desktop

# Install additional packages for humanoid robotics
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-ros-core \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-action-msgs \
  ros-humble-control-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-urdf \
  ros-humble-urdf-tutorial \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rosbridge-suite \
  ros-humble-vision-opencv \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy
```

### 3. Installing Development Tools

Install additional tools needed for development:

```bash
# Install development tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  git \
  wget \
  python3-pip \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool
```

### 4. Initialize rosdep

Initialize rosdep to manage dependencies:

```bash
# Initialize rosdep
sudo rosdep init
rosdep update
```

## Environment Setup

### 1. Setting Up Environment Variables

Add ROS 2 environment setup to your bash profile:

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Creating a Humanoid Robotics Workspace

Create a dedicated workspace for humanoid robotics development:

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Create a workspace configuration file
cat > .workspace_config << EOF
# Humanoid Robotics Workspace Configuration
# Created: $(date)
# Purpose: Dedicated workspace for humanoid robotics development

WORKSPACE_NAME=humanoid_ws
ROS_DISTRO=humble
DEVELOPMENT_FOCUS=humanoid_robotics
PACKAGES_PATH=~/humanoid_ws/src
INSTALL_PATH=~/humanoid_ws/install
BUILD_PATH=~/humanoid_ws/build
LOG_PATH=~/humanoid_ws/log
EOF
```

### 3. Installing Additional Humanoid-Specific Packages

Install packages commonly used in humanoid robotics:

```bash
# Navigate to your workspace
cd ~/humanoid_ws/src

# Clone essential humanoid robotics packages
git clone -b humble https://github.com/ros-controls/ros2_control.git
git clone -b humble https://github.com/ros-controls/ros2_controllers.git
git clone -b humble https://github.com/ros-planning/navigation2.git
git clone -b humble https://github.com/ros-perception/vision_opencv.git
git clone -b humble https://github.com/ros-simulation/gazebo_ros_pkgs.git

# Install dependencies using rosdep
cd ~/humanoid_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Workspace Management

### 1. Basic Workspace Structure

A typical humanoid robotics workspace follows this structure:

```
~/humanoid_ws/
├── src/                    # Source code for all packages
│   ├── humanoid_common/    # Common humanoid utilities
│   ├── perception/         # Vision, sensor processing packages
│   ├── control/            # Joint control, balance control packages
│   ├── navigation/         # Path planning, localization packages
│   ├── manipulation/       # Gripper, arm control packages
│   └── simulation/         # Gazebo, Unity integration packages
├── build/                  # Build artifacts (generated by colcon)
├── install/                # Installed packages (generated by colcon)
├── log/                    # Build logs (generated by colcon)
└── .workspace_config       # Workspace configuration
```

### 2. Creating a Humanoid Package

Create a basic package for humanoid-specific functionality:

```bash
# Navigate to workspace source directory
cd ~/humanoid_ws/src

# Create a humanoid utilities package
ros2 pkg create --build-type ament_python humanoid_utils --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Create a humanoid control package
ros2 pkg create --build-type ament_cmake humanoid_control --dependencies rclcpp control_msgs geometry_msgs
```

### 3. Package Structure for Humanoid Robotics

A well-structured humanoid package typically includes:

```bash
humanoid_example/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── launch/                 # Launch files for starting nodes
│   ├── humanoid_bringup.launch.py
│   └── simulation.launch.py
├── config/                 # Configuration files (YAML, parameters)
│   ├── controllers.yaml
│   └── humanoid_params.yaml
├── urdf/                   # URDF models for the humanoid
│   ├── humanoid.urdf.xacro
│   └── meshes/
├── src/                    # Source code (C++ or Python)
│   └── humanoid_node.cpp
├── scripts/                # Standalone scripts
├── test/                   # Test files
├── include/                # Header files (C++)
└── README.md               # Package documentation
```

## Colcon Build System

### 1. Understanding Colcon

Colcon is the build tool used in ROS 2. It provides several advantages for humanoid robotics development:

- **Parallel builds**: Faster compilation of multiple packages
- **Mixed language support**: Handle both C++ and Python packages
- **Workspace isolation**: Build packages in isolation to avoid conflicts
- **Incremental builds**: Only rebuild changed packages

### 2. Building Your Workspace

Build your humanoid workspace:

```bash
# Navigate to workspace root
cd ~/humanoid_ws

# Build all packages
colcon build --symlink-install

# Build specific package only
colcon build --packages-select humanoid_utils

# Build with verbose output (useful for debugging)
colcon build --event-handlers console_direct+

# Source the workspace
source install/setup.bash
```

### 3. Advanced Colcon Options

For humanoid robotics development, these options are particularly useful:

```bash
# Build with multiple cores (faster compilation)
colcon build --parallel-workers 4

# Build and run tests
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test

# Build specific packages and their dependencies
colcon build --packages-up-to humanoid_control

# Build with specific CMake options
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Verification and Testing

### 1. Basic ROS 2 Functionality Test

Verify your ROS 2 installation works correctly:

```bash
# Check ROS 2 version
ros2 --version

# Check available nodes
ros2 node list

# Check available topics
ros2 topic list

# Run a simple talker/listener demo
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### 2. Humanoid-Specific Tests

Run tests relevant to humanoid robotics:

```bash
# Check if essential humanoid topics exist
ros2 topic list | grep -E "joint_state|tf|imu|camera"

# Check URDF functionality
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Test robot state publishing
ros2 run robot_state_publisher robot_state_publisher
```

### 3. Workspace Validation Script

Create a validation script to check your setup:

```bash
# Create validation script
cat > ~/humanoid_ws/validate_setup.sh << 'EOF'
#!/bin/bash

echo "=== ROS 2 Humanoid Robotics Setup Validation ==="

# Check ROS 2 installation
echo "Checking ROS 2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 is installed: $(ros2 --version)"
else
    echo "✗ ROS 2 is not installed"
    exit 1
fi

# Check essential packages
echo "Checking essential packages..."
ESSENTIAL_PKGS=(
    "rclpy"
    "std_msgs"
    "sensor_msgs"
    "geometry_msgs"
    "nav_msgs"
    "action_msgs"
    "urdf"
    "xacro"
)

for pkg in "${ESSENTIAL_PKGS[@]}"; do
    if python3 -c "import $pkg" &> /dev/null; then
        echo "✓ $pkg is available"
    else
        echo "✗ $pkg is not available"
    fi
done

# Check workspace build
echo "Checking workspace build..."
if [ -d "~/humanoid_ws/install" ]; then
    echo "✓ Workspace is built"
else
    echo "✗ Workspace needs to be built"
fi

# Check colcon availability
if command -v colcon &> /dev/null; then
    echo "✓ Colcon is installed"
else
    echo "✗ Colcon is not installed"
fi

echo "=== Validation Complete ==="
EOF

chmod +x ~/humanoid_ws/validate_setup.sh
~/humanoid_ws/validate_setup.sh
```

## Advanced Configuration for Humanoid Robotics

### 1. Real-time Configuration

For real-time humanoid control, configure your system appropriately:

```bash
# Add current user to real-time groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Configure real-time settings (requires reboot)
cat > /tmp/99-ros-realtime.conf << EOF
# Real-time configuration for ROS 2 humanoid robotics
# Increase maximum locked memory for real-time applications
* soft memlock unlimited
* hard memlock unlimited

# Increase maximum number of processes
* soft nproc 99999
* hard nproc 99999
EOF

sudo cp /tmp/99-ros-realtime.conf /etc/security/limits.d/99-ros-realtime.conf
```

### 2. Network Configuration

Configure network settings for distributed humanoid robotics:

```bash
# Set up ROS 2 domain for isolated network
export ROS_DOMAIN_ID=42  # Set in ~/.bashrc for permanent effect

# Configure ROS 2 for multi-machine communication
export ROS_LOCALHOST_ONLY=0  # Allow communication over network
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Often better for multi-machine setups
```

### 3. Performance Optimization

Optimize for humanoid robotics performance:

```bash
# Create a performance configuration file
cat > ~/humanoid_ws/config/performance.yaml << EOF
/**:
  ros__parameters:
    use_sim_time: false  # Change to true when using simulation

# Performance parameters for humanoid control
humanoid_control_node:
  ros__parameters:
    control_frequency: 100  # Hz for real-time control
    thread_priority: 80     # Priority for control threads
    qos_overrides:
      /joint_commands:
        publisher:
          reliability: reliable
          durability: volatile
          history: keep_last
          depth: 1
EOF
```

## Troubleshooting Common Issues

### 1. Installation Problems

**Problem**: Package installation fails due to missing dependencies
```bash
# Solution: Update package lists and try again
sudo apt update
sudo apt upgrade
rosdep update
```

**Problem**: Permission denied during installation
```bash
# Solution: Ensure you're using the correct package manager
# Use apt for system packages, pip for Python packages in virtual environments
```

### 2. Workspace Build Problems

**Problem**: Build fails with undefined references
```bash
# Solution: Check dependencies in package.xml and CMakeLists.txt
# Ensure all required packages are listed and installed
```

**Problem**: Mixed Python/C++ workspace build issues
```bash
# Solution: Use separate build commands or ensure proper dependency ordering
colcon build --packages-select package_with_cpp
colcon build --packages-select package_with_python
```

### 3. Runtime Issues

**Problem**: Nodes can't communicate across machines
```bash
# Solution: Check network configuration and firewall settings
# Ensure ROS_DOMAIN_ID matches across machines
# Check that RMW_IMPLEMENTATION is consistent
```

**Problem**: High latency in real-time control
```bash
# Solution: Use appropriate QoS settings and consider real-time kernel
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # Potentially lower latency
```

## Best Practices for Humanoid Development

### 1. Workspace Organization
- Use separate workspaces for different projects
- Keep a clean workspace with minimal unrelated packages
- Regularly update and rebuild your workspace

### 2. Package Management
- Use descriptive package names that reflect their function
- Keep packages focused on a single responsibility
- Document package dependencies clearly

### 3. Configuration Management
- Use YAML files for configuration parameters
- Separate simulation and real-robot configurations
- Use launch files to manage complex node startups

## Next Steps

1. Build your humanoid workspace with `colcon build`
2. Source the workspace with `source install/setup.bash`
3. Test basic ROS 2 functionality with the validation script
4. Create your first humanoid-specific package
5. Set up version control for your workspace

## References

- ROS 2 Installation Guide: https://docs.ros.org/en/humble/Installation.html
- ROS 2 Workspaces Tutorial: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
- Colcon Build Tool: https://colcon.readthedocs.io/
- ROS 2 Configuration: https://docs.ros.org/en/humble/How-To-Guides/Setting-up-ROS-2-Environments.html
- Real-time Programming in ROS 2: https://docs.ros.org/en/humble/How-To-Guides/Real-Time-Programming.html