# 🤖 Trika - Autonomous Driving Car with ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

A comprehensive ROS2-based autonomous driving car project featuring advanced robotics control, localization, and simulation capabilities. Trika is designed for educational purposes and research in autonomous vehicle development.

## 📋 Table of Contents

- [Features](#-features)
- [Project Structure](#-project-structure)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Usage](#-usage)
- [Simulation](#-simulation)
- [Hardware Requirements](#-hardware-requirements)
- [API Documentation](#-api-documentation)
- [Contributing](#-contributing)
- [License](#-license)

## ✨ Features

### 🤖 Core Robotics
- **Differential Drive Control**: Advanced wheel control with configurable parameters
- **Real-time Kinematics**: TF2-based coordinate transformations
- **Sensor Integration**: Camera, IMU, and wheel encoder support
- **Localization**: SLAM and navigation capabilities

### 🎮 Control Systems
- **Multiple Controller Types**: Simple and advanced velocity controllers
- **Joystick Teleoperation**: Manual control support
- **Parameter Management**: Dynamic parameter configuration
- **Error Modeling**: Realistic wheel radius and separation error simulation

### 🌐 ROS2 Integration
- **Service/Client Architecture**: Custom service definitions
- **Topic-based Communication**: Publisher/Subscriber patterns
- **Launch System**: Modular launch file organization
- **Parameter Server**: Dynamic parameter management

### 🎯 Simulation & Visualization
- **Gazebo Integration**: Full physics simulation
- **RViz Visualization**: Real-time robot state visualization
- **URDF/XACRO Models**: Modular robot description
- **Mesh Support**: 3D model visualization

## 📁 Project Structure

```
trika_ws/
├── src/
│   ├── trika_controller/     # Robot control and motion planning
│   ├── trika_description/    # URDF models and visualization
│   ├── trika_firmware/       # Low-level hardware interface
│   ├── trika_localization/   # SLAM and navigation
│   ├── trika_msgs/          # Custom message and service definitions
│   ├── trika_cpp_examples/  # C++ implementation examples
│   └── trika_py_examples/   # Python implementation examples
├── install/                     # Built packages
├── log/                        # Build and runtime logs
└── README.md                   # This file
```

### Package Details

| Package | Description | Language |
|---------|-------------|----------|
| `trika_controller` | Motion control, wheel controllers, teleoperation | C++/Python |
| `trika_description` | URDF models, Gazebo integration, visualization | XML/XACRO |
| `trika_firmware` | Hardware abstraction layer | C++ |
| `trika_localization` | SLAM, navigation, pose estimation | C++/Python |
| `trika_msgs` | Custom ROS2 messages and services | IDL |
| `trika_cpp_examples` | C++ implementation examples | C++ |
| `trika_py_examples` | Python implementation examples | Python |

## 🔧 Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11
- **ROS2 Version**: Humble Hawksbill (recommended)
- **Python**: 3.8+
- **C++**: C++17 compatible compiler

### Required Dependencies
```bash
# ROS2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# Additional ROS2 packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-nav2-bringup

# Development tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-vcstool
```

## 🚀 Installation

### 1. Clone the Repository
```bash
# Create workspace directory
mkdir -p ~/trika_ws/src
cd ~/trika_ws/src

# Clone the repository
git clone <repository-url> .
```

### 2. Install Dependencies
```bash
# Navigate to workspace root
cd ~/trika_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Project
```bash
# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## ⚡ Quick Start

### 1. Launch Simulation
```bash
# Launch Gazebo simulation with robot
ros2 launch trika_description gazebo.launch.py
```

### 2. Start Controllers
```bash
# Launch robot controllers
ros2 launch trika_controller controller.launch.py
```

### 3. Teleoperation
```bash
# Launch joystick teleoperation
ros2 launch trika_controller joystick_teleop.launch.py
```

### 4. Visualization
```bash
# Launch RViz for visualization
ros2 launch trika_description display.launch.py
```

## 🎮 Usage

### Basic Commands

#### Robot Control
```bash
# Manual velocity control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Monitor robot state
ros2 topic echo /joint_states

# Check TF transforms
ros2 run tf2_tools view_frames
```

#### Parameter Management
```bash
# List all parameters
ros2 param list

# Get specific parameter
ros2 param get /trika_controller wheel_radius

# Set parameter
ros2 param set /trika_controller wheel_radius 0.035
```

#### Service Calls
```bash
# Call custom service
ros2 service call /add_two_ints trika_msgs/srv/AddTwoInts "{a: 5, b: 3}"

# Get transform service
ros2 service call /get_transform trika_msgs/srv/GetTransform "{frame_id: 'base_link', child_frame_id: 'wheel_left_link'}"
```

### Python Examples
```python
# Publisher example
ros2 run trika_py_examples simple_publisher

# Subscriber example
ros2 run trika_py_examples simple_subscriber

# Service client example
ros2 run trika_py_examples simple_service_client

# TF kinematics example
ros2 run trika_py_examples simple_tf_kinematics
```

## 🎯 Simulation

### Gazebo Environment
The project includes comprehensive Gazebo simulation support:

- **Physics Engine**: Realistic wheel dynamics and collision detection
- **Sensor Simulation**: Camera, IMU, and wheel encoders
- **Environment Models**: Custom world files for testing
- **Multi-robot Support**: Scalable simulation environment

### Launch Files
```bash
# Full simulation with controllers
ros2 launch trika_description gazebo.launch.py

# Display only (no physics)
ros2 launch trika_description display.launch.py

# Controller with error modeling
ros2 launch trika_controller controller.launch.py use_simple_controller:=false
```

### Configuration
Key simulation parameters can be adjusted:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | 0.033m | Wheel radius in meters |
| `wheel_separation` | 0.17m | Distance between wheels |
| `wheel_radius_error` | 0.005m | Simulated wheel radius error |
| `wheel_separation_error` | 0.02m | Simulated wheel separation error |

## 🔌 Hardware Requirements

### Minimum Requirements
- **Processor**: Intel i5 or AMD equivalent
- **Memory**: 8GB RAM
- **Storage**: 20GB free space
- **Graphics**: OpenGL 3.3 compatible

### Recommended Hardware
- **Processor**: Intel i7 or AMD Ryzen 7
- **Memory**: 16GB RAM
- **Storage**: 50GB SSD
- **Graphics**: Dedicated GPU with 4GB+ VRAM

### Physical Robot (Optional)
- **Chassis**: Differential drive platform
- **Motors**: 2x DC motors with encoders
- **Sensors**: Camera, IMU, wheel encoders
- **Controller**: Raspberry Pi 4 or similar SBC
- **Power**: 12V battery system

## 📚 API Documentation

### Core Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint states |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/add_two_ints` | `trika_msgs/AddTwoInts` | Simple arithmetic service |
| `/get_transform` | `trika_msgs/GetTransform` | TF transform query |

### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_radius` | double | 0.033 | Wheel radius in meters |
| `wheel_separation` | double | 0.17 | Wheel separation in meters |
| `use_simple_controller` | bool | true | Use simple velocity controller |
| `use_python` | bool | false | Use Python implementation |

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Development Guidelines
- Follow ROS2 coding standards
- Add tests for new features
- Update documentation
- Use meaningful commit messages

### Code Style
- **Python**: Follow PEP 8
- **C++**: Follow ROS2 C++ style guide
- **XML/XACRO**: Use consistent indentation

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2 Community**: For the excellent robotics framework
- **Gazebo Team**: For the powerful simulation environment
- **Contributors**: All who have helped improve this project

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-repo/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-repo/discussions)
- **Documentation**: [Wiki](https://github.com/your-repo/wiki)

---

**Made with ❤️ for the ROS2 community**
