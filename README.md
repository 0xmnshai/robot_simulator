# Robot Simulator Package

A ROS 2 robot simulator package that combines SDL graphics rendering with ROS integration, featuring ultrasonic sensor simulation and robot navigation capabilities.

## Features

- **3D Graphics Rendering**: SDL-based visualization of the robot and environment
- **ROS 2 Integration**: Full ROS 2 integration for robot control and sensing
- **Ultrasonic Sensor Simulation**: Simulated ultrasonic sensors for obstacle detection
- **Robot Control**: Interface for controlling robot movement and navigation

## Screenshots

### SDL Graphics Window
![SDL Window](https://raw.githubusercontent.com/0xmnshai/robot_simulator/main/screenshots/sdl_window.png)

### RViz Map Visualization
![RViz Map](https://raw.githubusercontent.com/0xmnshai/robot_simulator/main/screenshots/rviz_map.png)

## Project Structure

```
robot_simulator_pkg/
├── include/
│   └── robot_simulator_pkg/
│       ├── graphics.hpp          # Graphics rendering interface
│       ├── robot.hpp             # Robot model and physics
│       ├── ros_interface.hpp      # ROS 2 communication
│       └── ultrasonic.hpp         # Ultrasonic sensor simulation
├── src/
│   ├── graphics.cpp              # Graphics implementation
│   ├── robot.cpp                 # Robot implementation
│   ├── ros_interface.cpp          # ROS interface implementation
│   ├── ultrasonic.cpp            # Ultrasonic sensor implementation
│   └── main.cpp                  # Main entry point
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Components

### Robot
- Handles robot kinematics and physics
- Manages position and orientation
- Processes control commands

### Graphics
- Renders the simulation environment using SDL2
- Displays robot and sensor data
- Provides real-time visualization

### ROS Interface
- Publishes sensor data (ultrasonic readings, odometry)
- Subscribes to control commands
- Integrates with ROS 2 ecosystem

### Ultrasonic Sensor
- Simulates multiple ultrasonic sensors
- Ray-casting based distance measurement
- Real-time obstacle detection 